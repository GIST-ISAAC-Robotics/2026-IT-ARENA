"""r530 원본을 읽기만 하며 바이트·중복·Windows JSON 비용과 복원 계약을 측정한다."""
from collections import Counter
import difflib
import hashlib
import json
from pathlib import Path
import platform
import statistics
import time

from stage import DetailArchive,StatusEncoder,load_details,restore_status

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
SOURCE=ROOT/'artifacts/validation/2026-11-14/field_wall_resource_drive_r530_v1/run1/safety_status.jsonl'


def stats(values):
    values=sorted(values)
    return dict(n=len(values),min=values[0],median=statistics.median(values),
        p95=values[max(0,__import__('math').ceil(.95*len(values))-1)],max=values[-1],sum=sum(values))


raw=SOURCE.read_bytes();before_hash=hashlib.sha256(raw).hexdigest()
rows=[json.loads(line)['data'] for line in raw.splitlines()]
full=[json.dumps(r) for r in rows]
contexts={};counts=Counter();latch_sizes=[];context_sizes=[];top_sizes=Counter()
for item in rows:
    for key,value in item.items():top_sizes[key]+=len(json.dumps(value).encode())
    evidence=(item.get('partial_observation') or {}).get('latch_evidence')
    if not evidence:continue
    latch_sizes.append(len(json.dumps(evidence).encode()))
    context=evidence.get('correspondence_context')
    if context is not None:
        encoded=json.dumps(context);counts[encoded]+=1;context_sizes.append(len(encoded.encode()))
        # 저장 JSON은 매 행 새 객체다. 현재 소유자의 최초-capture/불변-object 계약을 재현한다.
        context=contexts.setdefault(encoded,context)
        evidence['correspondence_context']=context

archive=DetailArchive((HERE/'measured_contexts.jsonl').resolve())
encoder=StatusEncoder('compact_v1',archive)
first_capture_ms=[]
for context in contexts.values():
    item=next(r for r in rows if ((r.get('partial_observation') or {}).get('latch_evidence') or {}).get('correspondence_context') is context)
    start=time.perf_counter_ns();encoder.encode(item);first_capture_ms.append((time.perf_counter_ns()-start)/1e6)
    archive.log.queue.join()
compact=[encoder.encode(r) for r in rows]
archive.log.queue.join()
details=load_details(archive.path)
assert all(restore_status(json.loads(text),details)==r for text,r in zip(compact,rows))
(HERE/'compact_statuses.jsonl').write_text(''.join(text+'\n' for text in compact),encoding='utf-8')

# 대표군: 최소·중앙 크기·최대 payload. 동일 후보 객체를 번갈아 호출하는 조합은 피한다.
sizes=[len(s.encode()) for s in full]
ordered=sorted(range(len(rows)),key=lambda i:sizes[i])
representatives=[('small',ordered[0]),('median_size',ordered[len(rows)//2]),('largest',ordered[-1])]
timing={}
for name,index in representatives:
    item=rows[index];baseline=full[index]
    # 해당 context가 준비된 warm encode만 계측. 최초 저장은 위의 별도 값이다.
    encoder.encode(item);archive.log.queue.join()
    candidate=encoder.encode(item)
    methods=dict(full_dumps=lambda:json.dumps(item),compact_encode=lambda:encoder.encode(item),
        full_loads=lambda:json.loads(baseline),compact_loads=lambda:json.loads(candidate))
    timed={}
    for label,function in methods.items():
        for _ in range(10):function()
        measured=[]
        for _ in range(150):
            start=time.perf_counter_ns();function();measured.append((time.perf_counter_ns()-start)/1e6)
        timed[label]=stats(measured)
    timing[name]=dict(index=index,reference_s=item.get('reference_s'),full_bytes=len(baseline.encode()),
        compact_bytes=len(candidate.encode()),milliseconds=timed)

assert archive.close()

result=dict(scope='Windows saved JSON serialization and recoverable compact stage only; no ROS/network/Jetson timing',
    python=platform.python_version(),platform=platform.platform(),source=str(SOURCE.relative_to(ROOT)),
    source_sha256=before_hash,file_bytes=len(raw),rows=len(rows),
    source_reference_range_s=[rows[0]['reference_s'],rows[-1]['reference_s']],
    field_trial_s=12.,note='full log includes startup and STOP; 1015 rows span 20.30 simulation seconds',
    payload_bytes=stats(sizes),top_value_bytes=dict(top_sizes),latch_evidence_bytes=sum(latch_sizes),
    correspondence_context_rows=sum(counts.values()),unique_contexts=len(contexts),
    repeated_context_bytes=sum(context_sizes),unique_context_bytes=sum(len(x.encode()) for x in contexts),
    compact_payload_bytes=stats([len(x.encode()) for x in compact]),detail_file_bytes=archive.path.stat().st_size,
    roundtrip_rows=len(rows),first_capture_encode_ms=first_capture_ms,representative_timing=timing,
    control_output_fields_preserved=True,original_file_unchanged=hashlib.sha256(SOURCE.read_bytes()).hexdigest()==before_hash)
(HERE/'measurement.json').write_text(json.dumps(result,indent=2,ensure_ascii=False),encoding='utf-8')

# 이관 후보만 생성한다. 현재 공통/동결 소스에 쓰지 않는다.
source=ROOT/'src/arena_autonomy/arena_autonomy/lidar_safety.py'
before=source.read_bytes();text=before.decode();after=text
needle='        self.status = self.create_publisher(String, "/safety/status", 10)'
addition='''
        from arena_autonomy.guard_status_payload import DetailArchive, StatusEncoder
        status_detail = self.declare_parameter('safety_status_detail', 'full').value
        detail_path = self.declare_parameter('safety_status_evidence_path', '').value
        self.status_archive = DetailArchive(detail_path) if status_detail == 'compact_v1' else None
        self.status_encoder = StatusEncoder(status_detail, self.status_archive)'''
assert after.count(needle)==1;after=after.replace(needle,needle+addition)
needle='self.status.publish(String(data=json.dumps({'
assert after.count(needle)==1;after=after.replace(needle,'self.status.publish(String(data=self.status_encoder.encode({')
needle='\n\ndef main(args=None):'
addition='''
    def destroy_node(self):
        archive = getattr(self, 'status_archive', None)
        if archive is not None and not archive.close():
            self.get_logger().error('guard status detail writer did not finish')
        return super().destroy_node()
'''
assert after.count(needle)==1;after=after.replace(needle,'\n'+addition+needle)
compile(after,str(source),'exec')
(HERE/'lidar_safety.before.py').write_bytes(before)
(HERE/'lidar_safety.stage.py').write_text(after,encoding='utf-8')
(HERE/'integration.patch').write_text(''.join(difflib.unified_diff(text.splitlines(True),after.splitlines(True),
    fromfile='a/src/arena_autonomy/arena_autonomy/lidar_safety.py',tofile='b/src/arena_autonomy/arena_autonomy/lidar_safety.py')),encoding='utf-8')
(HERE/'source_manifest.json').write_text(json.dumps(dict(lidar_safety_before_sha256=hashlib.sha256(before).hexdigest(),
    stage_module_sha256=hashlib.sha256((HERE/'stage.py').read_bytes()).hexdigest(),production_changed=False),indent=2),encoding='utf-8')
print(json.dumps({k:result[k] for k in ('file_bytes','rows','source_reference_range_s','correspondence_context_rows',
    'unique_contexts','repeated_context_bytes','roundtrip_rows','original_file_unchanged')},indent=2))
print('payload bytes full/compact/details:',sum(sizes),sum(len(x.encode()) for x in compact),archive.path.stat().st_size)
for name,timed in timing.items():
    print(name,timed['full_bytes'],timed['compact_bytes'],{k:round(v['median'],4) for k,v in timed['milliseconds'].items()})
