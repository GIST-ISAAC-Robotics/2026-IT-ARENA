from pathlib import Path
import hashlib,json,zipfile
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
p=ROOT/'scripts/bridge_c1_native_preview.py';source=p.read_text()
source=source.replace('from arena_vehicle_interface.c1_native_preview import C1NativePreview','from sample_clock import C1SampleClock as C1NativePreview')
source=source.replace('payload, status = converter.convert(batch,','payload, status, due = converter.convert(batch,')
needle="                        text = json.dumps(payload, separators=(',', ':'), allow_nan=False)"
assert source.count(needle)==1
insert='''                        if payload is None:
                            counts['warmup_or_rejected'] += 1
                            mapped.write(json.dumps(dict(payload=None, diagnostic=status,
                                receipt_monotonic_ns=mono, receipt_ros_ns=ros), allow_nan=False)+'\\n')
                            continue
                        # 推定 끝시각을 현재로 자르지 않는다. 최대5ms 후보 대기만 허용한다.
                        while time.monotonic_ns() < due:
                            rclpy.spin_once(node, timeout_sec=max(0., min(.001, (due-time.monotonic_ns())/1e9)))
                        publication_mono = time.monotonic_ns()
                        publication_ros = node.get_clock().now().nanoseconds
'''.replace('推定','추정')
source=source.replace(needle,insert+needle)
needle='receipt_monotonic_ns=mono, receipt_ros_ns=ros), allow_nan=False)'
# 마지막 정상 발행 기록만 추가한다. warmup에는 publication 시각이 없다.
index=source.rfind(needle);assert index>=0
source=source[:index]+source[index:].replace(needle,'receipt_monotonic_ns=mono, receipt_ros_ns=ros, publication_monotonic_ns=publication_mono, publication_ros_ns=publication_ros), allow_nan=False)',1)
(A/'bridge_candidate.py').write_text(source)
(A/'source_manifest.json').write_text(json.dumps({'base_bridge':str(p.relative_to(ROOT)),'base_sha256':hashlib.sha256(p.read_bytes()).hexdigest(),
    'candidate_files':{n:hashlib.sha256((A/n).read_bytes()).hexdigest() for n in ('bridge_candidate.py','sample_clock.py')}},indent=2)+'\n')
with zipfile.ZipFile(ROOT/'build/r259_candidate.zip','x',zipfile.ZIP_DEFLATED) as z:
    for n in ('bridge_candidate.py','sample_clock.py','source_manifest.json'):z.write(A/n,n)
