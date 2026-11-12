"""실물 SDK 원 정수와 변환 배열을 별도 대조. 원시 실내 점은 공개하지 않는다."""
from pathlib import Path
from collections import Counter
import hashlib,json,math,tarfile
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
archive=ROOT/'build/r204_results_private.tar.gz'
digest=hashlib.sha256(archive.read_bytes()).hexdigest()
assert digest=='44032e627004b780117791e7b4cd88d997cfe6ffaee5af0eee7be982a5a10906'
private=ROOT/'build/r204_recovered';private.mkdir(exist_ok=False)
with tarfile.open(archive) as t:
    for m in t.getmembers():
        assert (private/m.name).resolve().is_relative_to(private) and (m.isfile() or m.isdir())
    t.extractall(private,filter='data')
capture=private/A.relative_to(ROOT)/'capture'
raw=[json.loads(x) for x in (capture/'sdk_raw.jsonl').open()]
mapped=[json.loads(x) for x in (capture/'converted.jsonl').open()]
counts=Counter();epochs=set()
assert len(raw)==len(mapped)>0
for sdk,record in zip(raw,mapped):
    p,d=record['payload'],record['diagnostic'];epochs.add(p['source_epoch'])
    assert len(sdk['nodes'])==len(p['ranges_m'])==len(p['angles_rad'])==len(p['acquired_at_s'])
    offset=record['receipt_ros_ns']-record['receipt_monotonic_ns']
    first=sdk['sdk_estimated_first_us']*1000+offset
    zero=invalid=0
    for i,((angle,distance,quality,flag),value) in enumerate(zip(sdk['nodes'],p['ranges_m'])):
        assert p['angles_rad'][i]==math.pi-angle*math.pi/32768.
        assert p['acquired_at_s'][i]==(first+i*200_000)/1e9
        if distance==0:
            assert value=='+inf';zero+=1
        elif quality==0:
            assert value is None;invalid+=1
        else:
            assert value==distance/4000.;counts['positive_quality_range']+=1
    assert d['zero_distance_points']==zero and d['invalid_quality_positive_distance_points']==invalid
    assert not any(d[k] for k in ('time_contract_verified','sensor_acquisition_timestamp','mounting_calibrated','drive_output','nonreturn_implies_free_space'))
    assert p['time_basis']=='unverified_estimate' and p['source_frame_id']=='c1_unmounted_preview'
    counts.update(batches=1,points=len(sdk['nodes']),zero_distance_to_inf=zero,positive_distance_bad_quality_to_null=invalid)
summary=json.loads((capture/'summary.json').read_bytes())
result=json.loads((private/A.relative_to(ROOT)/'run_result.json').read_bytes())
assert summary['counts']['published']==summary['counts']['received']==len(raw)
assert not summary['pending'] and not summary['forced_cleanup'] and summary['sensor_stop_ack']
assert result==dict(run_exit=0,acl_restored=True,device_owner_remaining=False,source_unchanged=True)
source=json.loads((A/'source_manifest.json').read_bytes())
assert all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in source.items())
audit=dict(passed=True,counts=dict(counts),source_epochs=len(epochs),all_angles_order_ranges_estimated_stamps_exact=True,
    private_archive_sha256=digest,private_raw_sha256=hashlib.sha256((capture/'sdk_raw.jsonl').read_bytes()).hexdigest(),
    private_converted_sha256=hashlib.sha256((capture/'converted.jsonl').read_bytes()).hexdigest(),
    time_contract_verified=False,drive_output=False,scope='one stationary diagnostic receipt, no motion or mounted calibration')
for name,data in [('audit.json',audit),('summary.json',summary),('run_result.json',result)]:
    with (A/name).open('x',encoding='utf8') as f:json.dump(data,f,indent=2);f.write('\n')
(A/'sdk.log').write_bytes((capture/'sdk.log').read_bytes())
print(json.dumps(audit))
