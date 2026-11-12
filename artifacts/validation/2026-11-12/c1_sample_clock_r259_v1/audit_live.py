import hashlib,json,math,statistics,zipfile
from pathlib import Path
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
p=ROOT/'build/r259_complete.zip';digest=hashlib.sha256(p.read_bytes()).hexdigest()
assert digest=='bb70692edec801db143ed27a14926b7881e38382b134b6c6131254e07aed0106'
folder=ROOT/'build/r259_recovered'
if not folder.exists():
    folder.mkdir()
    with zipfile.ZipFile(p) as z:
        for n in z.namelist():
            target=folder/n;assert target.resolve().is_relative_to(folder.resolve())
            target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(z.read(n))
raw=[json.loads(s) for s in (folder/'live/sdk_raw.jsonl').open()]
mapped=[json.loads(s) for s in (folder/'live/converted.jsonl').open()]
assert len(raw)==len(mapped)
wait=[];elapsed=[];age=[];residual=[];period=[];last=None;points=0;published=0;warmup=0
for original,row in zip(raw,mapped):
    payload=row['payload'];diag=row['diagnostic']
    assert diag['time_contract_verified'] is False
    if payload is None:warmup+=1;continue
    published+=1;n=len(original['nodes']);points+=n
    assert len(payload['acquired_at_s'])==n and payload['time_basis']=='unverified_estimate'
    assert payload['source_frame_id']=='c1_unmounted_preview'
    assert all(b>a for a,b in zip(payload['acquired_at_s'],payload['acquired_at_s'][1:]))
    if last is not None:assert payload['acquired_at_s'][0]>last
    last=payload['acquired_at_s'][-1]
    for i,(a,d,q,f) in enumerate(original['nodes']):
        assert payload['angles_rad'][i]==math.pi-a*math.pi/32768.
        assert payload['ranges_m'][i]==('+inf' if d==0 else d/4000. if q else None)
    assert row['publication_monotonic_ns']>=diag['estimated_end_monotonic_ns']
    assert last<=row['publication_ros_ns']/1e9+1e-6
    wait.append(diag['publication_wait_ns']/1e6)
    elapsed.append((row['publication_monotonic_ns']-row['receipt_monotonic_ns'])/1e6)
    age.append((row['publication_ros_ns']/1e9-last)*1000)
    residual.append(diag['sdk_anchor_residual_ns']/1e6);period.append(diag['sample_period_ns'])
summary=json.loads((folder/'live/summary.json').read_bytes())
assert published==summary['counts']['published']==summary['counts']['received']
assert warmup==20 and points==summary['counts']['received_points']
assert summary['pending']==0 and summary['sdk_returncode']==0 and summary['sensor_stop_ack'] and not summary['forced_cleanup']
def stats(values):
    s=sorted(values);return dict(min=s[0],median=statistics.median(s),p95=s[int((len(s)-1)*.95)],max=s[-1])
result=dict(input_batches=len(raw),warmup_batches=warmup,published_and_received=published,points=points,
    raw_geometry_and_order_preserved=True,interbatch_overlap=0,future_at_publication=0,
    requested_wait_ms=stats(wait),receipt_to_publication_ms=stats(elapsed),estimated_end_age_at_publication_ms=stats(age),
    sdk_anchor_residual_ms=stats(residual),estimated_sample_period_ns=stats(period),
    time_contract_verified=False,drive_output=False,recovered_sha256=digest,
    scope='ROS loopback/common decoder, geometry and clock consistency; no absolute acquisition accuracy or control/drive validation')
(A/'live_audit.json').write_text(json.dumps(result,indent=2)+'\n')
(A/'live_summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(result,indent=2))
