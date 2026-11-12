from pathlib import Path
import hashlib,json,shutil
A=Path(__file__).resolve().parent
old=A.parent/'traffic_solo_gap_runtime_r225_v1'
names=['run.py','assess.py','postprocess.py','node_entry.py','launch_prefix.py','delay_worker.py','delivery_profile.json']
for name in names:
    assert not (A/name).exists()
    shutil.copy2(old/name,A/name)
p=A/'run.py';s=p.read_text(encoding='utf-8')
changes={
 'r225: 현재 SOLO 저속 공백·80초 지연 교통1회.':'r230: 명령 만료 중 관측 갱신 분리·80초 지연 교통1회.',
 'config/tests/traffic_solo_gap_r224_sha256.json':'config/tests/stale_command_observation_r229_sha256.json',
 'explicit r225 delay only prefix; common r224 selected SOLO gap; independent safety unchanged':'explicit r230 delay only prefix; r229 independent stale-command observation; r227 equivalent queries',
 "baseline='expiry_reference_runtime_r220_v1'":"baseline='traffic_solo_gap_runtime_r225_v1'",
 "selected_change='r224 current SOLO oblique gap only; independent guard also selected'":"selected_change='r229 stale commands remain zero while valid observations continue; r227 identical motion-query batch'"}
for before,after in changes.items():
    assert s.count(before)==1,before
    s=s.replace(before,after)
p.write_text(s,encoding='utf-8')
(A/'parent_scripts.json').write_text(json.dumps({(old/name).as_posix():hashlib.sha256((old/name).read_bytes()).hexdigest() for name in names},indent=2)+'\n')
