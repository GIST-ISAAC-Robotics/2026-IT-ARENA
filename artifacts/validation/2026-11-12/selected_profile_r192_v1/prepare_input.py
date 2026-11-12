"""기존 r182 기록의 입력/출력은 보존하고 정책 선택만 헤더에 명시한다."""
import hashlib
import json
from pathlib import Path
A=Path(__file__).resolve().parent
source=A.parent/'guard_approach_runtime_r182_v1/two_actor_run1/run1/follow_decisions.jsonl'
destination=A/'input/follow_decisions.jsonl'
destination.parent.mkdir(parents=True,exist_ok=False)
tail_digest=hashlib.sha256()
with source.open('rb') as old,destination.open('xb') as new:
    original_header=old.readline();header=json.loads(original_header)
    assert 'guard_observation_policy' not in header['data']['follow_config']
    header['data']['follow_config']['guard_observation_policy']='selected_r181'
    new.write((json.dumps(header,separators=(',',':'),ensure_ascii=False)+'\n').encode())
    for block in iter(lambda:old.read(1024*1024),b''):
        new.write(block);tail_digest.update(block)
with destination.open('rb') as stream:
    stream.readline();copied=hashlib.sha256(stream.read()).hexdigest()
assert copied==tail_digest.hexdigest()
manifest=dict(source=str(source.name),original_trace_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
    derived_trace_sha256=hashlib.sha256(destination.read_bytes()).hexdigest(),
    original_header_sha256=hashlib.sha256(original_header).hexdigest(),
    unchanged_body_sha256=copied,changed_field='header.data.follow_config.guard_observation_policy',
    selected_value='selected_r181',scope='explicit configuration for recorded-output replay; not a new drive')
with (A/'derived_input.json').open('x') as out:json.dump(manifest,out,indent=2);out.write('\n')
print(json.dumps(manifest))
