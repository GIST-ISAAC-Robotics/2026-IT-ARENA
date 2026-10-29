"""종료 시각 자료 차이의 사후 분석. 원래 run1 판정을 변경하지 않는다."""
import hashlib
import json
from pathlib import Path

BASE=Path(__file__).resolve().parent
RUN=BASE/'run1'


def sha(path):
    with path.open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()


def main():
    target=BASE/'run1_tail_audit.json'
    if target.exists():raise FileExistsError(target)
    files=[RUN/name for name in ('report.json','ego_poses.jsonl','actor_poses.jsonl','stop_trace.json')]
    hashes={p.name:sha(p) for p in files}
    report=json.loads(files[0].read_text(encoding='utf-8'))
    ego=[json.loads(s) for s in files[1].read_text().splitlines()]
    actor=[json.loads(s) for s in files[2].read_text().splitlines()]
    stop=json.loads(files[3].read_text(encoding='utf-8'))
    tail=[row[0] for row in ego if row[0]>actor[-1][0]+.05+1e-9]
    stop_end=stop[-1]['time']
    output=dict(scope='timestamp-only post-audit; original invalid verdict retained',
        original_passed=report['passed'],original_valid=report['run_valid'],
        ego_last_s=ego[-1][0],actor_last_s=actor[-1][0],stop_evidence_end_s=stop_end,
        missing_after_actor_50ms_tail=tail,
        every_missing_after_stop_evidence=bool(tail) and min(tail)>stop_end,
        tail_count_matches_original=len(tail)==report['evaluation']['gap']['sampled_missing_actor_samples'],
        original_actor_coverage=report['evaluation']['coverage']['actor'],
        input_sha256=hashes,input_unchanged=all(sha(p)==hashes[p.name] for p in files),
        analysis_sha256=sha(Path(__file__)))
    target.write_text(json.dumps(output,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(output,ensure_ascii=False))


if __name__=='__main__':main()
