"""회수 자료와 원 시간 예산을 독립 확인한다. 실행/발행하지 않는다."""
import hashlib
import json
from pathlib import Path
import zipfile

A = Path(__file__).resolve().parent
ROOT = A.parents[3]


def main():
    archive = ROOT/'build/r273_results.zip'
    digest = hashlib.sha256(archive.read_bytes()).hexdigest()
    assert digest == 'c8cd699d3f456afa6680454ff073caaf2ae3a87bb0ff52a80ac172c83b4e1dad'
    with zipfile.ZipFile(archive) as z:
        for name in z.namelist():
            path = (A/name).resolve()
            assert path.is_relative_to(A) and not name.endswith('/')
            payload = z.read(name)
            if path.exists():
                assert path.read_bytes() == payload, name
            else:
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(payload)
    checked = {}
    for directory in sorted(A.iterdir()):
        if not directory.is_dir() or not (directory/'report.json').exists():
            continue
        report = json.loads((directory/'report.json').read_bytes())
        answers = json.loads((directory/'answers.json').read_bytes())
        events = [json.loads(s) for s in (directory/'events.jsonl').read_text().splitlines()]
        refs = [r['reference_s'] for r in events if r['kind']=='submitted']
        assert all(b-a >= .1-1e-9 for a,b in zip(refs,refs[1:]))
        assert all(not r['forced'] and r['exitcode']==0 for r in report['cleanup'])
        for r in answers:
            if r['speed']<=0:
                continue
            assert not r['retained']
            assert 0 <= r['now_s']-r['reference_s'] < .2
            assert 0 <= r['now_s']-r['current_source_first_s'] < .3
            assert 0 <= r['now_s']-r['original_path_source_s'] < .5
            assert r['proof']['accepted'] and not r['proof']['source_timestamp_renewed']
            assert not r['proof']['old_occupancy_permission_reused']
        checked[directory.name] = dict(positive=sum(r['speed']>0 for r in answers),
                                      submissions=len(refs), cleanup=report['cleanup'])
    assert len(checked)==4
    result = dict(archive_sha256=digest, checks=checked, source_times_preserved=True,
                  current_command_limit_s=.2,current_raw_limit_s=.3,geometry_limit_s=.5)
    (A/'audit.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result))


if __name__=='__main__':main()
