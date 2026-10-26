"""replay_execution.py 행 출력의 이전/현재 소스 대조. 처리 시간(ms) 외 모든 필드가 같아야 한다."""
import argparse
import json
from pathlib import Path


def main():
    p = argparse.ArgumentParser()
    p.add_argument('old', type=Path)
    p.add_argument('new', type=Path)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    if args.output.exists():
        raise ValueError('new output required')
    result = {'comparison': f'{args.new} vs {args.old}', 'ignored_fields': ['ms'], 'records': {}}
    for old in sorted(args.old.glob('rows_*.jsonl')):
        new = args.new/old.name
        a = [json.loads(x) for x in old.read_text(encoding='utf-8').splitlines()]
        b = [json.loads(x) for x in new.read_text(encoding='utf-8').splitlines()] if new.exists() else []
        mismatches = []
        for i, (x, y) in enumerate(zip(a, b)):
            x = {k: v for k, v in x.items() if k != 'ms'}
            y = {k: v for k, v in y.items() if k != 'ms'}
            if x != y:
                mismatches.append({'index': i, 't': x.get('t'),
                                   'fields': sorted(k for k in set(x) | set(y) if x.get(k) != y.get(k))})
        result['records'][old.name] = {'old_rows': len(a), 'new_rows': len(b), 'mismatch_count': len(mismatches),
                                       'first_10': mismatches[:10]}
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=1)+'\n', encoding='utf-8', newline='\n')
    print(json.dumps(result, ensure_ascii=False, indent=1))
    assert result['records'] and all(r['mismatch_count'] == 0 and r['old_rows'] == r['new_rows']
                                     for r in result['records'].values())


if __name__ == '__main__':
    main()
