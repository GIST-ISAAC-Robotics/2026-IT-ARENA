"""이미 회수한 결과만 분류한다. 제어기를 다시 실행하거나 판정을 바꾸지 않는다."""
import collections
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent
SOURCE = ROOT / 'revision_env_v2/remote_result/extracted/full/comparison_rows.jsonl'


def main():
    rows = [json.loads(line) for line in SOURCE.read_text(encoding='utf-8').splitlines()]
    categories = collections.Counter()
    paths = collections.defaultdict(collections.Counter)
    maxima = {}
    count_differences = []
    unexpected = []
    for row in rows:
        for difference in row.get('differences', []):
            category = difference['category']
            categories[category] += 1
            paths[category][json.dumps(difference['path'])] += 1
            if (category not in maxima or difference.get('absolute_difference', -1)
                    > maxima[category].get('absolute_difference', -1)):
                maxima[category] = difference
            if category == 'region_count':
                count_differences.append({
                    'kind': row['comparison_kind'], 'index': row['index'], **difference})
            elif difference.get('existing_comparator_equal') is not True:
                unexpected.append({'kind': row['comparison_kind'], 'index': row['index'], **difference})
    report = {
        'schema': 'r411_recovered_difference_classification_v1',
        'source_sha256': hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
        'original_historical_exact_reproduction_passed': False,
        'new_controller_execution': False,
        'comparison_tolerance_changed': False,
        'categories': dict(categories),
        'paths': {key: dict(value) for key, value in paths.items()},
        'maximum_difference_by_category': maxima,
        'existing_comparator_false_row_counts': dict(collections.Counter(
            row['comparison_kind'] for row in rows
            if row.get('existing_comparator_equal') is False)),
        'region_count_differences': count_differences,
        'other_nonmatching_differences': unexpected,
        'scope': 'stored finite corpus only; no claim of all-platform or closed-loop equivalence',
    }
    (ROOT / 'difference_review.json').write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({'categories': report['categories'], 'unexpected': len(unexpected)}))


if __name__ == '__main__':
    main()
