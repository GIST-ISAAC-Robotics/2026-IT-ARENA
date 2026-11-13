"""운영 기록을 사건 중심 시간축 CSV/HTML로 내보낸다. 원시 누락과 평가 정답을 숨기거나 섞지 않는다."""
import csv
import html
import io
import json

COLUMNS = ('sim_s', 'wall_s', 'source', 'kind', 'text', 'state', 'reason', 'behavior', 'side', 'plan_mps',
           'protected_mps', 'mcu_target_mps', 'motor_shaft_equiv_mps', 'steering_rad', 'mcu_state', 'safety_reason',
           'clearance_m', 'marker_ids', 'scan_age_s', 'rgb_age_s', 'gyro_age_s', 'encoder_age_s')
GAP_S = .6  # 5 Hz 표본의 3배. 이보다 비면 누락으로 표시한다. 팀 초안값.
LIMIT = 'truth'


def read_jsonl(path):
    rows, invalid = [], 0
    with open(path, encoding='utf-8') as stream:
        for line in stream:
            try:
                rows.append(json.loads(line))
            except ValueError:
                invalid += 1
    return rows, invalid


def flatten(row):
    result = {key: row.get(key) for key in COLUMNS}
    result['source'] = row.get('source') or ('sample' if row.get('row') == 'sample' else row.get('row'))
    ages = row.get('input_age_s') or {}
    for name in ('scan', 'rgb', 'gyro', 'encoder'):
        result[name+'_age_s'] = ages.get(name)
    return result


def decision_rows(path, start_s, end_s):
    """기존 판단 기록에서 이유·거동이 바뀐 지점만 뽑는다. 큰 스캔 입력은 복사하지 않는다."""
    rows, last = [], None
    with open(path, encoding='utf-8') as stream:
        for line in stream:
            try:
                event = json.loads(line)
            except ValueError:
                continue
            if event.get('kind') != 'call':
                continue
            data = event['data']
            stamp = (data.get('input') or {}).get('reference_s')
            output = data.get('output') or [None, None, {}]
            detail = output[2] if isinstance(output[2], dict) else {}
            key = (detail.get('reason'), detail.get('behavior'))
            if type(stamp) not in (int, float) or not start_s <= stamp <= end_s or key == last:
                continue
            last = key
            rows.append(dict(row='event', source='decision', kind='decision', sim_s=stamp,
                             text=f'판단 {key[0]} / {key[1]}', reason=key[0], behavior=key[1],
                             plan_mps=output[0], steering_rad=output[1]))
    return rows


def merge(rows, *, decisions=(), truth=()):
    merged = [flatten(row) for row in [*rows, *decisions]]
    merged.sort(key=lambda row: (row['sim_s'] is None, row['sim_s'] or 0., row['wall_s'] or 0.))
    samples = [row['wall_s'] for row in rows if row.get('row') == 'sample' and row.get('wall_s') is not None]
    gaps = [dict(after_wall_s=a, gap_s=b-a) for a, b in zip(samples, samples[1:]) if b-a > GAP_S]
    return dict(rows=merged, gaps=gaps, truth=[dict(row) for row in truth],
                sample_count=len(samples), event_count=len(rows)-len(samples)+len(decisions))


def window(rows, center_sim_s, before_s=5., after_s=5.):
    return [row for row in rows if row.get('sim_s') is not None
            and center_sim_s-before_s <= row['sim_s'] <= center_sim_s+after_s]


def render_csv(table):
    stream = io.StringIO()
    writer = csv.DictWriter(stream, fieldnames=COLUMNS, extrasaction='ignore', lineterminator='\n')
    writer.writeheader()
    for row in table['rows']:
        writer.writerow({key: json.dumps(value, ensure_ascii=False) if isinstance(value, (list, dict)) else value
                         for key, value in row.items()})
    return stream.getvalue()


def _cell(value):
    if value is None:
        return '<td class=none>-</td>'
    if isinstance(value, float):
        return f'<td>{value:.3f}</td>'
    return f'<td>{html.escape(str(value))}</td>'


def render_html(table, title='사건 시간축', note=''):
    head = ''.join(f'<th>{html.escape(name)}</th>' for name in COLUMNS)
    body = []
    for row in table['rows']:
        kind = 'event' if row['kind'] else 'sample'
        body.append(f'<tr class={kind}>'+''.join(_cell(row.get(name)) for name in COLUMNS)+'</tr>')
    gaps = ''.join(f"<li>wall {g['after_wall_s']:.3f} s 뒤 {g['gap_s']:.3f} s 표본 없음</li>" for g in table['gaps'])
    truth = ''
    if table['truth']:
        names = sorted({key for row in table['truth'] for key in row})
        truth = ('<h2>평가 정답(제어 입력 아님, 관측 기록과 별개)</h2><table><tr>'+
                 ''.join(f'<th>{html.escape(n)}</th>' for n in names)+'</tr>'+
                 ''.join('<tr>'+''.join(_cell(row.get(n)) for n in names)+'</tr>' for row in table['truth'])+'</table>')
    return f"""<!doctype html><html lang=ko><meta charset=utf-8><title>{html.escape(title)}</title>
<style>body{{font:13px system-ui,'Malgun Gothic',sans-serif;margin:12px;background:#fff;color:#111}}
table{{border-collapse:collapse;font-size:12px}}th,td{{border:1px solid #bbb;padding:2px 5px;white-space:nowrap}}
th{{position:sticky;top:0;background:#eee}}tr.event{{background:#fff4d6}}td.none{{color:#999}}.w{{overflow:auto;max-height:80vh}}</style>
<h1>{html.escape(title)}</h1><p>{html.escape(note)}</p>
<p>표본 {table['sample_count']}개, 사건 {table['event_count']}개. 계획=/drive, 보호 후=/drive/safe, MCU 적용 목표와 모터축 환산은 차체 실제 속도가 아니다.
원천 시각과 수신 시각의 시계 차이는 보정하지 않았다.</p>
<h2>기록 누락</h2><ul>{gaps or '<li>표본 간격 초과 없음(사건 사이 값 변화의 완전성을 뜻하지 않음)</li>'}</ul>
<div class=w><table><tr>{head}</tr>{''.join(body)}</table></div>{truth}</html>"""
