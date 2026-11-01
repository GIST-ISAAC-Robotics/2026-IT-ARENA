"""재생 실험을 새 스캔 판단 기준으로 나란히 출력한다."""
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
prefix = sys.argv[1]
names = sys.argv[2].split(',')
lo, hi = float(sys.argv[3]), float(sys.argv[4])
for name in names:
    r = json.loads((HERE / 'replay' / f'{prefix}_{name}.json').read_bytes())
    print(f"# {name} rows={len(r['rows'])} compared={r['compared']} mismatch={r['mismatch']}")
    for x in r['rows']:
        c = x.get('corridor') or {}
        if not lo <= x['t'] <= hi or not (c.get('fresh') or x['sel'] is not None):
            continue
        ag = [f"{'T' if a[0] else 'F'}{a[1] if a[1] is None else round(a[1],3)}" for a in x.get('agree', [])]
        print(f"{x['t']:6.2f} meas={x['meas']:+5.2f} v={x['v']:4.2f} s={x['s']:+.3f} remain={(x['remain'] or -1):5.2f} "
              f"{x['reason']} sel={x['sel']} acc={x['fresh_acc']} ahead={x['cand_ahead'] and round(x['cand_ahead'],2)} agree={ag} "
              f"| cor {c.get('reason')} n={c.get('confirmations')} rel={c.get('release_count')} "
              f"pa={c.get('previous_path_agrees')} d={c.get('transport_displacement_m') and round(c['transport_displacement_m'],3)}")
