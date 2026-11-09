"""r54의 무조건 반대 벽 고정을 실제 짧은/분기 관측으로 한정한 r55 기록."""
from pathlib import Path
A=Path(__file__).resolve().parent
s=(A/'register_r54.py').read_text(encoding='utf8')
s=s.replace('signal_posts_marker_launch_r53_sha256.json','junction_priority_r54_sha256.json')
s=s.replace("contract='config/tests/junction_priority_r54_sha256.json'","contract='config/tests/junction_divergence_r55_sha256.json'")
s=s.replace('before_r54.zip','before_r55.zip')
s=s.replace('r54: default-off junction retains current agreeing opposite wall before short bridge; reject forward divergent side switches; no threshold changes',
 'r55: retain opposite wall only when preferred path is short or forward-divergent; otherwise keep normal preferred-wall refresh')
s=s.replace('\\nM3_SUCCESSORS += (\'config/tests/junction_priority_r54_sha256.json\',)',
 '\\nM3_SUCCESSORS += (\'config/tests/junction_divergence_r55_sha256.json\',)')
exec(compile(s,str(A/'register_r54.py'),'exec'))
