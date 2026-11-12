import ast
from pathlib import Path
import zipfile
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
files=['src/arena_autonomy/arena_autonomy/guard_observation_policy.py',
       'src/arena_autonomy/arena_autonomy/local_pursuit.py',
       'src/arena_autonomy/arena_autonomy/g1_g2_node.py',
       'src/arena_bringup/launch/g1_follow.launch.py']
with zipfile.ZipFile(A/'before_sources.zip','x',zipfile.ZIP_DEFLATED) as archive:
    for name in files:archive.write(ROOT/name,name)
original=(A.parent/'guard_approach_cap_r181_v1/candidate.py').read_text(encoding='utf-8')
tree=ast.parse(original)
functions=[ast.get_source_segment(original,next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name==name))
           for name in ('supported','rebase')]
path=ROOT/files[0]
text=path.read_text(encoding='utf-8')
text=text.replace('import math\n','import math\nfrom arena_autonomy import current_path_recheck as common\nfrom arena_autonomy.local_follow import forward_planning_speed\n')
path.write_text(text+'\n\n'+'\n\n\n'.join(functions)+'\n',encoding='utf-8')
print('Preserved four sources; copied r181 current recheck functions unchanged')
