"""30 mm correspondence sensitivity only. Never writes or enables runtime policy."""
import ast
import inspect
from pathlib import Path
import sys
import textwrap

ROOT=Path(__file__).resolve().parents[4]
sys.path[:0]=[str(ROOT/p) for p in ('src/arena_autonomy','src/arena_vehicle_interface','scripts')]
from arena_autonomy import partial_observation as module

original=module.PartialObservationPolicy.evaluate
tree=ast.parse(textwrap.dedent(inspect.getsource(original)))

class OnlyPointCorrespondence(ast.NodeTransformer):
    changed=0
    def visit_Compare(self,node):
        if (isinstance(node.left,ast.Name) and node.left.id=='distance' and
            len(node.ops)==1 and isinstance(node.ops[0],ast.LtE) and
            len(node.comparators)==1 and isinstance(node.comparators[0],ast.Constant) and
            node.comparators[0].value==.02):
            node.comparators[0]=ast.copy_location(ast.Constant(value=.03),node.comparators[0])
            self.changed+=1
        return self.generic_visit(node)

change=OnlyPointCorrespondence();tree=change.visit(tree);assert change.changed==2
namespace=module.__dict__.copy()
exec(compile(ast.fix_missing_locations(tree),'read_only_30mm_correspondence_probe','exec'),namespace)
module.PartialObservationPolicy.evaluate=namespace['evaluate']
script=(Path(__file__).parent/'compare_saved.py').read_text(encoding='utf8')
assert script.count('saved_comparison.json')==1
script=script.replace('saved_comparison.json','noise_sensitivity_30mm.json')
script=script.replace("scope='saved stateful comparison only; no closed-loop driving'",
    "scope='30mm association sensitivity only; not a runtime adoption or safety validation', correspondence_m=.03")
try:
    exec(compile(script,'noise_sensitivity_wrapper','exec'),{'__file__':str(Path(__file__).resolve())})
finally:
    module.PartialObservationPolicy.evaluate=original
