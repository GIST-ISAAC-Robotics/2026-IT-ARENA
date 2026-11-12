"""시뮬 단독 노드에서 r317 관측 반환 기반 경로 이동 후보를 명시 선택한다."""
from pathlib import Path
import hashlib,runpy,sys
import candidate
from arena_autonomy.local_pursuit import LocalPursuit

executable,*arguments=sys.argv[1:]
assert Path(executable).name=='local_pursuit'
initialize=LocalPursuit.__init__
def checked(self,*args,**kwargs):
    initialize(self,*args,**kwargs)
    assert self.get_parameter('use_sim_time').value is True,'simulation only'
    print('r318_observed_fragment_candidate='+hashlib.sha256(Path(candidate.__file__).read_bytes()).hexdigest(),flush=True)
LocalPursuit.__init__=checked
candidate.install()
sys.argv=[executable,*arguments]
runpy.run_path(executable,run_name='__main__')
