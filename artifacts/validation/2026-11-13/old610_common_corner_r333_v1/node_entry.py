"""시험 등록의 공통 제어기 소스만 실행하며 별도 시설 후보를 주입하지 않는다."""
from pathlib import Path
import hashlib,json,runpy,sys
import arena_autonomy.local_follow as follow
from arena_autonomy.local_pursuit import LocalPursuit
A=Path(__file__).resolve().parent
expected=json.loads((A/'registered.json').read_text())['input_sha256']['src/arena_autonomy/arena_autonomy/local_follow.py']
assert hashlib.sha256(Path(follow.__file__).read_bytes()).hexdigest()==expected
executable,*arguments=sys.argv[1:]
assert Path(executable).name=='local_pursuit'
initialize=LocalPursuit.__init__
def checked(self,*args,**kwargs):
    initialize(self,*args,**kwargs)
    assert self.get_parameter('use_sim_time').value is True
    print('r333_common_local_follow_sha256='+expected,flush=True)
LocalPursuit.__init__=checked
sys.argv=[executable,*arguments]
runpy.run_path(executable,run_name='__main__')
