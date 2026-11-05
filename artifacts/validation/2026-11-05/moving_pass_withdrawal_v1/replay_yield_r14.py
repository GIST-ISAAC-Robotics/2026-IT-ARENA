"""앞차 뒤 복귀를 위한 완만한 양보. 실제 이동은 폐루프에서 별도 확인한다."""
import sys
from prepare import HERE,save
from replay_candidate import replay

if __name__=='__main__':
    paths=dict(constant=HERE.parent/'moving_pass_v1/closed_loop_side_pass_2/run1',
               accelerating=HERE/'baseline_r12/run1',earlier_entry=HERE/'candidate_r13/run1')
    out={key:replay(p) for key,p in paths.items()}
    save(HERE/(sys.argv[1]+'.json'),out)
    for key,v in out.items(): print(key,v['calls'],v['changed_motion_calls'],v['reasons'])
    assert out['constant']['changed_motion_calls']==0
    assert any('moving_pass_yield_for_return' in k for k in out['earlier_entry']['reasons'])
