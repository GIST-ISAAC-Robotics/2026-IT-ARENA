"""v1 평가기·네 조건은 유지하고 관측 감속의 해제 조건만 별도 비교한다."""
import closed_loop
from controller_v2 import PersistentYieldFollow

closed_loop.RelativeYieldFollow=PersistentYieldFollow
if __name__=='__main__':closed_loop.main()
