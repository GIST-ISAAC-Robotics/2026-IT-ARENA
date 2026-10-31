import closed_loop
from controller_v3 import RejoinYieldFollow

closed_loop.RelativeYieldFollow=RejoinYieldFollow
if __name__=='__main__':closed_loop.main()
