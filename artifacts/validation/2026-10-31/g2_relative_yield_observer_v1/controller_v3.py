from controller import RelativeYieldFollow
from observer_v3 import RejoinYieldObserver


class RejoinYieldFollow(RelativeYieldFollow):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.yield_observer=RejoinYieldObserver()
