from controller import RelativeYieldFollow
from observer_v2 import PersistentYieldObserver


class PersistentYieldFollow(RelativeYieldFollow):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.yield_observer=PersistentYieldObserver()
