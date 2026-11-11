"""C++ 내부 연산 비교. 원점/후보 창/동률 순서/NumPy 경계 판정은 유지한다."""
import ctypes
import inspect
from pathlib import Path
import numpy as np
from arena_autonomy import observation_support as module

_ORIGINAL = module.depth_support


class Kernel:
    def __init__(self, path):
        self.library = ctypes.CDLL(str(Path(path).resolve()))
        self.function = self.library.depth_evaluate
        integer = ctypes.c_int64
        pointer = ctypes.c_void_p
        self.function.argtypes = [integer]*4+[pointer]*10
        self.function.restype = ctypes.c_int
        self.rows = self.fallback_rows = self.calls = 0

    def evaluate(self, part, chosen, origins, directions, indices, valid, depth, pad, cycle, original):
        n = len(origins)
        if np.asarray(pad).shape != (n,):
            return original(part, chosen)
        arrays = [np.ascontiguousarray(v,dtype=t) for v,t in (
            (part,np.float64),(origins,np.float64),(directions,np.float64),
            (indices,np.int64),(valid,np.uint8),(depth,np.float64),
            (pad,np.float64),(chosen,np.int64))]
        result = np.zeros(len(part),dtype=np.uint8)
        uncertain = np.zeros(len(part),dtype=np.uint8)
        arrays.extend((result,uncertain))
        status = self.function(len(part),n,chosen.shape[1],cycle or 0,
            *(ctypes.c_void_p(a.ctypes.data) for a in arrays))
        if status != 0:
            raise RuntimeError(f'native depth kernel invalid input: {status}')
        fallback = np.flatnonzero(uncertain)
        if len(fallback):
            result[fallback] = original(part[fallback],chosen[fallback])
        self.calls += 1; self.rows += len(part); self.fallback_rows += len(fallback)
        return result.astype(bool)

    def function_variant(self):
        text = inspect.getsource(_ORIGINAL)
        assert text.count('    def evaluate(part, chosen):') == 1
        text = text.replace('    def evaluate(part, chosen):','    def numpy_evaluate(part, chosen):')
        hook = '''    def evaluate(part, chosen):
        return _kernel.evaluate(part, chosen, origins, directions, indices, valid,
                                depth, pad, cycle_count, numpy_evaluate)

'''
        point = '    for start in range(0, len(q), 128):'
        assert text.count(point) == 1
        text = text.replace(point, hook+'    for start in range(0, len(q), 512):')
        text = text.replace('q[start:start+128]','q[start:start+512]')
        scope = {**module.__dict__, '_kernel':self}
        exec(compile(text,'<native_depth_candidate>','exec'),scope)
        return scope['depth_support']
