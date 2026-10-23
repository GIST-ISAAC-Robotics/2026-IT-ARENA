"""v1 진단의 배열 비교 오류를 고친 동일 30조건. 운영 소스 변경 없음."""
import importlib.util
from pathlib import Path
import math
import numpy as np

BASE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('geometry_probe',BASE.parent/'g1_corner_geometry_v1/probe.py')
probe=importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)


def geometry(entry,kind):
    theta=np.linspace(-math.pi/2,0.,121)
    arc=np.column_stack((entry+.85*np.cos(theta),.425+.85*np.sin(theta)))
    segments=[((-3.,-.425),(entry,-.425)),*list(zip(arc[:-1],arc[1:])),
              ((entry+.85,.425),(entry+.85,3.)),((entry+.85,3.),(entry,3.))]
    if kind not in ('missing_inner','short_fake_inner'):
        segments += [((-3.,.425),(entry,.425))]
    if kind!='missing_inner':
        segments += [((entry,.425),(entry,3.))]
    if kind=='center_block': segments += probe.box(.26,0.,.20,.20)
    if kind=='side_block': segments += probe.box(.35,.18,.20,.20)
    if kind=='short_fake_inner':segments+=probe.box(0.,.50,.20,.15)
    return segments


probe.BASE=BASE
probe.geometry=geometry
probe.main()
