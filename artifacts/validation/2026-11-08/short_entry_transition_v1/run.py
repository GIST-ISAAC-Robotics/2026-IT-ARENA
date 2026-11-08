"""평가 전용 r39: r33과 같은 정지차 STOP -> RESET 없는 START. launch 인자 하나만 추가한다.

r33 run.py의 시나리오·정지차 위치·1.2 m 진행·25초 한도·STOP 정착 측정·인자를 그대로 쓰며
evaluation_stopped_entry_centering:=true 뒤에 evaluation_adaptive_entry_preview:=true만 붙인다.
출력·registered.json은 이 폴더 기준이다.
"""
from r33_tools import execute,launch_patch
execute('run.py',launch_patch())
