"""같은 실제 입력·같은 표본 선택에서 9점 검사 대신 광선별 깊이를 적용한다."""
import json
from pathlib import Path
import sys
import sensor_turn_probe as probe
from ray_turn_preview import classify


if __name__=='__main__':
    run,out=map(Path,sys.argv[1:])
    if out.exists(): raise FileExistsError(out)
    probe.classify=classify
    result=probe.measure(run)
    result['scope']='저장 입력·센서 기반 경로를 따라 반환, 무반환, 가려진 통로를 빔별로 비교. 폐루프·실물 결과가 아니다.'
    with out.open('x',encoding='utf-8') as f:json.dump(result,f,ensure_ascii=False,indent=2)
    print(result['counts'])
