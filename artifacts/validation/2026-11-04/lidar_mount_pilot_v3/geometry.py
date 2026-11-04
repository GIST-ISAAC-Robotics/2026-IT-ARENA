"""위치 후보의 평면 기하 선별. 광학점 모델과 CAD 본체 외곽 참고값을 구분한다."""
from pathlib import Path
import json,math
HERE=Path(__file__).resolve().parent
rows=[]
for x in (30,45,50,60,65,70):
 distance=math.hypot(80-x,62)
 center=math.degrees(math.atan2(62,80-x))
 half=math.degrees(math.asin(3.5/distance))+.5
 rows.append(dict(name='C'+str(x),sensor_x_mm=x,front_post_bearing_deg=center,
   front_post_half_mask_deg=half,front_unoccluded_fan_deg=2*(center-half),
   sensor_front_body_clearance_mm=100-x-27.8,
   sensor_to_front_post_distance_mm=distance,
   modeled_min_range_front_band_mm=max(0,50-(100-x))))
x=50;distance=math.hypot(10-x,62);center=math.degrees(math.atan2(62,10-x));half=math.degrees(math.asin(3.5/distance))+.5
rows.append(dict(name='JR50',sensor_x_mm=x,front_post_bearing_deg=center,front_post_half_mask_deg=half,
 front_unoccluded_fan_deg=2*(center-half),sensor_front_body_clearance_mm=100-x-27.8))
result=dict(rows=rows,selected=['C30','C65','JR50'],reason='C65는 C70보다 앞 본체 여유 5mm가 크고 전방 팬을 크게 넓힌다. 최종 부품 간섭/마운트/케이블/조향 검증 아님.',
 assumptions='차체 외곽200x150mm, C1 본체55.6mm, 지주Ø7, 각 구간여유0.5°, 공통광축75mm. 최소거리50mm는 광축 정면 선상 기하일 뿐 물체전체 미검출 판정 아님.')
(HERE/'geometry.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
print(json.dumps(result,ensure_ascii=False))
