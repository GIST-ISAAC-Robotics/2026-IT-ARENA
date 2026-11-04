"""기존 JR50/광축 105 mm 하부 배치 한 개를 고정한 비교. 새 설계나 동역학 해석 아님."""
import json
import math
from pathlib import Path


def wheel_gap(front):
    # 구형 CAD 앞바퀴 평면 사각형(64.4 x 26.1 mm)과 Ø7 지주 원의 부호 있는 외부 간격.
    samples = []
    for deg in range(-26,27,2):
        a=math.radians(deg)
        dx,dy=front-66.5,62-60.95
        u=abs(dx*math.cos(a)+dy*math.sin(a))-32.2
        v=abs(-dx*math.sin(a)+dy*math.cos(a))-13.05
        samples.append((math.hypot(max(u,0),max(v,0))+min(max(u,v),0)-3.5,deg))
    return round(min(samples)[0],2)


def box_gap(a,b):
    # 축정렬 직육면체 사이의 거리. 겹침이면 0 (간섭 체적은 계산하지 않음).
    return round(math.sqrt(sum(max(0,a[k]-b[k+3],b[k]-a[k+3])**2 for k in range(3))),2)


if __name__ == '__main__':
    prior=Path(__file__).resolve().parents[2]/'lidar_packaging_v3'/'lower_fit.json'
    lower=json.loads(prior.read_text(encoding='utf8'))['JR50_h105_stepped_steer26_diff1_posts']
    for s,f in [(50,10),(60,10),(60,15),(60,20),(65,10),(65,25)]:
        body=[s-27.8,-27.8,105-29.8-2,s+27.8,27.8,105+11.5]
        print(s,f,'front_post_to_steered_tire_mm',wheel_gap(f),'lidar_and_tray_to_parts_mm',
              {k:box_gap(body,b) for k,b in lower.items()})
