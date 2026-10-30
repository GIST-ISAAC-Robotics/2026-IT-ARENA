"""중앙 표적 통로의 해석적 성립 조건. 제어/보호 코드를 바꾸지 않는 진단이다.

입력은 현행 평가 장면 치수와 순차 스캔 시각표뿐이다. 차체 중심 y 구간을
평행 자세와 요각 자세에 대해 계산한다. 물체 운동 항의 값은 가정이며 측정값이 아니다.
"""
import json
import math
from pathlib import Path

WALL=.425;TARGET_HALF=.075;BODY_L=.20;BODY_W=.15
FIXED=.025+.03;WALL_PAD=FIXED
SCAN=.1;TICK=.02;REUSE_TICKS=4


def age_at_publish(bearing_deg):
    """시뮬레이터 순차 광선: -180도 시작, 양의 방위로 회전. 첫 광선 나이 .1 s."""
    return SCAN*(1-(bearing_deg+180)/360)


def half_extent(yaw):
    return BODY_L/2*abs(math.sin(yaw))+BODY_W/2*math.cos(yaw)


def corridor(pad_target,yaw=0.,pad_wall=WALL_PAD):
    h=half_extent(yaw)
    return TARGET_HALF+h+pad_target,WALL-h-pad_wall


models={
 'fixed_only':lambda age:0.,
 'existing_0.5mps':lambda age:.5*age,
 'onset_a2_v0.05':lambda age:.05*age+.5*2.*age*age,
 'onset_a3_v0.05':lambda age:.05*age+.5*3.*age*age,
 'onset_a5_v0.05':lambda age:.05*age+.5*5.*age*age,
}
rows=[]
for side,bearing in (('left_pass_target_on_right',-90.),('right_pass_target_on_left',90.)):
    fresh=age_at_publish(bearing);stale=fresh+REUSE_TICKS*TICK
    for name,model in models.items():
        for label,age in (('fresh',fresh),('stale',stale)):
            pad=FIXED+model(age)
            lo,hi=corridor(pad)
            row=dict(side=side,model=name,phase=label,age_s=round(age,4),target_pad_m=round(pad,4),
                y_low_m=round(lo,4),y_high_m=round(hi,4),width_mm=round((hi-lo)*1000,1))
            # 폭이 남는 최대 요각. 대략 추종 허용 요각의 상한이다.
            yaw=0.
            while yaw<.6 and (lambda b:b[1]-b[0])(corridor(pad,yaw))>=0:yaw+=.001
            row['max_parallel_yaw_rad']=round(max(0.,yaw-.001),3)
            rows.append(row)
result=dict(scope='analytic_center_target_rectangular_padding_parallel_or_yawed_body',
    geometry=dict(wall_inner_y_m=WALL,target_half_width_m=TARGET_HALF,body_length_m=BODY_L,body_width_m=BODY_W,
        fixed_padding_m=FIXED,wall_padding_m=WALL_PAD,scan_period_s=SCAN,reuse_ticks=REUSE_TICKS),
    physical_body_center_interval_m=[.15,.35],physical_clearance_at_center_m=.10,rows=rows,
    limits=['rectangular padding as in swept_limit','no tracking error, no path, no discrete sampling',
        'onset parameters are assumptions, not measured vehicle limits'])
Path('analytic.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
print(f"{'side':28}{'model':18}{'phase':6}{'age':>7}{'pad':>7}{'lo':>7}{'hi':>7}{'w_mm':>7}{'yawmax':>7}")
for r in rows:
    print(f"{r['side']:28}{r['model']:18}{r['phase']:6}{r['age_s']:7.3f}{r['target_pad_m']:7.3f}{r['y_low_m']:7.3f}{r['y_high_m']:7.3f}{r['width_mm']:7.1f}{r['max_parallel_yaw_rad']:7.3f}")
