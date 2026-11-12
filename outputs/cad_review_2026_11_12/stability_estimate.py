from pathlib import Path
import json,itertools,math
import numpy as np
OUT=Path(__file__).parent
g={r['id']:r for r in json.loads((OUT/'geometry.json').read_text(encoding='utf-8'))}
ids=[1841,1843,1844,1845,1846,1847,1851,2088,2089,2090,2091,2092,2368,2369,2370,2371,2372,2373]
vol=sum(g[i]['volume_mm3'] for i in ids)
ctr=sum(np.array(g[i]['uniform_centroid_mm'])*g[i]['volume_mm3'] for i in ids)/vol
# Masses are kg; unmeasured ranges are sensitivity assumptions, not measurements.
components=[
 ['배터리',.375,.375,[-56.5,-.5,84.7],'제품 표기'],
 ['Jetson',.175,.175,[60.65,.8,170],'NVIDIA 개발키트 표기; 질량중심 높이는 가정'],
 ['C1',.110,.110,[75.4,0,123],'제조사 표기; 질량중심 높이는 가정'],
 ['모터',.30,.50,[11.04,-22.06,67.74],'가정'],
 ['엔코더',.02,.05,[54.15,-23.52,80.56],'가정'],
 ['J01',.06,.16,[-49.66,1.7,168],'가정'],
 ['M01',.06,.16,[14.09,-3.51,205],'가정'],
 ['MD20A',.02,.05,[2.8,18.89,107.1],'가정'],
 ['서보',.055,.075,[76.49,17,76.4],'가정'],
 ['바퀴 4개',.08,.16,[0,0,32.2],'가정'],
 ['기어·축·베어링',.04,.09,[-35,0,35],'가정'],
 ['전방 조향 프레임·링크',.025,.065,[65,0,40],'가정'],
 ['확인된 PLA 구조',vol*.00124*.45/1000,vol*.00124*.85/1000,ctr.tolist(),'밀도1.24g/cm3·실효 충전45~85% 가정; 슬라이서 infill 비율과 다름'],
 ['소형 보드·카메라',.025,.05,[30,20,65],'가정'],
 ['케이블·체결구·CF 등',.05,.12,[0,0,105],'가정']]
mass=np.array([[c[i] for c in components] for i in [1,2]])
pos=np.array([c[3] for c in components]); m0=mass.mean(axis=0); cg0=m0@pos/m0.sum()
variable=np.where(mass[0]!=mass[1])[0]
allm=np.tile(mass[0],(2**len(variable),1))
for k,bits in enumerate(itertools.product([0,1],repeat=len(variable))):
 for i,b in zip(variable,bits):allm[k,i]=mass[b,i]
cgs=(allm@pos)/allm.sum(axis=1)[:,None]
track=123.899846726
tip=(track/2-abs(cgs[:,1]))/cgs[:,2]
out={'components':[dict(zip(['name','mass_min_kg','mass_max_kg','position_mm','status'],c)) for c in components],'printed_volume_cm3':vol/1000,'printed_solid_mass_g_at_density_1p24':vol*.00124,'printed_centroid_mm':ctr.tolist(),'nominal_mass_kg':float(m0.sum()),'nominal_cg_mm':cg0.tolist(),'total_mass_range_kg':[float(mass[0].sum()),float(mass[1].sum())],'cg_coordinate_ranges_mm':np.c_[cgs.min(axis=0),cgs.max(axis=0)].tolist(),'quasistatic_rollover_accel_g_range':[float(tip.min()),float(tip.max())],'nominal_rollover_accel_g':float((track/2-abs(cg0[1]))/cg0[2]),'nominal_front_axle_load_fraction':float((cg0[0]+64)/128),'note':'선택 질량/질량중심 가정의 정적 민감도. 모든 불확실성을 포함하는 보증 구간·실차 주행 한계가 아님.'}
# Plate change: compare 3 mm frame PLA at 65% effective fill vs 5 mm acrylic.
# Includes bosses in V/3 as a screening approximation; flat plate only is slightly lower.
plate_vol=sum(g[i]['volume_mm3'] for i in [2089,2090])
out['two_upper_frames_screen']={'pla_3mm_65pct_g':plate_vol*.00124*.65,'acrylic_5mm_g':plate_vol/3*5*.00119,'delta_g':plate_vol/3*5*.00119-plate_vol*.00124*.65}
out['turning_examples']=[{'speed_kmh':v*3.6,'radius_m':r,'lateral_accel_g':v*v/r/9.81} for v in [2,10/3.6] for r in [.5,1,1.5]]
(OUT/'stability.json').write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps({k:v for k,v in out.items() if k!='components'},ensure_ascii=False,indent=2))
