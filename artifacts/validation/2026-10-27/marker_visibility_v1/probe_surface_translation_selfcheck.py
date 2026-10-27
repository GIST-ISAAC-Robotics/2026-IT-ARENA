"""표면 정합 분석 함수의 제한 합성 대조. 런타임 채택/주행 시험이 아니다."""
import json
from pathlib import Path
import numpy as np
from probe_surface_translation import fit_translation

x=np.linspace(-.1,1.5,81)
y=np.linspace(-.425,.425,51)
walls=np.concatenate([np.column_stack((x,np.full(len(x),.425))),
                      np.column_stack((x,np.full(len(x),-.425))),
                      np.column_stack((np.full(len(y),1.5),y))])
ids=np.r_[np.arange(81),np.arange(100,181),np.arange(200,251)]
shift=np.array([.025,.012])
fit=fit_translation(walls-shift,walls,ids)
assert fit['accepted'] and np.linalg.norm(np.array(fit['translation_m'])-shift)<1e-5,fit
parallel=fit_translation(walls[:162]-shift,walls[:162],ids[:162])
assert not parallel['accepted'] and parallel['reason']=='translation_unobservable',parallel
large=fit_translation(walls-np.array([.055,.055]),walls,ids)
assert not large['accepted'],large
car=np.column_stack((np.full(12,.35),np.linspace(-.07,.07,12)))
disappeared=fit_translation(np.concatenate([walls,car])-shift,walls,ids)
assert disappeared['accepted'],disappeared
corrected_car=car-shift+np.array(disappeared['translation_m'])
nearest=np.min(np.linalg.norm(corrected_car[:,None]-walls[None,:],axis=2),axis=1)
assert np.min(nearest)>.3,nearest
result=dict(scope='limited_synthetic_offline_registration_only',passed=True,
            translated_static_scene=fit,parallel_walls=parallel,outside_bound=large,
            disappeared_car=disappeared,disappeared_car_minimum_current_distance_m=float(np.min(nearest)),
            residual_risks=['congruent motion of all visible surfaces cannot be identified as ego motion',
                            'moving dominant surfaces and real sensor errors require separate evidence'])
out=Path(__file__).with_name('surface_translation_selfcheck.json')
with out.open('x',encoding='utf-8') as stream:json.dump(result,stream,indent=2)
print(json.dumps({'passed':True,'synthetic_cases':4}))
