"""실제 run2 센서 좌표의 6cm 현 거절과 네 점 지지를 그린다."""
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

BASE=Path(__file__).resolve().parent


def main():
    source=BASE/'run2_reobservation_v1.json'
    target=BASE/'surface_witness_v2.png'
    receipt=BASE/'surface_witness_v2.json'
    if target.exists() or receipt.exists():raise FileExistsError(target)
    raw=source.read_bytes();data=json.loads(raw)
    evidence=data['initial_evidence'];context=evidence['correspondence_context']
    points=np.asarray(context['current_return_points_xy'])
    ids=np.asarray(context['current_return_source_indices'])
    patch=points[(ids>=377)&(ids<=380)]
    retained=np.asarray(data['rows'][0]['anchor']['corrected_points'])
    first=evidence['witnesses'][0]
    origin=np.asarray(first['origin_xy']);angle=first['direction_rad']
    endpoint=origin+np.array([np.cos(angle),np.sin(angle)])*.65
    fig,ax=plt.subplots(figsize=(7,9),layout='constrained')
    ax.scatter(points[:,0]*100,points[:,1]*100,s=18,c='#666666',label='Current measured returns')
    ax.plot(patch[:,0]*100,patch[:,1]*100,'o-',c='#1677b8',lw=2,label='Four consecutive returns 377-380')
    ax.plot(patch[:2,0]*100,patch[:2,1]*100,c='#e47b16',lw=3,label='68.08 mm chord (old limit: 60 mm)')
    ax.scatter(retained[:,0]*100,retained[:,1]*100,c='#b32988',marker='x',s=90,lw=2,label='Retained points after accepted alignment')
    ax.plot([origin[0]*100,endpoint[0]*100],[origin[1]*100,endpoint[1]*100],ls='--',c='#ca3030',lw=1.3,label='Missing ray 376 (range unknown)')
    for i,p in enumerate(patch):ax.annotate(str(377+i),p*100,xytext=(7,0),textcoords='offset points')
    ax.set_xlim(-1,6);ax.set_ylim(43,57);ax.set_aspect('equal')
    ax.set_xlabel('Vehicle x (cm), forward');ax.set_ylabel('Vehicle y (cm), left')
    ax.grid(alpha=.2);ax.legend(loc='upper center',bbox_to_anchor=(.5,-.09),fontsize=8)
    ax.set_title('Recorded scan at 133.56 s | local surface reobservation\n'
                 'Measured points only; candidate chord is not a new sensor return',fontsize=11)
    fig.savefig(target,dpi=160)
    assert source.read_bytes()==raw
    receipt.write_text(json.dumps(dict(input_sha256=hashlib.sha256(raw).hexdigest(),
        input_unchanged=True,scope='sensor-coordinate diagnostic plot; no object identity or free-space proof'),indent=2),encoding='utf-8')
    print(str(target))


if __name__=='__main__':main()
