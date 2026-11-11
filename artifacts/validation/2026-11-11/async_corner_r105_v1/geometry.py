"""저장 경계 조사에서 선택된 실제 점의 모서리 적합 실패를 분해한다."""
import json
from pathlib import Path
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy')]
from arena_autonomy.local_path import local_path
from arena_autonomy import piecewise_wall as pw


def main():
    data=json.loads((A/'probe.json').read_text('utf8'))
    chosen=[f for f in data['frames'] if 49.95 < f['reference_s'] < 50.3]
    figures,axes=plt.subplots(1,len(chosen),figsize=(5*len(chosen),5),squeeze=False)
    rows=[];original=pw.corner_path
    for frame,ax in zip(chosen,axes[0]):
        evidence=[]
        def corner(samples,side,offset,radius):
            result=original(samples,side,offset,radius);vertices=pw._vertices(samples)
            splits=[]
            if len(vertices)==3:
                for pts in (samples[:vertices[1]+1],samples[vertices[1]:]):
                    center=pts.mean(axis=0);t=np.linalg.svd(pts-center,full_matrices=False)[2][0]
                    residual=np.abs((pts-center) @ [-t[1],t[0]])
                    splits.append(dict(count=len(pts),extent_m=float(np.linalg.norm(pts[-1]-pts[0])),max_residual_m=float(max(residual))))
            evidence.append(dict(samples=samples.tolist(),vertices=vertices,lines=splits,accepted=result is not None))
            return result
        pw.corner_path=corner
        points=np.asarray(frame['points']);p,info=local_path(points,'right',.505,piecewise_corner=True,minimum_radius=.135/np.tan(.3710583448410034),narrow_occluder_bridge=True)
        pw.corner_path=original
        rows.append(dict(reference_s=frame['reference_s'],info={k:v for k,v in info.items() if k!='path_points'},evidence=evidence))
        ax.scatter(points[:,0],points[:,1],s=5,c='0.6',label='current returns')
        if p is not None:ax.plot(*p.T,c='#d62728',lw=2,label='current path')
        if frame['reference'] is not None:ax.plot(*np.asarray(frame['reference']).T,c='#1565c0',ls='--',label='transported path')
        for i,e in enumerate(evidence):
            v=np.asarray(e['samples']);ax.plot(*v.T,c='#008060',lw=1,alpha=.6,label='wall samples' if i==0 else None)
        ax.scatter([0],[0],c='black',marker='>',s=65)
        ax.set(xlim=(-.5,2),ylim=(-.8,1.5),title=f"t={frame['reference_s']:.2f}s",xlabel='forward x (m)',ylabel='left y (m)')
        ax.set_aspect('equal');ax.grid(alpha=.3);ax.legend(fontsize=7)
    figures.tight_layout();figures.savefig(A/'corner_transition.png',dpi=140);plt.close(figures)
    with (A/'geometry.json').open('x',encoding='utf8') as f:json.dump(rows,f,indent=2)
    for row in rows:
        print(json.dumps({**{k:v for k,v in row.items() if k!='evidence'},'evidence':[{k:v for k,v in e.items() if k!='samples'} for e in row['evidence']]}))

if __name__=='__main__':main()
