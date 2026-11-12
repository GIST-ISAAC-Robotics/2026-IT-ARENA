"""제작측 11/12 STL을 이전 원본과 비교. mm 단위/세로 Y는 기존 제작측 모델 기준."""
import importlib.util
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import numpy as np

ROOT=Path(__file__).resolve().parents[4]
OUT=Path(__file__).resolve().parent
OLD=ROOT/'artifacts/validation/2026-11-09/signal_post_source_v1'
spec=importlib.util.spec_from_file_location('prior_stl_audit',OLD/'analyze_stl.py')
audit=importlib.util.module_from_spec(spec);spec.loader.exec_module(audit)


def inspect(path):
    _,_,tri,_=audit.load(path);v=tri.reshape(-1,3);low=v.min(0);high=v.max(0)
    sections={}
    for z in (1,5,9,11,20,50,75,134.6,200,350,397,419,449,464,480,500,530,541):
        segments=audit.slice_xy(tri,1,low[1]+z)
        if not segments:continue
        p=np.concatenate(segments)[:,[0,2]]
        xs=np.unique(np.round(p[:,0],3));groups=np.split(xs,np.where(np.diff(xs)>30)[0]+1)
        sections[str(z)]=[]
        for group in groups:
            selected=p[(p[:,0]>=group.min()-.001)&(p[:,0]<=group.max()+.001)]
            lo,hi=selected.min(0),selected.max(0)
            sections[str(z)].append(dict(min_xz_mm=lo.tolist(),max_xz_mm=hi.tolist(),size_xz_mm=(hi-lo).tolist(),center_xz_mm=((lo+hi)/2).tolist()))
    return dict(triangles=len(tri),min_mm=low.tolist(),max_mm=high.tolist(),size_mm=(high-low).tolist(),
                components=audit.components(tri),sections=sections),tri


def main():
    old,ot=inspect(OLD/'download/STadium_starting_light.stl')
    new,nt=inspect(OUT/'STardium_starting_light_new.stl')
    result=dict(units='mm assumed from same manufacturer STL and previous dimensions',
                vertical_axis='raw Y, same orientation as old file; rendering inspected separately',old=old,new=new)
    (OUT/'geometry.json').write_text(json.dumps(result,indent=2)+'\n')
    fig,axes=plt.subplots(2,2,figsize=(12,10),layout='constrained')
    for col,(name,tri,data) in enumerate([('Previous',ot,old),('2026-11-12',nt,new)]):
        for row,xaxis in enumerate([0,2]):
            edges=np.concatenate((tri[:,[0,1],:],tri[:,[1,2],:],tri[:,[2,0],:]))
            edge=edges[:,:,[xaxis,1]].copy();edge[:,:,1]-=data['min_mm'][1]
            axes[row,col].add_collection(LineCollection(edge,lw=.15,color='#184d72'))
            axes[row,col].autoscale();axes[row,col].set_aspect('equal')
            axes[row,col].set(xlabel='STL X (mm)' if xaxis==0 else 'STL Z (mm)',ylabel='Height from base (mm)',title=name+(' front' if row==0 else 'side'))
            axes[row,col].grid(alpha=.2)
    fig.savefig(OUT/'comparison.png',dpi=150)
    for name,data in [('old',old),('new',new)]:
        print(json.dumps(dict(name=name,size_mm=data['size_mm'],components=data['components'],scan_plane134_6=data['sections'].get('134.6'),foot=data['sections'].get('5'))))


if __name__=='__main__':main()
