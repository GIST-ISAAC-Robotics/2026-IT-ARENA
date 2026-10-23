#!/usr/bin/env python3
"""직선-반원 연결 합성 코스. 정답 세계는 센서 생성/사후 평가 전용."""
import argparse
import json
import math
from pathlib import Path
from types import SimpleNamespace
import zipfile

import numpy as np
from shapely.geometry import LineString, Point, Polygon

from compare_corridor_closed_loop import SPEC, ROOT, digest, poses_at, run_case
from g0_geometry import Footprint


def stadium_hits(origins,directions,center_y,radius,half_length=1.):
    """수평 두 선분과 바깥쪽 반원 둘의 실제 교점만 사용한다."""
    hits=np.full(len(origins),np.inf)
    for y in (center_y-radius,center_y+radius):
        distance=np.full(len(origins),np.inf)
        np.divide(y-origins[:,1],directions[:,1],out=distance,where=np.abs(directions[:,1])>1e-14)
        x=origins[:,0]+np.where(np.isfinite(distance),distance,0.)*directions[:,0]
        valid=(distance>=0)&(np.abs(x)<=half_length+1e-12)
        hits=np.minimum(hits,np.where(valid,distance,np.inf))
    for sign in (-1,1):
        delta=origins-[sign*half_length,center_y]
        along=np.sum(delta*directions,axis=1)
        disc=along*along-np.sum(delta*delta,axis=1)+radius*radius
        root=np.sqrt(np.maximum(0.,disc))
        for distance in (-along-root,-along+root):
            x=origins[:,0]+distance*directions[:,0]
            valid=(disc>=0)&(distance>=0)&(sign*x>=half_length-1e-12)
            hits=np.minimum(hits,np.where(valid,distance,np.inf))
    return hits


class Stadium:
    def __init__(self,sign):
        if sign not in (-1,1):raise ValueError('left/right required')
        self.sign=sign
        self.radius=.15
        self.center_y=sign*(self.radius+.425)
        self.spine=LineString([(-1.,self.center_y),(1.,self.center_y)])
        self.circumference=4+math.tau*(self.radius+.425)
        self.minimum_progress_m=self.circumference/2

    def initial_pose(self):
        return np.array([-.8,0.,0.])

    def scan_at(self,now,truth):
        n=SPEC['rays_per_scan']
        first,increment=now-.1,.1/n
        poses=poses_at(truth,first+np.arange(n)*increment)
        yaw=poses[:,2]
        origins=poses[:,:2]+.06*np.column_stack((np.cos(yaw),np.sin(yaw)))
        angles=-math.pi+np.arange(n)*math.tau/n+yaw
        directions=np.column_stack((np.cos(angles),np.sin(angles)))
        ranges=np.minimum(stadium_hits(origins,directions,self.center_y,self.radius),
                          stadium_hits(origins,directions,self.center_y,self.radius+.85))
        ranges=np.where(ranges<.05,np.nan,ranges)
        nanos=round(first*1e9)
        return SimpleNamespace(header=SimpleNamespace(frame_id='laser_frame',
            stamp=SimpleNamespace(sec=nanos//10**9,nanosec=nanos%10**9)),ranges=ranges.tolist(),
            angle_min=-math.pi,angle_increment=math.tau/n,range_min=.05,range_max=12.,time_increment=increment)

    def audit(self,pose):
        polygon=Polygon(Footprint().polygon(*pose))
        near=polygon.distance(self.spine)
        far=max(Point(point).distance(self.spine) for point in polygon.exterior.coords)
        return dict(inside_road=near>=self.radius+.20 and far<=self.radius+.65,
                    collision=near<self.radius or far>self.radius+.85,body_gap_m=None)

    def coordinate(self,pose):
        x,y=pose[:2]
        y=self.sign*y
        cy,r=abs(self.center_y),self.radius+.425
        if x>1:
            return 2+r*(math.atan2(y-cy,x-1)+math.pi/2)
        if x<-1:
            angle=(math.atan2(y-cy,x+1)-math.pi/2)%math.tau
            return 4+math.pi*r+r*angle
        return x+1 if y<cy else 2+math.pi*r+1-x

    def progress(self,truth):
        coordinates=[self.coordinate(pose) for pose in truth[:,1:4]]
        unwrapped=np.unwrap(np.array(coordinates)*math.tau/self.circumference)*self.circumference/math.tau
        return float(unwrapped[-1]-unwrapped[0])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    sources={str(p.relative_to(ROOT)):digest(p) for folder in ('src/arena_autonomy/arena_autonomy',
             'src/arena_vehicle_interface/arena_vehicle_interface') for p in (ROOT/folder).glob('*.py')}
    for p in ('scripts/compare_corridor_closed_loop.py','scripts/compare_corridor_stadium.py','scripts/g0_geometry.py',
              'scripts/probe_corner_corridor.py','tests/test_corridor_closed_loop.py','tests/test_corridor_stadium.py'):
        sources[p]=digest(ROOT/p)
    with zipfile.ZipFile(args.output/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for p in sources:archive.write(ROOT/p,p)
    results=[]
    for sign in (1,-1):
        for mode in ('baseline','corridor'):
            result=run_case(sign,'open',mode,duration=12.,world=Stadium(sign))
            name=f'{sign}_{mode}.json'
            with (args.output/name).open('x') as stream:json.dump(result.pop('trace'),stream,allow_nan=False)
            result.update(trace=name,trace_sha256=digest(args.output/name))
            results.append(result)
            print(json.dumps(result),flush=True)
    report=dict(scope='synthetic_straight_turn_handoff_only',spec={**SPEC,'geometry':'stadium',
                'straight_segment_length_m':2.,'duration_s':12.,'minimum_progress_m':Stadium(1).minimum_progress_m},
                source_sha256=sources,source_unchanged=all(digest(ROOT/p)==h for p,h in sources.items()),
                results=results,runtime_changed=False,official_lap_verified=False,real_vehicle_verified=False)
    with (args.output/'report.json').open('x') as stream:json.dump(report,stream,indent=2,allow_nan=False)
    return 0 if report['source_unchanged'] else 2


if __name__=='__main__':raise SystemExit(main())
