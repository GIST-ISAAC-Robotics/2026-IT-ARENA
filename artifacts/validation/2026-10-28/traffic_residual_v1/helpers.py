import math
import numpy as np

def interpolated(rows,t,maximum_gap):
    i=int(np.searchsorted(rows[:,0],t))
    if i==0 or i==len(rows) or rows[i,0]-rows[i-1,0]>maximum_gap:raise ValueError('pose interpolation unavailable')
    a,b=rows[i-1],rows[i];f=(t-a[0])/(b[0]-a[0])
    yaw=a[3]+f*((b[3]-a[3]+math.pi)%math.tau-math.pi)
    return np.r_[a[1:3]+f*(b[1:3]-a[1:3]),yaw]

def world(p,pose):
    c,s=math.cos(pose[2]),math.sin(pose[2])
    return np.asarray(p)@np.array([[c,s],[-s,c]])+pose[:2]

def local(p,pose):
    c,s=math.cos(pose[2]),math.sin(pose[2])
    return (np.asarray(p)-pose[:2])@np.array([[c,-s],[s,c]])
