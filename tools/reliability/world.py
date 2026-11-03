# SPDX-License-Identifier: MIT
"""Offline exact consumed-window cross-check. No suffix selection, ROS or control."""
import json,math
from pathlib import Path

def angle(a,b):return abs(math.atan2(math.sin(a-b),math.cos(a-b)))
def check_windows(root,windows):
 root=Path(root);bounds=[(round(w['from_s']*1e9),round(w['to_s']*1e9)) for w in windows];buckets=[[] for _ in windows];errors=[]
 if not windows:return {'valid':False,'errors':['no consumed stop windows'],'windows':[]}
 with (root/'runtime/p0-08-gazebo.jsonl').open() as stream:
  for line in stream:
   row=json.loads(line)
   if row['kind']!='pose':continue
   if row['run_id']!=root.name or row['clock_epoch']!=root.name+':sim0':raise ValueError('world pose run/clock identity')
   m=row['protobuf'];stamp=m.get('header',{}).get('stamp',{});sec=int(stamp.get('sec',0));ns=int(stamp.get('nsec',0))
   if sec<0 or not 0<=ns<1000000000:raise ValueError('world source time')
   t=sec*1000000000+ns;chosen=[i for i,(lo,hi) in enumerate(bounds) if lo-1000000000<=t<=hi+1000000000]
   if not chosen:continue
   for pose in m.get('pose',[]):
    if pose.get('name')!='turtlebot3_waffle':continue
    v=pose['position'];q=pose['orientation'];xy=[float(v.get(k,0)) for k in ['x','y']];quat=[float(q.get(k,0)) for k in ['x','y','z','w']]
    if not all(math.isfinite(x) for x in xy+quat) or abs(sum(x*x for x in quat)-1)>1e-3:raise ValueError('world pose finite/unit quaternion')
    x,y,z,w=quat;yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z));sample=(t,xy,yaw,row['seq'])
    for i in chosen:buckets[i].append(sample)
 results=[]
 for i,(lo,hi) in enumerate(bounds):
  values=buckets[i];left=[j for j,x in enumerate(values) if x[0]<=lo];right=[j for j,x in enumerate(values) if x[0]>=hi];r={'attempt':windows[i]['attempt'],'from_ns':lo,'to_ns':hi,'valid':False,'reason':'missing bracketing samples'}
  if left and right:
   selected=values[max(0,left[-1]-1):right[0]+1];gaps=[b[0]-a[0] for a,b in zip(selected,selected[1:])]
   if len(selected)>=5 and hi-lo>=3000000000 and gaps and all(0<x<=500000000 for x in gaps):
    v=max(math.dist(a[1],b[1])/(dt*1e-9) for a,b,dt in zip(selected,selected[1:],gaps));w=max(angle(a[2],b[2])/(dt*1e-9) for a,b,dt in zip(selected,selected[1:],gaps));drift=max(math.dist(x[1],selected[0][1]) for x in selected);yaw_drift=max(angle(x[2],selected[0][2]) for x in selected)
    r.update(valid=v<=.01 and w<=.03 and drift<=.02 and yaw_drift<=.03,reason='EXACT_BRACKETED_WINDOW',max_v=v,max_w=w,drift_m=drift,yaw_drift_rad=yaw_drift,raw_seq_range=[selected[0][3],selected[-1][3]],samples=len(selected))
   else:r['reason']='source gap/order/window'
  if not r['valid']:errors.append('world exact consumed window '+r['attempt'])
  results.append(r)
 return {'valid':not errors,'errors':errors,'windows':results,'scope':'offline extra Gazebo pose cross-check; no replacement by a later suffix'}
