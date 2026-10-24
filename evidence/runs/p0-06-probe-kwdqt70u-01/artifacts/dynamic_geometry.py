"""Offline cylinder ray tests and bounded source-time interpolation. No ROS."""
import math
class PoseSeries:
    def __init__(self,samples,epoch,entity,max_gap_ns):
        from bisect import bisect_left
        self.bisect=bisect_left;self.samples=samples;self.times=[];self.gap=max_gap_ns
        if not samples:raise ValueError('missing pose')
        for s in samples:
            if s['epoch']!=epoch or s['entity_id']!=entity or s['frame']!='world':raise ValueError('entity/epoch/frame')
            if self.times and s['t_ns']<=self.times[-1]:raise ValueError('unordered/duplicate pose')
            if not all(math.isfinite(x) for x in s['xyyaw']):raise ValueError('nonfinite pose')
            self.times.append(s['t_ns'])
    def at(self,t):
        index=self.bisect(self.times,t)
        if index<len(self.times) and self.times[index]==t:return self.samples[index]['xyyaw']
        if index==0 or index==len(self.times):raise ValueError('no extrapolation')
        a,b=self.samples[index-1],self.samples[index]
        if b['t_ns']-a['t_ns']>self.gap:raise ValueError('truth gap')
        u=(t-a['t_ns'])/(b['t_ns']-a['t_ns'])
        da=math.atan2(math.sin(b['xyyaw'][2]-a['xyyaw'][2]),math.cos(b['xyyaw'][2]-a['xyyaw'][2]))
        return [a['xyyaw'][0]+u*(b['xyyaw'][0]-a['xyyaw'][0]),a['xyyaw'][1]+u*(b['xyyaw'][1]-a['xyyaw'][1]),a['xyyaw'][2]+u*da]
    def xyz_at(self,t):
        index=self.bisect(self.times,t)
        if index<len(self.times) and self.times[index]==t:return self.samples[index]['xyz']
        if index==0 or index==len(self.times):raise ValueError('no extrapolation')
        a,b=self.samples[index-1],self.samples[index]
        if b['t_ns']-a['t_ns']>self.gap:raise ValueError('truth gap')
        u=(t-a['t_ns'])/(b['t_ns']-a['t_ns'])
        return [a['xyz'][i]+u*(b['xyz'][i]-a['xyz'][i]) for i in range(3)]
def interpolate(samples,t,epoch,entity,max_gap_ns):
    return PoseSeries(samples,epoch,entity,max_gap_ns).at(t)

def ray_circle(origin,angle,circle):
    dx,dy=circle[0]-origin[0],circle[1]-origin[1]
    along=dx*math.cos(angle)+dy*math.sin(angle);disc=circle[2]**2-(dx*dx+dy*dy-along*along)
    if disc<0 or along<0:return None
    near=along-math.sqrt(disc)
    return near if near>0 else None
def classify(scan,robot,obstacle,occluders,extrinsic,tolerance=.05):
    # Missing frames, ranges or geometry never become false/occluded.
    if scan.get('frame_id')!='base_scan' or not scan.get('ranges') or extrinsic is None:return None,[]
    x,y,yaw=robot;ex,ey,ea=extrinsic
    origin=[x+math.cos(yaw)*ex-math.sin(yaw)*ey,y+math.sin(yaw)*ex+math.cos(yaw)*ey]
    evidence=[]
    for i,r in enumerate(scan['ranges']):
        angle=yaw+ea+scan['angle_min']+i*scan['angle_increment']
        target=ray_circle(origin,angle,obstacle)
        if target is None or not scan['range_min']<=target<=scan['range_max']:continue
        foreground=[d for c in occluders if (d:=ray_circle(origin,angle,c)) is not None and d<target-tolerance]
        expected=min(foreground) if foreground else target
        if r is None or not math.isfinite(r) or not scan['range_min']<=r<=scan['range_max']:return None,evidence
        evidence.append({'beam':i,'range':r,'target_surface':target,'foreground_surface':min(foreground) if foreground else None,'matches':abs(r-expected)<=tolerance})
    if len(evidence)<3 or not all(x['matches'] for x in evidence):return None,evidence
    blocked=[x['foreground_surface'] is not None for x in evidence]
    if all(blocked):return 'occluded',evidence
    if not any(blocked):return 'visible',evidence
    return 'partial',evidence

def polyline_crossing(robot,actor,max_gap_ns=50000000):
    """Require non-parallel segment intersection of the actual paths."""
    def cross(a,b):return a[0]*b[1]-a[1]*b[0]
    for a,b in zip(robot,robot[1:]):
        if not 0<b['t_ns']-a['t_ns']<=max_gap_ns:continue
        p=a['xyz'][:2];u=[b['xyz'][i]-p[i] for i in range(2)]
        if math.hypot(*u)<1e-6:continue
        for c,d in zip(actor,actor[1:]):
            if not 0<d['t_ns']-c['t_ns']<=max_gap_ns:continue
            q=c['xyz'][:2];v=[d['xyz'][i]-q[i] for i in range(2)]
            denom=cross(u,v);norm=math.hypot(*u)*math.hypot(*v)
            if not norm or abs(denom)/norm<.5:continue
            w=[q[i]-p[i] for i in range(2)];s=cross(w,v)/denom;t=cross(w,u)/denom
            if 0<=s<=1 and 0<=t<=1:
                rt=round(a['t_ns']+s*(b['t_ns']-a['t_ns']));at=round(c['t_ns']+t*(d['t_ns']-c['t_ns']))
                return {'xy':[p[i]+s*u[i] for i in range(2)],'robot_source_ns':[a['t_ns'],b['t_ns']],'actor_source_ns':[c['t_ns'],d['t_ns']],'robot_intersection_estimated_ns':rt,'actor_intersection_estimated_ns':at,'ahead_of_robot':at<=rt,'simultaneous_contact_claim':False}
    return None
