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
    def quat_at(self,t):
        index=self.bisect(self.times,t)
        def quat(s):
            return s.get('quaternion_xyzw',[0.,0.,math.sin(s['xyyaw'][2]/2),math.cos(s['xyyaw'][2]/2)])
        if index<len(self.times) and self.times[index]==t:return quat(self.samples[index])
        if index==0 or index==len(self.times):raise ValueError('no extrapolation')
        a,b=self.samples[index-1],self.samples[index]
        if b['t_ns']-a['t_ns']>self.gap:raise ValueError('truth gap')
        u=(t-a['t_ns'])/(b['t_ns']-a['t_ns']);qa,qb=quat(a),quat(b);dot=sum(x*y for x,y in zip(qa,qb))
        if dot<0:qb=[-x for x in qb];dot=-dot
        if dot>.9995:q=[(1-u)*x+u*y for x,y in zip(qa,qb)]
        else:
            theta=math.acos(max(-1.,min(1.,dot)));den=math.sin(theta)
            q=[(math.sin((1-u)*theta)*x+math.sin(u*theta)*y)/den for x,y in zip(qa,qb)]
        norm=math.sqrt(sum(x*x for x in q));return [x/norm for x in q]

def rotate(q,p):
    x,y,z,w=q;t=[2*(y*p[2]-z*p[1]),2*(z*p[0]-x*p[2]),2*(x*p[1]-y*p[0])]
    return [p[0]+w*t[0]+y*t[2]-z*t[1],p[1]+w*t[1]+z*t[0]-x*t[2],p[2]+w*t[2]+x*t[1]-y*t[0]]

def rpy_quat(rpy):
    r,p,y=[v/2 for v in rpy];cr,sr=math.cos(r),math.sin(r);cp,sp=math.cos(p),math.sin(p);cy,sy=math.cos(y),math.sin(y)
    return [sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy,cr*cp*cy+sr*sp*sy]

def ray_cylinder_3d(origin,direction,circle,bottom,height):
    dx,dy=origin[0]-circle[0],origin[1]-circle[1];vx,vy,vz=direction;a=vx*vx+vy*vy
    if a<1e-14:return None
    b=2*(dx*vx+dy*vy);c=dx*dx+dy*dy-circle[2]**2;disc=b*b-4*a*c
    if disc<0:return None
    candidates=[]
    for t in [(-b-math.sqrt(disc))/(2*a),(-b+math.sqrt(disc))/(2*a)]:
        if t>0 and bottom<=origin[2]+t*vz<=bottom+height:candidates.append(t)
    if abs(vz)>1e-14:
        for z in [bottom,bottom+height]:
            t=(z-origin[2])/vz
            if t>0 and (dx+t*vx)**2+(dy+t*vy)**2<=circle[2]**2:candidates.append(t)
    return min(candidates) if candidates else None

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
