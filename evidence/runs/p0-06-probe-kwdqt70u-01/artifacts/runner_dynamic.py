"""Scene operations using the existing supervisor, links and budgets."""
from pathlib import Path
import json,math,time
from runner_core import write_json
from inspect_dynamic_recording import clock_alignment,raw_rows
class DynamicFailure(RuntimeError):
    def __init__(self,code,detail=''):
        self.code=code;super().__init__(code+': '+str(detail))
class DynamicSession:
    def __init__(self,supervisor,run,profile,env,log,worker,link_type):
        self.supervisor=supervisor;self.run=Path(run);self.profile=profile;self.worker=worker;self.log=log;self.links={};self.items={};self.entity=None;self.release=None
        for role in ['capture','driver']:
            item=supervisor.spawn('scene-'+role,[profile['dynamic_profile']['executables'][role]['path'],'--run-dir',str(run)],env,Path(__file__).resolve().parents[1],ipc=True)
            self.items[role]=item;peer=link_type(item,run.name,log,supervisor,self.event);self.links[role]=peer;worker.sidecars.append(peer)
    def event(self,event,received):
        if event['type']=='FAILURE':raise DynamicFailure(event['data']['code'],event['data'])
    def rpc(self,role,op,args,budget):
        try:return self.links[role].rpc(op,args,budget.child(min(2,budget.remaining())))
        except RuntimeError as exc:
            code=str(exc).split(': ')[-1]
            if code.replace('_','').isupper():raise DynamicFailure(code,str(exc)) from exc
            raise
    def status(self,budget):return self.rpc('driver','status',{},budget)
    def prepare(self,budget):
        interfaces=self.rpc('driver','interfaces',{},budget)
        self.rpc('capture','snapshot',{},budget)
        while True:
            budget.require();s=self.status(budget);c=self.rpc('capture','status',{},budget)
            if all(c['counts'].get(k,0)>0 for k in ['clock','stats','pose','scene_snapshot']) and s['entity_id'] is not None and s['actual_xyz'] is not None and abs(s['sim_ns']-s['pose_stamp_ns'])<=50000000:break
            self.worker.pump()
        motion=json.loads((self.run/'artifacts/dynamic_motion.json').read_text())
        if math.dist(s['actual_xyz'],motion['knots'][0]['xyz'])>.02:raise ValueError('INITIAL_ACTUAL_POSE_MISMATCH')
        self.entity=s['entity_id'];isolation=self.worker.rpc('input_isolation',{},budget.child(5));alignment=clock_alignment(raw_rows(self.run/'runtime/gazebo-raw.jsonl',live=True),raw_rows(self.run/'runtime/observations.jsonl',live=True))
        write_json(self.run/'runtime/dynamic-readiness.json',{'driver':s,'capture':c,'clock_alignment':alignment,'input_isolation':isolation,'gazebo_interfaces':interfaces})
    def arm(self,budget,accepted=None):
        probe=self.profile['mode']=='probe'
        if probe:
            if accepted is not None:raise ValueError('probe cannot accept goal')
            t=self.status(budget)['sim_ns'];trigger='probe_armed';uid=None
        else:
            if not accepted or accepted['kind']!='response' or accepted['data']['accepted'] is not True:raise ValueError('NO_ACCEPTED_GOAL')
            t=round(accepted['ros_s']*1e9);trigger='goal_accepted';uid=accepted['data']['uuid']
        event={'event_id':self.run.name+':motion-once','clock_epoch':self.run.name+':sim0','trigger':trigger,'trigger_ns':t,'entity_id':self.entity}
        if uid:event['uuid']=uid
        reply=self.rpc('driver','arm',event,budget);self.release=reply['release_ns']
        self.log('PROBE_ARMED' if probe else 'GOAL_ACCEPTED_ARM',{'event':event,'reply':reply})
        self.worker.rpc('mark',{'name':'PROBE_ARMED' if probe else 'DYNAMIC_ARMED','data':event},budget.child(2))
    def quiesce(self,budget):
        self.rpc('driver','quiesce',{},budget)
        drain=budget.child(1)
        while self.status(drain)['inflight']:self.worker.pump()
    def pause_check(self,budget):
        if self.profile['mode']!='probe':raise ValueError('PAUSE_CHECK_PROBE_ONLY')
        self.quiesce(budget);self.worker.rpc('probe_pause',{'enter':True},budget.child(2));self.rpc('driver','pause',{},budget)
        until=budget.child(3)
        while True:
            s=self.status(until)
            if s['paused'] and s['sim_ns']==s['stats_sim_ns']:break
            self.worker.pump()
        self.rpc('capture','snapshot',{},budget);before=self.status(budget);hold=budget.child(self.profile['dynamic_profile']['pause_wall_s'])
        while hold.remaining()>0:
            self.worker.pump()
            if self.status(budget)['sim_ns']!=before['sim_ns']:raise ValueError('PAUSED_PHASE_ADVANCED')
        self.rpc('capture','snapshot',{},budget);self.rpc('driver','unpause',{},budget)
        recover=budget.child(10)
        while self.status(recover)['paused']:self.worker.pump()
        self.rpc('driver','resume_requests',{},recover);self.worker.rpc('probe_pause',{'enter':False},recover.child(2));self.worker.rpc('sensors',{},recover)
        self.log('PAUSE_CHECK',{'before':before,'after':self.status(recover),'actual_pose':'requires raw paused snapshots; no held-last-value synthesis'})
    def probe(self,budget):
        if self.profile['mode']!='probe':raise ValueError('probe mode')
        self.worker.rpc('probe_observe',{'duration_sim_s':.5},budget.child(5))
        sequence=self.profile['dynamic_profile']['probe_sequence']
        if any(x.startswith('contact') for x in sequence):
            motion=json.loads((self.run/'artifacts/dynamic_motion.json').read_text())
            points=[motion['knots'][0]['xyz'],motion['knots'][-1]['xyz']]
            for index,point in enumerate(points):
                self.quiesce(budget);self.rpc('driver','probe_place',{'xyz':point},budget)
                until=budget.child(5)
                while math.dist(self.status(until)['actual_xyz'],point)>.02:self.worker.pump()
                for positive in [True,False]:
                    self.rpc('driver','contact',{'positive':positive,'test_xy':points[1-index][:2]},budget)
                    self.worker.rpc('probe_observe',{'duration_sim_s':2},budget.child(8));self.rpc('driver','remove_contact',{},budget)
            point=motion['knots'][0]['xyz'];self.rpc('driver','probe_place',{'xyz':point},budget)
            until=budget.child(5)
            while math.dist(self.status(until)['actual_xyz'],point)>.02:self.worker.pump()
            self.rpc('driver','resume_requests',{},budget)
        self.arm(budget);self.wait_complete(budget,probe=True)
    def wait_complete(self,budget,probe=False):
        done=set();window=self.profile['dynamic_profile']['window_s']
        if self.release is None:raise ValueError('motion not armed')
        while True:
            budget.require();s=self.status(budget)
            if s['sim_ns']>self.release+round(window*1e9):raise TimeoutError('SCENE_WINDOW_EXCEEDED')
            if probe and 'pause' in self.profile['dynamic_profile']['probe_sequence']:
                for offset in [4000000000,10000000000]:
                    if s['sim_ns']>=self.release+offset and offset not in done:self.pause_check(budget);done.add(offset)
            if s['phase']=='FINISHED' and not s['inflight']:
                point=json.loads((self.run/'artifacts/dynamic_motion.json').read_text())['knots'][-1]['xyz']
                if math.dist(s['actual_xyz'],point)>.02:raise ValueError('FINAL_ACTUAL_POSE_MISMATCH')
                break
            if probe:self.worker.rpc('probe_observe',{'duration_sim_s':.25},budget.child(5))
            else:self.worker.pump()
        self.log('MOTION_FINISHED',s)
    def close(self,budget):
        results=[];self.worker.sidecars=[x for x in self.worker.sidecars if x not in self.links.values()]
        for role in ['driver','capture']:
            item=self.items[role];item.critical=False
            try:
                if role=='driver':self.quiesce(budget)
                self.rpc(role,'exit',{},budget)
            finally:results.append(self.supervisor.close(item,budget,initial_signal=False,grace=2))
        return results
