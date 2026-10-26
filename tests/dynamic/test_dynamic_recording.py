"""Raw evidence negatives and deterministic geometry; all fixtures."""
import copy,json,math,sys
from pathlib import Path
import pytest
from tools.dynamic.dynamic_geometry import interpolate, classify, ray_circle
from tools.dynamic.inspect_dynamic_recording import clock_alignment, actual_poses, inspect, sequence_valid
def poses():return [{'epoch':'e','entity_id':1,'frame':'world','t_ns':t,'xyyaw':[t*1e-9,0,0]} for t in [0,20000000,40000000]]
@pytest.mark.parametrize('bad',['missing','epoch','entity','order','gap','future','frame','nan'])
def test_alignment_rejects(bad):
 p=poses();t=10000000
 if bad=='missing':p=[]
 if bad=='epoch':p[0]['epoch']='other'
 if bad=='entity':p[0]['entity_id']=2
 if bad=='order':p.reverse()
 if bad=='gap':p[1]['t_ns']=200000000;p=p[:2]
 if bad=='future':t=90000000
 if bad=='frame':p[1]['frame']='map'
 if bad=='nan':p[1]['xyyaw'][0]=float('nan')
 with pytest.raises(ValueError):interpolate(p,t,'e',1,50000000)
def scan_at(target,front=None):
 angles=[-.08+i*.01 for i in range(17)]
 ranges=[]
 for a in angles:
  d=ray_circle([0,0],a,target)
  f=ray_circle([0,0],a,front) if front else None
  ranges.append(min(x for x in [d,f,10.] if x is not None))
 return {'frame_id':'base_scan','angle_min':-.08,'angle_increment':.01,'range_min':.05,'range_max':20.,'ranges':ranges}
def test_visible_requires_raw_ranges():
 s=scan_at([1,0,.1]);label,ev=classify(s,[0,0,0],[1,0,.1],[],[0,0,0]);assert label=='visible' and len(ev)>=3
 s['ranges']=[None]*17;assert classify(s,[0,0,0],[1,0,.1],[],[0,0,0])[0] is None
def test_occlusion_has_foreground_returns_and_geometry():
 s=scan_at([2,0,.1],[1,0,.2]);assert classify(s,[0,0,0],[2,0,.1],[[1,0,.2]],[0,0,0])[0]=='occluded'
 s['ranges']=[float('nan')]*17;assert classify(s,[0,0,0],[2,0,.1],[[1,0,.2]],[0,0,0])[0] is None
def test_out_of_fov_or_wrong_frame_not_occluded():
 s=scan_at([1,0,.1]);assert classify(s,[0,0,0],[-1,0,.1],[],[0,0,0])[0] is None
 s['frame_id']='wrong';assert classify(s,[0,0,0],[1,0,.1],[],[0,0,0])[0] is None
def test_clock_mapping_not_assumed_or_fitted():
 raw=[{'kind':'clock','receive_monotonic_s':i*.02,'protobuf':{'sim':{'sec':'0','nsec':i*20000000}}} for i in range(3)]
 obs=[{'kind':'clock','receive_monotonic_s':i*.02,'data':{'sim_s':i*.02}} for i in range(3)]
 assert clock_alignment(raw,obs)['fit_or_time_shift_applied'] is False
 for x in obs:x['data']['sim_s']+=1
 with pytest.raises(ValueError,match='ALIGNMENT_FAILED'):clock_alignment(raw,obs)
def test_no_raw_pose_cannot_derive_from_command(tmp_path):
 r=tmp_path/'fixture';(r/'artifacts').mkdir(parents=True);(r/'runtime').mkdir()
 (r/'artifacts/profile.json').write_text(json.dumps({'mode':'navigation_recording','dynamic_spec':{'dynamic':{}}}))
 (r/'runtime/motion-commands.jsonl').write_text('{"kind":"request","desired_xyz":[0,0,0]}\n')
 report=inspect(r,scans=[]);assert not report['dynamic_recording_valid'] and report['collision'] is None
def test_entity_tree_and_changes():
 raw=[{'kind':'scene','run_id':'r','clock_epoch':'e','protobuf':{'model':[{'id':'1','name':'scripted_obstacle'},{'id':'2','name':'turtlebot3_waffle'}]}}]
 raw.append({'kind':'scene','run_id':'r','clock_epoch':'e','protobuf':{'model':[{'id':'3','name':'scripted_obstacle'}]}})
 with pytest.raises(ValueError,match='ENTITY_MISMATCH'):actual_poses(raw,'r','e')
def test_triple_sequence_needs_duration_and_continuity():
 frames=[{'label':label,'t_ns':(j*6+i)*250000000} for j,label in enumerate(['visible','occluded','visible']) for i in range(6)]
 assert sequence_valid(frames,'observed_occlusion')
 frames[7]['label']=None;assert not sequence_valid(frames,'observed_occlusion')
def test_no_scan_frame_cannot_be_visibility():
 assert not sequence_valid([],'crossing')

# Additional end-to-end synthetic sidecar tests. They are never real runs.
def fixture_record(tmp_path):
 from tools.runner.runner_core import write_json
 run=tmp_path/'fixture_dynamic';(run/'runtime').mkdir(parents=True);(run/'logs').mkdir();(run/'artifacts').mkdir()
 epoch=run.name+':sim0'
 d={'truth_gap_ns':50000000,'max_gap_ns':50000000,'position_tolerance_m':.02,'range_tolerance_m':.05}
 profile={'mode':'navigation_recording','dynamic_spec':{'kind':'crossing','dynamic':d}}
 write_json(run/'manifest.json',{'record_origin':'fixture'})
 write_json(run/'artifacts/profile.json',profile)
 motion={'schema_version':1,'frame':'world','knots':[{'t_ns':0,'xyz':[0,0,.3]},{'t_ns':1000000000,'xyz':[.1,0,.3]}],'period_ns':20000000,'max_gap_ns':50000000,'max_speed_mps':.15,'hold_ns':2000000000}
 write_json(run/'artifacts/dynamic_motion.json',motion)
 g={'laser_extrinsic':{'xyz':[0,0,.2],'rpy':[0,0,0]},'world_map_alignment':{'scope':'synthetic_known_identity'},'scan_timing_semantics':'instantaneous','robot_world_z':0.,'actor_bottom_z':0.,'actor_radius':.1,'actor_height':.6,'occluders':[]}
 write_json(run/'artifacts/dynamic_geometry.json',g)
 def stamp(t):return {'sec':str(t//1000000000),'nsec':t%1000000000}
 raw=[{'kind':'scene','run_id':run.name,'clock_epoch':epoch,'receive_monotonic_s':100.,'protobuf':{'model':[{'id':'1','name':'scripted_obstacle'},{'id':'2','name':'turtlebot3_waffle'}]}}]
 observations=[];commands=[{'kind':'arm','event':{'clock_epoch':epoch,'trigger':'goal_accepted','trigger_ns':0,'uuid':'a'*32},'release_ns':1000000000}]
 for t in range(0,5000000001,20000000):
  x=max(0,min(.1,(t-1000000000)*1e-10))
  raw.append({'kind':'clock','run_id':run.name,'clock_epoch':epoch,'receive_monotonic_s':100+t*1e-9,'protobuf':{'sim':stamp(t)}})
  raw.append({'kind':'pose','run_id':run.name,'clock_epoch':epoch,'receive_monotonic_s':100+t*1e-9,'protobuf':{'header':{'stamp':stamp(t)},'pose':[{'name':'scripted_obstacle','id':'1','position':{'x':x,'z':.3},'orientation':{'w':1.}},{'name':'turtlebot3_waffle','id':'2','position':{'x':-1.},'orientation':{'w':1.}}]}})
  observations.append({'kind':'clock','receive_monotonic_s':100+t*1e-9,'data':{'sim_s':t*1e-9}})
  if 1000000000<=t<=4000000000:
   commands += [{'kind':'request','sim_ns':t,'entity_id':1,'desired_xyz':[x,0,.3]},{'kind':'response','response':{'ok':True}}]
 actions=[{'kind':'response','ros_s':0.,'data':{'accepted':True,'uuid':'a'*32}}]
 for name,rows in [('runtime/gazebo-raw.jsonl',raw),('runtime/observations.jsonl',observations),('runtime/motion-commands.jsonl',commands),('logs/action-client.jsonl',actions)]:
  (run/name).write_text(''.join(json.dumps(row)+'\n' for row in rows))
 scans=[]
 for t in range(1000000000,4200000000,200000000):
  x=max(0,min(.1,(t-1000000000)*1e-10));scan=scan_at([1+x,0,.1]);scan.update({'t_ns':t,'time_increment':0.,'scan_time':.2,'bag_message_index':len(scans)+1});scans.append(scan)
 return run,scans
def test_full_fixture_actual_data_motion_gate(tmp_path):
 run,scans=fixture_record(tmp_path)
 # The frozen observer remains stationary: this proves data/time/visibility,
 # but cannot establish the navigation crossing geometry.
 report=inspect(run,scans)
 assert report['record_origin']=='fixture' and report['motion_valid'] is True and report['time_valid'] is True and report['observation_valid'] is True
 assert report['dynamic_recording_valid'] is False and 'CROSSING_GEOMETRY_NOT_ESTABLISHED' in report['errors']
@pytest.mark.parametrize('case',['actual_wrong','missing_scan','truth_gap','entity_change','epoch_change','source_clock_offset','missing_foreground','scan_stamp_outside','per_beam_gap','commands_only'])
def test_raw_end_to_end_negatives(tmp_path,case):
 run,scans=fixture_record(tmp_path);rawpath=run/'runtime/gazebo-raw.jsonl';raw=[json.loads(x) for x in rawpath.read_text().splitlines()]
 if case=='actual_wrong':
  for row in raw:
   if row['kind']=='pose':row['protobuf']['pose'][0]['position']['x']+=1
 if case=='truth_gap':raw=[x for x in raw if x['kind']!='pose' or not 1500000000<int(x['protobuf']['header']['stamp']['sec'])*1000000000+x['protobuf']['header']['stamp']['nsec']<1800000000]
 if case=='entity_change':
  next(x for x in raw if x['kind']=='pose')['protobuf']['pose'][0]['id']='99'
 if case=='epoch_change':raw[-1]['clock_epoch']='different'
 if case=='source_clock_offset':
  for row in raw:
   if row['kind']=='clock':row['protobuf']['sim']['sec']=str(int(row['protobuf']['sim']['sec'])+2)
 if case=='commands_only':raw=[x for x in raw if x['kind']!='pose']
 rawpath.write_text(''.join(json.dumps(x)+'\n' for x in raw))
 if case=='missing_scan':scans=[]
 if case=='missing_foreground':
  for scan in scans:scan['ranges']=[None]*len(scan['ranges'])
 if case=='scan_stamp_outside':
  for scan in scans:scan['t_ns']+=100000000000
 if case=='per_beam_gap':
  for scan in scans:scan['time_increment']=1.
 report=inspect(run,scans)
 assert not report['dynamic_recording_valid'] and report['collision'] is None
 if case=='actual_wrong':assert 'ACTUAL_TRAJECTORY_ERROR' in report['errors']
 if case=='source_clock_offset':assert 'CLOCK_ALIGNMENT_FAILED' in report['errors']

def test_actual_crossing_is_not_parallel_overlap():
 from tools.dynamic.dynamic_geometry import polyline_crossing
 def row(x,y,t):return {'xyz':[x,y,0],'t_ns':t}
 horizontal=[row(-1,0,0),row(1,0,1)]
 assert polyline_crossing([row(0,-1,0),row(0,1,1)],horizontal)
 assert polyline_crossing(horizontal,horizontal) is None
 assert polyline_crossing([row(0,2,0),row(0,3,1)],horizontal) is None
def test_full_synthetic_crossing_can_pass_without_collision_claim(tmp_path):
 run,scans=fixture_record(tmp_path)
 rawpath=run/'runtime/gazebo-raw.jsonl';rows=[json.loads(x) for x in rawpath.read_text().splitlines()]
 for row in rows:
  if row['kind']!='pose':continue
  t=int(row['protobuf']['header']['stamp']['sec'])*1000000000+row['protobuf']['header']['stamp']['nsec']
  r=row['protobuf']['pose'][1];r['position']={'x':.05,'y':-1+max(0,(t-3000000000)*1e-9)}
  r['orientation']={'z':math.sin(math.pi/4),'w':math.cos(math.pi/4)}
 rawpath.write_text(''.join(json.dumps(x)+'\n' for x in rows))
 selected=[]
 for scan in scans:
  x=max(0,min(.1,(scan['t_ns']-1000000000)*1e-10));robot_y=-1+max(0,(scan['t_ns']-3000000000)*1e-9);scan['ranges']=[ray_circle([.05,robot_y],math.pi/2+scan['angle_min']+i*scan['angle_increment'],[x,0,.1]) or 10. for i in range(len(scan['ranges']))];selected.append(scan)
 report=inspect(run,selected)
 assert report['dynamic_recording_valid'] and report['record_origin']=='fixture' and report['collision'] is None,report['errors']
def test_injected_scan_cannot_certify_real_data(tmp_path):
 run,scans=fixture_record(tmp_path);(run/'manifest.json').write_text(json.dumps({'record_origin':'real'}))
 report=inspect(run,scans);assert not report['dynamic_recording_valid'] and 'INJECTED_SCAN_FOR_REAL_FORBIDDEN' in report['errors']

def test_absent_probe_facilities_remain_unknown():
 from tools.dynamic.inspect_dynamic_recording import probe_facilities
 flags,evidence=probe_facilities([],[],[],[])
 assert all(value is None for value in flags.values())
def test_nonplanar_truth_is_not_silently_flattened():
 from tools.dynamic.inspect_dynamic_recording import pose_value
 with pytest.raises(ValueError,match='NONPLANAR'):pose_value({'position':{},'orientation':{'x':.01,'w':math.sqrt(1-.01**2)}})

def test_partial_or_missing_sequence_is_not_filled(tmp_path):
 from tools.dynamic.inspect_dynamic_recording import raw_rows
 p=tmp_path/'fixture.jsonl';p.write_text('{"seq":1}\n{"seq":')
 with pytest.raises(ValueError,match='PARTIAL'):raw_rows(p)
 assert len(raw_rows(p,live=True))==1
 p.write_text('{"seq":1}\n{"seq":3}\n')
 with pytest.raises(ValueError,match='SEQUENCE_GAP'):raw_rows(p)

def test_bad_real_manifest_cannot_be_certified(tmp_path):
 run,scans=fixture_record(tmp_path);(run/'manifest.json').write_text(json.dumps({'record_origin':'real'}))
 report=inspect(run)
 assert not report['dynamic_recording_valid'] and 'RECORD_CONTRACT_INVALID' in report['errors']

def test_crossing_witness_preserves_time_order_and_rejects_long_gaps():
 from tools.dynamic.dynamic_geometry import polyline_crossing
 robot=[{'xyz':[0,-1,0],'t_ns':0},{'xyz':[0,1,0],'t_ns':10}]
 actor=[{'xyz':[-1,0,0],'t_ns':30},{'xyz':[1,0,0],'t_ns':40}]
 witness=polyline_crossing(robot,actor)
 assert witness['ahead_of_robot'] is False and witness['actor_intersection_estimated_ns']>witness['robot_intersection_estimated_ns']
 actor[1]['t_ns']=1000000000
 assert polyline_crossing(robot,actor) is None

def test_cached_scene_replies_are_not_actual_paused_pose():
 from tools.dynamic.inspect_dynamic_recording import probe_facilities
 rows=[]
 for i,(mono,sec) in enumerate([(100.,1),(103.,1),(110.,2),(113.,2)]):
  rows.append({'kind':'scene_snapshot','_line':i+1,'receive_monotonic_s':mono,'protobuf':{'header':{'stamp':{'sec':str(sec),'nsec':0}},'model':[{'name':'scripted_obstacle','pose':{'orientation':{'w':1}}}]}})
 rows += [{'kind':'stats','receive_monotonic_s':t,'protobuf':{'paused':True,'sim_time':{'sec':str(sec)}}} for t,sec in [(100.,1),(110.,2)]]
 flags,_=probe_facilities(rows,[],[],[])
 assert flags['paused_actual_snapshots'] is None
def test_command_trace_mismatch_is_preserved_even_if_actual_pose_is_good(tmp_path):
 run,scans=fixture_record(tmp_path);p=run/'runtime/motion-commands.jsonl';commands=[json.loads(x) for x in p.read_text().splitlines()]
 next(x for x in commands if x['kind']=='request')['desired_xyz'][0]+=.1
 p.write_text(''.join(json.dumps(x)+'\n' for x in commands))
 report=inspect(run,scans)
 assert not report['dynamic_recording_valid'] and 'COMMAND_TRAJECTORY_MISMATCH' in report['errors']

@pytest.mark.parametrize('missing',['position','orientation'])
def test_missing_pose_message_is_not_default_zero(missing):
 from tools.dynamic.inspect_dynamic_recording import pose_value
 p={'position':{},'orientation':{'w':1}};p.pop(missing)
 with pytest.raises(ValueError,match='FIELDS_MISSING'):pose_value(p)


def test_full_3d_ray_preserves_pitch_and_height():
 from tools.dynamic.dynamic_geometry import rpy_quat, rotate, ray_cylinder_3d
 q=rpy_quat([0.,.1,0.]);d=rotate(q,[1.,0.,0.])
 assert d[2]<0 and math.isclose(sum(x*x for x in d),1.)
 hit=ray_cylinder_3d([0,0,.3],d,[1,0,.1],0.,.6)
 assert math.isclose(hit,.9/math.cos(.1))
 assert ray_cylinder_3d([0,0,1.],d,[1,0,.1],0.,.6) is None

def test_full_quaternion_interpolation_no_extrapolation():
 from tools.dynamic.dynamic_geometry import PoseSeries, rpy_quat, rotate
 seq=[{'epoch':'e','entity_id':1,'frame':'world','t_ns':t,'xyz':[0,0,0],'xyyaw':[0,0,0],'quaternion_xyzw':rpy_quat([0,p,0])} for t,p in [(0,0.),(20000000,.1)]]
 series=PoseSeries(seq,'e',1,50000000)
 assert math.isclose(rotate(series.quat_at(10000000),[1,0,0])[2],-math.sin(.05))
 with pytest.raises(ValueError,match='extrapolation'):series.quat_at(30000000)

def test_fresh_paused_source_publications_not_cached_scene():
 from tools.dynamic.inspect_dynamic_recording import probe_facilities
 rows=[]
 for start,sec in [(100.,1),(110.,2)]:
  rows += [{'kind':'scene_snapshot','_line':len(rows)+1,'receive_monotonic_s':t,'protobuf':{}} for t in [start,start+3]]
  for i in range(31):
   t=start+i*.1
   rows.append({'kind':'stats','receive_monotonic_s':t,'protobuf':{'paused':True,'sim_time':{'sec':sec}}})
   rows.append({'kind':'pose','receive_monotonic_s':t,'protobuf':{'header':{'stamp':{'sec':sec}},'pose':[{'name':'scripted_obstacle','position':{'x':.1},'orientation':{'w':1}}]}})
 flags,_=probe_facilities(rows,[],[],[])
 assert flags['paused_actual_snapshots'] is True
 rows=[r for r in rows if r['kind']!='pose']
 assert probe_facilities(rows,[],[],[])[0]['paused_actual_snapshots'] is None

def test_missing_motion_window_scans_cannot_hide_in_shutdown(tmp_path):
 run,scans=fixture_record(tmp_path)
 report=inspect(run,[scan for scan in scans if scan['t_ns']<2500000000])
 assert 'SCAN_WINDOW_COVERAGE' in report['errors']

@pytest.mark.parametrize('case',['bounded_robot','unpaused','large_robot','actor_change','cumulative_drift'])
def test_paused_duplicate_bound_is_not_a_missing_data_fill(tmp_path,case):
 run,_=fixture_record(tmp_path)
 from tools.dynamic.inspect_dynamic_recording import raw_rows
 path=run/'runtime/gazebo-raw.jsonl';rows=raw_rows(path)
 index=next(i for i,row in enumerate(rows) if row['kind']=='pose' and row['protobuf']['header']['stamp']=={'sec':'1','nsec':200000000})
 original=rows[index];copies=[]
 for n in range(1,4 if case=='cumulative_drift' else 2):
  row=copy.deepcopy(original);which=0 if case=='actor_change' else 1
  row['protobuf']['pose'][which]['position']['x']+=n*(.6e-6 if case=='cumulative_drift' else 1e-4 if case=='large_robot' else 1e-7)
  copies.append(row)
 rows[index+1:index+1]=copies
 if case!='unpaused':rows.append({'kind':'stats','run_id':run.name,'clock_epoch':run.name+':sim0','protobuf':{'paused':True,'sim_time':{'sec':'1','nsec':200000000}}})
 for i,row in enumerate(rows,1):row['_line']=i
 if case=='bounded_robot':
  _,samples=actual_poses(rows,run.name,run.name+':sim0')
  item=next(s for s in samples['turtlebot3_waffle'] if s['t_ns']==1200000000)
  assert item['duplicate_position_spread_m']<=1e-6 and len(item['merged_raw_lines'])==2
 else:
  with pytest.raises(ValueError,match='CONFLICTING_DUPLICATE'):actual_poses(rows,run.name,run.name+':sim0')

def test_partial_transition_is_retained_but_not_full_occlusion():
 labels=['visible','partial','occluded','partial','visible']
 frames=[{'label':label,'t_ns':(j*8+i)*200000000} for j,label in enumerate(labels) for i in range(8)]
 assert sequence_valid(frames,'observed_occlusion')
 assert not sequence_valid([dict(x,label='partial' if x['label']=='occluded' else x['label']) for x in frames],'observed_occlusion')
 assert not sequence_valid([dict(x,label=None if x['label']=='partial' else x['label']) for x in frames],'observed_occlusion')

def test_clock_equal_source_join_survives_receive_queue_jitter():
 raw=[{'kind':'clock','receive_monotonic_s':i*.003,'protobuf':{'sim':{'sec':'0','nsec':i*3000000}}} for i in range(100)]
 # The source clocks agree exactly; the ROS callback is queued 27ms.
 obs=[{'kind':'clock','receive_monotonic_s':i*.003+(.027 if i==30 else 0.),'data':{'sim_s':i*.003}} for i in [10,20,30,40]]
 r=clock_alignment(raw,obs)
 assert r['exact_source_matches']==4 and r['max_difference_ns']==0 and not r['fit_or_time_shift_applied']
 for x in obs:x['receive_monotonic_s']+=.5
 with pytest.raises(ValueError,match='ALIGNMENT_FAILED'):clock_alignment(raw,obs)

def test_clock_source_join_never_repairs_clock_offset():
 raw=[{'kind':'clock','receive_monotonic_s':i*.003,'protobuf':{'sim':{'sec':'0','nsec':i*3000000}}} for i in range(100)]
 obs=[{'kind':'clock','receive_monotonic_s':i*.003,'data':{'sim_s':i*.003+1.}} for i in [10,20,30,40]]
 with pytest.raises(ValueError,match='ALIGNMENT_FAILED'):clock_alignment(raw,obs)

@pytest.mark.parametrize('offset',[-.06,.06])
def test_equal_source_join_cannot_hide_sustained_small_clock_offset(offset):
 raw=[{'kind':'clock','receive_monotonic_s':i*.003,'protobuf':{'sim':{'sec':'0','nsec':i*3000000}}} for i in range(200)]
 obs=[{'kind':'clock','receive_monotonic_s':i*.003,'data':{'sim_s':i*.003+offset}} for i in range(30,170)]
 with pytest.raises(ValueError,match='ALIGNMENT_FAILED'):clock_alignment(raw,obs)
