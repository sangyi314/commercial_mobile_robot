"""Offline dynamic evidence review. Never opens a ROS node or replays a bag."""
from pathlib import Path
import argparse,json,math,statistics
import validate_run as v
from runner_core import write_json,digest,within
from dynamic_geometry import interpolate,ray_circle,polyline_crossing,PoseSeries,rotate,rpy_quat,ray_cylinder_3d
from dynamic_contract import motion_rules

def ns(t):return int(t.get('sec',0))*1000000000+int(t.get('nsec',0))
def raw_rows(path,live=False):
    rows=[];text=Path(path).read_text()
    if text and not text.endswith('\n'):
        if not live:raise ValueError('SIDECAR_PARTIAL_LINE')
        text=text[:text.rfind('\n')+1]
    for i,line in enumerate(text.splitlines(),1):
        x=json.loads(line,object_pairs_hook=v.unique_json_object,parse_constant=v.reject_constant)
        v.check_json_tree(x,str(path));x['_line']=i;rows.append(x)
    if any('seq' in x for x in rows):
        sequence=[x.get('seq') for x in rows]
        if any(type(x) is not int for x in sequence) or sequence!=list(range(1,len(rows)+1)):raise ValueError('SIDECAR_SEQUENCE_GAP')
    return rows
def desired(m,t):
    if t<=0:return m['knots'][0]['xyz']
    for a,b in zip(m['knots'],m['knots'][1:]):
        if t<=b['t_ns']:
            u=(t-a['t_ns'])/(b['t_ns']-a['t_ns']);return [a['xyz'][j]+u*(b['xyz'][j]-a['xyz'][j]) for j in range(3)]
    return m['knots'][-1]['xyz']
def pose_value(p,allow3d=False):
    if not isinstance(p.get('position'),dict) or not isinstance(p.get('orientation'),dict):raise ValueError('TRUTH_POSE_FIELDS_MISSING')
    q=p['orientation'];x,y,z,w=[q.get(k,0.) for k in ['x','y','z','w']]
    if not allow3d and (abs(x)>1e-6 or abs(y)>1e-6):raise ValueError('NONPLANAR_TRUTH_UNSUPPORTED')
    if abs(x*x+y*y+z*z+w*w-1)>.001:raise ValueError('invalid pose quaternion')
    point=p['position']
    xyz=[point.get(k,0.) for k in ['x','y','z']]
    if not all(math.isfinite(a) for a in xyz):raise ValueError('nonfinite truth')
    return xyz,math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
def clock_alignment(raw,observations):
    gz=[(x['receive_monotonic_s'],ns(x['protobuf']['sim'])) for x in raw if x['kind']=='clock']
    ros=[x for x in observations if x['kind']=='clock']
    if len(gz)<3 or len(ros)<3:raise ValueError('CLOCK_ALIGNMENT_MISSING')
    if any(b[1]<a[1] or b[0]<a[0] for a,b in zip(gz,gz[1:])):raise ValueError('CLOCK_EPOCH_CHANGED')
    # /clock is the bridge of the recorded GZ clock. Match equal source stamps
    # first; callback receive times are transport observations, not source time.
    # 100ms is a bounded association window measured from real bridge delivery.
    # The independent 25ms source discrepancy bound remains unchanged.
    by_stamp={}
    for g in gz:by_stamp.setdefault(g[1],[]).append(g)
    residuals=[];delays=[];exact_matches=0;receipt_windows={};receipt_residuals=[]
    source_ros=[round(x['data']['sim_s']*1e9) for x in ros]
    if any(b<a for a,b in zip(source_ros,source_ros[1:])):raise ValueError('CLOCK_EPOCH_CHANGED')
    for x,s in zip(ros,source_ros):
        receive=x['receive_monotonic_s']
        nearby=[g for g in gz if abs(g[0]-receive)<=.025]
        if nearby:
            anchor=min(nearby,key=lambda g:abs(g[0]-receive))
            discrepancy=abs(anchor[1]-s);receipt_residuals.append(discrepancy)
            window=int(receive-ros[0]['receive_monotonic_s'])
            receipt_windows.setdefault(window,[]).append(discrepancy)
        exact=[g for g in by_stamp.get(s,[]) if abs(g[0]-receive)<=.1]
        if exact:
            g=min(exact,key=lambda g:abs(g[0]-receive));exact_matches+=1
        else:
            candidates=[g for g in gz if abs(g[0]-receive)<=.025]
            if not candidates:continue
            g=min(candidates,key=lambda g:abs(g[0]-receive))
        residuals.append(abs(g[1]-s));delays.append(receive-g[0])
    # Equal stamps alone could hide a constant clock offset within transport
    # latency. Keep independent, unshifted receive-proximity anchors in every
    # one-wall-second window; sustained offsets or sustained backlog fail.
    anchor_medians=[statistics.median(a) for a in receipt_windows.values()]
    if len(residuals)<3 or not anchor_medians or max(anchor_medians)>25000000 or max(residuals)>25000000:raise ValueError('CLOCK_ALIGNMENT_FAILED')
    return {'samples':len(residuals),'max_difference_ns':max(residuals),'fit_or_time_shift_applied':False,'method':'bounded_equal_source_stamp_join_then_nearest_receive_fallback','exact_source_matches':exact_matches,'source_discrepancy_limit_ns':25000000,'equal_stamp_receive_association_limit_s':.1,'observed_receive_delta_s':[min(delays),max(delays)],'unshifted_receipt_anchor_window_s':1.,'max_window_median_receipt_discrepancy_ns':max(anchor_medians),'max_raw_nearest_receipt_discrepancy_ns':max(receipt_residuals)}

def actual_poses(raw,run_id,epoch):
    mapping={};samples={}
    for x in raw:
        if x.get('run_id')!=run_id or x.get('clock_epoch')!=epoch:raise ValueError('ENTITY_EPOCH')
        if x['kind'] in ('scene','scene_snapshot'):
            for p in x['protobuf'].get('model',[]):
                if p.get('name') in ('scripted_obstacle','turtlebot3_waffle'):
                    name=p['name'];identity=int(p['id'])
                    if name in mapping and mapping[name]!=identity:raise ValueError('ENTITY_MISMATCH')
                    mapping[name]=identity
    if set(mapping)!= {'scripted_obstacle','turtlebot3_waffle'}:raise ValueError('ENTITY_TREE_MISSING')
    for x in raw:
        if x['kind']!='pose':continue
        p=x['protobuf'];t=ns(p['header']['stamp'])
        for pose in p.get('pose',[]):
            name=pose.get('name')
            if name not in mapping:continue
            if int(pose['id'])!=mapping[name]:raise ValueError('ENTITY_MISMATCH')
            xyz,angle=pose_value(pose,allow3d=True);samples.setdefault(name,[]).append({'epoch':epoch,'entity_id':mapping[name],'frame':'world','t_ns':t,'xyz':xyz,'xyyaw':[*xyz[:2],angle],'quaternion_xyzw':[pose['orientation'].get(k,0.) for k in ['x','y','z','w']],'raw_line':x['_line']})
    paused_stamps={ns(x['protobuf'].get('sim_time',{})) for x in raw if x['kind']=='stats' and x['protobuf'].get('paused') is True}
    for name,seq in list(samples.items()):
        unique=[]
        for item in seq:
            if unique and item['t_ns']==unique[-1]['t_ns']:
                previous=unique[-1];anchor_xyz=previous.get('duplicate_anchor_xyz',previous['xyz']);anchor_q=previous.get('duplicate_anchor_q',previous['quaternion_xyzw']);distance=math.dist(item['xyz'],anchor_xyz)
                dot=abs(sum(x*y for x,y in zip(item['quaternion_xyzw'],anchor_q)));angle=2*math.acos(min(1.,dot))
                # Real paused ECM boundary: robot settles 0.095 micrometre on the same sim stamp.
                # Preserve raw sources and variation; actor must remain exactly unchanged.
                consistent=(distance<=1e-6 and angle<=1e-5) if name=='turtlebot3_waffle' else (distance==0. and angle<=1e-12)
                if item['t_ns'] not in paused_stamps or not consistent:raise ValueError('TRUTH_CONFLICTING_DUPLICATE')
                item['duplicate_anchor_xyz']=anchor_xyz;item['duplicate_anchor_q']=anchor_q
                item['merged_raw_lines']=previous.get('merged_raw_lines',[previous['raw_line']])+[item['raw_line']]
                item['duplicate_position_spread_m']=max(previous.get('duplicate_position_spread_m',0.),distance)
                item['duplicate_angle_spread_rad']=max(previous.get('duplicate_angle_spread_rad',0.),angle)
                unique[-1]=item
                continue
            if unique and item['t_ns']<unique[-1]['t_ns']:raise ValueError('TRUTH_ORDER_OR_MISSING')
            unique.append(item)
        if len(unique)<3:raise ValueError('TRUTH_ORDER_OR_MISSING')
        samples[name]=unique
    if set(samples)!=set(mapping):raise ValueError('TRUTH_MISSING')
    return mapping,samples
def aligned_frame(scan,samples,mapping,epoch,g,tol,gap):
    ex=g['laser_extrinsic']
    if ex is None or g.get('world_map_alignment') is None or g.get('scan_timing_semantics') not in ('instantaneous','per_beam'):raise ValueError('FRAME_ALIGNMENT_UNVERIFIED')
    evidence=[];unknown=False
    for i,r in enumerate(scan['ranges']):
        t=scan['t_ns']+round(i*scan['time_increment']*1e9)
        robot=samples['turtlebot3_waffle'];actor=samples['scripted_obstacle']
        xyz=robot.xyz_at(t);q=robot.quat_at(t);ob=actor.xyz_at(t);aq=actor.quat_at(t)
        if abs(aq[0])+abs(aq[1])>1e-6:raise ValueError('NONVERTICAL_CYLINDER_UNSUPPORTED')
        offset=rotate(q,ex['xyz']);origin=[xyz[j]+offset[j] for j in range(3)]
        angle=scan['angle_min']+i*scan['angle_increment']
        direction=rotate(q,rotate(rpy_quat(ex['rpy']),[math.cos(angle),math.sin(angle),0.]))
        target=ray_cylinder_3d(origin,direction,[*ob[:2],g['actor_radius']],ob[2]-g['actor_height']/2,g['actor_height'])
        if target is None or not scan['range_min']<=target<=scan['range_max']:continue
        foreground=[distance for c in g['occluders'] if (distance:=ray_cylinder_3d(origin,direction,c,g.get('occluder_bottom_z',0.),g.get('occluder_height_m',.5))) is not None and distance<target-tol]
        expected=min(foreground) if foreground else target
        good=r is not None and math.isfinite(r) and scan['range_min']<=r<=scan['range_max'] and abs(r-expected)<=tol
        if not good:unknown=True
        evidence.append({'beam':i,'source_ns':t,'range':r if r is not None and math.isfinite(r) else None,'target_surface':target,'foreground':min(foreground) if foreground else None,'match':good})
    if scan['frame_id']!='base_scan' or unknown or len(evidence)<3:return None,evidence
    blocked=[x['foreground'] is not None for x in evidence]
    return ('occluded' if all(blocked) else 'partial' if any(blocked) else 'visible'),evidence

def sequence_valid(frames,kind):
    stages=[]
    for item in frames:
        label=item['label']
        if not stages or stages[-1]['label']!=label:stages.append({'label':label,'frames':[item]})
        else:stages[-1]['frames'].append(item)
    valid=[]
    for s in stages:
        if s['label']=='partial':continue # Keep partial frames in report; never count them as fully occluded.
        fs=s['frames'];dt=fs[-1]['t_ns']-fs[0]['t_ns']
        gaps=[b['t_ns']-a['t_ns'] for a,b in zip(fs,fs[1:])]
        if len(fs)>=5 and dt>=1000000000 and gaps and 0<min(gaps)<=max(gaps)<=300000000:valid.append(s['label'])
    if kind=='observed_occlusion':
        return any(valid[i:i+3]==['visible','occluded','visible'] for i in range(len(valid)-2))
    return 'visible' in valid

def probe_facilities(raw,commands,events,observations):
    result={'contacts_two_positions':None,'negative_old_location':None,'paused_actual_snapshots':None,'cold_initial_state':None,'robot_stationary':None,'visual_geometry':None}
    creates=[x for x in commands if x['kind']=='contact_create']
    windows=[]
    for i,c in enumerate(creates):
        end=creates[i+1]['sim_ns'] if i+1<len(creates) else max([x.get('sim_ns',0) for x in commands],default=0)
        messages=[r for r in raw if r['kind']=='contacts' and c['sim_ns']<=ns(r['protobuf'].get('header',{}).get('stamp',{}))<end]
        pairs=[]
        for row in messages:
            for contact in row['protobuf'].get('contact',[]):
                a,b=contact.get('collision1',{}),contact.get('collision2',{})
                if not a.get('id') or not b.get('id') or a.get('id')==b.get('id'):continue
                names=[a.get('name',''),b.get('name','')]
                if any('cmr_probe_ball::body::ball' in x for x in names):pairs.append(names)
        target_hits=[x for x in pairs if any('scripted_obstacle::body::cylinder_collision' in n for n in x)]
        ground_hits=[x for x in pairs if any('ground_plane' in n for n in x)]
        ball=[(ns(r['protobuf']['header']['stamp']),pose) for r in raw if r['kind']=='pose' and c['sim_ns']<=ns(r['protobuf']['header']['stamp'])<end for pose in r['protobuf'].get('pose',[]) if pose.get('name')=='cmr_probe_ball' and isinstance(pose.get('position'),dict)]
        zs=[pose['position'].get('z',0.) for _,pose in ball]
        response=None
        if len(zs)>=5 and all(math.isfinite(z) for z in zs):
            response=(zs[0]>.9 and zs[0]-zs[-1]>.3 and ((.60<zs[-1]<.66) if c['positive'] else (0.<zs[-1]<.06)))
        windows.append({'positive':c['positive'],'actual_actor_xyz':c['actual_actor_xyz'],'target_hits':len(target_hits),'ground_hits':len(ground_hits),'raw_messages':len(messages),'ball_actual_samples':len(zs),'ball_first_last_z':([zs[0],zs[-1]] if zs else None),'ball_entity_ids':sorted({int(pose['id']) for _,pose in ball}),'actual_motion_response':response})

    positives=[x for x in windows if x['positive']];negatives=[x for x in windows if not x['positive']]
    if len(positives)==2 and all(x['target_hits']>0 and x['actual_motion_response'] is True and len(x['ball_entity_ids'])==1 for x in positives):
        result['contacts_two_positions']=math.dist(positives[0]['actual_actor_xyz'],positives[1]['actual_actor_xyz'])>.3
    if len(negatives)==2 and all(x['ground_hits']>0 and x['target_hits']==0 and x['raw_messages']>=5 and x['actual_motion_response'] is True and len(x['ball_entity_ids'])==1 for x in negatives):result['negative_old_location']=True
    snapshots=[x for x in raw if x['kind']=='scene_snapshot']
    pairs=[]
    for a,b in zip(snapshots,snapshots[1:]):
        if b['receive_monotonic_s']-a['receive_monotonic_s']<2.5:continue
        try:
            paused_stats=[r for r in raw if r['kind']=='stats' and r['protobuf'].get('paused') is True and a['receive_monotonic_s']<=r['receive_monotonic_s']<=b['receive_monotonic_s']]
            stamps={ns(r['protobuf'].get('sim_time',{})) for r in paused_stats}
            if len(stamps)!=1:continue
            stamp=next(iter(stamps));observed=[]
            for row in raw:
                if row['kind']=='pose' and a['receive_monotonic_s']<=row['receive_monotonic_s']<=b['receive_monotonic_s'] and ns(row['protobuf']['header']['stamp'])==stamp:
                    observed += [(row['receive_monotonic_s'],pose_value(pose)) for pose in row['protobuf'].get('pose',[]) if pose.get('name')=='scripted_obstacle']
            if len(observed)>=5 and observed[-1][0]-observed[0][0]>=2.5 and all(pose==observed[0][1] for _,pose in observed):pairs.append((a['_line'],b['_line']))
        except (KeyError,StopIteration,ValueError,TypeError):pass
    if len(pairs)>=2:result['paused_actual_snapshots']=True
    first=[x for x in events if x.get('type')=='WORKER_COMMAND' and x.get('data',{}).get('data',{}).get('op')=='send']
    odom=[x for x in observations if x['kind']=='odom']
    if first:result['robot_stationary']=False
    elif odom:
        p=odom[0]['data']['position'];valid=all(all(isinstance(v,(float,int)) and math.isfinite(v) for key in ['position','linear','angular'] for v in x['data'][key]) and math.dist(p[:2],x['data']['position'][:2])<=.02 and math.hypot(*x['data']['linear'][:2])<=.01 and abs(x['data']['angular'][2])<=.03 for x in odom)
        result['robot_stationary']=valid
    visuals=[]
    for row in raw:
        if row['kind'] not in ('scene','scene_snapshot'):continue
        for m in row['protobuf'].get('model',[]):
            if m.get('name')!='scripted_obstacle':continue
            for link in m.get('link',[]):
                for visual in link.get('visual',[]):
                    c=visual.get('geometry',{}).get('cylinder')
                    if c:visuals.append(c)
    if visuals:result['visual_geometry']=all(abs(c.get('radius',-1)-.1)<1e-6 and abs(c.get('length',-1)-.6)<1e-6 for c in visuals)
    return result,{'contact_windows':windows,'paused_raw_line_pairs':pairs}

def read_scans(run):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import LaserScan
    metadata=v.load_strict(run/'bag/metadata.yaml')['rosbag2_bagfile_information']
    for name in metadata['relative_file_paths']:
        p=within(run/'bag',run/'bag'/name)
        if not p.is_file() or not p.stat().st_size:raise ValueError('BAG_SHARD_MISSING')
    reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(run/'bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    scans=[];index=0
    while reader.has_next():
        topic,data,bag_ns=reader.read_next();index+=1
        if topic!='/scan':continue
        m=deserialize_message(data,LaserScan)
        scans.append({'frame_id':m.header.frame_id,'t_ns':m.header.stamp.sec*1000000000+m.header.stamp.nanosec,'bag_receive_ros_ns':bag_ns,'bag_message_index':index,'ranges':list(m.ranges),'angle_min':m.angle_min,'angle_increment':m.angle_increment,'range_min':m.range_min,'range_max':m.range_max,'scan_time':m.scan_time,'time_increment':m.time_increment})
    if not scans or any(b['t_ns']<=a['t_ns'] for a,b in zip(scans,scans[1:])):raise ValueError('SCAN_MISSING_OR_UNORDERED')
    return scans

def inspect(run,scans=None):
    run=Path(run);report={'schema_version':1,'run_id':run.name,'purpose':None,'dynamic_recording_valid':False,'probe_pass':False,'motion_valid':None,'time_valid':None,'observation_valid':None,'collision':None,'errors':[],'frames':[],'source_files':[]}
    try:
        manifest=v.load_strict(run/'manifest.json');report['record_origin']=manifest['record_origin']
        if report['record_origin'] not in ('real','fixture'):raise ValueError('RECORD_ORIGIN')
        if scans is not None and report['record_origin']!='fixture':raise ValueError('INJECTED_SCAN_FOR_REAL_FORBIDDEN')
        if report['record_origin']=='real':
            contract=v.validate_run_dir(run)
            if contract['status'] not in ('VALID','INCOMPLETE') or (contract['status']=='INCOMPLETE' and [x['code'] for x in contract['errors']]!=['MISSING_RESULT']):raise ValueError('RECORD_CONTRACT_INVALID')

        p=v.load_strict(run/'artifacts/profile.json');spec=p['dynamic_spec'];d=spec['dynamic'];report['purpose']=p['mode'];epoch=run.name+':sim0'
        raw=raw_rows(run/'runtime/gazebo-raw.jsonl');obs=raw_rows(run/'runtime/observations.jsonl');cmd=raw_rows(run/'runtime/motion-commands.jsonl')
        mapping,samples=actual_poses(raw,run.name,epoch);report['entity_mapping']=mapping
        report['clock_alignment']=clock_alignment(raw,obs);report['time_valid']=True
        arms=[x for x in cmd if x['kind']=='arm']
        if len(arms)!=1 or arms[0]['event']['clock_epoch']!=epoch:raise ValueError('TRIGGER_BINDING')
        arm=arms[0];trigger=arm['event']['trigger']
        actions=raw_rows(run/'logs/action-client.jsonl')
        if p['mode']=='probe':
            if trigger!='probe_armed' or actions:raise ValueError('PROBE_SENT_GOAL')
        else:
            accepted=[x for x in actions if x['kind']=='response' and x['data']['accepted'] is True]
            if trigger!='goal_accepted' or len(accepted)!=1 or accepted[0]['data']['uuid']!=arm['event']['uuid'] or abs(round(accepted[0]['ros_s']*1e9)-arm['event']['trigger_ns'])>1000:raise ValueError('GOAL_ACCEPTANCE_BINDING')
        motion=motion_rules(v.load_strict(run/'artifacts/dynamic_motion.json'));release=arm['release_ns'];errors=[]
        seq=samples['scripted_obstacle'];motion_samples=[s for s in seq if release<=s['t_ns']<=release+motion['knots'][-1]['t_ns']+motion['hold_ns']]
        if not motion_samples or motion_samples[-1]['t_ns']<release+motion['knots'][-1]['t_ns']+1000000000:raise ValueError('MOTION_WINDOW_MISSING')
        gaps=[b['t_ns']-a['t_ns'] for a,b in zip(seq,seq[1:])]
        if not gaps or max(gaps)>d['truth_gap_ns']:raise ValueError('TRUTH_GAP')
        requests=[x for x in cmd if x['kind']=='request'];replies=[x for x in cmd if x['kind']=='response']
        if not requests or len(requests)!=len(replies) or any(not x['response']['ok'] for x in replies):raise ValueError('MOTION_SERVICE_COVERAGE')
        if any(b['sim_ns']-a['sim_ns']>d['max_gap_ns'] or b['sim_ns']<=a['sim_ns'] for a,b in zip(requests,requests[1:])):raise ValueError('MOTION_REQUEST_GAP')
        for x in requests:
            if x['entity_id']!=mapping['scripted_obstacle']:raise ValueError('ENTITY_MISMATCH')
            expected=desired(motion,x['sim_ns']-release)
            if len(x['desired_xyz'])!=3 or not all(math.isfinite(z) for z in x['desired_xyz']) or math.dist(x['desired_xyz'],expected)>1e-9:raise ValueError('COMMAND_TRAJECTORY_MISMATCH')
        max_command_step=max([math.dist(a['desired_xyz'],b['desired_xyz']) for a,b in zip(requests,requests[1:])],default=0.)
        if max_command_step>motion['max_speed_mps']*d['max_gap_ns']*1e-9+1e-9:raise ValueError('COMMAND_JUMP_LIMIT')
        latencies=[b['response']['receive_monotonic_s']-a['receive_monotonic_s'] for a,b in zip(requests,replies) if 'receive_monotonic_s' in a and 'receive_monotonic_s' in b['response']]
        if report['record_origin']=='real' and (len(latencies)!=len(requests) or any(not math.isfinite(x) or not 0<=x<=.5 for x in latencies)):raise ValueError('MOTION_SERVICE_TIMING')

        for s in motion_samples:errors.append(math.dist(s['xyz'],desired(motion,s['t_ns']-release)))
        report['trajectory']={'maximum_m':max(errors),'rms_m':math.sqrt(sum(x*x for x in errors)/len(errors)),'p95_m':sorted(errors)[math.ceil(.95*len(errors))-1],'max_truth_gap_ns':max(gaps),'sample_count':len(errors),'max_command_step_m':max_command_step,'service_latency_max_wall_s':max(latencies) if latencies else None,'skipped_schedule_points':sum(x.get('skipped_schedule_points',0) for x in requests),'time_shift_applied':False,'paused_source_duplicate_groups':[{k:s[k] for k in ['t_ns','merged_raw_lines','duplicate_position_spread_m','duplicate_angle_spread_rad']} for sequence in samples.values() for s in sequence if 'merged_raw_lines' in s]}
        if max(errors)>d['position_tolerance_m']:raise ValueError('ACTUAL_TRAJECTORY_ERROR')
        report['motion_valid']=True
        geometry=v.load_strict(run/'artifacts/dynamic_geometry.json');scans=read_scans(run) if scans is None else scans
        aligned={name:PoseSeries(seq,epoch,mapping[name],d['truth_gap_ns']) for name,seq in samples.items()}
        end_ns=release+motion['knots'][-1]['t_ns']+motion['hold_ns']
        chosen=[scan for scan in scans if release<=scan['t_ns']<=end_ns]
        report['analysis_window']={'start_ns':release,'end_ns':end_ns,'scope':'frozen motion plus terminal hold; startup and shutdown scans excluded explicitly','bag_scan_total':len(scans),'selected_scans':len(chosen),'outside_before':sum(scan['t_ns']<release for scan in scans),'outside_after':sum(scan['t_ns']>end_ns for scan in scans)}
        if not chosen or chosen[0]['t_ns']>release+300000000 or chosen[-1]['t_ns']<end_ns-300000000 or any(not 0<b['t_ns']-a['t_ns']<=300000000 for a,b in zip(chosen,chosen[1:])):raise ValueError('SCAN_WINDOW_COVERAGE')
        for s in chosen:
            try:label,ev=aligned_frame(s,aligned,mapping,epoch,geometry,d['range_tolerance_m'],d['truth_gap_ns'])
            except ValueError as exc:label,ev=None,[];report['errors'].append(str(exc))
            report['frames'].append({'t_ns':s['t_ns'],'bag_message_index':s.get('bag_message_index'),'bag_receive_ros_ns':s.get('bag_receive_ros_ns'),'label':label,'beams':ev})
        report['observation_valid']=sequence_valid(report['frames'],spec['kind'])
        if not report['observation_valid']:raise ValueError('VISIBILITY_NOT_ESTABLISHED')
        # Crossing needs independent actual path intersection; visibility alone does not imply crossing.
        if spec['kind']=='crossing' and p['mode']=='navigation_recording':
            intersection=polyline_crossing(samples['turtlebot3_waffle'],motion_samples)
            if not intersection:raise ValueError('CROSSING_GEOMETRY_NOT_ESTABLISHED')
            report['crossing']=intersection
            if not intersection['ahead_of_robot']:raise ValueError('CROSSING_NOT_AHEAD')
        report['dynamic_recording_valid']=not report['errors'] and p['mode']=='navigation_recording'
        if p['mode']=='probe':
            ev=raw_rows(run/'logs/events.jsonl')
            facilities,evidence=probe_facilities(raw,cmd,ev,obs)
            ready=v.load_strict(run/'runtime/dynamic-readiness.json')
            facilities['cold_initial_state']=ready['driver']['phase']=='WAITING' and math.dist(ready['driver']['actual_xyz'],motion['knots'][0]['xyz'])<=.02
            report['probe_facilities']={'checks':facilities,'evidence':evidence,'personal_visual_confirmation':None}
            report['probe_pass']=not report['errors'] and all(x is True for x in facilities.values())
            if not report['probe_pass']:report['errors'].append('PROBE_FACILITY_EVIDENCE_INCOMPLETE')
    except Exception as exc:report['errors'].append(str(exc))
    for name in ['runtime/gazebo-raw.jsonl','runtime/observations.jsonl','runtime/motion-commands.jsonl','logs/action-client.jsonl','bag/metadata.yaml']:
        f=run/name
        if f.is_file():report['source_files'].append({'path':name,'sha256':digest(f)})
    return report
def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run-dir',required=True);p.add_argument('--output',required=True);a=p.parse_args();report=inspect(a.run_dir);v.validate_structure(report,'dynamic_report',a.run_dir);write_json(a.output,report);print(json.dumps({k:v for k,v in report.items() if k!='frames'}));return 0 if report['dynamic_recording_valid'] or report['probe_pass'] else 3
if __name__=='__main__':raise SystemExit(main())
