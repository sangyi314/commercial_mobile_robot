"""Strict v2 preparation gates. No processes, ROS imports or network."""
from pathlib import Path
import copy,hashlib,json,math,re
import validate_run as v

def fingerprint(spec):
    def content(value):
        if isinstance(value,dict):return {k:content(v) for k,v in value.items() if k not in ('path','motion_ref')}
        if isinstance(value,list):return [content(v) for v in value]
        return value
    value={'resources':spec['resources'],'task':spec['task'],'kind':spec['kind'],'events':spec['dynamic_events'],
           'dynamic':{k:x for k,x in spec['dynamic'].items() if k not in ('preparation','purpose','trigger')}}
    return hashlib.sha256(json.dumps(content(value),sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()

def resource(ref,check=True):
    if not isinstance(ref,dict) or set(ref)!={'path','sha256'} or not isinstance(ref['path'],str) or '://' in ref['path'] or '\0' in ref['path'] or not re.fullmatch('[a-f0-9]{64}',ref.get('sha256') or ''):
        raise ValueError('invalid dynamic resource')
    p=v.host_path(ref['path'])
    if check:v.nonempty_file(p,str(p),ref['sha256'])
    return p

def motion_rules(m):
    if set(m)!={'schema_version','frame','knots','max_speed_mps','period_ns','max_gap_ns','hold_ns'} or m['schema_version']!=1 or m['frame']!='world':raise ValueError('motion fields/frame')
    knots=m['knots']
    if len(knots)<2 or knots[0]['t_ns']!=0 or m['period_ns']!=20000000 or not m['period_ns']<=m['max_gap_ns']<=50000000 or m['hold_ns']<1000000000:raise ValueError('motion timing')
    for x in knots:
        if set(x)!={'t_ns','xyz'} or type(x['t_ns']) is not int or x['t_ns']<0 or len(x['xyz'])!=3 or any(type(y) not in (int,float) or not math.isfinite(y) for y in x['xyz']):raise ValueError('motion finite values')
    if not 0<m['max_speed_mps']<=.15:raise ValueError('motion speed')
    for a,b in zip(knots,knots[1:]):
        dt=(b['t_ns']-a['t_ns'])*1e-9
        if dt<=0 or math.dist(a['xyz'],b['xyz'])/dt>m['max_speed_mps']+1e-10:raise ValueError('motion nonmonotonic/too fast')
    return m

def rules(spec,path,check_host=True):
    d=spec['dynamic']
    try:
        if spec['kind'] not in ('crossing','observed_occlusion') or spec['master_scenario_id']!={'crossing':'S1','observed_occlusion':'S3'}[spec['kind']]:raise ValueError('dynamic kind/master')
        if d['entity_name']!='scripted_obstacle' or d['frame']!='world' or d['time_domain']!='sim_ns':raise ValueError('entity/frame/time')
        expected='probe_armed' if d['purpose']=='probe' else 'goal_accepted'
        if d['trigger']!=expected or d['delay_ns']<1000000000:raise ValueError('trigger/delay')
        for role,typ in [('raw_pose','gz.msgs.Pose_V'),('raw_clock','gz.msgs.Clock'),('raw_stats','gz.msgs.WorldStatistics'),('entity_tree','gz.msgs.Scene')]:
            entries=[i for i in spec['interfaces'] if i['role']==role]
            if len(entries)!=1 or entries[0]['type']!=typ or entries[0]['usage']!='evaluation_only' or entries[0]['frame_id']!='world' or entries[0]['time_domain']!='gz_sim':raise ValueError('evaluation interface leak/frame')
        if len(spec['dynamic_events'])!=1 or spec['dynamic_events'][0]['actor_id']!=d['entity_name']:raise ValueError('single entity/event')
        if spec['dynamic_events'][0]['trigger'] is not None or spec['dynamic_events'][0]['motion_ref']!=d['motion']['path']:raise ValueError('conflicting legacy event reference')
        for name in ['actor','motion','geometry']:resource(d[name],check_host)
        if check_host:motion_rules(v.load_strict(resource(d['motion'])))
        if d['period_ns']!=20000000 or d['max_gap_ns']>50000000 or d['truth_gap_ns']>50000000:raise ValueError('unreviewed timing bounds')
        if spec['status']=='draft':return
        resource(d['preparation']['review'],check_host)
        if spec['status']=='ready_for_recording':resource(d['preparation']['probe'],check_host)
        if any(x is None or x['sha256'] is None for x in spec['resources'].values()) or any(spec['task'][k] is None for k in ('initial_pose','goal_pose','common_constraints_ref')):raise ValueError('ready requires resources/poses')
        for ref in spec['resources'].values():
            if check_host:v.nonempty_file(v.host_path(ref['path']),str(path),ref['sha256'])
        if check_host:
            lock=v.load_strict(v.host_path(spec['resources']['dependency_lock']['path']))
            if lock.get('unresolved') or not lock.get('resources'):raise ValueError('dependency closure unresolved')
            locked={}
            for ref in lock['resources']:
                p=resource(ref);key=str(p.resolve())
                if key in locked:raise ValueError('duplicate dependency')
                locked[key]=ref['sha256']
            for ref in [*spec['resources'].values(),d['actor'],d['motion'],d['geometry']]:
                if ref==spec['resources']['dependency_lock']:continue
                if locked.get(str(v.host_path(ref['path']).resolve()))!=ref['sha256']:raise ValueError('dependency not locked')
            prep=v.load_strict(resource(d['preparation']['review']))
            if prep.get('record_origin')!='real' or prep.get('scope')!='static_preparation' or prep.get('input_fingerprint')!=fingerprint(spec) or prep.get('candidate_clearance_reviewed') is not True:raise ValueError('static preparation identity/scope')
            if not prep.get('evidence'):raise ValueError('static preparation raw evidence missing')
            for ref in prep['evidence']:resource(ref)
            if spec['status']=='ready_for_recording':
                p=v.load_strict(resource(d['preparation']['probe']))
                required=['visual','collision','scan','time_alignment','pause','cold_start','truth_isolation']
                if p.get('record_origin')!='real' or p.get('input_fingerprint')!=fingerprint(spec) or not all(p.get('checks',{}).get(k) is True for k in required):raise ValueError('real probe checks absent')
                if not p.get('evidence'):raise ValueError('probe raw evidence absent')
                for ref in p['evidence']:resource(ref)
                run=resource(p['manifest']).parent
                if v.load_strict(run/'artifacts/profile.json').get('mode')!='probe':raise ValueError('not a probe purpose')
                if v.validate_run_dir(run)['status']!='VALID':raise ValueError('invalid real probe run')
                manifest=v.load_strict(run/'manifest.json');result=v.load_strict(run/'result.json')
                if manifest['record_origin']!='real' or result['status']['startup']!='READY' or result['status']['harness']!='COMPLETED' or result['status']['action']!='NOT_SENT' or result['status']['cleanup']!='SUCCEEDED':raise ValueError('not clean real probe')
    except (ValueError,KeyError,TypeError) as exc:v.fail('DYNAMIC_CONTRACT',str(path),exc)

def profile_inputs(spec,profile,method,legacy_loader,identity):
    if set(profile)-{'profile_version','scope','mode','base_profile','base_scenario','scenario_ref','executables','probe_sequence','window_s','pause_wall_s'}:raise ValueError('unknown dynamic profile field')
    if type(profile['profile_version']) is not int or profile['profile_version']!=2 or profile['scope']!='simulation_only' or method!='mppi':raise ValueError('dynamic mode/method')
    mode=profile['mode']
    if mode not in ('probe','navigation_recording') or spec['dynamic']['purpose']!=mode:raise ValueError('purpose mismatch')
    if spec['status']!=('ready_for_probe' if mode=='probe' else 'ready_for_recording'):raise ValueError('dynamic draft/not prepared')
    resource(profile['scenario_ref'])
    if v.load_strict(resource(profile['scenario_ref']))!=spec:raise ValueError('scenario/profile mismatch')
    if Path(profile['base_profile']).resolve()!=v.PROJECT_ROOT/'experiments/runner_profile.yaml' or Path(profile['base_scenario']).resolve()!=v.PROJECT_ROOT/'experiments/scenarios/p0_05_official_static.yaml':raise ValueError('unreviewed base profile')
    base,base_spec=legacy_loader(profile['base_scenario'],profile['base_profile'],method,identity)
    if spec['task']['initial_pose']!=base_spec['task']['initial_pose'] or spec['task']['common_constraints_ref']['sha256']!=base['files']['constraints']['sha256']:raise ValueError('unreviewed initial pose/common constraints')
    for name in ['robot','map_yaml','map_image']:
        if spec['resources'][name]['sha256']!=base['files'][name]['sha256']:raise ValueError('unreviewed baseline resource')
    if set(profile['executables'])!={'driver','capture'}:raise ValueError('executable roles')
    # executable paths are restricted to this repository's isolated cmr_sim install.
    for role in ('driver','capture'):
        p=resource(profile['executables'][role]).resolve()
        if not p.is_relative_to(v.PROJECT_ROOT/'install') or p.name!={'driver':'cmr_scene_driver','capture':'cmr_truth_capture'}[role]:raise ValueError('unreviewed executable')
    motion=motion_rules(v.load_strict(resource(spec['dynamic']['motion'])))
    if motion['knots'][-1]['t_ns']+motion['hold_ns']>profile['window_s']*1000000000:raise ValueError('motion exceeds scene budget')
    if not 1<=profile['window_s']<=90 or not 0<=profile['pause_wall_s']<=3:raise ValueError('dynamic budget')
    allowed={'interfaces','motion','pause','contact_positive','contact_negative'}
    if set(profile['probe_sequence'])!=allowed or len(profile['probe_sequence'])!=len(allowed):raise ValueError('fixed probe whitelist')
    base=copy.deepcopy(base);base.update({'profile_version':2,'mode':mode,'dynamic_profile':profile,'dynamic_spec':spec})
    for name in ('world','map_yaml','map_image','robot'):
        base['files'][name]=spec['resources'][name]
    for name in ('actor','motion','geometry'):
        base['files']['dynamic_'+name]=spec['dynamic'][name]
    goal=v.load_strict(resource(spec['dynamic']['goal_request']))
    gp=goal['pose']['pose'];gq=gp['orientation'];yaw=math.atan2(2*(gq['w']*gq['z']+gq['x']*gq['y']),1-2*(gq['y']**2+gq['z']**2))
    if goal['pose']['header']['frame_id']!='map' or any(abs(a-b)>1e-9 for a,b in zip([gp['position']['x'],gp['position']['y'],yaw],[spec['task']['goal_pose'][k] for k in ['x_m','y_m','yaw_rad']])):raise ValueError('task request mismatch')
    base['files']['goal']=spec['dynamic']['goal_request']
    if base['files']['initialpose']['sha256'] is None:raise ValueError('initialpose missing')
    base['recording']['qos']=str(v.PROJECT_ROOT/'experiments/recording_qos_p0_06.yaml')
    return base,spec

def freeze_inputs(run,spec):
    """Copy closure and preparation bytes; archived references remain run-relative."""
    from runner_core import atomic_write_new,write_json,digest
    frozen=copy.deepcopy(spec);art=Path(run)/'artifacts';counter=0;cached={}
    def freeze(ref):
        nonlocal counter
        if ref is None:return None
        p=resource({'path':ref['path'],'sha256':ref['sha256']});key=(str(p.resolve()),ref['sha256'])
        if key not in cached:
            name='dynamic_dep_'+str(counter);counter+=1
            atomic_write_new(art/name,p.read_bytes());cached[key]='artifacts/'+name
        result=copy.deepcopy(ref);result['path']=cached[key];return result
    lock=v.load_strict(v.host_path(spec['resources']['dependency_lock']['path']))
    locked=[freeze(ref) for ref in lock['resources']]
    for k,ref in frozen['resources'].items():
        if k!='dependency_lock':frozen['resources'][k]=freeze(ref)
    frozen['task']['common_constraints_ref']=freeze(frozen['task']['common_constraints_ref'])
    for k in ['actor','motion','geometry','goal_request']:frozen['dynamic'][k]=freeze(frozen['dynamic'][k])
    for k in ['review','probe']:
        ref=frozen['dynamic']['preparation'][k]
        if ref:
            prep=v.load_strict(v.host_path(ref['path']))
            for source in prep.get('evidence',[]):freeze(source)
        frozen['dynamic']['preparation'][k]=freeze(ref)
    write_json(art/'dynamic-archive-closure.json',{'resources':locked,'unresolved':[]})
    frozen['resources']['dependency_lock']=freeze(spec['resources']['dependency_lock'])
    frozen['dynamic_events'][0]['motion_ref']=frozen['dynamic']['motion']['path']
    return frozen

def archive_rules(spec,refs,directory,path):
    entries=[*spec['resources'].values(),spec['task']['common_constraints_ref']]
    d=spec['dynamic'];entries += [d[k] for k in ['actor','motion','geometry','goal_request']]
    entries += list(d['preparation'].values())
    actual={str(p.resolve()):a['sha256'].lower() for a,p in refs.values()}
    for ref in entries:
        if ref is None:continue
        p=Path(ref['path'])
        if p.is_absolute() or '..' in p.parts:v.fail('DYNAMIC_ARCHIVE_PATH',path,'v2 archive reference must be run relative')
        resolved=(Path(directory)/p).resolve()
        if actual.get(str(resolved))!=ref['sha256'].lower():v.fail('DYNAMIC_ARCHIVE_HASH',path,'v2 resource is not bound to archived bytes')

    lock=v.load_strict(Path(directory)/spec['resources']['dependency_lock']['path'])
    hashes=set(actual.values())
    if lock.get('unresolved') or any(x['sha256'].lower() not in hashes for x in lock.get('resources',[])):v.fail('DYNAMIC_ARCHIVE_CLOSURE',path,'locked content absent from run')
