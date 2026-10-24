"""After step-6 approval: create a new, static-reviewed probe input. No ROS."""
from pathlib import Path
import argparse,copy,json,xml.etree.ElementTree as ET
import validate_run as v
from dynamic_contract import fingerprint
from runner_core import write_json,digest
ROOT=Path(__file__).resolve().parents[1]
def structural(e):
    return (e.tag,sorted(e.attrib.items()),(e.text or '').strip(),[structural(c) for c in e])
def static_review(spec):
    actor=ET.parse(spec['dynamic']['actor']['path']).getroot().find('model')
    visual=actor.find('link/visual/geometry');collision=actor.find('link/collision/geometry')
    if structural(visual)!=structural(collision):raise ValueError('visual/collision geometry differs')
    if actor.findtext('static')!='true':raise ValueError('unreviewed actor dynamics')
    world=ET.parse(spec['resources']['world']['path']).getroot()
    for child in list(world.find('world')):
        if child.tag=='model' and child.get('name')=='scripted_obstacle':world.find('world').remove(child)
        elif child.tag=='plugin' and child.get('name') in ['gz::sim::systems::SceneBroadcaster','gz::sim::systems::Contact']:world.find('world').remove(child)
    original=ET.parse(ROOT/'evidence/env/p0-04-pair-q2MXrdh3/inputs/world-ogre1.sdf').getroot()
    if structural(world)!=structural(original):raise ValueError('static world changed')
    return {'schema_version':1,'scope':'static_file_review','visual_collision_same_geometry':True,'original_static_world_preserved':True,'physical_collision':None,'laser_visibility':None,'candidate_coordinates':'Engineer reviewed candidate clearances under explicit user authorization; runtime geometry pending.'}
def prepare(scenario,profile,output,ack,as_probe=False):
    if not ack:raise ValueError('explicit engineering review acknowledgement required')
    spec=v.load_strict(scenario);p=v.load_strict(profile)
    if as_probe:
        if spec['status']!='draft' or p['mode']!='navigation_recording':raise ValueError('only draft conversion')
        spec=copy.deepcopy(spec);p=copy.deepcopy(p);spec['scenario_id']+='-probe'
        spec['dynamic']['purpose']='probe';spec['dynamic']['trigger']='probe_armed';p['mode']='probe'
    if spec['status']!='draft' or spec['dynamic']['purpose']!='probe' or p['mode']!='probe':raise ValueError('only draft probe preparation')
    v.validate_structure(spec,'scenario',scenario);v.scenario_rules(spec,scenario)
    assessment=static_review(spec);output=Path(output).resolve()
    output.mkdir(parents=True,exist_ok=False)
    write_json(output/'static-review.json',assessment)
    lock=v.load_strict(spec['resources']['dependency_lock']['path'])
    files=[Path(x['path']) for x in lock['resources']]
    files += [Path(x['path']) for x in spec['resources'].values() if x and x!=spec['resources']['dependency_lock']]
    files += [Path(spec['dynamic'][k]['path']) for k in ['actor','motion','geometry','goal_request']]
    files += [Path(x['path']) for x in p['executables'].values()]
    if len(p['executables'])!=2:raise ValueError('executables not built')
    base=v.load_strict(p['base_profile'])
    files += [Path(p['base_profile']),Path(p['base_scenario']),Path(base['recording']['qos']),ROOT/'experiments/recording_qos_p0_06.yaml']
    files += [Path(x['config']) for x in base['methods'].values()]
    files += [Path(base['environment']['FASTRTPS_DEFAULT_PROFILES_FILE'])]
    files += [Path(base['files']['robot_source']['path'])]
    files += list((ROOT/'tools').glob('*.py'))+list((ROOT/'experiments/schemas').glob('*.json'))
    files += list((ROOT/'src/cmr_sim').rglob('*.cpp'))+list((ROOT/'src/cmr_sim').rglob('*.hpp'))
    closure={'resources':[{'path':str(f),'sha256':digest(f)} for f in sorted(set(files))],'unresolved':[]}
    write_json(output/'dependencies.json',closure)
    spec['resources']['dependency_lock'].update({'path':str(output/'dependencies.json'),'sha256':digest(output/'dependencies.json')})
    spec['status']='ready_for_probe';spec['scenario_revision']+='-reviewed'
    review={'schema_version':1,'record_origin':'real','scope':'static_preparation','input_fingerprint':fingerprint(spec),'candidate_clearance_reviewed':True,'acknowledgement':'Engineer review under explicit user continuation authorization; physical/interface/scan pending; personal observation and understanding not signed','evidence':[{'path':str(output/'static-review.json'),'sha256':digest(output/'static-review.json')}]}
    write_json(output/'preparation.json',review)
    spec['dynamic']['preparation']['review']={'path':str(output/'preparation.json'),'sha256':digest(output/'preparation.json')}
    write_json(output/'probe.yaml',spec)
    p['scenario_ref']={'path':str(output/'probe.yaml'),'sha256':digest(output/'probe.yaml')}
    write_json(output/'profile.yaml',p)
    status=v.validate_scenario(output/'probe.yaml')
    write_json(output/'validation.json',status)
    if status['status']!='VALID':raise ValueError(status)
    return {'scenario':str(output/'probe.yaml'),'profile':str(output/'profile.yaml'),'scope':'static permission for probe; no runtime claim'}
def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--scenario',required=True);ap.add_argument('--profile',required=True);ap.add_argument('--output-dir',required=True);ap.add_argument('--ack-review',action='store_true');ap.add_argument('--as-probe',action='store_true');a=ap.parse_args()
    print(json.dumps(prepare(a.scenario,a.profile,a.output_dir,a.ack_review,a.as_probe),ensure_ascii=False))
if __name__=='__main__':main()
