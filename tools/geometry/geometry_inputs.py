"""Strict, read-only P0-06 Gazebo raw-pose adapter for P0-07.

No ROS dependency. A frozen spec binds the source bytes and entity tree.
Protobuf scalar omission means zero only inside an existing pose message.
"""
import hashlib
import itertools
import json
import math
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

from tools.geometry.geometry_metrics import number, stamp

from tools import PROJECT_ROOT as ROOT
SOURCE_PATHS = ['manifest.json', 'result.json', 'artifacts/robot.sdf',
                'artifacts/dynamic_actor.sdf', 'artifacts/world.sdf',
                'artifacts/profile.json', 'artifacts/dynamic_motion.json',
                'runtime/gazebo-raw.jsonl', 'runtime/motion-commands.jsonl',
                'runtime/dynamic-readiness.json', 'logs/action-client.jsonl']


def fail(message):
    raise ValueError(message)


def unique(pairs):
    result = {}
    for k, v in pairs:
        if k in result: fail('DUPLICATE_JSON_KEY:'+k)
        result[k] = v
    return result


def loads(text):
    return json.loads(text, object_pairs_hook=unique,
                      parse_constant=lambda x: fail('NONFINITE_JSON:'+x))


def read(path):
    return loads(Path(path).read_text())


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(1048576), b''): h.update(block)
    return h.hexdigest()


def rows(path):
    with Path(path).open() as f:
        for line, text in enumerate(f, 1):
            try: yield line, loads(text)
            except (ValueError, TypeError) as e: fail(f'{path}:{line}: {e}')


def schema(value, name):
    from jsonschema import Draft202012Validator
    s = read(ROOT/'experiments/schemas'/f'{name}.schema.json')
    Draft202012Validator(s).validate(value)


def raw_time(s):
    if not isinstance(s, dict): fail('SOURCE_STAMP_REQUIRED')
    sec, ns = s.get('sec', 0), s.get('nsec', 0)
    if isinstance(sec, str) and sec.isdecimal(): sec = int(sec)
    if type(sec) is not int or type(ns) is not int or not 0 <= ns < 10**9:
        fail('SOURCE_STAMP_INVALID')
    return stamp(sec*10**9+ns)


def pose(p):
    if not isinstance(p.get('position'), dict) or not isinstance(p.get('orientation'), dict):
        fail('POSE_COMPONENT_MISSING')
    xyz = [number(p['position'].get(k, 0)) for k in 'xyz']
    q = [number(p['orientation'].get(k, 0)) for k in 'xyzw']
    if abs(math.hypot(*q)-1) > 1e-8: fail('NONUNIT_QUATERNION')
    return xyz, q


def rotate(q, v):
    x, y, z, w = q
    t = [2*(y*v[2]-z*v[1]), 2*(z*v[0]-x*v[2]), 2*(x*v[1]-y*v[0])]
    return [v[0]+w*t[0]+y*t[2]-z*t[1],
            v[1]+w*t[1]+z*t[0]-x*t[2], v[2]+w*t[2]+x*t[1]-y*t[0]]


def transform(xyz, q, v):
    return [a+b for a, b in zip(xyz, rotate(q, v))]


def sdf_pose(node):
    p = node.find('pose')
    if p is None: return [0., 0., 0.], [0., 0., 0., 1.]
    if p.attrib: fail('UNSUPPORTED_SDF_POSE_FRAME')
    values = [number(float(x)) for x in p.text.split()]
    if len(values) != 6: fail('SDF_POSE_SIZE')
    r, p, y = [v/2 for v in values[3:]]
    cr, cp, cy, sr, sp, sy = math.cos(r), math.cos(p), math.cos(y), math.sin(r), math.sin(p), math.sin(y)
    return values[:3], [sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy]


def collisions(path):
    root = ET.parse(path).getroot()
    model = root.find('model')
    if model is None or model.findall('model') or model.findall('include'): fail('UNSUPPORTED_NESTED_SDF')
    result = []
    for link in model.findall('link'):
        lp, lq = sdf_pose(link)
        for c in link.findall('collision'):
            cp, cq = sdf_pose(c); g = c.find('geometry')
            if g is None or len(g) != 1: fail('COLLISION_GEOMETRY')
            kind = g[0].tag
            if kind == 'box': dims = [float(x) for x in g.findtext('box/size').split()]; rho = math.hypot(*dims)/2
            elif kind == 'cylinder':
                dims = [float(g.findtext('cylinder/radius')), float(g.findtext('cylinder/length'))]; rho = math.hypot(dims[0], dims[1]/2)
            elif kind == 'sphere': dims = [float(g.findtext('sphere/radius'))]; rho = dims[0]
            else: fail('UNSUPPORTED_COLLISION:'+kind)
            for d in dims:
                if number(d) <= 0: fail('GEOMETRY_DIMENSION')
            result.append({'link': link.attrib['name'], 'collision': c.attrib['name'], 'kind': kind,
                           'dimensions': dims, 'link_xyz': lp, 'link_quaternion': lq,
                           'collision_xyz': cp, 'collision_quaternion': cq,
                           'shape_bound_m': rho, 'origin_bound_m': math.hypot(*lp)+math.hypot(*cp)+rho})
    if not result: fail('NO_COLLISIONS')
    return model.attrib['name'], result


def scene_identity(raw, names):
    found = None
    for line, row in rows(raw):
        if row['kind'] not in ('scene', 'scene_snapshot'): continue
        models = row['protobuf'].get('model', [])
        present = [m for m in models if m.get('name') in names]
        if found:
            for m in present:
                identity = {'id': m['id'], 'links': {l['name']: l['id'] for l in m.get('link', [])}}
                if identity != found['models'][m['name']]: fail('SCENE_IDENTITY_CHANGED')
        if len(present) != len(names): continue  # Incremental scene events may omit unchanged entities.
        current = {m['name']: {'id': m['id'], 'links': {l['name']: l['id'] for l in m.get('link', [])}} for m in present}
        if len(current) != len(names): fail('SCENE_DUPLICATE_MODEL')
        for m in present:
            links = m.get('link', [])
            if len({l['name'] for l in links}) != len(links) or len({l['id'] for l in links}) != len(links):
                fail('SCENE_DUPLICATE_LINK')
        if found and current != found['models']: fail('SCENE_IDENTITY_CHANGED')
        if found is None: found = {'models': current, 'line': line, 'seq': row['seq']}
    if found is None: fail('ROOT_SCENE_IDENTITY_MISSING')
    return found


def actual_model(entries, name, identity):
    """Select a model from an actual Pose_V, never a command/AMCL/odom fallback."""
    if any(p.get('name') == name and p['id'] != identity for p in entries): fail('ENTITY_REPLACED')
    candidates = [p for p in entries if p['id'] == identity]
    if not candidates: return None
    if len(candidates) != 1 or candidates[0].get('name') != name: fail('ENTITY_NAME_CONFLICT')
    pose(candidates[0])
    return candidates[0]


def scope(run):
    action = list(rows(run/'logs/action-client.jsonl'))
    accepted = [(n, r) for n, r in action if r['kind'] == 'response' and r['data'].get('accepted') is True]
    terminals = [(n, r) for n, r in action if r['kind'] == 'terminal']
    dispatch = [r for _, r in action if r['kind'] == 'dispatch']
    if len(accepted) != 1 or len(terminals) != 1 or len(dispatch) != 1: fail('UNIQUE_ACTION_REQUIRED')
    an, a = accepted[0]; tn, t = terminals[0]
    uuid = a['data']['uuid']
    if t['data']['uuid'] != uuid or dispatch[0]['data']['uuid'] != uuid: fail('ACTION_IDENTITY')
    arms = [(n, r) for n, r in rows(run/'runtime/motion-commands.jsonl') if r['kind'] == 'arm']
    if len(arms) != 1: fail('UNIQUE_TRIGGER_REQUIRED')
    mn, arm = arms[0]
    lo = round(number(a['ros_s'])*1e9); terminal = round(number(t['ros_s'])*1e9)
    if arm['event']['uuid'] != uuid or arm['event']['trigger_ns'] != lo: fail('TRIGGER_ACTION_ALIGNMENT')
    motion = read(run/'artifacts/dynamic_motion.json')
    end = arm['release_ns']+motion['knots'][-1]['t_ns']+motion['hold_ns']
    return {'interval_ns': [lo, max(terminal+5_000_000_000, end)],
            'rule': 'accepted_to_max_terminal_plus_5s_or_frozen_motion_end', 'uuid': uuid,
            'action_lines': [an, tn], 'motion_arm_line': mn, 'terminal': t['data']['action']}


def prepare(run):
    run = Path(run).resolve(strict=True); manifest = read(run/'manifest.json')
    if manifest['record_origin'] != 'real': fail('REAL_RUN_REQUIRED')
    rn, rc = collisions(run/'artifacts/robot.sdf'); on, oc = collisions(run/'artifacts/dynamic_actor.sdf')
    tree = scene_identity(run/'runtime/gazebo-raw.jsonl', [rn, on])
    return {'schema_version': 1, 'run_id': manifest['run_id'], 'record_origin': 'real',
            'epoch': manifest['clock']['clock_epoch'], 'frame': 'world',
            'scope': scope(run), 'max_gap_ns': 50_000_000,
            'robot_name': rn, 'obstacle_name': on, 'scene_identity': tree,
            'source_hashes': {p: sha(run/p) for p in SOURCE_PATHS},
            'robot_radius_rule': 'all_collision_3d_bound_about_model_origin',
            'physical_error_bound_m': None}


def verify_sources(run, spec):
    if set(spec['source_hashes']) != set(SOURCE_PATHS): fail('SOURCE_SET_MISMATCH')
    for rel, expected in spec['source_hashes'].items():
        path = (run/rel).resolve(strict=True)
        if not path.is_relative_to(run): fail('SOURCE_SYMLINK_ESCAPE')
        if sha(path) != expected: fail('SOURCE_HASH_CHANGED:'+rel)


def adapt(run, spec, validate_record=True):
    run = Path(run).resolve(strict=True); schema(spec, 'evaluation_spec'); verify_sources(run, spec)
    if validate_record:
        p = subprocess.run([sys.executable, '-B', str(ROOT/'tools/validate_run.py'), 'run', str(run)],
                           capture_output=True, text=True, timeout=120)
        if p.returncode: fail('RECORD_INVALID:'+p.stdout+p.stderr)
        validation = {'exit_code': p.returncode, 'stdout': p.stdout, 'stderr': p.stderr}
    else: validation = None  # Unit tests only; CLI never disables existing validator.
    manifest = read(run/'manifest.json')
    if manifest['record_origin'] != 'real' or manifest['run_id'] != spec['run_id'] or manifest['clock']['clock_epoch'] != spec['epoch']:
        fail('MANIFEST_IDENTITY')
    if scope(run) != spec['scope']: fail('SCOPE_CHANGED')
    profile = read(run/'artifacts/profile.json')
    raw_if = [i for i in profile['dynamic_spec']['interfaces'] if i['role'] == 'raw_pose']
    if raw_if != [{'role': 'raw_pose', 'topic': '/world/default/pose/info', 'type': 'gz.msgs.Pose_V', 'frame_id': 'world', 'time_domain': 'gz_sim', 'usage': 'evaluation_only'}]: fail('RAW_POSE_INTERFACE')
    alignment = read(run/'runtime/dynamic-readiness.json')['clock_alignment']
    if alignment['samples'] < 5 or alignment['max_difference_ns'] != 0 or alignment['fit_or_time_shift_applied'] is not False: fail('CLOCK_MAPPING_UNCONFIRMED')
    rn, rc = collisions(run/'artifacts/robot.sdf'); on, oc = collisions(run/'artifacts/dynamic_actor.sdf')
    if rn != spec['robot_name'] or on != spec['obstacle_name']: fail('RESOURCE_MODEL_IDENTITY')
    if len(oc) != 1 or oc[0]['kind'] != 'cylinder': fail('ACTOR_NOT_SINGLE_CYLINDER')
    ac = oc[0]
    if any(ac[k] != [0., 0., 0.] for k in ['link_xyz', 'collision_xyz']) or any(ac[k] != [0., 0., 0., 1.] for k in ['link_quaternion', 'collision_quaternion']): fail('ACTOR_OFFSET_UNSUPPORTED')
    tree = scene_identity(run/'runtime/gazebo-raw.jsonl', [rn, on])
    if tree != spec['scene_identity']: fail('ENTITY_TREE_CHANGED')
    rr = max(c['origin_bound_m'] for c in rc); ar = ac['dimensions'][0]
    objects = {n: {'name': n, 'entity_id': tree['models'][n]['id'], 'radius_m': r,
                   'center_offset_xyz': [0., 0., 0.], 'samples': []} for n, r in [(rn, rr), (on, ar)]}
    lo, hi = spec['scope']['interval_ns']; gap = spec['max_gap_ns']
    last_seq = 0; last_t = {}; excluded = 0; max_bound = None; clock_count = 0; pose_count = 0
    for line, row in rows(run/'runtime/gazebo-raw.jsonl'):
        if row['seq'] != last_seq+1 or row['run_id'] != spec['run_id'] or row['clock_epoch'] != spec['epoch'] or row['format_version'] != 1: fail(f'RAW_IDENTITY_SEQUENCE:{line}')
        last_seq = row['seq']; kind = row['kind']; pb = row['protobuf']
        if kind not in ('pose', 'clock'): continue
        t = raw_time(pb['header']['stamp'] if kind == 'pose' else pb['sim'])
        if t < last_t.get(kind, 0): fail(f'CLOCK_ROLLBACK:{line}')
        last_t[kind] = t
        if kind == 'clock': clock_count += 1; continue
        pose_count += 1
        if not lo-gap <= t <= hi+gap: excluded += 1; continue
        entries = pb['pose']; by_id = {p['id']: p for p in entries}
        if len(by_id) != len(entries): fail('DUPLICATE_POSE_ENTITY')
        for name, obj in objects.items():
            p = actual_model(entries, name, obj['entity_id'])
            if p is None: continue  # Missing data stays missing; alignment determines coverage.
            if p.get('name') != name: fail('ENTITY_NAME_CONFLICT')
            xyz, q = pose(p)
            links = tree['models'][name]['links']
            if name == rn:
                for c in rc:
                    lp = by_id.get(links[c['link']])
                    if lp is None or lp.get('name') != c['link']: fail('COLLISION_LINK_POSE_MISSING')
                    lxyz, lq = pose(lp)
                    if c['kind'] == 'box':
                        vertices = itertools.product(*[[-d/2, d/2] for d in c['dimensions']])
                        bound = max(math.hypot(*transform(lxyz, lq, transform(c['collision_xyz'], c['collision_quaternion'], v))) for v in vertices)
                    else: bound = math.hypot(*transform(lxyz, lq, c['collision_xyz']))+c['shape_bound_m']
                    max_bound = bound if max_bound is None else max(max_bound, bound)
                    if bound > rr+1e-12: fail('ROBOT_ENVELOPE_NOT_CONTAINING')
            else:
                lp = by_id.get(links[ac['link']])
                if lp is None: fail('ACTOR_LINK_POSE_MISSING')
                lxyz, lq = pose(lp)
                if math.hypot(*lxyz) > 1e-12 or abs(rotate(lq, [0.,0.,1.])[2]-1) > 1e-12 or abs(rotate(q, [0.,0.,1.])[2]-1) > 1e-12: fail('ACTOR_NOT_UPRIGHT_CENTERED')
            obj['samples'].append({'t_ns': t, 'entity_id': obj['entity_id'], 'frame': 'world', 'epoch': spec['epoch'],
                'xyz': xyz, 'quaternion_xyzw': q, 'center_xyz': transform(xyz, q, obj['center_offset_xyz']),
                'source': {'file': 'runtime/gazebo-raw.jsonl', 'line': line, 'seq': row['seq'], 'pose_index': entries.index(p)}})
    if not clock_count or not pose_count: fail('SOURCE_TIME_OR_POSE_MISSING')
    scale = max([rr, ar, 1.] + [abs(v) for o in objects.values() for s in o['samples'] for v in s['center_xyz']])
    data = {'schema_version': 1, 'record_origin': 'real', 'run_id': spec['run_id'], 'frame': 'world', 'epoch': spec['epoch'],
            'interval_ns': [lo, hi], 'max_gap_ns': gap, 'numeric_epsilon_m': max(1e-12, 64*math.ulp(scale)),
            'objects': list(objects.values()), 'pairs': [[rn, on]], 'physical_error_bound_m': None}
    schema(data, 'evaluation_input'); verify_sources(run, spec)
    robot_z = [s['xyz'][2] for s in objects[rn]['samples']]
    actor_z = [s['xyz'][2] for s in objects[on]['samples']]
    tilt = [math.acos(max(-1., min(1., rotate(s['quaternion_xyzw'], [0., 0., 1.])[2]))) for s in objects[rn]['samples']]
    audit = {'record_validation': validation, 'source_root': str(run), 'source_hashes': spec['source_hashes'],
             'scene_identity': tree, 'geometry': {'robot': rc, 'actor': oc, 'robot_radius_m': rr,
             'actor_radius_m': ar, 'max_observed_robot_bound_m': max_bound},
             'sampled_height_envelopes_world_m': {
                 rn: [min(robot_z)-rr, max(robot_z)+rr] if robot_z else None,
                 on: [min(actor_z)-ac['dimensions'][1]/2, max(actor_z)+ac['dimensions'][1]/2] if actor_z else None},
             'max_observed_robot_tilt_rad': max(tilt) if tilt else None,
             'pose_packets': pose_count, 'excluded_outside_scope_and_support': excluded,
             'raw_clock_packets': clock_count, 'clock_alignment': alignment,
             'transform': 'world<-model from raw Pose_V; zero model-center offset; collision links model-relative',
             'missing_scalar_semantics': 'protobuf default zero only inside present position/orientation messages',
             'unknowns': ['continuous physical trajectory error', 'global collision', 'unmodelled objects', 'first contact time']}
    return data, audit
