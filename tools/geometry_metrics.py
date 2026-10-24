"""Pure circle/linear-segment evaluation. No ROS, files or subprocesses."""
import bisect
import math
from fractions import Fraction

LIMIT = 1e6


def number(x, limit=LIMIT):
    if type(x) not in (int, float) or not math.isfinite(x) or abs(x) > limit:
        raise ValueError('INVALID_FINITE_NUMBER')
    return float(x)


def point(p):
    if not isinstance(p, (list, tuple)) or len(p) != 2:
        raise ValueError('POINT_DIMENSION')
    return [number(x) for x in p]


def radius(r):
    r = number(r)
    if r < 0:
        raise ValueError('NEGATIVE_RADIUS')
    return r


def stamp(t):
    if type(t) is not int or not 0 <= t <= 9_000_000_000_000_000:
        raise ValueError('INTEGER_SOURCE_TIME_REQUIRED')
    return t


def signed_clearance(a, b, ra, rb):
    a, b = point(a), point(b)
    return math.hypot(b[0]-a[0], b[1]-a[1])-radius(ra)-radius(rb)


def segment_minimum(a0, a1, b0, b1, ra, rb, t0, t1):
    a0, a1, b0, b1 = map(point, (a0, a1, b0, b1))
    ra, rb = radius(ra), radius(rb)
    stamp(t0); stamp(t1)
    if t1 < t0:
        raise ValueError('TIME_REVERSED')
    if t1 == t0 and (a0 != a1 or b0 != b1):
        raise ValueError('DEGENERATE_MOVEMENT')
    q0 = [b0[i]-a0[i] for i in range(2)]
    q1 = [b1[i]-a1[i] for i in range(2)]
    u = [q1[i]-q0[i] for i in range(2)]
    scale = max(map(abs, u))
    alpha = 0.0
    if scale:
        direction = [x/scale for x in u]
        d0 = math.fsum(q0[i]*direction[i] for i in range(2))
        d1 = math.fsum(q1[i]*direction[i] for i in range(2))
        if d0 < 0:
            alpha = 1.0 if d1 <= 0 else -d0/(d1-d0)
    q = [math.fsum((q0[i], alpha*u[i])) for i in range(2)]
    return {'clearance_m': math.hypot(*q)-ra-rb, 'alpha': alpha,
            'time_ns': t0+alpha*(t1-t0), 'segment_ns': [t0, t1],
            'relative_position_m': q}


def validated_series(samples, entity_id, frame, epoch):
    out = []
    for item in samples:
        stamp(item['t_ns']); point(item['center_xyz'][:2])
        if type(item['entity_id']) is not int:
            raise ValueError('ENTITY_INTEGER')
        for key, size in [('xyz', 3), ('center_xyz', 3), ('quaternion_xyzw', 4)]:
            if len(item[key]) != size: raise ValueError('POSE_DIMENSION')
            for value in item[key]: number(value)
        if abs(math.hypot(*item['quaternion_xyzw'])-1) > 1e-8:
            raise ValueError('QUATERNION_NORM')
        if item['entity_id'] != entity_id or item['frame'] != frame or item['epoch'] != epoch:
            raise ValueError('ENTITY_FRAME_EPOCH')
        if out and item['t_ns'] < out[-1]['t_ns']:
            raise ValueError('SOURCE_TIME_REVERSED')
        if out and item['t_ns'] == out[-1]['t_ns']:
            if any(item[k] != out[-1][k] for k in ('xyz', 'quaternion_xyzw', 'center_xyz')):
                raise ValueError('CONFLICTING_DUPLICATE')
            out[-1]['duplicate_sources'].append(item['source'])
            continue
        out.append({**item, 'duplicate_sources': list(item.get('duplicate_sources', []))})
    return out


def at(seq, times, t, gap):
    i = bisect.bisect_left(times, t)
    if i < len(times) and times[i] == t:
        return seq[i]['center_xyz'][:2], {'before': i, 'after': i, 'alpha': 0.0}
    if i == 0 or i == len(times):
        return None, 'NO_BRACKET_NO_EXTRAPOLATION'
    if times[i]-times[i-1] > gap:
        return None, 'RAW_GAP_EXCEEDED'
    alpha = (t-times[i-1])/(times[i]-times[i-1])
    return [seq[i-1]['center_xyz'][j]+alpha*(seq[i]['center_xyz'][j]-seq[i-1]['center_xyz'][j]) for j in range(2)], {'before': i-1, 'after': i, 'alpha': alpha}


def evaluate_pair(a, b, interval, gap, frame, epoch):
    lo, hi = interval; stamp(lo); stamp(hi)
    if hi < lo or type(gap) is not int or gap <= 0:
        raise ValueError('INTERVAL_OR_GAP')
    sa = validated_series(a['samples'], a['entity_id'], frame, epoch)
    sb = validated_series(b['samples'], b['entity_id'], frame, epoch)
    ta, tb = [x['t_ns'] for x in sa], [x['t_ns'] for x in sb]
    edges = sorted({lo, hi, *(t for t in ta+tb if lo <= t <= hi)})
    points = []; intervals = []; direct = []; aligned = []
    for t in edges:
        pa, ba = at(sa, ta, t, gap); pb, bb = at(sb, tb, t, gap)
        if pa is None or pb is None:
            points.append(None); continue
        row = {'clearance_m': signed_clearance(pa, pb, a['radius_m'], b['radius_m']),
               'time_ns': t, 'a_xy': pa, 'b_xy': pb, 'a_bracket': ba, 'b_bracket': bb,
               'direct': ba['before'] == ba['after'] and bb['before'] == bb['after']}
        points.append(row); aligned.append(row)
        if row['direct']: direct.append(row)
    minima = []; covered = 0
    for i, (left, right) in enumerate(zip(edges, edges[1:])):
        # Check original brackets at the interior even when both ends are samples.
        ma, ea = at(sa, ta, Fraction(left+right, 2), gap)
        mb, eb = at(sb, tb, Fraction(left+right, 2), gap)
        valid = points[i] is not None and points[i+1] is not None and ma is not None and mb is not None
        row = {'interval_ns': [left, right], 'valid': valid,
               'reason': None if valid else str(ea if ma is None else eb if mb is None else 'ENDPOINT_MISSING'),
               'minimum': None}
        if valid:
            p, q = points[i], points[i+1]
            m = segment_minimum(p['a_xy'], q['a_xy'], p['b_xy'], q['b_xy'], a['radius_m'], b['radius_m'], left, right)
            m['endpoint_brackets'] = [p['a_bracket'], q['a_bracket'], p['b_bracket'], q['b_bracket']]
            row['minimum'] = m; minima.append(m); covered += right-left
        intervals.append(row)
    pick = lambda values: min(values, key=lambda x: (x['clearance_m'], x['time_ns'])) if values else None
    if hi == lo:
        coverage = 'full' if aligned else 'none'; fraction = 1.0 if aligned else 0.0
    else:
        coverage = 'full' if covered == hi-lo else 'partial' if aligned else 'none'
        fraction = covered/(hi-lo)
    return {'pair': [a['name'], b['name']], 'coverage': coverage, 'covered_ns': covered,
            'fraction': fraction, 'measure': 'instant' if hi == lo else 'simulation_duration',
            'sample_min': pick(direct), 'aligned_sample_min': pick(aligned),
            'linear_model_min': pick(minima) if hi != lo else pick(aligned),
            'intervals': intervals, 'aligned_points': aligned,
            'normalized_samples': {a['name']: sa, b['name']: sb}}


def evaluate(data):
    epsilon = number(data['numeric_epsilon_m'])
    if epsilon < 0: raise ValueError('NEGATIVE_EPSILON')
    for obj in data['objects']:
        radius(obj['radius_m'])
        if type(obj['entity_id']) is not int or obj['entity_id'] <= 0:
            raise ValueError('ENTITY_INTEGER')
    objects = {x['name']: x for x in data['objects']}
    if len(objects) != len(data['objects']): raise ValueError('DUPLICATE_OBJECT')
    results = []
    for pair in data['pairs']:
        if len(pair) != 2 or pair[0] == pair[1] or any(n not in objects for n in pair):
            raise ValueError('EXPECTED_OBJECT_MISSING')
        results.append(evaluate_pair(objects[pair[0]], objects[pair[1]], data['interval_ns'], data['max_gap_ns'], data['frame'], data['epoch']))
    if not results: raise ValueError('EMPTY_SCOPE')
    full = all(x['coverage'] == 'full' for x in results)
    coverage = 'full' if full else 'none' if all(x['coverage'] == 'none' for x in results) else 'partial'
    for result in results:
        m = result['linear_model_min']; epsilon = data['numeric_epsilon_m']
        d = None if m is None else m['clearance_m']
        state = 'unknown' if d is None else 'touching_or_uncertain' if abs(d) <= epsilon else 'overlapping' if d < 0 else 'separated'
        result['nominal_class'] = state
        result['geometric_contact'] = True if state == 'overlapping' else False if state == 'separated' else None
        for source in ('sample_min', 'aligned_sample_min'):
            value = result[source]
            d_sample = None if value is None else value['clearance_m']
            result[source+'_geometric_contact'] = None if d_sample is None or abs(d_sample) <= epsilon else d_sample < 0
        result['minimum_scope'] = 'requested_scope' if result['coverage'] == 'full' else 'valid_subset_only'
    return {'coverage': coverage, 'evaluation_valid': full, 'pairs': results,
            'collision': None, 'contact_sensor_event': None,
            'physical_error_aware_contact': None,
            'physical_error_aware_class': 'unknown',
            'event_semantics': 'closest_approach is not first_contact',
            'linear_model_semantics': 'piecewise linear circle centers, not observed continuous physical motion'}
