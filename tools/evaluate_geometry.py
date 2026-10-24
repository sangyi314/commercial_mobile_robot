#!/usr/bin/env python3
"""Offline circle evaluation. --check does not create directories or output files."""
import argparse
import json
import os
from pathlib import Path
import sys
sys.dont_write_bytecode = True
from geometry_inputs import ROOT, adapt, read, schema, sha, verify_sources
from geometry_metrics import evaluate
from jsonschema.exceptions import ValidationError


def output_guard(path, protected):
    if not path.is_absolute(): raise ValueError('ABSOLUTE_OUTPUT_REQUIRED')
    if path.exists() or path.is_symlink(): raise ValueError('OUTPUT_ALREADY_EXISTS')
    if any(x.is_symlink() for x in path.parents): raise ValueError('OUTPUT_SYMLINK')
    resolved = path.resolve()
    if any(resolved.is_relative_to(p.resolve()) for p in protected): raise ValueError('OUTPUT_IN_SOURCE')


def atomic(path, value):
    temp = path.with_name(path.name+'.tmp')
    with temp.open('x') as f:
        json.dump(value, f, ensure_ascii=False, indent=2, allow_nan=False)
        f.write('\n'); f.flush(); os.fsync(f.fileno())
    os.replace(temp, path)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, epilog='Exit codes: 0 full valid scope (including overlap); 2 invalid format/identity/contract; 3 missing required input or no usable data; 4 partial coverage; 1 internal error. Collision stays unknown.')
    p.add_argument('--run-dir', type=Path); p.add_argument('--spec', type=Path)
    p.add_argument('--fixture-input', type=Path); p.add_argument('--allow-fixtures', action='store_true')
    group = p.add_mutually_exclusive_group(required=True)
    group.add_argument('--check', action='store_true'); group.add_argument('--output-dir', type=Path)
    a = p.parse_args(argv); created = False
    try:
        if a.fixture_input:
            if not a.allow_fixtures or a.run_dir or a.spec: raise ValueError('EXPLICIT_FIXTURE_ONLY')
            data = read(a.fixture_input); schema(data, 'evaluation_input')
            if data['record_origin'] != 'fixture': raise ValueError('FIXTURE_ORIGIN')
            spec = {'fixture_sha256': sha(a.fixture_input)}; audit = {'fixture_only': True}
            protected = [a.fixture_input.parent]
        else:
            if not a.run_dir or not a.spec or a.allow_fixtures: raise ValueError('REAL_RUN_AND_SPEC_REQUIRED')
            spec = read(a.spec); data, audit = adapt(a.run_dir, spec)
            protected = [a.run_dir]
        result = evaluate(data)
        code = {'full': 0, 'partial': 4, 'none': 3}[result['coverage']]
        if a.check:
            print(json.dumps({'precheck': 'valid', 'coverage': result['coverage'], 'exit_code': code,
                              'scope_ns': data['interval_ns'], 'objects': [{'name': x['name'], 'radius_m': x['radius_m'], 'samples': len(x['samples'])} for x in data['objects']]}))
            return code
        out = a.output_dir; output_guard(out, protected)
        out.parent.mkdir(parents=True, exist_ok=True); out.mkdir(); created = True
        atomic(out/'spec.json', spec); atomic(out/'input.json', data); atomic(out/'sources.json', audit)
        atomic(out/'intervals.json', [{'pair': x['pair'], 'intervals': x['intervals']} for x in result['pairs']])
        implementation = {str(path.relative_to(ROOT)): sha(path) for path in sorted((ROOT/'tools').glob('*geometry*.py'))}
        implementation.update({str(path.relative_to(ROOT)): sha(path) for path in sorted((ROOT/'experiments/schemas').glob('evaluation*.json'))})
        report = {'schema_version': 1, 'run_id': data['run_id'], 'record_origin': data['record_origin'],
                  'evaluation_id': out.name, 'spec_sha256': sha(out/'spec.json'), 'input_sha256': sha(out/'input.json'),
                  'scope_ns': data['interval_ns'], 'implementation': implementation, **result}
        report['interpretation'] = {
            'units': 'metres, integer source nanoseconds; no unit inference',
            'frame': data['frame'], 'clock_epoch': data['epoch'],
            'geometry': 'robot collision 3D bounding ball projected to XY; upright cylinder XY disk' if a.run_dir else 'explicit synthetic circle fixture',
            'containment_scope': 'checked at observed actual poses only; intersample containment not certified' if a.run_dir else 'synthetic',
            'height_envelopes_world_m': audit.get('sampled_height_envelopes_world_m', {}),
            'max_observed_robot_tilt_rad': audit.get('max_observed_robot_tilt_rad'),
            'interpolation': 'piecewise linear center model; pose-service jumps/holds and physical response are not reconstructed',
            'extrapolation_allowed': False, 'scope_excludes': ['other entities', 'unrecorded intervals', 'global collision', 'physical safety'],
            'numeric_epsilon_m': data['numeric_epsilon_m'],
            'position_error_bound_m': None, 'radius_error_bound_m': None,
            'source_time_error_bound_ns': None, 'interpolation_error_bound_m': None,
            'physical_clearance_band_m': None}
        report['artifacts'] = {name: sha(out/name) for name in ['spec.json','input.json','sources.json','intervals.json']}
        schema(report, 'evaluation_report')
        if a.run_dir: verify_sources(a.run_dir.resolve(), spec)
        atomic(out/'report.json', report)
        print(json.dumps({'report': str(out/'report.json'), 'coverage': result['coverage'], 'exit_code': code})); return code
    except FileNotFoundError as e:
        error = {'error_type': type(e).__name__, 'diagnostic': str(e), 'exit_code': 3}
        if created: atomic(a.output_dir/'failure.json', error)
        print(json.dumps(error), file=sys.stderr); return 3
    except (ValueError, KeyError, TypeError, OSError, ValidationError) as e:
        error = {'error_type': type(e).__name__, 'diagnostic': str(e), 'exit_code': 2}
        if created: atomic(a.output_dir/'failure.json', error)
        print(json.dumps(error), file=sys.stderr); return 2
    except Exception as e:
        # Schema and unexpected internal failures never become a successful report.
        error = {'error_type': type(e).__name__, 'diagnostic': str(e), 'exit_code': 1}
        if created: atomic(a.output_dir/'failure.json', error)
        print(json.dumps(error), file=sys.stderr); return 1


if __name__ == '__main__': raise SystemExit(main())
