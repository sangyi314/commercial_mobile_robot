#!/usr/bin/env python3
"""Prepare one concrete spec using archived inputs and the existing input checks.

stdout remains the output path; stderr contains a JSON preparation review.
This does not run the distance evaluator or create an evaluation report.
"""
import argparse
import json
from pathlib import Path
import sys
sys.dont_write_bytecode = True
from tools.geometry.geometry_inputs import ROOT, adapt, prepare, schema, verify_sources


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run-dir', required=True, type=Path)
    p.add_argument('--output', required=True, type=Path)
    a = p.parse_args(argv)
    try:
        run = a.run_dir.resolve(strict=True)
        output = a.output.resolve()
        if output.is_relative_to(run) or output.is_relative_to(ROOT/'evidence/runs'):
            raise ValueError('OUTPUT_IN_SOURCE')
        if any(x.is_symlink() for x in [a.output, *a.output.parents]):
            raise ValueError('OUTPUT_SYMLINK')
        if a.output.exists(): raise FileExistsError('OUTPUT_EXISTS')
        if not a.output.parent.is_dir(): raise ValueError('OUTPUT_PARENT_MISSING')
        data = prepare(run)
        schema(data, 'evaluation_spec')
        # Reuse all production input checks, including record validation,
        # archived geometry, actual link containment, raw identity and clocks.
        inputs, audit = adapt(run, data)
        review = {
            'status': 'input_adapter_validated',
            'output': str(output), 'run_id': data['run_id'],
            'record_origin': data['record_origin'], 'epoch': data['epoch'],
            'frame': data['frame'], 'scope': data['scope'],
            'scene_identity': data['scene_identity'],
            'source_hashes': data['source_hashes'],
            'geometry': {
                'robot_resource': 'artifacts/robot.sdf',
                'actor_resource': 'artifacts/dynamic_actor.sdf',
                'robot_radius_rule': data['robot_radius_rule'],
                'robot_radius_m': audit['geometry']['robot_radius_m'],
                'actor_radius_m': audit['geometry']['actor_radius_m']},
            'clock_alignment': audit['clock_alignment'],
            'max_gap_ns': data['max_gap_ns'],
            'physical_error_bound_m': data['physical_error_bound_m'],
            'samples': {o['name']: len(o['samples']) for o in inputs['objects']},
            'unknowns': audit['unknowns'],
            'next_step': 'evaluate_geometry.py --run-dir RUN --spec OUTPUT --check',
            'coverage': 'not computed during preparation; check separately',
            'catalog_updated': False}
        payload = json.dumps(data, indent=2, allow_nan=False)+'\n'
        explanation = json.dumps(review, ensure_ascii=False, allow_nan=False)
        verify_sources(run, data)
        with a.output.open('x') as f: f.write(payload)
        print(output)
        print(explanation, file=sys.stderr)
        return 0
    except Exception as e:
        print(type(e).__name__+': '+str(e), file=sys.stderr)
        return 2
