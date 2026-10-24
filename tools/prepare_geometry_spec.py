#!/usr/bin/env python3
"""Freeze identities and a predeclared interval; never select a distance minimum."""
import argparse
import json
from pathlib import Path
import sys
sys.dont_write_bytecode = True
from geometry_inputs import prepare, schema

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run-dir', required=True, type=Path)
    p.add_argument('--output', required=True, type=Path)
    a = p.parse_args()
    try:
        if a.output.resolve().is_relative_to(a.run_dir.resolve()): raise ValueError('OUTPUT_IN_SOURCE')
        if any(x.is_symlink() for x in [a.output, *a.output.parents]): raise ValueError('OUTPUT_SYMLINK')
        data = prepare(a.run_dir); schema(data, 'evaluation_spec')
        with a.output.open('x') as f: f.write(json.dumps(data, indent=2, allow_nan=False)+'\n')
        print(a.output.resolve()); return 0
    except Exception as e:
        print(type(e).__name__+': '+str(e), file=sys.stderr); return 2

if __name__ == '__main__': raise SystemExit(main())
