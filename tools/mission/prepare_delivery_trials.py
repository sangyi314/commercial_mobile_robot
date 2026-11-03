#!/usr/bin/env python3
"""Prepare new P1-04 derived inputs offline; no Gazebo/ROS Node is started."""
if __package__ in (None,''):
 from pathlib import Path
 from runpy import run_path
 run_path(str(Path(__file__).resolve().parents[1]/'__init__.py'))['bootstrap']()
import argparse,json
from tools.delivery.profile import prepare
if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--base-profile',required=True);p.add_argument('--install-prefix',required=True);p.add_argument('--output-dir',required=True);p.add_argument('--suffix',required=True);a=p.parse_args();print(json.dumps(prepare(a.base_profile,a.install_prefix,a.output_dir,a.suffix),indent=2))
