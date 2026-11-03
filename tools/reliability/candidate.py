# SPDX-License-Identifier: MIT
"""Freeze the actual worktree, including uncommitted/untracked source; never commits."""
import hashlib,json,subprocess,tarfile,os
from pathlib import Path
from tools import PROJECT_ROOT

def sha(path):
 h=hashlib.sha256()
 with Path(path).open('rb') as f:
  for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
 return h.hexdigest()
def source_manifest(root=PROJECT_ROOT):
 raw=subprocess.check_output(['git','ls-files','--cached','--others','--exclude-standard','-z'],cwd=root).decode().split('\0')
 return [{'path':n,'sha256':sha(root/n),'size':(root/n).stat().st_size} for n in sorted(set(raw)) if n and Path(n).parts[0] not in ['evidence','build','install','log','.git','.cache'] and (root/n).is_file()]
def source_id(rows):return hashlib.sha256(json.dumps(rows,sort_keys=True,separators=(',',':')).encode()).hexdigest()
def freeze(output,prefix=None):
 out=Path(output).resolve();out.mkdir(parents=True,exist_ok=False);rows=source_manifest();identity=source_id(rows)
 (out/'source.patch').write_bytes(subprocess.check_output(['git','diff','--binary'],cwd=PROJECT_ROOT))
 with tarfile.open(out/'source.tar.gz','w:gz') as tar:
  for row in rows:tar.add(PROJECT_ROOT/row['path'],arcname=row['path'],recursive=False)
 commands={}
 for argv in [['git','rev-parse','HEAD'],['git','status','--porcelain=v1'],['c++','--version'],['cmake','--version'],['uname','-a'],['dpkg-query','-W','ros-jazzy-rclcpp','ros-jazzy-nav2-controller','ros-jazzy-nav2-mppi-controller','ros-jazzy-nav2-rotation-shim-controller','ros-jazzy-ros-gz-bridge','libsqlite3-0','ros-jazzy-gz-sim-vendor']]:
  r=subprocess.run(argv,cwd=PROJECT_ROOT,capture_output=True,text=True,timeout=15);commands[' '.join(argv)]={'exit':r.returncode,'stdout':r.stdout,'stderr':r.stderr}
 binaries=[]
 if prefix:
  for pkg,names in {'cmr_mission':['mission_node','fake_nav2_server','map_identity'],'cmr_execution':['execution_gate','execution_observer','candidate_relay']}.items():
   for name in names:
    p=Path(prefix).resolve()/pkg/'lib'/pkg/name
    if not p.is_file():raise ValueError('binary missing '+str(p))
    binaries.append({'path':str(p),'sha256':sha(p)})
 result={'version':1,'candidate_id':identity,'files':rows,'commands':commands,'binaries':binaries,'source_archive_sha256':sha(out/'source.tar.gz'),'scope':'Engineering development candidate; acceptance status is separate','oracle_stop_assisted':True,'sensor_only_E2':False}
 (out/'candidate.json').write_text(json.dumps(result,indent=2));return result
