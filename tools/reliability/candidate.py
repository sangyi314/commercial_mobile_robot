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
def runtime_manifest(root=PROJECT_ROOT):
 # Pure filesystem inventory for --check: no Git command or subprocess.
 files=[]
 for folder in ['src','tools','tests']:
  for p in (root/folder).rglob('*'):
   if p.is_file() and '__pycache__' not in p.parts and p.suffix not in ['.pyc','.pyo'] and p.name!='.DS_Store':
    files.append({'path':str(p.relative_to(root)),'sha256':sha(p)})
 return sorted(files,key=lambda x:x['path'])
def source_id(rows):return hashlib.sha256(json.dumps(rows,sort_keys=True,separators=(',',':')).encode()).hexdigest()
def candidate_identity(source,binaries,external):
 value={'source_id':source,'binaries':binaries,'external_files':external}
 return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':')).encode()).hexdigest()
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
 external=set()
 for pattern in ['lib/*rosbag2*.so*','lib/*mppi*.so*','lib/*rotation_shim*.so*','lib/*collision_monitor*.so*','opt/gz_sim_vendor/lib/*diff-drive-system.so*','opt/gz_sim_vendor/lib/*odometry-publisher-system.so*']:
  external.update(p.resolve() for p in Path('/opt/ros/jazzy').glob(pattern) if p.is_file())
 for pkg,name in [('nav2_controller','controller_server'),('nav2_planner','planner_server'),('nav2_velocity_smoother','velocity_smoother'),('nav2_collision_monitor','collision_monitor'),('nav2_bt_navigator','bt_navigator'),('ros_gz_bridge','parameter_bridge')]:
  path=Path('/opt/ros/jazzy/lib')/pkg/name
  if path.is_file():external.add(path.resolve())
 for ref in binaries:
  proc=subprocess.run(['ldd',ref['path']],capture_output=True,text=True,timeout=10)
  if proc.returncode:raise ValueError('dynamic dependency inventory failed')
  for line in proc.stdout.splitlines():
   if '=>' in line:
    value=line.split('=>',1)[1].strip().split()[0]
    if value.startswith('/') and Path(value).is_file():external.add(Path(value).resolve())
 external=[{'path':str(path),'sha256':sha(path)} for path in sorted(external)]
 result={'version':1,'source_id':identity,'candidate_id':candidate_identity(identity,binaries,external),'external_files':external,'files':rows,'commands':commands,'binaries':binaries,'source_archive_sha256':sha(out/'source.tar.gz'),'scope':'Engineering development candidate; acceptance status is separate','oracle_stop_assisted':True,'sensor_only_E2':False}
 (out/'candidate.json').write_text(json.dumps(result,indent=2));return result
