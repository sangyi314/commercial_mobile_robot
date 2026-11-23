"""Versioned source-package snapshots for owned subprocesses; no processes."""
from hashlib import sha256
from pathlib import Path
import json
from tools import PROJECT_ROOT
from tools.runner.runner_core import atomic_write_new, digest

def snapshot_code(run):
    """Copy executable package and its schema closure into a new run."""
    run = Path(run)
    sources = []
    for folder, pattern in [('tools', '*.py'), ('tools', '*.bash'),
                            ('experiments/schemas', '*.json'),
                            ('src/cmr_bringup/config', '*.yaml'),
                            ('src/cmr_bringup/config', '*.xml')]:
        sources.extend(p for p in (PROJECT_ROOT/folder).rglob(pattern)
                       if '__pycache__' not in p.parts)
    files = {}
    worker = Path('tools/navigation/nav_goal.py')
    derived_worker = (run/'artifacts/p2-imu-source.json').is_file()
    for source in sorted(set(sources)):
        relative = source.relative_to(PROJECT_ROOT)
        if source.is_symlink():
            raise ValueError('SNAPSHOT_SOURCE_SYMLINK')
        target = run/'artifacts'/relative
        target.parent.mkdir(parents=True, exist_ok=True)
        data = source.read_bytes()
        if derived_worker and relative == worker:
            from tools.p2.costmap_snapshot import derive_worker
            worker_parent_sha256 = sha256(data).hexdigest()
            data = derive_worker(data)
        atomic_write_new(target, data)
        files[str(relative)] = digest(target)
    layout = {'layout_version': 2, 'package_root': 'artifacts/tools',
              'schema_root': 'artifacts/experiments/schemas',
              'config_root': 'artifacts/src/cmr_bringup/config',
              'worker': 'artifacts/tools/navigation/nav_goal.py',
              'bag_inspector': 'artifacts/tools/runner/inspect_bag.py',
              'dynamic_inspector': 'artifacts/tools/dynamic/inspect_dynamic_recording.py',
              'gui_probe': 'artifacts/tools/runner/gui_scene_probe.py',
              'files': files}
    if derived_worker:
        layout['worker_parent_sha256'] = worker_parent_sha256
    atomic_write_new(run/'artifacts/code-layout.json',
                     (json.dumps(layout, indent=2, allow_nan=False)+'\n').encode())
    return layout
