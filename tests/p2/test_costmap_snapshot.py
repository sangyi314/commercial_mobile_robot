import hashlib

from tools import PROJECT_ROOT
from tools.runner.archive import snapshot_code


def test_p2_worker_snapshot_derives_only_serializer(tmp_path):
    source = (PROJECT_ROOT / "tools/navigation/nav_goal.py").read_bytes()
    p1, p2 = tmp_path / "p1", tmp_path / "p2"
    for run in (p1, p2):
        (run / "artifacts").mkdir(parents=True)
    (p2 / "artifacts/p2-imu-source.json").write_text("{}\n")

    legacy = snapshot_code(p1)
    shadow = snapshot_code(p2)
    path = "tools/navigation/nav_goal.py"
    assert (p1 / legacy["worker"]).read_bytes() == source
    changed = (p2 / shadow["worker"]).read_bytes()
    assert changed.replace(
        b"from rosidl_runtime_py.convert import message_to_ordereddict\n"
        b" from tools.p2.costmap_snapshot import message_to_yaml\n",
        b"from rosidl_runtime_py.convert import message_to_ordereddict,message_to_yaml\n",
    ) == source
    assert changed != source
    assert shadow["files"][path] == hashlib.sha256(changed).hexdigest()
    assert shadow["worker_parent_sha256"] == hashlib.sha256(source).hexdigest()
    compile(changed, "derived_nav_goal.py", "exec")
