"""P2-01 offline audits and explicitly isolated ROS protocol tests."""
import csv
from pathlib import Path
import subprocess
import sys

import pytest


@pytest.mark.p2_case("P201.DeterministicAudit")
def test_deterministic_audit(p2_manifest):
    import rosbag2_py
    from rclpy.serialization import deserialize_message, serialize_message
    from sensor_msgs.msg import LaserScan
    from tools.p2.audit_bag import digest, inspect_files, open_reader

    parent = Path(p2_manifest["evidence"]) / "tests/P201.DeterministicAudit"
    parent.mkdir(parents=True, exist_ok=True)
    for number in range(1, 10000):
        attempt = parent / f"attempt-{number:03}"
        try:
            attempt.mkdir(exist_ok=False)
            break
        except FileExistsError:
            continue
    else:
        pytest.fail("audit attempt namespace exhausted")

    def run_audit(bag, output):
        result = subprocess.run(
            [sys.executable, "tools/p2/audit_bag.py", "--bag", str(bag),
             "--output", str(output), "--recorded-clock", "simulation"],
            capture_output=True, text=True, timeout=60, check=False)
        (attempt / (output.name + ".log")).write_text(result.stdout + result.stderr)
        assert result.returncode == 0

    inputs = p2_manifest["audit_inputs"]
    assert len(inputs) == 2
    for number, source in enumerate(inputs):
        bag = Path(source["bag"])
        assert digest(bag / "bag_0.mcap") == source["sha256"]
        before = inspect_files(bag)[1]
        first, second = (attempt / f"bag-{number}-{i}" for i in (1, 2))
        for output in (first, second):
            run_audit(bag, output)
        for name in ("audit.json", "samples.csv"):
            assert (first / name).read_bytes() == (second / name).read_bytes()
        assert inspect_files(bag)[1] == before

    # Use a new, explicitly synthetic copy with one corrupt stamp and no static TF.
    source = Path(inputs[0]["bag"])
    fixture = attempt / "fixture"
    reader = open_reader(source)
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/clock", "/scan", "/tf"]))
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(fixture), storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    for entry in reader.get_all_topics_and_types():
        if entry.name in ("/clock", "/scan", "/tf"):
            writer.create_topic(entry)
    changed = False
    while reader.has_next():
        name, raw, bag_ns = reader.read_next()
        if name == "/scan" and not changed:
            scan = deserialize_message(raw, LaserScan)
            scan.header.stamp.sec += 2
            raw = serialize_message(scan)
            changed = True
        writer.write(name, raw, bag_ns)
    del writer
    assert changed
    run_audit(fixture, attempt / "fixture-audit")
    with (attempt / "fixture-audit/samples.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    assert any("FUTURE_STAMP" in row["issues"] for row in rows)
    assert all("TF_UNAVAILABLE" in row["issues"] for row in rows)
