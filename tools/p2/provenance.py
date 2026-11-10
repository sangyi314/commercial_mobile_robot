"""Bind qualifications to actual source contents, including pre-existing dirty files."""
from hashlib import sha256
from pathlib import Path
import subprocess


def source_inventory():
    names = subprocess.check_output(
        ["git", "ls-files", "-z", "--cached", "--others", "--exclude-standard"]).split(b"\0")
    rows = {}
    for raw in names:
        if not raw:
            continue
        name = raw.decode()
        if not (name.startswith(("src/", "tools/", "tests/")) or name == "pytest.ini"):
            continue
        path = Path(name)
        if path.is_file() and path.suffix != ".pyc" and "__pycache__" not in path.parts:
            rows[name] = sha256(path.read_bytes()).hexdigest()
    if not rows:
        raise ValueError("empty source inventory")
    return dict(sorted(rows.items()))


def source_fingerprint():
    rows = source_inventory()
    return sha256("".join(name + "\0" + digest + "\n" for name, digest in rows.items()).encode()).hexdigest()
