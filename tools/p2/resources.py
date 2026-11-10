"""One-Hz observations of this Runner's owned processes, without controlling them."""
import json
import os
from pathlib import Path
import time


class ResourceSampler:
    def __init__(self, run, supervisor, gazebo):
        self.output = run / "runtime/p2-resources.jsonl"
        self.output.open("x").close()
        self.prefix = ("CMR_OWNERSHIP_TOKEN=" + supervisor.token + ":").encode()
        self.gazebo = gazebo
        self.next_sample = 0
        self.hz = os.sysconf("SC_CLK_TCK")
        self.page = os.sysconf("SC_PAGE_SIZE")

    def tick(self):
        now = time.monotonic_ns()
        if now < self.next_sample:
            return
        self.next_sample = now + 1_000_000_000
        rows = []
        for path in Path("/proc").iterdir():
            if not path.name.isdigit():
                continue
            try:
                if path.stat().st_uid != os.getuid():
                    continue
                owner = next((v for v in (path / "environ").read_bytes().split(b"\0")
                              if v.startswith(self.prefix)), None)
                if owner is None:
                    continue
                stat = (path / "stat").read_text().rsplit(") ", 1)[1].split()
                rows.append({"pid": int(path.name), "start_ticks": int(stat[19]),
                             "role": owner[len(self.prefix):].decode(),
                             "cpu_seconds": (int(stat[11]) + int(stat[12])) / self.hz,
                             "rss_bytes": int(stat[21]) * self.page,
                             "threads": int(stat[17])})
            except (OSError, ValueError, IndexError):
                continue
        stats = self.gazebo.latest.get("stats", {}).get("protobuf", {})
        with self.output.open("a") as stream:
            stream.write(json.dumps({"steady_ns": now, "simulation_stats": stats,
                                     "processes": rows}) + "\n")
