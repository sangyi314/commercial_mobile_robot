"""Accelerate existing stop audits of append-only P2 worker observations."""
from bisect import bisect_left, bisect_right
from operator import itemgetter
import os


_MAX_INDEXED_BODY_SAMPLES = 2048


def indexed_stop_enabled(rows):
    return (os.environ.get("CMR_P2_INDEX_STOP_OBSERVATIONS") == "1"
            and bool(rows) and "receive_monotonic_s" in rows[0])


def live_rows_since(rows, cutoff):
    if not indexed_stop_enabled(rows):
        return rows
    # The worker stamps each appended observation with its monotonic clock.
    left = bisect_left(rows, cutoff, key=itemgetter("receive_monotonic_s"))
    return rows[left:]


class LiveBodyObservations:
    def __init__(self, stamp_ns):
        self._stamp_ns = stamp_ns
        self._source = None

    def covering(self, rows, start_ns, end_ns):
        # The P2 delivery worker only appends. Restart or replacement of the
        # input list starts a new index, including in the closed-bag auditor.
        if (self._source is not rows or self._count > len(rows) or
                (self._count and rows[self._count - 1] is not self._last)):
            self._source = rows
            self._count = 0
            self._samples = []
            self._stamps = []
            self._ordered = True
            self._trimmed = False
        for index in range(self._count, len(rows)):
            row = rows[index]
            if row["kind"] == "body_odom":
                stamp = self._stamp_ns(row["data"]["stamp_s"])
                if self._stamps and stamp < self._stamps[-1]:
                    self._ordered = False
                if len(self._samples) == _MAX_INDEXED_BODY_SAMPLES:
                    del self._samples[:_MAX_INDEXED_BODY_SAMPLES // 2]
                    del self._stamps[:_MAX_INDEXED_BODY_SAMPLES // 2]
                    self._trimmed = True
                self._samples.append(row)
                self._stamps.append(stamp)
        self._count = len(rows)
        self._last = rows[-1] if rows else None

        # Match the existing auditors' exact predecessor and successor.
        # After source regression, leave as-received order to the auditors.
        if (not self._ordered or start_ns >= end_ns or
                (self._trimmed and start_ns < self._stamps[1])):
            return rows
        left = bisect_right(self._stamps, start_ns) - 1
        right = bisect_left(self._stamps, end_ns)
        if left < 0 or right == len(self._samples):
            return []
        return self._samples[max(0, left - 1):right + 1]
