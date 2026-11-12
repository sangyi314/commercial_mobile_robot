from tools.delivery.inspect_delivery_bag import body_receipt_crosscheck, body_window
from tools.p2.stop_observations import live_rows_since


def test_index_matches_original_for_gaps_restart_and_reordered_stamps(monkeypatch):
    rows = []
    for i in range(2300):
        stamp = i / 10
        data = dict(stamp_s=stamp, position=[0, 0, 0], quaternion_xyzw=[0, 0, 0, 1],
                    linear=[0, 0, 0], angular=[0, 0, 0])
        rows.append(dict(kind="body_odom", data=data, line=i + 1,
                         receive_monotonic_s=stamp + (0.3 if i >= 100 else 0)))

    live = rows[:90]
    report = dict(data=dict(from_ros_ns=3_000_000_000, to_ros_ns=6_050_000_000,
                            reason="WINDOW_PROVEN_BODY_POSE"), line=131)

    def collect(sequence):
        low, high = report["data"]["from_ros_ns"], report["data"]["to_ros_ns"]
        return body_window(sequence, report), body_receipt_crosscheck(sequence, low, high)

    def compare(sequence, expected):
        monkeypatch.delenv("CMR_P2_INDEX_STOP_OBSERVATIONS", raising=False)
        original = collect(sequence)
        monkeypatch.setenv("CMR_P2_INDEX_STOP_OBSERVATIONS", "1")
        assert collect(sequence) == original
        assert original[1]["valid"] is expected

    compare(live, True)
    assert live_rows_since(live, 3.0)[0] is live[30]
    live.extend(rows[90:130])
    report["data"].update(from_ros_ns=8_000_000_000, to_ros_ns=11_050_000_000)
    compare(live, False)
    report["data"].update(from_ros_ns=3_000_000_000, to_ros_ns=6_050_000_000)
    compare(live[:90], True)
    live.extend(rows[130:])
    compare(live, True)  # The required boundary has left the bounded cache.
    live.append(dict(live[-1], data=dict(live[-1]["data"], stamp_s=4.0), line=131))
    compare(live, True)
