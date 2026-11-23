"""Accelerated full Costmap snapshots for the derived P2 worker."""
import json


def derive_worker(source):
    """Keep the reviewed worker intact apart from its artifact serializer."""
    original = b"from rosidl_runtime_py.convert import message_to_ordereddict,message_to_yaml\n"
    replacement = (b"from rosidl_runtime_py.convert import message_to_ordereddict\n"
                   b" from tools.p2.costmap_snapshot import message_to_yaml\n")
    if source.count(original) != 1:
        raise ValueError("P2 worker serializer import changed")
    return source.replace(original, replacement)


def message_to_yaml(message, **options):
    from nav2_msgs.msg import Costmap
    from rosidl_runtime_py.convert import message_to_ordereddict, message_to_yaml as ros_yaml

    if not isinstance(message, Costmap) or options:
        return ros_yaml(message, **options)
    # JSON is valid YAML. Keep metadata and every cell under its original key.
    return json.dumps(message_to_ordereddict(message), ensure_ascii=False, allow_nan=False) + "\n"
