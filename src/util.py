import time

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose, Transform, TransformStamped, Vector3


def now():
    """Return builtin_interfaces.msg.Time object with the current CPU time"""
    cpu_time = time.time()
    sec = int(cpu_time)
    nanosec = int((cpu_time - sec) * 1000000)
    return Time(sec=sec, nanosec=nanosec)


def duration(t1: Time, t2: Time):
    """Compute duration from t1 to t2, return seconds (float)"""
    return float(t2.sec - t1.sec) + float(t2.nanosec - t1.nanosec) / 1000000.


def pose_to_transform(pose: Pose, stamp: Time, parent_frame: str, child_frame: str):
    """Convert geometry_msgs.msg.Pose to a geometry_msgs.msg.Transform"""
    v = Vector3(x=pose.position.x, y=pose.position.y, z=pose.position.z)
    transform_stamped = TransformStamped()
    transform_stamped.header.stamp = stamp
    transform_stamped.header.frame_id = parent_frame
    transform_stamped.child_frame_id = child_frame
    transform_stamped.transform = Transform(translation=v, rotation=pose.orientation)
    return transform_stamped
