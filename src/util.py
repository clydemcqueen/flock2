import time

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose, Transform, TransformStamped, Vector3


def now() -> Time:
    """Return builtin_interfaces.msg.Time object with the current CPU time"""
    cpu_time = time.time()
    sec = int(cpu_time)
    nanosec = int((cpu_time - sec) * 1.e9)
    return Time(sec=sec, nanosec=nanosec)


def duration(t1: Time, t2: Time) -> float:
    """Compute duration from t1 to t2, return seconds"""
    return float(t2.sec - t1.sec) + float(t2.nanosec - t1.nanosec) / 1.e9


def copy_pose_to_transform(pose: Pose, transform: Transform):
    """Copy fields from geometry_msgs.msg.Pose to a geometry_msgs.msg.Transform"""
    transform.translation.x = pose.position.x
    transform.translation.y = pose.position.y
    transform.translation.z = pose.position.z
    transform.rotation.x = pose.orientation.x
    transform.rotation.y = pose.orientation.y
    transform.rotation.z = pose.orientation.z
    transform.rotation.w = pose.orientation.w


def copy_pose_to_pose(pose1: Pose, pose2: Pose):
    """Copy fields from one geometry_msgs.msg.Pose to another"""
    pose2.position.x = pose1.position.x
    pose2.position.y = pose1.position.y
    pose2.position.z = pose1.position.z
    pose2.orientation.x = pose1.orientation.x
    pose2.orientation.y = pose1.orientation.y
    pose2.orientation.z = pose1.orientation.z
    pose2.orientation.w = pose1.orientation.w
