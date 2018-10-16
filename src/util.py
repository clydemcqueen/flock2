import time

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Transform, TransformStamped, Vector3


def now():
    cpu_time = time.time()
    sec = int(cpu_time)
    nanosec = int((cpu_time - sec) * 1000000)
    return Time(sec=sec, nanosec=nanosec)


def pose_to_transform(pose, stamp, parent_frame, child_frame):
    v = Vector3(x=pose.position.x, y=pose.position.y, z=pose.position.z)
    transform_stamped = TransformStamped()
    transform_stamped.header.stamp = stamp
    transform_stamped.header.frame_id = parent_frame
    transform_stamped.child_frame_id = child_frame
    transform_stamped.transform = Transform(translation=v, rotation=pose.orientation)
    return transform_stamped
