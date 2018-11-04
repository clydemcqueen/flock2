from geometry_msgs.msg import Point, Pose, Quaternion

import cv2
import numpy as np
import transformations as xf


# Transformation notation:
# Tst === T_source_target
# vector_target = T_source_target * vector_source

# From transformations.py: "Quaternions w+ix+jy+kz are represented as [w, x, y, z]."


def rvec_and_tvec_to_matrix(rvec, tvec) -> np.ndarray:
    """Rodrigues rotation and translation vector to 4x4 matrix"""
    t_matrix = xf.translation_matrix(tvec)
    R, _ = cv2.Rodrigues(rvec)
    r_matrix = xf.identity_matrix()
    r_matrix[:3, :3] = R
    return np.dot(t_matrix, r_matrix)


def pose_to_matrix(p: Pose) -> np.ndarray:
    """geometry_msgs.msg.Pose to 4x4 matrix"""
    t_matrix = xf.translation_matrix([p.position.x, p.position.y, p.position.z])
    r_matrix = xf.quaternion_matrix([p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z])
    return np.dot(t_matrix, r_matrix)


def matrix_to_pose(m: np.ndarray) -> Pose:
    """4x4 matrix to geometry_msgs.msg.Pose"""
    t = xf.translation_from_matrix(m)
    q = xf.quaternion_from_matrix(m)  # Order is [w, x, y, z]
    return Pose(position=Point(x=t[0], y=t[1], z=t[2]), orientation=Quaternion(x=q[1], y=q[2], z=q[3], w=q[0]))


class DetectArUco(object):

    def __init__(self, camera_matrix, distortion):
        """Initialize the detector.

        Args:
            camera_matrix: np.array(3, 3)
            distortion: np.array(5)
        """

        self._camera_matrix = camera_matrix
        self._distortion = distortion

        # ArUco data -- we're using 6x6 ArUco images
        self._aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        self._aruco_parameters = cv2.aruco.DetectorParameters_create()

        # Markers are 18cm x 18cm
        self._marker_length = 0.18

        # ID of the first marker we see
        self._first_marker_id = -1

        # First marker pose
        self._first_marker_pose = Pose(
            position=Point(x=0., y=0., z=1.),
            orientation=Quaternion(x=0.5, y=-0.5, z=-0.5, w=0.5))
        self._Tom = xf.inverse_matrix(pose_to_matrix(self._first_marker_pose))

        # Tcb transform
        self._Tcb = pose_to_matrix(Pose(
            position=Point(x=0.035, y=0., z=0.),
            orientation=Quaternion(x=-0.5, y=0.5, z=-0.5, w=0.5)))

    def detect(self, logger, color_mat):
        """Find all ArUco markers in an image.

        Args:
            logger: ROS logger
            color_mat: color cv2.Mat

        Returns:
            Image (cv2.Mat) with the markers drawn in green
            Drone pose (geometry_msgs.msg.Pose); None if we can't compute the pose
            A dictionary of marker poses (geometry_msgs.msg.Pose), keyed by marker id; {} if we can't compute the poses
        """

        # Color => gray for detection
        gray_mat = cv2.cvtColor(color_mat, cv2.COLOR_BGR2GRAY)

        # Detect markers
        corners, ids, _ = cv2.aruco.detectMarkers(gray_mat, self._aruco_dict, parameters=self._aruco_parameters)

        # Stop if no markers were detected
        if ids is None:
            return color_mat, None, None

        # Draw borders on the color image
        color_mat = cv2.aruco.drawDetectedMarkers(color_mat, corners)

        if self._first_marker_id < 0:
            # Grab the first marker we see
            self._first_marker_id = ids[0][0]
            logger.info('first marker has id %d' % self._first_marker_id)
        else:
            # Stop if the first marker wasn't detected
            found_first_marker = False
            for index in range(len(ids)):
                if ids[index][0] == self._first_marker_id:
                    found_first_marker = True
                    break
            if not found_first_marker:
                return color_mat, None, None

        # Compute transformations, each is marker_frame => camera_frame
        rvecs, tvecs, object_points = cv2.aruco.estimatePoseSingleMarkers(corners, self._marker_length,
                                                                          self._camera_matrix, self._distortion)

        # Compute the drone pose
        drone_pose = None
        Tbo = xf.identity_matrix()
        for index in range(len(ids)):
            if ids[index][0] == self._first_marker_id:
                # Tob = Tcb * Tmc * Tom
                Tmc = rvec_and_tvec_to_matrix(rvecs[index][0], tvecs[index][0])
                Tob = self._Tcb.dot(Tmc).dot(self._Tom)

                # sendTransform expects child=target and parent=source
                # We can't flip source and target because we want odom to be the parent of base_link
                # Instead, invert Tob to get Tbo
                Tbo = xf.inverse_matrix(Tob)
                drone_pose = matrix_to_pose(Tbo)
                break

        # Compute the marker poses
        marker_poses = {}
        for index in range(len(ids)):
            if ids[index][0] == self._first_marker_id:
                # This one is easy
                marker_poses[int(ids[index][0])] = self._first_marker_pose
            else:
                # Compute the pose of this marker
                # Tmo = Tbo * Tcb * Tmc
                Tmc = rvec_and_tvec_to_matrix(rvecs[index][0], tvecs[index][0])
                Tmo = Tbo.dot(self._Tcb).dot(Tmc)
                marker_poses[int(ids[index][0])] = matrix_to_pose(Tmo)

        return color_mat, drone_pose, marker_poses
