# coordinate_transform.py

import numpy as np
from geometry_msgs.msg import Pose

def bbox_center(bbox):
    """
    Get center (u, v) pixel from a bounding box.
    """
    x_min, y_min, x_max, y_max = bbox
    u = int((x_min + x_max) / 2)
    v = int((y_min + y_max) / 2)
    return u, v

def pixel_to_camera_frame(u, v, d, intrinsics):
    """
    Project 2D pixel (u,v) and depth d into 3D camera frame coordinates.
    """
    fx = intrinsics['fx']
    fy = intrinsics['fy']
    cx = intrinsics['cx']
    cy = intrinsics['cy']
    depth_scale = intrinsics.get('depth_scale', 1.0)

    z = d * depth_scale
    if z <= 0.001:
        return None
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    return np.array([x, y, z])

def quaternion_to_rotation_matrix(q):
    """
    Convert a ROS2 Quaternion to a 3×3 rotation matrix.
    """
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2*(y**2 + z**2),     2*(x*y - z*w),       2*(x*z + y*w)],
        [2*(x*y + z*w),           1 - 2*(x**2 + z**2), 2*(y*z - x*w)],
        [2*(x*z - y*w),           2*(y*z + x*w),       1 - 2*(x**2 + y**2)]
    ])
    return R

def transform_camera_to_world(p_cam, pose: Pose):
    """
    Transform point from camera frame to world frame using ROS2 Pose.
    """
    R = quaternion_to_rotation_matrix(pose.orientation)
    t = np.array([pose.position.x, pose.position.y, pose.position.z])
    return R @ p_cam + t

def project_bbox_to_3d(bbox, camera_pose: Pose, intrinsics, depth_image=None):
    """
    Main function: Convert bbox to 3D world coordinates.
    If depth image is not available, fallback to default depth.
    """
    u, v = bbox_center(bbox)

    try:
        d = depth_image[v, u] if depth_image is not None else 1.5
        if d == 0:
            d = 1.5
    except:
        d = 1.5

    p_cam = pixel_to_camera_frame(u, v, d, intrinsics)
    if p_cam is None:
        return (0.0, 0.0, 0.0)

    p_world = transform_camera_to_world(p_cam, camera_pose)
    return tuple(p_world)
