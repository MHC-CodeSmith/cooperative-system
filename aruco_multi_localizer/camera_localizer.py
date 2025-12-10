import numpy as np
import tf_transformations as tf

def estimate_camera_pose(marker_id, rvec, tvec, marker_global_poses):
    """
    Returns camera pose (R_wc, t_wc) given a detected marker
    """
    R_marker_cam, _ = cv2.Rodrigues(rvec)
    t_marker_cam = tvec.reshape(3,)

    # Invert to camera->marker
    R_cam_marker = R_marker_cam.T
    t_cam_marker = -R_marker_cam.T @ t_marker_cam

    marker_pos, marker_rpy = marker_global_poses[marker_id]
    R_world_marker = tf.euler_matrix(*marker_rpy)[:3, :3]
    t_world_marker = np.array(marker_pos)

    # World->camera
    R_world_cam = R_world_marker @ R_cam_marker
    t_world_cam = t_world_marker + R_world_marker @ t_cam_marker

    return R_world_cam, t_world_cam
