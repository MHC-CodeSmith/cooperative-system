from geometry_msgs.msg import PoseStamped

def publish_path(node, R_wc, t_wc, path):
    import tf_transformations as tf
    quat = tf.quaternion_from_matrix(
        np.vstack((np.hstack((R_wc, t_wc.reshape(3,1))), [0,0,0,1]))
    )

    pose = PoseStamped()
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.header.frame_id = "world"
    pose.pose.position.x = t_wc[0]
    pose.pose.position.y = t_wc[1]
    pose.pose.position.z = t_wc[2]
    pose.pose.orientation.x = quat[0]
    pose.pose.orientation.y = quat[1]
    pose.pose.orientation.z = quat[2]
    pose.pose.orientation.w = quat[3]

    path.header.stamp = pose.header.stamp
    path.header.frame_id = "world"
    path.poses.append(pose)
    node.path_pub.publish(path)
