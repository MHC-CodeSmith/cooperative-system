from geometry_msgs.msg import TransformStamped
import tf_transformations as tf

def publish_tf(node, R_wc, t_wc):
    quat = tf.quaternion_from_matrix(
        np.vstack((np.hstack((R_wc, t_wc.reshape(3,1))), [0,0,0,1]))
    )
    tf_msg = TransformStamped()
    tf_msg.header.stamp = node.get_clock().now().to_msg()
    tf_msg.header.frame_id = "world"
    tf_msg.child_frame_id = "camera"
    tf_msg.transform.translation.x = t_wc[0]
    tf_msg.transform.translation.y = t_wc[1]
    tf_msg.transform.translation.z = t_wc[2]
    tf_msg.transform.rotation.x = quat[0]
    tf_msg.transform.rotation.y = quat[1]
    tf_msg.transform.rotation.z = quat[2]
    tf_msg.transform.rotation.w = quat[3]

    node.tf_broadcaster.sendTransform(tf_msg)
