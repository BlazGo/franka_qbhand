#!/usr/bin/python
import sys
import rospy
import tf
import tf2_ros
import geometry_msgs.msg  
import logging

if __name__ == "__main__":
    rospy.init_node(name="flange_static_tf_broadcaster")
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_tfStamped = geometry_msgs.msg.TransformStamped()
    
    static_tfStamped.header.stamp = rospy.Time.now()
    static_tfStamped.header.frame_id = "panda_EE"
    static_tfStamped.child_frame_id = "flange"
    
    static_tfStamped.transform.translation.x = 0.5
    static_tfStamped.transform.translation.y = 0.0
    static_tfStamped.transform.translation.z = 0.0
    
    static_tfStamped.transform.rotation.x = 0.0
    static_tfStamped.transform.rotation.y = 0.0
    static_tfStamped.transform.rotation.z = 0.0
    static_tfStamped.transform.rotation.w = 1.0
    
    broadcaster.sendTransform(static_tfStamped)
    rospy.spin()
    