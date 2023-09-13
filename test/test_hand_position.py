import rospy, tf
import numpy as np
import time

from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from franka_gripper.msg import MoveActionGoal, GraspActionGoal
from std_msgs.msg import Float64

import my_log
log = my_log.logger(name=__name__)
    
class PandaRobot:
    def __init__(self) -> None:
        log.info("Initialising node.")
        rospy.init_node(name="panda_sim_test")
        self.pose_publisher = rospy.Publisher(name='/cartesian_impedance_example_controller/equilibrium_pose',
                                        data_class=PoseStamped,
                                        queue_size=10)
        
        self.qb_publisher = rospy.Publisher(name='/right_hand_v2s/synergy_command',
                                              data_class=Float64,
                                              queue_size=10)
        
        self.tf_listener = tf.TransformListener()
        log.info("Waiting for transform.")
        self.tf_listener.waitForTransform("/panda_link0", "/panda_EE", rospy.Time(), rospy.Duration(5))
    
    def get_cart_pose(self):
        trans, rot = self.tf_listener.lookupTransform("/panda_link0", "/panda_EE", rospy.Time(0))
        return trans, rot

    def grasp(self, value:float):
        log.info("Grasping")
        msg = Float64()
        msg.data = value
        self.qb_publisher.publish(msg)
    
    def cart_move(self, trans, rot):
        robot_pose = PoseStamped()
        robot_pose.header.frame_id = "panda_link0" 
        robot_pose.pose.position.x = trans[0]
        robot_pose.pose.position.y = trans[1]
        robot_pose.pose.position.z = trans[2]

        q4e = tf.transformations.quaternion_from_euler(rot[0], rot[1], rot[2])
        robot_pose.pose.orientation.x = q4e[0]
        robot_pose.pose.orientation.y = q4e[1]
        robot_pose.pose.orientation.z = q4e[2]
        robot_pose.pose.orientation.w = q4e[3]
        
        log.debug("Sending cart move command")
        self.pose_publisher.publish(robot_pose)
            
            
if __name__ == "__main__":
    import time
    robot = PandaRobot()
    trans, rot = robot.get_cart_pose()
    log.info(f"Trans: {trans}")
    log.info(f"Rot: {rot}")

    for i in range(100):
        robot.cart_move([0.6, 0.0, 0.3], [0, 1.57, 3.14])
        time.sleep(0.01)
    
    trans, rot = robot.get_cart_pose()
    log.info(f"Trans: {trans}")
    log.info(f"Rot: {rot}")
