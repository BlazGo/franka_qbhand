import rospy
from panda_robot import PandaArm
import franka_interface

if __name__ == '__main__':
    rospy.init_node("panda_demo") # initialise ros node
    r = PandaArm() # create PandaArm instance
