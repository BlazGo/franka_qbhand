import rospy
from panda_robot import PandaArm
from 
if __name__ == "__main__":
    rospy.init_node("panda_demo")
    
    r = PandaArm()
    r.move_to_neutral()