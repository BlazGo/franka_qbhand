import rospy
from franka_msgs.msg import FrankaState

class FrankaStateWrapper():
    def __init__(self) -> None:
        self.listener = rospy.Subscriber("/franka_state_controller/franka_states", FrankaState, callback=self.callback, queue_size=1)
    
    def callback(self, msg):
        print(msg.NE_T_EE)

if __name__ == "__main__":
    rospy.init_node("Franka_state_test", anonymous=True)
    fs = FrankaStateWrapper()
    rospy.spin()