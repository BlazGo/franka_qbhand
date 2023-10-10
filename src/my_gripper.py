import rospy
import numpy as np
from std_msgs.msg import Float64

class QbHand:
    def __init__(self) -> None:
        self.qb_publisher = rospy.Publisher(name='/right_hand_v2s/synergy_command',
                                              data_class=Float64,
                                              queue_size=10)
        self._current_value = 0.0
        self.send_command(0)
        rospy.sleep(1.0)
    
    def send_command(self, value:float):
        msg = Float64()
        msg.data = value
        self._current_value = value
        self.qb_publisher.publish(msg)
    
    def linear_move(self, value:float, move_time:float=2.0):
        steps = 10
        dt = move_time/steps
        
        ramp = np.linspace(self.state, value, steps)
        for i in ramp:
            self.send_command(i)
            rospy.sleep(dt)
    
    @property
    def state(self):
        return self._current_value
    
    
if __name__ == "__main__":
    rospy.init_node(name="qb_test")

    qb_cont = QbHand()
    
    qb_cont.send_command(1.0)
    rospy.sleep(1.0)
    qb_cont.send_command(0.0)
