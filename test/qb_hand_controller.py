import rospy
import numpy as np
from std_msgs.msg import Float64


# TODO spline ramp up ramp down or similar
class QbhandControll:
    def __init__(self) -> None:
        self.qb_publisher = rospy.Publisher(name='/right_hand_v2s/synergy_command',
                                              data_class=Float64,
                                              queue_size=10)
        self.current_value = 0.0
        self.send_command(0)
        rospy.sleep(1.0)
    
    def send_command(self, value:float):
        msg = Float64()
        msg.data = value
        self.qb_publisher.publish(msg)
    
    def linear_move(self, value:float):
        steps = 10
        move_time = 2.0
        dt = move_time/steps
        
        ramp = np.linspace(self.current_value, value, steps)
        for i in ramp:
            self.send_command(i)
            self.current_value = value
            rospy.sleep(dt)

    
if __name__ == "__main__":
    rospy.init_node(name="qb_test")

    qb_cont = QbhandControll()
    
    qb_cont.send_command(1.0)
    qb_cont.send_command(0.0)
