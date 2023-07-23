import rospy
from franka_msgs.srv import SetEEFrame, SetEEFrameRequest, SetEEFrameResponse
from std_msgs.msg import Float64MultiArray

class SetEEFrameService:
    def __init__(self,node_name:str="set_qbhand_EE_Frame") -> None:
        rospy.init_node(node_name)
        self.client = rospy.ServiceProxy("/franka_control/set_EE_frame",
                                         SetEEFrame)
    
    def send_request(self, NE_T_EE:list):
        msg = SetEEFrameRequest()
        msg.NE_T_EE = [1.0, 0.0, 0.0, 0.0,
                       0.0, 1.0, 0.0, 0.0,
                       0.0, 0.0, 1.0, 0.3,
                       0.0, 0.0, 0.0, 1.0]
        respone = self.client(msg)
        print(f"Response: {respone}")

if __name__ == "__main__":
    srv = SetEEFrameService()
    srv.send_request([])