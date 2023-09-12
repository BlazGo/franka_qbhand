import rospy
from franka_msgs.srv import SetKFrame, SetKFrameRequest, SetKFrameResponse

class SetKFrameService:
    def __init__(self) -> None:
        self.client = rospy.ServiceProxy("/franka_control/set_K_frame",
                                         SetKFrame)
    
    def send_request(self, EE_T_K:list) -> SetKFrameResponse:
        msg = SetKFrameRequest()
        msg.EE_T_K = EE_T_K
        response = self.client.call(msg)
        return response

if __name__ == "__main__":
    rospy.init_node("k_setup")

    srv_K_frame = SetKFrameService()
    response = srv_K_frame.send_request([1.0, 0.0, 0.0, 0.2,
                                         0.0, 1.0, 0.0, 0.0,
                                         0.0, 0.0, 1.0, 0.2,
                                         0.0, 0.0, 0.0, 1.0])
    print(response)