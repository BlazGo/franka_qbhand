import rospy
from franka_msgs.srv import SetJointImpedance, SetJointImpedanceRequest
from franka_msgs.srv import SetCartesianImpedance, SetCartesianImpedanceRequest, SetCartesianImpedanceResponse
from franka_msgs.srv import SetLoad, SetLoadRequest, SetLoadResponse


# Not correct?
class SetCartStiffSrv:
    def __init__(self, node_name:str="set_cart_stiffness") -> None:
        rospy.init_node(node_name)
        self.client = rospy.ServiceProxy("/cartesian_impedance_example_controller/dynamic_reconfigure_compliance_param_node/set_parameters",
                                         SetCartesianImpedance)
    
    def send_request(self) -> SetCartesianImpedanceResponse:
        msg = SetCartesianImpedanceRequest()
        msg.cartesian_stiffness = [400.0, 400.0, 400.0, 20.0, 20.0, 20.0]

        print(f"Sending rosservice msg: {msg}")

        response = self.client(msg)
        # response has attributes:
        # success: true/false
        # error: ''
        print(f"Recieved rosservice response: {response}")
        return response

if __name__ == "__main__":
    set_load = SetCartStiffSrv()
    set_load.send_request()
