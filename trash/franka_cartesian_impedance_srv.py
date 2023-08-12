import rospy
from franka_msgs.srv import SetJointImpedance, SetJointImpedanceRequest
from franka_msgs.srv import SetCartesianImpedance, SetCartesianImpedanceRequest, SetCartesianImpedanceResponse
from franka_msgs.srv import SetLoad, SetLoadRequest, SetLoadResponse
from dynamic_reconfigure.msg import ConfigDescription

# Not correct?
class SetCartStiffSrv:
    def __init__(self) -> None:
        self.client = rospy.ServiceProxy("/cartesian_impedance_example_controller/dynamic_reconfigure_compliance_param_node/set_parameters",
                                         SetCartesianImpedance)
    
    def send_request(self) -> SetCartesianImpedanceResponse:
        msg = SetCartesianImpedanceRequest()
        msg.cartesian_stiffness = [400.0, 400.0, 400.0, 20.0, 20.0, 20.0]

        print(f"Sending rosservice msg: {msg}")

        response = self.client.call(msg)
        # response has attributes:
        # success: true/false
        # error: ''
        print(f"Recieved rosservice response: {response}")
        return response


class DynamicStiff:
    def __init__(self) -> None:
        rospy.Publisher(name="/cartesian_impedance_example_controller/dynamic_reconfigure_compliance_param_node",
                        data_class=ConfigDescription,
                        queue_size=5)
    def publish(self):
        msg = ConfigDescription()
        msg.groups = ["Default"]
        msg.dflt.doubles
        print(msg)        

if __name__ == "__main__":
    node_name:str="set_cart_stiffness"
    rospy.init_node(node_name)

    # set_load = SetCartStiffSrv()
    # set_load.send_request()

    rospy.set_param(param_name="/cartesian_impedance_example_controller/dynamic_reconfigure_compliance_param_node/translational_stiffness",
                    param_value=100.0)

    pass