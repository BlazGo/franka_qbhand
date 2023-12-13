import numpy as np
import rospy
from dynamic_reconfigure.client import Client
from franka_msgs.srv import SetEEFrame, SetEEFrameRequest, SetEEFrameResponse

import my_log

if __name__ == "__main__":
    log = my_log.Logger(name="ros_tools", level=my_log.DEBUG)


class ComplianceWrapper:
    def __init__(self, compliance_topic = "/cartesian_impedance_example_controller/dynamic_reconfigure_compliance_param_node") -> object:
        self.compliance_client = Client(compliance_topic, timeout=1.0)

    def set_cart_stiffness(self, trans:float=None, rot:float=None, null:float=None):        
        # Create data
        change = {}
        if trans is not None:
            change["translational_stiffness"] = trans
        if rot is not None:
            change["rotational_stiffness"] = rot
        if null is not None:
            change["nullspace_stiffness"] = null

        # Update parameters
        response = self.compliance_client.update_configuration(change)
        return response


class CartGoalWrapper:
    def __init__(self) -> None:
        pass
    
    def get_cart_pose(self):
        pass
    
    
class EEFrameWrapper:
    def __init__(self, set_EE_frame_topic = "/franka_control/set_EE_frame") -> object:
        self.set_EE_frame_client = rospy.ServiceProxy(set_EE_frame_topic, SetEEFrame)
    
    def set_transform(self, NE_T_EE: np.ndarray) -> SetEEFrameResponse:
        """_summary_

        Args:
            NE_T_EE (np.ndarray): 4x4 transform matrix

        Returns:
            SetEEFrameResponse (string): response
        """
        msg = SetEEFrameRequest()
        
        NE_T_EE = np.reshape(NE_T_EE, (16), order = 'C').tolist() # msg format
        
        msg.NE_T_EE = NE_T_EE
        respone = self.set_EE_frame_client.call(msg)
        return respone



if __name__ == "__main__":
    rospy.init_node("ros_custom_tools_py", anonymous=True)
    
    # Set up the classes
    comp_client = ComplianceWrapper()
    cart_client = CartGoalWrapper()
    frame_EE_client = EEFrameWrapper()
    
    # Run communicatoion
    response = comp_client.set_cart_stiffness(trans=400.0)
    log.info(f"Set cartesian stiffness response: {response}")
    
    response = frame_EE_client.set_transform(np.eye((4)))
    log.info(f"Set EE frame response: {response}")
