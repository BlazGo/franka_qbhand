import rospy
from franka_msgs.srv import SetCartesianImpedance, SetCartesianImpedanceRequest

class SetCartesianImpedanceParams:
    def __init__(self, node_name:str="set_cartesian_impedance") -> None:
        rospy.init_node(node_name)
    
    def set_params(self,
                   name:str="/cartesian_impedance_example_controller/dynamic_reconfigure_compliance_param_node/translational_stiffness",
                   value:float=500.0):
        rospy.set_param(param_name=name,
                        param_value=value)

if __name__ == "__main__":
    set_load = SetCartesianImpedanceParams()
    set_load.set_params()
