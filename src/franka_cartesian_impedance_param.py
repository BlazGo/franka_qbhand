import rospy
from dynamic_reconfigure.client import Client

class SetCartesianImpedanceParams:
    def __init__(self, node_name:str="set_cartesian_impedance") -> None:
        rospy.init_node(node_name)
        self.client = Client("/cartesian_impedance_example_controller/dynamic_reconfigure_compliance_param_node", timeout=1, config_callback=self.callback)

    def set_param(self):
        self.client.update_configuration({"translational_stiffness": 100.0})

if __name__ == "__main__":
    set_load = SetCartesianImpedanceParams()
    set_load.set_param()
