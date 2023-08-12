import rospy
from dynamic_reconfigure.client import Client

class SetCartesianImpedanceParams:
    def __init__(self) -> None:
        self.client = Client("/cartesian_impedance_example_controller/dynamic_reconfigure_compliance_param_node", timeout=1, config_callback=self.callback)

    def set_impedance(self, trans:float=None, rot:float=None):
        config = {}
        if trans is not None:
            config["translational_stiffness"] = trans
        if rot is not None:
            config["translational_stiffness"] = rot
        self.client.update_configuration(config)

    def callback(self, *args):
        pass
    
if __name__ == "__main__":
    node_name:str="set_cartesian_impedance"
    rospy.init_node(node_name)

    set_load = SetCartesianImpedanceParams()
    set_load.set_impedance(trans=400.0, rot=20.0)
