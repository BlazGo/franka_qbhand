import rospy
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest


class SwitchControllersSrv:
    def __init__(self) -> None:
        self.client = rospy.ServiceProxy("controller_manager/switch_controller",
                                         SwitchController)
    
    def send_request(self, start_controllers:list, stop_controllers:list):
        msg = SwitchControllerRequest()
        msg.start_controllers = start_controllers
        msg.stop_controllers = stop_controllers
        respone = self.client.call(msg)
        print(f"Response: {respone}")


if __name__ == "__main__":
    rospy.init_node("ee_setup")
       
    switch_controllers = SwitchControllersSrv()
    #switch_controllers.send_request(start_controllers=[""], stop_controllers=["cartesian_impedance_example_controller"])
    switch_controllers.send_request(start_controllers=["cartesian_impedance_example_controller"], stop_controllers=[""])