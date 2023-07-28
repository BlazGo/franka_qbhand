import rospy
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest


class SwitchControllersSrv:
    STRICT: int = 2
    BEST_EFFORT: int = 1

    def __init__(self) -> None:
        self.client = rospy.ServiceProxy(
            "controller_manager/switch_controller", SwitchController
        )

    def send_request(
        self, start_controllers: list, stop_controllers: list, strictness: int = BEST_EFFORT, start_asap:bool=False
    ):
        msg = SwitchControllerRequest()
        msg.start_controllers = start_controllers
        msg.stop_controllers = stop_controllers
        msg.strictness = strictness
        msg.start_asap = start_asap
        msg.timeout = 0.0
        respone = self.client.call(msg)
        print(f"Response: {respone}")


if __name__ == "__main__":
    rospy.init_node("ee_setup")

    switch_controllers = SwitchControllersSrv()
    # switch_controllers.send_request(start_controllers=[""], stop_controllers=["cartesian_impedance_example_controller"])
    switch_controllers.send_request(
        start_controllers=["cartesian_impedance_example_controller"],
        stop_controllers=[""],
    )
