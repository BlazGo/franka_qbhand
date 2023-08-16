import rospy
import tf
from dynamic_reconfigure.client import Client
from geometry_msgs.msg import PoseStamped
from franka_msgs.msg import FrankaState
from franka_msgs.srv import SetEEFrame, SetEEFrameRequest, SetEEFrameResponse
from franka_msgs.srv import SetLoad, SetLoadRequest, SetLoadResponse
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest, SwitchControllerResponse
from controller_manager_msgs.srv import ListControllers, ListControllersRequest, ListControllersResponse
import numpy as np
import my_log

log = my_log.logger()


class PandaRobotCustom:
    # topics
    set_EE_frame_topic = "/franka_control/set_EE_frame"
    franka_state_topic = "/franka_state_controller/franka_states"
    compliance_topic = "/cartesian_impedance_example_controller/dynamic_reconfigure_compliance_param_node"
    set_EE_load_topic = "/franka_control/set_load"
    switch_controller_topic = "controller_manager/switch_controller"
    list_controllers_topic = "controller_manager/list_controllers"
    cartesian_pose_topic = "/cartesian_impedance_example_controller/equilibrium_pose"

    base_link = "panda_link0"
    EE_frame = "panda_EE"

    # controller strategies
    STRICT: int = 2
    BEST_EFFORT: int = 1

    def __init__(self):
        log.info("Initializing PandaRobot class")

        self.compliance_client = Client(self.compliance_topic, timeout=1)
        self.state_listener = rospy.Subscriber(self.franka_state_topic,
                                               FrankaState,
                                               callback=self.franka_state_callback,
                                               queue_size=1,)

        self.cartesian_goal_publisher = rospy.Publisher(
            self.cartesian_pose_topic, PoseStamped, queue_size=1)
        self.tf_listener = tf.TransformListener()

        # Services
        self.set_EE_frame_client = rospy.ServiceProxy(
            self.set_EE_frame_topic, SetEEFrame)
        self.set_EE_load_client = rospy.ServiceProxy(
            self.set_EE_load_topic, SetLoad)
        self.switch_controller_proxy = rospy.ServiceProxy(
            self.switch_controller_topic, SwitchController)
        self.list_controllers_proxy = rospy.ServiceProxy(
            self.list_controllers_topic, ListControllers)
        rospy.sleep(0.5)

        log.info(f"Panda initialization {my_log.GREEN}DONE")

        self.init_robot()

    def init_robot(self):
        self.used_controllers = [cnt.name for cnt in self.list_controllers()]
        
        # Stiffness setup
        log.info(f"Old stiffness: {self.get_stiffness()}")
        self.set_stiffness(trans=400.0, rot=20.0, null=1.0)
        log.info(f"New stiffness: {self.get_stiffness()}")
        # Wait for he robot to stop moving
        rospy.sleep(2.0)
        
        # Get init pose
        log.info(f"{self.get_state().q}, {self.get_state().NE_T_EE}")
        pose = self.get_cart_pose()
        log.info(f"Cartesian pose: {pose}")
        self.q_init = self.get_state().q
        self.cart_init = pose

        log.info(f"Secondary initialization  {my_log.GREEN}DONE")

    def franka_state_callback(self, msg) -> None:
        # Save the state
        self.state = msg
        log.debug("Callback recieved")

    def get_state(self) -> FrankaState:
        # Return the robot state msg
        # TODO prettier parsing?
        return self.state

    def get_cart_pose(self, frame0=None,  frame1=None):
        if frame0 is None:
            frame0 = self.base_link
        if frame1 is None:
            frame1 = self.EE_frame

        if self.tf_listener.frameExists(frame0) and self.tf_listener.frameExists(frame1):
            t = self.tf_listener.getLatestCommonTime(frame0, frame1)
            position, quaternion = self.tf_listener.lookupTransform(frame0, frame1, t)
            log.info(f"Trans: {position}, Quat: {quaternion}")
        else:
            log.error(f"Transform could not be fetched ('{frame0}' to '{frame1}')")
            return None
        return position, quaternion

    def set_stiffness(self, trans: float = None, rot: float = None, null: float = None):

        # Create data
        changes = []
        if trans is not None:
            changes.append({"translational_stiffness": trans})
        if rot is not None:
            changes.append({"rotational_stiffness": rot})
        if null is not None:
            changes.append({"nullspace_stiffness": null})

        # Update parameters
        responses = []
        for change in changes:
            response = self.compliance_client.update_configuration(change)
            rospy.sleep(0.25)
            responses.append(response)
        return responses

    def get_stiffness(self):
        return rospy.get_param(param_name=self.compliance_topic + "")

    def set_NE_T_EE_transform(self, NE_T_EE: np.ndarray) -> SetEEFrameResponse:
        msg = SetEEFrameRequest()
        msg.NE_T_EE = NE_T_EE
        respone = self.set_EE_frame_client.call(msg)
        return respone

    def set_EE_load(self, mass: float = 0.0, F_x_center_load: list = [0.0, 0.0, 0.0], load_inertia: list = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) -> SetLoadResponse:
        msg = SetLoadRequest()
        msg.mass = mass                        # [kg]
        msg.F_x_center_load = F_x_center_load  # translation vector
        msg.load_inertia = load_inertia        # inertia matrix (one line)

        response = self.set_EE_load_client.call(msg)
        # response has attributes:
        # success: true/false
        # error: ''
        return response

    def switch_controller(self, start_controllers: list = None, stop_controllers: list = None, strictness: int = BEST_EFFORT, start_asap: bool = False) -> SwitchControllerResponse:
        if start_controllers is None and stop_controllers is None:
            raise ValueError("Both arguments cannot be None!")
        if start_controllers is None:
            start_controllers = [""]
        if stop_controllers is None:
            stop_controllers = [""]

        msg = SwitchControllerRequest()
        msg.start_controllers = start_controllers
        msg.stop_controllers = stop_controllers
        msg.strictness = strictness
        msg.start_asap = start_asap
        msg.timeout = 1.0

        response = self.switch_controller_proxy.call(msg)
        return response

    def list_controllers(self) -> ListControllersResponse:
        msg = ListControllersRequest()
        response = self.list_controllers_proxy.call(msg)
        return response.controller  # Unpacks the actual list from msg

    def cartesian_move(self, goal):
        msg = PoseStamped()

        msg.header.frame_id = "/panda_link0"
        msg.header.stamp = rospy.Time.now()

        msg.pose.position.x = goal[0]
        msg.pose.position.y = goal[1]
        msg.pose.position.z = goal[2]

        msg.pose.orientation.x = goal[3]
        msg.pose.orientation.y = goal[4]
        msg.pose.orientation.z = goal[5]
        msg.pose.orientation.w = goal[6]

        self.cartesian_goal_publisher.publish(msg)

    def cartesian_move_smooth(self, goal):
        pass


if __name__ == "__main__":
    rospy.init_node("panda_custom", anonymous=True)
    robot = PandaRobotCustom()
    log.info(robot.get_state().NE_T_EE)

    mass = 0.76
    Fx_frame = [0.1, 0.0, 0.05]
    inertia = [0.001333, 0.0,     0.0,
               0.0,      0.00347, 0.0,
               0.0,      0.0,     0.00453]
    log.info(robot.used_controllers)
    # response = robot.set_EE_load(mass=mass, F_x_center_load=Fx_frame, load_inertia=inertia)
