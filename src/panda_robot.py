import numpy as np
import rospy
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from dynamic_reconfigure.client import Client
from geometry_msgs.msg import PoseStamped
from franka_msgs.msg import FrankaState
from franka_msgs.srv import SetEEFrame, SetEEFrameRequest, SetEEFrameResponse
from franka_msgs.srv import SetLoad, SetLoadRequest, SetLoadResponse
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest, SwitchControllerResponse
from controller_manager_msgs.srv import ListControllers, ListControllersRequest, ListControllersResponse

import my_log
import my_tools
from my_gripper import QbHand

# EE frame parameters
TOOL_MASS: float = 0.76 * 2 # [kg] (compensation mass on opposite side)
TOOL_MASS: float = 0.0 # [kg] we already set it in launch file

# the two parameters below are not accurate
F_X_CENTER_TOOL: list = [0.12, -0.0, 0.05]
TOOL_INERTIA:list = [0.01, 0.0, 0.0,
                     0.0, 0.01, 0.0,
                     0.0, 0.0, 0.01]
    
class PandaRobot:
    # topics
    set_EE_frame_topic = "/franka_control/set_EE_frame"
    franka_state_topic = "/franka_state_controller/franka_states"
    
    compliance_topic = "/cartesian_impedance_example_controller/dynamic_reconfigure_compliance_param_node"
    
    set_EE_load_topic = "/franka_control/set_load"
    switch_controller_topic = "controller_manager/switch_controller"
    list_controllers_topic = "controller_manager/list_controllers"
    cartesian_pose_topic = "/cartesian_impedance_example_controller/equilibrium_pose"

    BASE_LINK: str = "panda_link0"
    EE_frame: str = "panda_EE"

    Q_INIT: list = [0, -0.785398163, 0, -2.35619449, 0, 1.57079632679, 0.785398163397]
    DEFAULT_CART_STIFFNESS_TRANS:float = 200.0
    DEFAULT_CART_STIFFNESS_ROT:float = 10.0
    DEFAULT_CART_STIFFNESS_NULL:float = 0.5
    
    # controller strategies
    STRICT: int = 2
    BEST_EFFORT: int = 1

    def __init__(self, log_level=my_log.INFO, move_feedback=False):
        self.log = my_log.logger(log_level)
        self.log.info("Initializing PandaRobot class")

        # checking position to wait to finish the move
        self.move_feedback = move_feedback
        self.trans_tolerance = 0.001 # [m]
        self.rot_tolerance = 0.01 # [rad]
        
        self.compliance_client = Client(self.compliance_topic, timeout=1)
        self.state_listener = rospy.Subscriber(self.franka_state_topic,
                                               FrankaState,
                                               callback=self.franka_state_callback,
                                               queue_size=1,)

        self.cartesian_goal_publisher = rospy.Publisher(self.cartesian_pose_topic, PoseStamped, queue_size=1)
        self.tf_listener = tf.TransformListener()

        # Services
        self.set_EE_frame_client = rospy.ServiceProxy(self.set_EE_frame_topic, SetEEFrame)
        self.set_EE_load_client = rospy.ServiceProxy(self.set_EE_load_topic, SetLoad)
        self.switch_controller_proxy = rospy.ServiceProxy(self.switch_controller_topic, SwitchController)
        self.list_controllers_proxy = rospy.ServiceProxy(self.list_controllers_topic, ListControllers)
        rospy.sleep(0.5)

        self.used_controllers:list = [cnt.name for cnt in self.list_controllers(minimal=False)]
        self.log.info(f"Loaded controlers: {self.used_controllers}")
        
        # Stiffness setup
        self.set_stiffness(trans=400.0, rot=20.0, null=1.0)
        self.log.info(f"Stiffness: {self.get_stiffness()}")
        
        rospy.sleep(1.0)
        
        response = self.switch_controller(stop_controllers=self.used_controllers)
        self.log.debug(f"Response switch controllers: {response}")
        response = self.list_controllers(minimal=True)
        self.log.info(f"Controllers: {response}")

        rospy.sleep(1.0)
        
        response = self.set_EE_load(TOOL_MASS,
                                    F_X_CENTER_TOOL,
                                    TOOL_INERTIA)
        self.log.debug(f"Response set EE load: {response}")
        
        # EE frame transformation construction
        T_trans = np.eye(4)
        T_trans[0:3, 3] = np.array([0.20, -0.003, 0.053]).T

        T_rot = np.eye(4)
        T_rot[0:3, 0:3] = my_tools.get_rotation_matrix(angle=np.deg2rad(-45), axis="z")

        T_new = np.matmul(T_rot, T_trans)
        T_new = np.reshape(T_new, (16), order = 'F') # msg formaat

        self.log.debug(f"{T_new}")

        # Set the EE (End Effector) frame transform
        response = self.set_NE_T_EE_transform(NE_T_EE=T_new)
        self.log.debug(f"Response set EE trans: {response}")
        
        rospy.sleep(1.0)

        response = self.switch_controller(start_controllers=self.used_controllers)
        self.log.debug(f"Response switch controllers: {response}")
        
        self.log.info("Initializing QbHand")
        self.gripper = QbHand()
        
        rospy.sleep(1.0)

        self.log.info(f"Panda initialization {my_log.GREEN}DONE")

    def franka_state_callback(self, msg) -> None:
        # Save the state
        self.state = msg

    def get_state(self) -> FrankaState:
        # Return the robot state msg
        # TODO prettier parsing?
        return self.state

    def get_cart_pose(self, frame0=BASE_LINK, frame1=EE_frame):
        if self.tf_listener.frameExists(frame0) and self.tf_listener.frameExists(frame1):
            t = self.tf_listener.getLatestCommonTime(frame0, frame1)
            trans, rot = self.tf_listener.lookupTransform(frame0, frame1, t)
            return trans, rot
        else:
            self.log.error(f"Transform could not be fetched ('{frame0}' to '{frame1}')")
            return None

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
        rospy.sleep(0.5)
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
        rospy.sleep(0.5)
        return response

    def switch_controller(self, start_controllers: list = [""], stop_controllers: list = [""], strictness: int = BEST_EFFORT, start_asap: bool = False) -> SwitchControllerResponse:
        self.log.debug("Switching controllers")            
        self.log.debug(f"Starting: {start_controllers}")
        self.log.debug(f"Stopping: {stop_controllers}")
        
        msg = SwitchControllerRequest()
        msg.start_controllers = start_controllers
        msg.stop_controllers = stop_controllers
        msg.strictness = strictness
        msg.start_asap = start_asap
        msg.timeout = 1.0

        response = self.switch_controller_proxy.call(msg)
        rospy.sleep(1.0)
        return response

    def list_controllers(self, minimal=True) -> ListControllersResponse:
        msg = ListControllersRequest()
        response = self.list_controllers_proxy.call(msg)
        if minimal:
            cnts = [cnt.name for cnt in response.controller]
            states = [cnt.state for cnt in response.controller]
            response = []
            for i in range(len(cnts)):
                response.append([cnts[i], states[i]])
            return response 
        else:
            return response.controller # Unpacks the actual list from msg

    def cart_move(self, trans, rot):
        msg = PoseStamped()

        msg.header.frame_id = "/panda_link0"
        msg.header.stamp = rospy.Time.now()

        msg.pose.position.x = trans[0]
        msg.pose.position.y = trans[1]
        msg.pose.position.z = trans[2]

        msg.pose.orientation.x = rot[0]
        msg.pose.orientation.y = rot[1]
        msg.pose.orientation.z = rot[2]
        msg.pose.orientation.w = rot[3]

        self.cartesian_goal_publisher.publish(msg)

    def cart_pid(self, trans, rot):
        t_move = 4.0
        err = 0

    def cart_move_smooth(self, trans, rot):
        # TODO maybe add pid compensation
        n_steps = 10
        t_move = 4.0
        dt = t_move / n_steps
        t = np.linspace(0, 1, n_steps)

        trans_start, rot_start_q = self.get_cart_pose()
        
        # calculate difference from start
        diff_trans = []
        for i in range(3):
            diff_trans.append(trans[i] - trans_start[i])
            
        diff_trans = np.asarray(diff_trans)
        trans_start = np.asanyarray(trans_start)
        
        for t_curr in t:
            value = my_tools.interpolate(t_curr)
            curr_trans = trans_start + diff_trans*value
            self.cart_move(trans=curr_trans,rot=rot)
            rospy.sleep(dt)    
        
        # keep publishing the last position
        timeout = 0
        if self.move_feedback == True:
            for _ in range(4):
                self.cart_move(trans=curr_trans,rot=rot)
                rospy.sleep(dt)
                
                _trans, _rot = self.get_cart_pose()
                for i, ax in enumerate(_trans):
                    diff = trans[i] -_trans[i]
                    if diff > self.trans_tolerance:
                        continue
                       
    def grasp(self, value:float, t:float=2.0):
        self.gripper.linear_move(value, move_time=t)

if __name__ == "__main__":

    rospy.init_node("panda_custom", anonymous=True)
    robot = PandaRobot(log_level=my_log.INFO)
    
    trans, rot = robot.get_cart_pose()
    print(trans, euler_from_quaternion(rot))
    
    robot.cart_move_smooth([0.6, 0.2, 0.4], [-1, 0, 0, 0])
    robot.cart_move_smooth([0.6, 0.0, 0.3], [-1, 0, 0, 0])
    robot.grasp(1.0)
    robot.grasp(0.0)
    
    trans, rot = robot.get_cart_pose()
    print(trans, euler_from_quaternion(rot))