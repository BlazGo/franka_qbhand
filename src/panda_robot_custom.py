import rospy
from dynamic_reconfigure.client import Client
from franka_msgs.msg import FrankaState
from franka_msgs.srv import SetEEFrame, SetEEFrameRequest, SetEEFrameResponse
from franka_msgs.srv import SetLoad, SetLoadRequest, SetLoadResponse
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
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
        
        # Services
        self.set_EE_frame_client = rospy.ServiceProxy(self.set_EE_frame_topic, SetEEFrame)
        self.set_EE_load_client = rospy.ServiceProxy(self.set_EE_load_topic, SetLoad)
        self.controller_client = rospy.ServiceProxy(self.switch_controller_topic, SwitchController)
         
        rospy.sleep(0.5)
        log.info("Panda initialized")
    
    def franka_state_callback(self, msg):
        # Save the state
        self.state = msg
        log.debug("Callback recieved")    
        
    def get_state(self):
        # Return the robot state msg
        return self.state

    def set_stiffness(self, trans: float = None, rot: float = None, null: float = None):
        # Create data
        config = {}
        if trans is not None:
            config["translational_stiffness"] = trans
        if rot is not None:
            config["translational_stiffness"] = rot
        if null is not None:
            config["null_stiffness"] = null

        # Update parameters
        response = self.compliance_client.update_configuration(config)
        return response
    
    def set_NE_T_EE_transform(self, NE_T_EE:np.ndarray):
        msg = SetEEFrameRequest()
        msg.NE_T_EE = NE_T_EE
        respone = self.set_EE_frame_client.call(msg)
        return respone
    
    def set_EE_load(self, 
                     mass:float=0.0,
                     F_x_center_load:list=[0.0, 0.0, 0.0],
                     load_inertia:list=[0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0]) -> SetLoadResponse:
        msg = SetLoadRequest()
        msg.mass = mass                        # [kg] 
        msg.F_x_center_load = F_x_center_load  # translation vector
        msg.load_inertia = load_inertia        # inertia matrix (one line)
        
        response = self.set_EE_load_client.call(msg)
        # response has attributes:
        # success: true/false
        # error: ''
        return response

    def switch_controllers(self, start_controllers: list, stop_controllers: list, strictness: int = BEST_EFFORT, start_asap:bool=False):
        msg = SwitchControllerRequest()
        msg.start_controllers = start_controllers
        msg.stop_controllers = stop_controllers
        msg.strictness = strictness
        msg.start_asap = start_asap
        msg.timeout = 1.0
        
        response = self.controller_client.call(msg)
        return response

if __name__ == "__main__":
    rospy.init_node("panda_custom", anonymous=True)
    robot = PandaRobotCustom()
    print(robot.get_state().NE_T_EE)
    
    mass = 0.76
    Fx_frame = [0.1, 0.0, 0.05]
    inertia = [0.001333, 0.0,     0.0,
               0.0,      0.00347, 0.0,
               0.0,      0.0,     0.00453]
    
    response = robot.set_EE_load(mass=mass, F_x_center_load=Fx_frame, load_inertia=inertia)
    
