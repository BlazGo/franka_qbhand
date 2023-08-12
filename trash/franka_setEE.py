import rospy
from franka_msgs.srv import SetLoad, SetLoadRequest, SetLoadResponse
from franka_msgs.srv import SetEEFrame, SetEEFrameRequest, SetEEFrameResponse
from std_msgs.msg import Float64MultiArray

class SetEEFrameService:
    def __init__(self) -> None:
        self.client = rospy.ServiceProxy("/franka_control/set_EE_frame",
                                         SetEEFrame)
    
    def send_request(self, NE_T_EE:list):
        msg = SetEEFrameRequest()
        msg.NE_T_EE = NE_T_EE
        respone = self.client.call(msg)
        return respone
    
    
class SetLoadService:
    def __init__(self) -> None:
        self.client = rospy.ServiceProxy("/franka_control/set_load",
                                         SetLoad)
    
    def send_request(self, 
                     mass:float=0.0,
                     F_x_center_load:list=[0.0, 0.0, 0.0],
                     load_inertia:list=[0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0]) -> SetLoadResponse:
        msg = SetLoadRequest()
        msg.mass = mass # [kg] 
        msg.F_x_center_load = F_x_center_load # translation vector
        msg.load_inertia = load_inertia # inertia matrix (one line)
        
        response = self.client.call(msg)
        # response has attributes:
        # success: true/false
        # error: ''
        return response


if __name__ == "__main__":
    rospy.init_node("ee_setup")
    
    import tf
    tf_listener = tf.TransformListener()
    tf_listener.waitForTransform("/panda_link0", "/panda_EE", rospy.Time(), rospy.Duration(5))
    t = tf_listener.lookupTransform("/panda_link0", "/panda_EE", rospy.Time(0))
    print(t)
    
    srv_ee_load = SetLoadService()
    srv_ee_load.send_request(mass=0.76, 
                             F_x_center_load=[0.4, 0.0, 0.1])
    
    t = tf_listener.lookupTransform("/panda_link0", "/panda_EE", rospy.Time(0))
    print(t)
    
    srv_ee = SetEEFrameService()
    srv_ee.send_request([1.0, 0.0, 0.0, 0.0,
                         0.0, 1.0, 0.0, 0.0,
                         0.0, 0.0, 1.0, 0.0,
                         0.0, 0.0, 0.0, 1.0])
    
    t = tf_listener.lookupTransform("/panda_link0", "/panda_EE", rospy.Time(0))
    print(t)