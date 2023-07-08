import rospy
from franka_msgs.srv import SetLoad, SetLoadRequest, SetLoadResponse

class SetLoadService:
    def __init__(self) -> None:
        node = rospy.init_node("set_qbhand_load")
        self.client = rospy.ServiceProxy("/franka_control/set_load", SetLoad)
    
    def send_request(self, 
                     mass:float=1.0,
                     F_x_center_load:list=[0.0, 0.0, 0.0],
                     load_inertia:list=[0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0]):
        message = SetLoadRequest()
        message.mass = mass # [kg] 
        message.F_x_center_load = F_x_center_load # translation vector
        message.load_inertia = load_inertia # inertia matrix (one line)
        
        response = self.client(message)
        print(f"Recieved rosservice response: {response}")
        return response

if __name__ == "__main__":
    set_load = SetLoadService()
    set_load.send_request()