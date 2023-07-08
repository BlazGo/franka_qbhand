import rospy
from franka_msgs.srv import SetLoad, SetLoadRequest, SetLoadResponse

class SetLoadService:
    def __init__(self, node_name:str="set_qbhand_load") -> None:
        rospy.init_node(node_name)
        self.client = rospy.ServiceProxy("/franka_control/set_load", SetLoad)
    
    def send_request(self, 
                     mass:float=1.0,
                     F_x_center_load:list=[0.0, 0.0, 0.0],
                     load_inertia:list=[0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0]) -> SetLoadResponse:
        msg = SetLoadRequest()
        msg.mass = mass # [kg] 
        msg.F_x_center_load = F_x_center_load # translation vector
        msg.load_inertia = load_inertia # inertia matrix (one line)
        
        response = self.client(msg)
        # response has attributes:
        # success: true/false
        # error: ''
        print(f"Recieved rosservice response: {response}")
        return response

if __name__ == "__main__":
    set_load = SetLoadService()
    set_load.send_request()