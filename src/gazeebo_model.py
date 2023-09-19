import rospy
import tf
from gazebo_msgs.srv import SpawnModel, DeleteModel, GetModelState
from geometry_msgs.msg import Point, Pose, Quaternion
import my_log

log = my_log.logger()


class Model:
    
    SERVICE_TOPIC = "gazebo/spawn_sdf_model"
    DELETE_TOPIC = "gazebo/delete_model"
    SPAWN_TOPIC = "/gazebo/spawn_sdf_model"
    STATE_TOPIC = "gazebo/get_model_state"
    
    def __init__(self, name='/home/bg/custom_models/hca_box/model.sdf'):
        self.MODEL_XML = name
        self.MODEL_NAME = "hca_box"
        
        rospy.wait_for_service(self.SERVICE_TOPIC, timeout=10.0)
        
        self._delete_model = rospy.ServiceProxy(self.DELETE_TOPIC, DeleteModel)
        self._spawn_model = rospy.ServiceProxy(self.SPAWN_TOPIC, SpawnModel)
        self._get_model_state = rospy.ServiceProxy(self.STATE_TOPIC, GetModelState)

    def spawn(self, pose=[0.6, 0, 0.05, 0, 0, 0]):
        log.info(f"Spawning model: @{self.MODEL_NAME} {pose}")
        quat = tf.transformations.quaternion_from_euler(pose[3], pose[4], pose[5])
        orient = Quaternion(quat[0], quat[1], quat[2], quat[3])
        model_pose = Pose(position = Point(x=pose[0], y=pose[1], z=pose[2]),
                          orientation = orient)

        self._spawn_model(model_name=self.MODEL_NAME,
                         model_xml=open(self.MODEL_XML, 'r').read(),
                         robot_namespace='',
                         initial_pose=model_pose,
                         reference_frame='world')
        rospy.sleep(1)

    def delete(self):
        log.info(f"Spawning model: @{self.MODEL_NAME}")
        self._delete_model(model_name=self.MODEL_NAME)
        rospy.sleep(1)

    def get_state(self):
        return self._get_model_state(self.MODEL_NAME, "link")

    def get_pose(self):
        state = self.get_state()
        return state.pose


if __name__ == "__main__":
    model = Model()
    model.spawn()
    print(model.get_pose())
    model.delete()