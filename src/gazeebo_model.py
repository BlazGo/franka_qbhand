import rospy
import tf
from gazebo_msgs.srv import SpawnModel, DeleteModel, GetModelState
from geometry_msgs.msg import Point, Pose, Quaternion

class Model:
    def __init__(self, name='/home/bg/custom_models/hca_box/model.sdf'):
        """
        __init__ initialises model required services
             
        :param name: define path/name to the file [.sdf] filetype
        """
        
        self.model_name = name

        rospy.wait_for_service("gazebo/spawn_sdf_model")
        self.delete_model = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
        self.spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.get_model_state = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)

    def spawn(self, pose=[0.6, 0, 0.05, 0, 0, 0]):
        quat = tf.transformations.quaternion_from_euler(pose[3], pose[4], pose[5])
        orient = Quaternion(quat[0], quat[1], quat[2], quat[3])
        model_pose = Pose(position = Point(x=pose[0], y=pose[1], z=pose[2]),
                          orientation = orient)

        self.spawn_model(model_name="hca_box",
                         model_xml=open(self.model_name, 'r').read(),
                         robot_namespace='',
                         initial_pose=model_pose,
                         reference_frame='world')
        rospy.sleep(1)

    def delete(self):
        self.delete_model(model_name="hca_box")
        rospy.sleep(1)

    def get_state(self):
        pose = self.get_model_state("hca_box", "link")
        return pose
