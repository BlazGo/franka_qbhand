import rospy

namespace = rospy.get_param("arm_id")
print(namespace)
com_qb = rospy.get_param(f"{namespace}_model_spawner/com_qb", default="0")
print(com_qb)