import rospy
import subprocess
from gazebo_msgs.srv import DeleteModel

model_name = "quadrotor"
rospy.wait_for_service('/gazebo/delete_model')
delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
delete_model(model_name)
rospy.loginfo("Model %s deleted successfully", model_name)
subprocess.run(["roslaunch", "hector_quadrotor_gazebo", "spawn_quadrotor.launch"])

# model_path = "/home/burakzdd/catkin_ws/src/hector-quadrotor-noetic/hector_quadrotor/hector_quadrotor_description/urdf/quadrotor.gazebo.xacro"
# pose = Pose()
# pose.position.x = 0
# pose.position.y = 0
# pose.position.z = 0.3
# spawn_model(model_name, model_path, pose)








# model_name = "quadrotor"
# rospy.wait_for_service('/gazebo/delete_model')
# delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
# delete_model(model_name)
# rospy.loginfo("Model %s deleted successfully", model_name)
# # subprocess.run(["roslaunch", "hector_quadrotor_gazebo", "quadrotor_empty_world.launch"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
# model_path = "/home/burakzdd/catkin_ws/src/hector-quadrotor-noetic/hector_quadrotor/hector_quadrotor_description/urdf/quadrotor.gazebo.xacro"
# pose = Pose()
# pose.position.x = 0
# pose.position.y = 0
# pose.position.z = 0.3
# spawn_model(model_name, model_path, pose)
        