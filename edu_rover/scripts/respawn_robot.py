#! /usr/bin/env python3

import rospy
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest
from geometry_msgs.msg import Pose

rospy.init_node('spawn_model')

print("Waiting for Service /gazebo/spawn_urdf_model")
rospy.wait_for_service('/gazebo/spawn_urdf_model')

spawn_model_service = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
req = SpawnModelRequest()
req.model_name = "edu_rover"
req.model_xml = open('/home/ros/catkin_ws/src/edu_rover/urdf/edu_rover.urdf','r').read()
#"-file $(find edu_rover)/urdf/edu_rover.urdf"
#(find catkin_ws/src/edu_rover/urdf/edu_rover.urdf)
#open('/usr/share/gazebo-9/models/ground_plane/model.sdf', 'r').read()
req.robot_namespace = "edu_rover"
Position = Pose()
Position.position.x = 0.0
Position.position.y = 0.0
Position.position.z = 1.0
Position.orientation.x = -1.0
Position.orientation.y = 0.0
Position.orientation.z = 0.0
Position.orientation.w = 0.0
req.initial_pose = Position
req.reference_frame = 'world'

print("Spawning Model"+str(req.model_name))
status_message = spawn_model_service(req)
print(status_message)
