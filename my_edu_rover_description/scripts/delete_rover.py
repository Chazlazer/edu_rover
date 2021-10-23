#! /usr/bin/env python3

import rospy
from gazebo_msgs.srv import DeleteModel, DeleteModelRequest

rospy.init_node('remove_model_service_client')
print("Waiting for service /gazebo/delete_model")

rospy.wait_for_service("/gazebo/delete_model")
delete_model_service = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
kk = DeleteModelRequest()
kk.model_name = 'edu_rover'
print("Deleteing model" + str(kk))
status_message = delete_model_service(kk)
print(status_message)