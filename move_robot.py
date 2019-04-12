#!/usr/bin/env python

# Created on: December, 2019
# Author(s): Pradheep Krishna Muthukrishnan Padmanabhan
# Copyright (c) [2019]
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from gazebo_msgs.srv import *
from std_msgs.msg import String
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_commander.conversions import pose_to_list
import tf.transformations
import numpy as np

class Manipulation():
	def __init__(self, model_state):
		self.pose_x = model_state.pose.position.x
		self.pose_y = model_state.pose.position.y
		self.pose_z = model_state.pose.position.z
		self.ori_x = model_state.pose.orientation.x
		self.ori_y = model_state.pose.orientation.y
		self.ori_z = model_state.pose.orientation.z
		self.ori_w = model_state.pose.orientation.w
		moveit_commander.roscpp_initialize(sys.argv)
		self.robot = moveit_commander.RobotCommander()
		self.scene = moveit_commander.PlanningSceneInterface()
		self.group = moveit_commander.MoveGroupCommander("arm")
		self.gripper = moveit_commander.MoveGroupCommander("gripper")

		self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
										  moveit_msgs.msg.DisplayTrajectory)
		self.display_trajectory = moveit_msgs.msg.DisplayTrajectory()

	def transform_calculate(self, obj, base):
		obj_trans = np.dot(tf.transformations.translation_matrix((obj.pose.position.x, obj.pose.position.y, obj.pose.position.z)), tf.transformations.quaternion_matrix([obj.pose.orientation.x, obj.pose.orientation.y, obj.pose.orientation.z, obj.pose.orientation.w]))
		base_trans = np.dot(tf.transformations.translation_matrix((base.pose.position.x, base.pose.position.y, base.pose.position.z)), tf.transformations.quaternion_matrix([base.pose.orientation.x, base.pose.orientation.y, base.pose.orientation.z, obj.pose.orientation.w]))
		result_trans = np.dot(tf.transformations.inverse_matrix(base_trans), obj_trans)
		return result_trans


	def move_group_pick(self, compute_ik, transformation_mat):
		t = geometry_msgs.msg.Pose()
		position = tf.transformations.translation_from_matrix(transformation_mat)
		orientation = tf.transformations.quaternion_from_matrix(transformation_mat)
		t.position.x = position[0]
		t.position.y = position[1]
		t.position.z = position[2]
		t.orientation.x = orientation[0]
		t.orientation.y = orientation[1]
		t.orientation.z = orientation[2]
		t.orientation.w = orientation[3] 
		# With this function, we will be able t achieve the pre grasp position 
		scene = PlanningSceneInterface()
		p = PoseStamped()
		start = rospy.get_time()
		seconds = rospy.get_time()
		box_name = "obj1"
		while (seconds - start < 5) and not rospy.is_shutdown():
		  # Test if the box is in attached objects
		  attached_objects = scene.get_attached_objects([box_name])
		  is_attached = len(attached_objects.keys()) > 0
		  is_known = box_name in scene.get_known_object_names()
		  seconds = rospy.get_time()
		self.group.set_named_target("initial")
		self.group.go()
		request = GetPositionIKRequest()
		request.ik_request.group_name = "arm"
		request.ik_request.ik_link_name = "wrist_3_link"
		request.ik_request.attempts = 20
	    #request.ik_request.pose_stamped.header.frame_id = "base"
		request.ik_request.pose_stamped.header.frame_id = "ur10_base_link"
	    
	    #Set the desired orientation for the end effector HERE
		request.ik_request.pose_stamped.pose.position.x = t.position.x -0.2
		request.ik_request.pose_stamped.pose.position.y = t.position.y 
		request.ik_request.pose_stamped.pose.position.z = t.position.z -0.4
		request.ik_request.pose_stamped.pose.orientation.x = -0.707
		request.ik_request.pose_stamped.pose.orientation.y = 0 
		request.ik_request.pose_stamped.pose.orientation.z = 0
		request.ik_request.pose_stamped.pose.orientation.w = 0.707
		response = compute_ik(request)
		group = self.group
		group.set_pose_target(request.ik_request.pose_stamped)
		group.go()
		self.gripper.set_named_target("close")
		self.gripper.go()
		rospy.sleep(8)
		self.group.set_named_target("initial")
		self.group.go()

	def move_group_place(self, compute_ik, transformation_mat):
		t1 = geometry_msgs.msg.Pose()
		position = tf.transformations.translation_from_matrix(transformation_mat)
		orientation = tf.transformations.quaternion_from_matrix(transformation_mat)
		t1.position.x = position[0]
		t1.position.y = position[1]
		t1.position.z = position[2]
		t1.orientation.x = orientation[0]
		t1.orientation.y = orientation[1]
		t1.orientation.z = orientation[2]
		t1.orientation.w = orientation[3] 
		request1 = GetPositionIKRequest()
		request1.ik_request.group_name = "arm"
		request1.ik_request.ik_link_name = "wrist_3_link"
		request1.ik_request.attempts = 20
		request1.ik_request.pose_stamped.header.frame_id = "ur10_base_link"
		print(t1.position.x,t1.position.y,t1.position.z)
		request1.ik_request.pose_stamped.pose.position.x = t1.position.x 
		request1.ik_request.pose_stamped.pose.position.y = t1.position.y 
		request1.ik_request.pose_stamped.pose.position.z = t1.position.z 
		request1.ik_request.pose_stamped.pose.orientation.x = -0.707
		request1.ik_request.pose_stamped.pose.orientation.y = 0 
		request1.ik_request.pose_stamped.pose.orientation.z = 0
		request1.ik_request.pose_stamped.pose.orientation.w = 0.707
		response1 = compute_ik(request1)
		group = self.group
		group.set_pose_target(request1.ik_request.pose_stamped)
		group.go()
		self.group.set_named_target("finish")
		group.go()
		self.gripper.set_named_target("open")
		self.gripper.go()
		self.group.set_named_target("initial")
		self.group.go()
		moveit_commander.roscpp_shutdown()




	def movebase_client(self):
		client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
		while not client.wait_for_server(rospy.Duration(5)):
			rospy.loginfo("Waiting for the move_base action server to come up")

		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = "map"
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose.position.x = self.pose_x+ 0.75
		goal.target_pose.pose.position.y = self.pose_y
		goal.target_pose.pose.position.z = self.pose_z
		goal.target_pose.pose.orientation.w = 3.14
		
		client.send_goal(goal)
		wait = client.wait_for_result()
		if not wait:
			rospy.logerr("Action server not available!")
			rospy.signal_shutdown("Action server not available!")
		else:
			return client.get_result()


def main():
	roscpp_initialize(sys.argv)
	rospy.init_node('move_group_python_interface', anonymous=True)
	get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
	compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
	rospy.wait_for_service("/gazebo/get_model_state")
	current_model_state = get_model_state('coke_can','world')
	current_model_state_robot = get_model_state('mmo_700','world')
	current_model_state_table = get_model_state('cafe_table','world')
	manip = Manipulation(current_model_state)
	manip1 = Manipulation(current_model_state_table)
	flag = 0

	while not rospy.is_shutdown():
		trans = manip.transform_calculate(current_model_state, current_model_state_robot)
		manip.move_group_pick(compute_ik, trans)
		result = manip1.movebase_client()
		if(result):
			trans1 = manip1.transform_calculate(current_model_state_table, current_model_state_robot)
			manip1.move_group_place(compute_ik, trans1) 
			flag = 1

if __name__=='__main__':
	main()

