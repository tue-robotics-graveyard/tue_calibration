#! /usr/bin/env python
import roslib; roslib.load_manifest('tue_calibration')
import rospy
import sys
import random
import tf
import tf_server
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from tue_calibration.srv import getPose

rospy.init_node('test_calibration_tool')
laser_pose = PoseStamped()
#tf_listener = tf.TransformListener()
tf_listener = tf_server.TFClient()

def getLaserPose(req):
	laser_pose.header.stamp = rospy.Time.now()
	laser_pose.header.frame_id = "/amigo/base_laser"
	laser_pose.pose.position.x = random.uniform(2.5, 3.5)
	laser_pose.pose.position.y = random.uniform(-1.0, 1.0)
	laser_pose.pose.position.z = 0.0
	laser_pose.pose.orientation.x = 0.0
	laser_pose.pose.orientation.y = 0.0
	laser_pose.pose.orientation.z = 0.0
	laser_pose.pose.orientation.w = 1.0
	rospy.loginfo("Laser pose = {0}".format(laser_pose))
	return laser_pose
	
def getKinectPose(req):
	tmp_laser_pose = laser_pose
	# Correct for z-offset
	laser_pose.pose.position.z = laser_pose.pose.position.z - (0.24-0.1015)
	tmp_laser_pose.header.stamp = rospy.Time.now()
	kinect_pose = tf_listener.transformPose("/amigo/top_kinect/openni_rgb_optical_frame", tmp_laser_pose)
	rospy.loginfo("Kinect pose = {0}".format(kinect_pose))
	return kinect_pose

if __name__ == '__main__':
	
	
	
	# Publishers and services
	jointstate_pub = rospy.Publisher('/amigo/joint_states', JointState)
	laser_service = rospy.Service('/laser_line_detector/toggle_line_detector', getPose, getLaserPose)
	kinect_service = rospy.Service('/kinect_checkerboard_detector/toggle_checkerboard_detector', getPose, getKinectPose)
	
	# Fill jointstate topic
	jointstate_msg = JointState()
	jointstate_msg.header.stamp = rospy.Time.now()
	jointstate_msg.name.append("torso_joint")
	jointstate_msg.name.append("neck_pan_joint")
	jointstate_msg.name.append("neck_tilt_joint")
	jointstate_msg.position.append(float(sys.argv[1]))
	jointstate_msg.position.append(float(sys.argv[2]))
	jointstate_msg.position.append(float(sys.argv[3]))
	
	r = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		
		jointstate_pub.publish(jointstate_msg)
		r.sleep()
