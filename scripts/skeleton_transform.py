#!/usr/bin/env python

import roslib
roslib.load_manifest('tf')
import rospy
import tf
import sys
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

BASE_FRAME = '/tracker_depth_frame'
JOINTS = [
        'head',
        'neck',
        'torso',
        'left_shoulder',
        'left_elbow',
        'left_hand',
        'left_hip',
        'left_knee',
        'left_foot',
        'right_shoulder',
        'right_elbow',
        'right_hand',
        'right_hip',
        'right_knee',
        'right_foot',
        ]


if __name__ == '__main__':
	rospy.init_node('skeleton_transform', anonymous=True)

	with_logging = rospy.get_param("log_skeleton", "false")

	listener = tf.TransformListener()
	pub_skl = rospy.Publisher('skeleton_data', Marker, queue_size = 10)
	rate = rospy.Rate(15.0)
	person = {}

	#can cope with upto 10 people in the scene
	for subj in range(1,11):
		person[subj] = {}
		person[subj]['flag'] = 0

		for i in JOINTS:
			person[subj][i] = dict()
			person[subj][i]['value'] = [0,0,0]
			person[subj][i]['t_old'] = 0

	while not rospy.is_shutdown():
		for subj in range(1,9):
			for i in JOINTS:
				if listener.frameExists(BASE_FRAME):
					try:
						t = listener.getLatestCommonTime(BASE_FRAME,  "tracker/user_%d/%s" % (subj, i))
						if t != person[subj][i]['t_old']:
							person[subj][i]['t_old'] = t
							person[subj]['flag'] = 1
							(person[subj][i]['value'], rot) = listener.lookupTransform(BASE_FRAME, "tracker/user_%d/%s" % (subj, i), rospy.Time(0))
					except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
						continue

		list_of_subs = [subj for subj in person if person[subj]['flag'] == 1]

		#for subj in range(1,11):
		for subj in list_of_subs:
			msg = Marker()
			msg.id = subj

			#if person[subj]['flag'] == 1:
			# for joint, val in person[subj].items():
			# 	print joint, val

			for i in JOINTS:
				position = Point()
				position.x = person[subj][i]['value'][0]
				position.y = person[subj][i]['value'][1]
				position.z = person[subj][i]['value'][2]
				msg.points.append(position)
			person[subj]['flag'] = 0
			pub_skl.publish(msg)

	rate.sleep()
