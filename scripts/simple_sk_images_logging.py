#!/usr/bin/env python

import roslib
roslib.load_manifest('tf')
import rospy
import tf
import sys, os
import cv2
import yaml
import rosbag
import numpy as np
from std_msgs.msg import Header, String
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
# from strands_navigation_msgs.msg import TopologicalMap
from skeleton_tracker.msg import joint_message, skeleton_tracker_state, skeleton_message, skeleton_complete
from mongodb_store.message_store import MessageStoreProxy
import sensor_msgs.msg
from cv_bridge import CvBridge
import getpass, datetime
import actionlib
from scitos_ptu.msg import *
import strands_gazing.msg
import topological_navigation.msg
from mary_tts.msg import maryttsAction, maryttsGoal

class SkeletonImageLogger(object):
    """Used to store rgb images/skeleton data recorded during the deployment
       Also needs to request consent from the person who was stored, before keeping it.
    """

    def __init__(self, detection_threshold = 1000, camera='head_xtion', database='message_store', collection='consent_images'):
        # directory to store images
        self.dir1 = '/home/' + getpass.getuser() + '/SkeletonImages/'
        self.camera = camera
        self.frame = 1
        # opencv stuff
        self.cv_bridge = CvBridge()

        # listeners
        rospy.Subscriber('/'+self.camera+'/rgb/sk_tracks', sensor_msgs.msg.Image, callback=self.rgb_sk_callback, queue_size=10)

    def rgb_sk_callback(self, msg1):
        rgb = self.cv_bridge.imgmsg_to_cv2(msg1, desired_encoding="passthrough")
        rgb = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        if self.frame < 10:
            cv2.imwrite(self.dir1+'image_000'+str(self.frame)+'.jpg',rgb)
        elif self.frame < 100:
            cv2.imwrite(self.dir1+'image_00'+str(self.frame)+'.jpg',rgb)
        elif self.frame < 1000:
            cv2.imwrite(self.dir1+'image_0'+str(self.frame)+'.jpg',rgb)
        elif self.frame < 10000:
            cv2.imwrite(self.dir1+'image_'+str(self.frame)+'.jpg',rgb)
        self.frame += 1


if __name__ == '__main__':
    rospy.init_node('simple_skeleton_image_logger', anonymous=True)

    sk_images = SkeletonImageLogger(detection_threshold = 100)
    while not rospy.is_shutdown():
        pass
