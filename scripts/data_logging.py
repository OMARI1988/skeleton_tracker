#!/usr/bin/env python

import roslib
roslib.load_manifest('tf')
import rospy
import tf
import sys
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion
# from strands_navigation_msgs.msg import TopologicalMap
from skeleton_tracker.msg import joint_message, skeleton_tracker_state, skeleton_message, skeleton_complete
# from mongodb_store.message_store import MessageStoreProxy
import sensor_msgs.msg
import cv2
from cv_bridge import CvBridge
import os
import datetime


class SkeletonManager(object):

    def __init__(self):
        self.baseFrame = '/head_xtion_depth_optical_frame'
        self.joints = [
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

        # directory to store the data
        self.date = str(datetime.datetime.now().date())
        self.dir1 = '/home/lucie02/Datasets/Lucie/'+self.date+'/'
        print 'checking if folder exists:',self.dir1
        if not os.path.exists(self.dir1):
            print '  -create folder:',self.dir1
            os.makedirs(self.dir1)
        # get the last skeleton recorded
        self.sk_mapping = {}

        # flags to make sure we recived every thing
        self._flag_robot = 0
        self._flag_rgb = 0
        self._flag_rgb_sk = 0
        # self._flag_depth = 0

        # opencv stuff
        self.cv_bridge = CvBridge()

        # listeners
        rospy.Subscriber("/robot_pose", Pose, callback=self.robot_callback, queue_size=10)
        rospy.Subscriber('skeleton_data/incremental', skeleton_message,callback=self.incremental_callback, queue_size = 1)
        rospy.Subscriber('skeleton_data/complete', skeleton_complete,callback=self.complete_callback, queue_size = 10)
        rospy.Subscriber("/head_xtion/rgb/image_color", sensor_msgs.msg.Image, callback=self.rgb_callback, queue_size=10)
        rospy.Subscriber("/camera/rgb/sk_tracks", sensor_msgs.msg.Image, callback=self.rgb_sk_callback, queue_size=10)
        # rospy.Subscriber("/camera/depth/image", sensor_msgs.msg.Image, callback=self.depth_callback, queue_size=10)


    def robot_callback(self, msg):
        self.robot_pose = msg
        if self._flag_robot == 0:
            print 'robot pose recived'
            self._flag_robot = 1

    def incremental_callback(self, msg):
        self.inc_sk = msg
        print self.inc_sk.uuid
        if self._flag_robot and self._flag_rgb and self._flag_rgb_sk:
            if self.inc_sk.uuid not in self.sk_mapping:
                self.sk_mapping[self.inc_sk.uuid] = {}
                self.sk_mapping[self.inc_sk.uuid]['frame'] = 1
                self.sk_mapping[self.inc_sk.uuid]['time'] = str(datetime.datetime.now().time()).split('.')[0]+'_'
                t = self.sk_mapping[self.inc_sk.uuid]['time']
                print '  -new skeletong detected with id:',self.inc_sk.uuid
                print '  -creating folder:',t+self.inc_sk.uuid
                if not os.path.exists(self.dir1+t+self.inc_sk.uuid):
                    os.makedirs(self.dir1+t+self.inc_sk.uuid)
                    os.makedirs(self.dir1+t+self.inc_sk.uuid+'/rgb')
                    os.makedirs(self.dir1+t+self.inc_sk.uuid+'/rgb_sk')
                    os.makedirs(self.dir1+t+self.inc_sk.uuid+'/robot')
                    os.makedirs(self.dir1+t+self.inc_sk.uuid+'/skeleton')

            t = self.sk_mapping[self.inc_sk.uuid]['time']
            if os.path.exists(self.dir1+t+self.inc_sk.uuid):
                # setup saving dir and frame
                d = self.dir1+t+self.inc_sk.uuid+'/'
                f = self.sk_mapping[self.inc_sk.uuid]['frame']
                if f < 10:          f_str = '0000'+str(f)
                elif f < 100:          f_str = '000'+str(f)
                elif f < 1000:          f_str = '00'+str(f)
                elif f < 10000:          f_str = '0'+str(f)
                elif f < 100000:          f_str = str(f)

                # save rgb image
                cv2.imwrite(d+'rgb/rgb_'+f_str+'.jpg',self.rgb)
                cv2.imwrite(d+'rgb_sk/sk_'+f_str+'.jpg',self.rgb_sk)

                # save robot_pose
                f1 = open(d+'robot/robot_'+f_str+'.txt','w')
                f1.write('position\n')
                f1.write('x:'+str(self.robot_pose.position.x)+'\n')
                f1.write('y:'+str(self.robot_pose.position.y)+'\n')
                f1.write('z:'+str(self.robot_pose.position.z)+'\n')
                f1.write('orientation\n')
                f1.write('x:'+str(self.robot_pose.orientation.x)+'\n')
                f1.write('y:'+str(self.robot_pose.orientation.y)+'\n')
                f1.write('z:'+str(self.robot_pose.orientation.z)+'\n')
                f1.write('w:'+str(self.robot_pose.orientation.w)+'\n')
                f1.close()

                # save skeleton data
                f1 = open(d+'skeleton/skl_'+f_str+'.txt','w')
                # print self.inc_sk.joints[0]
                f1.write('time:'+str(self.inc_sk.joints[0].time.secs)+','+str(self.inc_sk.joints[0].time.nsecs)+'\n')
                for i in self.inc_sk.joints:
                    f1.write(i.name+'\n')
                    f1.write('position\n')
                    f1.write('x:'+str(i.pose.position.x)+'\n')
                    f1.write('y:'+str(i.pose.position.y)+'\n')
                    f1.write('z:'+str(i.pose.position.z)+'\n')
                    f1.write('orientation\n')
                    f1.write('x:'+str(i.pose.orientation.x)+'\n')
                    f1.write('y:'+str(i.pose.orientation.y)+'\n')
                    f1.write('z:'+str(i.pose.orientation.z)+'\n')
                    f1.write('w:'+str(i.pose.orientation.w)+'\n')
                f1.close()

                # update frame number
                if self.inc_sk.uuid in self.sk_mapping:
                    self.sk_mapping[self.inc_sk.uuid]['frame'] += 1

    def complete_callback(self, msg):
        print '  -stopped logging user:',msg.uuid
        self.sk_mapping.pop(msg.uuid,None)
        # self.robot_pose = msg

    def rgb_callback(self, msg1):
        rgb = self.cv_bridge.imgmsg_to_cv2(msg1, desired_encoding="passthrough")
        self.rgb = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        if self._flag_rgb == 0:
            print 'rgb recived'
            self._flag_rgb = 1

    def rgb_sk_callback(self, msg1):
        rgb = self.cv_bridge.imgmsg_to_cv2(msg1, desired_encoding="passthrough")
        self.rgb_sk = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        if self._flag_rgb_sk == 0:
            print 'rgb+sk recived'
            self._flag_rgb_sk = 1

    # def depth_callback(self, msg):
    #     self.depth = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    #     if self._flag_depth == 0:
    #         print 'depth recived'
    #         self._flag_depth = 1

if __name__ == '__main__':
    rospy.init_node('skeleton_publisher', anonymous=True)

    sk_manager = SkeletonManager()
    while not rospy.is_shutdown():
        if str(datetime.datetime.now().date()) != sk_manager.date:
            print 'new day!'
            sk_manager.date = str(datetime.datetime.now().date())
            sk_manager.dir1 = '/home/lucie02/Datasets/Lucie/'+sk_manager.date+'/'
            print 'checking if folder exists:',sk_manager.dir1
            if not os.path.exists(sk_manager.dir1):
                print '  -create folder:',sk_manager.dir1
                os.makedirs(sk_manager.dir1)

        pass
