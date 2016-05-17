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
        self.nav_goal_waypoint = None
        self.camera = camera
        self.baseFrame = '/'+self.camera+'_depth_optical_frame'
        self.joints = ['head', 'neck', 'torso', 'left_shoulder', 'left_elbow', 'left_hand',
                'left_hip', 'left_knee', 'left_foot', 'right_shoulder', 'right_elbow',
                'right_hand', 'right_hip', 'right_knee', 'right_foot']

        # directory to store the data
        self.date = str(datetime.datetime.now().date())
        # self.dir1 = '/home/lucie02/Datasets/Lucie/'+self.date+'/'

        self.dir1 = '/home/' + getpass.getuser() + '/SkeletonDataset/pre_consent/' + self.date+'/'
        print 'checking if folder exists:', self.dir1
        if not os.path.exists(self.dir1):
            print '  -create folder:',self.dir1
            os.makedirs(self.dir1)

        self.filepath = os.path.join(roslib.packages.get_pkg_dir("skeleton_tracker"), "config")
        try:
            self.config = yaml.load(open(os.path.join(self.filepath, 'config.ini'), 'r'))
        except:
            print "no config file found"

        # PTU state - based upon current_node callback
        self.ptu_action_client = actionlib.SimpleActionClient('/SetPTUState', PtuGotoAction)
        self.ptu_action_client.wait_for_server()

        # get the last skeleton recorded
        self.sk_mapping = {}

        # flags to make sure we recived every thing
        self._flag_robot = 0
        self._flag_rgb = 0
        self._flag_rgb_sk = 0
        self._flag_depth = 0
        self.request_sent_flag = 0
        self.after_a_number_of_frames = detection_threshold
        self.consent_ret = None

        # opencv stuff
        self.cv_bridge = CvBridge()

        # mongo store
        self.msg_store = MessageStoreProxy(collection=collection, database=database)

        # publishers
        self.publish_consent_req = rospy.Publisher('skeleton_data/consent_req', String, queue_size = 10)
        self.publish_consent_req.publish("init")
        self.publish_consent_pose = rospy.Publisher('skeleton_data/consent_pose', PoseStamped, queue_size = 10, latch=True)

        # listeners
        # rospy.Subscriber("/current_node", String, callback=self.curr_node_callback, queue_size=1)
        # rospy.Subscriber("/closest_node", String, callback=self.close_node_callback, queue_size=1)
        rospy.Subscriber("/robot_pose", Pose, callback=self.robot_callback, queue_size=10)
        # rospy.Subscriber('skeleton_data/incremental', skeleton_message,callback=self.incremental_callback, queue_size = 10)
        rospy.Subscriber('skeleton_data/complete', skeleton_complete,callback=self.complete_callback, queue_size = 10)
        rospy.Subscriber("/skeleton_data/consent_ret", String, callback=self.consent_ret_callback, queue_size=1)
        rospy.Subscriber('/'+self.camera+'/rgb/image_color', sensor_msgs.msg.Image, callback=self.rgb_callback, queue_size=10)
        rospy.Subscriber('/'+self.camera+'/rgb/sk_tracks', sensor_msgs.msg.Image, callback=self.rgb_sk_callback, queue_size=10)
        rospy.Subscriber('/'+self.camera+'/rgb/white_sk_tracks', sensor_msgs.msg.Image, callback=self.white_sk_callback, queue_size=10)
        rospy.Subscriber('/'+self.camera+'/depth/image' , sensor_msgs.msg.Image, self.depth_callback, queue_size=10)

		# PTU state
        self.ptu_action_client = actionlib.SimpleActionClient('/SetPTUState', PtuGotoAction)
        self.ptu_action_client.wait_for_server()

        # gazing action server
        self.gaze_client()

        # topo nav move
        self.nav_client()

        # speak
        self.speak()


    def robot_callback(self, msg):
        self.robot_pose = msg
        if self._flag_robot == 0:
            print 'robot pose recived'
            self._flag_robot = 1


    def callback(self, msg, waypoint):
        self.inc_sk = msg
        if str(datetime.datetime.now().date()) != self.date:
            print 'new day!'
            self.date = str(datetime.datetime.now().date())
            self.dir1 = '/home/' + getpass.getuser() + '/SkeletonDataset/pre_consent/'+self.date+'/'
            print 'checking if folder exists:',self.dir1
            if not os.path.exists(self.dir1):
                print '  -create folder:',self.dir1
                os.makedirs(self.dir1)
        # print self.inc_sk.uuid
        if self._flag_robot and self._flag_rgb and self._flag_rgb_sk:
            if self.inc_sk.uuid not in self.sk_mapping:
                self.sk_mapping[self.inc_sk.uuid] = {}
                self.sk_mapping[self.inc_sk.uuid]['frame'] = 1
                self.sk_mapping[self.inc_sk.uuid]['time'] = str(datetime.datetime.now().time()).split('.')[0]+'_'
                t = self.sk_mapping[self.inc_sk.uuid]['time']
                print '  -new skeleton detected with id:', self.inc_sk.uuid
                # print '  -creating folder:',t+self.inc_sk.uuid
                if not os.path.exists(self.dir1+t+self.inc_sk.uuid):
                    os.makedirs(self.dir1+t+self.inc_sk.uuid)
                    os.makedirs(self.dir1+t+self.inc_sk.uuid+'/rgb')
                    os.makedirs(self.dir1+t+self.inc_sk.uuid+'/depth')
                    os.makedirs(self.dir1+t+self.inc_sk.uuid+'/rgb_sk')
                    os.makedirs(self.dir1+t+self.inc_sk.uuid+'/robot')
                    os.makedirs(self.dir1+t+self.inc_sk.uuid+'/skeleton')

                    # create the empty bag file (closed in /skeleton_action)
                    self.bag_file = rosbag.Bag(self.dir1+t+self.inc_sk.uuid+'/detection.bag', 'w')

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
                # todo: make these rosbags sometime in the future
                cv2.imwrite(d+'rgb/rgb_'+f_str+'.jpg', self.rgb)
                cv2.imwrite(d+'depth/depth_'+f_str+'.jpg', self.xtion_img_d_rgb)
                cv2.imwrite(d+'rgb_sk/sk_'+f_str+'.jpg', self.rgb_sk)

                try:
                    self.bag_file.write('rgb', self.rgb_msg)
                    self.bag_file.write('depth', self.depth_msg)
                    self.bag_file.write('rgb_sk', self.rgb_sk_msg)
                except:
                    rospy.logwarn("Can not write rgb, depth, and rgb_sk to a bag file.")

                # save robot_pose in bag file
                x=float(self.robot_pose.position.x)
                y=float(self.robot_pose.position.y)
                z=float(self.robot_pose.position.z)
                xo=float(self.robot_pose.orientation.x)
                yo=float(self.robot_pose.orientation.y)
                zo=float(self.robot_pose.orientation.z)
                wo=float(self.robot_pose.orientation.w)
                p = Point(x, y, z)
                q = Quaternion(xo, yo, zo, wo)
                robot = Pose(p,q)
                self.bag_file.write('robot', robot)

                # save robot_pose in text file
                f1 = open(d+'robot/robot_'+f_str+'.txt','w')
                f1.write('position\n')
                f1.write('x:'+str(x)+'\n')
                f1.write('y:'+str(y)+'\n')
                f1.write('z:'+str(z)+'\n')
                f1.write('orientation\n')
                f1.write('x:'+str(xo)+'\n')
                f1.write('y:'+str(yo)+'\n')
                f1.write('z:'+str(zo)+'\n')
                f1.write('w:'+str(wo)+'\n')
                f1.close()

                # save skeleton data in bag file
                #x=float(self.robot_pose.position.x)
                #y=float(self.robot_pose.position.y)
                #z=float(self.robot_pose.position.z)
                #xo=float(self.robot_pose.orientation.x)
                #yo=float(self.robot_pose.orientation.y)
                #zo=float(self.robot_pose.orientation.z)
                #wo=float(self.robot_pose.orientation.w)
                #p = Point(x, y, z)
                #q = Quaternion(xo, yo, zo, wo)
                #skel = Pose(p,q)
                #bag.write('skeleton', skel)


                # save skeleton datain text file
                f1 = open(d+'skeleton/skl_'+f_str+'.txt','w')
                # print self.inc_sk.joints[0]
                f1.write('time:'+str(self.inc_sk.joints[0].time.secs)+'.'+str(self.inc_sk.joints[0].time.nsecs)+'\n')
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

                #publish the gaze request of person on every detection:
                if self.inc_sk.joints[0].name == 'head':
                    head = Header(frame_id='head_xtion_depth_optical_frame')
                    look_at_pose = PoseStamped(header = head, pose=self.inc_sk.joints[0].pose)
                    self.publish_consent_pose.publish(look_at_pose)
                #self.gazeClient.send_goal(self.gazegoal)

                # all this should happen given a good number of detections:
                print "%s out of %d frames are obtained" % (self.sk_mapping[self.inc_sk.uuid]['frame'], self.after_a_number_of_frames)
                if self.sk_mapping[self.inc_sk.uuid]['frame'] == self.after_a_number_of_frames and self.request_sent_flag == 0:
                    print "storing the %sth image to mongo for the webserver..." % self.after_a_number_of_frames
                    # Skeleton on white background
                    query = {"_meta.image_type": "white_sk_image"}
                    white_sk_to_mongo =  self.msg_store.update(message=self.white_sk_msg, meta={'image_type':"white_sk_image"}, message_query=query, upsert=True)
                    # Skeleton on rgb background
                    query = {"_meta.image_type": "rgb_sk_image"}
                    rgb_sk_img_to_mongo = self.msg_store.update(message=self.rgb_sk_msg, meta={'image_type':"rgb_sk_image"}, message_query=query, upsert=True)
                    # Skeleton on depth background
                    query = {"_meta.image_type": "depth_image"}
                    depth_img_to_mongo = self.msg_store.update(message=self.depth_msg, meta={'image_type':"depth_image"}, message_query=query, upsert=True)

                    consent_msg = "Check_consent_%s" % (t)
                    print consent_msg
                    # I think this should be a service call - so it definetly returns a value.
                    self.request_sent_flag = 1
                    # move and speak: (if no target waypoint, go to original waypoint)
                    # self.reset_ptu()
                    try:
                        self.navgoal.target = self.config[waypoint]['target']
                    except:
                        self.navgoal.target = waypoint
                    if self.navgoal.target != waypoint:
                        self.nav_goal_waypoint = waypoint  #to return to after consent
                        self.navClient.send_goal(self.navgoal)
                        result = self.navClient.wait_for_result()
                        if not result:
                            self.go_back_to_where_I_came_from()

                    self.publish_consent_req.publish(consent_msg)
                    rospy.sleep(0.1)
                    if self.request_sent_flag:
                        self.speaker.send_goal(maryttsGoal(text=self.speech))
                    while self.consent_ret is None:
                        rospy.sleep(0.1)

                    # Move Eyes - look up and down to draw attension.
        return self.consent_ret

    def go_back_to_where_I_came_from(self):
        if self.nav_goal_waypoint is not None and self.nav_goal_waypoint != self.config[self.nav_goal_waypoint]['target']:
            try:
                self.navgoal.target = self.config[self.nav_goal_waypoint]['target']
            except:
                print "nav goal not set - staying at %s" % self.navgoal.target
            self.navClient.send_goal(self.navgoal)
            self.navClient.wait_for_result()

    def consent_ret_callback(self, msg):
        if self.request_sent_flag == 0: return
        print "got consent ret callback, %s" % msg
        self.consent_ret=msg
        # self.request_sent_flag = 0
        # when the request is returned, go back to previous waypoint
        self.speaker.send_goal(maryttsGoal(text="Thank you"))
        self.go_back_to_where_I_came_from()

    def reset_ptu(self):
        ptu_goal = PtuGotoGoal();
        ptu_goal.pan = 0
        ptu_goal.tilt = 0
        ptu_goal.pan_vel = 30
        ptu_goal.tilt_vel = 30
        self.ptu_action_client.send_goal(ptu_goal)

    def gaze_client(self):
        rospy.loginfo("Creating gaze client")
        _as = actionlib.SimpleActionClient('gaze_at_pose', strands_gazing.msg.GazeAtPoseAction)
        _as.wait_for_server()
        gazegoal = strands_gazing.msg.GazeAtPoseGoal()
        gazegoal.topic_name = '/skeleton_data/consent_pose'
        gazegoal.runtime_sec = 60
        _as.send_goal(gazegoal)

    def nav_client(self):
        rospy.loginfo("Creating nav client")
        self.navClient = actionlib.SimpleActionClient('topological_navigation', topological_navigation.msg.GotoNodeAction)
        self.navClient.wait_for_server()
        self.navgoal = topological_navigation.msg.GotoNodeGoal()

    def speak(self):
        self.speaker = actionlib.SimpleActionClient('/speak', maryttsAction)
        got_server = self.speaker.wait_for_server(rospy.Duration(1))
        while not got_server:
            rospy.loginfo("Data Consent is waiting for marytts action...")
            got_server = self.speaker.wait_for_server(rospy.Duration(1))
            if rospy.is_shutdown():
                return
        self.speech = "Please may I get your consent to store data I just recorded."

    def complete_callback(self, msg):
        # print '  -stopped logging user:', msg.uuid
        self.sk_mapping.pop(msg.uuid, None)
        # self.robot_pose = msg

    def rgb_callback(self, msg1):
        self.rgb_msg = msg1
        rgb = self.cv_bridge.imgmsg_to_cv2(msg1, desired_encoding="passthrough")
        self.rgb = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        if self._flag_rgb == 0:
            print 'rgb recived'
            self._flag_rgb = 1

    def rgb_sk_callback(self, msg1):
        self.rgb_sk_msg = msg1
        rgb = self.cv_bridge.imgmsg_to_cv2(msg1, desired_encoding="passthrough")
        self.rgb_sk = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        if self._flag_rgb_sk == 0:
            print 'rgb+sk recived'
            self._flag_rgb_sk = 1

    def white_sk_callback(self, msg1):
        self.white_sk_msg = msg1

    def depth_callback(self, imgmsg):

        self.depth_msg = imgmsg
        depth_image = self.cv_bridge.imgmsg_to_cv2(imgmsg, desired_encoding="passthrough")
        depth_array = np.array(depth_image, dtype=np.float32)
        cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
        self.xtion_img_d_rgb = depth_array*255
        if self._flag_depth == 0:
            print 'depth recived'
            self._flag_depth = 1


if __name__ == '__main__':
    rospy.init_node('skeleton_image_logger', anonymous=True)

    sk_images = SkeletonImageLogger(detection_threshold = 100)
    while not rospy.is_shutdown():
        pass
