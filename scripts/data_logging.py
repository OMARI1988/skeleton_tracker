#!/usr/bin/env python

import roslib
roslib.load_manifest('tf')
import rospy
import tf
import sys, os
import cv2
import yaml
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
from skeleton_tracker.srv import *

class SkeletonImageLogger(object):
    """Used to store rgb images/skeleton data recorded during the deployment
       Also needs to request consent from the person who was stored, before keeping it.
    """

    def __init__(self, camera='head_xtion', database='message_store', collection='consent_images'):
        self.camera = camera
        self.baseFrame = '/'+self.camera+'_depth_optical_frame'
        self.joints = ['head', 'neck', 'torso', 'left_shoulder', 'left_elbow', 'left_hand',
                'left_hip', 'left_knee', 'left_foot', 'right_shoulder', 'right_elbow',
                'right_hand', 'right_hip', 'right_knee', 'right_foot']

        # directory to store the data
        self.date = str(datetime.datetime.now().date())
        # self.dir1 = '/home/lucie02/Datasets/Lucie/'+self.date+'/'
        self.dir1 = '/home/' + getpass.getuser() + '/SkeletonDataset/'+self.date+'/'
        print 'checking if folder exists:', self.dir1
        if not os.path.exists(self.dir1):
            print '  -create folder:',self.dir1
            os.makedirs(self.dir1)

        self.filepath = os.path.join(roslib.packages.get_pkg_dir("skeleton_tracker"), "config")
        self.config = yaml.load(open(os.path.join(self.filepath, 'config.ini'), 'r'))
        print "config loaded:", self.config

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
        self.after_a_number_of_frames = 10
	    self.consent_ret = None

        # opencv stuff
        self.cv_bridge = CvBridge()

        # mongo store
        self.msg_store = MessageStoreProxy(collection=collection, database=database)

        # publishers
        self.publish_consent_req = rospy.Publisher('skeleton_data/consent_req', String, queue_size = 10)
        self.publish_consent_pose = rospy.Publisher('skeleton_data/consent_pose', PoseStamped, queue_size = 10)
        self.publish_consent_req.publish("init")

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
            self.dir1 = '/home/' + getpass.getuser() + '/SkeletonDataset/'+self.date+'/'
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
                print '  -new skeleton detected with id:',self.inc_sk.uuid
                print '  -creating folder:',t+self.inc_sk.uuid
                if not os.path.exists(self.dir1+t+self.inc_sk.uuid):
                    os.makedirs(self.dir1+t+self.inc_sk.uuid)
                    os.makedirs(self.dir1+t+self.inc_sk.uuid+'/rgb')
                    os.makedirs(self.dir1+t+self.inc_sk.uuid+'/depth')
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
		        # todo: make these rosbags sometime in the future
                cv2.imwrite(d+'rgb/rgb_'+f_str+'.jpg',self.rgb)
                cv2.imwrite(d+'depth/depth_'+f_str+'.jpg',self.xtion_img_d_rgb)
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

		        # all this should happen given a good number of detections:
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
                    self.publish_consent_req.publish(consent_msg)
		            self.request_sent_flag = 1
                    self.gazeClient.send_goal(self.gazegoal)

        		    # move and speak: (if no target, go to original waypoint)
        		    try:
                        self.navgoal.target = self.config[waypoint]['target']
                    except:
			            self.navgoal.target = waypoint
        		    self.navClient.send_goal(self.navgoal)
        		    result = self.navClient.wait_for_result()
                    self.previous_target = self.navgoal.target  #to return to after consent
        		    if not result:
        			    self.go_back_to_where_I_came_from()

        		    if self.request_sent_flag:
                        self.speaker.send_goal(maryttsGoal(text=self.speech))

		    # Move Eyes?
	        #if self.request_sent_flag = 0:

        return self.consent_ret

    def go_back_to_where_I_came_from(self):
    	self.navgoal.target = self.config[self.previous_target]['target']
    	self.navClient.send_goal(self.navgoal)
        self.navClient.wait_for_result()

    def consent_ret_callback(self, msg):
        self.consent_ret=msg
    	self.request_sent_flag = 0
    	self.speaker.send_goal(maryttsGoal(text="Thanks"))
    	# when the request is returned, go back to previous waypoint
    	self.go_back_to_where_I_came_from()
    	del_srv = rospy.ServiceProxy("/delete_images_service", DeleteImages)
        result = del_srv()
        print result

    def gaze_client(self):
        rospy.loginfo("Creating gaze client")
        self.gazeClient = actionlib.SimpleActionClient('gaze_at_pose', strands_gazing.msg.GazeAtPoseAction)
        self.gazeClient.wait_for_server()
        self.gazegoal = strands_gazing.msg.GazeAtPoseGoal()
        self.gazegoal.topic_name = '/skeleton_data/consent_pose'
        self.gazegoal.runtime_sec = 30

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
        print '  -stopped logging user:', msg.uuid
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
        self.xtion_img_d = self.cv_bridge.imgmsg_to_cv2(imgmsg, desired_encoding="passthrough")
        self.xtion_img_d.setflags(write=True) # allow to change the values
        fgmask = cv2.convertScaleAbs(self.xtion_img_d) # cv2 stuff
        self.xtion_img_d_rgb = cv2.cvtColor(fgmask, cv2.COLOR_GRAY2BGR) # cv2 stuff
        if self._flag_depth == 0:
            print 'depth recived'
            self._flag_depth = 1


if __name__ == '__main__':
    rospy.init_node('skeleton_image_logger', anonymous=True)

    sk_images = SkeletonImageLogger()
    while not rospy.is_shutdown():
        pass
