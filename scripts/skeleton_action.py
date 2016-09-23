#! /usr/bin/env python
import roslib
import sys, os
import rospy
import yaml
import actionlib
import rosbag
import getpass, datetime
import shutil
from std_msgs.msg import String
from scitos_ptu.msg import *
from skeleton_publisher import SkeletonManager
from data_logging import SkeletonImageLogger
from skeleton_tracker.msg import skeletonAction, skeletonActionResult, skeleton_message
from skeleton_tracker.srv import *
from std_srvs.srv import Empty, EmptyResponse
from consent_tsc.msg import ManageConsentAction, ConsentResult, ManageConsentGoal

class skeleton_server(object):
    def __init__(self):
        # Start server
        rospy.loginfo("Skeleton Publisher starting an action server")
        # self.skeleton_msg = None
        rospy.Subscriber('skeleton_data/incremental', skeleton_message,callback=self.incremental_callback, queue_size = 10)
        self._as = actionlib.SimpleActionServer("skeleton_action", skeletonAction, \
                                                    execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        self.sk_publisher = SkeletonManager()
        self.image_logger = SkeletonImageLogger(detection_threshold = 300)
        self.skeleton_msg = skeleton_message()  #default empty
        self.filepath = os.path.join(roslib.packages.get_pkg_dir("skeleton_tracker"), "config")
        try:
            self.config = yaml.load(open(os.path.join(self.filepath, 'config.ini'), 'r'))
            print "config file loaded", self.config.keys()
        except:
            print "no config file found"

        # PTU state - based upon current_node callback
        self.ptu_action_client = actionlib.SimpleActionClient('/SetPTUState', PtuGotoAction)
        self.ptu_action_client.wait_for_server()

        # publishers
        # self.publish_rec = rospy.Publisher('skeleton_data/recording_started', String, queue_size = 1)
        # self.publish_rec.publish("init")


    def consent_client(self, duration):
        rospy.loginfo("Creating consent client")
        ret = None
        start = rospy.Time.now()
        end = rospy.Time.now()
        try:
            consent_client = actionlib.SimpleActionClient('manage_consent', ManageConsentAction)
            if consent_client.wait_for_server(timeout=rospy.Duration(10)):
                goal = ManageConsentGoal()
                consent_client.send_goal(goal)

                # here you should check whether you've been preempted, shutdown etc. while waiting for consent
                while not self._as.is_preempt_requested() and (end - start).secs < duration:

                    if consent_client.wait_for_result(timeout = rospy.Duration(1)):
                        result = consent_client.get_result()
                        int_result = result.result.result

                        if int_result == ConsentResult.DEPTH_AND_RGB:
                            # print 'depth+rgb'
                            ret = "everything"

                        elif int_result == ConsentResult.DEPTH_ONLY:
                            # print 'just depth'
                            ret = "depthskel"
                        else:
                            # print 'no consent'
                            ret = "nothing"
                        break
                    end = rospy.Time.now()

                if (end - start).secs >= duration:
                    print "timed out"

                if self._as.is_preempt_requested():
                    consent_client.cancel_all_goals()
            else:
                rospy.logwarn('No manage consent server')
        except Exception, e:
            rospy.logwarn('Exception when trying to manage consent: %s' % e)
        return ret


    def signal_start_of_recording(self):
        rospy.wait_for_service('signal_recording_started', timeout=10)
        signal_recording_started = rospy.ServiceProxy('signal_recording_started', Empty)
        # tell the webserver to say we've started recording
        signal_recording_started()


    def return_to_main_webpage(self):
        rospy.wait_for_service('return_to_main_webpage', timeout=10)
        main_webpage_return = rospy.ServiceProxy('return_to_main_webpage', Empty)
        # tell the webserver to go back to the main page - if no consent was requested
        main_webpage_return()


    def execute_cb(self, goal):
        self.signal_start_of_recording()
        duration = goal.duration
        start = rospy.Time.now()
        end = rospy.Time.now()
        self.image_logger.stop_image_callbacks = 0   #start the image callbacks in the logger
        self.sk_publisher._initialise_data()
        self.sk_publisher.robot_pose_flag = 1

        print goal
        self.set_ptu_state(goal.waypoint)
        consented_uuid = ""
        self.skeleton_msg.uuid = ""

        consent_msg = None
        request_consent = 0
        first_time = 1

        while (end - start).secs < duration.secs:
            if self._as.is_preempt_requested():
                 break

            #Make sure that the message changes before calling the logger.
            skel_msg = self.skeleton_msg
            self.skeleton_msg = skeleton_message() #clear / default msg

            if consent_msg is None and request_consent is 0:
                self.sk_publisher.publish_skeleton()
                if first_time:
                    rospy.sleep(0.1)  # wait until something is published
                    first_time = 0

                #when a skeleton incremental msg is received
                if skel_msg.uuid is not "":
                    #prev_uuid = skel_msg.uuid
                    #self.sk_publisher.logged_uuid = prev_uuid
                    request_consent = self.image_logger.callback(skel_msg, goal.waypoint)
                    if request_consent is 1:
                        consented_uuid = skel_msg.uuid

            elif consent_msg is not None:
                print "breaking loop for: %s" % consented_uuid
                break

            elif request_consent is 1:
                self.reset_ptu()
                print "Consent requested: %s" % consented_uuid
                new_duration = duration.secs - (end - start).secs
                #print "dur:", new_duration, type(new_duration), start.secs, end.secs, duration.secs
                consent_msg = self.consent_client(new_duration)
                print "consent returned: %s: %s" % (consent_msg, consented_uuid)

            skel_msg.uuid = ""
            end = rospy.Time.now()

        # after the action reset ptu and stop publisher
        print "exited loop"
        if consent_msg is None:
            consent_msg = "nothing"

        self.reset_all()

        # if no skeleton was recorded for the threshold
        if request_consent is 0:
            self.return_to_main_webpage()

        # try:
        #     self.image_logger.bag_file.close()
        # except AttributeError:
        #     print "no bag file to close"

        if self._as.is_preempt_requested():
            print "The action is being preempted, cancelling everything. \n"
            self.return_to_main_webpage()
            return self._as.set_preempted()
        self._as.set_succeeded(skeletonActionResult())

        try:
            proxy = rospy.ServiceProxy("/delete_images_service", DeleteImages)
            if consented_uuid != "":
                req = DeleteImagesRequest(str(end), consented_uuid, str(consent_msg))
                proxy(req)
                print "deleted..."
        except rospy.ServiceException:
            print "deleter service is not running. Moved if consent was given."
            #if consented_uuid != "":
            self.move_consented_data(consented__uuid, consent_msg)
            #self.move_consented_data(skel_msg.uuid, consent_msg)

        print "finished action\n"


    def move_consented_data(self, uuid, consent):
        """Even if deleter is not running, move consented data"""
        dataset = '/home/' + getpass.getuser() +'/SkeletonDataset/pre_consent/'
        dataset_path = os.path.join(dataset, str(datetime.datetime.now().date()))
        dataset_consented_path = os.path.join('/home', getpass.getuser(), 'SkeletonDataset/SafeZone')
        if not os.path.exists(dataset_consented_path):
            os.makedirs(dataset_consented_path)

        # find the specific recording to keep (either most images or most recent)
        try:
            for d in os.listdir(dataset_path):
                if uuid in d:
                    location = os.path.join(dataset_path, d)
                    if "nothing" not in consent:
                        new_location = os.path.join(dataset_consented_path, d)
                        os.rename(location, new_location)
        except:
            rospy.logerr("File(s) or directory(ies) not be found - not moved: %s: %s" % uuid, dataset_consented_path)


    def reset_all(self):
        # self.image_logger.stop_image_callbacks = 0   #stop the image callbacks in the logger
        self.sk_publisher.robot_pose_flag = 0        #stop the callbacks in the pub
        self.reset_ptu()
        # self.publish_rec.publish("finished")   #the cb for this shows the recording webpage
        self.image_logger.go_back_to_where_I_came_from()

        ## remove data stored in the publisher (save memory)
        self.sk_publisher._initialise_data()
        self.sk_publisher.accumulate_data = {}


    def incremental_callback(self, msg):
        # print ">", msg.uuid  # doesnt receive empty messages anymore
        self.skeleton_msg = msg

    def reset_ptu(self):
        ptu_goal = PtuGotoGoal();
        ptu_goal.pan = 0
        ptu_goal.tilt = 0
        ptu_goal.pan_vel = 30
        ptu_goal.tilt_vel = 30
        self.ptu_action_client.send_goal(ptu_goal)
        self.ptu_action_client.wait_for_result()

    def set_ptu_state(self, waypoint):
        ptu_goal = PtuGotoGoal();
        try:
            ptu_goal.pan = self.config[waypoint]['pan']
            ptu_goal.tilt = self.config[waypoint]['tilt']
            ptu_goal.pan_vel = self.config[waypoint]['pvel']
            ptu_goal.tilt_vel = self.config[waypoint]['tvel']
            self.ptu_action_client.send_goal(ptu_goal)
            self.ptu_action_client.wait_for_result()
        except KeyError:
            self.reset_ptu()


if __name__ == "__main__":
    rospy.init_node('skeleton_action_server')

    skeleton_server()
    rospy.spin()
