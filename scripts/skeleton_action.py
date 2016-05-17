#! /usr/bin/env python
import threading
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

class skeleton_server(object):
    def __init__(self):
        # Start server
        rospy.loginfo("Skeleton Publisher starting an action server")
        self.skeleton_msg = None
        rospy.Subscriber('skeleton_data/incremental', skeleton_message,callback=self.incremental_callback, queue_size = 10)
        self._as = actionlib.SimpleActionServer("skeleton_action", skeletonAction, \
                                                    execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        self.sk_publisher = SkeletonManager()
        self.image_logger = SkeletonImageLogger(detection_threshold = 100)
        self.skeleton_msg = skeleton_message()  #default empty
        self.filepath = os.path.join(roslib.packages.get_pkg_dir("skeleton_tracker"), "config")
        try:
            self.config = yaml.load(open(os.path.join(self.filepath, 'config.ini'), 'r'))
            print "config loaded:", self.config
        except:
            print "no config file found"

        # PTU state - based upon current_node callback
        self.ptu_action_client = actionlib.SimpleActionClient('/SetPTUState', PtuGotoAction)
        self.ptu_action_client.wait_for_server()

        # publishers
        self.publish_rec = rospy.Publisher('skeleton_data/recording_started', String, queue_size = 1)
        self.publish_rec.publish("init")


    def execute_cb(self, goal):
        duration = goal.duration
        start = rospy.Time.now()
        end = rospy.Time.now()
        self.publish_rec.publish("started_rec_callback")   #the cb for this shows the recording webpage
        print goal
        self.set_ptu_state(goal.waypoint)

        prev_uuid = ""
        thread = None
        while (end - start).secs < duration.secs:
            if self.image_logger.request_sent_flag != 1:
                if self._as.is_preempt_requested():
                    self.reset_all()
                    return self._as.set_preempted()

                self.sk_publisher.publish_skeleton()
                rospy.sleep(0.01)  # wait until something is published

                #when a skeleton incremental msg is received
                if self.skeleton_msg.uuid != "":
                    prev_uuid = self.skeleton_msg.uuid
                    self.sk_publisher.logged_uuid = prev_uuid

                    if self.image_logger.request_sent_flag != 1:
                        thread = threading.Thread(
                            target=self.image_logger.callback,
                            args=(self.skeleton_msg, goal.waypoint,)
                        )
                        thread.start()
                        # self.image_logger.callback(self.skeleton_msg, goal.waypoint)
                        #print "consent: ", self.image_logger.consent_ret
            else:
                self.reset_ptu()

                if self.image_logger.consent_ret != None:  #if consent is given:
                    print "got consent"
                    break
                rospy.sleep(0.1)
            end = rospy.Time.now()

        if thread is not None:
            thread.join()
        # after the action reset everything
        self.image_logger.request_sent_flag = 0
        self.reset_all()

        try:
            self.image_logger.bag_file.close()
        except AttributeError:
            print "no bag file to close"

        try:
            previous_consent = self.image_logger.consent_ret.data
        except AttributeError:  # if nothinging is returned :(
            print "no previous consent"
            previous_consent = "everything"

        self.image_logger.consent_ret = None
        self._as.set_succeeded(skeletonActionResult())

        try:
            proxy = rospy.ServiceProxy("/delete_images_service", DeleteImages)
            if prev_uuid != "":
                req = DeleteImagesRequest(str(end), prev_uuid, str(previous_consent))
                print "deleting images..."
                proxy(req)
                print "deleted..."
        except rospy.ServiceException:
            print "deleter service is not running. Cannot delete data."
            if prev_uuid != "":
                self.move_consented_data(prev_uuid, previous_consent)


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
            rospy.logerr("File(s) or directory(ies) can not be found!")


    def reset_all(self):
        self.reset_ptu()
        self.publish_rec.publish("finished")   #the cb for this shows the recording webpage
        self.image_logger.go_back_to_where_I_came_from()

    def incremental_callback(self, msg):
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
