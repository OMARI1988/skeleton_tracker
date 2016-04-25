#!/usr/bin/env python
import roslib
import rospy
import sys, os
from std_msgs.msg import String
from mongodb_store.message_store import MessageStoreProxy
import sensor_msgs.msg
import datetime
import getpass


class ImageDeleter(object):

    def __init__(self, database='message_store', collection='people_skeleton'):

        # directory to delete the data from
        self.date = str(datetime.datetime.now().date())
        self.dir1 = '/home/' + getpass.getuser() + '/SkeletonDataset/'+self.date+'/'

        self.time_now = datetime.datetime.now()
        self.del_window_start = (self.time_now - datetime.timedelta(hours=1)).time()
        self.del_window_end = (self.time_now - datetime.timedelta(hours=0.5)).time()

        print 'folder', self.dir1
        print 'date', self.date
        print 'time', self.time_now
        print 'del ', self.del_window_start,  self.del_window_end

        self.consent_req = None
        self.consent_ret = None

        # mongo store
        self.msg_store = MessageStoreProxy(collection=collection, database=database)
        self.publish_consent_req = rospy.Publisher('skeleton_data/consent', String, queue_size = 10)

        # listeners
        rospy.Subscriber("/skeleton_data/consent_req", String, callback=self.consent_req_callback, queue_size=10)
        rospy.Subscriber('skeleton_data/consent_ret', String, callback=self.consent_ret_callback, queue_size = 10)

        self.remover_of_images()


    def remover_of_images(self):
        print "removing..."
        print 'del ', self.del_window_start,  self.del_window_end


    def consent_req_callback(self, msg):
        self.consent_req = msg


    def consent_ret_callback(self, msg):
        self.consent_ret = msg
        if self.consent_ret == "everything":
            rospy.sleep(10.*60)

        elif self.consent_ret == "nothing":


if __name__ == '__main__':
    rospy.init_node('skeleton_image_logger', anonymous=True)

    deleter = ImageDeleter()

    while not rospy.is_shutdown():
        deleter.remover_of_images()
