#!/usr/bin/env python

import roslib
roslib.load_manifest('tf')
import rospy
import tf
import sys
from visualization_msgs.msg import Marker   
from geometry_msgs.msg import Point
from skeleton_tracker.msg import skeleton_tracker_state, skeleton_complete
from mongodb_store.message_store import MessageStoreProxy

class SkeletonManager(object):
   
    def __init__(self):
    
        # self.name = name
        # self.map_info = rospy.get_param("~map_info", "")
        self.baseFrame = '/tracker_depth_frame'
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
                
        self.users = {}
        self.accumulate_data = {}
        
        self.listener = tf.TransformListener()
        self.state_listener = rospy.Subscriber("skeleton_data/state", skeleton_tracker_state, self.state_callback)

        self._with_logging = rospy.get_param("~log_skeleton", "false")
        self.publish_skl_incr = rospy.Publisher('skeleton_data/incremental', Marker, queue_size = 10)         
        self.publish_skl_comp = rospy.Publisher('skeleton_data/complete', skeleton_complete, queue_size = 10)
        self.rate = rospy.Rate(15.0)
   
        self._initialise_data()
        if self._with_logging:
            # rospy.loginfo("Connecting to mongodb...")
            _store_client = MessageStoreProxy(collection="people_skeleton")
            
    def _initialise_data(self):
        self.data = {}
        
        #can cope with upto 10 people in the scene
        for subj in range(1,11):
            self.data[subj] = {}
            self.data[subj]['flag'] = 0
            
            self.users[subj] = ("Nothing", 0)
            
            for i in self.joints:
                self.data[subj][i] = dict()
                self.data[subj][i]['value'] = [0,0,0]
                self.data[subj][i]['t_old'] = 0


    def _publish_data(self):
        while not rospy.is_shutdown():
            for subj in range(1,9):
                for i in self.joints:
                    if self.listener.frameExists(self.baseFrame):
                        try:
                            t = self.listener.getLatestCommonTime(self.baseFrame,  "tracker/user_%d/%s" % (subj, i))
                            if t != self.data[subj][i]['t_old']:
                                self.data[subj][i]['t_old'] = t
                                self.data[subj]['flag'] = 1
                                (self.data[subj][i]['value'], rot) = self.listener.lookupTransform(self.baseFrame, "tracker/user_%d/%s" % (subj, i), rospy.Time(0))
                        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                            continue

                (message, timepoint) = self.users[subj]
                if message == "Out of Scene":
                    if subj in self.accumulate_data:
                        complete_msg = skeleton_complete()
                        complete_msg.userID = subj
                        complete_msg.skeleton_data = self.accumulate_data[subj]
                        print "publishing complete data for: ", subj, " of len: ", len(self.accumulate_data[subj])
                        self.publish_skl_comp.publish(complete_msg)
                        self.users[subj] = ("Nothing", 0)
                        del self.accumulate_data[subj]

            list_of_subs = [subj for subj in self.data if self.data[subj]['flag'] == 1]
            for subj in list_of_subs:
                msg = Marker()
                msg.id = subj

                for i in self.joints:
                    position = Point()
                    position.x = self.data[subj][i]['value'][0]
                    position.y = self.data[subj][i]['value'][1]
                    position.z = self.data[subj][i]['value'][2]
                    msg.points.append(position)
                self.data[subj]['flag'] = 0
                self.publish_skl_incr.publish(msg)
                self._accumulate_data(msg)
                
            self.rate.sleep()
    

    def state_callback(self, msg):
        self.users[msg.userID] = (msg.message, msg.timepoint)
        # print "callback: ", msg.userID
        # print ">>", self.users
              
                
    def _accumulate_data(self, current_msg):
        (message, timepoint) = self.users[current_msg.id]
        #print "accumulating data for: ", current_msg.id, ". currently stored for: ", self.accumulate_data.keys(), message
        
        if message == "New":
            if current_msg.id in self.accumulate_data:
                self.accumulate_data[current_msg.id].append(current_msg)
            else:
                self.accumulate_data[current_msg.id] = [current_msg]
        else:
            print "WHY HERE?"                


    def publish_skeleton(self):
        self._publish_data()


if __name__ == '__main__':
    rospy.init_node('skeleton_transform', anonymous=True)

    sk_manager = SkeletonManager()
    sk_manager.publish_skeleton()
