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

        self.data = {} #c urrent tf frame data for 15 joints 
        self.accumulate_data = {} # accumulates multiple tf frames
        self.users = {} # keeps the tracker state message, timepoint and UUID
        
        # logging to mongo:
        self._with_logging = rospy.get_param("~log_skeleton", "false")
        self._message_store = rospy.get_param("~message_store", "people_skeleton")
        
        # listeners:
        self.tf_listener = tf.TransformListener()
        #self.uuid_listener = rospy.Subscriber("/people", user_ID, self.uuid_callback)       
        self.state_listener = rospy.Subscriber("skeleton_data/state", skeleton_tracker_state, self.tracker_state_callback)
        
        # publishers:
        self.publish_skl_incr = rospy.Publisher('skeleton_data/incremental', Marker, queue_size = 10)
        self.publish_skl_comp = rospy.Publisher('skeleton_data/complete', skeleton_complete, queue_size = 10)
        self.rate = rospy.Rate(15.0)

        # initialise data to recieve tf data
        self._initialise_data()
        
        # initialise mongodb client
        if self._with_logging:
            rospy.loginfo("Connecting to mongodb...%s" % self._message_store)
            self._store_client = MessageStoreProxy(collection=self._message_store)

            
    def _initialise_data(self):
        #to cope with upto 10 people in the scene
        for subj in xrange(1,9):
            self.data[subj] = {}
            self.data[subj]['flag'] = 0
            self.users[subj] = {"message": "No message"}
            
            for i in self.joints:
                self.data[subj][i] = dict()
                self.data[subj][i]['value'] = [0,0,0]
                self.data[subj][i]['t_old'] = 0


    def _get_tf_data(self):
        while not rospy.is_shutdown():
            for subj in xrange(1,9):
                joints_found = True
                for i in self.joints:
                    if self.tf_listener.frameExists(self.baseFrame) and joints_found is True:
                        try:
                            tp = self.tf_listener.getLatestCommonTime(self.baseFrame,  "tracker/user_%d/%s" % (subj, i))
                            if tp != self.data[subj][i]['t_old']:
                                self.data[subj][i]['t_old'] = tp
                                self.data[subj]['flag'] = 1
                                (self.data[subj][i]['value'], rot) = self.tf_listener.lookupTransform(self.baseFrame, \
                                                                        "tracker/user_%d/%s" % (subj, i), rospy.Time(0))
                        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                            joints_found = False
                            continue

                #If the tracker_state is 'Out of Scene' publish the accumulated skeleton
                if self.users[subj]["message"] == "Out of Scene" and subj in self.accumulate_data:
                    self._publish_complete_data(subj)
                    self.data[subj]['flag'] = 0
                    
                #print "h ", self.data[subj]['flag'], self.users[subj]["message"]
                    
            
            #For all subjects, publish the incremental skeleton and accumulate into self.data also.
            list_of_subs = [subj for subj in self.data if self.data[subj]['flag'] == 1]
            #print ">>>", list_of_subs
            for subj in list_of_subs:
                if self.users[subj]["message"] != "New": 
                    continue  # this catches cases where they leave the scene but still have /tf data
                
                #print ">", subj
                incr_msg = Marker()
                incr_msg.id = subj
                for i in self.joints:
                    position = Point()
                    position.x = self.data[subj][i]['value'][0]
                    position.y = self.data[subj][i]['value'][1]
                    position.z = self.data[subj][i]['value'][2]
                    incr_msg.points.append(position)
                self.data[subj]['flag'] = 0
                
                #publish the instant frame message on /incremental
                self.publish_skl_incr.publish(incr_msg)
                
                #accumulate the messages
                if self.users[subj]["message"] == "New":
                    self._accumulate_data(subj, incr_msg)
                    
                elif self.users[subj]["message"] == "No message":
                    print "Just published this user. Get over it"
                else:
                    raise RuntimeError("this should never have occured; why is message not `New` or `Out of Scene' ??? ")

            self.rate.sleep()

    def _accumulate_data(self, subj, current_msg):
        # accumulate the multiple skeleton messages until user goes out of scene
        if current_msg.id in self.accumulate_data:
            self.accumulate_data[current_msg.id].append(current_msg)
        else:
            self.accumulate_data[current_msg.id] = [current_msg]


    def _publish_complete_data(self, subj):
        # when user goes "out of scene" publish their accumulated data
        
        msg = skeleton_complete(uuid = self.users[subj]["uuid"], \
                                skeleton_data = self.accumulate_data[subj])
        self.publish_skl_comp.publish(msg)
        rospy.loginfo("User #%s: published as %s" % (subj, msg.uuid))

        # remove the user from the users dictionary and the accumulated data dict.
        self.users[subj]["message"] = "No message"
        del self.accumulate_data[subj]
        del self.users[subj]["uuid"]
        
        if self._with_logging:
            query = {"uuid" : msg.uuid}
            meta = {"map" : "don't know"}
            #self._store_client.insert(traj_msg, meta)
            self._store_client.update(message=msg, message_query=query, meta=meta, upsert=True)


    def tracker_state_callback(self, msg):
        self.users[msg.userID]["uuid"] = msg.uuid
        self.users[msg.userID]["message"] = msg.message
        self.users[msg.userID]["timepoint"] = msg.timepoint
              

    def publish_skeleton(self):
        self._get_tf_data()


if __name__ == '__main__':
    rospy.init_node('skeleton_transform', anonymous=True)

    sk_manager = SkeletonManager()
    sk_manager.publish_skeleton()
