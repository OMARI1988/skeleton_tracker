#! /usr/bin/env python

import sys
import rospy
import actionlib

from skeleton_publisher import SkeletonManager
from skeleton_tracker.msg import skeletonAction


class skeleton_server(object):
    def __init__(self):
        # Start server
        rospy.loginfo("Skeleton Publisher starting an action server")

        self._as = actionlib.SimpleActionServer("skeleton_action", skeletonAction, \
                                                    execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def cond(self):
        return self._as.is_preempt_requested()

    def execute_cb(self, goal):
        sk_manager = SkeletonManager()
        
        if not self.cond(): 
            sk_manager.publish_skeleton()
	    #Split on multiple methods to allow system to stop the action server
        self._as.set_succeeded(skeletonActionResult())


if __name__ == "__main__":
    rospy.init_node('skeleton_action_server')
   
    skeleton_server()
    rospy.spin()
