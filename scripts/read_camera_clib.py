#!/usr/bin/env python
from rosparam import upload_params
from yaml import load
import rospy
from std_msgs.msg import Float64MultiArray
import numpy as np

def talker():
    pub = rospy.Publisher('/image_calib', Float64MultiArray, queue_size=10)
    rospy.init_node('read_image_clib', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    yamlfile = load_params_from_yaml('/home/lucie01/.ros/camera_info/rgb_PS1080_PrimeSense.yaml')
    for i in range(len(yamlfile['distortion_coefficients']['data'])):
    	yamlfile['distortion_coefficients']['data'][i] = float(yamlfile['distortion_coefficients']['data'][i])
    print yamlfile['distortion_coefficients']['data']
    while not rospy.is_shutdown():
        msg_to_send = Float64MultiArray()
        msg_to_send.data = yamlfile['distortion_coefficients']['data']
        pub.publish(msg_to_send)
        rate.sleep()

def load_params_from_yaml(full_path):
    f = open(full_path, 'r')
    yamlfile = load(f)
    f.close()
    upload_params('/', yamlfile)
    for key in yamlfile:
	    print key,':', yamlfile[key]
    return yamlfile

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

