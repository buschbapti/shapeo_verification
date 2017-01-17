#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from shapeo_verification.srv import *
#from shapeo_verification.msg import * #same with obj_pose for compatibility
import numpy as np
#import tf
import os
from copy import deepcopy
import cv2
import time
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from shapeo_vision import verify_toy
#from rotations import compose_rotationZXY, decompose_rotationZXY
#from quaternion import quaternion_from_matrix

bridge = CvBridge()
image_frame = []
listener = None
n_cams = 1
rgb_topic = '/usb_cam1/image_raw'
#rgb_topic = '/kinect/rgb/image_color'

#cam_matrices = ['cams_folder/rgb_Default.xml']
#cam_train_matrix = ['cams_folder/rgb_Default.xml']
#cams_to_world = ['cams_folder/calibration_tf_single_cam.xml']

def verification_process(request):
    global n_cams, send_image, depth_image, image_frame, depth_frame
    print('The requested verification shapeo id is: ')
    print(request)
    simulate=0#not real rostopics.
    shape_id = Int32()
    shape_id = deepcopy(request.shapeo_verify.data)
    
    homedir = os.getenv("HOME")
    #objFile = '%s/Objects/objectNames.txt' % (homedir)
    shapeo_verified = Int32()
    
    if simulate==1:
        shapeo_verified.data = np.int32(np.round(np.random.rand()))
    else:
        start_time = time.time()
        image_list=[]
        for i in range(int(n_cams)):
            scene = []
            send_image = 1
            while send_image>0:
                doNothing = 1
            scene = deepcopy(image_frame)
            image_list.append(scene)
        end_time = time.time()
        print ('Image capture took %f seconds' % (end_time-start_time))
        verification, verification_confidence, excessive_insertion = verify_toy(shape_id, 
                                                       image_list) 
        shapeo_verified.data = np.uint8(verification)
    print(shapeo_verified)
    if shapeo_verified.data == 1:
        print('Verification confirmed')
    else:
        print('Verification failed')
    return ShapeoResponse(shapeo_verified)
    
def verification_server():
    rospy.init_node('verify_shapeo_node')
    global listener, n_cams, rgb_topic, depth_topic, send_image, depth_image
    
    #listener = tf.TransformListener()
    send_image = 0
    depth_image = 0
    #sub_depth_1 = rospy.Subscriber(scene_name, Image, depthframeCallback)
    #sub_depth_1 = rospy.Subscriber(depth_topic, Image, depthframeCallback)
    sub_rgb_1 = rospy.Subscriber(rgb_topic, Image, frameCallback)
                
    g = rospy.Service('verify_shapeo', Shapeo, verification_process) # verify_shapeo is name of service
    print "Ready to verify insertion."
    rospy.spin()

def frameCallback(image):
    global image_frame, send_image
    if send_image==1:
        image_frame = bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
        image_frame = np.array(image_frame)
        send_image=0
    
def depthframeCallback(image):
    global depth_frame, depth_image
    if depth_image==1:
        depth_frame = bridge.imgmsg_to_cv(image, '16UC1')
        depth_frame = np.array(depth_frame, dtype=np.uint16)
        depth_frame = np.double(np.copy(depth_frame))/1000.0
        depth_image=0
    
if __name__ == "__main__":
    verification_server()