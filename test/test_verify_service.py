import numpy as np
import rospy
from copy import deepcopy
from std_msgs.msg import Int32
from shapeo_verification.srv import *
import time

obj_poses=[]
obj_names = ['shapeoObj1']
conf_thres = 0.005


def verify_shapeo_call(shape_id):
    print('********************')
    print 'Verify Shapeo'
    rospy.wait_for_service('verify_shapeo')
    arg_to_verify = ShapeoRequest()
    print 'waiting for objects to be verify ...'
    #obj_to_verify = Int32()
    arg_to_verify.shapeo_verify.data = 2#id of the shape to be verified
    #arg_to_verify.append(obj_to_verify)
    try:
        verify_shape = rospy.ServiceProxy('verify_shapeo',Shapeo)
        verify_ret = verify_shape(arg_to_verify)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    print('********************')
    if verify_ret.data==0:
        print('failed')
    elif verify_ret.data==1:
        print('confirmed')
    
if __name__ == '__main__':
    print('started')    
    rospy.init_node('demo_verification')
    shape_id = 1
    verify_shapeo_call(shape_id)
     