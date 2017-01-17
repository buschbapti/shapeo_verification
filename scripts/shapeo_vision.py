#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
#import tf
import os
from copy import deepcopy
import cv2
import time
from std_msgs.msg import Int32
import math
from scipy import ndimage as nd
#from rotations import compose_rotationZXY, decompose_rotationZXY
#from quaternion import quaternion_from_matrix

MEAN_HUE = 26.0 #hues between MEAN_HUE-HUE_TOL will be accepted
HUE_TOL  = 3.0
BB_CUBE = [560,310,1100,590] #Bbox for detecting the cube in the image.
centx_max = 450.0
centx_min = 70.0
centy_max = 350.0
centy_min = 50.0
general_width = 243.0
general_height = 100.0
height_tol = 10#all in pixels
max_angle = 35#in angles

def get_rotation(shape_id):
    '''
    The amount of rotation in image plane for the given shape.
    Due to kinematics, this will be different for each shape.
    '''
    rotation_angles=[0, 0, 0, 0, 0]
    rotation_angle = rotation_angles[shape_id]
    return rotation_angle

def rotate_image(image, angle, channels):
      '''Rotate image "angle" degrees.
    
      How it works:
        - Creates a blank image that fits any rotation of the image. To achieve
          this, set the height and width to be the image's diagonal.
        - Copy the original image to the center of this blank image
        - Rotate using warpAffine, using the newly created image's center
          (the enlarged blank image center)
        - Translate the four corners of the source image in the enlarged image
          using homogenous multiplication of the rotation matrix.
        - Crop the image according to these transformed corners
      '''
    
      diagonal = int(math.sqrt(pow(image.shape[0], 2) + pow(image.shape[1], 2)))
      offset_x = (diagonal - image.shape[0])/2
      offset_y = (diagonal - image.shape[1])/2
      if channels==3:
          dst_image = np.zeros((diagonal, diagonal, channels), dtype='uint8')
          dst_image[offset_x:(offset_x + image.shape[0]), \
                offset_y:(offset_y + image.shape[1]), \
                :] = image
      else:
          dst_image = np.zeros((diagonal, diagonal))
          dst_image[offset_x:(offset_x + image.shape[0]), \
                offset_y:(offset_y + image.shape[1])] = image
      image_center = (diagonal/2, diagonal/2)
    
      R = cv2.getRotationMatrix2D(image_center, angle, 1.0)
      
      dst_image = cv2.warpAffine(dst_image, R, (diagonal, diagonal), flags=cv2.INTER_LINEAR)
    
      # Calculate the rotated bounding rect
      x0 = offset_x
      x1 = offset_x + image.shape[0]
      x2 = offset_x
      x3 = offset_x + image.shape[0]
    
      y0 = offset_y
      y1 = offset_y
      y2 = offset_y + image.shape[1]
      y3 = offset_y + image.shape[1]
    
      corners = np.zeros((3,4))
      corners[0,0] = x0
      corners[0,1] = x1
      corners[0,2] = x2
      corners[0,3] = x3
      corners[1,0] = y0
      corners[1,1] = y1
      corners[1,2] = y2
      corners[1,3] = y3
      corners[2:] = 1
    
      c = np.dot(R, corners)
    
      x = int(c[0,0])
      y = int(c[1,0])
      left = x
      right = x
      up = y
      down = y
    
      for i in range(4):
        x = int(c[0,i])
        y = int(c[1,i])
        if (x < left): left = x
        if (x > right): right = x
        if (y < up): up = y
        if (y > down): down = y
      if (up<0): up=0
      if (down<0): down=0
      if (left<0): left=0
      if (right<0): right=0
      
      h = down - up
      w = right - left
      
      if channels==3:    
          cropped = np.zeros((w, h, channels), dtype='uint8')
          cropped[:, :, :] = dst_image[left:(left+w), up:(up+h), :]
      else:
          cropped = np.zeros((w, h))
          cropped[:, :] = dst_image[left:(left+w), up:(up+h)]
      return cropped

def cv_imshow(image_temp):
    WINDOW_NAME = 'Press any key to close'
    cv2.namedWindow(WINDOW_NAME)
    cv2.startWindowThread()
    # Display an image
    cv2.imshow(WINDOW_NAME,image_temp)
    cv2.waitKey(0) 
    cv2.destroyAllWindows()

def median_thres(gray,sigma=3.0,thres=0.0):
    img_proc = nd.filters.median_filter(gray, size=(np.int8(sigma),np.int8(sigma)))
    inds = ((np.abs(img_proc[:,:])>thres))
    img_proc = img_proc*0.0
    img_proc[inds] = 255
    return img_proc

def gauss_thres(img,sigma=3.0,thres=0.0):
    img_proc = nd.filters.gaussian_filter(img, np.double(sigma))
    inds = ((np.abs(img_proc[:,:])>thres))
    img_proc = img_proc*0.0
    img_proc[inds] = 255
    return img_proc

    
def verify_toy(shape_id, image_list):
    #obj_name = request.find_gesture
    for image in image_list:
        channels = np.size(image,2)
        rotation_angle = get_rotation(shape_id-1)
        if rotation_angle!=0:
            im_rotated = rotate_image(image, rotation_angle, channels)
        else:
            im_rotated = deepcopy(image)
            new_image = im_rotated[np.int32(np.size(im_rotated,0)/2-np.size(image,0)/2):
                               np.int32(np.size(im_rotated,0)/2+np.size(image,0)/2),
                               np.int32(np.size(im_rotated,1)/2-np.size(image,1)/2):
                               np.int32(np.size(im_rotated,1)/2+np.size(image,1)/2),
                               :]
            
        #cv_imshow(im_rotated)
        cv2.imwrite('/home/c7031091/Documents/3rdHand/verification/exp1/imgs/obj99_wrong_insertion2.png',new_image)
        hue_image = cv2.cvtColor(new_image, cv2.cv.CV_BGR2HSV)
        cv2.imwrite('/home/c7031091/Documents/3rdHand/verification/exp1/imgs/obj99_wrong_insertion2huebgr.png',hue_image)
        gray = hue_image[:,:,0]
        cv2.imwrite('/home/c7031091/Documents/3rdHand/verification/exp1/imgs/obj99_wrong_insertion2huebgrgray.png',gray)
        gray[gray<MEAN_HUE-HUE_TOL]=0
        gray[gray>MEAN_HUE+HUE_TOL]=0
        cv2.imwrite('/home/c7031091/Documents/3rdHand/verification/exp1/imgs/obj99_wrong_insertion2huebgrgraymasked.png',gray)
        img_proc = median_thres(gray,sigma=7.0)
        cv2.imwrite('/home/c7031091/Documents/3rdHand/verification/exp1/imgs/obj99_wrong_insertion2huebgrgraymaskednew.png',img_proc)
        grayd = np.uint8(gray*(img_proc/255))
        labeled_array, num_features = nd.label(grayd[BB_CUBE[1]:BB_CUBE[3],BB_CUBE[0]:BB_CUBE[2]])
        hists = nd.histogram(labeled_array, 0, num_features, num_features, labeled_array)
        objLabel = np.argmax(hists)
        labeled_array[labeled_array!=objLabel]=0
        labeled_array[labeled_array==objLabel]=1
        labeled_array_img=np.uint8(labeled_array*255)
        #newlabels = gauss_thres(labeled_array_img,sigma=3.0,thres=40.0)
        newlabels  = deepcopy(labeled_array_img)
        newlabels[newlabels>0]=1
        labeled_array = np.uint8(newlabels)
        cv2.imwrite('/home/c7031091/Documents/3rdHand/verification/exp1/imgs/obj99_wrong_insertion2huebgrgraylabels.png',labeled_array_img)
        newmaskedimage = deepcopy(new_image[BB_CUBE[1]:BB_CUBE[3],BB_CUBE[0]:BB_CUBE[2],:])
        for k in range(3):
            newmaskedimage[:,:,k] = new_image[BB_CUBE[1]:BB_CUBE[3],BB_CUBE[0]:BB_CUBE[2],k]*labeled_array
        cv2.imwrite('/home/c7031091/Documents/3rdHand/verification/exp1/imgs/obj99_wrong_insertion2huebgrnewmask.png',newmaskedimage)
        
        ret,thresh = cv2.threshold(labeled_array_img,127,255,0)
        contours,hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cnt = contours[0]
        rect = cv2.minAreaRect(cnt)
        print(rect)
        centroid = rect[0]
        dims = rect[1]
        angle = rect[2]
        
        verification = 0
        verification_confidence = 0.85
        larger_dim = np.max(dims)
        smaller_dim = np.min(dims)
        ratio = larger_dim/general_width
        expected_height = ratio*general_height
        excessive_insertion = expected_height - smaller_dim
        if centroid[0]<centx_max and centroid[0]>centx_min:
            if centroid[1]<centy_max and centroid[1]>centy_min:
                if smaller_dim>expected_height-height_tol and smaller_dim<expected_height+height_tol:
                    if np.abs(angle)<max_angle:
                        verification=1
        '''
        newmaskedimagegray = cv2.cvtColor(newmaskedimage, cv2.cv.CV_BGR2GRAY)
        cv_imshow(newmaskedimagegray)
        ret,thresh = cv2.threshold(newmaskedimagegray,30,255,0)
        contours,hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cnt = contours[0]
        rect = cv2.minAreaRect(cnt)
        '''
    return verification, verification_confidence, excessive_insertion
    
if __name__ == "__main__":
    shape_id = 1
    img = cv2.imread('/home/c7031091/Documents/3rdHand/verification/exp1/imgs/obj99_wrong_insertion2.png')
    image_list = []
    image_list.append(img)
    verify_toy(shape_id, image_list)