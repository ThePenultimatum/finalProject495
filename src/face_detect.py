#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Dec 11 15:52:46 2018

@author: vnoelifant
"""
# import modules
import numpy as np
import cv2 as cv


# create function to detect and crop a face in image
def face_crop(image):
    
    # load face training data
    face_cascade = cv.CascadeClassifier('haarcascade_frontalface_default.xml')
    
    #read in the image
    img = cv.imread(image)
    
    #resize image if needed
    #cv.resize(img,(320,240))
    
    # convert the baseImage to a gray-based image
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    
    # detect face
    faces = face_cascade.detectMultiScale(gray, 1.3, 5)
    
    # add padding to final rectangle to increase its size around face
    # add width padding
    pad_w = 20
    # add height padding
    pad_h = 20
    
    # create rectangle for face
    for f in faces:
        # define vertices of face rectangle
        x,y,w,h = [v for v in f]
        cv.rectangle(img,(x-pad_w,y-pad_h),(x+w+pad_w,y+h+pad_h),(255,0,0),2) # blue color
        # create cropped faces
        face = img[y-pad_h:y+h+pad_h,x-pad_w:x+w+pad_w]
        
        # remove rectangle by further cropping
        # face = face[5:100, 5:100]

    # show uncropped face 
    cv.imshow('img',img)
    # show the cropped face
    cv.imshow('face',face)
    # display the window infinitely until any keypress
    cv.waitKey(0)
    cv.destroyAllWindows()
    return face

# main part of script execution
if __name__ == "__main__":
    face_crop("face.jpg")