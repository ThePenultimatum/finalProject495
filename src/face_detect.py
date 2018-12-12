#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Dec 11 15:52:46 2018

@author: vnoelifant
"""

import numpy as np
import cv2 as cv

def face_crop(image):
    
    # load face training data
    face_cascade = cv.CascadeClassifier('haarcascade_frontalface_default.xml')
    
    #read in image
    img = cv.imread('face.jpg')
    
    #resize image if needed
    #cv.resize(img,(320,240))
    
    # convert the baseImage to a gray-based image
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    
    # detect face
    faces = face_cascade.detectMultiScale(gray, 1.3, 5)
    
    # add padding to final rectangle to increase its size around face
    padding = 20
    
    # create rectangle for face
    for f in faces:
        # define vertices of face rectangle
        x,y,w,h = [v for v in f]
        cv.rectangle(img,(x-padding,y-padding),(x+w+padding,y+h+padding),(255,0,0),2) # blue color
        # create cropped face
        face = img[y-padding:y+h+padding,x-padding:x+w+padding]

     
    # show the cropped face
    cv.imshow('face',face)
    # display the window infinitely until any keypress
    cv.waitKey(0)
    cv.destroyAllWindows()
    return face


if __name__ == "__main__":
    face_crop("face.jpg")