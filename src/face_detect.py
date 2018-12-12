#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Dec 11 15:52:46 2018

@author: vnoelifant
"""

import numpy as np
import cv2 as cv


face_cascade = cv.CascadeClassifier('haarcascade_frontalface_default.xml')
#eye_cascade = cv.CascadeClassifier('haarcascade_eye.xml')

#load webcam (for later, live webcam)
# cap = cv.VideoCapture(0)

# Capture frame-by-frame
# ret, img = cap.read()

img = cv.imread('face.jpg')
#cv.resize(img,(320,240))
# convert the baseImage to a gray-based image
gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

faces = face_cascade.detectMultiScale(gray, 1.3, 5)

padding = 20

for f in faces:
    x,y,w,h = [v for v in f]
    cv.rectangle(img,(x-padding,y-padding),(x+w+padding,y+h+padding),(255,0,0),2) # blue color
    # find eyes in region of face
    #roi_gray = gray[y:y+h, x:x+w]
    #roi_color = img[y:y+h, x:x+w]
    #eyes = eye_cascade.detectMultiScale(roi_gray)
    #for (ex,ey,ew,eh) in eyes:
        #cv.rectangle(roi_color,(ex,ey),(ex+ew,ey+eh),(0,255,0),2)
    sub_face = img[y-padding:y+h+padding,x-padding:x+w+padding]
    face_file = 'face_cropped.jpg'
    cv.imwrite(face_file,sub_face)
    
cv.imshow('face',img)
# display the window infinitely until any keypress
cv.waitKey(0)
cv.destroyAllWindows()