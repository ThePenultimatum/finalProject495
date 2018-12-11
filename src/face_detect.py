#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Dec 11 15:52:46 2018

@author: vnoelifant
"""

import numpy as np
import cv2 as cv
face_cascade = cv.CascadeClassifier('haarcascade_frontalface_default.xml')
eye_cascade = cv.CascadeClassifier('haarcascade_eye.xml')

#load webcam (for later, live webcam)
# cap = cv.VideoCapture(0)

# Capture frame-by-frame
# ret, img = cap.read()

img = cv.imread('face.jpg')
#cv.resize(img,(320,240))
# convert the baseImage to a gray-based image
gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

faces = face_cascade.detectMultiScale(gray, 1.3, 5)
for (x,y,w,h) in faces:
    cv.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2) # blue color
    # find eyes in region of face
    roi_gray = gray[y:y+h, x:x+w]
    roi_color = img[y:y+h, x:x+w]
    eyes = eye_cascade.detectMultiScale(roi_gray)
    for (ex,ey,ew,eh) in eyes:
        cv.rectangle(roi_color,(ex,ey),(ex+ew,ey+eh),(0,255,0),2)
cv.imshow('img',img)
# display the window infinitely until any keypress
cv.waitKey(0)
cv.destroyAllWindows()