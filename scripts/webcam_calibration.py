#!/usr/bin/env python
import rospy


class CameraCalibration(object):

    def __init__(self):
        
        rospy.init_node('camera_calibration', anonymous=True)
        rospy.Subscriber('/camera_driver/image_raw', Image, callback)

        while not rospy.is_shutdown():
            rospy.spin()



if __name__ == '__main__':
    cameracalibration = CameraCalibration
