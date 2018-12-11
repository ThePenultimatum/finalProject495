#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Int8MultiArray
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class Process(object):
    def __init__(self):
        rospy.init_node('img_processor')
        self.cmd_sub = rospy.Subscriber('cmd', String, self.cmd_callback)
        self.traj_pub = rospy.Publisher('traj', Int8MultiArray, queue_size = 1)
        self.res_pub = rospy.Publisher('result', String, queue_size = 5)
    
    def cmd_callback(self, msg):
        print('ok')
        if msg.data == 'S':
            img = rospy.wait_for_message('camera_driver/image_raw', Image)
            bridge = CvBridge()
            try:
                cv_img = bridge.imgmsg_to_cv2(img, "bgr8")
            except CvBridgeError, err:
                rospy.logerr(err)
                return        
            h, w = cv_img.shape[:2]
            frame_size = 100
            if h > w:
                h_new = frame_size
                w_new = (frame_size * w) / h
            else:
                w_new = frame_size
                h_new = (frame_size * h) / w
            resized_img = cv2.resize(cv_img, (h_new, w_new), cv2.INTER_AREA)
            cannied_img = cv2.Canny(resized_img, 200, 400)
            traj = []
            r, c = cannied_img.shape
            edges = np.array(cannied_img)
            queue = []
            for i in range(1, r - 1):
                for j in range(1, c - 1):
                    if edges[i, j] > 0:
                        queue.append([i, j])
                        traj.append([i, j])
                        while queue:
                            v = queue.pop()
                            if edges[v[0], v[1]] > 0:
                                edges[v[0], v[1]] = 0
                                traj.append(v)
                                v0_1 = v[0] - 1
                                v1_1 = v[1] - 1
                                neighbors = np.argwhere(edges[v0_1: v[0] + 2, v1_1: v[1] + 2] > 0)
                                # Begin making it skinny
                                if len(neighbors) > 1:                       
                                    for k in neighbors:
                                        if v0_1 + k[0] != v[0] and k[1] + v1_1 != v[1]:
                                            edges[v0_1 + k[0], v[1]] = 0
                                            edges[v[0], k[1] + v1_1] = 0
                                    neighbors = np.argwhere(edges[v0_1: v[0] + 2, v1_1: v[1] + 2] > 0)
                                # End make it skinny
                                if len(neighbors) > 0:      
                                    neighbors += np.array([[v0_1, v1_1]] * len(neighbors))
                                    #print(neighbors.tolist())
                                    queue += neighbors.tolist()  
                                else:
                                    traj.append([-1, -1]) 
            traj_msg = Int8MultiArray()
            traj_msg.data = traj                 
            self.traj_pub.publish(traj_msg)
            self.res_pub.publish('T') # Trajectory ready
            # Shut down the node if possible  
            print(traj_msg.data)        

def main():
    processor = Process()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == "__main__":
    main()
