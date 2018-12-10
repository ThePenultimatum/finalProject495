#!/usr/bin/env python
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Point


class Perception(object):

    def __init__(self):

        rospy.init_node('perception', anonymous=True)
        self.point_pub = rospy.Publisher("/me495/target_position", Point, queue_size = 10)

        # AR_tag localization
        self.state = [0, 0, 0, 0]
        self.pointlist = [Point(), Point(), Point(), Point()]
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.callback_ar)
        rospy.sleep(1)

        # Communicate with manipulation node
        rospy.Subscriber('/me495/current-position', Point, self.callback_point)


    def callback_ar(self, alvarMarkers):
        self.state = [0, 0, 0, 0]
        self.pointlist = [Point(), Point(), Point(), Point()]
        for i in range(len(alvarMarkers.markers)):
            for j in range(4):
                if alvarMarkers.markers[i].id == j:
                    self.pointlist[j]=alvarMarkers.markers[i].pose.pose.position
                    self.state[j] = 1


    def callback_point(self, point):
        for i in range(4):
            if self.state[i] == 1:
                x = 0.0
                y = 0.0
                z = 0.0
                for j in range(10):
                    x = x + self.pointlist[i].x
                    y = y + self.pointlist[i].y
                    z = z + self.pointlist[i].z
                    rospy.sleep(0.1)
                self.point = Point()
                self.point.x = x / 10 + 0.15
                self.point.y = y / 10
                self.point.z = z / 10 + 0.2
            else:
                self.point = Point()
                self.point.x = 0
                self.point.y = 0
                self.point.z = 0
            self.point_pub.publish(self.point)


if __name__ == "__main__":
    perception = Perception()
    while not rospy.is_shutdown():
        rospy.spin()
