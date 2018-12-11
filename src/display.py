#!/usr/bin/env python

# Copyright (c) 2013-2018, Rethink Robotics Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import intera_interface
import rospy
from sensor_msgs.msg import Image

class Display(object):
        def __init__(self):
            rospy.init_node("displayer", anonymous=True)
            self.head_display = intera_interface.HeadDisplay()
            self.cmd_sub = rospy.Subscriber('camera_driver/image_raw', Image, self.img_callback)

        def img_callback(self, msg):
            self.head_display.display_image(msg)

def main():
    displayer = Display()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()        
