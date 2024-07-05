#!/usr/bin/env python3
"""Program to rotate Vector by 90 degrees"""

from time import sleep

import rospy
from rospy import Publisher
from anki_vector_ros.msg import RobotStatus
from anki_vector_ros.msg import Drive

def main():
    print("Setting up publishers")
    move_pub = Publisher("/motors/wheels", Drive, queue_size=1)

    # Small delay to setup publishers
    sleep(1)
    
    v=50.0 # mm/s
    dt= 1.506 # s

    print("Executing commands")
    move_pub.publish(-v, v, 0.0, 0.0)
    sleep(dt)
    move_pub.publish(0.0, 0.0, 0.0, 0.0)
    sleep(0.5)

if __name__ == "__main__":
    rospy.init_node("vector_hello_world")
    rospy.wait_for_message("/status", RobotStatus)

    main()
