#!/usr/bin/env python3
"""Sample program to make Vector move."""

from time import sleep

import rospy
from rospy import Publisher
from anki_vector_ros.msg import RobotStatus
from anki_vector_ros.msg import Drive


def main():
    print("Setting up publishers")
    move_pub = Publisher("/motors/wheels", Drive, queue_size=1)

    # Need small delay to setup publishers
    move_pub.publish(0.0, 0.0, 0.0, 0.0)
    sleep(1.0)
    v=50.0   # mm/s
    dist=150.0 # mm
    dt= dist/v # s

    print("Executing commands")
    itrs=15
    for i in range(itrs):
        print(f'in iteration {i}')
        move_pub.publish(v, v, 0.0, 0.0)
        sleep(dt)
        move_pub.publish(0.0, 0.0, 0.0, 0.0)
        sleep(3.0)

    move_pub.publish(0.0, 0.0, 0.0, 0.0)

if __name__ == "__main__":
    rospy.init_node("vector_hello_world")
    rospy.wait_for_message("/status", RobotStatus)

    main()
