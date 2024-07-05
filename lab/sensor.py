#!/usr/bin/env python3
"""Detect when fistbump has been done and react accordingly."""

import rospy
from rospy import Subscriber
from anki_vector_ros.msg import Proximity, RobotStatus
import pandas as pd
import numpy as np

# int i = 0

class FistBump:
    def __init__(self):
        self.count=0
        self.list_of_exp = []
        self.proximity_sub = Subscriber("/proximity", Proximity, self.proxim_callback)

    def proxim_callback(self, proximity):
        print(f'\r[{self.count}] {proximity.distance}', end='')
        
        self.count += 1
        
        self.list_of_exp.append(proximity.distance)
        if self.count == 500:
            self.list_of_exp = np.array(self.list_of_exp)
            df = pd.DataFrame(self.list_of_exp)
            df.to_csv("37cm_dark.csv")
            print()
            rospy.signal_shutdown("bye dude")
    

if __name__ == "__main__":
    rospy.init_node("fist_bump_test")
    rospy.wait_for_message("/status", RobotStatus)

    FistBump()
    rospy.spin()