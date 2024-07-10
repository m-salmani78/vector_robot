#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
import numpy as np
import time

from configs import CONFIG


class Robot:
    def __init__(self, x=0.0, y=0.0, theta=0.0, model_name="robot"):
        self.model_name = model_name
        self.sensor_distance = 0
        self.laser_subscriber = rospy.Subscriber(
            "/vector/laser", Range, self.laser_callback
        )
        self.velocity_publisher = rospy.Publisher(
            "/vector/cmd_vel", Twist, queue_size=1
        )
        time.sleep(2)
        self.stop()

    def laser_callback(self, data):
        self.sensor_distance = data.distance / 1000

    def stop(self):
        self.velocity_publisher.publish(0.0, 0.0, 0.0, 0.0)

    def set_speed(self, vl=0.0, vr=0.0):
        self.velocity_publisher.publish(vl, vr, 0.0, 0.0)

    def translate(self, time, vx=50):
        self.set_speed(vx, vx)
        t0 = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - t0 <= time:
            if self.sensor_distance < CONFIG["VECTOR_LENGTH"]:
                self.stop()
            else:
                pass
        self.stop()

    def rotate(self, DEGREE=90):  # 90, -90
        V = 20.0 / 10  # mm/s
        L = 17.6  # cm
        PI = 3.1415926535897
        b = 4.8  # cm

        vl = V if DEGREE > 0 else -V
        vr = -vl
        r = L / (2 * PI)
        w = V / r
        ur = w
        ul = -w
        uw = r / (2 * b) * (ur - ul)
        t = PI / (2 * uw)
        self.set_speed(vr * 10, vl * 10)
        t0 = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - t0 <= t:
            pass
        self.stop()

    def get_sensor_line(self, x, y, theta, range=CONFIG["SENSOR_MAX_DISTANCE"]):
        x1 = x
        x2 = x + range * np.cos(theta)
        y1 = y
        y2 = y + range * np.sin(theta)
        return [(x1, x2), (y1, y2)]
