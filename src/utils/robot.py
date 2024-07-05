#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Range
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from gazebo_msgs.srv import GetModelState
import matplotlib.pyplot as plt
import numpy as np
import time

from default_config import DEFAULT_CONFIG
cfg = DEFAULT_CONFIG

PI = 3.1415926535897


class Robot:
    def __init__(self, x=0.0, y=0.0, theta=0.0, model_name='robot'):
        self.model_name = model_name
        self.sensor_distance = 0
        rospy.init_node('vector_controller', anonymous=True)
        self.laser_subscriber = rospy.Subscriber('/vector/laser', Range, self.laser_callback)
        self.velocity_publisher = rospy.Publisher('/vector/cmd_vel', Twist, queue_size=10)
        time.sleep(2)
        self.stop()
        self.set_state(x, y, theta)
        

    def set_state(self, x, y, theta):
        state_msg = ModelState()
        state_msg.model_name = self.model_name
        state_msg.pose.position.x = x
        state_msg.pose.position.y = y
        state_msg.pose.position.z = 0.0
        q = quaternion_from_euler(0.0, 0.0, theta)
        state_msg.pose.orientation.x = q[0]
        state_msg.pose.orientation.y = q[1]
        state_msg.pose.orientation.z = q[2]
        state_msg.pose.orientation.w = q[3]

        rospy.wait_for_service('/gazebo/set_model_state')
        print('$$$ Service: "/gazebo/set_model_state" for set called.')
        try:
            set_state = rospy.ServiceProxy(
                '/gazebo/set_model_state', SetModelState)
            resp = set_state(state_msg)
        except rospy.ServiceException as e:
            print("$$$ Service call failed: %s" % e)

    def get_state(self):
        rospy.wait_for_service('/gazebo/get_model_state')
        print('$$$ Service: "/gazebo/set_model_state" for get called.')
        try:
            get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            state = get_state(model_name=self.model_name)
            x = state.pose.position.x
            y = state.pose.position.y
            z = state.pose.position.z
            rot_q = state.pose.orientation
            roll, pitch, theta = euler_from_quaternion(
                [rot_q.x, rot_q.y, rot_q.z, rot_q.w])
            return x, y, z, theta
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def laser_callback(self, data):
        self.sensor_distance = data.range

    
    def stop(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

    def set_speed(self, vx=0.0, vz=0.0):
        vel_msg = Twist()
        vel_msg.linear.x = vx
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = vz
        self.velocity_publisher.publish(vel_msg)

    def translate(self, time, vx=0.02):
        t0 = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - t0 <= time:
            if self.sensor_distance < cfg["VECTOR_LENGTH"]:
                self.stop()
            else:
                self.set_speed(vx)
        self.stop()

    def rotate(self, w, angle):  # w(|w|):degree/sec, angle:degree
        if (w == 0 or angle == 0):
            return
        time = abs(angle / w) * 2  # *2 for 90deg instead of 45
        angular_speed = w*PI/180

        if angle < 0:
            z_speed = -abs(angular_speed)
        elif angle > 0:
            z_speed = abs(angular_speed)
        else:
            z_speed = 0

        t0 = rospy.Time.now().to_sec()
        while (rospy.Time.now().to_sec() - t0 < time):
            self.set_speed(vz=z_speed)
        self.stop()

    def plot_robot(self):
        # fig,axe = plt.gcf(),plt.gca()
        x, y, z, theta = self.get_state()
        print(f'$$$ plot_robot: x={x:0.3f}, y={y:0.3f}, theta={theta:0.3f}')
        # print(x,y)
        plt.plot(x, y, color='green', marker="o", markersize=8)
        plt.plot(*self.get_sensor_line(x, y, theta), c="blue")

    def get_sensor_line(self, x, y, theta, range=cfg["SENSOR_MAX_DISTANCE"]):
        x1 = x
        x2 = x + range * np.cos(theta)
        y1 = y
        y2 = y + range * np.sin(theta)
        return [(x1, x2), (y1, y2)]
