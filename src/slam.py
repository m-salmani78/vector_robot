#!/usr/bin/env python3
import rospy
from matplotlib import pyplot as plt
from matplotlib import colors
import random
import numpy as np
from utils.robot import Robot
import time
from default_config import DEFAULT_CONFIG
cfg = DEFAULT_CONFIG


class Map:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.robot_grid = [5, 5]
        self.robot_angle = 0
        self.occupancy_grid = [[0 for _ in range(y)] for _ in range(x)]  # 0:None, 1: Robot, 2:Filled, 3:Empty
        self.color_map = colors.ListedColormap(['Gray', 'Green', 'Red', 'White'])
        
    
    def draw_map(self):
        self.occupancy_grid[self.robot_grid[1]][self.robot_grid[0]] = 1
        plt.pcolor(self.occupancy_grid, cmap=self.color_map, edgecolors='k', linewidths=1, vmin=0, vmax=3)
        self.occupancy_grid[self.robot_grid[1]][self.robot_grid[0]] = 0
    
    def update_occupancy_grid(self, sensor_distance):
        for d in np.arange(0.0, sensor_distance, 0.05):
            obstacle_grid_x = self.robot_grid[0]
            obstacle_grid_y = self.robot_grid[1]
            if self.robot_angle == 0:
                obstacle_grid_x = self.robot_grid[0] + round(d / 0.05)
            if self.robot_angle == 90:
                obstacle_grid_y = self.robot_grid[1] + round(d / 0.05)
            if self.robot_angle == 180:
                obstacle_grid_x = self.robot_grid[0] - round(d / 0.05)
            if self.robot_angle == 270:
                obstacle_grid_y = self.robot_grid[1] - round(d / 0.05)
            self.occupancy_grid[obstacle_grid_y][obstacle_grid_x] = 3
        if sensor_distance <= 0.37:
            obstacle_grid_x = self.robot_grid[0]
            obstacle_grid_y = self.robot_grid[1]
            if self.robot_angle == 0:
                obstacle_grid_x = self.robot_grid[0] + round(sensor_distance / 0.05)
            if self.robot_angle == 90:
                obstacle_grid_y = self.robot_grid[1] + round(sensor_distance / 0.05)
            if self.robot_angle == 180:
                obstacle_grid_x = self.robot_grid[0] - round(sensor_distance / 0.05)
            if self.robot_angle == 270:
                obstacle_grid_y = self.robot_grid[1] - round(sensor_distance / 0.05)
            self.occupancy_grid[obstacle_grid_y][obstacle_grid_x] = 2
    
    def update_robot_grid(self, move):
        if self.robot_angle == 0:
            self.robot_grid[0] += round(move / 0.05)
        if self.robot_angle == 90:
            self.robot_grid[1] += round(move / 0.05)
        if self.robot_angle == 180:
            self.robot_grid[0] -= round(move / 0.05)
        if self.robot_angle == 270:
            self.robot_grid[1] -= round(move / 0.05)
    
    def update_robot_angle(self, angle):
        self.robot_angle = (self.robot_angle + angle) % 360

class Slam:
    def __init__(self, x, y, theta):
        plt.figure(figsize=(10, 10))
        self.map = Map(50, 50)
        self.dist_dict = {0: 0, 2.5: 0.05, 5: 0.1, 7.5: 0.15}
        self.angle_dict = {0: 0, 90: 182, -90: -182}
        self.robot = Robot(x=x, y=y, theta=theta, model_name='vector')
    
    def visualize(self):
        plt.clf()
        self.map.draw_map()
        plt.draw()
        plt.pause(0.01)
    
    def select_translation_time(self):
        translate_time = np.random.choice(cfg["TRANSLATION_CHOICES"])
        if self.robot.sensor_distance < self.dist_dict[translate_time] + cfg["VECTOR_LENGTH"]:
            translate_time = 0.0
        return translate_time
    
    def select_rotate_angle(self):
        rotate_angle = np.random.choice(cfg["ROTATION_CHOICES"])
        return rotate_angle
    
    def run_slam(self):
        time.sleep(5)
        last_control = None
        while not rospy.is_shutdown():
            if self.robot.sensor_distance < 0.10:
                r = 0.6
            else:
                r = 0.4
            self.map.update_occupancy_grid(self.robot.sensor_distance)
            robot_move = random.choices(population=['translation', 'rotation'], weights=[1-r, r])[0]
            if robot_move == 'translation':
                # translate
                translation_time = self.select_translation_time()
                last_control = translation_time
                print(f'--- target distance: {self.dist_dict[translation_time]}')
                self.robot.translate(translation_time)
                last_control = translation_time
                self.map.update_robot_grid(self.dist_dict[translation_time])
            elif robot_move == 'rotation':
                # rotate
                rotation_angle = self.select_rotate_angle()
                if last_control == -rotation_angle:
                    rotation_angle = -rotation_angle  # Prevent rotating back
                last_control = rotation_angle
                print(f'--- target angle: {self.angle_dict[rotation_angle]}')
                self.robot.rotate(cfg["ROTATION_SPEED"], rotation_angle)
                self.map.update_robot_angle(rotation_angle)
            self.visualize()
            time.sleep(0.1)
                
            
    

def main():
    slam = Slam(x=-0.4, y=0.2, theta=0)
    slam.run_slam()

if __name__ == "__main__":
    main()