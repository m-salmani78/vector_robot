import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import math
from default_config import DEFAULT_CONFIG

class Particle:
    def __init__(self, x, y, theta) -> None:
        self.x = x
        self.y = y
        self.theta = theta
        self.weight = 0
    
    def plot_particle(self, is_best=False):
        plot_weight = np.clip(self.weight * 10000000 + 0.4, 0, 1)
        plt.plot(self.x, self.y, color=(plot_weight, 0, 0, 0.9), marker="o", markersize=min(plot_weight + 1, 10))
        plt.plot(*self.get_sensor_line(range=0.01), c="blue", alpha=0.5)
        if is_best:
            plt.plot(self.x, self.y, color=(1, 0, 0, 1), marker="s", markersize=6)
            plt.plot(*self.get_sensor_line(range=0.01), c="blue", alpha=0.5)

    def move(self, distance):
        self.x += distance * np.cos(self.theta)
        self.y += distance * np.sin(self.theta)

    def rotate(self, angle):
        """
            angle: radian
        """
        self.theta += angle
        self.theta = self.normalize_angle(self.theta)
    
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def set_weight(self, weight):
        self.weight = weight

    def get_sensor_line(self, range=DEFAULT_CONFIG["SENSOR_MAX_DISTANCE"]):
        x1 = self.x
        x2 = self.x + range * np.cos(self.theta)
        y1 = self.y
        y2 = self.y + range * np.sin(self.theta)
        return [(x1, x2), (y1, y2)]
