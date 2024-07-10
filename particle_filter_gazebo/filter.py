#!/usr/bin/env python3
import rospy
from threading import Thread
import time
import matplotlib.pyplot as plt
import random
import numpy as np
import math
import scipy.stats as stats
from sensor_msgs.msg import Range
from p_tqdm import p_map, t_map
import warnings

warnings.filterwarnings("ignore")

from robot import Robot
from map import Map
from particle import Particle
from configs import CONFIG

map = None
sensor_distance = None

sensor_model = {
    0: {"mean": 0.0, "std": 0.01},
    5: {"mean": 0.004736, "std": 0.035173},
    10: {"mean": 0.006546, "std": 0.022644},
    20: {"mean": 0.011082, "std": 0.040112},
    30: {"mean": 0.012548, "std": 0.093023},
    37: {"mean": 0.000918, "std": 0.188409},
}


def parallel_compute_weight(particle: Particle):
    global map
    if not map.valid_point(particle.x, particle.y):
        return 0
    x, y = particle.get_sensor_line()
    min_distance = map.find_closest_intersection(x1=x[0], x2=x[1], y1=y[0], y2=y[1])
    if min_distance < 0.35:
        sensor_distance_model = round(min_distance / 5) * 5
    else:
        sensor_distance_model = 37
    min_distance += sensor_model[sensor_distance_model]["mean"]
    std = sensor_model[sensor_distance_model]["std"]
    weight = stats.norm(min_distance, std).pdf(np.clip(sensor_distance + 0.042, 0, 0.4))
    weight = weight + particle.weight * 0.2
    return weight


class ParticleFilter:
    def __init__(self, x, y, theta, map_file):
        self.robot = Robot(x=x, y=y, theta=theta, model_name="robot")
        self.map = Map(map_file)
        global map
        map = self.map
        self.laser_subscriber = rospy.Subscriber(
            "/vector/laser", Range, self.laser_callback
        )
        self.dist_dict = {0.0: 0, 5: 0.1, 10: 0.2, 15: 0.3}
        self.angle_dict = {0: 0, 90: 182, -90: -182}
        self.first_plot = True

        self.translation_model = {  # duration to mean and std of speed (mm/s)
            0.0: {"mean": 0.0, "std": 0.0},
            5.0: {"mean": -0.008467, "std": 0.000806713},
            10.0: {"mean": -0.009100, "std": 0.000571276},
            15.0: {"mean": -0.008634, "std": 0.000658264},
        }

        self.rotation_model = {  # deg to mean and std of w (for 90deg/s=1.57rad/s)
            0: {"mean": 0.0, "std": 0.0},
            90: {"mean": 1.564, "std": 0.0014},
            -90: {"mean": -1.577, "std": 0.0012},
        }

        self.rotation_translation_model = {  # deg to mean and std of v
            0: {"mean": 0.0, "std": 0.0},
            90: {"mean": 0.0, "std": 1.0e-6},
            -90: {"mean": 0.0, "std": 1.0e-6},
        }

        time.sleep(1)  # Wait for sensor to start
        self.particles = self.generate_particles(CONFIG["PARTICLES"])
        self.filter_thread = Thread(target=self.run_filter)
        self.filter_thread.start()

    def laser_callback(self, data):
        self.sensor_distance = data.range
        global sensor_distance
        sensor_distance = self.sensor_distance

    def visualize(self):
        if self.first_plot:
            plt.figure(figsize=(12.8, 9.6))
            self.first_plot = False
        else:
            plt.clf()
        self.map.draw_map()
        self.robot.plot_robot()
        for p in self.particles:
            p.plot_particle()

        self.best_particle().plot_particle(is_best=True)
        plt.draw()
        plt.pause(0.1)

    def best_particle(self):
        particle_weights = [p.weight for p in self.particles]
        return self.particles[np.argmax(particle_weights)]

    def select_translation_time(self):
        translate_time = np.random.choice(list(self.dist_dict.keys())[1:])
        if (
            self.robot.sensor_distance
            < self.dist_dict[translate_time] + CONFIG["VECTOR_LENGTH"]
        ):
            translate_time = 0.0
        return translate_time

    def select_rotate_angle(self):
        rotate_angle = np.random.choice(list(self.angle_dict.keys())[1:])
        return rotate_angle

    def generate_particles(self, count):
        particles: list[Particle] = []
        xmin, ymin, xmax, ymax = self.map.get_map_coordinates()
        for i in range(count):
            x = np.random.uniform(xmin, xmax)
            y = np.random.uniform(ymin, ymax)
            theta = np.random.choice([-90, 90, 180, 0]) * math.pi / 180.0
            while not self.map.valid_point(x, y):
                x = np.random.uniform(xmin, xmax)
                y = np.random.uniform(ymin, ymax)
            particle = Particle(x, y, theta)
            particles.append(particle)
        return particles

    def generate_gaussian_particles(self, indexes):
        particles = []
        xmin, ymin, xmax, ymax = self.map.get_map_coordinates()
        g_std = 0.01
        for idx in indexes:
            x = np.random.normal(self.particles[idx].x, g_std)
            y = np.random.normal(self.particles[idx].y, g_std)
            theta = np.random.normal(self.particles[idx].theta, 0.5)
            while not self.map.valid_point(x, y):
                x = np.random.normal(self.particles[idx].x, g_std)
                y = np.random.normal(self.particles[idx].y, g_std)
            particle = Particle(x, y, theta)
            particles.append(particle)
        return particles

    def translate_particles(self, translation_time):
        for i in range(len(self.particles)):
            v_mean = self.translation_model[translation_time]["mean"]
            v_std = self.translation_model[translation_time]["std"]
            particle_v = np.random.normal(v_mean, v_std)
            distance = particle_v * translation_time
            self.particles[i].move(distance)

    def rotate_particles(self, angle):
        for particle in self.particles:
            w_mean = self.rotation_model[angle]["mean"]
            w_std = self.rotation_model[angle]["std"]
            particle_w = np.random.normal(w_mean, w_std)
            time = abs(angle / CONFIG["ROTATION_SPEED"])
            particle_angle = particle_w * time
            particle.rotate(particle_angle)
            distance = (
                np.random.normal(
                    self.rotation_translation_model[angle]["mean"],
                    self.rotation_translation_model[angle]["std"],
                )
                * time
            )
            particle.move(distance)

    def compute_particle_weights(self):
        new_weights = p_map(parallel_compute_weight, self.particles, num_cpus=4)
        new_weights = new_weights / np.sum(new_weights)
        for idx, particle in enumerate(self.particles):
            particle.set_weight(new_weights[idx])

    def resample_particles(self):
        keeping_particles_count = int(0.4 * len(self.particles))
        gaussian_particles_count = int(0.55 * len(self.particles))
        random_particles_count = len(self.particles) - (
            keeping_particles_count + gaussian_particles_count
        )
        self.particles.sort(key=lambda x: x.weight, reverse=True)
        self.particles = self.particles[:keeping_particles_count]

        particle_weights = [p.weight for p in self.particles]
        cumulative_sum = np.cumsum(particle_weights)
        cumulative_sum[-1] = 1.0  # avoid round-off errors
        selected_particles_indexes = np.searchsorted(
            cumulative_sum, np.random.random(gaussian_particles_count)
        )  # roulette wheel sampling
        gaussian_particles = self.generate_gaussian_particles(
            selected_particles_indexes
        )
        self.particles.extend(gaussian_particles)
        self.particles.extend(self.generate_particles(random_particles_count))

    def compute_range_scores(self):
        x, y, z, theta = self.robot.get_state()
        distances = np.array(
            [np.sqrt((p.x - x) ** 2 + (p.y - y) ** 2) for p in self.particles]
        )
        for r in [0.05, 0.1, 0.15]:
            print(
                f"### In {r:.2f}: {np.sum(distances <= r) / CONFIG['PARTICLES'] * 100 :.2f}%"
            )

    def convergence_rate(self):
        x_avg, y_avg = 0.0, 0.0
        for p in self.particles:
            x_avg += p.x
            y_avg += p.y
        x_avg /= len(self.particles)
        y_avg /= len(self.particles)
        distances = np.array(
            [np.sqrt((p.x - x_avg) ** 2 + (p.y - y_avg) ** 2) for p in self.particles]
        )
        print(
            f"### In 0.05 of the average particles: {np.sum(distances <= 0.05) / CONFIG['PARTICLES'] * 100 :.2f}%"
        )

    def run_filter(self):
        time.sleep(1)
        last_control = None
        explore_state = True
        rotate_explore_count = 0
        while not rospy.is_shutdown():
            if explore_state:
                robot_move = "rotation"
            else:
                if self.robot.sensor_distance <= 0.05:
                    r = 0.9
                else:
                    r = 0.1
                robot_move = random.choices(
                    population=["translation", "rotation"], weights=[1 - r, r]
                )[0]
            print(
                f"&&& Robot move: {robot_move}\n, Sensor distance: {self.robot.sensor_distance}"
            )

            if robot_move == "translation":
                translation_time = self.select_translation_time()
                last_control = translation_time
                print(f"--- target distance: {self.dist_dict[translation_time]}")
                self.robot.translate(translation_time)
                self.translate_particles(translation_time)
                rotate_explore_count = 0
                explore_state = True
            elif robot_move == "rotation":
                if explore_state:
                    rotation_angle = 90
                    rotate_explore_count += 1
                    print(f"&&& rotate count: {rotate_explore_count}")
                    if rotate_explore_count == 4:
                        explore_state = False
                else:
                    rotation_angle = self.select_rotate_angle()
                    if last_control == -rotation_angle:
                        rotation_angle = -rotation_angle  # Prevent rotating back
                last_control = rotation_angle
                print(f"--- target angle: {self.angle_dict[rotation_angle]}")
                self.robot.rotate(CONFIG["ROTATION_SPEED"], rotation_angle)
                self.rotate_particles(rotation_angle)
            time.sleep(0.1)
            print("Compute Weights...")
            self.compute_particle_weights()
            print("Visualizing...")
            self.visualize()
            self.compute_range_scores()
            self.convergence_rate()
            print("Resample Particles...")
            self.resample_particles()


def main():
    ParticleFilter(
        x=-0.175,
        y=0.175,
        theta=0,
        map_file="/home/user/catkin_ws/src/anki_description/world/sample1.world",
    )


if __name__ == "__main__":
    main()
