#!/usr/bin/env python3
import rospy
from threading import Thread
import time
import matplotlib.pyplot as plt
import threading
import random
import numpy as np
import math
import scipy.stats as stats
from sensor_msgs.msg import Range
from shapely.geometry import Point, Polygon, LineString
from p_tqdm import p_map, t_map
from pathos.multiprocessing import ProcessingPool as Pool
from anki_vector_ros.msg import Proximity
import warnings
warnings.filterwarnings("ignore")

from utils.robot_lab import Robot
from utils.map_lab import Map
from utils.particle import Particle
from default_config import DEFAULT_CONFIG
cfg = DEFAULT_CONFIG

map_lines = None
map = None
sensor_distance = None

sensor_model = {
    0: {'mean': 0.0, 'std': 0.01}, 
    5: {'mean': 0.049990 - 0.05, 'std': 1.899974 * 0.01}, 
    10: {'mean': 0.100664 - 0.10, 'std': 1.389642 * 0.01},
    15: {'mean': 0.150070 - 0.15, 'std': 1.899974 * 0.01},
    20: {'mean': 0.200070 - 0.20, 'std': 1.899974 * 0.01},
    25: {'mean': 0.248000 - 0.25, 'std': 2.899974 * 0.01},
    30: {'mean': 0.285244 - 0.30, 'std': 3.526537 * 0.01},
    37: {'mean': 0.344468 - 0.37, 'std': 4.352222 * 0.01},
}

def compute_particle_weight_parallel(particle):
    global map, sensor_distance
    if not map.valid_point(particle.x, particle.y):
        return 0
    x, y = particle.get_sensor_line()
    min_distance = map.find_closest_intersection(x1=x[0], x2=x[1], y1=y[0], y2=y[1])
    # if min_distance < 0.35:
    #     sensor_distance_model = round(min_distance / 5) * 5
    #     min_distance += np.random.normal(sensor_model[sensor_distance_model]['mean'], sensor_model[sensor_distance_model]['std'])
    # else:
    #     min_distance += np.random.normal(sensor_model[37]['mean'], sensor_model[37]['std'])
    # std = 0.02
    if min_distance < 0.32:
        sensor_distance_model = round(min_distance / 0.05) * 5
    else:
        sensor_distance_model = 37
    min_distance += sensor_model[sensor_distance_model]['mean']
    std = sensor_model[sensor_distance_model]['std']
    if min_distance > 0.35 and sensor_distance > 0.35:
        min_distance = 0.4
        sensor_distance = 0.4
    weight = stats.norm(min_distance, std).pdf(np.clip(sensor_distance + 0.03, 0, 0.4))
    weight = weight + particle.weight * 0.2
    return weight

class ParticleFilter:
    def __init__(self, x, y, theta, map_file):
        rospy.init_node("vector_hello_world")
        self.robot = Robot(x=x, y=y, theta=theta, model_name='vector')
        self.map = Map(map_file)
        global map
        map = self.map
        self.laser_subscriber = rospy.Subscriber("/proximity", Proximity, self.laser_callback)
        self.dist_dict = {0: 0, 1.0: 0.05, 2.0: 0.1, 3.0: 0.15}
        self.angle_dict = {0: 0, 90: 182, -90: -182}

        self.translation_model = {  # duration to mean and std of speed (mm/s)
            0.0: {'mean': 0.0, 'std': 0.0},
            1.0: {'mean': 4.196666667 * 0.01, 'std': 0.6573107874 * 0.01},
            2.0: {'mean': 4.586666667 * 0.01, 'std': 0.3695135043 * 0.01},
            3.0: {'mean': 4.671111111 * 0.01, 'std': 0.2422331187 * 0.01}
        }

        self.rotation_model = {  # deg to mean and std of w (for 90deg/s=1.57rad/s)
            0: {'mean': 0.0, 'std': 0.0},
            90: {'mean': 85.6, 'std': 4.718756898},
            -90: {'mean': -84.6, 'std': 5.966573556},
        }

        # self.rotation_translation_model = {  # deg to mean and std of v
        #     0: {'mean': 0.0, 'std': 0.0},
        #     90: {'mean': 0.006, 'std': 6.14e-6},
        #     -90: {'mean': -0.007, 'std': 7.92e-6},
        # }
        

        time.sleep(1)  # Wait for sensor to start
        self.particles = self.generate_particles(cfg["PARTICLES"])
        self.filter_thread = Thread(target=self.run_filter)
        self.filter_thread.start()

    def laser_callback(self, data):
        self.sensor_distance = data.distance / 1000
        global sensor_distance
        sensor_distance = self.sensor_distance

    def visualize(self):
        plt.clf()  # Clear the current figure.
        self.map.draw_map()
        # self.robot.plot_robot()
        for p in self.particles:
            p.plot_particle()
        
        self.best_particle().plot_particle(is_best=True)
        plt.draw()
        plt.pause(0.01)

    def best_particle(self):
        particle_weights = [p.weight for p in self.particles]
        return self.particles[np.argmax(particle_weights)]

    def select_translation_time(self):
        translate_time = np.random.choice(cfg["TRANSLATION_CHOICES"])
        if self.robot.sensor_distance < self.dist_dict[translate_time] + cfg["VECTOR_LENGTH"]:
            # print(f"Not moving forward: Too close to wall ({self.robot.sensor_distance})")
            translate_time = 0.0
        return translate_time

    def select_rotate_angle(self):
        rotate_angle = np.random.choice(cfg["ROTATION_CHOICES"])
        return rotate_angle

    def generate_particles(self, count):
        particles = []
        xmin, ymin, xmax, ymax = self.map.get_map_coordinates()
        for i in range(count):
            x = np.random.uniform(xmin, xmax)
            y = np.random.uniform(ymin, ymax)
            theta = np.random.choice([-90, 90, 180, 0]) * math.pi / 180.0
            if np.random.uniform(0, 1) < 0.0: 
                x, y, theta = -0.4, 0.2, 0  # For test
            while not self.map.valid_point(x, y):
            # while self.compute_particle_weight(Particle(x, y, theta)) <= cfg["WEIGHT_CUTOFF"]:
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
            v_mean = self.translation_model[translation_time]['mean']
            v_std = self.translation_model[translation_time]['std']
            particle_v = np.random.normal(v_mean, v_std)
            distance = particle_v * translation_time
            self.particles[i].move(distance)

    def rotate_particles(self, angle):
        for particle in self.particles:
            w_mean = self.rotation_model[angle]['mean']
            w_std = self.rotation_model[angle]['std']
            particle_deg = np.random.normal(w_mean, w_std)
            particle_angle = particle_deg * math.pi / 180
            particle.rotate(particle_angle)
            
            

    # def compute_particle_weight(self, particle):
    #     if not self.map.valid_point(particle.x, particle.y):
    #         return 0
    #     x, y = particle.get_sensor_line()
    #     min_distance = self.map.find_closest_intersection(x1=x[0], x2=x[1], y1=y[0], y2=y[1])
    #     weight = stats.norm(min_distance, 0.03).pdf(np.clip(self.sensor_distance + 0.042, 0, 0.4))
    #     weight = weight + particle.weight * 0.2
    #     return weight

    def compute_particle_weights(self):
        new_weights = p_map(compute_particle_weight_parallel, self.particles, num_cpus=4)
        # new_weights = [self.compute_particle_weight(p) for p in self.particles]
        new_weights = new_weights / np.sum(new_weights)
        for idx, particle in enumerate(self.particles):
            particle.set_weight(new_weights[idx])

    def resample_particles(self):
        keeping_particles_count = int(cfg["KEEP_BEST_PARTICLES_RATE"] * len(self.particles))
        gaussian_particles_count = int(cfg["GAUSSIAN_PARTICLES_RATE"] * len(self.particles))
        random_particles_count = len(self.particles) - (keeping_particles_count + gaussian_particles_count)
        self.particles.sort(key=lambda x: x.weight, reverse=True)
        self.particles = self.particles[:keeping_particles_count]
        
        particle_weights = [p.weight for p in self.particles]
        cumulative_sum = np.cumsum(particle_weights)
        cumulative_sum[-1] = 1.  # avoid round-off errors
        selected_particles_indexes = np.searchsorted(cumulative_sum, np.random.random(gaussian_particles_count))  # roulette wheel sampling
        gaussian_particles = self.generate_gaussian_particles(selected_particles_indexes)
        self.particles.extend(gaussian_particles)
        self.particles.extend(self.generate_particles(random_particles_count))

    # def compute_range_scores(self):
    #     x, y, z, theta = self.robot.get_state()
    #     distances = np.array([np.sqrt((p.x-x)**2 + (p.y-y)**2) for p in self.particles])
    #     for r in [0.05, 0.1, 0.15]:
    #         print(f"### In {r:.2f}: {np.sum(distances <= r) / cfg['PARTICLES'] * 100 :.2f}%")

    def convergence_rate(self):
        particle_weights = [p.weight for p in self.particles]
        # med = np.median(particle_weights)
        med = np.quantile(particle_weights, 0.75)
        top = [p for p in self.particles if p.weight > med]
        top_x = np.average([p.x for p in top])
        top_y = np.average([p.y for p in top])
        best = self.particles[np.argmax(particle_weights)]
        distances = np.array([np.sqrt((p.x-top_x)**2 + (p.y-top_y)**2) for p in self.particles])
        for r in [0.05, 0.1, 0.15]:
            print(f"### In {r:.2f} of AVG: {np.sum(distances <= r) / cfg['PARTICLES'] * 100 :.2f}%")
        distances = np.array([np.sqrt((p.x-best.x)**2 + (p.y-best.y)**2) for p in self.particles])
        for r in [0.05, 0.1, 0.15]:
            print(f"### In {r:.2f} of BEST: {np.sum(distances <= r) / cfg['PARTICLES'] * 100 :.2f}%")
    

    def run_filter(self):
        time.sleep(1)
        last_control = None
        explore_state = True
        rotate_explore_count = 0
        iteration = 0
        while not rospy.is_shutdown():
            if self.robot.sensor_distance < 0.05:
                r = 0.5
            else:
                r = 0.1
            robot_move = random.choices(population=['translation', 'rotation'], weights=[1-r, r])[0]
            # print('###', robot_move, self.robot.sensor_distance)
            if explore_state:
                robot_move = 'rotation'
            if cfg["MANUAL"]:
                control = input("[4: L, 6: R, 8: Forward] Control? ")
                print(control)
                if control == "4":
                    robot_move = 'rotation'
                    rotation_angle = +90
                if control == "6":
                    robot_move = 'rotation'
                    rotation_angle = -90
                if control == "8":
                    robot_move = 'translation'
            if robot_move == 'translation':
                # translate
                translation_time = self.select_translation_time()
                last_control = translation_time
                print(f'--- target distance: {self.dist_dict[translation_time]}')
                self.robot.translate(translation_time)
                self.translate_particles(translation_time)
                rotate_explore_count = 0
                explore_state = True
            elif robot_move == 'rotation':
                # rotate
                if not cfg["MANUAL"]:
                    rotation_angle = self.select_rotate_angle()
                    if last_control == -rotation_angle:
                        rotation_angle = -rotation_angle  # Prevent rotating back
                    if explore_state:
                        rotation_angle = 90
                        rotate_explore_count += 1
                        if rotate_explore_count == 4:
                            explore_state = False
                    last_control = rotation_angle
                # print(f'--- target angle: {self.angle_dict[rotation_angle]}')
                self.robot.rotate(rotation_angle)
                self.rotate_particles(rotation_angle)
            time.sleep(0.1)
            print("Compute Weights...")
            self.compute_particle_weights()
            print("Resample Particles...")
            self.resample_particles()
            print("Compute Weights...")
            self.compute_particle_weights()
            print("Resample Particles...")
            self.resample_particles()
            iteration += 1
            print(f"[{iteration}] Visualizing...")
            self.visualize()
            print("Sensor:", self.sensor_distance)
            # self.compute_range_scores()
            self.convergence_rate()
            

def main():
    particle_filter = ParticleFilter(x=-0.4, y=0.2, theta=0, map_file=cfg["MAP_FILE"])
    # while True:
    #     particle_filter.visualize()


if __name__ == '__main__':
    main()
