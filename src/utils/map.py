#!/usr/bin/env python3
import xml.etree.ElementTree as ET
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Point, Polygon, LineString
from default_config import DEFAULT_CONFIG


class Map:
    def __init__(self, map_file):
        self.objects = self.parse_map(map_file)
        self.add_offset()
        self.xmin, self.ymin, self.xmax, self.ymax = self.get_map_coordinates()
        print('$$$ MAP',self.xmin, self.ymin, self.xmax, self.ymax)

    def get_points(self, xc, yc, theta, w, h):
        upper_right = [xc+w*np.sin(theta)/2+h*np.cos(theta)/2, 
                       yc+h*np.sin(theta)/2-w*np.cos(theta)/2]
        upper_left = [xc-w*np.sin(theta)/2+h*np.cos(theta)/2,
                      yc+h*np.sin(theta)/2+w*np.cos(theta)/2]
        lower_right = [xc+w*np.sin(theta)/2-h*np.cos(theta)/2, 
                       yc-h*np.sin(theta)/2-w*np.cos(theta)/2]
        lower_left = [xc-w*np.sin(theta)/2-h*np.cos(theta)/2,
                      yc-h*np.sin(theta)/2+w*np.cos(theta)/2]
        return {'upper_right': upper_right, 'upper_left': upper_left, 'lower_right': lower_right, 'lower_left': lower_left}

    def parse_map(self, map_file):
        tree = ET.parse(map_file)
        root = tree.getroot()
        objects = {}
        for model in root.findall('./world/model'):
            for child in model:
                if (child.tag == 'pose'):
                    offset = child.text.split()
                    self.x_offset, self.y_offset = float(offset[0]), float(offset[1])

                if (child.tag == 'link'):
                    pose = child.find('pose')  # Center
                    size = child.find('collision/geometry/box/size')  # Size
                    if (pose != None and size != None):
                        name = child.attrib['name']
                        xc, yc, z, roll, pitch, yaw = child.find('pose').text.split()  # Center of each cube (Pose)
                        w, h, _ = size.text.split()  # Width & Height (& z - length) of each cube
                        objects[name] = self.get_points(float(xc), float(yc), float(yaw), float(h), float(w))
        return objects

    def add_offset(self):
        for item in self.objects:
            object = self.objects[item]
            for point in object:
                self.objects[item][point][0] += self.x_offset
                self.objects[item][point][1] += self.y_offset

    def get_lines(self):
        lines = []
        for name, object in self.objects.items():
            lines.append([[object['upper_right'][0], object['upper_right'][1]], [
                object['lower_right'][0], object['lower_right'][1]]])
            lines.append([[object['lower_right'][0], object['lower_right'][1]], [
                object['lower_left'][0], object['lower_left'][1]]])
            lines.append([[object['lower_left'][0], object['lower_left'][1]], [
                object['upper_left'][0], object['upper_left'][1]]])
            lines.append([[object['upper_left'][0], object['upper_left'][1]], [
                object['upper_right'][0], object['upper_right'][1]]])
        return lines

    def draw_map(self):
        for name, object in self.objects.items():
            line1 = [[object['upper_right'][0], object['lower_right'][0]], [
                object['upper_right'][1], object['lower_right'][1]]]
            line2 = [[object['lower_right'][0], object['lower_left'][0]], [
                object['lower_right'][1], object['lower_left'][1]]]
            line3 = [[object['lower_left'][0], object['upper_left'][0]], [
                object['lower_left'][1], object['upper_left'][1]]]
            line4 = [[object['upper_left'][0], object['upper_right'][0]], [
                object['upper_left'][1], object['upper_right'][1]]]
            
            for line in [line1, line2, line3, line4]:
                plt.plot(line[0], line[1], c='black')
                plt.fill("i", "j", facecolor='gray', edgecolor='black', linewidth=1,
                         data={"i": [line1[0], line2[0], line3[0], line4[0]], 
                               "j": [line1[1], line2[1], line3[1], line4[1]]})

    def point_in_object(self, x, y):
        point = Point(x, y)
        for name, object in self.objects.items():
            point1 = object['upper_right']
            point2 = object['lower_right']
            point3 = object['lower_left']
            point4 = object['upper_left']
            poly = Polygon([point1, point2, point3, point4])
            if point.within(poly) or poly.touches(point):
                return True
        return False

    def get_map_coordinates(self):
        x = []
        y = []
        for name, object in self.objects.items():
            for point_name in object:
                point = object[point_name]
                x.append(point[0])
                y.append(point[1])
        return min(x), min(y), max(x), max(y)

    def point_in_map(self, x, y):
        return x < self.xmax and x > self.xmin and y < self.ymax and y > self.ymin

    def valid_point(self, x, y):
        return (not self.point_in_object(x, y)) and (self.point_in_map(x, y))
    
    def find_intersection(self, point_1, point_2, point_3, point_4):
        line1 = LineString([tuple(point_1), tuple(point_2)])
        line2 = LineString([tuple(point_3), tuple(point_4)])
        intersection = line1.intersection(line2)
        return intersection
    
    def find_closest_intersection(self, x1, y1, x2, y2):
        """
        Find closest intersection of a directed line with the map
        """
        min_distance = DEFAULT_CONFIG["SENSOR_MAX_DISTANCE"]
        for line in self.get_lines():
            intersection = self.find_intersection((x1, y1), (x2, y2), line[0], line[1])
            if intersection:
                distance = np.sqrt((intersection.x - x1)**2 + (intersection.y - y1)**2)
                min_distance = min(min_distance, distance)
        return min_distance

# map = Map('sample1.world')
# map.draw_map()
# plt.show()
