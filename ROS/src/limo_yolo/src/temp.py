#!/usr/bin/env python3.6

import rospy
from geometry_msgs.msg import PointStamped
from limo_yolo.msg import map
import matplotlib.pyplot as plt

class MapPlotter:
    def __init__(self):
        rospy.init_node('map_plotter', anonymous=True)
        self.fig, self.ax = plt.subplots()
        self.ax.set_aspect('equal')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.points = []

    def plot_points(self, msg):
        print("test")
        self.ax.clear()
        for point in msg.goal:
            self.points.append((point.point.x, point.point.y))
        if self.points:
            x, y = zip(*self.points)
            self.ax.plot(x, y, 'ro')
        self.ax.set_title('Map with Goals')
        self.ax.grid(True)
        plt.draw()
        plt.pause(0.001)

    def subscribe_to_map(self):
        rospy.Subscriber('/map', map, self.plot_points)
        print("test")

        rospy.spin()

if __name__ == '__main__':
    map_plotter = MapPlotter()

    map_plotter.subscribe_to_map()
