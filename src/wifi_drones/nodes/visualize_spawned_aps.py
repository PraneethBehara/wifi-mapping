#!/usr/bin/env python

import rospy
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

import config

class visualize():

    def __init__(self):
        self.loc = config.load_env
        self.nap = len(self.loc[self.loc.keys()[0]])
        self.vizpub = rospy.Publisher('spawned_aps', Marker, queue_size=10)

    def run(self):
        while not rospy.is_shutdown():
            self.publish_aps()

    def publish_aps(self):
        rate = rospy.Rate(10)
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.id = 10
        marker.type = marker.CUBE_LIST
        marker.action = marker.ADD
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.25
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 2.0
        marker.color.b = 1.0
        marker.points = self.get_points()
        self.vizpub.publish(marker)
        rate.sleep()

    def get_points(self):
        points = []
        for i in range(self.nap):
            p = Point()
            p.x = self.loc['x'][i]
            p.y = self.loc['y'][i]
            p.z = self.loc['z'][i]
            points.append(p)
        return points

def main():
    rospy.init_node('visualize_spawned_aps')
    visualize().run()

if __name__ == '__main__':
    main()
    rospy.spin()

