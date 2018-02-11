#!/usr/bin/env python

import rospy
import numpy as np

from std_msgs.msg import String
from fast_localize_simulation import Fast_localize
from math import floor, sqrt
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

import config

class GetSimulatedSS():

    def __init__(self, places, Z):
        self.Z = Z
        self.places = places
        self.loc = config.load_env
        self.nap = len(self.loc[self.loc.keys()[0]])
        self.pub_domap = rospy.Publisher('domAP', String, queue_size=10)
        self.rate = rospy.Rate(10)

    def run(self):
        # while not rospy.is_shutdown():
        #     self.pub_domap.publish('jkhkjh')
        #     self.rate.sleep()
        return self.read_accesspoints()

    def read_accesspoints(self):
        cc = self.places[-1]
        for i in range(self.nap):
            bssid = i+1
            if len(self.Z) < self.nap:
                self.Z.update({bssid: [self.get_quality(cc, i)]})
            else:
                self.Z[bssid].append(self.get_quality(cc, i))
        return self.Z

    def ITU_model(self, d):
        f = 2400
        Pf = 11
        N = 30
        Rx = -60
        L = 20*np.log10(f) + N*np.log10(d) + Pf - 28
        Pr = 23 - (L + Rx + 6)
        return Pr

    def FAL_model(self, d):
        C = -30 #35
        n = 3
        RSS = C - 10*n*np.log10(d) - 30 #+ np.random.normal(0,2,1)
        return RSS

    def get_quality(self, cc, i):
        RSS = self.FAL_model(sqrt((self.loc['x'][i] - cc[0]) ** 2 + (self.loc['y'][i] - cc[1]) ** 2 + (self.loc['z'][i] - 0) ** 2))
        if RSS <= -60: #80
            return 0.0
        elif RSS >= -20: #55
            return 100.0
        else:
        #     return 2*(RSS) #+20
        return 2 * (RSS + 100 - 30)
        # return RSS