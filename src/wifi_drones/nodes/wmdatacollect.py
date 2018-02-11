#!/usr/bin/env python

'''
Description : This node collects, avg the wifi data and localizes the aps
Subcriptions : bcc, APInfo
Publications : aploc
'''

import rospy
import numpy as np
import matplotlib.pyplot as plt
from manual_measurements.msg import AccessPointInfo, WifiInfo
from wmdatatools_praneeth import WifiDataCleaner
from fast_localize_ap import Fast_localize

class WifiExpo:

	def __init__(self, topic, cc, waits):
		self.topic = topic
		self.Z = {}
		self.cc = cc
		self.wifidb = {}
		self.flag = True
		self.waits = waits

	def run(self):
		rospy.Subscriber(self.topic, WifiInfo, self.callback)
		rospy.spin()

	def callback(self, data):
		if self.flag == True:
			self.flag = False
			self.getZ(data)

	def getZ(self, rawdata):
		apunion = {}
		locations = []
		places = []
		places.append(self.cc)
		currdict = WifiDataCleaner.raw2Dict(rawdata.accesspoint)
		len_aplist = 0 if not self.Z == True else len(self.Z[self.Z.keys()[0]])
		self.wifidb.update({self.cc: currdict})
		apunion = set(currdict.keys()).union(set(self.Z.keys()))
		for ap in apunion:
			bssid = ap
			if bssid in currdict.keys() in self.Z.keys():
				print "case1"
				self.Z[bssid].append(currdict[bssid])
			elif bssid in currdict.keys() not in self.Z.keys():
				print "case2"
				self.Z.update({bssid:[-1]*len_aplist})
				self.Z[bssid].append(currdict[bssid])
			elif bssid not in currdict.keys() in self.Z.keys():
				print 'case3'
				self.Z[bssid].append(-1)
		if (len(places) > self.waits):
			locations = Fast_localize.locate_aps(places, self.Z)
		else:
			rospy.loginfo('Not enough places visited (< {})'.format(self.waits))

# if __name__ == '__main__':
    # WifiDataCollector(topic = '/APInfo', cc = '(4,5)', waits = '4').run()