#!/usr/bin/env python

import tf
import rospy
import cv2
import heapq
import scipy as sc
import numpy as np
import matplotlib.pyplot as pyplt

from math import floor, sqrt
from scipy import ndimage
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose, Point
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import OccupancyGrid, Odometry
from drones_move.srv import *
from manual_measurements.msg import AccessPointInfo, WifiInfo
from simulate import GetSimulatedSS

import config
import time
import pydb

class BotState:
	READY = 'READY'
	BUSY  = 'BUSY'

class BotStep:
	ROT = 'ROT'
	MOV = 'MOV'

class Publishers:

	def __init__(self, vizgoal=None, viz_aps=None, show_frontiers=None, show_other_frontiers=None):
		self.vizgoal = vizgoal
		self.viz_aps = viz_aps
		self.show_frontiers = show_frontiers
		self.show_other_frontiers = show_other_frontiers

class Subscribers:

	def __init__(self, subscriptions=[]):
		for s in subscriptions:
			rospy.Subscriber(s[0], s[1], s[2])

class Potential:

	def __init__(self, pose=None, gain=None, cost=None):
		self.pose = pose
		self.gain = gain
		self.cost = cost

class WifiDataCollector:

	def __init__(self, topifc, cc, waits):
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
		# if (len(places) > self.waits):
		# 	locations = Fast_localize.locate_aps(places, self.Z)
		# else:
		# 	rospy.loginfo('Not enough places visited (< {})'.format(self.waits))

		# if __name__ == '__main__':
		# WifiDataCollector(topic = '/APInfo', cc = '(4,5)', waits = '4').run()

class PQ:
	def __init__(self):
		self.index = 0
		self.high = 9999

	def push(self, queue, priority, item):
		heapq.heappush(queue, (-priority, item))

	def pop(self, queue):
		return heapq.heappop(queue)[-1]

	def showqueue(self, queue):
		print queue

	def makeheap(self, queue):
		print heapq.heapify(queue)

	def has_unknown_neighbors(self, grid, r, c):
		result = 0
		rows, cols = grid.shape
		if r > 0:
			if c > 0:
				if grid[r - 1][c - 1] == -1:
					result += 1
			if c < cols - 1:
				if grid[r - 1][c + 1] == -1:
					result += 1
		if r < rows - 1:
			if c > 0:
				if grid[r + 1][c - 1] == -1:
					result += 1
			if c < cols - 1:
				if grid[r + 1][c + 1] == -1:
					result += 1
		if c > 0:
			if grid[r][c - 1] == -1:
				result += 1
		if c < cols - 1:
			if grid[r][c + 1] == -1:
				result += 1
		if r > 0:
			if grid[r - 1][c] == -1:
				result += 1
		if r < rows - 1:
			if grid[r + 1][c] == -1:
				result += 1
		return result

class Explorer:

	def __init__(self, grid, odom, readystate):
		self.grid = grid
		self.odom = odom
		self.readystate = readystate
		self.pose = None
		self.origin = None
		self.resolution = 0.05
		self.cres = 10
		self.gridkernel = -1*np.ones((self.cres,self.cres), dtype=np.float32)
		self.visited = []
		self.time = []
		self.fqueue = {}
		self.expoAP = None
		self.covered_aps = []
		self.memory = {}
		self.Z = {}
		self.wifi_db = {}
		self.dop_db = {}
		self.dx_db = {}
		self.state = BotState.READY
		self.tfbr = tf.TransformBroadcaster()
		self.tfli = tf.TransformListener()
		self.rate = rospy.Rate(10)
		self.srv_move_turtle = rospy.ServiceProxy(config.bot_mover['srv']['move_bot'], MoveTurtlebot)
		# self.srv_make_plan = rospy.ServiceProxy(config.move_base['srv']['make_plan'], GetPlan)
		self.frontier = []
		self.botstep = BotStep.ROT
		self.pub = Publishers( vizgoal = rospy.Publisher(config.controller['pub']['viz_goal'], Marker, queue_size=10)
							   , viz_aps = rospy.Publisher(config.controller['pub']['viz_aps'], Marker, queue_size=10)
							   , show_frontiers= rospy.Publisher(config.controller['pub']['show_frontiers'], Marker, queue_size=10)
							   , show_other_frontiers= rospy.Publisher(config.controller['pub']['show_other_frontiers'], Marker, queue_size=10))
		self.vizgoal = Marker()
		self.viz_aps = Marker()
		self.show_frontiers = Marker()
		self.show_other_frontiers = Marker()
		self.start_time = time.time()

	def run(self):
		Subscribers([ (self.grid, OccupancyGrid, self.grid_callback)
						, (self.odom, Odometry, self.odom_callback)
						, (self.readystate, String, self.ready_state_callback) ])
		while not rospy.is_shutdown():
			self.pub.vizgoal.publish(self.vizgoal)
			self.pub.viz_aps.publish(self.viz_aps)
			self.pub.show_frontiers.publish(self.show_frontiers)
			self.pub.show_other_frontiers.publish(self.show_other_frontiers)
			self.rate.sleep()

	def ready_state_callback(self, data):
		self.state = data.data

	def move(self, command, pose):
		try:
			resp1 = self.srv_move_turtle(command, pose)
		except rospy.ServiceException, e:
			self.log("Service call failed: %s"%e)

	def grid_callback(self, data):
		self.log(self.expoAP)
		self.log(self.state)
		self.publish_groundtruth_marker()
		if self.state == BotState.READY:
			self.origin = data.info.origin
			# self.publish_origin_marker()
			# self.tfbr.sendTransform( self.to_transtuple(self.origin.position)
			# 						 , self.to_orientuple(self.origin.orientation)
			# 						 , rospy.Time.now() , 'grid', 'world' )
			if self.botstep == BotStep.ROT:
				self.botstep = BotStep.MOV
				self.move('THREE60', Pose())
			elif self.botstep == BotStep.MOV:
				self.botstep = BotStep.ROT

				self.visited.append((self.pose.position.x, self.pose.position.y))
				self.time.append(time.time() - self.start_time)
				self.logtofile('visited1.py', self.visited)
				self.logtofile('time.py', self.time)
				print("--- %s seconds ---" % (time.time() - self.start_time))

				RSS = GetSimulatedSS(self.visited, self.Z).run()
				# print RSS
				curr_domap = self.get_dominant_AP(RSS)

				if self.expoAP == None or self.is_cluster_explored():
					if self.expoAP == None:
						self.expoAP = curr_domap
					else:
						# pydb.debugger()
						self.log('cluster is explored')
						self.covered_aps.append(self.expoAP)
						del self.fqueue[self.expoAP]
						print "################### finding next best queue ####################"
						# del self.fqueue[self.expoAP]
						self.expoAP = self.pick_next_queue() #sum of queues
						self.log('domAP changed to : ' + str(self.expoAP))
				# pydb.debugger()
				grid     = ( np.array(data.data, dtype=np.float32)
							 .reshape(data.info.height, data.info.width) )
				customgrid = self.create_customgrid(grid.copy())
				# self.visualize(grid, customgrid, 'actual_resolution', 'lower_resolution')

				self.save_grid(customgrid.copy(), self.expoAP)

				cr, cc = self.pose_in_grid()
				horizon = self.compute_frontiers(customgrid.copy(), cr, cc)
				# horizon_ind = self.remove_explored(customgrid.copy(), self.get_max_ind(horizon.copy()))
				horizon_ind = self.get_max_ind(horizon.copy())
				entropy = self.entropy(customgrid.copy(), horizon_ind)
				cost = self.cost(customgrid.copy(), horizon_ind, cr, cc)
				gain = self.info_gain(entropy, cost)

				if curr_domap == self.expoAP:
					self.log('in same cluster , updating frontier queue')
				else:
					self.log('entering different cluster :' + str(curr_domap))
				self.update_frontier_queue(gain.copy(), horizon_ind, curr_domap)
				self.publish_frontiers(self.queue(self.fqueue[self.expoAP]))
				# pydb.debugger()
				# self.publish_frontiers_others(self.getallqueues(self.fqueue), [1.0, 1.0, 1.0])
				if len(self.fqueue[self.expoAP]) != 0:
					self.log('picking goal')
					goal = self.find_first_empty(grid.copy(), self.pose_in_ogrid(PQ().pop(self.fqueue[self.expoAP])))
					print "~~~~~~~~all queues~~~~~~~~ : ", len(self.getallqueues(self.fqueue))
					print '~~~~~~~~dom queue~~~~~~~~ : ', len(self.fqueue[self.expoAP])
					gpose = self.get_npose_from_indices(goal)
					np.set_printoptions(threshold=0,precision=2)
					self.publish_goal_marker(gpose)
					self.move('MOVETO', gpose)

	def pick_next_queue(self):
		# pydb.debugger()
		gain = dict((k,0) for k in self.fqueue)
		for ap in self.fqueue.keys():
			sum = 0
			for each in self.fqueue[ap]:
				sum += -each[0]
			gain[ap] = sum
		# for each in self.covered_aps:
			# del gain[each]
		return max(gain, key=gain.get)

	def remove_explored(self, grid, inds):
		count = 0
		for each in inds:
			count+=1
			if grid[each[0]][each[1]]== 0:
				inds.remove(inds[count])
				print "popped1", each
				count-=1
			else:
				neighbors = self.vicinity([each], 1)
				for each in neighbors:
					if grid[each[0]][each[1]] == 0:
						inds.remove(inds[count])
						print "popped2", each
						count-=1
			# pydb.debugger()
		return inds

	def vicinity(self, list, radius):
		result = []
		for each in list:
			for r in range(-radius, radius):
				for c in range(-radius, radius):
					result.append((each[0] + r, each[1] + c))
		return result

	def pose_in_ogrid(self, ind):
		return (ind[0]*self.cres, ind[1]*self.cres)

	def find_first_empty(self, grid, ind):
		for i in range(10):
			for j in range(10):
				if grid[ind[0]+i][ind[1]+j] == 0.0:
					return [ind[0] + i, ind[1] + j]

	def create_customgrid(self, grid):
		rmargin = self.cres - (grid.shape[0]%self.cres) ;	cmargin = self.cres - (grid.shape[1]%self.cres)
		if rmargin == self.cres:
			rmargin = 0
		if cmargin == self.cres:
			cmargin = 0
		cr = (grid.shape[0]+rmargin)/self.cres ; cc = (grid.shape[1]+cmargin)/self.cres
		l_pad_width = cmargin/2 ; r_pad_width = cmargin - l_pad_width
		u_pad_width = rmargin/2 ; b_pad_width = rmargin - u_pad_width
		grid = np.lib.pad(grid, ((u_pad_width, b_pad_width), (l_pad_width, r_pad_width)), 'constant', constant_values=((-1,-1),(-1,-1)))
		result = -1*np.ones((cr,cc), dtype=np.float32)
		# pydb.debugger()
		for i in range(cr):
			for j in range(cc):
				sum = np.sum(grid[self.cres*i:(i*self.cres)+self.cres, self.cres*j:(j*self.cres)+self.cres])
				if sum > -80 and sum < 1:
					result[i][j] = 0
				# elif sum < -95:
				# 	result[i*self.cres][j*self.cres] = -1
				if sum > 0:
					result[i][j] = 100
		return result

	def publish_frontiers(self, inds):
		gh = []
		for each in inds:
			# pydb.debugger()
			pose = self.get_npose_from_indices(self.pose_in_ogrid(each))
			gh.append((pose.position.x, pose.position.y))
		points = self.do_list_to_points(gh)
		marker = Marker()
		marker.header.frame_id = 'map'
		marker.id = 5
		marker.type = Marker.CUBE_LIST
		marker.action = Marker.ADD
		marker.scale.x = 0.1
		marker.scale.y = 0.1
		marker.scale.z = 0.1
		marker.color.a = 1.0
		marker.color.b = 1.0
		marker.color.r = 1.0
		marker.points = points
		self.show_frontiers = marker

	def publish_frontiers_others(self, inds, color):
		list = []
		for each in inds:
			list.append(self.pose_in_ogrid(each))
		points = self.do_list_to_points(list)
		marker = Marker()
		marker.header.frame_id = 'map'
		marker.id = 16
		marker.type = Marker.CUBE_LIST
		marker.action = Marker.ADD
		marker.scale.x = 0.1
		marker.scale.y = 0.1
		marker.scale.z = 0.1
		marker.color.a = 1.0
		marker.color.r = color[0]
		marker.color.g = color[1]
		marker.color.b = color[2]
		marker.points = points
		self.show_other_frontiers = marker

	def update_frontier_queue(self, gain, f_inds, curr_ap):
		# pydb.debugger()
		union = self.vicinity(self.getallqueues(self.fqueue), 2)
		if curr_ap not in self.fqueue.keys():
			self.fqueue.update({curr_ap: []})
		tempdict = self.fqueue.copy()
		for each in self.fqueue.keys():
			self.fqueue[each] = []
		for each in f_inds:
			if each not in union:
				self.push(curr_ap, each, gain[each[0]][each[1]])
			else:
				for ap in tempdict.keys():
					if each in self.vicinity(self.queue(tempdict[ap]),2):
						print 'there in ap: ',ap
						self.push(ap, each, gain[each[0]][each[1]])

	def push(self, ap, ind, gain):
		PQ().push(self.fqueue[ap], gain, (ind[0], ind[1]))

	def form_memory(self, horizon_ind, gain, apid):
		# pydb.debugger()
		union = self.vicinity(self.getallqueues(self.fqueue), 2)
		self.fqueue[self.expoAP] = []
		for each in horizon_ind:
			if each in union:
				print "already there" , each
				PQ().push(self.fqueue[self.expoAP], gain[each[0]][each[1]], (each[0], each[1]))
			# self.fqueue[self.expoAP].append((gain[each[0]][each[1]], each))
			else:
				print "not there, adding to corresponding queue : ", apid
				if apid in self.fqueue.keys():
					PQ().push(self.fqueue[apid], gain[each[0]][each[1]], (each[0], each[1]))
				else:
					self.fqueue.update({apid: []})
					PQ().push(self.fqueue[apid], gain[each[0]][each[1]], (each[0], each[1]))

	def queue(self, list):
		result = []
		for i in range(len(list)):
			result.append(list[i][-1])
		return result

	def getallqueues(self, dict):
		result = []
		for ap in dict.keys():
			for i in range(len(dict[ap])):
				result.append(dict[ap][i][-1])
		return result

	def remove_other_cluster_frontiers(self, horizon_ind):
		# pydb.debugger()
		union = self.getallqueues(self.fqueue)
		index = 0
		for each in horizon_ind:
			if each in self.vicinity(union, 3):
				horizon_ind.remove(horizon_ind[index])
				index-=1
			index +=1
		return horizon_ind

	def is_cluster_explored(self):
		if len(self.fqueue[self.expoAP]) == 0:
			return True

	def save_grid(self, grid, ap):
		np.set_printoptions(threshold=np.nan, linewidth=np.nan)
		result = np.zeros(grid.shape, dtype=np.int32)
		result[grid == 0.0] = 1
		self.logtofile(str(ap), result)
		np.set_printoptions(threshold=1)

	def get_dominant_AP(self, RSS):
		for ap in RSS:
			RSS[ap] = list(reversed(RSS[ap]))
		return max(RSS, key=RSS.get)

	def check_goal_correctness(self, goal, grid):
		if grid[goal[0][0]][goal[1][1]] == 0.0:
			print "_/ goal"
		else:
			print "X goal!"

	def update_apdb(self, aplocs):
		for key in aplocs.keys():
			if key in self.wifi_db.keys():
				self.wifi_db[key] = aplocs[key]
			else:
				self.wifi_db.update({key: aplocs[key]})
		print 'wifi_db: ', self.wifi_db

	def show_aps_db(self):
		marker = Marker()
		marker.header.frame_id = 'map'
		marker.id = 1
		marker.type = Marker.CUBE_LIST
		marker.action = Marker.ADD
		marker.scale.x = 0.75
		marker.scale.y = 0.75
		marker.scale.z = 0.5
		marker.color.a = 1.0
		marker.color.g = 1.0
		marker.color.r = 1.0
		marker.text = 'some text'
		marker.points = self.do_dict_to_points(self.wifi_db)
		self.viz_aps = marker
	# self.rate.sleep()

	def do_dict_to_points(self, dict):
		points = []
		for ap in dict.keys():
			p = Point()
			p.x = dict[ap][0]
			p.y = dict[ap][1]
			p.z = 3
			points.append(p)
		return points

	# def get_queue(self, gain):
	# 	q = Q.Queue()
	# 	q.put(np.unravel_index(np.argmax(gain),gain.shape))
	# 	for r in range (gain.shape[0]):
	# 		for c in range(gain.shape[1]):
	# 			if gain[r][c]>np.max(gain)/5: # and distance.euclidean(gain[r][c],) > 20:
	# 				q.put([r,c])
	# 	# while not q.empty():
	# 	# 	print q.get()
	# 	# return q

	def compute_frontiers(self, grid, cr, cc):
		rows, cols = grid.shape
		result = np.zeros(grid.shape, dtype=np.float32)
		for r in range(rows):
			for c in range(cols):
				if grid[r][c] == 0:
					if self.has_unknown_neighbors(grid, r, c) > 3:
						result[r][c] = 1
		# self.visualize(grid,result,'grid','raw_horizon')
		# result = self.clean(result)
		# self.visualize(result, grid, 'cleaned', 'grid')
		# return self.remove_obstacles(self.scan_horizon(result, grid), grid)
		# return self.remove_obstacles(result, grid)
		return result

	def clean(self, image):
		se1 = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
		# se2 = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
		mask = cv2.morphologyEx(image, cv2.MORPH_CLOSE, se1)
		# self.visualize(mask, input)
		# result = cv2.morphologyEx(mask, cv2.MORPH_OPEN, se2)
		# self.visualize(result, input)
		return mask

	def scan_horizon(self, horizon, grid):
		fLeft = self.doScan(horizon)
		fTop = np.rot90(self.doScan(np.rot90(horizon)),-1)
		fRight = np.rot90(self.doScan(np.rot90(horizon,2)),2)
		fDown = np.rot90(self.doScan(np.rot90(horizon,-1)))
		projection = fLeft | fTop | fRight | fDown
		# self.visualize(horizon, projection, 'horizon', 'filtered_projection')
		return projection

	def doScan(self, image):
		result = np.zeros(image.shape, dtype=np.int)
		for r in range(image.shape[0]):
			for c in range(image.shape[1]):
				if image[r][c] == 1:
					result[r][c] = 1
					break
		return result

	def remove_obstacles(self, horizon, grid):
		grid[grid > 0] = 200
		horizon[self.gaussian_smooth(grid, 6.0) > 1] = 0     #threshold_for_inflation
		return horizon

	def check_nearfield(self,  grid, cr, cc, radius):
		sum = 0
		for r in range(cr-radius, cr+radius):
			for c in range(cc-radius, cc+radius):
				if r < grid.shape[0] and r > 0 and c < grid.shape[1] and c > 0:
					sum += grid[r][c]
		return sum

	def remove_nearfield(self, image, grid, cr, cc):
		# self.visualize(image, image, 'before', 'before')
		for r in range(cr-40, cr+40):
			for c in range(cc-40, cc+40):
				if r > grid.shape[0] or r < 0 or c > grid.shape[1] or c < 0:
					continue
				image[r][c] = 0
		# self.visualize(image, image, 'after_clearing', 'after_clearing')
		return image

	def gaussian_smooth(self, image, sigma):
		smoothened_image = sc.ndimage.filters.gaussian_filter(image, sigma=sigma, order=0, output=None, mode='reflect', cval=0.0)
		return smoothened_image

	def highlight_obstacles_back(self, grid, gaus_grid):
		gaus_grid[grid == np.max(grid)] = 100
		return gaus_grid

	def has_unknown_neighbors(self, grid, r, c):
		result = 0
		rows, cols = grid.shape
		if r > 0:
			if c > 0:
				if grid[r-1][c-1] == -1:
					result += 1
			if c < cols - 1:
				if grid[r-1][c+1] == -1:
					result += 1
		if r < rows - 1:
			if c > 0:
				if grid[r+1][c-1] == -1:
					result += 1
			if c < cols - 1:
				if grid[r+1][c+1] == -1:
					result += 1
		if c > 0:
			if grid[r][c-1] == -1:
				result += 1
		if c < cols - 1:
			if grid[r][c+1] == -1:
				result += 1
		if r > 0:
			if grid[r-1][c] == -1:
				result += 1
		if r < rows - 1:
			if grid[r+1][c] == -1:
				result += 1
		return result

	def find_edges(self, bi_img, mul_param):
		sx = ndimage.sobel(bi_img, 0, mode='constant')
		sy = ndimage.sobel(bi_img, 1, mode='constant')
		edges = np.hypot(sx, sy)
		edges[edges < mul_param * np.max(edges)] = 0
		return edges

	def do_list_to_points(self, list):
		points = []
		for i in range(len(list)):
			p = Point()
			p.x = list[i][0]
			p.y = list[i][1]
			p.z = 0
			points.append(p)
		return points

	def cost(self, grid, horizon_ind, cr, cc):
		result = np.zeros(grid.shape, dtype=np.float32)
		for i in range(len(horizon_ind)):
			r = horizon_ind[i][0]; c = horizon_ind[i][1]
			result[r][c] = sqrt((r-cr)**2 + (c-cc)**2)
		return result

	def entropy(self, grid, flist):
		result = np.zeros(grid.shape, dtype=np.float32)
		grid[grid == -1] = 2
		grid[grid == 100] = 0
		for each in flist:
			result[each[0]][each[1]] = self.sum_nearfield(grid, each[0], each[1], radius=4)
		# result[result < 1920] = 0.0
		return result

	def info_gain(self, entropy, cost):
		info_gain = entropy - 0.5*cost
		return info_gain

	def sum_filter(self, grid, horizon_ind):
		result = np.zeros(grid.shape, dtype=np.float32)
		for i in range(len(horizon_ind)):
			r = horizon_ind[i][0]; c = horizon_ind[i][1]
			result[r][c] = self.sum_neighbors(grid, r, c)
		return result

	def get_max_ind(self, array):
		# ind = np.argwhere(array==np.amax(array))
		ind = []
		max = np.max(array)
		for i in range(array.shape[0]):
			for j in range(array.shape[1]):
				if array[i][j] == max:
					ind.append((i,j))
		return ind

	def sum_nearfield(self,  grid, cr, cc, radius):
		sum = 0
		for r in range(cr-radius, cr+radius):
			for c in range(cc-radius, cc+radius):
				if r < grid.shape[0] and r > 0 and c < grid.shape[1] and c > 0:
					sum += grid[r][c]
		return sum

	def sum_neighbors(self, grid, r, c):
		result = 0
		rows, cols = grid.shape
		if r > 0:
			result += grid[r-1][c]
			if c > 0:
				result += grid[r-1][c-1]
			if c < cols - 1:
				result += grid[r-1][c+1]

		if r < rows - 1:
			result += grid[r+1][c]
			if c > 0:
				result += grid[r+1][c-1]
			if c < cols - 1:
				result += grid[r+1][c+1]

		if c > 0:
			result += grid[r][c-1]
		if c < cols - 1:
			result += grid[r][c+1]

		result += grid[r][c]
		return result

	def logtofile(self, filename, data):
		np.set_printoptions(threshold=np.nan)
		f = open("thesisresults/" + filename, 'w')
		f.write('data = '+ repr(data) + '\n')
		f.close()
		np.set_printoptions(threshold=1)

	def pose_from_mark(self, goal):
		i, j = goal
		p = Pose()
		p.position.x = self.origin.position.x + i
		p.position.y = self.origin.position.y + j
		p.position.z = 0.0

		p.orientation.x = self.origin.orientation.x
		p.orientation.y = self.origin.orientation.y
		p.orientation.z = self.origin.orientation.z
		p.orientation.w = self.origin.orientation.w
		return p

	def get_npose_from_indices(self, goal):
		r, c = goal
		p = Pose()
		p.position.x = self.origin.position.x + c*self.resolution
		p.position.y = self.origin.position.y + r*self.resolution
		p.position.z = 0.0

		p.orientation.x = self.origin.orientation.x
		p.orientation.y = self.origin.orientation.y
		p.orientation.z = self.origin.orientation.z
		p.orientation.w = self.origin.orientation.w
		return p

	def mark_from_pose(self, gpose):
		x, y = gpose.position.x - self.origin.position.x, gpose.position.y - self.origin.position.y
		return int(floor(x)), int(floor(y))

	def pose_in_grid(self):
		c, r = abs(self.pose.position.x - self.origin.position.x)/self.resolution, abs(self.pose.position.y - self.origin.position.y)/self.resolution
		return int(floor(r)),int(floor(c))

	def odom_callback(self, data):
		self.pose = data.pose.pose

	def to_transtuple(self, p):
		return (p.x, p.y, p.z)

	def to_orientuple(self, q):
		return (q.x, q.y, q.z, q.w)

	def log(self, message):
		rospy.loginfo(message)

	def visualize(self, img1, img2, title1, title2):
		pyplt.subplot(1,2,1)
		pyplt.title(title1)
		pyplt.imshow(img1, cmap='binary', interpolation='none')
		pyplt.subplot(1,2,2)
		pyplt.title(title2)
		pyplt.imshow(img2, cmap='binary', interpolation='none')
		pyplt.show()

	def publish_origin_marker(self):
		marker = Marker()
		marker.header.frame_id = '/map'
		marker.id = 2
		marker.type = Marker.CUBE
		marker.action = Marker.ADD
		marker.scale.x = 0.2
		marker.scale.y = 0.2
		marker.scale.z = 0.2
		marker.color.a = 1.0
		marker.color.g = 1.0
		marker.pose = self.origin
		self.vizgoal = marker

	def publish_groundtruth_marker(self):
		marker = Marker()
		marker.header.frame_id = '/map'
		marker.id = 3
		marker.type = Marker.CUBE
		marker.action = Marker.ADD
		marker.scale.x = 0.2
		marker.scale.y = 0.2
		marker.scale.z = 0.2
		marker.color.a = 1.0
		marker.color.b = 1.0
		marker.pose.position.x = 0.0
		marker.pose.position.y = 0.0
		marker.pose.position.z = 0.0
		marker.pose.orientation.x = 0.0
		marker.pose.orientation.y = 0.0
		marker.pose.orientation.z = 0.0
		marker.pose.orientation.w = 0.0
		self.vizgoal = marker

	def publish_goal_marker(self, gpose):
		marker = Marker()
		marker.header.frame_id = '/map'
		marker.id = 1
		marker.type = Marker.CUBE
		marker.action = Marker.ADD
		marker.scale.x = 0.2
		marker.scale.y = 0.2
		marker.scale.z = 0.1
		marker.color.a = 1.0
		marker.color.r = 1.0
		marker.pose = gpose
		self.vizgoal = marker

		def get_potential(self, goal, origin, resolution, blocks):
			i, j = goal
			mapentropy =  self.get_entropy(blocks)

		################################### Unused. beg ############################################

		def get_entropy(self, blocks):
			return np.sum(np.add(np.multiply(blocks, np.log(blocks)), np.multiply(1 - blocks, np.log(1 - blocks))))

		def publish10(self, publisher, message):
			for i in range(1, 10):
				publisher.publish(message)
				self.rate.sleep()

		# def compute_frontier(self, blocks, origin, resolution):
		# 	frontier = []
		# 	rows, cols = blocks.shape
		# 	for i in range(rows):
		# 		for j in range(cols):
		# 			if self.is_interesting(blocks[i, j]):
		# 				# frontier.append(self.pose_from_mark((i, j), origin, resolution))
		# 				frontier.append(self.potential((i, j), origin, resolution, blocks))
		# 	return frontier

		def is_interesting(self, block):
			return True if block > 0.4 and block < 0.6 else False

		def find_closest(self, frontier):
			result = None
			mindist = 999

			for f in frontier:
				distance = sqrt((self.pose.position.x - f.position.x) ** 2 + (self.pose.position.y - f.position.y) ** 2)
				if distance < mindist:
					result = f
					mindist = distance

			return result

		def get_pose_stamped(self, translation, rotation):
			p = PoseStamped()
			p.header.stamp = rospy.Time.now()
			p.header.frame_id = 'map'
			p.pose.position.x = translation[0]
			p.pose.position.y = translation[1]
			p.pose.position.z = translation[2]

			p.pose.orientation.x = rotation[0]
			p.pose.orientation.y = rotation[1]
			p.pose.orientation.z = rotation[2]
			p.pose.orientation.w = rotation[3]

			return p

		def filter_frontier2(self, frontier, low, high):
			filtered = []
			for f in frontier:
				distance = sqrt((self.pose.position.x - f.position.x) ** 2 + (self.pose.position.y - f.position.y) ** 2)
				if distance > low and distance < high:
					filtered.append(f)

			return filtered

		# def filter_frontier(self, frontier):
		# 	return [f for f in frontier if self.is_pose_reachable(f)]

		def is_pose_reachable(self, pose):
			result = False

			try:
				resp = self.srv_make_plan( self.get_pose_stamped(self.odom.pose.pose)
										   , self.get_pose_stamped(pose) )
				if len(resp.path.poses) > 0:
					result = True
			except rospy.ServiceException, e:
				self.log("Service call failed: %s"%e)

			return results

		def is_window_explorable(self, window):
			return True if np.sum(window == -1) > 300 else False

	################################## Unused. end #################################################

def main():
	rospy.init_node('Grider', anonymous=True)
	Explorer(config.map_combiner['pub']['map_explore'], config.turtlebot['pub']['odom'], config.bot_mover['pub']['ready_state']).run()

if __name__ == '__main__':
	main()