#!/usr/bin/env python

import tf
import rospy
import Queue as Q
import scipy as sc
import numpy as np
import matplotlib.pyplot as pyplt

from collections import defaultdict
from math import floor, sqrt
from scipy import ndimage
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose, Point
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import OccupancyGrid, Odometry
from drones_move.srv import *
from manual_measurements.msg import AccessPointInfo, WifiInfo
# from wmdatacollect.py import WifiExpo
from simulate import WifiExpoSimulation

import config

import pydb

class BotState:
	READY = 'READY'
	BUSY  = 'BUSY'

class BotStep:
	ROT = 'ROT'
	MOV = 'MOV'

class Publishers:

	def __init__(self, vizgoal=None, viz_aps=None, viz_inflated_obstacles=None, viz_filtered_horizon=None):
		self.vizgoal = vizgoal
		self.viz_aps = viz_aps
		self.viz_inflated_obstacles = viz_inflated_obstacles
		self.viz_filtered_horizon = viz_filtered_horizon

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

class Explorer:

	def __init__(self, grid, odom, readystate):
		self.grid = grid
		self.odom = odom
		self.readystate = readystate
		self.pose = None
		self.origin = None
		self.resolution = 0.05
		self.visited = []
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
							   , viz_filtered_horizon = rospy.Publisher(config.controller['pub']['viz_filtered_horizon'], Marker, queue_size=10)
							   , viz_inflated_obstacles = rospy.Publisher(config.controller['pub']['viz_inflated_obstacles'], Marker, queue_size=10))
		self.vizgoal = Marker()
		self.viz_aps = Marker()
		self.viz_filtered_horizon = Marker()
		self.viz_inflated_obstacles = Marker()

	def run(self):
		Subscribers([ (self.grid, OccupancyGrid, self.grid_callback)
						, (self.odom, Odometry, self.odom_callback)
						, (self.readystate, String, self.ready_state_callback) ])
		while not rospy.is_shutdown():
			self.pub.vizgoal.publish(self.vizgoal)
			self.pub.viz_aps.publish(self.viz_aps)
			self.pub.viz_inflated_obstacles.publish(self.viz_inflated_obstacles)
			self.pub.viz_filtered_horizon.publish(self.viz_filtered_horizon)
			self.rate.sleep()

	def ready_state_callback(self, data):
		self.state = data.data

	def move(self, command, pose):
		try:
			resp1 = self.srv_move_turtle(command, pose)
		except rospy.ServiceException, e:
			self.log("Service call failed: %s"%e)

	def grid_callback(self, data):
		aplocs = []
		self.log(self.state)
		self.publish_groundtruth_marker()
		if self.state == BotState.READY:
			self.origin = data.info.origin
			self.publish_origin_marker()
			self.tfbr.sendTransform( self.to_transtuple(self.origin.position)
									 , self.to_orientuple(self.origin.orientation)
									 , rospy.Time.now() , 'grid', 'world' )
			if self.botstep == BotStep.ROT:
				self.botstep = BotStep.MOV
				self.move('THREE60', Pose())
			elif self.botstep == BotStep.MOV:
				self.botstep = BotStep.ROT
				self.visited.append((self.pose.position.x, self.pose.position.y))
				#aplocs = WifiDataCollector(topic ='/APInfo', cc = '(self.pose.position.x, self.pose.position.y)', waits = '4').run()
				aplocs = WifiExpoSimulation(places = self.visited, Z=self.Z, dop_db=self.dop_db, dx_db=self.dx_db).run()
				if aplocs:
					self.update_apdb(aplocs)
				self.show_aps_db()
				grid     = ( np.array(data.data, dtype=np.float32)
							 .reshape(data.info.height, data.info.width) )
				cr, cc = self.get_indices_wrt_grid()
				horizon = self.get_horizon(grid, cr, cc)
				horizon_ind = self.get_max_ind(horizon)
				# self.publish_filtered_horizon(horizon_ind)
				# score = self.get_apscore_matrix(horizon)
				entropy = self.entropy(grid, horizon)
				cost = self.cost(grid, horizon_ind, cr, cc)
				gain = self.info_gain(entropy, cost)
				# queue = self.get_queue(gain)
				mark = np.unravel_index(gain.argmax(), gain.shape)
				gpose = self.get_npose_from_indices(mark)
				np.set_printoptions(threshold=0,precision=2)

				#BEG#~~~~~~~~~~~~~~~~~~ Debug Logs TBD ~~~~~~~~~~~~~~~~~~#BEG#
				self.log('Bot Pose: {0} {1}'.format(self.pose.position.x, self.pose.position.y))
				self.log('Bot Block: {0} {1}'.format(cr, cc))
				self.log('Horizon: \n{0}'.format(np.max(horizon)))
				self.log('Filtered_Horizon: \n{0}'.format(np.max(horizon)))
				self.log('Entropy: \n{0}'.format(entropy[mark[0]][mark[1]]))
				self.log('Cost: \n{0}'.format(cost[mark[0]][mark[1]]))
				self.log('Gain at horizon: \n{0}'.format(gain[mark[0]][mark[1]]))
				self.log('Next Block: \n{0} {1}'.format(mark[0], mark[1]))
				self.log('Next Pose: \n{0} {1}'.format(gpose.position.x, gpose.position.y))
				#END#~~~~~~~~~~~~~~~~~~ Debug Logs TBD ~~~~~~~~~~~~~~~~~~#END#

				#~~~Horizon check~~~#
				if horizon[mark[0]][mark[1]] == 1:
					print " _/ Mark on horizon"
				else:
					print " X Mark NOT on horizon!"
				#~~~~~~~~~~~~~~~~~~~#
				self.publish_goal_marker(gpose)
				self.move('MOVETO', gpose)

	def update_apdb(self, aplocs):
		# pydb.debugger()
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
		marker.type = marker.CUBE_LIST
		marker.action = marker.ADD
		marker.scale.x = 0.75
		marker.scale.y = 0.75
		marker.scale.z = 0.5
		marker.color.a = 1.0
		marker.color.g = 1.0
		marker.color.r = 1.0
		marker.text = 'some text'
		marker.points = self.ap_points(self.wifi_db)
		self.viz_aps = marker
		self.rate.sleep()

	def ap_points(self, dict):
		points = []
		for ap in dict.keys():
			p = Point()
			p.x = dict[ap][0]
			p.y = dict[ap][1]
			p.z = 3
			points.append(p)
		return points

	def get_queue(self, gain):
		q = Q.PriorityQueue()
		q.put(np.unravel_index(np.argmax(gain),gain.shape))
		for r in range (gain.shape[0]):
			for c in range(gain.shape[1]):
				if gain[r][c]>np.max(gain)/5: # and distance.euclidean(gain[r][c],) > 20:
					q.put([r,c])
		while not q.empty():
			print q.get()
		return q

	def get_horizon(self, grid, cr, cc):
		result = np.zeros(grid.shape, dtype=np.float32)
		rows, cols = grid.shape
		for r in range(rows):
			for c in range(cols):
				if grid[r][c] == 0:
					if self.has_unknown_neighbors(grid, r, c) > 4:
						result[r][c] = 1
		#self.visualize(grid,result)
		#horizon_unproc = self.filter_horizon(result, grid, cr, cc)
		horizon_proc = self.remove_obstacles(result, grid)
		return horizon_proc

	def filter_horizon(self, raw_horizon, grid, cr, cc):
		#raw_horizon = ndimage.binary_fill_holes(ndimage.gaussian_filter(raw_horizon, 1), structure=np.ones((4,4))).astype(int)
		edges = self.find_edges(raw_horizon, 0.8)
		edges[edges > 0] = 1
		self.visualize(edges,edges)
		#horizon = self.remove_nearfield(edges, grid, cr, cc)
		return edges

	def remove_obstacles(self, horizon, grid):
		cgrid = np.zeros(grid.shape, dtype=np.float32)
		cgrid = grid
		cgrid[grid > 0] = 2 * np.max(grid)
		smoothened_grid = self.gaussian_smooth(grid, 6.0)
		inflate_grid = self.highlight_obstacles_back(grid, smoothened_grid)
		# self.publish_inflated_obstacles(self.get_max_ind(inflate_grid))
		horizon[smoothened_grid > 2] = 0     #threshold_for_inflation
		horizon[horizon > 0] = 1
		return horizon

	def gaussian_smooth(self, image, sigma):
		smoothened_image = sc.ndimage.filters.gaussian_filter(image, sigma=sigma, order=0, output=None, mode='reflect', cval=0.0)
		return smoothened_image

	def highlight_obstacles_back(self, grid, smoothened_grid):
		smoothened_grid[grid == np.max(grid)] = 100
		return smoothened_grid

	def gaussian_filter(self, img, sd):
		img = ndimage.gaussian_filter(img, sd)
		return img

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

	def get_points(self, ind):
		points = []
		for i in range(len(ind)):
			p = Point()
			p.x = ind[i][0]
			p.y = ind[i][1]
			p.z = 0
			points.append(p)
		return points

	def publish_filtered_horizon(self, ind):
		marker = Marker()
		marker.header.frame_id = '/map'
		marker.id = 4
		marker.type = marker.POINTS
		marker.action = marker.ADD
		marker.scale.x = 0.05
		marker.scale.y = 0.05
		marker.scale.z = 0.05
		marker.color.a = 1.0
		marker.color.r = 1.0
		marker.points = self.get_points(ind)
		self.viz_filtered_horizon = marker

	def publish_inflated_obstacles(self, ind):
		marker = Marker()
		marker.header.frame_id = '/map'
		marker.id = 6
		marker.type = marker.POINTS
		marker.action = marker.ADD
		marker.scale.x = 0.05
		marker.scale.y = 0.05
		marker.scale.z = 0.05
		marker.color.a = 1.0
		marker.color.b = 1.0
		marker.points = self.get_points(ind)
		self.viz_inflated_obstacles = marker

	def publish_origin_marker(self):
		marker = Marker()
		marker.header.frame_id = '/map'
		marker.id = 2
		marker.type = marker.CUBE
		marker.action = marker.ADD
		marker.scale.x = 0.2
		marker.scale.y = 0.2
		marker.scale.z = 0.2
		marker.color.a = 1.0
		marker.color.r = 1.0
		marker.pose = self.origin
		self.vizgoal = marker

	def publish_groundtruth_marker(self):
		marker = Marker()
		marker.header.frame_id = '/map'
		marker.id = 3
		marker.type = marker.CUBE
		marker.action = marker.ADD
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
		marker.type = marker.CUBE
		marker.action = marker.ADD
		marker.scale.x = 0.2
		marker.scale.y = 0.2
		marker.scale.z = 0.2
		marker.color.a = 1.0
		marker.color.g = 1.0
		marker.pose = gpose
		self.vizgoal = marker

	def cost(self, grid, horizon_ind, i, j):
		result = np.zeros(grid.shape, dtype=np.float32)
		for i in range(len(horizon_ind)):
			r = horizon_ind[i][0]; c = horizon_ind[i][1]
			result[r][c] = sqrt( (r-i)**2 + (c-j)**2 )
		return result

	def entropy(self, grid, horizon):
		r, c = grid.shape
		result = np.zeros(grid.shape, dtype=np.float32)
		egrid = np.zeros(grid.shape, dtype=np.float32)
		egrid = grid
		egrid[grid == -1] = 2
		egrid[grid == 100] = 0
		for i in range(r):
			for j in range(c):
				if horizon[i][j]==1:
					result[i][j] = self.sum_nearfield(egrid, i, j)
		return result

	def info_gain(self, entropy, cost):
		info_gain = entropy - 0.02*cost
		return info_gain

	def get_cluster(self, observations):
		return sc.cluster.vq.kmeans2(observations, 4, iter=10, thresh=1e-05, minit='random', missing='warn')

	def sum_filter(self, grid, horizon_ind):
		result = np.zeros(grid.shape, dtype=np.float32)
		for i in range(len(horizon_ind)):
			r = horizon_ind[i][0]; c = horizon_ind[i][1]
			result[r][c] = self.sum_neighbors(grid, r, c)
		return result

	def get_max_ind(self, matrix):
		ind = np.argwhere(matrix==np.amax(matrix))
		return ind

	def sum_nearfield(self,  grid, cr, cc):
		sum = 0
		if grid.shape[0] - cr > 20 and grid.shape[0] + cr > 20 and grid.shape[1] + cc > 20 and grid.shape[1] - cc > 20:
			for r in range(-20,20):
				for c in range(-20,20):
					sum += grid[cr + r][cc + c]
				# print sum
		else :
			for r in range(grid.shape[0] - cr):
				for c in range(grid.shape[1] - cc):
					sum += grid[cr + r][cc + c]
				# print sum
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

	def pose_from_mark(self, mark):
		i, j = mark
		p = Pose()
		p.position.x = self.origin.position.x + i
		p.position.y = self.origin.position.y + j
		p.position.z = 0.0

		p.orientation.x = self.origin.orientation.x
		p.orientation.y = self.origin.orientation.y
		p.orientation.z = self.origin.orientation.z
		p.orientation.w = self.origin.orientation.w

		return p

	def get_npose_from_indices(self, mark):
		r, c = mark
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

	def get_indices_wrt_grid(self):
		c, r = abs(self.pose.position.x - self.origin.position.x)/self.resolution, abs(self.pose.position.y - self.origin.position.y)/self.resolution
		return int(floor(r)),int(floor(c))

	def odom_callback(self, data):
		self.pose = data.pose.pose

	# def get_current_mark(self):
	# 	return self.mark_from_pose(self.pose)

	def to_transtuple(self, p):
		return (p.x, p.y, p.z)

	def to_orientuple(self, q):
		return (q.x, q.y, q.z, q.w)

	def log(self, message):
		rospy.loginfo(message)

	def visualize(self, img1, img2):
		pyplt.subplot(1,2,1)
		pyplt.imshow(img1, interpolation='none')
		pyplt.subplot(1,2,2)
		pyplt.imshow(img2, interpolation='none')
		pyplt.show()

	def get_potential(self, mark, origin, resolution, blocks):
		i, j = mark
		mapentropy =  self.get_entropy(blocks)

	################################### Unused. beg ############################################

	def get_entropy(self, blocks):
		return np.sum(np.add(np.multiply(blocks, np.log(blocks)), np.multiply(1 - blocks, np.log(1 - blocks))))

	def publish10(self, publisher, message):
		for i in range(1, 10):
			publisher.publish(message)
			self.rate.sleep()

	def compute_frontier(self, blocks, origin, resolution):
		frontier = []

		rows, cols = blocks.shape
		for i in range(rows):
			for j in range(cols):
				if self.is_interesting(blocks[i, j]):
					# frontier.append(self.pose_from_mark((i, j), origin, resolution))
					frontier.append(self.potential((i, j), origin, resolution, blocks))
		return frontier

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

	def filter_frontier(self, frontier):
		return [f for f in frontier if self.is_pose_reachable(f)]

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