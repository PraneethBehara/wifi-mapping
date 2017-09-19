#!/usr/bin/env python

import roslib
import rospy
from math import pi
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry
from simulate import WifiExpoSimulation
from visualization_msgs.msg import Marker

import pydb

class do_polygon:

    def __init__(self):
        self.pose = None
        self.Z = {}
        self.visited = []
        self.dop_db = {}
        self.wifi_db = {}
        self.dx_db = {}
        self.rate = rospy.Rate(10)
        self.trace = [(1, 0, 0), (0.707, 0.707, 0), (0, 1, 0), (-0.707, 0.707, 0), (-1, 0, 0), (-0.707, 0.707, 0),(0, -1, 0), (0.707, -0.707, 0)]
        self.pub_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        self.vizpub = rospy.Publisher('viz_aps', Marker, queue_size=10)

    def run(self):
        rospy.Subscriber('/odom', Odometry, self.callback, queue_size = 1)
        while not rospy.is_shutdown():
            self.do_loops()
            self.rate.sleep()

    def callback(self, data):
        self.pose = data.pose.pose

    def do_loops(self):
        print "loop complete"
        if not self.pose==None:
            self.move()
            self.move()
            self.move()
            self.turnl()
            self.move()
            self.move()
            self.move()
            self.turnl()
            self.move()
            self.move()
            self.move()
            self.move()
            self.move()
            self.move()
            self.move()
            self.turnl()
            # self.turnr()
            self.move()
            self.move()
            self.move()
            self.turnl()
            self.move()
            self.move()
            self.move()
            self.turnl()
            # self.move()
            # self.move()
            # self.turnr()
            # self.move()
            # self.move()
            # self.turnl()
            # self.move()
            # self.move()
            # self.turnl()
            # self.move()
            # self.move()
            # self.turnr()
            # self.move()
            # self.move()
            # self.turnl()
            # self.move()
            # self.move()
            # self.turnl()
            # self.move()
            # self.move()
            # self.turnr()

    def move(self):
        twist = Twist()
        start = rospy.get_time()
        while rospy.get_time() - start < 5:
            twist.linear.x = 0.5
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0
            self.pub_vel.publish(twist)
            self.rate.sleep()
        self.visited.append((self.pose.position.x, self.pose.position.y))
        print 'getting wifi_data'
        aplocs = WifiExpoSimulation(places=self.visited, Z=self.Z, dop_db=self.dop_db, dx_db=self.dx_db).run()
        if aplocs:
            self.update_apdb(aplocs)
        self.show_aps_db()

    def turnl(self):
        twist = Twist()
        start4 = rospy.get_time()
        while rospy.get_time() - start4 < 2:
            twist.linear.x = 0
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = pi / 3
            self.pub_vel.publish(twist)
            self.rate.sleep()

    def turnr(self):
        twist = Twist()
        start4 = rospy.get_time()
        while rospy.get_time() - start4 < 2:
            twist.linear.x = 0
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = -pi / 4
            self.pub_vel.publish(twist)
            self.rate.sleep()

    def update_apdb(self, aplocs):
        # pydb.debugger()
        for key in aplocs.keys():
            if key in self.wifi_db.keys():
                self.wifi_db[key] = aplocs[key]
            else:
                self.wifi_db.update({key: aplocs[key]})
        # print 'wifi_db: ', self.wifi_db

    def show_aps(self, aplocs):
        marker = Marker()
        marker.header.frame_id = '/odom'
        marker.id = 2
        marker.type = marker.CUBE_LIST
        marker.action = marker.ADD
        marker.scale.x = 0.75
        marker.scale.y = 0.75
        marker.scale.z = 0.5
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.b = 1.0
        marker.points = self.ap_points(aplocs)
        self.vizpub.publish(marker)
        self.rate.sleep()

    def show_aps_db(self):
        marker = Marker()
        marker.header.frame_id = '/odom'
        marker.id = 1
        marker.type = marker.CUBE_LIST
        marker.action = marker.ADD
        marker.scale.x = 0.75
        marker.scale.y = 0.75
        marker.scale.z = 0.5
        marker.color.a = 1.0
        marker.color.g = 1.0
        marker.color.r = 1.0
        marker.points = self.ap_points(self.wifi_db)
        self.vizpub.publish(marker)
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

    def cleanup(self):
        twist = Twist()
        self.pub_vel.publish(twist)

        # def simple_move(self):
        #     trace = [(1, 0, 0), (0.707, 0.707, 0), (0, 1, 0), (-0.707, 0.707, 0), (-1, 0, 0), (-0.707, 0.707, 0),(0, -1, 0), (0.707, -0.707, 0)]
        #     for each in trace:
        #         goal = MoveBaseGoal()
        #         goal.target_pose.pose.position.x = each[0]
        #         goal.target_pose.pose.position.y = each[1]
        #         goal.target_pose.pose.orientation.w = 1.0
        #         goal.target_pose.header.frame_id = 'odom'
        #         goal.target_pose.header.stamp = rospy.Time.now()
        #         self.sac.wait_for_server()
        #         print 'connected'
        #         self.sac.send_goal(goal)
        #         self.sac.wait_for_result()
        #         print self.sac.get_result()

if __name__=="__main__":
    rospy.init_node('move_client', anonymous=True)
    do_polygon().run()
