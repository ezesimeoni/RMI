#!/usr/bin/env python

""" This is the starter code for the robot localization project """

import rospy

from std_msgs.msg import Header, String, ColorRGBA
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion, Vector3
from nav_msgs.srv import GetMap
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from copy import deepcopy

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
from random import gauss

import math
import random
import time

import numpy as np
from numpy.random import random_sample, normal
from occupancy_field import OccupancyField

from helper_functions import (convert_pose_inverse_transform,
                              convert_translation_rotation_to_pose,
                              convert_pose_to_xy_and_theta,
                              angle_diff,sum_vectors)

class Particle(object):
    def __init__(self,x=0.0,y=0.0,theta=0.0,w=1.0):
        self.w = w
        self.theta = theta
        self.x = x
        self.y = y

    def as_pose(self):
        orientation_tuple = tf.transformations.quaternion_from_euler(0,0,self.theta)
        return Pose(position=Point(x=self.x,y=self.y,z=0), orientation=Quaternion(x=orientation_tuple[0], y=orientation_tuple[1], z=orientation_tuple[2], w=orientation_tuple[3]))


class ParticleFilter:
    """ The class that represents a Particle Filter ROS Node
        Attributes list:
            initialized: a Boolean flag to communicate to other class methods that initializaiton is complete
            base_frame: the name of the robot base coordinate frame (should be "base_link" for most robots)
            map_frame: the name of the map coordinate frame (should be "map" in most cases)
            odom_frame: the name of the odometry coordinate frame (should be "odom" in most cases)
            scan_topic: the name of the scan topic to listen to (should be "scan" in most cases)
            n_particles: the number of particles in the filter
            linear_mov: the amount of linear movement before triggering a filter update
            angular_mov: the amount of angular movement before triggering a filter update
            laser_max_distance: the maximum distance to an obstacle we should use in a likelihood calculation
            pose_listener: a subscriber that listens for new approximate pose estimates (i.e. generated through the rviz GUI)
            particle_pub: a publisher for the particle cloud
            laser_subscriber: listens for new scan data on topic self.scan_topic
            tf_listener: listener for coordinate transforms
            particle_cloud: a list of particles representing a probability distribution over robot poses
            current_odom_xy_theta: the pose of the robot in the odometry frame when the last filter update was performed.
                                   The pose is expressed as a list [x,y,theta] (where theta is the yaw)
            map: the map we will be localizing ourselves in.  The map should be of type nav_msgs/OccupancyGrid
    """
    def __init__(self):
        self.initialized = False        # make sure we don't perform updates before everything is setup
        rospy.init_node('RMI_pf')

        self.base_frame = "base_link"   # the frame of the robot base
        self.map_frame = "map"          # the name of the map coordinate frame
        self.odom_frame = "odom"        # the name of the odometry coordinate frame
        self.scan_topic = "scan"        # the topic where we will get laser scans from

        self.n_particles = 20
        self.linear_mov = 0.1
        self.angular_mov = math.pi/10
        self.laser_max_distance = 2.0
        self.sigma = 0.05

        # Descomentar essa linha caso /initialpose seja publicada
        # self.pose_listener = rospy.Subscriber("initialpose", 
        #     PoseWithCovarianceStamped, 
        #     self.update_initial_pose)
        self.laser_subscriber = rospy.Subscriber(self.scan_topic, LaserScan, self.scan_received)
        self.particle_pub = rospy.Publisher("particlecloud_rmi", PoseArray, queue_size=1)
        self.tf_listener = TransformListener()

        self.particle_cloud = []
        self.current_odom_xy_theta = []

        self.map_server = rospy.ServiceProxy('static_map', GetMap)
        self.map = self.map_server().map
        self.occupancy_field = OccupancyField(self.map)
        self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(1.0))
        
        self.initialized = True

    def update_particles_with_odom(self, msg):
        new_odom_xy_theta = convert_pose_to_xy_and_theta(self.odom_pose.pose)
        # print 'new_odom_xy_theta', new_odom_xy_theta
        # Pega a posicao da odom (x,y,tehta)
        if self.current_odom_xy_theta:
            old_odom_xy_theta = self.current_odom_xy_theta
            delta = (new_odom_xy_theta[0] - self.current_odom_xy_theta[0],
                     new_odom_xy_theta[1] - self.current_odom_xy_theta[1],
                     new_odom_xy_theta[2] - self.current_odom_xy_theta[2])
            self.current_odom_xy_theta = new_odom_xy_theta
            # print 'delta', delta
        else:
            self.current_odom_xy_theta = new_odom_xy_theta
            return

        for particle in self.particle_cloud:
            d = math.sqrt((delta[0]**2) + (delta[1]**2))
            # print 'particle_theta_1', particle.theta
            particle.x += d * (math.cos(particle.theta) + normal(0,0.01))
            particle.y += d * (math.sin(particle.theta) + normal(0,0.01))
            particle.theta = self.current_odom_xy_theta[2] #+ normal(0,0.05)

    # Systematic Resample
    def resample_particles(self):
        self.normalize_particles()
        # for particle in self.particle_cloud:
            # print 'TODAS PART', particle.w, particle.x, particle.y
        weights = []
        for particle in self.particle_cloud:
            weights.append(particle.w)

        newParticles = []
        N = len(weights)

        positions = (np.arange(N) + random.random()) / N

        cumulative_sum = np.cumsum(weights)
        i, j = 0, 0
        while i < N:
            if positions[i] < cumulative_sum[j]:
                newParticles.append(deepcopy(self.particle_cloud[j]))
                i += 1
            else:
                j += 1

        self.particle_cloud = newParticles

    def update_particles_with_laser(self, msg):
        depths = []
        for dist in msg.ranges:
            if not np.isnan(dist):
                depths.append(dist)
        fullDepthsArray = msg.ranges[:]

        # Verifica se ha objetos proximos ao robot
        if len(depths) == 0:
            self.closest = 0
            self.position = 0
        else:
            self.closest = min(depths)
            self.position = fullDepthsArray.index(self.closest)
        # print 'self.position, self.closest', self.position, self.closest, self.xy_theta_aux
        # print msg, '/scan'

        for index, particle in enumerate(self.particle_cloud):
            tot_prob = 0.0
            for index, scan in enumerate(depths):
                x,y = self.transform_scan(particle, scan, index)
                # print 'x,y, scan', x, y, scan
                # usa o metodo get_closest_obstacle_distance para buscar o obstaculo mais proximo dentro do range x,y do grid map
                d = self.occupancy_field.get_closest_obstacle_distance(x,y)
                # quanto mais proximo de zero mais relevante 
                tot_prob += math.exp((-d**2)/(2*self.sigma**2))

            tot_prob = tot_prob/len(depths)
            if math.isnan(tot_prob):
                particle.w = 1.0
            else:
                particle.w = tot_prob
            # print 'LASER', particle.x, particle.y, particle.w

    def transform_scan(self, particle, distance, theta):
        return (particle.x + distance * math.cos(math.radians(particle.theta + theta)),
                particle.y + distance * math.sin(math.radians(particle.theta + theta)))

    def update_initial_pose(self, msg):
        xy_theta = convert_pose_to_xy_and_theta(msg.pose.pose)
        self.initialize_particle_cloud(xy_theta)

    def initialize_particle_cloud(self, xy_theta=None):
        print 'Cria o set inicial de particulas'
        if xy_theta == None:
            xy_theta = convert_pose_to_xy_and_theta(self.odom_pose.pose)
            x, y, theta = xy_theta
        
        # Altere este parametro para aumentar a circunferencia do filtro de particulas
        # Na VM ate 1 e suportado
        rad = 0.5

        self.particle_cloud = []
        self.particle_cloud.append(Particle(xy_theta[0], xy_theta[1], xy_theta[2]))

        # print 'particle_values_W', self.particle_cloud[0].w
        # print 'particle_values_X', self.particle_cloud[0].x
        # print 'particle_values_Y', self.particle_cloud[0].y
        # print 'particle_values_THETA', self.particle_cloud[0].theta

        for i in range(self.n_particles-1):
            # initial facing of the particle
            theta = random.random() * 360

            # compute params to generate x,y in a circle
            other_theta = random.random() * 360
            radius = random.random() * rad
            # x => straight ahead
            x = radius * math.sin(other_theta) + xy_theta[0]
            y = radius * math.cos(other_theta) + xy_theta[1]
            particle = Particle(x,y,theta)
            self.particle_cloud.append(particle)

        self.normalize_particles()

    def normalize_particles(self):
        tot_weight = sum([particle.w for particle in self.particle_cloud]) or 1.0
        for particle in self.particle_cloud:
            particle.w = particle.w/tot_weight

    def publish_particles(self, msg):
        particles_conv = []
        for p in self.particle_cloud:
            particles_conv.append(p.as_pose()) # transforma a particula em POSE para ser entendida pelo ROS
        # print 'PARTII', [particles.x for particles in self.particle_cloud]
        # Publica as particulas no rviz (particloud_rmi)
        self.particle_pub.publish(PoseArray(header=Header(stamp=rospy.Time.now(), frame_id=self.map_frame), poses=particles_conv))

    def scan_received(self, msg):
        # print msg
        """ This is the default logic for what to do when processing scan data.
            Feel free to modify this, however, I hope it will provide a good
            guide.  The input msg is an object of type sensor_msgs/LaserScan """
        if not(self.initialized):
            # wait for initialization to complete
            return

        if not(self.tf_listener.canTransform(self.base_frame,msg.header.frame_id,msg.header.stamp)):
            # need to know how to transform the laser to the base frame
            # this will be given by either Gazebo or neato_node
            return

        if not(self.tf_listener.canTransform(self.base_frame,self.odom_frame,msg.header.stamp)):
            # need to know how to transform between base and odometric frames
            # this will eventually be published by either Gazebo or neato_node
            return

        # print 'msg.header.frame_id', msg.header.frame_id
        # calculate pose of laser relative ot the robot base
        p = PoseStamped(header=Header(stamp=rospy.Time(0),
                                      frame_id=msg.header.frame_id))
        self.laser_pose = self.tf_listener.transformPose(self.base_frame,p)

        # find out where the robot thinks it is based on its odometry
        # listener.getLatestCommonTime("/base_link",object_pose_in.header.frame_id)
        # p = PoseStamped(header=Header(stamp=msg.header.stamp,
        p = PoseStamped(header=Header(stamp=self.tf_listener.getLatestCommonTime(self.base_frame, self.map_frame),
        # p = PoseStamped(header=Header(stamp=rospy.Time.now(),
                                      frame_id=self.base_frame),
                        pose=Pose())
        # p_aux = PoseStamped(header=Header(stamp=self.tf_listener.getLatestCommonTime("/base_link","/map"),
        p_aux = PoseStamped(header=Header(stamp=self.tf_listener.getLatestCommonTime(self.odom_frame, self.map_frame),
        # p_aux = PoseStamped(header=Header(stamp=rospy.Time.now(),
                                      frame_id=self.odom_frame),
                        pose=Pose())
        odom_aux = self.tf_listener.transformPose(self.map_frame, p_aux)
        odom_aux_xy_theta = convert_pose_to_xy_and_theta(odom_aux.pose)
        # print 'odom_aux_xy_theta', odom_aux_xy_theta

        self.odom_pose = self.tf_listener.transformPose(self.odom_frame, p)
        # print 'self.odom_pose', self.odom_pose
        # (trans, root) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        # self.odom_pose = trans
        # print trans, root
        new_odom_xy_theta = convert_pose_to_xy_and_theta(self.odom_pose.pose)
        # new_odom_xy_theta = convert_pose_to_xy_and_theta(self.laser_pose.pose)
        xy_theta_aux = (new_odom_xy_theta[0]+odom_aux_xy_theta[0], 
            new_odom_xy_theta[1]+odom_aux_xy_theta[1], new_odom_xy_theta[2])
        self.xy_theta_aux = xy_theta_aux

        if not(self.particle_cloud):
            self.initialize_particle_cloud(xy_theta_aux)
            self.current_odom_xy_theta = new_odom_xy_theta

        elif (math.fabs(new_odom_xy_theta[0] - self.current_odom_xy_theta[0]) > self.linear_mov or
              math.fabs(new_odom_xy_theta[1] - self.current_odom_xy_theta[1]) > self.linear_mov or
              math.fabs(new_odom_xy_theta[2] - self.current_odom_xy_theta[2]) > self.angular_mov):

            self.update_particles_with_odom(msg)
            self.update_particles_with_laser(msg)
            self.resample_particles()

        self.publish_particles(msg)

if __name__ == '__main__':
    n = ParticleFilter()
    r = rospy.Rate(5)

    while not(rospy.is_shutdown()):
        r.sleep()
