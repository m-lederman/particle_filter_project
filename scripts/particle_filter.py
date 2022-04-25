#!/usr/bin/env python3

import rospy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion, Point, Pose, PoseArray, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, String

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import numpy as np
from numpy.random import random_sample, normal
import math

from random import randint, random

from measurement_update_likelihood_field import compute_prob_zero_centered_gaussian
from likelihood_field import LikelihoodField

from sklearn.neighbors import NearestNeighbors
from copy import deepcopy


def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw"""

    yaw = (euler_from_quaternion([
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w])
            [2])

    return yaw


def draw_random_sample(choices, probabilities, n):
    """ Draws a random sample of n elements from a given list of choices and their specified probabilities.
    We recommend that you fill in this function using random_sample.
    """
    out = []
    boundaries = np.cumsum(probabilities)
    for _ in range(n):
        rand = random_sample()
        i = np.searchsorted(boundaries, rand)
        #print(f'i: {i}')
        out.append(choices[i])
    return out


class Particle:

    def __init__(self, pose, w):

        # particle pose (Pose object from geometry_msgs)
        self.pose = pose

        # particle weight
        self.w = w

    # helper functions for getting and setting particle
    # position and facing angle
    def get_x(self):
        '''
        Returns the x-position of the particle
        '''
        return self.pose.position.x

    def get_y(self):
        '''
        Returns the y-position of the particle
        '''
        return self.pose.position.y

    def set_x(self, x):
        '''
        Sets the x position of the particle to `x`
        '''
        self.pose.position.x = x

    def set_y(self, y):
        '''
        Sets the y position of the particle to `y`
        '''
        self.pose.position.y = y

    def get_yaw(self):
        '''
        Returns the facing angle of the particle
        '''
        return get_yaw_from_pose(self.pose)

    def set_yaw(self, yaw):
        '''
        Sets the facing angle of the particle to `yaw`
        '''
        euler = list(euler_from_quaternion([self.pose.orientation.x,
                                            self.pose.orientation.y,
                                            self.pose.orientation.z,
                                            self.pose.orientation.w]))
        euler[2] = yaw
        self.pose.orientation = Quaternion(*quaternion_from_euler(*euler))


class ParticleFilter:


    def __init__(self):

        # once everything is setup initialized will be set to true
        self.initialized = False        


        # initialize this particle filter node
        rospy.init_node('turtlebot3_particle_filter')

        # set the topic names and frame names
        self.base_frame = "base_footprint"
        self.map_topic = "map"
        self.odom_frame = "odom"
        self.scan_topic = "scan"

        # inialize our map
        self.map = OccupancyGrid()

        # the number of particles used in the particle filter
        self.num_particles = 500

        # initialize the particle cloud array
        self.particle_cloud = []

        # initialize the estimated robot pose
        self.robot_estimate = Pose()

        # set threshold values for linear and angular movement before we preform an update
        self.lin_mvmt_threshold = 0.1        
        self.ang_mvmt_threshold = (np.pi / 6)

        self.odom_pose_last_motion_update = None


        # Setup publishers and subscribers

        # publish the current particle cloud
        self.particles_pub = rospy.Publisher("particle_cloud", PoseArray, queue_size=10)

        # publish the estimated robot pose
        self.robot_estimate_pub = rospy.Publisher("estimated_robot_pose", PoseStamped, queue_size=10)

        # subscribe to the map server
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.get_map)

        # subscribe to the lidar scan from the robot
        rospy.Subscriber(self.scan_topic, LaserScan, self.robot_scan_received)

        # enable listening for and broadcasting corodinate transforms
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()

        # initialize the likelihood field
        self.likelihood_field = LikelihoodField()

        # intialize the particle cloud
        self.initialize_particle_cloud()

        self.initialized = True



    def get_map(self, data):
        self.map = data
    

    def initialize_particle_cloud(self):

        # wait until we have received the map from the map server
        while not self.map.data:
            pass

        # occupancy values below the threshold are considered open
        # (values range from 0 to 100)
        threshold = 50
        # get all of the open positions in the map
        open_spaces = []
        map_width = self.map.info.width
        map_origin_x = self.map.info.origin.position.x
        map_origin_y = self.map.info.origin.position.y
        map_yaw = get_yaw_from_pose(self.map.info.origin)
        map_resolution = self.map.info.resolution
        for i, val in enumerate(self.map.data):
            if val < threshold and val != -1:
                # this cell is open: calculate the real position of the cell
                # and add it to the list
                cell_y = i // map_width
                cell_x = i % map_width
                position_y = map_origin_y + map_resolution * (cell_y * np.cos(map_yaw) - cell_x * np.sin(map_yaw))
                position_x = map_origin_x + map_resolution * (cell_x * np.cos(map_yaw) + cell_y * np.sin(map_yaw))
                open_spaces.append((position_y, position_x))

        # draw uniformly distributed positions from list using draw_random_sample
        # (this approach would make using different starting distributions easier)
        probabilities = [1.0 / len(open_spaces) for _ in open_spaces]
        positions = draw_random_sample(open_spaces, probabilities, self.num_particles)

        # create particles at these positions with random yaw and uniform weights
        self.particle_cloud = []
        for y, x in positions:
            point = Point()
            point.x = x
            point.y = y
            point.z = 0
            orientation = Quaternion(*quaternion_from_euler(0.0, 0.0, random_sample() * 2 * np.pi))
            pose = Pose()
            pose.position = point
            pose.orientation = orientation
            particle = Particle(pose, 1.0)
            self.particle_cloud.append(particle)

        self.normalize_particles()

        self.publish_particle_cloud()


    def normalize_particles(self):
        # make all the particle weights sum to 1.0
        total_weight = sum(particle.w for particle in self.particle_cloud)
        if total_weight == 0.0:
            # if all weights are 0.0, make the weights uniform
            # probably won't happen here but useful general edge case
            uniform_w = 1.0 / self.num_particles
            for particle in self.particle_cloud:
                particle.w = uniform_w
        else:
            for particle in self.particle_cloud:
                particle.w = particle.w / total_weight



    def publish_particle_cloud(self):

        particle_cloud_pose_array = PoseArray()
        particle_cloud_pose_array.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        particle_cloud_pose_array.poses = []

        for part in self.particle_cloud:
            particle_cloud_pose_array.poses.append(part.pose)

        self.particles_pub.publish(particle_cloud_pose_array)




    def publish_estimated_robot_pose(self):

        robot_pose_estimate_stamped = PoseStamped()
        robot_pose_estimate_stamped.pose = self.robot_estimate
        robot_pose_estimate_stamped.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        self.robot_estimate_pub.publish(robot_pose_estimate_stamped)



    def resample_particles(self):
        probabilities = [particle.w for particle in self.particle_cloud]
        #print(probabilities, sum(probabilities))
        new_particles_indexes = draw_random_sample(list(range(self.num_particles)), probabilities, self.num_particles)
        new_particles = []
        for i in new_particles_indexes:
            new_particles.append(deepcopy(self.particle_cloud[i]))
        self.particle_cloud = new_particles



    def robot_scan_received(self, data):

        # wait until initialization is complete
        if not(self.initialized):
            return

        # we need to be able to transfrom the laser frame to the base frame
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # wait for a little bit for the transform to become avaliable (in case the scan arrives
        # a little bit before the odom to base_footprint transform was updated) 
        self.tf_listener.waitForTransform(self.base_frame, self.odom_frame, data.header.stamp, rospy.Duration(0.5))
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # calculate the pose of the laser distance sensor 
        p = PoseStamped(
            header=Header(stamp=rospy.Time(0),
                          frame_id=data.header.frame_id))

        self.laser_pose = self.tf_listener.transformPose(self.base_frame, p)

        # determine where the robot thinks it is based on its odometry
        p = PoseStamped(
            header=Header(stamp=data.header.stamp,
                          frame_id=self.base_frame),
            pose=Pose())

        self.odom_pose = self.tf_listener.transformPose(self.odom_frame, p)

        # we need to be able to compare the current odom pose to the prior odom pose
        # if there isn't a prior odom pose, set the odom_pose variable to the current pose
        if not self.odom_pose_last_motion_update:
            self.odom_pose_last_motion_update = self.odom_pose
            return


        if self.particle_cloud:

            # check to see if we've moved far enough to perform an update

            curr_x = self.odom_pose.pose.position.x
            old_x = self.odom_pose_last_motion_update.pose.position.x
            curr_y = self.odom_pose.pose.position.y
            old_y = self.odom_pose_last_motion_update.pose.position.y
            curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
            old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

            if (np.abs(curr_x - old_x) > self.lin_mvmt_threshold or 
                np.abs(curr_y - old_y) > self.lin_mvmt_threshold or
                np.abs(curr_yaw - old_yaw) > self.ang_mvmt_threshold):
                print("Updating")

                # This is where the main logic of the particle filter is carried out

                self.update_particles_with_motion_model()
                print('done1')
                self.update_particle_weights_with_measurement_model(data)
                print('done2')

                self.normalize_particles()
                print('done3')

                self.resample_particles()
                print('done4')

                self.normalize_particles()
                print('done5')

                self.update_estimated_robot_pose()
                print('done6')

                self.publish_particle_cloud()
                self.publish_estimated_robot_pose()
                print('done7')

                self.odom_pose_last_motion_update = self.odom_pose
                print('done')



    def update_estimated_robot_pose(self):
        # based on the particles within the particle cloud, update the robot pose estimate
        
        #calculate the average x and y position and orientation of the robot and push those to a new particle
        averagepose = Pose()
        for particle in self.particle_cloud:
            averagepose.position.x += particle.pose.position.x * particle.w
            averagepose.position.y += particle.pose.position.y * particle.w
            averagepose.orientation.z += particle.pose.orientation.z * particle.w
        
        #verify that there are particles near the average point
        particlesnearby = 0
        for particle in self.particle_cloud:
            for x in range(0,6):
                for y in range(0,6):
                    if particle.pose.position.x == averagepose.position.x + x and particle.pose.position.y == averagepose.position.y + y:
                        particlesnearby += 1
                
        #find the particle with the largest weight
        greatest_weight = self.particle_cloud[0]
        for particle in self.particle_cloud:
            if particle.w > greatest_weight.w:
                greatest_weight = particle
   
        #if there aren't enough particles near the average point, assume there is an error and settle for the point with the largest weight
        #if particlesnearby >= 10:
        #self.robot_estimate = averagepose
        # else:
        self.robot_estimate = greatest_weight.pose
        



    
    def update_particle_weights_with_measurement_model(self, data):
        # Given the laser scan data, compute the importance weights (w) 
        # for each particle using the likelihood field measurement algorithm.

        z_hit = 0.95
        z_random = 0.05

        for particle in self.particle_cloud:
            # for each particle, set weights to product of gaussian likelihoods for
            # each distance reading from the laser scan
            q = 1.0
            for scan_direction in range(0, 360, 15):
                if data.ranges[scan_direction] != 0.0:
                    scan_end_x = particle.get_x() + data.ranges[scan_direction] * np.cos(particle.get_yaw() + scan_direction)
                    scan_end_y = particle.get_y() + data.ranges[scan_direction] * np.sin(particle.get_yaw() + scan_direction)
                    dist = self.likelihood_field.get_closest_obstacle_distance(scan_end_x, scan_end_y)
                    # get_closest_obsticle_distance returns NaN if coordinates are outside
                    # of full map boundaries (including -1's outside maze)
                    if not math.isnan(dist):
                        # use standard deviation 0.1
                        q *= z_hit * compute_prob_zero_centered_gaussian(dist, 0.75) + z_random / 4.0
            # set weight to likelihood
            particle.w = q


        

    def update_particles_with_motion_model(self):
        # based on the how the robot has moved (calculated from its odometry), we'll  move
        # all of the particles correspondingly

        curr_x = self.odom_pose.pose.position.x
        old_x = self.odom_pose_last_motion_update.pose.position.x
        curr_y = self.odom_pose.pose.position.y
        old_y = self.odom_pose_last_motion_update.pose.position.y
        curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
        old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

        dx = curr_x - old_x
        dy = curr_y - old_y
        avg_yaw = (curr_yaw + old_yaw) / 2

        print(f'yaw: {curr_yaw}')
        print(f'dx: {dx}')
        print(f'dy: {dy}')
        # linear movement in direction of robot orientation (positive forward, negative backward)
        linear_movement = dx * np.cos(avg_yaw) + dy * np.sin(avg_yaw)
        dyaw = curr_yaw - old_yaw
        for particle in self.particle_cloud:
            # rotate particle
            particle.set_yaw(particle.get_yaw() + dyaw * normal(1.0, 0.1))

            # move particle
            particle_linear_movement = linear_movement * normal(1.0, 0.1)
            particle.set_x(particle.get_x() + particle_linear_movement * np.cos(particle.get_yaw()))
            particle.set_y(particle.get_y() + particle_linear_movement * np.sin(particle.get_yaw()))


if __name__=="__main__":
    

    pf = ParticleFilter()

    rospy.spin()









