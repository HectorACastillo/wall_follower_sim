#!/usr/bin/env python2

import numpy as np
from scipy import stats

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
    SIDE = rospy.get_param("wall_follower/side")
    VELOCITY = rospy.get_param("wall_follower/velocity")
    DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")
    SLICE_ANGLE = np.pi/2
    KP = 1.0
    KI = 1.0
    KD = 1.0

    integrator = 0.0

    def __init__(self):
        rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.callback)


    def callback(self, scan):
        pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size = 10)
        pub_error = rospy.Publisher("error", Float32, queue_size = 10)
        raw_ranges = np.array(scan.ranges)
        floor = int(raw_ranges.size/2 + (self.SIDE*np.pi - self.SLICE_ANGLE)/(2*scan.angle_increment))
        ceil = int(raw_ranges.size/2 + (self.SIDE*np.pi + self.SLICE_ANGLE)/(2*scan.angle_increment))
        range_slice = raw_ranges[floor:ceil]
        angle_slice = np.arange(floor, ceil)*scan.angle_increment + scan.angle_min
        wall_coords = np.array([np.cos(angle_slice), np.sin(angle_slice)])
        # find the wall
        slope, intercept, r, p, std_error = stats.linregress(wall_coords)

        # find closest point on line to calculate error
        e = self.DESIRED_DISTANCE - np.sqrt(np.amin(np.square(wall_coords[0,:])*np.square(wall_coords[1,:])))
        self.integrator += e
        pub_error.publish(e)

        ack = AckermannDriveStamped()
        ack.drive.steering_angle = self.KP*e + self.KD*slope
        
        ack.drive.steering_angle_velocity = 0.0
        ack.drive.speed = self.VELOCITY
        ack.drive.acceleration = 0.0
        ack.drive.jerk = 0.0
        rospy.loginfo(ack)
        pub.publish(ack)




if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()
