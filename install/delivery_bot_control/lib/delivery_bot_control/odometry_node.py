#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import JointState #joint state holds joint names, positions, velocities, and efforts (torques)
from nav_msgs.msg import Odometry #odometry holds pose and velocity of the robot at a given time
from geometry_msgs.msg import TransformStamped #transform stamped holds the transformation between two coordinate frames
from tf2_ros import TransformBroadcaster #transform broadcaster publishes transformations between coordinate frames
import numpy as np
import math
import time


#this node listens to joint states and computes the robot's odometry
#it publishes the odometry as an Odometry message and broadcasts the transformation
# this allows other nodes to know the robot's position and orientation in the world
class OdometryNode(Node):
    def __init__(self):
        super().__init__('omni_odometry')

        #parameters
        self.declare_parameter('wheel_radius', 0.024) #CHANGE VALUE
        self.declare_parameter('wheel_distance', 0.1155) #CHANGE VALUE
        r = self.get_parameter('wheel_radius').value
        L = self.get_parameter('wheel_distance').value

        #transformation matrix for wheel velocities
        #invert to compute linear and angular velocities from wheel velocities
        thetas = np.deg2rad([0, 120, 240])
        M = np.vstack([[-np.sin(t), np.cos(t), L] for t in thetas])
        self.M_inv = np.linalg.inv(M)

        #initialize position and orientation
        #x, y are the position in the world frame, theta is the orientation
        #last_time is used to compute the time difference between two consecutive joint states
        #multiply by 1e-9 to convert nanoseconds to seconds
        #dont use seconds directly as they are not precise enough
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now().nanoseconds * 1e-9

        #subscribe to joint states
        #whenever a new message is received, the joint_states_callback is called
        #queue size is set to 10
        self.joint_sub = self.create_subscription(
            JointState, 
            '/joint_states',
            self.joint_states_callback,
            10
        )
        #publisher for odometry messages
        #on the topic /odom with a queue size of 10
        #this allows other nodes to know the robot's position and orientation
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        #transform broadcaster to publish transformations between coordinate frames

        self.tf_broadcaster = TransformBroadcaster(self)

        self.joint_names = ['wheel1_joint', 
                            'wheel2_joint',
                            'wheel3_joint']
        self.r = r 

    def joint_states_callback(self, msg: JointState):

        # compute elapsed time since last callback
        # get the current time in seconds
        # and compute the difference with the last time
        # update the last_time to the current time
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = now - self.last_time
        self.last_time = now


        omega_wheels = [] #initialize a list to hold the wheel velocities
        for name in self.joint_names: #iterate over the joint names
            idx = msg.name.index(name) #find the index of the joint name in the message
            omega_wheels.append(msg.velocity[idx]) #append the wheel velocity to the list
        omega_wheels = np.array(omega_wheels) #convert the list to a numpy array

        v_rim = omega_wheels * self.r #compute the rim velocities by multiplying the wheel velocities by the wheel radius
        vx, vy, omega = self.M_inv.dot(v_rim) #compute the linear and angular velocities by multiplying the rim velocities by the inverse transformation matrix

        dx = vx * dt #compute the change in x position
        dy = vy * dt # compute the change in y position
        dtheta = omega * dt #compute the change in orientation

        self.x += dx * math.cos(self.theta) - dy * math.sin(self.theta) #compute the new x position
        self.y += dx * math.sin(self.theta) + dy * math.cos(self.theta) #compute the new y position
        self.theta += dtheta #compute the new orientation

        # f) publish Odometry message
        odom = Odometry() 
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        # convert θ → quaternion
        qz = math.sin(self.theta/2)
        qw = math.cos(self.theta/2)
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x  = vx
        odom.twist.twist.linear.y  = vy
        odom.twist.twist.angular.z = omega
        self.odom_pub.publish(odom)

        # g) broadcast TF
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z    = qz
        t.transform.rotation.w    = qw
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = OdometryNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    