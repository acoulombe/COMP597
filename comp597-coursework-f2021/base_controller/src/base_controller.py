#!/usr/bin/env python3

import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, Twist
from gazebo_msgs.msg import ModelStates
from std_srvs.srv import Trigger
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from planner import Planner

class base_controller_node():

    def __init__(self):
        
        # Initialize ROS
        rospy.init_node('base_controller', anonymous=False)

        # Initialize publisher and subscriber streams
        cmd_topic = rospy.get_param('base_controller/cmd_topic_name')
        self.twistPublisher = rospy.Publisher(cmd_topic, Twist, queue_size=1)
        self.stateSubscriber = rospy.Subscriber('/gazebo/model_states', ModelStates, self.stateCallback)
        self.laserSubscriber = rospy.Subscriber('/scan', LaserScan, self.scanCallback)

        # Make Trigger service to start robot
        self.starterService = rospy.Service("base_controller/start", Trigger, self.startRobot)
        self.start = False

        # Set control frequency
        f = rospy.get_param('base_controller/update_frequency')
        self.rate = rospy.Rate(f)

        # Get Planner parameters
        self.model_name = rospy.get_param('base_controller/model_name')

        current_state = np.array([
            rospy.get_param('base_controller/initial/x'),
            rospy.get_param('base_controller/initial/y'),
            rospy.get_param('base_controller/initial/phi')
        ])
        self.pose = current_state

        target_state = np.array([
            rospy.get_param('base_controller/target/x'),
            rospy.get_param('base_controller/target/y'),
            rospy.get_param('base_controller/target/phi')
        ])

        min_d = rospy.get_param('base_controller/min_distance_to_obstacles')
        
        # Get robot limits
        self.v_lim = rospy.get_param('base_controller/limit/speed')
        self.w_lim = rospy.get_param('base_controller/limit/omega')

        self.plan = Planner(min_d)
        self.plan.set_goal(target_state)

        self.run()

    def startRobot(self, req):
        self.start = True
        return True, "Started Robot"

    def scanCallback(self, msg):
        # Get locations of obstacle points
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_inc = msg.angle_increment
        angles = np.arange(angle_max, angle_min-angle_inc, -angle_inc)

        # Get distance from the data
        range_min = msg.range_min
        range_max = msg.range_max
        ranges = list(msg.ranges)

        # Make the obstacle models
        self.obstacles_d = np.vstack(
            [angles, ranges]
        )

    def stateCallback(self, state):
        # Get model data
        names = state.name
        idx = names.index(self.model_name)
        p = state.pose[idx]
        
        # Get Euler angles
        quat = [
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w,
        ]
        euler = euler_from_quaternion(quat)

        # Get velocities
        v = state.twist[idx]

        self.pose = np.array([
            p.position.x,
            p.position.y,
            euler[2],
            v.linear.x,
            v.linear.y,
            v.angular.z
        ])

    def run(self):
        while not rospy.is_shutdown():
            if self.start:
                cmd = Twist()

                a = self.plan.get_action(self.pose, self.obstacles_d)

                v = np.sqrt(a[0] ** 2 + a[1] ** 2)
                w = a[2]

                # Check velocity limits
                if abs(v) > self.v_lim:
                    v = np.sign(v)*self.v_lim

                if abs(w) > self.w_lim:
                    w = np.sign(w) * self.w_lim

                cmd.linear.x = v
                cmd.angular.z = w

                self.twistPublisher.publish(cmd)
                
            self.rate.sleep()


if __name__ == '__main__':
    n = base_controller_node()
    rospy.spin()
