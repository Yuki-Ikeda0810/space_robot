#!/usr/bin/env python3
# -*- encoding:utf-8 -*-
import rospy
import sys
import math
import os

from std_msgs.msg import Float64MultiArray
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

class Demo(object):
    """ Demo class """
    def __init__(self):
        """ Initializer """
        # command messages
        self.cmd_msg1 = Float64MultiArray()
        self.cmd_msg2 = Float64MultiArray()
        self.cmd_msg3 = Float64MultiArray()
        self.cmd_msg4 = Float64MultiArray()
        self.cmd_msg5 = Float64MultiArray()

        self.cmd_msg_vel = Twist()

        # publishers
        self.pub_arm1 = rospy.Publisher('/chaser/cmd_arm1', Float64MultiArray, queue_size=10)
        self.pub_arm2 = rospy.Publisher('/chaser/cmd_arm2', Float64MultiArray, queue_size=10)
        self.pub_arm3 = rospy.Publisher('/chaser/cmd_arm3', Float64MultiArray, queue_size=10)
        self.pub_arm4 = rospy.Publisher('/chaser/cmd_arm4', Float64MultiArray, queue_size=10)
        self.pub_arm5 = rospy.Publisher('/chaser/cmd_arm5', Float64MultiArray, queue_size=10)
        self.pub_chaser_vel = rospy.Publisher('/chaser/cmd_vel', Twist, queue_size=10)

    def model_state(self, message):
        """ get model name and pose
        @param message: model state published from gazebo
        @type  message: gazebo_msgs/ModelStates
        """
        self.chaser_name = message.name[0]
        self.chaser_pose = message.pose[0]
        self.debris_name = message.name[1]
        self.debris_pose = message.pose[1]

    def arm_state1(self, message):
        """ get arm state from /chaser/obs_arm1
        @param message: arm joint state published from chaser plugin
        @type  message: sensor_msgs/JointState
        """
        self.arm_joint_1_position = message.position[0]
        self.arm_joint_2_position = message.position[1]

    def arm_state2(self, message):
        """ get arm state from /chaser/obs_arm2
        @param message: arm joint state published from chaser plugin
        @type  message: sensor_msgs/JointState
        """
        self.arm_joint_3_position = message.position[0]
        self.arm_joint_4_position = message.position[1]

    def arm_state3(self, message):
        """ get arm state from /chaser/obs_arm3
        @param message: arm joint state published from chaser plugin
        @type  message: sensor_msgs/JointState
        """
        self.arm_joint_5_position = message.position[0]
        self.arm_joint_6_position = message.position[1]
        pass

    def create_command(self, name, pose, move_flag):
        """ create command to change object pose
        @param pose: target pose
        @type  pose: dict
        @param use_rostopic: True/use rostopic pub, False/use rosservice
        @type  use_rostopic: boolean
        @param move_flag: set x linear velocity or not
        @type  move_flag: boolean
        @returns: str
        """

        if move_flag:
            linear_vel  = 0.1
            angular_vel = 0.5
        else:
            linear_vel  = 0.0
            angular_vel = 0.0

        command_head = "rosservice call /gazebo/set_model_state '{model_state: {model_name: %s, pose:{ position: {x: %f, y: %f, z: %f}, orientation: {x: %f, y: %f, z: %f, w: %f}}, twist:{ linear: {x: %f, y: 0, z: 0}, angular:{x: %f, y: 0.0, z: 0}}, reference_frame: world }}'"

        if name == 'chaser':
            pos_x = pose.position.x + 0.65 
            angular_vel = 0.0
        else:
            pos_x = pose.position.x

        pose_command = ''
        pose_command = command_head % (
                            name,
                            pos_x,
                            pose.position.y,
                            pose.position.z,
                            pose.orientation.x,
                            pose.orientation.y,
                            pose.orientation.z,
                            pose.orientation.w,
                            linear_vel,
                            angular_vel,
                        )

        return pose_command

    def set_chaser_initial_pose(self):
        """ set chaser initial pose """
        target_vel = [0.0, -0.5, 0.0, 0.0, 0.5, 0.0]

        r = rospy.Rate(10) # 10Hz
        state = 1
        cnt = 0

        initial_chaser_pose = self.chaser_pose
        initial_debris_pose = self.debris_pose

        while cnt < 100:
            self.cmd_msg1.data = [2.0, target_vel[0], target_vel[1]]
            self.cmd_msg2.data = [2.0, target_vel[2], target_vel[3]]
            self.cmd_msg3.data = [2.0, target_vel[4], target_vel[5]]
            self.cmd_msg4.data = [1.0, 0.0, -0.1]
            self.cmd_msg5.data = [1.0, -0.1, -0.1]
                
            self.pub_arm1.publish(self.cmd_msg1)
            self.pub_arm2.publish(self.cmd_msg2)
            self.pub_arm3.publish(self.cmd_msg3)
            self.pub_arm4.publish(self.cmd_msg4)
            self.pub_arm5.publish(self.cmd_msg5)

            cnt += 1
            r.sleep()

        self.cmd_msg1.data = [2.0, 0.0, 0.0]
        self.cmd_msg2.data = [2.0, 0.0, 0.0]
        self.cmd_msg3.data = [2.0, 0.0, 0.0]

        self.pub_arm1.publish(self.cmd_msg1)
        self.pub_arm2.publish(self.cmd_msg2)
        self.pub_arm3.publish(self.cmd_msg3)


    def run(self):
        """ run ros node """
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_state)
        rospy.Subscriber("/chaser/obs_arm1", JointState, self.arm_state1)
        rospy.Subscriber("/chaser/obs_arm2", JointState, self.arm_state2)
        rospy.Subscriber("/chaser/obs_arm3", JointState, self.arm_state3)

        # initialize ros node
        rospy.init_node('getit_simulation_demo')

        r = rospy.Rate(10) # 10Hz
        
        # set chaser initial pose
        self.set_chaser_initial_pose()

        # start simulation
        rospy.sleep(5)

        # set velocity
        com = self.create_command(self.chaser_name, self.chaser_pose, True)
        os.system(com)
        com = self.create_command(self.debris_name, self.debris_pose, True)
        os.system(com)

        target_reached = False
        target_vel = [0.0, 0.5, 0.0, 0.0, -0.6, 0.0]

        while not rospy.is_shutdown():
            # reach hand and get debri
            self.cmd_msg1.data = [2.0, target_vel[0], target_vel[1]]
            self.cmd_msg2.data = [2.0, target_vel[2], target_vel[3]]
            self.cmd_msg3.data = [2.0, target_vel[4], target_vel[5]]

            if self.arm_joint_2_position > -0.1 and self.arm_joint_2_position < 0.1:
                target_vel[1] = 0.0
            if self.arm_joint_5_position > -0.1 and self.arm_joint_5_position < 0.1:
                target_vel[4] = 0.0
        
            if target_vel[1] == 0.0 and target_vel[5] == 0.0:
                target_reached = True

            if target_reached:
                self.cmd_msg1.data = [0.0, 0.0, 0.0]
                self.cmd_msg2.data = [0.0, 0.0, 0.0]
                self.cmd_msg3.data = [0.0, 0.0, 0.0]

            self.pub_arm1.publish(self.cmd_msg1)
            self.pub_arm2.publish(self.cmd_msg2)
            self.pub_arm3.publish(self.cmd_msg3)

            if target_reached:
                # open end effectors
                self.cmd_msg4.data = [1.0, 0.0, 0.3]
                self.cmd_msg5.data = [1.0, 0.3, 0.3]
                self.pub_arm4.publish(self.cmd_msg4)
                self.pub_arm5.publish(self.cmd_msg5)

                # set chaser base velocity to stop
                rospy.sleep(1.0)
      
                self.cmd_msg_vel.linear.x = 0.0
                self.cmd_msg_vel.linear.y = 0.0
                self.cmd_msg_vel.linear.z = 0.0
                self.cmd_msg_vel.angular.x = 0.0
                self.cmd_msg_vel.angular.y = 0.0
                self.cmd_msg_vel.angular.z = 0.0
                self.pub_chaser_vel.publish(self.cmd_msg_vel)

            else:
                self.cmd_msg4.data = [1.0, 0.0, -0.1]
                self.cmd_msg5.data = [1.0, -0.1, -0.1]
                
                self.pub_arm4.publish(self.cmd_msg4)
                self.pub_arm5.publish(self.cmd_msg5)

            r.sleep()

if __name__ == '__main__':
    try:
        node = Demo()
        node.run()
    except:
        pass
