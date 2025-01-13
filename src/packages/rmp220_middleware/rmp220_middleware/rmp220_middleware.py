#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from enum import Enum
from segway_msgs.srv import RosSetChassisEnableCmd
from segway_msgs.msg import ChassisModeFb

import atexit
import signal
import sys

class State(Enum):  # Best practice for enum usage in both ROS1 and ROS2
    DISABLED = 0  # solid yellow
    ENABLED = 1   # solid green
    PASSIVE = 2   # solid white (push)
    STOPPED = 3   # solid red
    PAUSED = 4    # no extra visual feedback, solid yellow

class StateMachineNode:
    def __init__(self):
        rospy.init_node('state_machine_node', anonymous=True)

        # Initialize state and other variables
        self.state = State.DISABLED
        self.timeout = 20.0  # Timeout in seconds

        # Create twist class for publishing velocities
        self.twist = Twist()

        self.latest_cmd_vel = Twist()
        self.abs_x = 0.0
        self.abs_z = 0.0

        # Create publishers, subscribers, timers, and service clients
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_out', Twist, queue_size=10)
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel_mux', Twist, self.cmd_vel_callback)
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.timer = rospy.Timer(rospy.Duration(0.01), self.timer_callback)  # 100 Hz

        # Create service clients for chassis enable and disable
        rospy.wait_for_service('set_chassis_enable')
        self.chassis_enable_client = rospy.ServiceProxy('set_chassis_enable', RosSetChassisEnableCmd)

        # Create a subscriber for the chassis status topic
        self.chassis_mode_sub = rospy.Subscriber('/chassis_mode_fb', ChassisModeFb, self.chassis_mode_callback)

    def chassis_mode_callback(self, msg):
        if self.state == State.PAUSED:  # Save processing time by skipping if PAUSED
            return
        else:
            if msg.chassis_mode == 0:
                self.state = State.DISABLED
                rospy.loginfo('Set chassis_mode to %d' % self.state.value)
            if msg.chassis_mode == 1:
                self.state = State.ENABLED
                rospy.loginfo('Set chassis_mode to %d' % self.state.value)
            if msg.chassis_mode == 2:
                self.state = State.PASSIVE
                rospy.loginfo('Set chassis_mode to %d' % self.state.value)
            if msg.chassis_mode == 3:
                self.state = State.STOPPED
                rospy.loginfo('Set chassis_mode to %d' % self.state.value)

    def enable_chassis(self):
        req = RosSetChassisEnableCmd.Request()
        req.ros_set_chassis_enable_cmd = True
        self.chassis_enable_client.call(req)
        self.state = State.ENABLED
        rospy.loginfo('Enabling chassis...')

    def pause_chassis(self):
        req = RosSetChassisEnableCmd.Request()
        req.ros_set_chassis_enable_cmd = False
        self.chassis_enable_client.call(req)
        self.state = State.PAUSED
        rospy.loginfo('Pausing chassis...')

    def disable_chassis(self):
        req = RosSetChassisEnableCmd.Request()
        req.ros_set_chassis_enable_cmd = False
        self.chassis_enable_client.call(req)
        self.state = State.DISABLED
        rospy.loginfo('Disabling chassis...')

    def joy_callback(self, msg):
        start_button = msg.buttons[7]  # Joystick button 'start'
        select_button = msg.buttons[6]  # Joystick button 'select'

        if start_button == 1:
            rospy.loginfo("State: ENABLED (Button 'start')")
            self.enable_chassis()
            self.timeout = 20
        if select_button == 1:
            rospy.loginfo("State: DISABLED (Button 'select')")
            self.pause_chassis()

    def cmd_vel_callback(self, msg):
        # Update the latest cmd_vel and compute absolute values for linear and angular velocity
        self.latest_cmd_vel = msg
        self.abs_x = abs(msg.linear.x)
        self.abs_z = abs(msg.angular.z)

        # Reset timeout when receiving commands
        self.timeout = 20.0

    def timer_callback(self, event):
        if self.state == State.PAUSED or self.state == State.STOPPED or self.state == State.PASSIVE:
            return  # Do nothing if chassis is disabled, stopped, or passive
        if self.state == State.ENABLED:
            if self.timeout <= 0:
                self.state = State.DISABLED
                rospy.loginfo("State: DISABLED (Timeout)")
                self.disable_chassis()
            else:
                self.timeout -= 0.01  # at 100 Hz this equals to -1 per second
                self.cmd_vel_pub.publish(self.latest_cmd_vel)
        if self.state == State.DISABLED and (self.abs_x > 0.002 or self.abs_z > 0.002):
            self.state = State.ENABLED
            rospy.loginfo("State: ENABLED (cmd_vel)")
            self.enable_chassis()

if __name__ == '__main__':
    try:
        node = StateMachineNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        node.disable_chassis()
