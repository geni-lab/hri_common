#!/usr/bin/env python
# Copyright (c) 2014, OpenCog Foundation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the OpenCog Foundation nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

__author__ = 'Jamie Diprose'


import rospy
from ros_blender_bridge import Joint
from std_msgs.msg import Float64
from dynamixel_controllers.srv import SetSpeed
from ros_pololu_servo.msg import MotorCommand


class PololuJoint(Joint):

    def __init__(self, joint_name):
        Joint.__init__(self, joint_name)
        self.msg = MotorCommand()
        self.msg.joint_name = self.joint_name
        self.motor_pub = rospy.Publisher('pololu/command', MotorCommand, queue_size=10)

    def set_position(self, position):
        self.msg.position = position
        self.msg.acceleration = self.acceleration
        self.msg.speed = self.speed
        self.motor_pub.publish(self.msg)

    def set_speed(self, speed):
        self.speed = speed

    def set_acceleration(self, acceleration):
        self.acceleration = acceleration


class DynamixelJoint(Joint):

    def __init__(self, joint_name):
        Joint.__init__(self, joint_name)
        self.motor_pub = rospy.Publisher(self.joint_name + '/command', Float64, queue_size=10)
        self.set_speed_srv = rospy.ServiceProxy(self.joint_name + '/set_speed', SetSpeed)

    def set_position(self, position):
        self.motor_pub.publish(position)

    def set_speed(self, speed):
        self.set_speed_srv(speed)

    def set_acceleration(self, acceleration):
        pass