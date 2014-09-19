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
from std_msgs.msg import Float32
from ros_pololu_servo.srv import MotorRange
from ros_pololu_servo.msg import MotorCommand


def interpolate(value, old_min, old_max, new_min, new_max):
    # Width of each range
    old_range = old_max - old_min
    new_range = new_max - new_min

    # Scale old value into range between 0 and 1
    scaled_value = (value - old_min) / old_range

    # Convert the scaled value into the new range
    new_val = new_min + (scaled_value * new_range)

    return new_val


class LipSync():

    def __init__(self):
        self.motor_pub = rospy.Publisher('pololu/command', MotorCommand, queue_size=1)
        self.motor_range_srv = rospy.ServiceProxy('pololu/motor_range', MotorRange)

        # Setup motor command
        self.cmd = MotorCommand()
        self.cmd.joint_name = 'jaw_joint'
        self.cmd.speed = 1.0
        self.cmd.acceleration = 1.0

        # Add some comment here.

        # Get motor range
        rospy.loginfo('Waiting for pololu/motor_range service...')
        self.motor_range_srv.wait_for_service()
        rospy.loginfo('Found')

        response = self.motor_range_srv(self.cmd.joint_name)
        self.max_range = max([response.min, response.max])
        self.sub = rospy.Subscriber('speech_strength', Float32, self.speech_strength_callback, queue_size=1)

    def speech_strength_callback(self, msg):
        strength = msg.data
        self.cmd.position = interpolate(strength, 0.0, 1.0, 0.0, self.max_range)
        self.motor_pub.publish(self.cmd)

