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

import rospy
from hri_msgs.msg import ExpressionAction, ExpressionResult
from hri_framework import IExpressionActionServer
from hri_framework import MultiGoalActionServer
from hri_common import Expression
import threading
from ros_pololu_servo.msg import MotorCommand
from ros_pololu_servo.srv import MotorRange
import time


def clamp(self, value, min_value, max_value):
    return max(min(value, max_value), min_value)

def interpolate(value, old_min, old_max, new_min, new_max):
    # Width of each range
    old_range = old_max - old_min
    new_range = new_max - new_min

    # Scale old value into range between 0 and 1
    scaled_value = (value - old_min) / old_range

    # Convert the scaled value into the new range
    new_val = new_min + (scaled_value * new_range)

    return new_val


class MiniHeadExpressionServer(IExpressionActionServer):

    def __init__(self):
        self.action_server = MultiGoalActionServer('expression', ExpressionAction, auto_start=False)
        self.action_server.register_goal_callback(self.goal_callback)
        self.motor_pub = rospy.Publisher('pololu/command', MotorCommand, queue_size=10)
        self.motor_range_srv = rospy.ServiceProxy('pololu/motor_range', MotorRange)
        self.rate = rospy.Rate(10)

    def start(self):
        self.action_server.start()

    def goal_callback(self, goal_handle):
        #self.action_server.set_accepted(goal_handle)
        new_goal = goal_handle.get_goal()
        rospy.loginfo('Accepted new goal: {0}'.format(new_goal))

        found = False
        for name, member in Expression.__members__.items():
            if name == new_goal.expression:
                found = True

        if not found:
            rospy.logerr('{0} is not a valid expression. Valid expressions are: {1}'.format(new_goal.expression, Expression))
            self.action_server.set_aborted(goal_handle)
            return

        expression_thread = threading.Thread(target=self.start_expression, args=[goal_handle])
        expression_thread.run()

    def start_expression(self, goal_handle):
        goal = goal_handle.get_goal()

        expression = Expression[goal.expression]
        speed = goal.speed
        intensity = goal.intensity
        duration = goal.duration

        # Checking for default values (-1 means nothing was specified, hence we should use the default values)
        if speed == -1:
            speed = expression.default_speed

        if intensity == -1:
            intensity = expression.default_intensity

        if duration == -1 and expression.default_duration is not None:
            duration = expression.default_duration

        if expression in [Expression.smile, Expression.frown_mouth]:
            joint_name = 'smile_joint'
        elif expression in [Expression.open_mouth]:
            joint_name = 'jaw_joint'
        elif expression in [Expression.frown]:
            joint_name = 'brow_joint'

        negative = True
        if expression in [Expression.smile, Expression.open_mouth]:
            negative = False

        response = self.motor_range_srv(joint_name)

        msg = MotorCommand()
        msg.joint_name = joint_name

        min_range = min([response.min, response.max])
        max_range = max([response.min, response.max])

        if negative:
            msg.position = interpolate(intensity, 0.0, 1.0, 0.0, min_range)
        else:
            msg.position = interpolate(intensity, 0.0, 1.0, 0.0, max_range)

        msg.speed = interpolate(speed, 0.0, 1.0, 0.0, 0.5)
        msg.acceleration = interpolate(speed, 0.0, 1.0, 0.0, 0.4)

        start = time.time()
        while not rospy.is_shutdown() and not self.action_server.is_preempt_requested(goal_handle) and (time.time() - start) < duration:
            if duration >= 0.0 and (time.time() - start) < duration:
                break

            self.motor_pub.publish(msg)
            self.rate.sleep()

        if not self.action_server.is_preempt_requested(goal_handle):
            self.action_server.set_succeeded(goal_handle)

    def stop_expression(self, goal_handle):
        raise NotImplementedError('Please implement this method')

if __name__ == "__main__":
    rospy.init_node('expression_server')
    server = MiniHeadExpressionServer()
    server.start()
    rospy.spin()