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
from hri_msgs.msg import GazeAction, GazeActionFeedback
from hri_framework import IGazeActionServer
from geometry_msgs.msg import PointStamped, Point
from std_srvs.srv import Empty
import tf
from math import sqrt
from ros_blender_bridge.srv import SetSpeed, SetAcceleration
from hri_framework.entity_proxy import EntityProxy
from std_msgs.msg import Bool



def interpolate(value, old_min, old_max, new_min, new_max):
    # Width of each range
    old_range = old_max - old_min
    new_range = new_max - new_min

    # Scale old value into range between 0 and 1
    scaled_value = (value - old_min) / old_range

    # Convert the scaled value into the new range
    new_val = new_min + (scaled_value * new_range)

    return new_val


class BlenderGazeServer(IGazeActionServer):

    GAZE_REACHED = 0.1

    def __init__(self, controller_name, target_name):
        super(BlenderGazeServer, self).__init__()

        self.tl = tf.TransformListener()
        self.controller_name = controller_name
        self.target_name = target_name
        self.target_reached = False

        self.gaze_target_pub = rospy.Publisher(self.controller_name + '/' + self.target_name, PointStamped, queue_size=1)
        self.enable_srv = rospy.ServiceProxy(self.controller_name + '/enable', Empty)
        self.disable_srv = rospy.ServiceProxy(self.controller_name + '/disable', Empty)
        self.speed_srv = rospy.ServiceProxy(self.controller_name + '/set_speed', SetSpeed)
        self.accel_srv = rospy.ServiceProxy(self.controller_name + '/set_acceleration', SetAcceleration)

        rospy.Subscriber(self.controller_name + '/target_reached', Bool, self.target_reached_callback)

    def target_reached_callback(self, msg):
        self.target_reached = msg.data

    def execute(self, gaze_goal):
        self.enable_srv()
        self.speed_srv(gaze_goal.speed)
        self.speed_srv(interpolate(gaze_goal.speed, 0.0, 1.0, 0.0, 0.3))
        self.target_reached = False

        while not rospy.is_shutdown() and not self.action_server.is_preempt_requested() and self.action_server.is_active():
            entity = EntityProxy(gaze_goal.target)
            entity_tf_frame = entity.tf_frame()

            try:
                (target_trans, target_rot) = self.tl.lookupTransform(self.origin_frame, entity_tf_frame, rospy.Time(0))
                point_stamped = PointStamped()
                point_stamped.header.frame_id = self.origin_frame
                point_stamped.header.stamp = rospy.Time().now()
                point_stamped.point = Point(x=target_trans[0], y=target_trans[1], z=target_trans[2])
                self.gaze_target_pub.publish(point_stamped)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            try:
                (curr_trans, curr_rot) = self.tl.lookupTransform(self.gaze_frame, entity_tf_frame, rospy.Time(0))
                y = curr_trans[1]
                z = curr_trans[2]
                distance_to_target = sqrt(y*y + z*z)
                rospy.loginfo('gaze_frame: {0}, entity_tf_frame: {1}, y: {2}, z: {3}, distance: {4}'.format(self.gaze_frame, entity_tf_frame, y, z, distance_to_target))
                self.send_feedback(distance_to_target)

                if distance_to_target < BlenderGazeServer.GAZE_REACHED: #or self.target_reached: #TODO: check min max motor positions because person could be out of bounds of gaze
                    self.action_server.set_succeeded()

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            self.rate.sleep()

        self.disable_srv()
