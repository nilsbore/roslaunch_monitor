#!/usr/bin/env python

# Copyright (c) 2011, Dorian Scholz, TU Darmstadt
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the TU Darmstadt nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import division
import math
import random
import time
from functools import partial

from python_qt_binding.QtCore import Signal, Slot, QSignalMapper, QTimer, qWarning

import roslib
import rospy
import genpy
from rqt_gui_py.plugin import Plugin
from roslaunch_monitor.launch_widget import LaunchWidget
from rqt_py_common.topic_helpers import get_field_type

from roslaunch_monitor.srv import NodeAction, NodeActionRequest
from roslaunch_monitor.launch_server_client import LaunchServerClient

class RQTMonitorPlugin(Plugin):
    
    feedback_received_sig = Signal(int)

    def __init__(self, context):
        super(RQTMonitorPlugin, self).__init__(context)
        self.setObjectName('RQTMonitorPlugin')

        # create widget
        self._widget = LaunchWidget()
        self._widget.add_launch.connect(self.add_launch)
        self._widget.change_launch.connect(self.change_launch)
        #self._widget.publish_once.connect(self.publish_once)
        self._widget.remove_launch.connect(self.remove_launch)
        self._widget.clean_up_launches.connect(self.clean_up_launches)
        self.feedback_received_sig.connect(self.feedback_received)
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        #self._publishers = {}
        self._launches = {}
        self._id_counter = 0

        # add our self to the main window
        context.add_widget(self._widget)

    def feedback_cb(self, launch_id, msg):
        cpu_percent = ["{:.2f}".format(v) for v in msg.cpu_percent] + ["" for name in msg.dead_nodes]
        ram_mb = ["{:.2f}".format(v) for v in msg.ram_mb] + ["" for name in msg.dead_nodes]
        nbr_restarts = ["{:d}".format(v) for v in msg.nbr_restarts] + ["" for name in msg.dead_nodes]
        nodes = msg.alive_nodes + ["* " + name for name in msg.dead_nodes]
        
        self._launches[launch_id]['nodes'] = nodes
        self._launches[launch_id]['cpu'] = cpu_percent
        self._launches[launch_id]['ram'] = ram_mb
        self._launches[launch_id]['restarts'] = nbr_restarts

        rospy.loginfo(msg)
        self.feedback_received_sig.emit(launch_id)

    @Slot(int)
    def feedback_received(self, launch_id):
        #print "Dir:", dir(self.context)
        self._widget.launch_tree_widget.model().feedback_received(self._launches[launch_id])

    @Slot(int, str, str, str, object)
    def change_launch(self, launch_id, package_name, column_name, new_value, setter_callback):
        if column_name == "package":
            rospy.loginfo("Checkbox!")
            rospy.loginfo(new_value)
            if new_value == "True": # checked
                rospy.loginfo("Checked!")
                monitor_server = LaunchServerClient()
                monitor_server.launch(self._launches[launch_id]['package_name'],
                                      self._launches[launch_id]['file_name'],
                                      [], [])
                monitor_server.add_callback(partial(self.feedback_cb, launch_id))
                self._launches[launch_id]['launch_client'] = monitor_server
            elif self._launches[launch_id]['launch_client'] is not None:
                rospy.loginfo("Not Checked!")
                self._launches[launch_id]['launch_client'].cancel_cb()
                self._launches[launch_id]['launch_client'] = None
                self._launches[launch_id]['nodes'] = []
                self._launches[launch_id]['cpu'] = []
                self._launches[launch_id]['ram'] = []
                self._launches[launch_id]['restarts'] = []
                self.feedback_received(launch_id)
                #self.feedback_received_sig.emit(launch_id)

    @Slot(str, str, str, bool)
    def add_launch(self, package_name, file_name, arguments, enabled):
        launch_info = {
            'package_name': str(package_name),
            'file_name': str(file_name),
            'arguments': str(arguments),
            'enabled': bool(enabled),
        }
        self._add_launch(launch_info)

    def _add_launch(self, launch_info):
        launch_info['launch_id'] = self._id_counter
        launch_info['launch_client'] = None
        launch_info['nodes'] = []
        launch_info['cpu'] = []
        launch_info['ram'] = []
        launch_info['restarts'] = []
        self._id_counter += 1

        # create launch and timer
        #try:
        #    launch_info['publisher'] = rospy.Publisher(launch_info['topic_name'], type(publisher_info['message_instance']), queue_size=100)
        #except TypeError:
        #    pass

        # add launch info to _publishers dict and create signal mapping
        self._launches[launch_info['launch_id']] = launch_info
        #self._timeout_mapper.setMapping(launch_info['timer'], publisher_info['publisher_id'])

        self._widget.launch_tree_widget.model().add_launch(launch_info)

    #@Slot(int)
    #def publish_once(self, publisher_id):
    #   publisher_info = self._publishers.get(publisher_id, None)
    #    if publisher_info is not None:
    #        publisher_info['counter'] += 1
    #        self._fill_message_slots(publisher_info['message_instance'], publisher_info['topic_name'], publisher_info['expressions'], publisher_info['counter'])
    #        publisher_info['publisher'].publish(publisher_info['message_instance'])

    @Slot(int)
    def remove_launch(self, launch_id):
        launch_info = self._launches.get(launch_id, None)
        if launch_info is not None:
            #publisher_info['publisher'].unregister()
            del self._launches[launch_id]

    def save_settings(self, plugin_settings, instance_settings):
        launch_copies = []
        for launch in self._launches.values():
            launch_copy = {}
            launch_copy.update(launch)
            launch_copy['enabled'] = False
            del launch_copy['launch_client']
            launch_copy['nodes'] = []
            launch_copy['cpu'] = []
            launch_copy['ram'] = []
            launch_copy['restarts'] = []
            #del launch_copy['publisher']
            launch_copies.append(launch_copy)
        instance_settings.set_value('launches', repr(launch_copies))

    def restore_settings(self, plugin_settings, instance_settings):
        launches = eval(instance_settings.value('launches', '[]'))
        for launch in launches:
            self._add_launch(launch)

    def clean_up_launches(self):
        self._widget.launch_tree_widget.model().clear()
        for launch_info in self._launches.values():
            #publisher_info['publisher'].unregister()
            pass
        self._launches = {}

    def shutdown_plugin(self):
        self._widget.shutdown_plugin()
        self.clean_up_launches()
