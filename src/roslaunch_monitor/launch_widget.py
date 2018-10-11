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
import os

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Signal, Slot
from python_qt_binding.QtGui import QIcon
from python_qt_binding.QtWidgets import QWidget

import roslib
import rosmsg
import rospkg
import rospy

from qt_gui_py_common.worker_thread import WorkerThread
from rqt_py_common.extended_combo_box import ExtendedComboBox
from rqt_py_common.rqt_roscomm_util import RqtRoscommUtil
from roslaunch_monitor.launch_tree_widget import LaunchTreeWidget


# main class inherits from the ui window class
class LaunchWidget(QWidget):
    add_launch = Signal(str, str, str, bool)
    change_launch = Signal(int, str, str, str, object)
    #publish_once = Signal(int)
    remove_launch = Signal(int)
    clean_up_launches = Signal()

    def __init__(self, parent=None):
        super(LaunchWidget, self).__init__(parent)
        self._topic_dict = {}
        self._update_thread = WorkerThread(self._update_thread_run, self._update_finished)

        self._rospack = rospkg.RosPack()
        ui_file = os.path.join(self._rospack.get_path('roslaunch_monitor'), 'resource', 'rqt_monitor_plugin.ui')
        loadUi(ui_file, self, {'ExtendedComboBox': ExtendedComboBox, 'LaunchTreeWidget': LaunchTreeWidget})
        self.refresh_button.setIcon(QIcon.fromTheme('view-refresh'))
        self.refresh_button.clicked.connect(self.refresh_combo_boxes)
        self.add_launch_button.setIcon(QIcon.fromTheme('add'))
        self.remove_launch_button.setIcon(QIcon.fromTheme('remove'))
        self.clear_button.setIcon(QIcon.fromTheme('edit-clear'))

        self.refresh_combo_boxes()

        self.launch_tree_widget.model().item_value_changed.connect(self.change_launch)
        self.launch_tree_widget.remove_publisher.connect(self.remove_launch)
        #self.publisher_tree_widget.publish_once.connect(self.publish_once)
        self.remove_launch_button.clicked.connect(self.launch_tree_widget.remove_selected_publishers)
        self.clear_button.clicked.connect(self.clean_up_launches)

    def shutdown_plugin(self):
        self._update_thread.kill()

    @Slot()
    def refresh_combo_boxes(self):
        self._update_thread.kill()
        self.file_combo_box.setEnabled(False)
        self.package_combo_box.setEnabled(False)
        self.file_combo_box.setEditText('updating...')
        self.package_combo_box.setEditText('updating...')
        self._update_thread.start()

    # this runs in a non-gui thread, so don't access widgets here directly
    def _update_thread_run(self):
        # update type_combo_box
        message_type_names = []
        try:
            # this only works on fuerte and up
            packages = sorted([pkg_tuple[0] for pkg_tuple in rosmsg.iterate_packages(self._rospack, rosmsg.MODE_MSG)])
        except:
            # this works up to electric
            packages = sorted(rosmsg.list_packages())
        for package in packages:
            for base_type_str in rosmsg.list_msgs(package, rospack=self._rospack):
                message_class = roslib.message.get_message_class(base_type_str)
                if message_class is not None:
                    message_type_names.append(base_type_str)

        # TODO: get all ROS packages and launch files here instead
        #self.type_combo_box.setItems.emit(sorted(message_type_names))
        packages = sorted([pkg_tuple[0]
                           for pkg_tuple
                           in RqtRoscommUtil.iterate_packages('launch')])
        self.package_combo_box.setItems.emit(packages)

        # update topic_combo_box
        #_, _, topic_types = rospy.get_master().getTopicTypes()
        #self._topic_dict = dict(topic_types)
        #self.topic_combo_box.setItems.emit(sorted(self._topic_dict.keys()))

    @Slot()
    def _update_finished(self):
        self.package_combo_box.setEnabled(True)
        self.file_combo_box.setEnabled(True)

    @Slot(str)
    def on_package_combo_box_currentIndexChanged(self, package):
        #if topic_name in self._topic_dict:
        #    self.type_combo_box.setEditText(self._topic_dict[topic_name])
        #pass
        _launch_instance_list = RqtRoscommUtil.list_files(package, 'launch')
        _launchfile_instances = [x.split('/')[1] for x in _launch_instance_list]
        self.file_combo_box.setItems.emit(_launchfile_instances)
        #self.file_combo_box.clear()
        #self.file_combo_box.addItems(_launchfile_instances)

    @Slot()
    def on_add_launch_button_clicked(self):
        package_name = str(self.package_combo_box.currentText())
        file_name = str(self.file_combo_box.currentText())
        arguments = str(self.arguments_combo_box.currentText())
        enabled = False
        self.add_launch.emit(package_name, file_name, arguments, enabled)
