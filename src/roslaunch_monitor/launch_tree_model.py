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
import threading

from python_qt_binding.QtCore import Qt, Signal, QSize
from python_qt_binding.QtGui import QStandardItem, QIcon, QColor

from rqt_py_common.message_tree_model import MessageTreeModel
from rqt_py_common.data_items import ReadonlyItem, CheckableItem

class LaunchTreeModel(MessageTreeModel):
    _column_names = ['package', 'file', 'CPU(%)', 'RAM(MB)', 'restarts']
    item_value_changed = Signal(int, str, str, str, object)

    def __init__(self, parent=None):
        super(LaunchTreeModel, self).__init__(parent)
        self._column_index = {}
        for column_name in self._column_names:
            self._column_index[column_name] = len(self._column_index)
        self.clear()

        self._item_change_lock = threading.Lock()
        self.itemChanged.connect(self.handle_item_changed)

    def clear(self):
        super(LaunchTreeModel, self).clear()
        self.setHorizontalHeaderLabels(self._column_names)

    def get_launch_ids(self, index_list):
        return [item._user_data['launch_id'] for item in self._get_toplevel_items(index_list)]

    def remove_items_with_parents(self, index_list):
        for item in self._get_toplevel_items(index_list):
            self.removeRow(item.row())

    def handle_item_changed(self, item):
        if not self._item_change_lock.acquire(False):
            #qDebug('PublisherTreeModel.handle_item_changed(): could not acquire lock')
            return
        # lock has been acquired
        package_name = item._path
        column_name = self._column_names[item.column()]
        if item.isCheckable():
            new_value = str(item.checkState() == Qt.Checked)
            if new_value == "True":
                icon = QIcon.fromTheme('media-playback-start')
            else:
                icon = QIcon.fromTheme('media-playback-stop')
            item.setIcon(icon)
        else:
            new_value = item.text().strip()
        #print 'PublisherTreeModel.handle_item_changed(): %s, %s, %s' % (topic_name, column_name, new_value)

        self.item_value_changed.emit(item._user_data['launch_id'], package_name, column_name, new_value, item.setText)

        # release lock
        self._item_change_lock.release()

    def remove_launch(self, launch_id):
        for top_level_row_number in range(self.rowCount()):
            item = self.item(top_level_row_number)
            if item is not None and item._user_data['launch_id'] == launch_id:
                self.removeRow(top_level_row_number)
                return top_level_row_number
        return None

    def update_launch(self, launch_info):
        top_level_row_number = self.remove_launch(launch_info['launch_id'])
        self.add_launch(launch_info, top_level_row_number)

    def feedback_received(self, launch_info):
        package_name = launch_info['package_name']
        launch_id = launch_info['launch_id']
        user_data = {'launch_id': launch_info['launch_id']}

        for top_level_row_number in range(self.rowCount()):
            kwargs = {
                'user_data': user_data,
                'top_level_row_number': top_level_row_number,
            }
            parent_item = self.item(top_level_row_number)
            if parent_item is not None and parent_item.isCheckable() and parent_item._user_data['launch_id'] == launch_id:
                #if not self._item_change_lock.acquire(False):
                #    #qDebug('PublisherTreeModel.handle_item_changed(): could not acquire lock')
                #    print "Could not get item lock..."
                #    return
                if parent_item.hasChildren():
                    parent_item.removeRows(0, parent_item.rowCount())
                for node, cpu, ram, restarts in zip(launch_info['nodes'], launch_info['cpu'],
                                                    launch_info['ram'], launch_info['restarts']):
                    row = []
                    for item in self._get_data_items_for_path(package_name, node, cpu, ram, restarts, **kwargs):
                        item._path = package_name
                        item._user_data = kwargs.get('user_data', None)
                        #item.setFlags(item.flags() & ~Qt.ItemIsEnabled)
                        item.setEnabled(False)
                        row.append(item)
                    parent_item.appendRow(row)
                #self._item_change_lock.release()

    def add_launch(self, launch_info, top_level_row_number=None):
        # recursively create widget items for the message's slots
        parent = self
        package_name = launch_info['package_name']
        file_name = launch_info['file_name']
        user_data = {'launch_id': launch_info['launch_id']}
        kwargs = {
            'user_data': user_data,
            'top_level_row_number': top_level_row_number,
        }
        parent_row = []
        for item in self._get_data_items_for_path(package_name, file_name, "", "", "", **kwargs):
            item._path = package_name
            item._user_data = kwargs.get('user_data', None)
            parent_row.append(item)

        if kwargs.get('top_level_row_number', None) is not None:
            parent.insertRow(kwargs['top_level_row_number'], parent_row)
        else:
            parent.appendRow(parent_row)

        for node, cpu, ram, restarts in zip(launch_info['nodes'], launch_info['cpu'],
                                            launch_info['ram'], launch_info['restarts']):
            row = []
            for item in self._get_data_items_for_path(package_name, node, cpu, ram, restarts, **kwargs):
                item._path = package_name
                item._user_data = kwargs.get('user_data', None)
                row.append(item)
            parent_row[0].appendRow(row)

        # fill tree widget columns of top level item
        if launch_info['enabled']:
            parent_row[self._column_index['package']].setCheckState(Qt.Checked)
        else:
            pass
        parent_row[0].emitDataChanged()

    def _get_data_items_for_path(self, package_name, file_name, cpu, ram, restarts, **kwargs):
        file_item = ReadonlyItem(file_name)
        if len(file_name) > 0 and file_name[0] == '*':
            package_item = ReadonlyItem(package_name)
            package_item.setData(QColor(255,0,0), Qt.ForegroundRole)
            file_item.setData(QColor(255,0,0), Qt.ForegroundRole)
        elif len(cpu) == 0: # NOTE: this will not work for dead nodes
            package_item = CheckableItem(package_name)
            icon = QIcon.fromTheme('media-playback-stop')
            package_item.setIcon(icon)
        else:
            package_item = ReadonlyItem(package_name)

        return (package_item, file_item, ReadonlyItem(cpu), ReadonlyItem(ram), ReadonlyItem(restarts))

