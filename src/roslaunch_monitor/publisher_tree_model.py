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

from python_qt_binding.QtCore import Qt, Signal
from python_qt_binding.QtGui import QStandardItem

from rqt_py_common.message_tree_model import MessageTreeModel
from rqt_py_common.data_items import ReadonlyItem, CheckableItem

class PublisherTreeModel(MessageTreeModel):
    _column_names = ['package', 'file', 'CPU(%)', 'RAM(MB)', 'restarts']
    item_value_changed = Signal(int, str, str, str, object)

    def __init__(self, parent=None):
        super(PublisherTreeModel, self).__init__(parent)
        self._column_index = {}
        for column_name in self._column_names:
            self._column_index[column_name] = len(self._column_index)
        self.clear()

        self._item_change_lock = threading.Lock()
        self.itemChanged.connect(self.handle_item_changed)

    def clear(self):
        super(PublisherTreeModel, self).clear()
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
        
        #top_level_row_number = self.remove_launch(launch_info['launch_id'])
        #self.add_launch(launch_info, top_level_row_number)
        #return
        
        package_name = launch_info['package_name']
        launch_id = launch_info['launch_id']
        user_data = {'launch_id': launch_info['launch_id']}

        for top_level_row_number in range(self.rowCount()):
            kwargs = {
                'user_data': user_data,
                'top_level_row_number': top_level_row_number,
            }
            item = self.item(top_level_row_number)
            if item is not None and item.isCheckable() and item._user_data['launch_id'] == launch_id:
                if not self._item_change_lock.acquire(False):
                    #qDebug('PublisherTreeModel.handle_item_changed(): could not acquire lock')
                    return
                count = 0
                for node, cpu, ram, restarts in zip(launch_info['nodes'], launch_info['cpu'],
                                                    launch_info['ram'], launch_info['restarts']):
                    print "Adding node: ", node
                    row = []
                    for item in self._get_data_items_for_path(package_name, node, cpu, ram, restarts, **kwargs):
                        item._path = package_name
                        item._user_data = kwargs.get('user_data', None)
                        row.append(item)
                    item.appendRow(row)
                    #self.item_value_changed.emit(item._user_data['launch_id'], package_name, "package_name", row, item.appendRow)
                    #item.emitDataChanged()
                    count += 1
                
                item.setRowCount(item.rowCount() + count)
                self.itemChanged.emit(item)

                # release lock
                self._item_change_lock.release()
                #for item2 in row:
                #    self.itemChanged.emit(item2)

                #item.setChild(row)
                #item.insertRow(count, row)
                #self.insertRow(top_level_row_number, row)

                print dir(item)
                #item.setSelectable(True)
                #self.itemChanged.emit(item)
                #item.emitDataChanged()
                #self.layoutChanged.emit()
                #self.setRowCount(10)
                #self.childEvent()
                print dir(self)
                #self.expand(top_level_row_number)
                print item.hasChildren()
                print item.rowCount()
                #item.itemChanged(item)
                #item.itemChanged.emit()
                #self.itemChanged.emit(item)
                #self.cellChanged.emit(top_level_row_number, 0)
                #self.expand(top_level_row_number)

            #self.handle_item_changed(item)

    def add_launch(self, launch_info, top_level_row_number=None):
        # recursively create widget items for the message's slots
        parent = self
        #slot = publisher_info['message_instance']
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

        #parent.insertRow(kwargs['top_level_row_number'], row)
        if kwargs.get('top_level_row_number', None) is not None:
            parent.insertRow(kwargs['top_level_row_number'], parent_row)
        else:
            parent.appendRow(parent_row)
        #parent.appendRow(row)

        for node, cpu, ram, restarts in zip(launch_info['nodes'], launch_info['cpu'],
                                            launch_info['ram'], launch_info['restarts']):
            row = []
            for item in self._get_data_items_for_path(package_name, node, cpu, ram, restarts, **kwargs):
                item._path = package_name
                item._user_data = kwargs.get('user_data', None)
                row.append(item)
            parent_row[0].appendRow(row)

        print parent_row[0].hasChildren()
        print parent_row[0].rowCount()

        # fill tree widget columns of top level item
        if launch_info['enabled']:
            parent_row[self._column_index['package']].setCheckState(Qt.Checked)
            #publisher_info['timer'].stop()

        #top_level_row[self._column_index['rate']].setText(str(publisher_info['rate']))

    def _get_data_items_for_path(self, package_name, file_name, cpu, ram, restarts, **kwargs):
        if len(cpu) == 0: # NOTE: this will not work for dead nodes
            package_item = CheckableItem(package_name)
        else:
            package_item = ReadonlyItem(package_name)

        return (package_item, ReadonlyItem(file_name), ReadonlyItem(cpu), ReadonlyItem(ram), ReadonlyItem(restarts))

