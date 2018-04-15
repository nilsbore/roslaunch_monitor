#! /usr/bin/env python

import rospy
import sys
#from __future__ import print_function
from roslaunch_monitor.srv import NodeAction, NodeActionRequest
from roslaunch_monitor.srv import MonitorLaunch, MonitorLaunchResponse, MonitorLaunchRequest
from roslaunch_monitor.srv import CancelMonitorLaunch, CancelMonitorLaunchResponse, CancelMonitorLaunchRequest
from roslaunch_monitor.launch_server_client import LaunchServerClient

import npyscreen
import curses

# encoding: utf-8
npyscreen.DISABLE_RESIZE_SYSTEM = False
npyscreen.FormBaseNew.ALLOW_RESIZE = True
npyscreen.FormBaseNew.FIX_MINIMUM_SIZE_WHEN_CREATED = False # NOTE: this seems useless

class FeedbackEvent(npyscreen.Event):

    def __init__(self, name, msg):

        super(FeedbackEvent, self).__init__(name)
        self.msg = msg

class MonitorWidget(npyscreen.GridColTitles):

    def h_move_or_exit_down(self, _input):
        
        row, col = self.edit_cell
        if row >= len(self.values)-1:
            self.h_exit_down(_input)
        else:
            self.h_move_line_down(_input)
    
    def h_move_or_exit_up(self, _input):
        
        row, col = self.edit_cell
        if row <= 0:
            self.h_exit_up(_input)
        else:
            self.h_move_line_up(_input)

    def custom_print_cell(self, actual_cell, cell_display_value):

        if len(cell_display_value) > 0 and \
           (cell_display_value[0] == "*" or cell_display_value[0] == "-"):
            actual_cell.color = 'DANGER'

    # NOTE: I'm a *KING* among coders
    def calculate_area_needed(self):

        if hasattr(self, "values") and self.values is not None:
            return len(self.values)+3, 0
        else:
            return 6, 0

class MonitorCollection(object):

    def __init__(self, form, pkg, launch_file, nbr):

        self.monitor_server = LaunchServerClient()
        self.monitor_server.launch(pkg, launch_file)
        self.monitor_server.add_callback(self.feedback_cb)

        self.name = pkg+"/"+launch_file
        self.title = form.add(npyscreen.TitleText, name=self.name, editable=False)
        self.widget = form.add(MonitorWidget, name=self.name, relx=2, 
                               columns = 4, column_width = None, col_margin=0, row_height = 1,
                               col_titles = ["Node name", "CPU Percent", "RAM MB", "Nbr restarts"],
                               always_show_cursor = False, select_whole_line = True, scroll_exit=False)
        self.feedback_name = "FEEDBACK_"+str(nbr)
        form.parentApp.add_event_hander(self.feedback_name, self.ev_test_event_handler)
        self.widget.add_handlers({curses.ascii.NL:   self.handle_node,
                                  curses.ascii.CR:   self.widget.h_exit_down,
                                  curses.ascii.TAB:  self.widget.h_exit_down,
                                  curses.KEY_DOWN:   self.widget.h_move_or_exit_down,
                                  curses.KEY_UP:     self.widget.h_move_or_exit_up,
                                  #curses.KEY_LEFT:   self.widget.h_exit_left,
                                  #curses.KEY_RIGHT:  self.widget.h_exit_right,
                                  "^P":              self.widget.h_exit_up,
                                  "^N":              self.widget.h_exit_down,
                                  curses.ascii.ESC:  self.widget.h_exit_escape, # NOTE: this is good
                                  curses.KEY_MOUSE:  self.widget.h_exit_mouse})

    def handle_node(self, event):

        row, col = self.widget.edit_cell
        node_name = self.widget.values[row][0]
        if len(node_name) == 0 or node_name[0] == "*":
            return
        menu = self.widget.parent.new_menu(name=node_name)
        menu.addItem("Kill", onSelect=self.kill_node, arguments=(node_name,))
        menu.addItem("Restart", onSelect=self.restart_node, arguments=(node_name,))
        menu.addItem("Cancel")
        self.widget.parent.popup_menu(menu)

    def kill_node(self, node_name):

        self.monitor_server.node_action(node_name, NodeActionRequest.KILL)
        
    def restart_node(self, node_name):

        self.monitor_server.node_action(node_name, NodeActionRequest.RESTART)
    
    def feedback_cb(self, msg):    
        self.widget.parent.parentApp.queue_event(FeedbackEvent(self.feedback_name, msg))
    
    def ev_test_event_handler(self, event):
        self.widget.values = [list(v) for v in zip(event.msg.alive_nodes, event.msg.cpu_percent, event.msg.ram_mb, event.msg.nbr_restarts)]
        self.widget.values += [["* " + name, "-", "-", "-"] for name in event.msg.dead_nodes]
        self.widget.parent.display()

class MonitorEvent(npyscreen.Event):

    def __init__(self, name, pkg, launch_file):

        super(MonitorEvent, self).__init__(name)
        self.pkg = pkg
        self.launch_file = launch_file

class CancelMonitorEvent(npyscreen.Event):

    def __init__(self, name, launch_id):

        super(CancelMonitorEvent, self).__init__(name)
        self.launch_id = launch_id

class MonitorLaunchNode(object):

    def __init__(self, app):
        
        self.node_service = rospy.Service('~monitor_launch', MonitorLaunch, self.launch_cb)
        self.node_service = rospy.Service('~cancel_launch', CancelMonitorLaunch, self.cancel_cb)
        self._app = app
        rospy.Timer(rospy.Duration(2.), self.spin_cb)

    def spin_cb(self, event):

        for launch_id, m in self._app.monitors.items():
            if not m.monitor_server.spin():
                self._app.queue_event(CancelMonitorEvent("CANCELMONITOR", launch_id))

    def launch_cb(self, req):

        launch_id = self._app.nbr_monitors
        self._app.queue_event(MonitorEvent("ADDMONITOR", req.pkg, req.launch_file))

        return MonitorLaunchResponse(launch_id)
    
    def cancel_cb(self, req):

        self._app.queue_event(CancelMonitorEvent("CANCELMONITOR", req.launch_id))

        return CancelMonitorLaunchResponse()

class MonitorApp(npyscreen.StandardApp):

    def cancel_cb(self):

        curses.beep()
        self.switchForm(None)

    def add_monitor(self, event):
        
        form = self.getForm("MAIN")
        self.monitors[self.nbr_monitors] = MonitorCollection(form, event.pkg, event.launch_file, self.nbr_monitors)
        self.nbr_monitors += 1

    def cancel_monitor(self, event):

        if event.launch_id in self.monitors:
            monitor = self.monitors.pop(event.launch_id)
            monitor.monitor_server.cancel_cb()
            monitor.title.hidden = True
            monitor.widget.hidden = True
            monitor.title.display()
            monitor.widget.display()
            #self.getForm("MAIN").display()

    def onStart(self):
        self.addForm("MAIN", npyscreen.FormBaseNewWithMenus, name=rospy.get_name())
        form = self.getForm("MAIN")
        self.dummy = form.add(npyscreen.DummyWidget)
        self.add_event_hander("ADDMONITOR", self.add_monitor)
        self.add_event_hander("CANCELMONITOR", self.cancel_monitor)
        self.monitors = {}
        self.nbr_monitors = 0
        rospy.on_shutdown(self.cancel_cb)
        self.getForm("MAIN").how_exited_handers[npyscreen.wgwidget.EXITED_ESCAPE]  = self.cancel_cb
        self.node = MonitorLaunchNode(self)

if __name__ == '__main__':

    rospy.init_node('monitor_server')
    
    App = MonitorApp()
    if len(sys.argv) > 2:
        #App.queue_event(MonitorEvent("ADDMONITOR", sys.argv[1], sys.argv[2]))
        App.queue_event(MonitorEvent("ADDMONITOR", "rfs_slam", "test_sim.launch"))
        App.queue_event(MonitorEvent("ADDMONITOR", "rfs_slam", "slam.launch"))
        #App.queue_event(MonitorEvent("ADDMONITOR", "auv_sensors", "auv_sensors.launch"))
    App.run()

