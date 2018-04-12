#! /usr/bin/env python

import rospy
import sys
#from __future__ import print_function
import roslaunch_monitor.msg
from roslaunch_monitor.srv import NodeAction, NodeActionRequest

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import actionlib_tutorials.msg
import npyscreen
import curses

# encoding: utf-8
npyscreen.DISABLE_RESIZE_SYSTEM = False
npyscreen.FormBaseNew.ALLOW_RESIZE = True
npyscreen.FormBaseNew.FIX_MINIMUM_SIZE_WHEN_CREATED = False 

class MonitorServer(object):

    def __init__(self):

        self.alive_nodes = []
        self.cpu_percent = []
        self.ram_mb = []
        self._feedback_cb = None

    def feedback_cb(self, msg):
        #print msg
        self.alive_nodes = msg.alive_nodes
        self.cpu_percent = msg.cpu_percent
        self.ram_mb = msg.ram_mb
        if self._feedback_cb is not None:
            self._feedback_cb(msg)

    def node_action(self, node_name, action):

        req = NodeActionRequest()
        req.node_name = node_name
        req.action = action
        req.goal_id = self.client.gh.comm_state_machine.action_goal.goal_id.id #gh.get_goal_id()
        rospy.wait_for_service('/launch_monitor_server/node_action')
        try:
            service = rospy.ServiceProxy('/launch_monitor_server/node_action', NodeAction)
            resp = service(req)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def launch(self, pkg, launch_file):
        # Creates the SimpleActionClient, passing the type of the action
        # (FibonacciAction) to the constructor.
        self.client = actionlib.SimpleActionClient('/launch_monitor_server', roslaunch_monitor.msg.LaunchAction)

        # Waits until the action server has started up and started
        # listening for goals.
        self.client.wait_for_server()

        # Creates a goal to send to the action server.
        goal = roslaunch_monitor.msg.LaunchGoal(pkg=pkg, launch_file=launch_file)

        # Sends the goal to the action server.
        self.client.send_goal(goal, feedback_cb=self.feedback_cb)

        rospy.on_shutdown(self.cancel_cb)
        # Waits for the server to finish performing the action.
        #client.wait_for_result()

        # Prints out the result of executing the action
        #return client.get_result()  # A FibonacciResult

    def cancel_cb(self):

        if self.client is None:
            return

        rospy.loginfo("Early shutting down launch server...")
        self.client.cancel_goal()
        self.client.wait_for_result()
        rospy.loginfo("Finished shutting down launch server...")

    def add_callback(self, cb):

        self._feedback_cb = cb

    def spin(self):

        if not self.client.wait_for_server(timeout=rospy.Duration(0.5)) or \
               (self.client.get_state() != actionlib.SimpleGoalState.ACTIVE and \
                self.client.get_state() != actionlib.SimpleGoalState.PENDING):
            rospy.loginfo("Launch server was aborted, quitting...")
            self.client.stop_tracking_goal()
            self.client = None
            return False
        return True

#class TestForm(npyscreen.Form):
#    
#    def create(self):
#        self.s  = self.add(npyscreen.TitleSlider, out_of=12, name = "Slider")
#        self.ms2= self.add(npyscreen.TitleMultiSelect, max_height =4, value = [1,], name="Pick Several",
#                values = ["1","2","3"], scroll_exit=True)
#        self.ms3= self.add(npyscreen.TitleMultiSelect, max_height =4, value = [1,], name="Pick Several",
#                values = ["1", "2", "3"], scroll_exit=True)
#        self.mytext = self.add(npyscreen.TitleText, name = "Text:",)
#    #    self.add_event_hander("TESTEVENT", self.ev_test_event_handler)

class FeedbackEvent(npyscreen.Event):

    def __init__(self, name, msg):

        super(FeedbackEvent, self).__init__(name)
        self.msg = msg

class MonitorWidget(npyscreen.GridColTitles):

    #def when_value_edited(self):
    #    self.parent.display()

    #def when_cursor_moved(self):
    #    self.parent.display()

    #def when_parent_changes_value(self):
    #    self.display()

    def custom_print_cell(self, actual_cell, cell_display_value):

        pass
    
class MonitorCollection(object):

    #_is_init = False

    def __init__(self, form, pkg, launch_file, nbr):

        self.monitor_server = MonitorServer()
        self.monitor_server.launch(pkg, launch_file)
        self.monitor_server.add_callback(self.feedback_cb)

        self.name = pkg+"/"+launch_file
        #if not MonitorCollection._is_init:
        #    self.title = form.add(npyscreen.TitleText, name=self.name, rely=1+6*nbr)
        #    #MonitorCollection._is_init = True
        #else:
        self.title = form.add(npyscreen.TitleText, name=self.name, editable=False) #, rely=4+10*nbr)
        #self.title = form.add_widget_intelligent(npyscreen.TitleText, name=self.name) #, rely=4+10*nbr)
        #self.widget = form.add_widget_intelligent(MonitorWidget, name=self.name, #rely=2+6*nbr, 
        self.widget = form.add(MonitorWidget, name=self.name, #rely=2+6*nbr, 
                               columns = 3, column_width = None, col_margin=0, row_height = 1,
                               col_titles = ["Node name", "CPU Percent", "RAM MB"], values=[],
                               always_show_cursor = False, select_whole_line = True, scroll_exit=True)
        self.feedback_name = "FEEDBACK_"+str(nbr)
        form.parentApp.add_event_hander(self.feedback_name, self.ev_test_event_handler)
        self.widget.add_handlers({curses.ascii.NL:   self.handle_node,
                                  curses.ascii.CR:   self.widget.h_exit_down,
                                  curses.ascii.TAB:  self.widget.h_exit_down,
                                  #curses.KEY_DOWN:   self.widget.h_exit_down,
                                  #curses.KEY_UP:     self.widget.h_exit_up,
                                  #curses.KEY_LEFT:   self.widget.h_exit_left,
                                  #curses.KEY_RIGHT:  self.widget.h_exit_right,
                                  "^P":              self.widget.h_exit_up,
                                  "^N":              self.widget.h_exit_down,
                                  curses.ascii.ESC:  self.widget.h_exit_escape, # NOTE: this is good
                                  curses.KEY_MOUSE:  self.widget.h_exit_mouse})

    def handle_node(self, event):

        row, col = self.widget.edit_cell
        node_name = self.widget.values[row][0]
        menu = self.widget.parent.new_menu(name=node_name)
        menu.addItem("Kill", onSelect=self.kill_node, arguments=(node_name,))
        menu.addItem("Restart", onSelect=self.restart_node, arguments=(node_name,))
        menu.addItem("Cancel") #, onSelect=self.restart_node, arguments=(node_name,))
        self.widget.parent.popup_menu(menu)

    def kill_node(self, node_name):

        self.monitor_server.node_action(node_name, NodeActionRequest.KILL)
        
    def restart_node(self, node_name):

        self.monitor_server.node_action(node_name, NodeActionRequest.RESTART)

    def update_widget(self, form):

        self.widget = None
        self.widget = form.add(MonitorWidget, name=self.name, #rely=2+6*nbr, 
                               columns = 3, column_width = None, col_margin=0, row_height = 1,
                               col_titles = ["Node name", "CPU Percent", "RAM MB"], values=[],
                               always_show_cursor = False, select_whole_line = True, scroll_exit=True)
        #self.widget.set_relyx(-1, -1)
    
    def feedback_cb(self, msg):    
        self.widget.parent.parentApp.queue_event(FeedbackEvent(self.feedback_name, msg))
    
    def ev_test_event_handler(self, event):
        self.widget.values = [list(v) for v in zip(event.msg.alive_nodes, event.msg.cpu_percent, event.msg.ram_mb)]
        self.widget.parent.display()

class MonitorEvent(npyscreen.Event):

    def __init__(self, name, pkg, launch_file):

        super(MonitorEvent, self).__init__(name)
        self.pkg = pkg
        self.launch_file = launch_file

class MonitorApp(npyscreen.StandardApp):

    def cancel_cb(self):

        #self.parent.parentApp.switchForm(None)
        curses.beep()
        self.switchForm(None)

    def add_monitor(self, event):
        
        form = self.getForm("MAIN")
        #offset = 4+sum(4+len(m.widget.values) for m in self.monitors)
        form.nextrely = 2+10*self.nbr_monitors #self.nextrely
        #form.nextrely = 2
        #form._clear_all_widgets()
        #for m in self.monitors:
        #    m.update_widget(form)
        self.monitors.append(MonitorCollection(form, event.pkg, event.launch_file, self.nbr_monitors))
        self.nbr_monitors += 1
        self.nextrely = form.nextrely #+2
        #rospy.loginfo("Nexrely: ", self.nextrely)
        #form.display()

    def onStart(self):
        #self.keypress_timeout_default = 2
        self.addForm("MAIN", npyscreen.FormBaseNewWithMenus, name=rospy.get_name())
        #self.addForm("MAIN", npyscreen.FormMutt, name=rospy.get_name())
        #self.addForm("MAIN", npyscreen.Form, name=rospy.get_name())
        #self.title = self.getForm("MAIN").add(npyscreen.TitleText, name="Press ESC to exit...")
        self.dummy = self.getForm("MAIN").add(npyscreen.DummyWidget) #, rely=4+10*nbr)
        self.add_event_hander("ADDMONITOR", self.add_monitor)
        self.monitors = []
        self.nbr_monitors = 0
        rospy.on_shutdown(self.cancel_cb)
        self.getForm("MAIN").how_exited_handers[npyscreen.wgwidget.EXITED_ESCAPE]  = self.cancel_cb
        self.nextrely = self.getForm("MAIN").nextrely#+2
        #rospy.loginfo("Nexrely: ", self.nextrely)

    #def onCleanExit(self):
    #
    #    npyscreen.notify_wait("Goodbye!")

if __name__ == '__main__':

    rospy.init_node('monitor_server')
    
    App = MonitorApp()
    if len(sys.argv) > 2:
        #App.queue_event(MonitorEvent("ADDMONITOR", sys.argv[1], sys.argv[2]))
        App.queue_event(MonitorEvent("ADDMONITOR", "rfs_slam", "test_sim.launch"))
        App.queue_event(MonitorEvent("ADDMONITOR", "rfs_slam", "slam.launch"))
    App.run()

