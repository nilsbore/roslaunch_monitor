#! /usr/bin/env python

import rospy
import sys
#from __future__ import print_function
import roslaunch_monitor.msg

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import actionlib_tutorials.msg

class MonitorServer(object):

    def feedback_cb(self, msg):
        print msg

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

    def spin(self):

        if not self.client.wait_for_server(timeout=rospy.Duration(0.5)) or \
               (self.client.get_state() != actionlib.SimpleGoalState.ACTIVE and \
                self.client.get_state() != actionlib.SimpleGoalState.PENDING):
            rospy.loginfo("Launch server was aborted, quitting...")
            self.client.stop_tracking_goal()
            self.client = None
            return False
        return True

if __name__ == '__main__':
    rospy.init_node('monitor_server')
    monitor_server = MonitorServer()

    monitor_server.launch(sys.argv[1], sys.argv[2])
    while monitor_server.spin():
        rospy.sleep(0.2)

