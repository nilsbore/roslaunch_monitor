#! /usr/bin/env python
import rospy
from roslaunch_monitor.monitor_app import MonitorApp, MonitorEvent
import sys

if __name__ == '__main__':

    rospy.init_node('monitor_server')
    
    App = MonitorApp()
    if len(sys.argv) > 2:
        #App.queue_event(MonitorEvent("ADDMONITOR", sys.argv[1], sys.argv[2]))
        App.queue_event(MonitorEvent("ADDMONITOR", "rfs_slam", "test_sim.launch"))
        App.queue_event(MonitorEvent("ADDMONITOR", "rfs_slam", "slam.launch"))
        #App.queue_event(MonitorEvent("ADDMONITOR", "auv_sensors", "auv_sensors.launch"))
    App.run()

