#! /usr/bin/env python
import rospy
from roslaunch_monitor.monitor_app import MonitorApp, MonitorEvent
import sys

if __name__ == '__main__':

    rospy.init_node('monitor_server')
    
    App = MonitorApp()
    if len(sys.argv) > 2:
        #App.queue_event(MonitorEvent("ADDMONITOR", sys.argv[1], sys.argv[2]))
        slam_monitor_cfg = {'test_slam_node': [{'condition': 'ram_mb', 'action': 'RESTART', 'limit': 60, 'window:': 100},
                                               {'condition': 'nbr_restarts', 'action': 'KILL', 'limit': 1}]}
        sim_monitor_cfg = {'test_slam_sim_node': [{'condition': 'ram_mb', 'action': 'RESTART', 'limit': 50, 'window:': 1},
                                                  {'condition': 'nbr_restarts', 'action': 'KILL', 'limit': 1}]}
        App.queue_event(MonitorEvent("ADDMONITOR", "rfs_slam", "test_sim.launch", sim_monitor_cfg))
        App.queue_event(MonitorEvent("ADDMONITOR", "rfs_slam", "slam.launch", slam_monitor_cfg))
        #App.queue_event(MonitorEvent("ADDMONITOR", "auv_sensors", "auv_sensors.launch"))
    App.run()

