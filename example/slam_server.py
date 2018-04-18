#! /usr/bin/env python
import rospy
from roslaunch_monitor.monitor_app import MonitorApp
import sys

if __name__ == '__main__':

    rospy.init_node('slam_server')
    
    App = MonitorApp()
    slam_monitor_cfg = {'test_slam_node': [{'condition': 'ram_mb', 'action': 'RESTART', 'limit': 90, 'window': 100},
                                           {'condition': 'nbr_restarts', 'action': 'KILL', 'limit': 1}]}
    sim_monitor_cfg = {'test_slam_sim_node': [{'condition': 'ram_mb', 'action': 'RESTART', 'limit': 30, 'window': 2, 'delay': 0.1},
                                              {'condition': 'nbr_restarts', 'action': 'KILL', 'limit': 10}]}
    App.queue_launch("rfs_slam", "test_sim.launch", sim_monitor_cfg, ["namespace"], ["my_auv"])
    App.queue_launch("rfs_slam", "slam.launch", slam_monitor_cfg, ["sensor_range", "namespace"], ["43", "my_auv"])

    App.run()

