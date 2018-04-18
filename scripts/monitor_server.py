#! /usr/bin/env python
import rospy
from roslaunch_monitor.monitor_app import MonitorApp
import sys

if __name__ == '__main__':

    rospy.init_node('monitor_server')

    parameters = []
    values = []
    for arg in sys.argv[3:]:
        pos = arg.find(":=")
        if pos > 0 and pos + 2 < len(arg):
            parameters.append(arg[:pos])
            values.append(arg[pos+2:])
    
    App = MonitorApp()
    if len(sys.argv) > 2:
        #monitor_cfg = {'test_slam_sim_node': [{'condition': 'ram_mb', 'action': 'RESTART', 'limit': 30, 'window': 2, 'delay': 0.1},
        #                                      {'condition': 'nbr_restarts', 'action': 'KILL', 'limit': 10}]}
        monitor_cfg = {}
        App.queue_launch(sys.argv[1], sys.argv[2], monitor_cfg, parameters, values)
    App.run()

