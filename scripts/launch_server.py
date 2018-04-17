#!/usr/bin/python
import rospy
from roslaunch_monitor.launch_monitor_server import LaunchMonitorServer

if __name__ == '__main__':
    rospy.init_node('launch_monitor_server')
    server = LaunchMonitorServer(rospy.get_name())
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        server.spin()
        rate.sleep()
