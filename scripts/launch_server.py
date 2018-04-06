#!/usr/bin/python

import roslaunch
import rospy
import psutil
from multiprocessing import Pool
import roslaunch_monitor.msg
from functools import partial
import actionlib
import threading

def get_pid_stats(pid):
    proc = psutil.Process(pid)
    return proc.cpu_percent(0.1), proc.memory_info().rss

class ProcessListener(roslaunch.pmon.ProcessListener):

    def process_died(self, name, exit_code):
        rospy.logwarn("%s died with code %s", name, exit_code)

class LaunchMonitorServer(object):

    def __init__(self, name):

        self._action_name = name
        self._as = actionlib.ActionServer(
            self._action_name, roslaunch_monitor.msg.LaunchAction,
            self.execute_cb, self.cancel_cb, auto_start=False)
        self._as.start()
        #self.p = {}
        # a lock to mutex access to the master object
        self.master_lock = threading.Lock()
        self._feedback_timer = None
        self._parent = None
        self._queued_handles = []
        rospy.loginfo('Server %s is up', self._action_name)

    def spin(self):

        for gh in self._queued_handles:
            goal = gh.get_goal()
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)

            #cli_args = ['rfs_slam', 'test_slam.launch']
            cli_args = [goal.pkg, goal.launch_file]
            roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)
            process_listener = ProcessListener()
            self._parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file, process_listeners=[process_listener])
            self._parent.start()
            self._feedback_timer = rospy.Timer(rospy.Duration(1), partial(self.feedback_callback, gh))
        self._queued_handles = []


    def cancel_cb(self, gh):
        rospy.loginfo('cancel roslaunch goal ' + gh.get_goal_id().id)
        #self.p[gh.get_goal_id()].terminate()
        self._feedback_timer.shutdown()
        self._parent.shutdown()
        gh.set_canceled()

    def execute_cb(self, gh):

        #monitor_thread = threading.Thread(
        #    target=self.monitor_thread_entry, args=(gh, 1))
        rospy.loginfo('trigger roslaunch goal ' + gh.get_goal_id().id)

        self._queued_handles.append(gh)

        while not rospy.is_shutdown() and gh in self._queued_handles:
            rospy.sleep(0.1)

        gh.set_accepted()

    def feedback_callback(self, gh, event):

        _feedback = roslaunch_monitor.msg.LaunchFeedback()

        try:
            pool = Pool(12) #processes=self.nbr_threads)
            result = pool.map(get_pid_stats, (p.pid for p in self._parent.pm.procs))
        finally:
            pool.close()
            pool.join()

        for res, p in zip(result, self._parent.pm.procs):
            print "Parent pm procs name: ", p.name
            print "Parent pm procs pid: ", p.pid
            print "Cpu percent: ", res[0]
            print "RAM used (MB): ", 1e-6*float(res[1])

        _feedback.alive_nodes = [p.name for p in self._parent.pm.procs]
        _feedback.cpu_percent = [r[0] for r in result]
        _feedback.ram_mb = [1e-6*float(r[1]) for r in result]

        gh.publish_feedback(_feedback)

if __name__ == '__main__':
    rospy.init_node('launch_monitor_server')
    server = LaunchMonitorServer(rospy.get_name())
    while not rospy.is_shutdown():
        server.spin()
