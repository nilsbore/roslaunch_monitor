#!/usr/bin/python

import roslaunch
import rospy
import psutil
from multiprocessing import Pool
import roslaunch_monitor.msg
from functools import partial
import actionlib
#import threading

def get_pid_stats(pid):
    proc = psutil.Process(pid)
    #return proc.cpu_percent(0.1), proc.memory_info().rss
    return proc.get_cpu_percent(0.1), proc.get_memory_info().rss

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
        #self.master_lock = threading.Lock()
        self._feedback_timers = {}
        #self._feedback_timer = None
        self._parents = {}
        #self._parent = None
        self._queued_handles = {}
        rospy.loginfo('Server %s is up', self._action_name)

    def spin(self):

        for goal_id, gh in self._queued_handles.items():
            goal = gh.get_goal()

            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)

            cli_args = [goal.pkg, goal.launch_file]
            roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)
            process_listener = ProcessListener()
            _parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file, process_listeners=[process_listener])
            _parent.start()
            self._parents[goal_id] = _parent
            #self._parent = _parent
            self._feedback_timers[goal_id] = rospy.Timer(rospy.Duration(1), partial(self.feedback_callback, gh, _parent))
            #self._feedback_timer = rospy.Timer(rospy.Duration(1), partial(self.feedback_callback, gh))

        self._queued_handles = {}

    def cancel_cb(self, gh):
        goal_id = gh.get_goal_id().id
        rospy.loginfo('cancel roslaunch goal ' + goal_id)
        #self.p[gh.get_goal_id()].terminate()
        self._feedback_timers.pop(goal_id).shutdown()
        self._parents.pop(goal_id).shutdown()
        gh.set_canceled()

    def execute_cb(self, gh):

        #monitor_thread = threading.Thread(
        #    target=self.monitor_thread_entry, args=(gh, 1))
        goal_id = gh.get_goal_id().id
        rospy.loginfo('trigger roslaunch goal ' + goal_id)

        #self._queued_handles.append(gh)
        self._queued_handles[goal_id] = gh

        while not rospy.is_shutdown() and goal_id in self._queued_handles:
            rospy.sleep(0.1)

        gh.set_accepted()

    def feedback_callback(self, gh, _parent, event):

        _feedback = roslaunch_monitor.msg.LaunchFeedback()
        #goal_id = gh.get_goal_id().id
        #_parent = self._parents[goal_id]

        try:
            pool = Pool(12) #processes=self.nbr_threads)
            result = pool.map(get_pid_stats, (p.pid for p in _parent.pm.procs))
        finally:
            pool.close()
            pool.join()

        #for res, p in zip(result, self._parent.pm.procs):
        #    print "Parent pm procs name: ", p.name
        #    print "Parent pm procs pid: ", p.pid
        #    print "Cpu percent: ", res[0]
        #    print "RAM used (MB): ", 1e-6*float(res[1])

        _feedback.alive_nodes = [p.name for p in _parent.pm.procs]
        _feedback.cpu_percent = [r[0] for r in result]
        _feedback.ram_mb = [1e-6*float(r[1]) for r in result]

        gh.publish_feedback(_feedback)

if __name__ == '__main__':
    rospy.init_node('launch_monitor_server')
    server = LaunchMonitorServer(rospy.get_name())
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        server.spin()
        rate.sleep()
