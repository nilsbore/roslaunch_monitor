import roslaunch
import rospy
import psutil
from multiprocessing import Pool
import roslaunch_monitor.msg
from functools import partial
import actionlib
from roslaunch_monitor.srv import NodeAction, NodeActionResponse, NodeActionRequest
from collections import Counter
import sys
#import threading

def get_pid_stats(pid):
    try:
        proc = psutil.Process(pid)
        #return proc.cpu_percent(0.1), proc.memory_info().rss
        #return proc.get_cpu_percent(0.1), proc.get_memory_info().rss
        return 0., proc.memory_info().rss
    except: # (AssertionError, TypeError):
        return 0., 0.

class ProcessListener(roslaunch.pmon.ProcessListener):

    def __init__(self, goal_id):
        super(ProcessListener, self).__init__()
        self._goal_id = goal_id

    def process_died(self, name, exit_code):
        rospy.logwarn("Goal id %s: %s died with code %s", self._goal_id, name, exit_code)

class LaunchMonitorServer(object):

    def __init__(self, name):

        self._action_name = name
        self._as = actionlib.ActionServer(
            self._action_name, roslaunch_monitor.msg.LaunchAction,
            self.execute_cb, self.cancel_cb, auto_start=False)
        self._as.start()
        self.node_service = rospy.Service('~node_action', NodeAction, self.node_action_cb)
        #self.p = {}
        # a lock to mutex access to the master object
        #self.master_lock = threading.Lock()
        self._feedback_timers = {}
        self._parents = {}
        self._queued_handles = {}
        rospy.loginfo('Server %s is up', self._action_name)

    def node_action_cb(self, req):

        rospy.loginfo("Got callback with goal id %s and name %s", req.goal_id, req.node_name)
        if req.goal_id not in self._parents:
            return NodeActionResponse()
        _parent = self._parents[req.goal_id]

        for p in _parent.pm.procs:
            if p.name == req.node_name:
                if req.action == NodeActionRequest.RESTART:
                    rospy.loginfo("Found %s, restarting...", p.name)
                    p.respawn = True
                    #p.respawn_delay = 0.1
                    p.stop()
                elif req.action == NodeActionRequest.KILL:
                    rospy.loginfo("Found %s, killing...", p.name)
                    p.respawn = False
                    p.stop()
                break

        return NodeActionResponse()

    def spin(self):

        for goal_id, gh in self._queued_handles.items():
            goal = gh.get_goal()

            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)

            print "Goal pars: ", goal.parameters, goal.values
            #if goal.launch_file == "slam.launch":
            #    sys.argv.append("sensor_range:=40.0")
            for arg, val in zip(goal.parameters, goal.values):
                sys.argv.append("{}:={}".format(arg, val))

            cli_args = [goal.pkg, goal.launch_file]
            roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)
            process_listener = ProcessListener(goal_id)
            _parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file, process_listeners=[process_listener])

            _parent.start()
            self._parents[goal_id] = _parent
            self._feedback_timers[goal_id] = rospy.Timer(rospy.Duration(1), partial(self.feedback_callback, gh, _parent))

            for arg, val in zip(goal.parameters, goal.values):
                sys.argv.pop()
            #gh.set_active()
            print "GH methods: ", dir(gh)
            print "AP methods: ", dir(self._as)

        self._queued_handles = {}

    def cancel_cb(self, gh):
        goal_id = gh.get_goal_id().id
        rospy.loginfo('cancel roslaunch goal ' + goal_id)
        if goal_id not in self._parents:
            #gh.set_canceled()
            return
        self._feedback_timers.pop(goal_id).shutdown()
        self._parents.pop(goal_id).shutdown()
        gh.set_canceled()

    def execute_cb(self, gh):

        #monitor_thread = threading.Thread(
        #    target=self.monitor_thread_entry, args=(gh, 1))
        goal_id = gh.get_goal_id().id
        rospy.loginfo('trigger roslaunch goal ' + goal_id)

        self._queued_handles[goal_id] = gh

        while not rospy.is_shutdown() and goal_id in self._queued_handles:
            rospy.sleep(0.1)

        gh.set_accepted()

    def feedback_callback(self, gh, _parent, event):

        _feedback = roslaunch_monitor.msg.LaunchFeedback()

        try:
            pool = Pool(12) #processes=self.nbr_threads)
            result = pool.map(get_pid_stats, (p.pid for p in _parent.pm.procs))
        finally:
            pool.close()
            pool.join()

        _feedback.alive_nodes = [p.name for p in _parent.pm.procs]
        _feedback.cpu_percent = [r[0] for r in result]
        _feedback.ram_mb = [1e-6*float(r[1]) for r in result]
        _feedback.nbr_restarts = [p.spawn_count-1 for p in _parent.pm.procs]
        # NOTE: maybe we can use spawn_count instead of our own nbr_restarts?
        _feedback.dead_nodes = [p.name for p in _parent.pm.dead_list]
        #deads = [(p.name, p.spawn_count) for p in self.dead_list]

        gh.publish_feedback(_feedback)
        
        if _parent.pm.is_shutdown:
            #gh.set_aborted()
            #gh.set_cancel_requested()
            #gh.set_canceled()
            #gh.set_aborted()
            self.cancel_cb(gh)

