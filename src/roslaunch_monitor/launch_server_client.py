import rospy
import roslaunch_monitor.msg
from roslaunch_monitor.srv import NodeAction, NodeActionRequest
import actionlib

class LaunchServerClient(object):

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

    def launch(self, pkg, launch_file, parameters, values):
        # Creates the SimpleActionClient, passing the type of the action
        # (FibonacciAction) to the constructor.
        self.client = actionlib.SimpleActionClient('/launch_monitor_server', roslaunch_monitor.msg.LaunchAction)

        # Waits until the action server has started up and started
        # listening for goals.
        self.client.wait_for_server()

        # Creates a goal to send to the action server.
        goal = roslaunch_monitor.msg.LaunchGoal(pkg=pkg, launch_file=launch_file, parameters=parameters, values=values)

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

        #rospy.loginfo("Early shutting down launch server...")
        self.client.cancel_goal()
        self.client.wait_for_result()
        #rospy.loginfo("Finished shutting down launch server...")

    def add_callback(self, cb):

        self._feedback_cb = cb

    def spin(self):

        if not self.client.wait_for_server(timeout=rospy.Duration(0.5)) or \
               (self.client.get_state() != actionlib.SimpleGoalState.ACTIVE and \
                self.client.get_state() != actionlib.SimpleGoalState.PENDING):
            #rospy.loginfo("Launch server was aborted, quitting...")
            self.client.stop_tracking_goal()
            self.client = None
            return False
        return True
