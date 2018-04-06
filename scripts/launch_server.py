#!/usr/bin/python

import roslaunch
import rospy
import psutil
from multiprocessing import Pool

def get_pid_stats(pid):
    proc = psutil.Process(pid)
    return proc.cpu_percent(0.1), proc.memory_info().rss

class ProcessListener(roslaunch.pmon.ProcessListener):

    def process_died(self, name, exit_code):
        rospy.logwarn("%s died with code %s", name, exit_code)

if __name__ == "__main__":
    rospy.init_node("remote_launcher")

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    cli_args = ['rfs_slam', 'test_slam.launch']
    roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)
    process_listener = ProcessListener()
    parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file, process_listeners=[process_listener])
    pool = Pool(processes=10) # start 4 worker processes

    parent.start()
    while not rospy.is_shutdown():
        rospy.sleep(10.)
        #print "===================="
        #print "Parent: ", dir(parent)
        #print "Parent pm: ", dir(parent.pm)
        #print "Parent pm procs: ", parent.pm.procs
        #print "Parent pm procs[0]: ", dir(parent.pm.procs[0])
        
        #names = parent.pm.get_active_names()
        #for name in names:
        #    print name
        #    print parent.pm.get_process(name).get_info()

        #procs = [psutil.Process(p.pid) for p in parent.pm.procs]
        #result = pool.map(lambda p: (p.cpu_percent(0.1), p.memory_info().rss), procs)
        try:
            pool = Pool(12) #processes=self.nbr_threads)
            result = pool.map(get_pid_stats, (p.pid for p in parent.pm.procs))
        finally:
            pool.close()
            pool.join()

        #for i, p in enumerate(parent.pm.procs):
        #    print "Parent pm procs[%d] name: " % i, p.name
        #    print "Parent pm procs[%d] pid: " % i, p.pid
        #    proc = psutil.Process(p.pid)
        #    print "Cpu percent: ", proc.cpu_percent(0.1)
        #    print "RAM used (MB): ", 1e-6*float(proc.memory_info().rss)

        for res, p in zip(result, parent.pm.procs):
            print "Parent pm procs name: ", p.name
            print "Parent pm procs pid: ", p.pid
            print "Cpu percent: ", res[0]
            print "RAM used (MB): ", 1e-6*float(res[1])
    
    parent.shutdown()
