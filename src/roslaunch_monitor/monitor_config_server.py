from roslaunch_monitor.srv import NodeActionRequest
import rospy

# node: gmm_map_server
#   condition: nbr_restarts
#     limit: 10
#     action: kill
#     window: 1
#   condition: ram_mb
#     limit: 100
#     action: restart
#     window: 2
#     delay: 0.5
#   condition: cpu_percent
#     limit: 10
#     action: restart
#     window: 20
#     delay: 0.5

class NodeActionCondition(object):

    def __init__(self, pars):

        self.condition = pars["condition"]
        self.limit = pars["limit"]
        self.action = pars["action"]
        if "window" in pars:
            self.window = pars["window"]
        else:
            self.window = 1
        if "delay" in pars:
            self.delay = pars["delay"]
        else:
            self.delay = 0.

    def cfg_action(self):
        #rospy.loginfo("Got condition %s with limit %f with action %s", self.condition, self.limit, self.action)
        if self.action == "KILL":
            return (NodeActionRequest.KILL, self.delay)
        elif self.action == "RESTART":
            return (NodeActionRequest.RESTART, self.delay)
        else:
            return (None, None)

class NodeConfig(object):

    def __init__(self, name, cfg):

        self.cfg = [NodeActionCondition(c) for c in cfg]
        self.cfg.sort(key=lambda c: c.action == "RESTART")
        self.nbr_restarts = 0
        self.reset()

    def reset(self):

        self.count = 0
        self.cpu_percent = 0.
        self.ram_mb = 0.
        self.values = {"cpu_percent": self.cpu_percent, "ram_mb": self.ram_mb, "nbr_restarts": self.nbr_restarts}

    def add_meas(self, cpu_percent, ram_mb, nbr_restarts):

        if nbr_restarts > self.nbr_restarts:
            self.reset()

        self.cpu_percent = (self.count*self.cpu_percent+cpu_percent)/float(self.count+1)
        self.ram_mb = (self.count*self.ram_mb+ram_mb)/float(self.count+1)
        self.nbr_restarts = nbr_restarts
        self.values = {"cpu_percent": self.cpu_percent, "ram_mb": self.ram_mb, "nbr_restarts": self.nbr_restarts}
        self.count += 1

    def cfg_action(self):

        for c in self.cfg:
            if self.count > c.window and self.values[c.condition] > c.limit:
                return c.cfg_action()

        return None, None

    def get_monitor_strs(self):

        indices = {"cpu_percent": 1, "ram_mb": 2, "nbr_restarts": 3}
        strs = ["", "", "", ""]
        for c in self.cfg:
            if c.condition == "nbr_restarts":
                strs[indices[c.condition]] = " / {:d}".format(c.limit)
            elif c.condition in indices:
                strs[indices[c.condition]] = " / {:.2f}".format(c.limit)
        return strs

class MonitorConfigServer(object):

    def __init__(self, cfg):

        self.cfg = {key: NodeConfig(key, val) for key, val in cfg.items()}

    def base_name(self, pname):

        pos = pname.find('-')
        if pos == -1:
            return None
        return pname[:pos]

    def add_meas(self, msg):

        for i, pname in enumerate(msg.alive_nodes):
            name = self.base_name(pname)
            if name is not None and name in self.cfg:
                self.cfg[name].add_meas(msg.cpu_percent[i], msg.ram_mb[i], msg.nbr_restarts[i])

    def cfg_action(self, pname):

        name = self.base_name(pname)
        if name is not None and name in self.cfg:
            return self.cfg[name].cfg_action()

        return None, None

    def get_monitor_strs(self):

        return {key: val.get_monitor_strs() for key, val in self.cfg.items()}
