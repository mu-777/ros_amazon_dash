#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import time
import os
from yaml import load, CLoader as Loader
import socket
import rospy

try:
    import amazon_dash.listener
    from amazon_dash.listener import Device, last_execution
    from amazon_dash.config import only_root_write, oth_w_perm
    from amazon_dash.exceptions import SecurityException
    from amazon_dash.scan import scan
except ImportError:
    rospy.logerr('Not found pyhton package "amazon_dash"')
    rospy.logerr('-> You should do "pip install amazon_dash"')
    sys.exit()

try:
    import logging
    from my_common.rospy_logging import ConnectPythonLoggingToROS
except ImportError:
    rospy.logwarn("Fail to connect phue logger to ros")
else:
    amazon_dash.listener.logger.addHandler(ConnectPythonLoggingToROS())
    amazon_dash.listener.logger.setLevel(logging.DEBUG)

DEFAULT_NODE_NAME = "ros_amazon_dash"


class Config(dict):
    def __init__(self, file, **kwargs):
        super(Config, self).__init__(**kwargs)
        # if (not os.getuid() and not only_root_write(file)) or oth_w_perm(file):
        #     raise SecurityException('There should be no permissions for other users in the file "{}". {}.'.format(
        #         file, 'Removes write permission for others' if os.getuid()
        #         else 'Only root must be able to write to file'
        #     ))
        self.file = file
        self.read()

    def read(self):
        self.update(load(open(self.file), Loader))


class Listener(object):
    root_allowed = False

    def __init__(self, config_path):
        self.config = Config(config_path)
        self.settings = self.config.get('settings', {})
        self.devices = {key.lower(): Device(key, value) for key, value in self.config['devices'].items()}
        assert len(self.devices) == len(self.config['devices']), "Duplicate(s) MAC(s) on devices config."

    def on_push(self, device):
        src = device.src.lower()
        if last_execution[src] + self.settings.get('delay', 10) > time.time():
            return
        last_execution[src] = time.time()
        print "pushed!"

    def execute(self, device):
        src = device.src.lower()
        device = self.devices[src]
        device.execute(root_allowed=self.root_allowed)

    def run(self, root_allowed=False):
        self.root_allowed = root_allowed
        scan(self.on_push, lambda d: d.src.lower() in self.devices)


# --------------------------------------------
if __name__ == '__main__':
    rospy.init_node(DEFAULT_NODE_NAME, anonymous=True, log_level=rospy.DEBUG)

    try:
        Listener(os.path.abspath(os.path.dirname(__file__)) + "/amazon-dash.yml").run(True)
    except socket.error as e:
        rospy.logerr(e)
        rospy.logerr("You should do...")
        rospy.logerr("$ sudo -s")
        rospy.logerr("# cd <your catkin_workspace>")
        rospy.logerr("# source devel/setup.bash")
        rospy.logerr("# rosrun ros_amazon_dash ros_amazon_dash.py")
    else:
        # rospy.spin()
        from std_msgs.msg import String
        pub = rospy.Publisher('chatter', String, queue_size=10)
        r = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            str = "hello world %s" % rospy.get_time()
            rospy.loginfo(str)
            pub.publish(str)
            r.sleep()
