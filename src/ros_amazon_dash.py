#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import time
import os
from yaml import load, CLoader as Loader
import socket
import rospy
from std_msgs.msg import String

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
    rospy.logwarn("Not found rospy_logging")
else:
    amazon_dash.listener.logger.addHandler(ConnectPythonLoggingToROS())
    amazon_dash.listener.logger.setLevel(logging.DEBUG)

DEFAULT_NODE_NAME = "ros_amazon_dash"
DEFAULT_TOPIC_NAME_PUSHED = "pushed"


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

    def __init__(self, devices, settings):
        self.settings = settings
        self.devices = devices
        print self.devices

    def on_push(self, callback):
        def _on_push(device):
            src = device.src.lower()
            if last_execution[src] + self.settings.get('delay', 10) > time.time():
                return
            last_execution[src] = time.time()
            if not self.devices.has_key(src):
                rospy.logwarn("Not found " + src + " in yaml file")
                return
            rospy.loginfo("Pushed: " + self.devices[src]["name"])
            if callback is not None:
                callback(str(self.devices[src]["name"]))

        return _on_push

    def execute(self, device):
        src = device.src.lower()
        device = self.devices[src]
        device.execute(root_allowed=self.root_allowed)

    def run(self, root_allowed=False, callback=None):
        self.root_allowed = root_allowed
        scan(self.on_push(callback), lambda d: d.src.lower() in self.devices)


# --------------------------------------------
if __name__ == '__main__':
    rospy.init_node(DEFAULT_NODE_NAME, anonymous=True, log_level=rospy.DEBUG)
    pub = rospy.Publisher(DEFAULT_TOPIC_NAME_PUSHED, String, queue_size=10)
    devices = rospy.get_param("~devices", {'ff:ff:ff:ff:ff:ff': {'cmd': 'No config file',
                                                                 'user': 'No config file',
                                                                 'name': 'No config file'}
                                           })
    settings = rospy.get_param("~settings", {'delay': 0})

    try:
        Listener(devices, settings).run(True, pub.publish)
    except socket.error as e:
        rospy.logerr(e)
        rospy.logerr("You should do...")
        rospy.logerr("$ sudo -s")
        rospy.logerr("# cd <your catkin_workspace>")
        rospy.logerr("# source devel/setup.bash")
        rospy.logerr("# rosrun ros_amazon_dash ros_amazon_dash.py")
        rospy.logerr("if you want to know mac address of the dash, do...")
        rospy.logerr("# sudo amazon-dash discovery")
    else:
        rospy.spin()
