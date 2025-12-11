#!/usr/bin/env python3
"""
Created on: May 2, 2017
    Author: Wenxing Mei <mwx36mwx@gmail.com>

TOPIC: 'reload_config'
cwd: $HOME/humanoid/src/dconfig/scripts
Watching directory: ../dmotion
Launched by dmotion/launch/dmotion.launch
Watch config directory, call `rosparam load` and publish msg when file is changed.
"""

import subprocess
import rospy
import os
from std_msgs.msg import String
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
from time import sleep

rospy.init_node('config_watchdog')
ROBOT_ID = rospy.get_param('~RobotId')
PATH = '../{}'.format(ROBOT_ID)
MOTION_CONFIG = [
    'motion.yml',
    'motor.yml',
    'kick.yml',
    'fastkick.yml',
    'sidekick.yml',
    'setup.yml',
    'pvhipY.yml',
    'goalie.yml'
]
MOTION_TOPIC = '/humanoid/ReloadMotionConfig'
VISION_CONFIG = [
    'amcl.yml',
    'misc.yml',
    'camera.yml',
    'localization.yml'
]
VISION_TOPIC = '/humanoid/ReloadVisionConfig'
BEHAVIOR_CONFIG = [
    'move.yml',
]
BEHAVIOR_TOPIC = '/humanoid/ReloadBehaviorConfig'


class Handler(FileSystemEventHandler):
    """File Handler."""

    def __init__(self):
        """Init."""
        super(Handler, self).__init__()
        self.pub_motion = rospy.Publisher(MOTION_TOPIC, String, queue_size=1)
        self.pub_vision = rospy.Publisher(VISION_TOPIC, String, queue_size=1)
        self.pub_behavior = rospy.Publisher(BEHAVIOR_TOPIC, String, queue_size=1)
        rospy.loginfo('CWD: {}'.format(os.getcwd()))

    def on_modified(self, event):
        """Callback on modified."""
        if not event.is_directory:
            _, filename = os.path.split(event.src_path)
            if filename.split('.')[-1] in ['yml', 'yaml']:
                if filename in MOTION_CONFIG:
                    rospy.loginfo(
                        'motion: rosparam load {}/dmotion/{}'.format(PATH, filename))
                    subprocess.call(
                        ['rosparam', 'load', '{}/dancer-io/{}'.format(PATH, filename), 'dmotion_{}'.format(ROBOT_ID)])
                    sleep(1)
                    self.pub_motion.publish('reload')
                    rospy.loginfo('config changed, published reload msg')
                elif filename in VISION_CONFIG:
                    rospy.loginfo(
                        'vision: rosparam load {}/dvision/{}'.format(PATH, filename))
                    subprocess.call(
                        ['rosparam', 'load', '{}/dvision/{}'.format(PATH, filename), 'dvision_{}'.format(ROBOT_ID)])
                    sleep(1)
                    self.pub_vision.publish('')
                    rospy.loginfo('config changed, published reload msg')
                elif filename in BEHAVIOR_CONFIG:
                    rospy.loginfo(
                        'behavior: rosparam load {}/dbehavior/{}'.format(PATH, filename))
                    subprocess.call(
                        ['rosparam', 'load', '{}/dbehavior/{}'.format(PATH, filename), 'dbehavior_{}'.format(ROBOT_ID)])
                    sleep(1)
                    self.pub_behavior.publish('')
                    rospy.loginfo('config changed, published reload msg')
            else:
                rospy.logwarn('{} not yaml file'.format(event.src_path))


if __name__ == '__main__':
    try:
        event_handler = Handler()
        observer = Observer()
        observer.schedule(event_handler, path=PATH, recursive=True)
        observer.start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
