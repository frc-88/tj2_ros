#!/usr/bin/env python3

"""Node to record a rosbag with start/stop/pause control through service calls.
Example call:
    rosrun utilities rosbag_controlled_recording.py _rosbag_command:="rosbag record -o /home/foo/test_bag /bar_topic" _record_from_startup:=false
Then start/pause/resume/stop can be controlled through:
    rosservice call /rosbag_controlled_recording/start
    rosservice call /rosbag_controlled_recording/pause_resume
    rosservice call /rosbag_controlled_recording/pause_resume
    rosservice call /rosbag_controlled_recording/stop
Note that pausing does not modify the recorded time of messages, i.e. the bag's total length is unaffected. A list of
  pause-resume times is logged when stopping, in case the paused period needs to be (manually) removed afterwards.
If this node is killed recording is also stopped. If recording was paused, it is momentarily resumed before stopping.
"""

import re
import os
import time
import psutil
import subprocess
import shlex
import signal

import rospy
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import String
from rosgraph_msgs.msg import Log

from tj2_match_watcher.srv import TriggerBag, TriggerBagResponse


def signal_process_and_children(pid, signal_to_send, wait=False):
    process = psutil.Process(pid)
    for children in process.children(recursive=True):
        if signal_to_send == 'suspend':
            children.suspend()
        elif signal_to_send == 'resume':
            children.resume()
        else:
            children.send_signal(signal_to_send)
    if wait:
        process.wait()


def format_to_columns(input_list, cols):
    """Adapted from https://stackoverflow.com/questions/171662/formatting-a-list-of-text-into-columns"""
    max_width = max(map(len, input_list))
    justify_list = list(map(lambda x: x.ljust(max_width + 4), input_list))
    lines = (''.join(justify_list[i:i + cols]) for i in range(0, len(justify_list), cols))
    return '\n'.join(lines)


def load_topics(path):
    with open(path) as file:
        return file.read().splitlines()


class RosbagControlledRecorder(object):
    """Record a rosbag with service calls to control start, stop  and pause"""

    def __init__(self):
        self.name = "rosbag_controlled_recording"
        rospy.init_node(
            self.name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
        # Get parameters
        self.rosbag_command_param = rospy.get_param('~rosbag_command')  # str with rosbag command line command to be issued
        self.record_from_startup = rospy.get_param('~record_from_startup', False)  # whether to start node already recording
        self.topics_path = rospy.get_param('~topics_path', "topics.txt")

        topics = load_topics(self.topics_path)

        # Services
        self.start_service = rospy.Service('~start', TriggerBag, self.start_recording_srv)
        self.resume_service = rospy.Service('~resume', Empty, self.resume_recording_srv)
        self.pause_service = rospy.Service('~pause', Empty, self.pause_recording_srv)
        self.stop_service = rospy.Service('~stop', TriggerBag, self.stop_recording_srv)

        self.rosout_sub = rospy.Subscriber("/rosout_agg", Log, self.rosout_callback, queue_size=25)

        self.bag_status_pub = rospy.Publisher("bag_status", String, queue_size=10)
        self.bag_name_pub = rospy.Publisher("bag_name", String, queue_size=10)

        # Recording is also stopped on node shutdown. This allows stopping to be done via service call or regular Ctrl-C
        rospy.on_shutdown(self.stop_recording_srv)

        self.topics = topics
        self.raw_rosbag_command = self.rosbag_command_param + " " + " ".join(topics)
        self.rosbag_command = shlex.split(self.raw_rosbag_command)
        self.rosbag_node_name = self.get_record_node_name()
        self.recording_started = False
        self.recording_paused = False
        self.recording_stopped = False
        self.pause_resume_times = []
        self.process_pid = None
        self.bag_name = ""
        self.bag_status = "idle"
        if self.record_from_startup:
            self.start_recording_srv()

    def get_bag_prefix(self):
        match = re.search(r"-o ([\S]+)", self.raw_rosbag_command)
        if match is None:
            raise ValueError("Failed to find bag directory in rosbag command (add -o to your command): %s" % self.raw_rosbag_command)
        return match.group(1)
    
    def get_record_node_name(self):
        match = re.search(r"__name:=(\S+)\s", self.raw_rosbag_command)
        if match is None:
            raise ValueError("Failed to find bag node name: %s" % self.raw_rosbag_command)
        return match.group(1)

    def get_latest_bag(self):
        prefix = self.get_bag_prefix()
        bag_dir = os.path.dirname(prefix)
        bag_name_prefix = os.path.basename(prefix)
        latest_modify = 0.0
        latest_path = ""
        for filename in os.listdir(bag_dir):
            if filename.startswith(bag_name_prefix):
                path = os.path.join(bag_dir, filename)
                modify_time = os.path.getmtime(path)
                if modify_time > latest_modify:
                    latest_modify = modify_time
                    latest_path = path
        return latest_path

    def start_recording_srv(self, service_message=None):
        if self.recording_started:
            rospy.logwarn("Recording has already started - nothing to be done")
            return TriggerBagResponse(True, self.bag_name)
            
        self.recording_started = True
        self.bag_name = ""
        process = subprocess.Popen(self.rosbag_command)
        rospy.loginfo("Started recording rosbag: %s" % process.pid)
        self.process_pid = process.pid
        self.bag_status = "recording"

        start_time = rospy.Time.now()
        while len(self.bag_name) == 0:
            if rospy.Time.now() - start_time > rospy.Duration(20.0):
                rospy.logerr("Failed to detect start of bag.")
                signal_process_and_children(self.process_pid, signal.SIGINT, wait=True)
                return TriggerBagResponse(False, self.bag_name)
            rospy.sleep(1.0)
        rospy.loginfo("Found recording start flag. Bag name is '%s'" % self.bag_name)

        self.recording_paused = False
        self.pause_recording_srv()

        # self.bag_name = self.get_latest_bag()

        return TriggerBagResponse(True, self.bag_name)

    def resume_recording_srv(self, service_message=None):
        if self.recording_started:
            if self.recording_paused:
                signal_process_and_children(self.process_pid, 'resume')
                self.recording_paused = False
                rospy.loginfo("Recording resumed")
                self.pause_resume_times.append(rospy.get_time())
            else:
                rospy.loginfo("Recording is already resumed")
            # self.bag_name = self.get_latest_bag()
            self.bag_status = "recording"
        else:
            rospy.logwarn("Recording not yet started - nothing to be done")
        return EmptyResponse()

    def pause_recording_srv(self, service_message=None):
        if self.recording_started:
            if not self.recording_paused:
                rospy.loginfo("Recording paused")
                signal_process_and_children(self.process_pid, 'suspend')
                self.recording_paused = True
                self.pause_resume_times.append(rospy.get_time())
            else:
                rospy.loginfo("Recording is already paused")
            self.bag_status = "paused"
            # self.bag_name = self.get_latest_bag()
        else:
            rospy.logwarn("Recording not yet started - nothing to be done")
        return EmptyResponse()

    def stop_recording_srv(self, service_message=None):
        if not self.recording_started:
            rospy.logwarn("Recording has already stopped - nothing to be done")
            return TriggerBagResponse(True, self.bag_name)

        if self.process_pid is not None:
            if self.recording_paused:  # need to resume process in order to cleanly kill it
                self.resume_recording_srv()
            if self.pause_resume_times:  # log pause/resume times for user's reference
                pause_resume_str = list(map(str, self.pause_resume_times))
                pause_resume_str[0:0] = ['PAUSE', 'RESUME']
                rospy.loginfo("List of pause and resume times:\n%s\n", format_to_columns(pause_resume_str, 2))
            signal_process_and_children(self.process_pid, signal.SIGINT, wait=True)
            self.process_pid = None
            self.recording_paused = False
            self.recording_started = False
            rospy.loginfo("Stopped recording rosbag. Path is '%s'" % self.bag_name)
        self.recording_stopped = True
        # self.bag_name = self.get_latest_bag()
        response = TriggerBagResponse(True, self.bag_name)
        self.bag_status = "idle"
        return response

    def rosout_callback(self, msg):
        if self.rosbag_node_name in msg.name:
            match = re.search(r"Recording to \'(.+)\'\.", msg.msg)
            if match is not None:
                self.bag_name = match.group(1)

    def run(self):
        clock_rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            clock_rate.sleep()

            bag_name_msg = String()
            bag_name_msg.data = self.bag_name
            self.bag_name_pub.publish(bag_name_msg)

            bag_status_msg = String()
            bag_status_msg.data = self.bag_status
            self.bag_status_pub.publish(bag_status_msg)


if __name__ == "__main__":
    node = RosbagControlledRecorder()
    try:
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting %s node" % node.name)
