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


class RosbagControlledRecorder(object):
    """Record a rosbag with service calls to control start, stop  and pause"""

    def __init__(self, rosbag_command_, record_from_startup_=False):
        self.raw_rosbag_command = rosbag_command_
        self.rosbag_command = shlex.split(self.raw_rosbag_command)
        self.recording_started = False
        self.recording_paused = False
        self.recording_stopped = False
        self.pause_resume_times = []
        self.process_pid = None
        self.bag_name = ""
        self.bag_status = "idle"
        if record_from_startup_:
            self.start_recording_srv()

    def get_bag_prefix(self):
        match = re.search(r"-o ([\S]+)", self.raw_rosbag_command)
        if match is None:
            raise ValueError("Failed to find bag directory in rosbag command (add -o to your command): %s" % self.raw_rosbag_command)
        return match.group(1)

    def start_recording_srv(self, service_message=None):
        if self.recording_started:
            rospy.logwarn("Recording has already started - nothing to be done")
        else:
            creation_time = time.time()
            process = subprocess.Popen(self.rosbag_command)
            self.process_pid = process.pid
            rospy.loginfo("Started recording rosbag: %s" % self.process_pid)
            self.recording_started = True
            prefix = self.get_bag_prefix()
            rospy.loginfo("Bag has prefix: %s" % prefix)
            bag_dir = os.path.dirname(prefix)
            bag_name_prefix = os.path.basename(prefix)
            self.bag_name = ""

            for attempt in range(10):
                while len(self.bag_name) == 0:
                    for filename in os.listdir(bag_dir):
                        if filename.startswith(bag_name_prefix):
                            path = os.path.join(bag_dir, filename)
                            modify_time = os.path.getmtime(path)
                            if modify_time >= creation_time:
                                self.bag_name = path
                                break
                        if time.time() - creation_time > 10.0:
                            self.bag_name = ""
                            signal_process_and_children(self.process_pid, signal.SIGINT, wait=True)
                            process = subprocess.Popen(self.rosbag_command)
                            self.process_pid = process.pid
                            rospy.loginfo("Started recording rosbag: %s" % self.process_pid)
                            break
                    time.sleep(0.25)
                if len(self.bag_name) > 0:
                    break
                else:
                    rospy.logwarn("Failed to create bag file. Trying again. Attempt #%s" % attempt)
        if len(self.bag_name) == 0:
            raise RuntimeError("Bag file was not created!")
        
        rospy.loginfo("rosbag reports bag path is '%s'" % self.bag_name)
        self.bag_status = "recording"

        rospy.sleep(1.0)
        self.recording_paused = False
        self.pause_recording_srv()

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
            self.bag_status = "recording"
            return EmptyResponse()
        else:
            rospy.logwarn("Recording not yet started - nothing to be done")

    def pause_recording_srv(self, service_message=None):
        if self.recording_started:
            if not self.recording_paused:
                signal_process_and_children(self.process_pid, 'suspend')
                self.recording_paused = True
                rospy.loginfo("Recording paused")
                self.pause_resume_times.append(rospy.get_time())
            else:
                rospy.loginfo("Recording is already paused")
            self.bag_status = "paused"
            return EmptyResponse()
        else:
            rospy.logwarn("Recording not yet started - nothing to be done")

    def stop_recording_srv(self, service_message=None):
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
        response = TriggerBagResponse(True, self.bag_name)
        self.bag_name = ""
        self.bag_status = "idle"
        return response


if __name__ == '__main__':
    rospy.init_node('rosbag_controlled_recording')

    # Get parameters
    rosbag_command = rospy.get_param('~rosbag_command')  # str with rosbag command line command to be issued
    record_from_startup = rospy.get_param('~record_from_startup', False)  # whether to start node already recording

    # Start recorder object
    recorder = RosbagControlledRecorder(rosbag_command, record_from_startup)

    # Services
    start_service = rospy.Service('~start', TriggerBag, recorder.start_recording_srv)
    resume_service = rospy.Service('~resume', Empty, recorder.resume_recording_srv)
    pause_service = rospy.Service('~pause', Empty, recorder.pause_recording_srv)
    stop_service = rospy.Service('~stop', TriggerBag, recorder.stop_recording_srv)


    bag_status_pub = rospy.Publisher("bag_status", String, queue_size=10)
    bag_name_pub = rospy.Publisher("bag_name", String, queue_size=10)

    # Recording is also stopped on node shutdown. This allows stopping to be done via service call or regular Ctrl-C
    rospy.on_shutdown(recorder.stop_recording_srv)

    # rospy.spin()

    clock_rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        clock_rate.sleep()

        bag_name_msg = String()
        bag_name_msg.data = recorder.bag_name
        bag_name_pub.publish(bag_name_msg)

        bag_status_msg = String()
        bag_status_msg.data = recorder.bag_status
        bag_status_pub.publish(bag_status_msg)
