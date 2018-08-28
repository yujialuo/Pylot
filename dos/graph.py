from __future__ import print_function
import signal
import six
import sys
import os
import time
import rospy
from multiprocessing import Process, Manager
import subprocess

from dos.streams.ros_stream import RosStream


STREAM_TABLE = {
    'ros': 'RosStream'
}


class Synchronizer(object):
    def __init__(self):
        self.n = 0
        m = Manager()
        self.count = m.Value('i', 0)
        self.barrier = m.Semaphore(0)

    def set_total(self, n):
        self.n = n

    def wait(self):
        self.count.value += 1
        if self.count.value == self.n:
            self.barrier.release()
            print("======================= Sync =============================")
        self.barrier.acquire()
        self.barrier.release()


def sigint_handler(signal, frame):
    try:
        sys.exit(0)
    except SystemExit:
        os._exit(0)


class Graph(object):
    def __init__(self, framework='ros'):
        self.ops = dict()             # All Ops
        self.framework = framework    # ROS, Ray, etc
        self.synchronizer = Synchronizer()

    def stream_type(self):
        return eval(STREAM_TABLE[self.framework])

    def add_op(self, name, op):
        self.ops[name] = op

    def info(self):
        for name, op in six.iteritems(self.ops):
            print("[Op] {}".format(name))
            inputs = [x.name for x in op.inputs]
            outputs = [x.name for x in op.outputs]
            print("inputs: {}".format(inputs))
            print("outputs: {}".format(outputs))

    def init_op(self, name):
        if self.framework == 'ros':
            rospy.init_node(name, anonymous=True)
            print("[ROS Node] {}".format(name))

    def spin_op(self):
        if self.framework == 'ros':
            rospy.spin()

    def run(self):

        if self.framework == 'ros':
            # Run roscore in a different process
            subprocess.Popen('roscore')
            # wait a bit to be sure the roscore is really launched
            time.sleep(1)

        signal.signal(signal.SIGINT, sigint_handler)

        self.synchronizer.set_total(len(self.ops))

        # Run all ops
        print("=========== Initiate and Run All Operators ===============")
        procs = list()
        for k in self.ops:
            op = self.ops[k]
            p = Process(target=op._run_process)
            procs.append(p)
            p.start()
        for p in procs:
            p.join()

    def sync(self):
        self.synchronizer.wait()


default_graph = Graph()


def get_current_graph():
    return default_graph
