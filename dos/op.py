import datetime as dt
from time import sleep
from threading import Thread
from Queue import Queue

import dos.graph as graph


def make_op(functions, output_types=None, output_names=None,
            freq=None, sync=False, name=None):
    op = Op(freq=freq, sync=sync, name=name)
    op.register(functions, output_types, output_names)
    return op


class Op(object):
    def __init__(self, freq=None, sync=False, name=None):
        self.graph = graph.get_current_graph()
        self.name = name if name is not None else "default_op"
        self.in_num = 0
        self.out_num = 0
        self.inputs = None
        self.outputs = None
        self.output_types = None
        self.output_names = None
        self.freq = freq
        self.sync = sync

        if sync:  # if sync, must set a freq
            assert self.freq is not None

        if freq:
            self.last_tf = dt.datetime.now()
            self.pull_queues = list()

        self.graph.add_op(self.name, self)  # Add op to graph

    def __call__(self, *in_stream):
        """
        Set input stream info
        Send callback function to input stream (No sender/receiver initiation)
        """
        if in_stream:
            self.inputs = in_stream
            self.in_num = len(self.inputs)
            for i in range(self.in_num):
                self.inputs[i].send_to(self._callback, i)
                if self.freq:
                    self.pull_queues.append(Queue())

        if self.out_num:
            # change the number of outputs based on #inputs
            # (don't change on sync)
            if not self.sync and self.in_num and self.out_num != self.in_num:
                self.out_num = self.in_num
                self.output_types = tuple(self.out_num*[self.output_types[0]])
            if self.output_names is None:
                self.output_names = tuple(['{}_out_{}'.format(self.name, i)
                                           for i in range(self.out_num)])

            stream_type = self.graph.stream_type()
            self.outputs = tuple([stream_type(name, t) for name, t in
                                  zip(self.output_names, self.output_types)])
            return self.outputs[0] if self.out_num == 1 else self.outputs

    def register(self, funcs, output_types=None, output_names=None):
        """
        Set execution functions and output stream info
        """
        self.funcs = funcs if type(funcs) is tuple else tuple([funcs])

        if output_types:
            if type(output_types) is not tuple:
                output_types = tuple([output_types])
            self.out_num = len(output_types)
            self.output_types = output_types
            self.output_names = output_names
            assert len(self.funcs) == self.out_num

        if self.sync:  # pulling mode: can only have 1 func, at most 1 output
            assert len(self.funcs) == 1 and self.out_num <= 1

    def _callback(self, data, i):
        if self.freq:  # freq restriction - save to a queue first
            self.pull_queues[i].put(data)
        else:  # No freq restriction - real-time sending outputs
            self._execute_and_send(data, i)

    def _execute_and_send(self, data, i):
        if self.sync:
            result = self.funcs[i](*data)
        elif self.in_num == len(self.funcs):
            result = self.funcs[i](data)
        else:
            result = self.funcs[0](data, i)
        self._do_work(i, result)

    def _execute_and_send_by_freq(self, data_queue, i):
        while True:
            self._wait()
            if self.sync:
                data = self._get_all_inputs()  # data is a list here
            else:
                data = data_queue.get()
            self._execute_and_send(data, i)

    def _get_all_inputs(self):
        inputs = list()
        for q in self.pull_queues:
            inputs.append(q.get(block=True))
        return inputs

    def _do_work(self, i, result=None):
        if self.inputs is None:  # Generator mode
            results = self.funcs[i]()
            for r in results:
                self.outputs[i].add(r)
        elif self.outputs:
            self.outputs[i].add(result)

    def _wait(self):
        """
        If user specified frequency, wait to pull data from input streams
        """
        if self.freq:
            gap = 1 / self.freq
            if (dt.datetime.now() - self.last_tf).seconds < gap:
                sleep(gap - (dt.datetime.now() - self.last_tf).seconds)
            self.last_tf = dt.datetime.now()

    def init(self):
        """
        User can overwrite
        """
        pass

    def spin(self):
        """
        User can overwrite
        """
        self.graph.spin_op()

    def run(self):
        try:
            # self.init()
            if self.inputs:
                if self.freq:  # Pulling based on frequency
                    if self.sync:
                        t = Thread(target=self._execute_and_send_by_freq,
                                   args=(None, 0))
                        t.setDaemon(True)
                        t.start()
                    else:
                        for i in range(self.in_num):
                            t = Thread(
                                target=self._execute_and_send_by_freq,
                                args=(self.pull_queues[i], i))
                            t.setDaemon(True)
                            t.start()
                    while True:
                        pass
                else:
                    self.spin()
            else:  # Generator
                for i in range(len(self.funcs)):
                    t = Thread(target=self._do_work, args=(i,))
                    t.setDaemon(True)
                    t.start()
                while True:
                    pass

        except EOFError as e:
            print("Executor Op {} terminating because of {}".format(
                self.name, e))

    # Only invoked by graph
    def _run_process(self):

        self.graph.init_op(self.name)   # Initiate Node

        if self.outputs:
            for out in self.outputs:
                out.init_sender()       # Initiate Senders

        if self.inputs:
            for inp in self.inputs:
                inp.init_receiver()     # Initiate Receivers

        self.init()
        self.graph.sync()               # Sync initiation
        self.run()
