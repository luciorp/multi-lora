# -*- coding: utf-8 -*-
"""
Created on Tue Apr 9 16:29:23 2019

@author: lucio
"""

import thread6, Queue
import subprocess as sbp

class Companion(object):
    "A companion process manager"
    def __init__(self, cmdline):
        "Start the companion process"
        self.companion= sbp.Popen(
            cmdline, shell=False,
            stdin=sbp.PIPE,
            stdout=sbp.PIPE)
        self.putque= Queue.Queue()
        self.getque= Queue.Queue()
        thread6.start_new_thread(self._sender, (self.putque,))
        thread6.start_new_thread(self._receiver, (self.getque,))

    def _sender(self, que):
        "Actually sends the data to the companion process"
        while 1:
            datum= que.get()
            if datum is Ellipsis:
                break
            self.companion.stdin.write(datum)
            if not datum.endswith('\n'):
                self.companion.stdin.write('\n')

    def _receiver(self, que):
        "Actually receives data from the companion process"
        while 1:
            datum= self.companion.stdout.readline()
            que.put(datum)

    def close(self):
        self.putque.put(Ellipsis)

    def send(self, data):
        "Schedule a long line to be sent to the companion process"
        self.putque.put(data)

    def recv(self):
        "Get a long line of output from the companion process"
        return self.getque.get()
