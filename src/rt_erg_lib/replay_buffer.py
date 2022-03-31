#!/usr/bin/env python
import random
#import time
#import rospy

"""
A class to sample trajectory points from the agents that serve as a memory
of the regions that have been explored.
"""

class ReplayBuffer(object):
    def __init__(self, capacity):
        self.capacity = capacity
        self.buffer = []
        self.position = 0
        #self.start_time = time.time()

    def reset(self):
        self.buffer = []
        self.position = 0

    def push(self, state):
        if len(self.buffer) < self.capacity:
            self.buffer.append(None)
        self.buffer[self.position] = state
        self.position = (self.position + 1) % self.capacity
        #with open("replay_buffer_timestamps.txt", "a") as f:
        #    f.write("replay buffer time is: {}\n".format(time.time() - self.start_time))

    def sample(self, batch_size):
        if batch_size == -1:
            states = self.buffer[:batch_size]
        else:
            states = random.sample(self.buffer, batch_size)
        return states

    def __len__(self):
        return len(self.buffer)
