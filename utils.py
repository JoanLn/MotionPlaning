# utils.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to Clemson University and the authors.
# 
# Author: Ioannis Karamouzas (ioannis@g.clemson.edu)

import math 
import numpy as np
#import itertools
#import heapq

""" Returns the oriented bounding box of a given configuration q """
def getRobotPlacement(q, robot_width, robot_height):
    
    vertices = [# coordinates assuming theta is 0 
        [q[0] - robot_width*0.5, q[1] - robot_height*0.5], # bottom left
        [q[0] + robot_width*0.5, q[1] - robot_height*0.5], # bottom right ....
        [q[0] + robot_width*0.5, q[1] + robot_height*0.5],
        [q[0] - robot_width*0.5, q[1] + robot_height*0.5],
    ]
    ctheta = math.cos(q[2])
    stheta = math.sin(q[2])
    points = []
    for x, y in vertices:
        # place center of obb at the origin
        x -= q[0] 
        y -= q[1] 
        # apply rotation
        x_new = x * ctheta - y * stheta
        y_new = x * stheta + y * ctheta 
        points.append([x_new + q[0], -y_new + q[1]]) # the neg sign is due to the fact that the y axis is flipped in canvas

    return points

class Value():
    """A helper class for adding f & g values to your PriorityQueue """

    def __init__(self, f, g):
        self.g = g
        self.f = f

class OrderedSet:
    """ An ordered list of elements """
    
    def __init__(self):
        self._container = []
    
    def add(self, item):
        if item in self._container:
            self._container.append(item)
        else:
            self._container.append(item)

    def has(self, item):
        return self._container.__contains__

    def remove(self, item):
        if item in self._container:
            self._container.remove(item)
    
    def clear(self):
        self._container.clear()
    
    def __contains__(self, item):
        return self._container.__contains__(item)

    def __len__(self):
        return self._container.__len__()
    
    def __iter__(self):
        return self._container.__iter__()
    
    def pop(self, last=True):
        if last:
            e = self._container.pop()
        else:
            e = self._container.pop(0)
        return e

class PriorityQueueValue:
    """
        A Queue in which the minimum (or maximum) element (as determined by f and
        order) is returned first.
    """
    def __init__(self, order=min, f=lambda v:v):
        if order == min or order == "min":
            self.order = min
        elif order == max or order == "max":
            self.order = max
        else:
            raise KeyError("order must be min or max")
        self.f = f

        self._dict = {}
  
    def get(self, item):
        return self._dict.__getitem__(item)

    def put(self, item, value):
        if item not in self._dict:
            self._dict[item] = value
        else:
            self._dict[item] = value

    def has(self, item):
        return self._dict.__contains__(item)

    def remove(self, item):
        if item in self._dict:
            del self._dict[item]

    def pop(self):
        if len(self._dict) > 0:
            tar = self.order(self._dict, key=lambda k: self.f(self._dict.get(k)))
            val = self._dict[tar]
            del self._dict[tar]
            return tar, val
        raise IndexError("pop from empty priority queue")

    def __iter__(self):
        return self._dict.__iter__()
    
    def __contains__(self, item):
        return self._dict.__contains__(item)

    def __len__(self):
        return self._dict.__len__()

    def __getitem__(self, key):
        return self._dict.__getitem__(key)
    
    def __setitem__(self, key, value):
        return self._dict.__setitem__(key, value)
    
    def __delitem__(self, key):
        return self._dict.__delitem__(key)

class PriorityQueue:
    def __init__(self):
        self.pq = []
        self.entry_finder = {}
        self.counter = itertools.count()

    def push(self, item):
        # check for duplicate
        if item in self.entry_finder: return
        count = next(self.counter)
        # use x-coordinate as a primary key (heapq in python is min-heap)
        entry = [item.x, count, item]
        self.entry_finder[item] = entry
        heapq.heappush(self.pq, entry)

    def remove_entry(self, item):
        entry = self.entry_finder.pop(item)
        entry[-1] = 'Removed'

    def pop(self):
        while self.pq:
            priority, count, item = heapq.heappop(self.pq)
            if item != 'Removed':
                del self.entry_finder[item]
                return item
        raise KeyError('pop from an empty priority queue')

    def top(self):
        while self.pq:
            priority, count, item = heapq.heappop(self.pq)
            if item !='Removed':
                del self.entry_finder[item]
                self.push(item)
                return item
        raise KeyError('top from an empty priority queue')

    def empty(self):
        return not self.pq

class Point:
   x = 0.0
   y = 0.0
   
   def __init__(self, x, y):
       self.x = x
       self.y = y

class Event:
    x = 0.0
    p = None
    a = None
    valid = True
    
    def __init__(self, x, p, a):
        self.x = x
        self.p = p
        self.a = a
        self.valid = True

class Arc:
    p = None
    pprev = None
    pnext = None
    e = None
    s0 = None
    s1 = None
    
    def __init__(self, p, a=None, b=None):
        self.p = p
        self.pprev = a
        self.pnext = b
        self.e = None
        self.s0 = None
        self.s1 = None

class Segment:
    start = None
    end = None
    done = False
    
    def __init__(self, p):
        self.start = p
        self.end = None
        self.done = False

    def finish(self, p):
        if self.done: return
        self.end = p
        self.done = True      
