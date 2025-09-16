import numpy as np
from astar import AStar
from rrt import RRT

class AStarPlanner:
  __slots__ = ['boundary', 'blocks']
  

  def __init__(self, boundary, blocks):
    self.boundary = boundary
    self.blocks = blocks

  def plan(self,start,goal):
    return AStar.plan(start, goal, self.boundary, self.blocks)


class RRTPlanner:
  __slots__ = ['boundary', 'blocks']
  

  def __init__(self, boundary, blocks):
    self.boundary = boundary
    self.blocks = blocks

  def plan(self,start,goal):
    rrt = RRT(start, goal, self.blocks, self.boundary, max_steer_distance=0.5, goal_sample_rate=10, max_iter=100000)
    path = rrt.plan()
    return path