
# priority queue for OPEN list
import math
import numpy as np
from collision_checker import CollisionChecker
from pqdict import pqdict

class AStarNode(object):
  def __init__(self, pqkey, hval):
    self.pqkey = pqkey
    self.g = math.inf
    self.h = hval
    self.parent_node = None
    self.parent_action = None
    self.closed = False
  
  def fvalue(self, epsilon = 1):
    return self.g + epsilon * self.h


def euclidean(coord1, coord2):
  x1, y1, z1 = coord1
  x2, y2, z2 = coord2

  return math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)


class AStar(object):

  @staticmethod
  def plan(start_coord, goal_coord, boundary, blocks, epsilon = 1):


    # NODE DICT to maintain coord => AStar mapping
    # OPEN to maintain OPEN nodes
    node_dict = dict()
    collision_dict = dict()
    OPEN = pqdict()

    numofdirs = 27

    [dX,dY,dZ] = np.meshgrid([-1,0,1],[-1,0,1],[-1,0,1])
    dR = np.vstack((dX.flatten(),dY.flatten(),dZ.flatten()))
    dR = dR / 3

    start_key = tuple(start_coord)
    goal_key = tuple(goal_coord)

    
    start_node = AStarNode(start_key, euclidean(start_key, goal_key))
    start_node.g = 0

    node_dict[start_key] = start_node
    collision_dict[start_key] = False

    OPEN[start_node] = start_node.fvalue(epsilon)

    pr = 0

    exit_node = None

    while len(OPEN) > 0:

      curr_node, curr_value = OPEN.popitem()
      curr_node.closed = True

      # print(curr_node.pqkey)
      

      if sum((np.array(curr_node.pqkey) -  goal_coord)**2) <= 0.1:
        print("Completing....")
        exit_node = curr_node
        break
      
      # if pr < 100:
      #   print("Debugging", curr_node.pqkey, goal_coord, curr_node.fvalue(epsilon), sum((np.array(curr_node.pqkey) -  goal_coord)**2))
      #   pr = pr + 1

      for k in range(numofdirs):

      
        next_key = tuple(np.array(curr_node.pqkey) + dR[:,k])

        if next_key not in collision_dict:
          collision_dict[next_key] = CollisionChecker.check_path(np.array([next_key]), boundary, blocks)
        

        # CHECK FOR COLLISION
        if collision_dict[next_key]:
          continue

        # CHECK IF NEXT NODE IS CLOSED
        if next_key in  node_dict and node_dict[next_key].closed == True:
          continue

        # INITIALIZE NEXT NODE
        if next_key in node_dict:
          next_node =  node_dict[next_key]
        else:
          next_node =  AStarNode(next_key,  euclidean(next_key, goal_key))
          node_dict[next_key] = next_node

        
        cij = euclidean(curr_node.pqkey, next_key)

        if next_node.g > (curr_node.g + cij):
          next_node.g = curr_node.g + cij
          next_node.parent_node = curr_node

          OPEN[next_node] = next_node.fvalue(epsilon)

    

    path = [exit_node.pqkey]
    path_node = exit_node

    while path_node.parent_node != None:
      path.append(path_node.parent_node.pqkey)
      path_node = path_node.parent_node

    path.reverse()

    return  np.array(path)

    



        

















