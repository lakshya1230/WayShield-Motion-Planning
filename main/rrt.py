from collision_checker import CollisionChecker
import random
import math
import numpy as np


class Node:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.parent = None
        self.cost = 0.0

class RRT:
    def __init__(self, start, goal, blocks, boundary, max_steer_distance=5.0, goal_sample_rate=5, max_iter=500):
            """
            Initialize RRT
            
            Parameters:
                start (tuple): Start position (x, y)
                goal (tuple): Goal position (x, y)
                blocks (list): List of obstacles as (x, y, radius)
                boundary (numpy array): Area bounds as (min_x, max_x, min_y, max_y)
                max_steer_distance (float): Maximum extension length for new nodes
                goal_sample_rate (int): Percentage chance to sample goal instead of random point
                max_iter (int): Maximum number of iterations
            """
            self.start = Node(*start)
            self.goal = Node(*goal)
            self.blocks = blocks
            self.boundary = boundary
            self.max_steer_distance = max_steer_distance
            self.goal_sample_rate = goal_sample_rate
            self.max_iter = max_iter
            self.vertices = [self.start]

    
    # Implement with Goal Node Sampling
    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = Node(
                random.uniform(self.boundary[0][0], self.boundary[0][3]),
                random.uniform(self.boundary[0][1], self.boundary[0][4]),
                random.uniform(self.boundary[0][2], self.boundary[0][5])
            )
            
        else:
            rnd = Node(self.goal.x, self.goal.y, self.goal.z)
        return rnd

    @staticmethod
    def get_nearest_node(vertices, node):

        """Find the nearest node in node_list to the given node"""

        nearest_node = vertices[0]
        min_dist = float('inf')
        for vertex in vertices:
            dist = (vertex.x - node.x)**2 + (vertex.y - node.y)**2
            if dist < min_dist:
                min_dist = dist
                nearest_node = vertex
        return nearest_node

    
    def calc_dist(self, from_node, to_node):
        """Calculate Euclidean distance between two nodes"""
        return math.sqrt((from_node.x - to_node.x)**2 + (from_node.y - to_node.y)**2)

    def steer(self, from_node, to_node):
        
        # Calculate direction vector
        dir = np.array([
            to_node.x - from_node.x,
            to_node.y - from_node.y,
            to_node.z - from_node.z
        ])
        
        dist = np.linalg.norm(dir)
        
        if dist <= self.max_steer_distance:
            new_node = Node(to_node.x, to_node.y, to_node.z)
        else:
            unit_dir = dir / dist
            scaled_dir = unit_dir * self.max_steer_distance
            
            new_node = Node(
                from_node.x + scaled_dir[0],
                from_node.y + scaled_dir[1],
                from_node.z + scaled_dir[2]
            )
        
        new_node.parent = from_node
        new_node.cost = from_node.cost + min(dist, self.max_steer_distance)
        return new_node

    def plan(self):
        for i in range(self.max_iter):
            
            # Random Sampling
            rnd_node = self.get_random_node()
            
            # GET Nearest Node for the sampled node
            nearest_node = self.get_nearest_node(self.vertices, rnd_node)
            
            # Steer from NEAREST => RANDOM
            new_node = self.steer(nearest_node, rnd_node)
            
            if CollisionChecker.check(nearest_node, new_node, self.boundary, self.blocks) == False:
                # print("Adding new node")
                # self.display(new_node)
                # print("Distance : ", self.calc_dist(new_node, self.goal))
                self.vertices.append(new_node)  
                
                # Check if goal is reached
                if self.calc_dist(new_node, self.goal) <= self.max_steer_distance:
                    if CollisionChecker.check(new_node, self.goal, self.boundary, self.blocks) == False:
                        self.goal.parent = new_node
                        self.goal.cost = new_node.cost + self.calc_dist(new_node, self.goal)
                        self.vertices.append(self.goal)
                        return self.final_path()

        return None 

    
    """Construct the final path from goal to start"""
    def final_path(self):
        path = []
        node = self.goal

        while node.parent is not None:
            path.append([node.x, node.y, node.z])
            node = node.parent

        path.append([self.start.x, self.start.y, self.start.z])
        path.reverse()  
        path = np.array(path)
        return path

    def display(self, node):
        print(f"Node : x={node.x}, y={node.y}, z={node.z}, parent={node.parent}")