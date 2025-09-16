import numpy as np

class CollisionChecker:

    @staticmethod
    def check_path_boundary(planned_path, boundary):

        pathx = planned_path[:, 0][:, np.newaxis]
        pathy = planned_path[:, 1][:, np.newaxis]
        pathz = planned_path[:, 2][:, np.newaxis]

        xmin, ymin, zmin, xmax, ymax, zmax , *_  = boundary[0]

        is_inside = (pathx > xmin) & (pathx < xmax) & (pathy > ymin) & (pathy < ymax)  & (pathz > zmin) & (pathz < zmax)
        has_False = np.any(~is_inside)
        # print(has_False)
        return has_False

    @staticmethod
    def check_boundary(start_node, end_node, boundary):

        xmin, ymin, zmin, xmax, ymax, zmax , *_  = boundary[0]

        sx, sy, sz = start_node.x, start_node.y, start_node.z
        ex, ey, ez = end_node.x, end_node.y, end_node.z

        is_start_inside = (sx > xmin) & (sx < xmax) & (sy > ymin) & (sy < ymax) & (sz > zmin) & (sz < zmax)
        is_end_inside = (ex > xmin) & (ex < xmax) & (ey > ymin) & (ey < ymax)  & (ez > zmin) & (ez < zmax)

        return ~(is_start_inside & is_end_inside)

    @staticmethod
    def check_path_blocks(planned_path, blocks):
        pathx = planned_path[:, 0][:, np.newaxis]
        pathy = planned_path[:, 1][:, np.newaxis]
        pathz = planned_path[:, 2][:, np.newaxis]

        xmin = blocks[:, 0]
        ymin = blocks[:, 1]
        zmin = blocks[:, 2]

        xmax = blocks[:, 3]
        ymax = blocks[:, 4]
        zmax = blocks[:, 5]

        collision_matrix = (pathx >= xmin) & (pathx <= xmax) & (pathy >= ymin) & (pathy <= ymax)  & (pathz >= zmin) & (pathz <= zmax)

        has_True = np.any(collision_matrix)
        indices = np.argwhere(collision_matrix)
        return has_True

    @staticmethod
    def check_blocks(start_node, end_node, blocks):
 
        start = np.array([start_node.x, start_node.y, start_node.z])
        end = np.array([end_node.x, end_node.y, end_node.z])
        dir = end - start

        for block in blocks:
            mins = np.array(block[0:3])
            maxs = np.array(block[3:6])


            intersect = True
            for i in range(3):
                if abs(dir[i]) < 1e-6:
                    if start[i] < mins[i] or start[i] > maxs[i]:
                        return False  
                else:
                    t1 = (mins[i] - start[i]) / dir[i]
                    t2 = (maxs[i] - start[i]) / dir[i]

                    if max(t1,t2) <= 0 or min(t1,t2) > 1:
                        intersect = False

            if intersect:
                return True

        return False

    @staticmethod
    def check_path(planned_path, boundary, blocks):
        is_collision = CollisionChecker.check_path_boundary(planned_path, boundary) | CollisionChecker.check_path_blocks(planned_path, blocks)
        return is_collision

    @staticmethod
    def check(start_node, end_node, boundary, blocks):
        is_collision = CollisionChecker.check_boundary(start_node, end_node, boundary) | CollisionChecker.check_blocks(start_node, end_node, blocks)
        return is_collision



      
