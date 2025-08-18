"""
A* grid planning
author: Atsushi Sakai(@Atsushi_twi)
        Nikos Kanargias (nkana@tee.gr)
See Wikipedia article (https://en.wikipedia.org/wiki/A*_search_algorithm)
"""

import time
from obstacle_list import *
import math
import numpy as np
import matplotlib.pyplot as plt
from find_intersect_line import *

# CHANGED HERE: Added for MC randomness, infinity in costs, and path storage.
import random
from math import inf
import pickle

global show_animation
show_animation = False

# CHANGED HERE:
use_trained_path = True  # Set to False for first run (compute and save path). Set to True for subsequent runs (load and reuse path).


# The `AStarPlanner` class is initialized with start and goal positions, obstacle information,
# resolution, and end point for A* path planning in a grid map.
class AStarPlanner:

    def __init__(self, start, goal, obstacleList_rect, randArea):
        """
        This function initializes a grid map for A* path planning with specified start, goal, obstacles,
        resolution, and end point.

        :param start: The `start` parameter represents the starting position for path planning. It could be
        a tuple containing the x and y coordinates of the starting point in the grid map
        :param goal: The `goal` parameter in the `__init__` function represents the goal position in the
        grid map for A* path planning. It is the destination point that the algorithm will try to reach from
        the start position
        :param obstacleList_rect: The `obstacleList_rect` parameter is a list of rectangles representing
        obstacles in the grid map. Each rectangle is defined by its coordinates in the form of (x, y, width,
        height), where (x, y) is the bottom-left corner of the rectangle, and width and height represent
        :param randArea: The `randArea` parameter represents the area in which the random points will be
        generated. It is a list containing the minimum and maximum coordinates of the area in the form
        `[min_x, max_x, min_y, max_y]`
        :param resolution: The `resolution` parameter in the `__init__` function represents the grid
        resolution, which is the distance between each grid cell in the grid map. It determines the level of
        detail in the map representation and affects the accuracy of the path planning algorithm. A smaller
        resolution value results in a finer grid
        :param end_point: The `end_point` parameter in the `__init__` function is used to specify the end
        point or destination for the A* path planning algorithm. This point represents the goal that the
        algorithm will try to reach from the starting point
        """

        """
        Initialize grid map for a star planning
        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]
        """

        self.start = start
        self.goal = goal
        x_min, x_max, y_min, y_max = randArea

        # Calculate side lengths
        width = x_max - x_min
        height = y_max - y_min

        # Take minimum side length
        min_side = min(width, height)

        # Define resolution as 1% of minimum side
        resolution = min_side * 0.03
        self.resolution = resolution

        self.min_x = x_min
        self.min_y = y_min
        self.max_x = x_max
        self.max_y = y_max
        self.x_width = (self.max_x - self.min_x) / self.resolution
        self.y_width = (self.max_y - self.min_y) / self.resolution

        self.obstacleList_rect = obstacleList_rect

        # self.calc_obstacle_map(obstacle_x, obstacle_y)
        self.motion = self.get_motion_model() * resolution

    # CHANGED HERE:
    def monte_carlo_planning(self, current, goal, all_obstacles):
        """
        Pure Monte Carlo pathfinding: Run many random simulations (episodes) from current to goal, 
        select the best collision-free path with min cost. No model, just trials.
        """
        # CHANGED HERE: New method for pure MC when switching due to new obstacle.
        print(f"MC initialized at: {current}")  # Debug: Confirm start point
        best_path = None
        min_cost = inf
        trials = 5000  # Increased from 1000 for better chance of finding path.
        max_steps = 500  # Increased from 200 to allow longer paths.
        goal_dist_threshold = self.resolution * 2  # Close enough to goal.
        straight_line_cost = math.hypot(goal[0] - current[0], goal[1] - current[1])  # For early stopping threshold.
        
        for trial in range(trials):
            path = [current[:]]  # Start with current position.
            pos = current[:]
            cost = 0.0
            for step in range(max_steps):
                # NEW: Bias angle towards goal for better exploration.
                goal_angle = math.atan2(goal[1] - pos[1], goal[0] - pos[0])  # Direction to goal.
                angle_noise = random.gauss(0, math.pi / 4)  # Gaussian noise, std dev 45 degrees.
                angle = goal_angle + angle_noise  # Biased random direction.
                
                # NEW: Adaptive step size based on distance to goal.
                dist_to_goal = math.hypot(goal[0] - pos[0], goal[1] - pos[1])
                max_step = min(self.resolution * 10, dist_to_goal / 2)  # Cap at half remaining distance.
                step = random.uniform(self.resolution, max_step)  # Larger when far, smaller when close.
                
                dx = step * math.cos(angle)
                dy = step * math.sin(angle)
                new_pos = [pos[0] + dx, pos[1] + dy]
                
                # Check if line from pos to new_pos is collision-free.
                if self.is_collision_not_detected_rect(pos, new_pos, all_obstacles) and self.verify_node(self.Node(new_pos[0], new_pos[1], 0, -1), self.Node(pos[0], pos[1], 0, -1)):
                    path.append(new_pos)
                    cost += step
                    pos = new_pos
                    
                    # If close to goal, check if better.
                    if math.hypot(pos[0] - goal[0], pos[1] - goal[1]) < goal_dist_threshold:
                        if cost < min_cost:
                            min_cost = cost
                            best_path = path
                        # NEW: Early stopping if path is reasonably good (e.g., <1.5x straight-line).
                        if cost < 1.5 * straight_line_cost:
                            print(f"Early stopping: Good path found in trial {trial} with cost {cost}")
                            # NEW: Prune the path before returning.
                            if best_path and self.is_collision_not_detected_rect(best_path[-1], goal, all_obstacles):
                                best_path.append(goal)
                            pruned_best_path = self.prune_path_modified(best_path, all_obstacles)
                            return (best_path, pruned_best_path)
                            # return pruned_best_path if pruned_best_path else best_path    # for pruned path!
                            # return best_path
                        break  # End this trial.
                else:
                    break  # Invalid move, end trial.
        
        if best_path:
            # NEW: Prune the final best path.
            pruned_best_path = self.prune_path_modified(best_path, all_obstacles)
            return (best_path, pruned_best_path)
            # return pruned_best_path if pruned_best_path else best_path    # for pruned path!
            # return best_path
        else:
            print("No path found with MC after all trials.")
            return None  # Or fallback to another method, e.g., self.planning() with updated obstacles.
    
    # def monte_carlo_planning(self, current, goal, all_obstacles):
    #     """
    #     Pure Monte Carlo pathfinding: Run many random simulations (episodes) from current to goal, 
    #     select the best collision-free path with min cost. No model, just trials.
    #     """
    #     # CHANGED HERE: New method for pure MC when switching due to new obstacle.
    #     best_path = None
    #     min_cost = inf
    #     trials = 1000  # Adjust if too slow (number of episodes/trials).
    #     max_steps = 200  # Max steps per trial to avoid infinite loops.
    #     for _ in range(trials):
    #         path = [current[:]]  # Start with current position.
    #         pos = current[:]
    #         cost = 0.0
    #         for _ in range(max_steps):
    #             # Random direction and step size (like random action in MC).
    #             angle = random.uniform(0, 2 * math.pi)
    #             step = random.uniform(self.resolution, self.resolution * 5)  # Small to medium steps.
    #             dx = step * math.cos(angle)
    #             dy = step * math.sin(angle)
    #             new_pos = [pos[0] + dx, pos[1] + dy]
    #             # Check if line from pos to new_pos is collision-free.
    #             if self.is_collision_not_detected_rect(pos, new_pos, all_obstacles) and self.verify_node(self.Node(new_pos[0], new_pos[1], 0, -1), self.Node(pos[0], pos[1], 0, -1)):
    #                 path.append(new_pos)
    #                 cost += step
    #                 pos = new_pos
    #                 # If close to goal, check if better.
    #                 if math.hypot(pos[0] - goal[0], pos[1] - goal[1]) < self.resolution * 2:
    #                     if cost < min_cost:
    #                         min_cost = cost
    #                         best_path = path
    #                     break  # End this trial.
    #             else:
    #                 break  # Invalid move, end trial.
    #     return best_path
    
    # CHANGED HERE:
    def is_collision_not_detected_rect(self, node, nearNode, obstacleList_rect):
        # CHANGED HERE: Added flexibility for node as list [x,y] (for MC inputs).
        p1 = Point(node.x, node.y) if hasattr(node, 'x') else Point(node[0], node[1])
        q1 = Point(nearNode.x, nearNode.y) if hasattr(nearNode, 'x') else Point(nearNode[0], nearNode[1])
        collision = 0
        for rect in obstacleList_rect:
            for i in range(4):
                p2_x = rect[i, 0]
                p2_y = rect[i, 1]
                q2_x = rect[(i + 1) % 5, 0]  # Close the rectangle loop.
                q2_y = rect[(i + 1) % 5, 1]
                p2 = Point(p2_x, p2_y)
                q2 = Point(q2_x, q2_y)
                if doIntersect(p1, q1, p2, q2):
                    collision += 1
                    return False
        return True  # Safe.

    # The Node class represents a node in a grid with attributes for position, cost, and parent index.
    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return (
                str(self.x)
                + ","
                + str(self.y)
                + ","
                + str(self.cost)
                + ","
                + str(self.parent_index)
            )

    def planning(self):
        """
        The `planning` function implements the A* path search algorithm to find a path from a start
        position to a goal position while avoiding obstacles, and returns the final path and a pruned
        version of the path.
        :return: The `planning` method returns a list containing two elements: the original path `path`
        and the pruned path `p_path`.
        """
        """
        A star path search
        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]
        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        start_node = self.Node(self.start[0], self.start[1], 0.0, -1)
        goal_node = self.Node(self.goal[0], self.goal[1], 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node
        path = []
        if self.is_collision_not_detected_rect(
            start_node, goal_node, self.obstacleList_rect
        ):
            path.append(self.goal)
            path.append(self.start)

            return [path, path]

        while 1:

            if len(open_set) == 0:
                # print("Open set is empty..")
                break

            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost
                + self.calc_heuristic(goal_node, open_set[o]),
            )
            current = open_set[c_id]
            if self.is_collision_not_detected_rect(
                goal_node, current, self.obstacleList_rect
            ):
                goal_node.parent_index = c_id
                goal_node.cost = current.cost + self.calc_heuristic(goal_node, current)
                closed_set[c_id] = current
                break

            # show graph
            if show_animation:  # pragma: no cover
                for i, _ in enumerate(self.motion):
                    plt.plot(
                        self.calc_grid_position(
                            current.x + self.motion[i][0], self.min_x
                        ),
                        self.calc_grid_position(
                            current.y + self.motion[i][1], self.min_y
                        ),
                        "xr",
                    )
                plt.plot(
                    self.calc_grid_position(current.x, self.min_x),
                    self.calc_grid_position(current.y, self.min_y),
                    "xc",
                )

                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect(
                    "key_release_event",
                    lambda event: [exit(0) if event.key == "escape" else None],
                )
                # if len(closed_set.keys()) % 10 == 0:
                #     plt.pause(0.001)
            dist = self.calc_heuristic(goal_node, current)

            if (
                current.x == goal_node.x
                and current.y == goal_node.y
                or dist <= self.resolution
            ):

                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(
                    current.x + self.motion[i][0],
                    current.y + self.motion[i][1],
                    current.cost + self.motion[i][2],
                    c_id,
                )
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node, current):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        path = self.calc_final_path(goal_node, closed_set)
        p_path = self.prune_path_modified(path, self.obstacleList_rect)

        return [path, p_path]

    def calc_final_path(self, goal_node, closed_set):
        """
        The function `calc_final_path` generates the final path from the goal node by tracing back
        through the closed set of nodes.

        :param goal_node: The `goal_node` parameter in the `calc_final_path` method represents the node
        that is the goal of the pathfinding algorithm. It is the destination node that the algorithm has
        determined as the target to reach
        :param closed_set: The `closed_set` parameter in the `calc_final_path` method is a set that
        contains all the nodes that have been visited and evaluated during the search process. It is
        typically used in pathfinding algorithms like A* to keep track of the nodes that have already
        been explored
        :return: The function `calc_final_path` returns a list of coordinates representing the final path
        from the starting node to the goal node. Each coordinate is a list containing the x and y
        coordinates of a node in the path.
        """
        # generate final course
        path = []
        path.append([goal_node.x, goal_node.y])
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            path.append([n.x, n.y])

            parent_index = n.parent_index

        return path

    @staticmethod
    def calc_heuristic(n1, n2):
        """
        The function calculates the heuristic distance between two points using the Euclidean distance
        formula.

        :param n1: The `calc_heuristic` function you provided calculates the Euclidean distance between two
        points `n1` and `n2` using the formula `d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)`, where `w` is
        :param n2: It seems like you were about to provide some information about the parameter `n2` but the
        message got cut off. Could you please provide more details or let me know how I can assist you
        further?
        :return: The function `calc_heuristic` returns the Euclidean distance between two points `n1` and
        `n2` multiplied by the weight `w`.
        """
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_position):
        """
        The function `calc_grid_position` calculates the grid position based on the index and minimum
        position provided.

        :param index: The `index` parameter in the `calc_grid_position` function represents the position
        of an element within a grid or array. It is used to calculate the final position within the grid
        by adding it to the `min_position` parameter
        :param min_position: The `min_position` parameter in the `calc_grid_position` function represents
        the minimum position value that you want to add to the index to calculate the final position in
        the grid. This parameter allows you to specify the starting position in the grid before adding the
        index value to it
        :return: the calculated grid position, which is the sum of the index and the minimum position.
        """
        """
        calc grid position
        :param index:
        :param min_position:
        :return:
        """
        pos = index + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        """
        The function calculates the index of a position based on a minimum position and resolution.

        :param position: The `position` parameter represents the current position value for which you
        want to calculate the index
        :param min_pos: The `min_pos` parameter represents the minimum position value. It is used in the
        calculation of the index for a given position relative to the minimum position
        :return: The function `calc_xy_index` is returning the index of a position calculated based on
        the input parameters `position`, `min_pos`, and `resolution`. The index is calculated by
        subtracting `min_pos` from `position`, dividing the result by `resolution`, and rounding the
        result.
        """
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        """
        The function calculates the index of a node in a grid based on its x and y coordinates.

        :param node: The `calc_grid_index` function takes a `node` object as a parameter. The function
        calculates and returns the grid index of the node based on its x and y coordinates relative to
        the minimum x and y values stored in the object (`self.min_x` and `self.min_y`) and the
        :return: The function `calc_grid_index` is returning the index of a node in a grid based on its x
        and y coordinates relative to the minimum x and y values of the grid.
        """
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node, current):
        """
        The function `verify_node` checks if a given node is within specified boundaries and does not
        collide with obstacles.

        :param node: The `node` parameter represents a point in a 2D space with coordinates `x` and `y`. The
        function `verify_node` is used to check if this node is within certain boundaries (`min_x`, `min_y`,
        `max_x`, `max_y`) and if it coll
        :param current: It seems like you were about to provide the definition or explanation of the
        `current` parameter but it got cut off. Could you please provide more information about the
        `current` parameter so that I can assist you further with the `verify_node` function?
        :return: The function `verify_node` returns a boolean value - `True` if the conditions for the node
        being verified are met, and `False` otherwise.
        """

        if node.x < self.min_x:
            return False
        elif node.y < self.min_y:
            return False
        elif node.x > self.max_x:
            return False
        elif node.y > self.max_y:
            return False

        # collision check
        if not self.is_collision_not_detected_rect(
            node, current, self.obstacleList_rect
        ):
            # if self.obmap[int(node.x)][int(node.y)]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):
        """
        The function calculates an obstacle map based on given obstacle coordinates and resolution.

        :param ox: The `ox` parameter in the `calc_obstacle_map` function represents a list of x-coordinates
        of obstacles in the environment. These x-coordinates are used to calculate the obstacle map based on
        the robot's resolution and safety radius
        :param oy: It seems like you were about to provide some information about the `oy` parameter in your
        code snippet. Could you please provide more details or let me know how I can assist you further with
        this code?
        """

        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)

        # obstacle map generation
        self.obstacle_map = [
            [False for _ in range(int(self.y_width))] for _ in range(int(self.x_width))
        ]
        for ix in range(int(self.x_width)):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(int(self.y_width)):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obstacle_map[ix][iy] = True
                        break

    # def is_collision_not_detected_rect(self, node, nearNode, obstacleList_rect):
    #     collision = 0

    #     for rect in obstacleList_rect:
    #         for i in range(4):
    #             p1 = Point(node.x, node.y)
    #             q1 = Point(nearNode.x, nearNode.y)
    #             p2_x = rect[i, 0]
    #             p2_y = rect[i, 1]
    #             q2_x = rect[i + 1, 0]
    #             q2_y = rect[i + 1, 1]
    #             p2 = Point(p2_x, p2_y)
    #             q2 = Point(q2_x, q2_y)
    #             if doIntersect(p1, q1, p2, q2):
    #                 collision = collision + 1
    #                 return False

    #     return True  # safe
    def is_collision_not_detected_rect(self, node, nearNode, obstacleList_rect):
        # CHANGED HERE: Added flexibility for node as list [x,y] (for MC inputs).
        p1 = Point(node.x, node.y) if hasattr(node, 'x') else Point(node[0], node[1])
        q1 = Point(nearNode.x, nearNode.y) if hasattr(nearNode, 'x') else Point(nearNode[0], nearNode[1])
        collision = 0
        for rect in obstacleList_rect:
            for i in range(4):
                p2_x = rect[i, 0]
                p2_y = rect[i, 1]
                q2_x = rect[(i + 1) % 4, 0]  # Use modulo to close the rectangle loop correctly (changed from %5 to %4, as rectangles have 4 sides).
                q2_y = rect[(i + 1) % 4, 1]
                p2 = Point(p2_x, p2_y)
                q2 = Point(q2_x, q2_y)
                if doIntersect(p1, q1, p2, q2):
                    collision += 1
                    return False
        return True  # Safe.

    def prune_path_modified(self, path, obstacleList_rect):
        """
        The function `prune_path_modified` removes intermediate points from a path if they do not result in
        a collision with obstacles.

        :param path: The `path` parameter in the `prune_path_modified` function seems to be a list of points
        representing a path. Each point is a tuple of coordinates (x, y)
        :param obstacleList_rect: It looks like the code snippet you provided is a function called
        `prune_path_modified` that takes a path and a list of obstacles represented as rectangles
        (`obstacleList_rect`). The function iterates through the path and removes intermediate points if
        they are not necessary for the path or if they collide with
        :return: The function `prune_path_modified` is returning the pruned version of the input `path` list
        after removing any unnecessary points that do not contribute to the path or collide with obstacles.
        """
        pruned_path_modified = [p for p in path]
        i = 0
        l = len(pruned_path_modified)

        while i < (len(pruned_path_modified) - 2):
            n1 = np.array(pruned_path_modified[i])
            n2 = np.array(pruned_path_modified[i + 2])
            p1x = n1[0]
            p1y = n1[1]
            q1x = n2[0]
            q1y = n2[1]
            p1 = Point(p1x, p1y)
            q1 = Point(q1x, q1y)
            if p1x == q1x and p1y == q1y:
                pruned_path_modified.remove(pruned_path_modified[i + 1])

            elif self.is_collision_not_detected_rect(p1, q1, obstacleList_rect):

                pruned_path_modified.remove(pruned_path_modified[i + 1])

            else:
                i = i + 1
        l = len(pruned_path_modified)

        return pruned_path_modified

    @staticmethod
    def get_motion_model():
        """
        The `get_motion_model` function returns a motion model consisting of different motion directions and
        their associated costs.
        :return: The `get_motion_model` function returns a NumPy array representing a motion model. Each row
        in the array represents a motion with the format [dx, dy, cost], where dx and dy are the changes in
        x and y coordinates respectively, and cost is the associated cost of that motion.
        """
        # dx, dy, cost
        motion = np.array(
            [
                [1, 0, 1],
                [0, 1, 1],
                [-1, 0, 1],
                [0, -1, 1],
                [-1, -1, math.sqrt(2)],
                [-1, 1, math.sqrt(2)],
                [1, -1, math.sqrt(2)],
                [1, 1, math.sqrt(2)],
            ]
        )

        return motion


import time
import random
import matplotlib.pyplot as plt
import numpy as np


def is_point_inside_polygon(point, polygon):
    """Check if the point is inside the polygon using the ray-casting algorithm."""
    x, y = point
    n = len(polygon)
    inside = False
    p1x, p1y = polygon[0]
    for i in range(n + 1):
        p2x, p2y = polygon[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x, p1y = p2x, p2y
    return inside


def is_valid_point(point, obstacles):
    """Check if the point is inside any obstacle."""
    for obs in obstacles:
        if is_point_inside_polygon(point, obs):
            return False
    return True

# CHANGED HERE
def simulate_movement(a_star, path, obstacles, new_obstacles, goal):
    """
    Simulate movement along path, check for new obstacles, switch to MC if blocked.
    """
    # CHANGED HERE: New function to simulate "runtime" movement and dynamic avoidance.
    pruned_path = path[1][::-1]  # Use pruned path, reverse to start -> goal.
    current_pos = pruned_path[0]
    final_path = [current_pos]
    blocked = False
    for i in range(1, len(pruned_path)):
        next_pos = pruned_path[i]
        if a_star.is_collision_not_detected_rect(current_pos, next_pos, new_obstacles):
            final_path.append(next_pos)
            current_pos = next_pos
        else:
            print(f"New obstacle detected at current_pos: {current_pos}! Switching to MC.")
            blocked = True
            all_obstacles = list(obstacles) + list(new_obstacles)
            mc_path = a_star.monte_carlo_planning(current_pos, goal, all_obstacles)
            
            # Debug: Print MC start point to confirm
            print(f"MC starting from: {current_pos}, goal: {goal}")
            
            if mc_path:
                final_path += mc_path[1:]  # Append MC path (skips current to avoid duplication)
                print("MC path found by avoiding new obstacle.")
                break
            else:
                print("MC failed, falling back to A* from current")
                a_star.start = current_pos  # Set A* start to current position
                new_path = a_star.planning()  # Recompute A* from current
                if new_path and new_path[1]:
                    final_path += new_path[1][1:]  # Append pruned path from current
                else:
                    print("Fallback A* also failed from current position.")
                a_star.start = [sx, sy]  # Reset to original start
                break

    if not blocked:
        print("Using same path by avoiding obstacles.")
    # Plot final path in magenta.
    plt.plot([x for x,y in final_path], [y for x,y in final_path], "-+m", linewidth=0.5, label="Final Path (with avoidance)")
    # Plot new obstacles in black.
    for rect in new_obstacles:
        plt.plot(rect[:, 0], rect[:, 1], "-k", linewidth=0.5, label="New Obstacle")
    return final_path


# UPDATED
if __name__ == "__main__":
    try:
        # Your existing code...
        name_obstacleList = list(obstacles_list.keys())
        obstacleList = list(obstacles_list.values())
        print(__file__ + " start!!")
        sx = 100.0
        sy = 25.0
        gx = 35.0
        gy = 95.0
        start = [sx, sy]
        goal = [gx, gy]
        randArea = [0, 100, 0, 100]
        M1 = obstacleList[1]  # Your obstacle.

        # CHANGED HERE: Load or compute path based on flag.
        if use_trained_path:
            with open('path-planning-using-DP-and-MC/path.pkl', 'rb') as f:
                path = pickle.load(f)
            print("Loaded trained A* path.")
        else:
            a_star = AStarPlanner(start, goal, M1, randArea)
            path = a_star.planning()
            with open('path-planning-using-DP-and-MC/path.pkl', 'wb') as f:
                pickle.dump(path, f)
            print("Computed and saved A* path.")

        # CHANGED HERE: Add new obstacles here for third run (example below).
        # new_obstacles = np.array([])  # Empty for first/second run.

        # For third run, uncomment and add your new obstacle, e.g.:
        # new_obstacles = np.array([[[50, 40], [55, 40], [55, 50], [50, 50], [50, 40]]])  # New rect on path.
        # new_obstacles = np.array([[[50, 70], [65, 70], [65, 80], [50, 80], [50, 70]]])  # New rect on path.
        # new_obstacles = np.array([[[50, 70], [65, 70], [65, 100], [50, 100], [50, 70]]])  # New rect on path.
        new_obstacles = np.array([[[80, 40], [90, 40], [90, 80], [80, 80], [80, 40]]])  # New rect on path.

        # Simulate movement and handle avoidance.
        a_star = AStarPlanner(start, goal, M1, randArea)  # Re-init for methods.

        mc_result = a_star.monte_carlo_planning(start, goal, list(M1) + list(new_obstacles))
        if mc_result:
            mc_path, pruned_mc_path = mc_result
            
            # Plotting
            plt.figure(figsize=(10, 10))
            
            # Plot obstacles
            for rect in M1:
                plt.plot(rect[:, 0], rect[:, 1], "-r", linewidth=0.5, label="Original Obstacles")
            for rect in new_obstacles:
                plt.plot(rect[:, 0], rect[:, 1], "-k", linewidth=0.5, label="New Obstacles")
            
            # Plot A* paths
            plt.plot([x for (x, y) in path[0]], [y for (x, y) in path[0]], "-+g", 
                    linewidth=0.5, label="A* Original Path")
            plt.plot([x for (x, y) in path[1]], [y for (x, y) in path[1]], "-+b", 
                    linewidth=0.5, label="A* Pruned Path")
            
            # Plot MC paths
            if mc_path:
                plt.plot([x for x,y in mc_path], [y for x,y in mc_path], "-+c", 
                        linewidth=0.5, label="MC Original Path")
            if pruned_mc_path:
                plt.plot([x for x,y in pruned_mc_path], [y for x,y in pruned_mc_path], "-+m", 
                        linewidth=1.5, label="MC Pruned Path")
            
            # Plot start and goal
            plt.plot(sx, sy, "or", label="Start")
            plt.plot(gx, gy, "og", label="Goal")
            
            plt.legend()
            plt.grid(True)
            plt.axis("equal")
            plt.title("Path Planning: A* vs Monte Carlo")
            plt.show()
        else:
            print("Monte Carlo planning failed to find a path")

        # In __main__
        # final_path = simulate_movement(a_star, path, M1, new_obstacles, goal)
        # plt.plot([x for x, y in final_path], [y for x, y in final_path], "-+m", linewidth=0.5, label="Final Path (with avoidance)")

        # # Your existing plotting (add to it).
        # for rect in M1:
        #     plt.plot(rect[:, 0], rect[:, 1], "-r", linewidth=0.5, label="Obstacles")
        # plt.plot([x for (x, y) in path[1]], [y for (x, y) in path[1]], "-+b", linewidth=0.5, label="WayPoints")
        # plt.plot([x for (x, y) in path[0]], [y for (x, y) in path[0]], "-+g", linewidth=0.5, label="WayPoints")
        # plt.show()

    except KeyboardInterrupt:
        print("error")