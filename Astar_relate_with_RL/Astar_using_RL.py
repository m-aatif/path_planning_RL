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

global show_animation
show_animation = False


# The `AStarPlanner` class is initialized with initial_state and terminal_state positions, obstacle information,
# resolution, and end point for A* path planning in a grid map.
class AStarPlanner:

    def __init__(self, initial_state, terminal_state, obstacleList_rect, randArea):
        """
        This function initializes a grid map for A* path planning with specified initial_state, terminal_state, obstacles,
        resolution, and end point.

        :param initial_state: The `initial_state` parameter represents the starting position for path planning. It could be
        a tuple containing the x and y coordinates of the starting point in the grid map
        :param terminal_state: The `terminal_state` parameter in the `__init__` function represents the terminal_state position in the
        grid map for A* path planning. It is the destination point that the algorithm will try to reach from
        the initial_state position
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
        point or destination for the A* path planning algorithm. This point represents the terminal_state that the
        algorithm will try to reach from the starting point
        """

        """
        Initialize grid map for a star planning
        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]
        """

        self.initial_state = initial_state
        self.terminal_state = terminal_state
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
        self.motion = self.get_action_space() * resolution

    # The State class represents a State in a grid with attributes for position, cumulative_reward, and parent index.
    class State:
        def __init__(self, x, y, cumulative_reward, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cumulative_reward = cumulative_reward
            self.parent_index = parent_index

        def __str__(self):
            return (
                str(self.x)
                + ","
                + str(self.y)
                + ","
                + str(self.cumulative_reward)
                + ","
                + str(self.parent_index)
            )

    def planning(self):
        """
        The `planning` function implements the A* path search algorithm to find a path from a initial_state
        position to a terminal_state position while avoiding obstacles, and returns the final path and a pruned
        version of the path.
        :return: The `planning` method returns a list containing two elements: the original path `path`
        and the pruned path `p_path`.
        """
        """
        A star path search
        input:
            s_x: initial_state x position [m]
            s_y: initial_state y position [m]
            gx: terminal_state x position [m]
            gy: terminal_state y position [m]
        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        start_node = self.State(self.initial_state[0], self.initial_state[1], 0.0, -1)
        goal_node = self.State(self.terminal_state[0], self.terminal_state[1], 0.0, -1)

        frontier_set, explored_set = dict(), dict()
        frontier_set[self.discretize_state(start_node)] = start_node
        path = []
        if self.is_collision_not_detected_rect(
            start_node, goal_node, self.obstacleList_rect
        ):
            path.append(self.terminal_state)
            path.append(self.initial_state)

            return [path, path]

        while 1:

            if len(frontier_set) == 0:
                # print("Open set is empty..")
                break

            c_id = min(
                frontier_set,
                key=lambda o: frontier_set[o].cumulative_reward
                + self.estimate_value(goal_node, frontier_set[o]),
            )
            current = frontier_set[c_id]
            if self.is_collision_not_detected_rect(
                goal_node, current, self.obstacleList_rect
            ):
                goal_node.parent_index = c_id
                goal_node.cumulative_reward = current.cumulative_reward + self.estimate_value(goal_node, current)
                explored_set[c_id] = current
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
                # if len(explored_set.keys()) % 10 == 0:
                #     plt.pause(0.001)
            dist = self.estimate_value(goal_node, current)

            if (
                current.x == goal_node.x
                and current.y == goal_node.y
                or dist <= self.resolution
            ):

                goal_node.parent_index = current.parent_index
                goal_node.cumulative_reward = current.cumulative_reward
                break

            # Remove the item from the open set
            del frontier_set[c_id]

            # Add it to the closed set
            explored_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                State = self.State(
                    current.x + self.motion[i][0],
                    current.y + self.motion[i][1],
                    current.cumulative_reward + self.motion[i][2],
                    c_id,
                )
                n_id = self.discretize_state(State)

                # If the State is not safe, do nothing
                if not self.is_valid_state(State, current):
                    continue

                if n_id in explored_set:
                    continue

                if n_id not in frontier_set:
                    frontier_set[n_id] = State  # discovered a new State
                else:
                    if frontier_set[n_id].cumulative_reward > State.cumulative_reward:
                        # This path is the best until now. record it
                        frontier_set[n_id] = State

        path = self.calc_final_path(goal_node, explored_set)
        p_path = self.prune_path_modified(path, self.obstacleList_rect)

        return [path, p_path]

    def calc_final_path(self, goal_node, explored_set):
        """
        The function `calc_final_path` generates the final path from the terminal_state State by tracing back
        through the closed set of nodes.

        :param goal_node: The `goal_node` parameter in the `calc_final_path` method represents the State
        that is the terminal_state of the pathfinding algorithm. It is the destination State that the algorithm has
        determined as the target to reach
        :param explored_set: The `explored_set` parameter in the `calc_final_path` method is a set that
        contains all the nodes that have been visited and evaluated during the search process. It is
        typically used in pathfinding algorithms like A* to keep track of the nodes that have already
        been explored
        :return: The function `calc_final_path` returns a list of coordinates representing the final path
        from the starting State to the terminal_state State. Each coordinate is a list containing the x and y
        coordinates of a State in the path.
        """
        # generate final course
        path = []
        path.append([goal_node.x, goal_node.y])
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = explored_set[parent_index]
            path.append([n.x, n.y])

            parent_index = n.parent_index

        return path

    @staticmethod
    def estimate_value(n1, n2):
        """
        The function calculates the heuristic distance between two points using the Euclidean distance
        formula.

        :param n1: The `estimate_value` function you provided calculates the Euclidean distance between two
        points `n1` and `n2` using the formula `d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)`, where `w` is
        :param n2: It seems like you were about to provide some information about the parameter `n2` but the
        message got cut off. Could you please provide more details or let me know how I can assist you
        further?
        :return: The function `estimate_value` returns the Euclidean distance between two points `n1` and
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

    def discretize_state(self, State):
        """
        The function calculates the index of a State in a grid based on its x and y coordinates.

        :param State: The `discretize_state` function takes a `State` object as a parameter. The function
        calculates and returns the grid index of the State based on its x and y coordinates relative to
        the minimum x and y values stored in the object (`self.min_x` and `self.min_y`) and the
        :return: The function `discretize_state` is returning the index of a State in a grid based on its x
        and y coordinates relative to the minimum x and y values of the grid.
        """
        return (State.y - self.min_y) * self.x_width + (State.x - self.min_x)

    def is_valid_state(self, State, current):
        """
        The function `is_valid_state` checks if a given State is within specified boundaries and does not
        collide with obstacles.

        :param State: The `State` parameter represents a point in a 2D space with coordinates `x` and `y`. The
        function `is_valid_state` is used to check if this State is within certain boundaries (`min_x`, `min_y`,
        `max_x`, `max_y`) and if it coll
        :param current: It seems like you were about to provide the definition or explanation of the
        `current` parameter but it got cut off. Could you please provide more information about the
        `current` parameter so that I can assist you further with the `is_valid_state` function?
        :return: The function `is_valid_state` returns a boolean value - `True` if the conditions for the State
        being verified are met, and `False` otherwise.
        """

        if State.x < self.min_x:
            return False
        elif State.y < self.min_y:
            return False
        elif State.x > self.max_x:
            return False
        elif State.y > self.max_y:
            return False

        # collision check
        if not self.is_collision_not_detected_rect(
            State, current, self.obstacleList_rect
        ):
            # if self.obmap[int(State.x)][int(State.y)]:
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

    def is_collision_not_detected_rect(self, State, nearNode, obstacleList_rect):
        colision = 0

        for rect in obstacleList_rect:
            for i in range(4):
                p1 = Point(State.x, State.y)
                q1 = Point(nearNode.x, nearNode.y)
                p2_x = rect[i, 0]
                p2_y = rect[i, 1]
                q2_x = rect[i + 1, 0]
                q2_y = rect[i + 1, 1]
                p2 = Point(p2_x, p2_y)
                q2 = Point(q2_x, q2_y)
                if doIntersect(p1, q1, p2, q2):
                    colision = colision + 1
                    return False

        return True  # safe

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
    def get_action_space():
        """
        The `get_action_space` function returns a motion model consisting of different motion directions and
        their associated costs.
        :return: The `get_action_space` function returns a NumPy array representing a motion model. Each row
        in the array represents a motion with the format [dx, dy, cumulative_reward], where dx and dy are the changes in
        x and y coordinates respectively, and cumulative_reward is the associated cumulative_reward of that motion.
        """
        # dx, dy, cumulative_reward
        action_space = np.array(
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

        return action_space


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


if __name__ == "__main__":

    try:
        # accessing the list of name and values of obstacles
        name_obstacleList = list(obstacles_list.keys())
        obstacleList = list(obstacles_list.values())

        show_animation = True

        print(__file__ + " initial_state!!")

        # initial_state and terminal_state position
        sx = 100.0
        sy = 25.0
        gx = 35.0
        gy = 95.0
        initial_state = [sx, sy]
        terminal_state = [gx, gy]
        grid_size = 3
        robot_radius = 1

        # set obstable positions

        obs_area = 0
        total_area = 10000
        # for rect in M1:
        #     polygon = Polygon(rect)
        #     obs_area=obs_area+polygon.area
        #     print(polygon.area)
        M1 = obstacleList[1]

        print(obs_area * 100 / (total_area))

        print(len(M4))
        print(M4.size)
        print(M4.shape[1])
        start_time = time.time()

        randArea = [0, 100, 0, 100]
        x_min, x_max, y_min, y_max = randArea

        # Calculate side lengths
        width = x_max - x_min
        height = y_max - y_min

        # Take minimum side length
        min_side = min(width, height)

        # Define resolution as 1% of minimum side
        resolution = min_side * 0.01

        a_star = AStarPlanner(initial_state, terminal_state, M1, randArea)
        path = a_star.planning()
        # path_pruned = a_star.prune_path_modified(path, M1)

        end_time = time.time()
        print("\n  required time to calculate is :", end_time - start_time)

        for rect in M1:

            plt.plot(rect[:, 0], rect[:, 1], "-r", linewidth=0.5, label="Obstacles")
            plt.plot(
                [x for (x, y) in path[1]],
                [y for (x, y) in path[1]],
                "-+b",
                linewidth=0.5,
                label="WayPoints",
            )
            plt.plot(
                [x for (x, y) in path[0]],
                [y for (x, y) in path[0]],
                "-+g",
                linewidth=0.5,
                label="WayPoints",
            )

        plt.show()

    except KeyboardInterrupt:
        print("error")
