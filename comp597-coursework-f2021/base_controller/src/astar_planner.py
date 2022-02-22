import numpy as np
import matplotlib.pyplot as plt

class Astar_Planner():

    PLAN = 1
    CORRECT_HEADING = 2
    GO_TO = 3
    GOAL_BEHAVIOUR = 4
    GET_NEXT_GOAL = 5

    def __init__(self,  min_x, max_x, min_y, max_y, grid_res, max_depth, min_d):
        self.goal = None
        self.max_depth = int(max_depth)
        self.min_d = min_d

        self.grid = Grid(min_x, max_x, min_y, max_y, grid_res, min_d)

        self.state = self.PLAN
        self.intermediate_goal = np.array([0, 0, 0])
        self.plan = None

        plt.ion()
        plt.show()

    def set_goal(self, goal):
        """Set goal to specified pose

        Parameters
        ----------
        goal : np.array
            goal pose to reach
        """
        self.goal = goal

    def get_action(self, state, env):
        """Get the next action that will bring the robot
        closer to the goal

        Parameters
        ----------
        state : np.array
            current state of the robot
        env : np.array
            lidar data where data is organized as row 1: angles, row 2: distance to object

        Returns
        -------
        velocity : np.array
            velocity for each state
        """
        # Update world grid
        self.grid.reset()
        self.grid.update_grid_occupancy(state, env)

        if self.state == self.GET_NEXT_GOAL:
            if self.plan.size == 0:
                self.state = self.PLAN
            else:
                self.intermediate_goal = self.plan[0,:]
                self.plan = self.plan[1:,:]
                self.state = self.CORRECT_HEADING

        if self.state == self.PLAN:
            if np.linalg.norm(self.goal[0:2] - state[0:2]) < 0.5:
                self.intermediate_goal = self.goal
                self.plan = np.array([])
                self.state = self.GOAL_BEHAVIOUR
            else:
                # Get Path fragment
                path = Astar(state, self.goal, self.grid, self.max_depth)
                print(f"Path: {path} - {state} to {self.goal}")
                if path is None:
                    # Find safer location to start plan
                    s_grid = self.grid.get_grid_cell(state[0], state[1])
                    search_size = 1
                    new_s_grid = s_grid
                    while new_s_grid == s_grid:
                        region = self.grid.grid[
                            max(0, s_grid[0]-search_size):min(s_grid[0]+search_size+1, self.grid.size_y),
                            max(0, s_grid[1]-search_size):min(s_grid[1]+search_size+1, self.grid.size_x)
                        ]
                        safe_cells = np.where(region == 0)
                        print(region)
                        print(safe_cells)
                        print(s_grid)
                        if safe_cells[0].size == 0:
                            search_size += 1
                        else:
                            new_s_grid = [
                                safe_cells[0][0] + max(0, s_grid[0]-search_size),
                                safe_cells[1][0] + max(0, s_grid[1]-search_size)
                            ]
                            restore_state = self.grid.get_physical_location(new_s_grid[0], new_s_grid[1])
                            path = np.reshape(np.array(restore_state), (-1, 2))
                            print(f"Path: {path}")

                # Plan has no further actions, we are at the goal
                if len(path) == 0:
                    self.state = self.GOAL_BEHAVIOUR
                    return np.array([0, 0, 0])
                
                # Take first path element to execute
                self.intermediate_goal = path[0,:]
                self.plan = path[1:,:]
                self.state = self.CORRECT_HEADING

                # if path is None:
                robot_coords = self.grid.get_grid_cell(state[0], state[1])
                self.grid.grid[robot_coords[0], robot_coords[1]] = 0.5
                for point in path:
                    path_coords = self.grid.get_grid_cell(point[0], point[1])
                    self.grid.grid[path_coords[0], path_coords[1]] = 0.5

                plt.imshow(self.grid.grid, 'gray')
                plt.xlim(0, self.grid.size_x)
                plt.ylim(0, self.grid.size_y)
                plt.draw()
                plt.pause(0.01)
                plt.clf()

                self.grid.grid[robot_coords[0], robot_coords[1]] = 0
                for point in path:
                    path_coords = self.grid.get_grid_cell(point[0], point[1])
                    self.grid.grid[path_coords[0], path_coords[1]] = 0

        if self.state == self.CORRECT_HEADING:
            print("Correct Heading")
            # Get desired heading difference
            velocity = self.intermediate_goal[0:2] - state[0:2]
            theta_des = np.arctan2(velocity[1], velocity[0])
            err = theta_des - state[2]
            w_des = np.arctan2(np.sin(err), np.cos(err))
            vx_des = 0
            vy_des = 0
            # Heading not right
            if abs(w_des) < 0.2:
                self.state = self.GO_TO
            # We reached destination already
            if np.linalg.norm(velocity) < 0.5:
                self.state = self.GET_NEXT_GOAL

        elif self.state == self.GO_TO:
            print("Go to")
            # Get desired heading difference
            velocity = self.intermediate_goal[0:2] - state[0:2]
            vx_des = velocity[0]
            vy_des = velocity[1]
            theta_des = np.arctan2(vy_des, vx_des)
            err = theta_des - state[2]
            w_des = np.arctan2(np.sin(err), np.cos(err))

            # We reached destination
            if np.linalg.norm(velocity) < 0.5:
                self.state = self.GET_NEXT_GOAL

        elif self.state == self.GOAL_BEHAVIOUR:
            print("Goal")
            vx_des = 0
            vy_des = 0
            err = self.goal[2] - state[2]
            w_des = np.arctan2(np.sin(err), np.cos(err))


        print(f"V : ({vx_des},{vy_des})\t\t W: {w_des}")

        return np.array([vx_des, vy_des, w_des])


class Grid():

    def __init__(self, min_x, max_x, min_y, max_y, resolution, min_d):
        size_x = int((max_x - min_x) / resolution + 1)
        size_y = int((max_y - min_y) / resolution + 1)
        self.size_x = size_x
        self.size_y = size_y
        self.min_x = min_x
        self.min_y = min_y
        self.max_x = max_x
        self.max_y = max_y
        self.grid = np.zeros((self.size_y, self.size_x))
        self.resolution = resolution
        self.middle = [
            int(size_y/2),
            int(size_x/2)
        ]

        self.min_d = int(min_d / resolution)

    def get_physical_location(self, row, col):
        x = col*self.resolution + self.min_x
        y = row*self.resolution + self.min_y

        return np.array([x,y])

    def get_grid_cell(self, x, y):
        col = int((x - self.min_x)/self.resolution)
        row = int((y - self.min_y)/self.resolution)

        return [row, col]

    def update_grid_occupancy(self, x, lidar_data):
        # Update grid occupancy based on distance information
        for sample in lidar_data.T:
            if sample[1] < np.inf and sample[1] > (self.min_d * 0.25 * self.resolution):
                direction = sample[0] + x[2]
                x_occupied = sample[1] * np.cos(direction) + x[0]
                y_occupied = sample[1] * np.sin(direction) + x[1]
                grid_coords = self.get_grid_cell(x_occupied, y_occupied)
                if(
                    grid_coords[0] < (self.size_y)
                    and
                    grid_coords[0] >= 0
                    and
                    grid_coords[1] < (self.size_x)
                    and
                    grid_coords[1] >= 0
                ):
                    self.grid[grid_coords[0], grid_coords[1]] = 1
                    self.grid[grid_coords[0]-self.min_d:grid_coords[0]+self.min_d, grid_coords[1]-self.min_d:grid_coords[1]+self.min_d] = 1

    def reset(self):
        self.grid = np.zeros((self.size_y, self.size_x))

    def get_path(self, graph):
        path = []
        cell = graph
        while cell.parent is not None:
            path = [self.get_physical_location(cell.row, cell.col)] + path
            cell = cell.parent

        return path 


class Cell():

    def __init__(self, row, col, parent, depth):
        self.row = row
        self.col = col
        self.parent = parent
        self.g = 0
        self.f = 0
        self.depth = depth


def Astar(x, goal, grid_obj, max_depth = None):
    """Use the A* algorithm to find the shortest path
    between the start and goal positions in the graph/grid world

    Parameters
    ----------
    x : list or np.array
        initial state of the robot (x,y) in continuous space
    goal : list or np.array
        goal state of the robot (x,y) in continuous space
    grid : Grid
        model of the environment using an occupancy grid
    max_depth : int
        maximum depth of the search before returning a solution

    Returns
    -------
    traj : np.array
        (x,y) trajectory to follow to reach the goal
    """
    open_list = []
    closed_list = []
    start_cell = grid_obj.get_grid_cell(x[0], x[1])
    open_list.append(Cell(start_cell[0], start_cell[1], None, 0))
    grid_goal = grid_obj.get_grid_cell(goal[0], goal[1])

    while len(open_list) > 0:
        # Find best cell to explore
        f_min = float("inf")
        idx_min = None
        for idx in range(0, len(open_list)):
            cell = open_list[idx]
            if cell.f < f_min:
                f_min = cell.f
                idx_min = idx
        if idx_min is None:
            # print("Death by empty possibilities")
            return None     # No Path exists

        # Check if Goal is reached
        curr_cell = open_list.pop(idx_min)
        closed_list.append(curr_cell)
        if (
            [curr_cell.row, curr_cell.col] == grid_goal 
            or
            (max_depth is not None and curr_cell.depth > max_depth)
        ):
            return np.array(grid_obj.get_path(curr_cell))

        # Add candidates to search,This will be 8 cell surrounding
        candidates = []

        if curr_cell.col > 0:
            candidates += [
                Cell(curr_cell.row, curr_cell.col-1, curr_cell, curr_cell.depth + 1),
            ]
        if curr_cell.col > 0 and curr_cell.row > 0:
            candidates += [
                Cell(curr_cell.row-1, curr_cell.col-1, curr_cell, curr_cell.depth + 1),
            ]
        if curr_cell.col > 0 and curr_cell.row < (grid_obj.size_y-1):
            candidates += [
                Cell(curr_cell.row+1, curr_cell.col-1, curr_cell, curr_cell.depth + 1),
            ]
        if curr_cell.col < (grid_obj.size_x-1):
            candidates += [
                Cell(curr_cell.row, curr_cell.col+1, curr_cell, curr_cell.depth + 1),
            ]
        if curr_cell.row > 0:
            candidates += [
                Cell(curr_cell.row-1, curr_cell.col, curr_cell, curr_cell.depth + 1),
            ]
        if curr_cell.col < (grid_obj.size_x-1) and curr_cell.row > 0:
            candidates += [
                Cell(curr_cell.row-1, curr_cell.col+1, curr_cell, curr_cell.depth + 1),
            ]
        if curr_cell.col < (grid_obj.size_x-1) and curr_cell.row < (grid_obj.size_y-1):
            candidates += [
                Cell(curr_cell.row+1, curr_cell.col+1, curr_cell, curr_cell.depth + 1),
            ]
        if curr_cell.row < (grid_obj.size_y-1):
            candidates += [
                Cell(curr_cell.row+1, curr_cell.col, curr_cell, curr_cell.depth + 1),
            ]

        # Check candidates for duplicates
        for s in candidates:
            x0 = [curr_cell.row, curr_cell.col]
            xF = [s.row, s.col]
            # Make sure cell is reachable
            if grid_obj.grid[s.row, s.col] == 1:
                # print(f"{s} : Invalid due to Collision")
                continue

            # Check if previously explored
            skip_cell = False
            for idx in range(0, len(closed_list)):
                cell = closed_list[idx]
                if [cell.row, cell.col] == xF:
                    skip_cell = True
                    break
            if skip_cell:
                # print(f"{s} : Invalid due previously Explored")
                continue
            # Add cell cost
            s.g = curr_cell.g + path_cost(x0, xF) 
            s.f = s.g + heuristic(xF, np.array(grid_goal))
            # Check if cell is already in open_list
            skip_add = False
            for idx in range(0, len(open_list)):
                cell = open_list[idx]
                if [cell.row, cell.col] == xF:
                    if cell.g <= s.g:
                        skip_add = True
                    break
            if skip_add:
                # print(f"{s} : Invalid due to duplicate")
                continue

            # Add candidate to search
            open_list.append(s)

    # print("Death by end of loop...")
    
def path_cost(x0, xF):
    return np.sqrt( (xF[0] - x0[0])**2 + (xF[1] - x0[1])**2 )

def heuristic(x, goal):
    return np.sqrt( (goal[0] - x[0])**2 + (goal[1] - x[1])**2 )


if __name__ == "__main__":
    import matplotlib.pyplot as plt

    grid_param = [
        -10, 10, -10, 50, 1.0
    ]

    grid = Grid(grid_param[0], grid_param[1], grid_param[2], grid_param[3], grid_param[4], 3)
    # grid.update_grid_occupancy(
    #     [1,2,0], 
    #     np.array([
    #         [np.pi/2, 0, -np.pi/2],
    #         [5, np.inf, 10]
    #     ])
    # )

    bound_y = [12, 26]
    bound_x = [7, 28]

    grid.grid[bound_x[0]:bound_x[1],bound_y[0]:bound_y[1]] = 1

    # plt.imshow(grid.grid, 'gray')
    # plt.show()

    traj = Astar([0,0], [5,45], grid, 20)

    print(traj.T)
    robot_coords = grid.get_grid_cell(0, 5)
    grid.grid[robot_coords[1], robot_coords[0]] = 0.5
    for point in traj:
        path_coords = grid.get_grid_cell(point[0], point[1])
        grid.grid[path_coords[1], path_coords[0]] = 0.5
    
    plt.imshow(grid.grid, 'gray')
    plt.show()

    # plt.plot(traj[:,0], traj[:,1])
    # plt.xlim(grid_param[0], grid_param[1])
    # plt.ylim(grid_param[2], grid_param[3])
    # plt.show()
