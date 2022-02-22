import numpy as np

class BugPlanner():

    # State Machine states
    WAIT = 1
    CORRECT_HEADING = 2
    GO_TO_GOAL = 3
    GOAL_BEHAVIOUR = 4
    TURN_SIDE_TO_OBS = 5
    WALL_FOLLOW = 6

    # Robot Views
    FRONT_LEFT_ANG = 0.5
    FRONT_RIGHT_ANG = -0.5


    def __init__(self, d_min):
        self.goal = None
        self.state = self.WAIT
        self.d_min = d_min
        self.saved_heading = 0

    def set_goal(self, goal):
        """Set goal to specified pose

        Parameters
        ----------
        goal : np.array
            goal pose to reach
        """
        self.goal = goal
        self.state = self.CORRECT_HEADING

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
        if self.state == self.CORRECT_HEADING:
            print("Correct Heading")
            # Get desired heading difference
            velocity = self.goal[0:2] - state[0:2]
            theta_des = np.arctan2(velocity[1], velocity[0])
            w_des = theta_des - state[2]
            vx_des = 0
            vy_des = 0
            if abs(w_des) < 1e-1:
                self.state = self.GO_TO_GOAL

        elif self.state == self.GO_TO_GOAL:
            print("Go to Goal")
            # Get desired heading difference
            velocity = self.goal[0:2] - state[0:2]
            vx_des = velocity[0]
            vy_des = velocity[1]
            theta_des = np.arctan2(vy_des, vx_des)
            w_des = theta_des - state[2]

            # Obstable in front of use
            dist = self.getForwardObstacleDistance(env)
            print(f"Dist: {dist}")
            if dist <= self.d_min:
                self.state = self.TURN_SIDE_TO_OBS
                vx_des = 0
                vy_des = 0
                w_des = 0
                self.saved_heading = state[2]

            # We reached destination, just turn in place
            if np.linalg.norm(velocity) < 0.5:
                self.state = self.GOAL_BEHAVIOUR

        elif self.state == self.TURN_SIDE_TO_OBS:
            print("Turn from Obstacle")
            # Left turn 90 degrees
            w_des = self.saved_heading + np.pi/2 - state[2]
            vx_des = 0
            vy_des = 0
            if abs(w_des) < 1e-1:
                self.state = self.WALL_FOLLOW

        elif self.state == self.WALL_FOLLOW:
            print("Follow Wall of Obstacle")
            vx_des = 1
            vy_des = 1

            # Check that obstacle is still beside the robot
            d_side = self.getRightObstacleDistance(env)
            print(f"Dist Side: {d_side}")

            # Use P-Controller to wall follow
            w_des = self.d_min - d_side
            
            # Check for obstable in front of robot
            d_front = self.getForwardObstacleDistance(env)
            print(f"Dist Front: {d_front}")
            if d_front <= self.d_min:
                self.state = self.TURN_SIDE_TO_OBS
                vx_des = 0
                vy_des = 0
                w_des = 0
                self.saved_heading = state[2]

            else: # Check if robot heading sees the goal
                delta_pose = self.goal[0:2] - state[0:2]
                req_heading = np.arctan2(delta_pose[1], delta_pose[0])
                if req_heading > (state[2] + self.FRONT_RIGHT_ANG) and req_heading < (state[2] + self.FRONT_LEFT_ANG):
                    self.state = self.CORRECT_HEADING

        elif self.state == self.GOAL_BEHAVIOUR:
            print("Goal Behaviour")
            vx_des = 0
            vy_des = 0
            w_des = self.goal[2] - state[2]

        else:
            print("Do nothing")
            return np.array([0, 0, 0])

        print(f"V : ({vx_des},{vy_des})\t\t W: {w_des}")

        return np.array([vx_des, vy_des, w_des])

    def getForwardObstacleDistance(self, env):
        """Check the environment in front of the robot to see if there is an obstacle

        Parameters
        ----------
        env : np.array
            lidar data where data is organized as row 1: angles, row 2: distance to object

        Returns
        -------
        d_min : float
            Smallest forward distance to the robot
        """
        indices = np.where((env[0,:] > self.FRONT_RIGHT_ANG) & (env[0,:] < self.FRONT_LEFT_ANG))[0]
        d_min = float("Inf")

        for i in indices:
            d_sample = env[1,i]
            dist = d_sample# * np.cos(env[0,i])
            if dist < d_min and dist > (self.d_min * 0.5):
                d_min = dist

        return d_min

    def getRightObstacleDistance(self, env):
        """Check the environment on right of the robot to see if there is an obstacle

        Parameters
        ----------
        env : np.array
            lidar data where data is organized as row 1: angles, row 2: distance to object

        Returns
        -------
        d_min : float
            Smallest right side distance to the robot
        """
        indices = np.where(env[0,:] < self.FRONT_RIGHT_ANG)[0]
        d_min = float("Inf")

        for i in indices:
            d_sample = env[1,i]
            dist = d_sample# * -np.sin(env[0,i])
            if dist < d_min and dist > (self.d_min * 0.5):
                d_min = dist

        return d_min