import numpy as np

class Planner():

    # State Machine states
    WAIT = 1
    CORRECT_HEADING = 2
    GO_TO_GOAL = 3
    GOAL_BEHAVIOUR = 4
    TURN_SIDE_TO_OBS = 5
    WALL_FOLLOW = 6
    CHOOSE_OBS_TURN = 7

    # Robot Views
    FRONT_LEFT_ANG = 0.5
    FRONT_RIGHT_ANG = -FRONT_LEFT_ANG
    BACK_LEFT_ANG = 2
    BACK_RIGHT_ANG = -BACK_LEFT_ANG


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
        # Check if we are at goal everytime to not overshoot
        velocity = self.goal[0:2] - state[0:2]
        vx_des = velocity[0]
        vy_des = velocity[1]
        if np.linalg.norm(velocity) < 0.5:
            self.state = self.GOAL_BEHAVIOUR

        # Bug State Machine
        if self.state == self.CORRECT_HEADING:
            print("Correct Heading")
            # Get desired heading difference
            velocity = self.goal[0:2] - state[0:2]
            theta_des = np.arctan2(velocity[1], velocity[0])
            err = theta_des - state[2]
            w_des = np.arctan2(np.sin(err), np.cos(err))
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
            err = theta_des - state[2]
            w_des = np.arctan2(np.sin(err), np.cos(err))

            # Obstable in front of use
            dist = self.getForwardObstacleDistance(env)
            print(f"Dist: {dist}")
            if dist <= self.d_min:
                self.state = self.CHOOSE_OBS_TURN
                vx_des = 0
                vy_des = 0
                w_des = 0
                self.saved_heading = state[2]

            # We reached destination, just turn in place
            if np.linalg.norm(velocity) < 0.5:
                self.state = self.GOAL_BEHAVIOUR

        elif self.state == self.CHOOSE_OBS_TURN:
            # Check which direction is better to go in
            d_left = self.getLeftObstacleDistance(env)
            d_right = self.getRightObstacleDistance(env)
            d_front = self.getForwardObstacleDistance(env)
            d_max_right = self.getMaxRightObstacleDistance(env)
            d_max_left = self.getMaxLeftObstacleDistance(env)

            dx = self.goal[0:2] - state[0:2]
            theta_des = np.arctan2(dx[1], dx[0])
            err = theta_des - state[2]
            theta_err = np.arctan2(np.sin(err), np.cos(err))

            print(f"Dist: Front={d_front}, Left={d_left}, Right={d_right}, Max | Left={d_max_left}, Right={d_max_right}")
            print(f"Theta Error: {theta_err}")

            self.turn_goal = 0

            # Check that goal direction has no obstacles
            if theta_err >= self.FRONT_LEFT_ANG:
                if d_left > self.d_min or d_max_left > d_max_right:
                    print("Bias Turn Left")
                    self.turn_goal = np.pi/2
                else:
                    print("Last Resort Turn Right")
                    self.turn_goal = -np.pi/2
            # Check that other direction is not blocked by obstacle
            elif abs(theta_err) < self.FRONT_LEFT_ANG:
                if d_left > self.d_min and d_max_left > d_max_right:
                    print("No Bias Turn Left")
                    self.turn_goal = np.pi/2
                elif d_right > self.d_min and d_max_left < d_max_right:
                    print("No Bias Turn Right")
                    self.turn_goal = -np.pi/2
                else:
                    # Default
                    print("Default Turn Left")
                    self.turn_goal = -np.pi/2
            else:
                if d_right > self.d_min or d_max_right > d_max_left:
                    print("Bias Turn Right")
                    self.turn_goal = -np.pi/2
                else:
                    print("Last Resort Turn Left")
                    self.turn_goal = np.pi/2

            self.state = self.TURN_SIDE_TO_OBS
            vx_des = 0
            vy_des = 0
            w_des = 0

        elif self.state == self.TURN_SIDE_TO_OBS:
            print("Turn from Obstacle")            
            err = self.saved_heading + self.turn_goal - state[2]
            w_des = np.arctan2(np.sin(err), np.cos(err))
            vx_des = 0
            vy_des = 0

            d_front = self.getForwardObstacleDistance(env)

            if abs(w_des) < 1e-1 or d_front > (self.d_min * 2):
                self.state = self.WALL_FOLLOW

        elif self.state == self.WALL_FOLLOW:
            print("Follow Wall of Obstacle")
            vx_des = 1
            vy_des = 1

            # Check that obstacle is still beside the robot
            d_right = self.getRightObstacleDistance(env)
            d_left =  self.getLeftObstacleDistance(env)
            print(f"Dist Side: L{d_left} R{d_right}")

            # Use P-Controller to wall follow
            if d_right < d_left:
                w_des = self.d_min - d_right
            else:
                w_des = -(self.d_min - d_left)

            # Check for obstable in front of robot
            d_front = self.getForwardObstacleDistance(env)
            print(f"Dist Front: {d_front}")
            if d_front <= self.d_min:
                self.state = self.CHOOSE_OBS_TURN
                vx_des = 0
                vy_des = 0
                w_des = 0
                self.saved_heading = state[2]

            else: # Check if robot heading sees the goal
                delta_pose = self.goal[0:2] - state[0:2]
                req_heading = np.arctan2(delta_pose[1], delta_pose[0])
                if(
                    req_heading > (state[2] + self.FRONT_RIGHT_ANG) and req_heading < (state[2] + self.FRONT_LEFT_ANG)
                    or
                    (d_right > (self.d_min*2) and d_left > (self.d_min*2))
                    # or
                    # self.isGoalClear(req_heading, delta_pose, env)
                ):
                    self.state = self.CORRECT_HEADING

        elif self.state == self.GOAL_BEHAVIOUR:
            print("Goal Behaviour")
            vx_des = 0
            vy_des = 0
            err = self.goal[2] - state[2]
            w_des = np.arctan2(np.sin(err), np.cos(err))

        else:
            print("Do nothing")
            return np.array([0, 0, 0])

        print(f"V : ({vx_des},{vy_des})\t\t W: {w_des}")

        return np.array([vx_des, vy_des, w_des*2])

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
            dist = d_sample * np.cos(env[0,i])
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
        indices = np.where((env[0,:] < self.FRONT_RIGHT_ANG*2) & (env[0,:] > self.BACK_RIGHT_ANG))[0]
        d_min = float("Inf")

        for i in indices:
            d_sample = env[1,i]
            dist = d_sample * abs(np.sin(env[0,i]))
            if dist < d_min and dist > (self.d_min * 0.5):
                d_min = dist

        return d_min

    def getLeftObstacleDistance(self, env):
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
        indices = np.where((env[0,:] > self.FRONT_LEFT_ANG*2) & (env[0,:] < self.BACK_LEFT_ANG))[0]
        d_min = float("Inf")

        for i in indices:
            d_sample = env[1,i]
            dist = d_sample * abs(np.sin(env[0,i]))
            if dist < d_min and dist > (self.d_min * 0.5):
                d_min = dist

        return d_min

    def getMaxRightObstacleDistance(self, env):
        """Check the environment on right of the robot to see if there is an obstacle

        Parameters
        ----------
        env : np.array
            lidar data where data is organized as row 1: angles, row 2: distance to object

        Returns
        -------
        d_max : float
            Largest right side distance to the robot
        """
        indices = np.where((env[0,:] < self.FRONT_RIGHT_ANG*2) & (env[0,:] > self.BACK_RIGHT_ANG))[0]
        d_max = 0

        for i in indices:
            d_sample = env[1,i]
            dist = d_sample * abs(np.sin(env[0,i]))
            if dist > d_max and dist > (self.d_min * 0.5):
                d_max = dist

        return d_max

    def getMaxLeftObstacleDistance(self, env):
        """Check the environment on right of the robot to see if there is an obstacle

        Parameters
        ----------
        env : np.array
            lidar data where data is organized as row 1: angles, row 2: distance to object

        Returns
        -------
        d_max : float
            Largest right side distance to the robot
        """
        indices = np.where((env[0,:] > self.FRONT_LEFT_ANG*2) & (env[0,:] < self.BACK_LEFT_ANG))[0]
        d_max = 0

        for i in indices:
            d_sample = env[1,i]
            dist = d_sample * abs(np.sin(env[0,i]))
            if dist > d_max and dist > (self.d_min * 0.5):
                d_max = dist

        return d_max

    def isGoalClear(self, goal_direction, goal_delta, env):
        """Check the environment on right of the robot to see if there is an obstacle

        Parameters
        ----------
        goal_direction: float
            angle of direction of goal with respect to robot (rads)
        goal_delta : np.array
            difference between the goal and the robot pose
        env : np.array
            lidar data where data is organized as row 1: angles, row 2: distance to object

        Returns
        -------
        d_max : float
            Largest right side distance to the robot
        """
        indices = np.where((env[0,:] > (goal_direction + self.FRONT_RIGHT_ANG)) & (env[0,:] < (goal_direction + self.FRONT_LEFT_ANG)))[0]
        d_min = float("Inf")

        for i in indices:
            d_sample = env[1,i]
            dist = d_sample * np.cos(env[0,i])
            if dist < d_min and dist > (self.d_min * 0.5):
                d_min = dist

        return (d_min > np.linalg.norm(goal_delta))


