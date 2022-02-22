import numpy as np

class SimplePlanner():

    WAIT = 1
    CORRECT_HEADING = 2
    GO_TO_GOAL = 3
    GOAL_BEHAVIOUR = 4

    def __init__(self):
        self.goal = None
        self.state = self.WAIT

    def set_goal(self, goal):
        """Set goal to specified pose

        Parameters
        ----------
        goal : np.array
            goal pose to reach
        """
        self.goal = goal
        self.state = self.CORRECT_HEADING

    def get_action(self, state):
        """Get the next action that will bring the robot
        closer to the goal

        Parameters
        ----------
        state : np.array
            current state of the robot

        Returns
        -------
        velocity : np.array
            velocity for each state
        """
        if self.state == self.CORRECT_HEADING:
            # Get desired heading difference
            velocity = self.goal[0:2] - state[0:2]
            theta_des = np.arctan2(velocity[1], velocity[0])
            w_des = theta_des - state[2]
            vx_des = 0
            vy_des = 0
            if w_des < 1e-2:
                self.state = self.GO_TO_GOAL

        elif self.state == self.GO_TO_GOAL:
            # Get desired heading difference
            velocity = self.goal[0:2] - state[0:2]
            vx_des = velocity[0]
            vy_des = velocity[1]
            theta_des = np.arctan2(vy_des, vx_des)
            w_des = theta_des - state[2]

            # We reached destination, just turn in place
            if np.linalg.norm(velocity) < 0.5:
                self.state = self.GOAL_BEHAVIOUR

        elif self.state == self.GOAL_BEHAVIOUR:
            vx_des = 0
            vy_des = 0
            w_des = self.goal[2] - state[2]

        else:
            return np.array([0, 0, 0])

        print(f"V : ({vx_des},{vy_des})\t\t W: {w_des}")

        return np.array([vx_des, vy_des, w_des])