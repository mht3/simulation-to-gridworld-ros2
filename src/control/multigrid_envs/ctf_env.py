from gym_multigrid.multigrid import *
import time
import numpy as np

class Nav2DemoEnv(MultiGridEnv):
    """
    Environment that emulates the Nav2 demo world
    """

    def __init__(self, size=None, width=None, height=None, max_steps=100, see_through_walls=False, seed=2,
                 agents=None, partial_obs=True, agent_view_size=7, actions_set=..., objects_set=...,
                 agent_top=None):
        
        self.world = World
        agents = []
        agents.append(Agent(self.world, 0))
        self.agent_top = agent_top
        super().__init__(size, 
                         width, 
                         height, 
                         max_steps, 
                         see_through_walls, 
                         seed, 
                         agents,
                         agent_view_size=agent_view_size)
        
    def _gen_grid(self, width, height):
        self.grid = Grid(width, height)

        # Generate surrounding walls
        self.grid.horz_wall(self.world, 0, 0)
        self.grid.horz_wall(self.world, 0, height-1)
        self.grid.vert_wall(self.world, 0, 0)
        self.grid.vert_wall(self.world, width-1, 0)

        # generate middle wall
        self.grid.vert_wall(self.world, width//2, height//4, length=int(height//2 + 1))

        # TODO: strangely flips agent to the correct orientation
        self.place_obj(Box(self.world, color='red'), top=(1,1))

        # Top left location of each box
        obstacle_positions_horz = [int(width/10), width - int(width/10) - width//4 - 1, int(width/10), width - int(width/10) - width//4 - 1]
        obstacle_positions_vert = [int(height/10), int(height/10), height - int(height/10)- height//4 - 1, height - int(height/10)-height//4 - 1]
        for horz_pos in obstacle_positions_horz:
            for vert_pos in obstacle_positions_vert:
                self.grid.vert_wall(self.world, horz_pos, vert_pos, length=height//4)
                self.grid.vert_wall(self.world, horz_pos+width//4, vert_pos, length=height//4+1)
                self.grid.horz_wall(self.world, horz_pos, vert_pos, length=width//4)
                self.grid.horz_wall(self.world, horz_pos, vert_pos+height//4, length=width//4+1)


        for a in self.agents:
            self.place_agent(a, top=self.agent_top, size=(1,1))
    
    def _reward(self, i, rewards, reward=1):
        """
        Compute the reward to be given upon success
        """
        for j,a in enumerate(self.agents):
            if a.index==i or a.index==0:
                rewards[j]+=reward
            if self.zero_sum:
                if a.index!=i or a.index==0:
                    rewards[j] -= reward

    def _handle_pickup(self, i, rewards, fwd_pos, fwd_cell):
        if fwd_cell:
            if fwd_cell.can_pickup():
                if fwd_cell.index in [0, self.agents[i].index]:
                    fwd_cell.cur_pos = np.array([-1, -1])
                    self.grid.set(*fwd_pos, None)
                    self._reward(i, rewards, fwd_cell.reward)

    def _handle_drop(self, i, rewards, fwd_pos, fwd_cell):
        pass

    def step(self, actions):
        obs, rewards, done, info = MultiGridEnv.step(self, actions)
        return obs, rewards, done, info
    

class EasyCTF(Nav2DemoEnv):
    def __init__(self):
        # ~40x40 grid for a 10x10 m world.
        # Each grid space represents 1/4 m
        gazebo_size = 10
        multigrid_facor = 4
        # size represents multigrid size. dimensions are 38x38 to subtract off size of walls.
        size= gazebo_size*multigrid_facor - int(0.5*multigrid_facor)
        # y starting position of turtlebot (center of y axis)
        y_pos = size // 2
        # x_position slightly off to the right of center of grid. 
        x_pos = size // 2 + multigrid_facor * 2
        super().__init__(size=size, agent_top=(x_pos, y_pos))