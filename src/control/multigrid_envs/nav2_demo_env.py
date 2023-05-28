from gym_multigrid.multigrid import *
import time

class Nav2DemoEnv(MultiGridEnv):
    """
    Environment that emulates the Nav2 demo world
    """

    def __init__(self, 
                 size=None, 
                 width=None, 
                 height=None, 
                 max_steps=100, 
                 see_through_walls=False, 
                 seed=2, 
                 agents=None, 
                 partial_obs=True, 
                 agent_view_size=7, 
                 actions_set=..., 
                 objects_set=...,
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
        self.place_obj(Box(self.world, color='red'), top=(1,1))
        obstacle_positions_horz = [int((width-2)/3)+1, int((width-2)/2)+1, int(2*(width-2)/3)+1]
        obstacle_positions_vert = [int((height-2)/3)+1, int((height-2)/2)+1, int(2*(height-2)/3)+1]
        for horz_pos in obstacle_positions_horz:
            for vert_pos in obstacle_positions_vert:
                self.grid.horz_wall(self.world, horz_pos, vert_pos, 1)
        
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
    

class Nav2Demo4HEnv24x24N2(Nav2DemoEnv):
    def __init__(self):
        super().__init__(size=26,
                         agent_top=(10, 11))