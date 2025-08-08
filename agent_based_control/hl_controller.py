from coordination.coordinator import coordinated_path_dict
from agent_based_control.signal_agent import get_movement_num
import numpy as np

def get_demand_level():
    level_codes = [0,  # 6-7
                   0,  # 7
                   2,  # 8
                   1,  # 9
                   0,  # 10
                   0,  # 11
                   1,  # 12
                   0,  # 13
                   0,  # 14
                   0,  # 15
                   1,  # 16
                   2,  # 17
                   1,  # 18
                   0,  # 19
                   0,  # 20
                   0]  # 21

    return level_codes


def get_node_num():
    return 6

def hl_action_space():
    coorinated_paths_dict = coordinated_path_dict()
    return len(coorinated_paths_dict)

def hl_obs_space():
    node_num = get_node_num()
    state_num = 1 + 2 * node_num #demand 1state*2dir*node

    return state_num

class HL_Controller:
    def __init__(self,nodes=None,coord_duration = None,measure_for_coordination_time=None, episode_length_sec = None,  start_time = None):
        self.coord_duration = coord_duration
        self.measure_for_coordination_time = measure_for_coordination_time
        self.episode_length_sec = episode_length_sec
        self.start_time = start_time
        self.demand_level = get_demand_level()
        self.nodes = nodes #tl_nodes
        self.reset()
        self.action_space = hl_action_space()

        self.reset_state()
        self.reset_reward()

    def reset_reward(self):
        self.reward_queue = []
        self.reward_main_line_stop = []
        self.reward_main_line_avg_speed = []
        self.reward = 0


    def get_coord_info(self):
        return {"coordination_start_time": self.coordination_start_time,
                "active_action":self.active_action}

    def get_info(self):
        info_dict = {}
        info_dict['hl_reward'] = self.reward
        info_dict['reward_queue'] = np.mean(self.reward_queue)
        info_dict['reward_main_line_stop'] = np.sum(self.reward_main_line_stop)
        info_dict['reward_main_line_avg_speed'] = np.mean(self.reward_main_line_avg_speed)
        return info_dict

    def reset(self):
        self.active_action = None
        self.coordination_start_time = 0

    def action(self,action,cur_sec):
        #新策略 input
        self.coordination_start_time = cur_sec
        self.active_action = action


    def if_measure_for_coord(self,cur_sec):
        if cur_sec < self.coordination_start_time + self.measure_for_coordination_time:
            return True
        else:
            return False

    def update_reward(self):

        step_reward_queue = 0
        step_reward_main_line_stop = 0
        step_reward_main_line_avg_speed = []

        for node_id, node in self.nodes.items():
            step_reward_queue += node.reward_queue
            step_reward_main_line_stop += node.reward_main_line_stop
            step_reward_main_line_avg_speed.append(node.reward_main_line_avg_speed)


        step_reward_main_line_avg_speed = np.mean(step_reward_main_line_avg_speed)

        self.reward_queue.append(step_reward_queue)   #within the control step (last sim step), total queues along the corridor
        self.reward_main_line_stop.append(step_reward_main_line_stop) # within the control step (5s), total stops along the corridor
        self.reward_main_line_avg_speed.append(step_reward_main_line_avg_speed)  #avg speed

        self.eval_hl_info  = {
            "mainline_stops":step_reward_main_line_stop,
            "mainline_avg_speed": step_reward_main_line_avg_speed,
            "queue_length": step_reward_queue,
        }


    def get_reward(self):
        coefficient = [-1,-0.01,10]

        self.reward = coefficient[0] * np.mean(self.reward_queue) + coefficient[1] * np.sum(self.reward_main_line_stop) + coefficient[2] * np.mean(self.reward_main_line_avg_speed)
        return self.reward

    def reset_reward(self):
        self.reward_queue = []
        self.reward_main_line_stop = []
        self.reward_main_line_avg_speed = []
        self.reward = 0

    def reset_state(self):
        self.state_main_line_queue = {node_id:[] for node_id in self.nodes}
        #self.state_main_line_stop =  {node_id:[] for node_id in self.nodes}
        #self.state_main_line_speed =  {node_id:[] for node_id in self.nodes}
        self.state_bar_main_line_queue = {node_id:[] for node_id in self.nodes}
        #self.state_bar_main_line_stop =  {node_id:[] for node_id in self.nodes}
        #self.state_bar_main_line_speed =  {node_id:[] for node_id in self.nodes}


    def update_state(self):

        for node_id,node in self.nodes.items():
            self.state_main_line_queue[node_id].append(node.state_main_line_queue[0])
            #self.state_main_line_stop[node_id].append(node.state_main_line_stop[0])
            #self.state_main_line_speed[node_id].append(node.state_main_line_speed[0])
            self.state_bar_main_line_queue[node_id].append(node.state_main_line_queue[1])
            #self.state_bar_main_line_stop[node_id].append(node.state_main_line_stop[1])
            #self.state_bar_main_line_speed[node_id].append(node.state_main_line_speed[1])

    def get_obs(self, cur_sec):

        coord_step = int((cur_sec - self.start_time) / (self.coord_duration+self.measure_for_coordination_time))
#
        assert coord_step < len(self.demand_level)

        next_demand_level = self.demand_level[coord_step]

        main_line_queue = []
        #main_line_stop = []
        #main_line_speed = []
        bar_main_line_queue = []
        #bar_main_line_stop = []
        #bar_main_line_speed = []

        for node_id in self.nodes:
            main_line_queue.append(np.mean(self.state_main_line_queue[node_id]))
            #main_line_stop.append(np.mean(self.state_main_line_stop[node_id]))
            #main_line_speed.append(np.mean(self.state_main_line_speed[node_id]))
            bar_main_line_queue.append(np.mean(self.state_bar_main_line_queue[node_id]))
            #bar_main_line_stop.append(np.mean(self.state_bar_main_line_stop[node_id]))
            #bar_main_line_speed.append(np.mean(self.state_bar_main_line_speed[node_id]))
        '''
        # 计算每个指标的均值和标准差
        obs = [next_demand_level] + \
              [np.mean(main_line_queue), np.std(main_line_queue)] + \
              [np.mean(bar_main_line_queue), np.std(bar_main_line_queue)] + \
              [np.mean(main_line_stop), np.std(main_line_stop)] + \
              [np.mean(bar_main_line_stop), np.std(bar_main_line_stop)] + \
              [np.mean(main_line_speed), np.std(main_line_speed)] + \
              [np.mean(bar_main_line_speed), np.std(bar_main_line_speed)]
        '''

        obs = [next_demand_level] + main_line_queue + bar_main_line_queue # 1+2*6
        action_mask = [1] * self.action_space
        return obs, action_mask


    def need_next_action(self, next_sec):
        if self.active_action is not None:
            if next_sec - self.coordination_start_time < self.coord_duration + self.measure_for_coordination_time:
                # if not last for enough time
                # stick to current action
                return False
        return True