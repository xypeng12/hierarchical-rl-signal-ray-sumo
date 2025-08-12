def get_movement_num():
    return 8  #s l *4
def ll_obs_space():
    movement_num = get_movement_num()
    state_num = movement_num * 7  # arrival, queue, speed neighbor's movement(4*12)
    '''
    max_saturated_n = 6
    max_green_band_n = 6
    state_num += max(2*max_saturated_n+3, 2*max_green_band_n+2)
    '''

    return state_num

def ll_action_space():
    phase_num = 8 
    return phase_num

class SignalAgent:
    def __init__(self, node_id,agent_id,node=None, default_policy="low_level_IIC"):
        self.node_id = node_id
        self.agent_id = agent_id
        self.node = node
        self.policy = default_policy
        self.action_space = ll_action_space()
        self.reward = 0

        self.phases = None
        self.next_step_feasible_phases = None
        self.align_with_coordination = True

        self.coef_vehwait = 0.3


    def update_policy(self):
        if self.node.tl_type == 'isolated_control':
            self.policy = 'low_level_IIC'
        elif self.node.tl_type == 'green_band_control':
            self.policy = 'low_level_GWC'
        elif self.node.tl_type == 'saturated_control':
            self.policy = 'low_level_SCC'

    def action(self,action,cur_sec):
        if self.next_step_feasible_phases is not None:
            feasible_phases = self.next_step_feasible_phases
        else:
            feasible_phases = self.node.coordinated_feasible_phases(cur_sec)

        action_phase = self.node.phase_names[action]

        if action_phase in feasible_phases:
            self.align_with_coordination = True
        else:
            self.align_with_coordination = False

        self.node.set_phase(action_phase)

        self.phases = {'align_with_coordination': self.align_with_coordination,
                        "feasible_phases": feasible_phases,
                       "action_phase": action_phase}

    def get_obs(self, next_sec = 0):

        cur_state = self.node.state_arrival + self.node.state_queue + self.node.state_speed + self.node.state_related_node  #+ self.node.state_coord   #node.state_coord may need to be delete

        if self.policy == 'low_level_IIC':
            action_mask = [1] * self.action_space
        else:
            self.next_step_feasible_phases = self.node.coordinated_feasible_phases(next_sec)
            action_mask = [1 if name in self.next_step_feasible_phases else 0 for name in self.node.phase_names]


        return cur_state, action_mask

    def get_reward(self):

        self.reward = -self.node.reward_queue - self.coef_vehwait * self.node.reward_vehwait

        return self.reward

    def get_info(self):

        info_dict = self.node.get_info()
        info_dict["sum_reward"] = self.reward

        if self.align_with_coordination:
            info_dict['align_with_coordination'] = 1
        else:
            info_dict['align_with_coordination'] = 0

        return info_dict

    def get_coord_action(self,cur_sec):
        if self.policy in {"low_level_GWC", "low_level_SCC"}:
            phase = self.node.max_weight_phase(phase_names=self.node.coordinated_feasible_phases(cur_sec))
        else:
            phase = self.node.max_weight_phase(phase_names=self.node.coordinated_feasible_phases(cur_sec))

        action = self.node.phase_names.index(phase)
        return action
