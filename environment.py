from ray.rllib.env.multi_agent_env import MultiAgentEnv
import csv

import gymnasium as gym
import numpy as np
import sumolib
import os
import traci
import sys
import json

from agent_based_control.hl_controller import HL_Controller,hl_obs_space,hl_action_space
from agent_based_control.signal_agent import SignalAgent,ll_obs_space,ll_action_space
from coordination.coordinator import Coordinator,coordinated_path_dict,coordinated_paths_GWC,coordinated_paths_SCC
from network import Network
import random

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

class MyEnv(MultiAgentEnv):
    def __init__(self, config):
        super().__init__()

        # ===== 加载来自 config 的参数 =====
        self.use_gui = config.get("use_gui", True)
        self.training_type = config.get("training_type", "low_level_IIC")
        self.control_interval = config.get("control_interval", 3)
        self.episode_length_sec = config.get("episode_length_sec", 3600)

        # 判断 sim_dir 是什么类型
        user_sim_dir = config.get("sim_dir", None)
        self.init_dir_list(user_sim_dir)

        self.warm_up_time = 600
        self.measure_for_coordination_time = 600
        self.coord_duration = 3600 - self.measure_for_coordination_time


        self.temp = 0
        self.cur_sec = 0
        self.current_episode = 0 #0 for first train

        # ===== SUMO 启动方式 =====
        if self.use_gui:
            self._sumo_binary = sumolib.checkBinary("sumo-gui")
        else:
            self._sumo_binary = sumolib.checkBinary("sumo")

        self.init_network()

        self.low_level_policies = ["low_level_IIC", "low_level_GWC", "low_level_SCC"]
        self.agents = self.possible_agents = self.init_agents()
        self.init_space()

        self.init_coordinator()

        self.last_joint_option = 0

        self.only_model = config.get("only_model", False)
        self.evaluation_mode = config.get("evaluation_mode", False)

        self.init_log()

        if not hasattr(self, "_action_space_in_preferred_format"):
            self._action_space_in_preferred_format = None
        if not hasattr(self, "_obs_space_in_preferred_format"):
            self._obs_space_in_preferred_format = None


    def init_log(self):
        if self.only_model:
            self.log_dir = f"results/only_model"
        elif self.evaluation_mode:
            self.log_dir = f"results/evaluation"
        else:
            self.log_dir = f"results/{self.training_type}"

        os.makedirs(self.log_dir, exist_ok=True)

        self.step_log_path = os.path.join(self.log_dir, "step_data.csv")
        self.episode_log_path = os.path.join(self.log_dir, "episode_summary.csv")

        if not os.path.exists(self.step_log_path):
            with open(self.step_log_path, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["training_type","demand_level","episode", "cur_sec", "agent_id", "action", "info", "reward", "observation"])

        if not os.path.exists(self.episode_log_path):
            with open(self.episode_log_path, "w", newline="") as f:
                writer = csv.writer(f)

                name_list =["training_type", "demand_level", "episode", "vehicle_num", "throughput",
                                 "total_waiting_time","total_travel_time",
                                 "average_waiting_time","average_travel_time",
                                 "total_one_dir_coord_path_waiting_time",
                                 "total_two_dir_coord_path_waiting_time",
                                 "total_one_dir_coord_path_travel_time",
                                 "total_two_dir_coord_path_travel_time",
                                 "branch_travel_time", "total_sum_reward", "total_reward_queue",
                                 "total_reward_vehwait", "align_rate"]

                if self.training_type == "high_level":
                    name_list+=["total_hl_reward", "hl_reward_queue", "hl_reward_main_line_stop", "hl_reward_main_line_avg_speed"]

                writer.writerow(name_list)

        self.step_buffer = []

    def init_dir_list(self,user_sim_dir):
        default_policy2demands = {
            "high_level": ["high_level_sim"],
            "low_level_IIC": ["low_demand", "medium_demand", "high_demand"],
            "low_level_GWC": ["low_demand", "medium_demand", "high_demand"],
            "low_level_SCC": ["low_demand", "medium_demand", "high_demand"],
        }

        if isinstance(user_sim_dir, str):
            self.sim_dir_list = [user_sim_dir]
        elif isinstance(user_sim_dir, list):
            self.sim_dir_list = user_sim_dir
        else:
            self.sim_dir_list = default_policy2demands[self.training_type]

        self.cur_sim_dir = self.sim_dir_list[0]
        self.set_xml(f'simulation/{self.cur_sim_dir}')

    def set_xml(self, dir):
        self._net = os.path.join(dir, 'exp.net.xml')
        self._route = os.path.join(dir, 'exp.rou.xml')
        self._cfg = os.path.join(dir, 'exp.sumocfg')

    def init_network(self):
        self.network = Network(step_length=self.control_interval, sumocfg_file=self._cfg, net_file=self._net)
        self.network._init_network(sim=False)

    def init_agents(self):
        signals = {}
        tl_nodes = self.network.tl_nodes.keys()

        for node_id in tl_nodes:
            agent_id = f"signal_{node_id}"

            if self.training_type == "high_level":
                signals[agent_id] = SignalAgent(node_id, agent_id, node=self.network.tl_nodes[node_id], default_policy="low_level_IIC")
            else:
                signals[agent_id] = SignalAgent(node_id, agent_id, node=self.network.tl_nodes[node_id], default_policy=self.training_type)

        self.signals = signals


        if self.training_type=='high_level':
            agent_ids = []
            for signal_id in self.signals:
                for policy in self.low_level_policies:
                    agent_ids.append(f"{policy}_{signal_id}")

            self.hl_controller = HL_Controller(nodes=self.network.tl_nodes, coord_duration = self.coord_duration, measure_for_coordination_time=self.measure_for_coordination_time,
                                               episode_length_sec = self.episode_length_sec, start_time = self.warm_up_time + self.measure_for_coordination_time)
            agent_ids.append('high_level')

        else:
            agent_ids = list(signals.keys())

        return agent_ids


    def init_space(self):
        self.observation_spaces = {}
        self.action_spaces = {}

        if self.training_type!='high_level':
            for agent_id in self.agents:
                if agent_id.startswith("signal_"):
                    obs_dim = ll_obs_space()
                    act_dim = ll_action_space()
                    self.observation_spaces[agent_id] = gym.spaces.Dict(
                        {
                            "action_mask": gym.spaces.Box(0.0, 1.0, shape=(act_dim,)),
                            "observations": gym.spaces.Box(low=-1.0, high=100.0, shape=(obs_dim,),
                                                           dtype=np.float32),
                        }
                    )
                    self.action_spaces[agent_id] = gym.spaces.Discrete(act_dim)
        else:
            for agent_id in self.agents:
                if agent_id == 'high_level':
                    obs_dim = hl_obs_space()
                    act_dim = hl_action_space()
                    self.observation_spaces[agent_id] = gym.spaces.Dict(
                        {"action_mask": gym.spaces.Box(0.0, 1.0, shape=(act_dim,)),
                         "observations": gym.spaces.Box(low=-1.0, high=100.0, shape=(obs_dim,), dtype=np.float32)
                         })

                    self.action_spaces[agent_id] = gym.spaces.Discrete(act_dim)
                elif agent_id.startswith("low_level"):
                    obs_dim = ll_obs_space()
                    act_dim = ll_action_space()
                    self.observation_spaces[agent_id] = gym.spaces.Dict(
                        {
                            "action_mask": gym.spaces.Box(0.0, 1.0, shape=(act_dim,)),
                            "observations": gym.spaces.Box(low=-1.0, high=100.0, shape=(obs_dim,),
                                                           dtype=np.float32),
                        }
                    )
                    self.action_spaces[agent_id] = gym.spaces.Discrete(act_dim)

    def sim_step(self, measure_for_learning = False, measure_for_coordination = False):
        self.cur_sec += self.control_interval   #222此处修改过self.cur_sec += self.control_interval的位置！！！ 没有重跑结果

        for _ in range(self.control_interval):
            traci.simulation.step()
            if measure_for_learning:
                self.throughput += traci.simulation.getArrivedNumber()
                self.vehicle_num += traci.simulation.getDepartedNumber()

        if measure_for_learning:
            if self.training_type == "high_level":
                self.network.update_state_and_reward(enable_high_level=True)
            else:
                self.network.update_state_and_reward()
        if measure_for_coordination:
            self.network.measure_for_coordination()

    def reset_sim(self):
        if len(self.sim_dir_list)==1:
            self.cur_sec = self.sim_dir_list[0]
        else:
            self.cur_sim_dir = random.choice(self.sim_dir_list)

        # 设置仿真文件路径
        self.set_xml(f'simulation/{self.cur_sim_dir}')


        sumo_cmd = [self._sumo_binary,
                    '-n', self._net,
                    '-r', self._route,
                    '--quit-on-end',
                    "--collision.action", "none",
                    '--waiting-time-memory', '10000',
                    '--random',
                    "--step-length", "1",  # 每步仿真按整秒推进
                    '--no-step-log',
                    '--no-warnings']

        if self._seed is not None:
            sumo_cmd.append(f'--seed={self._seed}')

        traci.start(sumo_cmd)
        self.cur_sec = 0

    def init_coordinated_paths(self):
        if self.training_type == 'high_level':
            self.coordinated_paths_dict = coordinated_path_dict()
            self.coordinated_paths = self.coordinated_paths_dict[0]
        elif self.training_type == 'low_level_GWC':
            self.coordinated_paths = coordinated_paths_GWC()
        elif self.training_type == 'low_level_SCC':
            self.coordinated_paths = coordinated_paths_SCC()
        elif self.training_type == 'low_level_IIC':
            self.coordinated_paths = []

    def init_coordinator(self):
        self.init_coordinated_paths()
        self.coordinator = Coordinator(coordinated_paths=self.coordinated_paths, nodes=self.network.nodes, tl_nodes=self.network.tl_nodes, edges=self.network.edges,
                                       connections=self.network.connections, cur_sec=0)

    def close(self):
        if traci.isLoaded():
            try:
                traci.close()
            except Exception as e:
                print(f"Failed to close SUMO: {e}")

    def generate_seed(self, seed = None):
        if seed is None:
            seed = random.randint(1, 10000)

        self._seed = seed
        np.random.seed(seed)
        random.seed(seed)
        return seed

    def reset(self, *, seed=None, options=None):
        self.generate_seed(seed = seed)
        print(f"Using seed: {self._seed}")

        self.close()

        self.current_episode += 1

        self.reset_sim()

        while self.cur_sec <= self.warm_up_time:
            self.sim_step(measure_for_learning = False, measure_for_coordination = False)


        self.init_coordinator()

        self.coordinator.update_coordinated_paths(self.coordinated_paths, self.cur_sec, left_time = min(self.coord_duration,self.episode_length_sec - self.cur_sec - self.measure_for_coordination_time))

        self.measure_for_coord = True
        while self.cur_sec <= self.warm_up_time + self.measure_for_coordination_time:
            self.sim_step(measure_for_learning = False, measure_for_coordination = self.measure_for_coord)

        self.measure_for_coord = False
        if self.training_type == "high_level":
            self.network.update_state_and_reward(enable_high_level=True)
        else:
            self.network.update_state_and_reward()   #Note: learning process not include,reset obs may be empty

        self.coordinator.set_coordinated_plans(self.cur_sec)

        self.reset_record()

        obs = {}
        self.hl_controller.update_state()
        self.hl_controller.update_reward()
        if self.training_type=='high_level':
            observations, action_mask= self.hl_controller.get_obs(self.cur_sec)
            obs["high_level"] = {"observations": np.array(observations, dtype=np.float32),
                                 "action_mask": np.array(action_mask, dtype=np.float32)
                                 }
            self.hl_controller.reset()
            for signal_agent_id in self.signals:
                signal_agent = self.signals[signal_agent_id]
                observations, action_mask = signal_agent.get_obs(next_sec=self.cur_sec + self.control_interval)

                for policy in self.low_level_policies:
                    agent_id = policy+"_" + signal_agent_id
                    obs[agent_id] = {'observations': np.array(observations, dtype=np.float32),
                                     'action_mask': np.array(action_mask, dtype=np.float32)}

                signal_agent.update_policy()

        else:
            for agent_id in self.agents:
                if agent_id.startswith("signal_"):
                    signal_agent = self.signals[agent_id]
                    observations, action_mask = signal_agent.get_obs(next_sec=self.cur_sec + self.control_interval)
                    obs[agent_id] = {'observations': np.array(observations, dtype=np.float32),
                                     'action_mask': np.array(action_mask, dtype=np.float32)}

                    signal_agent.update_policy()
        return {**obs}, {}

    def reset_record(self):
        self.vehicle_num = 0
        self.throughput = 0
        self.total_waiting_time = 0
        self.total_travel_time = 0
        self.align_coord_times = 0
        self.total_steps_of_all_agents = 0
        self.total_one_dir_coord_path_travel_time = 0
        self.total_two_dir_coord_path_travel_time = 0
        self.total_one_dir_coord_path_waiting_time = 0
        self.total_two_dir_coord_path_waiting_time = 0
        self.total_sum_reward = 0
        self.total_reward_queue = 0
        self.total_reward_vehwait = 0
        if self.training_type=='high_level':
            self.total_hl_reward = 0
            self.hl_reward_queue = 0
            self.hl_reward_main_line_stop = 0
            self.hl_reward_main_line_avg_speed = 0


    def step(self, action_dict):
        if self.training_type == 'high_level':

            print(214323455432, self.cur_sec, self.signals["signal_n1"].node.tl_type)

            if "high_level" in action_dict:
                self.hl_controller.action(action_dict['high_level'],self.cur_sec)
                self.coordinate_paths = self.coordinated_paths_dict[self.hl_controller.active_action]
                self.coordinator.update_coordinated_paths(self.coordinate_paths, self.cur_sec,
                                                          left_time=min(self.coord_duration,
                                                                        self.episode_length_sec - self.cur_sec - self.measure_for_coordination_time))
                for signal_agent_id in self.signals:
                    signal_agent = self.signals[signal_agent_id]
                    signal_agent.policy = 'low_level_IIC'  # use IIC policy for measure coord info

                self.measure_for_coord = True

            if self.measure_for_coord and not self.hl_controller.if_measure_for_coord(self.cur_sec):
                self.measure_for_coord = False
                self.coordinator.set_coordinated_plans(self.cur_sec)
                for signal_agent_id in self.signals:
                    signal_agent = self.signals[signal_agent_id]
                    signal_agent.update_policy()

            signal_action_dict = {}
            for signal_agent_id in self.signals:
                signal_agent = self.signals[signal_agent_id]
                policy = signal_agent.policy
                agent_id = policy+"_"+signal_agent_id
                action = action_dict[agent_id]
                signal_agent.action(action,self.cur_sec)
                signal_action_dict[signal_agent_id] = action

        else:
            for agent_id, action in action_dict.items():
                if agent_id.startswith("signal_"):
                    signal_agent = self.signals[agent_id]
                    signal_agent.action(action, self.cur_sec)
            signal_action_dict = action_dict

        self.sim_step(measure_for_learning = True, measure_for_coordination = self.measure_for_coord)

        signal_obs = {}
        signal_reward = {}
        signal_record_infos = {}
        for agent_id in self.signals:
            signal_agent = self.signals[agent_id]
            observations, action_mask = signal_agent.get_obs(next_sec=self.cur_sec + self.control_interval)
            signal_obs[agent_id] = {'observations': np.array(observations, dtype=np.float32),
                                    'action_mask': np.array(action_mask, dtype=np.float32)}
            signal_reward[agent_id] = signal_agent.get_reward()
            signal_record_infos[agent_id] = signal_agent.phases

        if self.training_type == 'high_level':
            obs = {}
            reward = {}
            for policy in self.low_level_policies:
                for signal_agent_id in self.signals:
                    agent_id = policy+"_"+signal_agent_id
                    obs[agent_id] = signal_obs[signal_agent_id]
                    reward[agent_id] = signal_reward[signal_agent_id]

            self.hl_controller.update_reward()
            self.hl_controller.update_state()
            hl_record_infos = {'high_level': self.hl_controller.get_coord_info()}
            hl_action = {'high_level': self.hl_controller.active_action}

            if self.hl_controller.need_next_action(self.cur_sec + self.control_interval):
                observations, action_mask = self.hl_controller.get_obs(self.cur_sec)
                hl_obs = {'high_level': {"observations": np.array(observations, dtype=np.float32),
                                         "action_mask": np.array(action_mask, dtype=np.float32)
                                         }}

                hl_reward = {'high_level': self.hl_controller.get_reward()}

                obs = {**obs, **hl_obs}
                reward = {**reward, **hl_reward}
                self.update_record({**signal_action_dict, **hl_action}, {**signal_obs, **hl_obs},
                                   {**signal_reward, **hl_reward},
                               {**signal_record_infos, **hl_record_infos}, high_obs_reward_record = True)

                self.hl_controller.reset_reward()
                self.hl_controller.reset_state()
            else:
                self.update_record({**signal_action_dict, **hl_action}, {**signal_obs},
                                   {**signal_reward},
                                   {**signal_record_infos, **hl_record_infos})
        else:
            obs = {**signal_obs}
            reward = signal_reward
            # for test!!!
            # if self.cur_sec >= 2600:
            self.update_record(signal_action_dict, signal_obs, signal_reward,signal_record_infos)


        done0 = False
        if self.cur_sec >= self.episode_length_sec:
            self.save_record()
            done0 = True

        terminateds = {}
        for agent_id, action in action_dict.items():
            terminateds[agent_id] = done0
        terminateds["__all__"] = done0

        return obs, reward, terminateds, {}, {}

    def update_record(self, action_dict, obs_dict, reward_dict,record_infos_dict,high_obs_reward_record=False):

        for agent_id in self.signals:
            signal_agent = self.signals[agent_id]
            info_dict = signal_agent.get_info()
            self.total_waiting_time += info_dict['waiting_time']
            self.total_travel_time += info_dict['travel_time']
            self.total_steps_of_all_agents += 1
            self.align_coord_times += info_dict['align_with_coordination']
            self.total_one_dir_coord_path_waiting_time += info_dict['one_dir_coord_path_waiting_time']
            self.total_two_dir_coord_path_waiting_time += info_dict['two_dir_coord_path_waiting_time']
            self.total_one_dir_coord_path_travel_time += info_dict['one_dir_coord_path_travel_time']
            self.total_two_dir_coord_path_travel_time += info_dict['two_dir_coord_path_travel_time']
            self.total_sum_reward += info_dict['sum_reward']
            self.total_reward_queue += info_dict['reward_queue']
            self.total_reward_vehwait += info_dict['reward_vehwait']
        if high_obs_reward_record:
            info_dict = self.hl_controller.get_info()
            self.total_hl_reward += info_dict['hl_reward']
            self.hl_reward_queue += info_dict['reward_queue']
            self.hl_reward_main_line_stop += info_dict['reward_main_line_stop']
            self.hl_reward_main_line_avg_speed += info_dict['reward_main_line_avg_speed']

        if self.current_episode!=2 and self.current_episode % 8 != 0:
            return

        for agent_id in record_infos_dict.keys():
            act = action_dict.get(agent_id,None)
            rew = reward_dict.get(agent_id, None)
            infos = record_infos_dict[agent_id]

            if agent_id in obs_dict:
                obs = obs_dict[agent_id]['observations']
                obs_serialized = json.dumps(obs.tolist() if isinstance(obs, np.ndarray) else obs)
            else:
                obs_serialized = None

            row = [self.training_type, self.cur_sim_dir,self.current_episode, self.cur_sec,
                       agent_id, act, infos, rew, obs_serialized]
            self.step_buffer.append(row)


    def save_record(self):
        # 写入 step_data.csv
        if self.step_buffer:
            with open(self.step_log_path, "a", newline="") as f:
                writer = csv.writer(f)
                writer.writerows(self.step_buffer)
            self.step_buffer = []  # ✅ 清空缓存

        branch_travel_time = self.total_travel_time - self.total_two_dir_coord_path_travel_time

        average_waiting_time = self.total_waiting_time / max(self.vehicle_num, 1)
        avergae_travel_time = self.total_travel_time / max(self.vehicle_num, 1)
        align_rate = self.align_coord_times / self.total_steps_of_all_agents


        row = [self.training_type, self.cur_sim_dir, self.current_episode, self.vehicle_num, self.throughput,
               self.total_waiting_time, self.total_travel_time,
               average_waiting_time, avergae_travel_time,
               self.total_one_dir_coord_path_waiting_time, self.total_two_dir_coord_path_waiting_time,
               self.total_one_dir_coord_path_travel_time, self.total_two_dir_coord_path_travel_time,
               branch_travel_time, self.total_sum_reward, self.total_reward_queue, self.total_reward_vehwait, align_rate]

        if self.training_type =="high_level":
            row+=[self.total_hl_reward, self.hl_reward_queue, self.hl_reward_main_line_stop, self.hl_reward_main_line_avg_speed]

        with open(self.episode_log_path, "a", newline="") as f:
            csv.writer(f).writerow(row)


