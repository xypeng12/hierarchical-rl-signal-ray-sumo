from ray.rllib.callbacks.callbacks import RLlibCallback
from ray.rllib.core.rl_module.rl_module import RLModuleSpec
from ray.rllib.algorithms.ppo.torch.default_ppo_torch_rl_module import DefaultPPOTorchRLModule
from typing import Dict, Optional, Union, List
from ray.rllib.examples.rl_modules.classes.action_masking_rlm import (
    ActionMaskingTorchRLModule,
)
import gymnasium as gym

from ray.rllib.core.rl_module.rl_module import RLModule
from ray.rllib.policy import Policy
from ray.rllib.utils.metrics.metrics_logger import MetricsLogger
from ray.rllib.utils.typing import PolicyID

from ray.rllib.env.env_runner import EnvRunner


def generate_spec(env, spec_type):
    # 每种 low-level 策略

    if spec_type in {"low_level_GWC", "low_level_SCC", "low_level_IIC"}:

        agent_id = next(iter(env.observation_spaces))
        obs_space = env.observation_spaces[agent_id]
        act_space = env.action_spaces[agent_id]

        return RLModuleSpec(
            module_class=ActionMaskingTorchRLModule,
            observation_space=obs_space,
            action_space=act_space,
            model_config={
                "fcnet_hiddens": [256, 128],
                "fcnet_activation": "relu"
                },
        )

    # high-level 策略，同时返回所有子模块
    elif spec_type == "high_level":

        for agent_id in env.observation_spaces:
            if agent_id != "high_level":
                low_obs_space = env.observation_spaces[agent_id]
                low_act_space = env.action_spaces[agent_id]
                break
        else:
            raise ValueError("No low-level agent found!")
        high_obs_space = env.observation_spaces["high_level"]
        high_act_space = env.action_spaces["high_level"]

        low_level_GWC = RLModuleSpec(
                module_class=ActionMaskingTorchRLModule,
                observation_space=low_obs_space,
                action_space=low_act_space,
                model_config={
                "fcnet_hiddens": [256, 128],
                "fcnet_activation": "relu"
                })
        low_level_SCC = RLModuleSpec(
                module_class=ActionMaskingTorchRLModule,
                observation_space=low_obs_space,
                action_space=low_act_space,
                model_config={
                "fcnet_hiddens": [256, 128],
                "fcnet_activation": "relu"
                })
        low_level_IIC = RLModuleSpec(
                module_class=ActionMaskingTorchRLModule,
                observation_space=low_obs_space,
                action_space=low_act_space,
                model_config={
                "fcnet_hiddens": [256, 128],
                "fcnet_activation": "relu"
                })
        high_level = RLModuleSpec(
                module_class=ActionMaskingTorchRLModule,
                observation_space=high_obs_space,
                action_space=high_act_space,
                model_config={
                "fcnet_hiddens": [256, 128],
                "fcnet_activation": "relu"
                })
        return low_level_GWC, low_level_SCC, low_level_IIC, high_level
    else:
        raise ValueError(f"Unknown spec_type: {spec_type}")

def policy_mapping_fn(agent_id, episode, **kwargs):
    if agent_id == "high_level":
        return "high_level"
    if agent_id.startswith("low_level_"):
        return agent_id[:13]


'''
class Highlevel_Callbacks(RLlibCallback):
    def on_episode_created(self, *, episode, **kwargs):
        # Initialize an empty list in the `custom_data` property of `episode`.
        for agent_id in episode.agent_ids:
            if agent_id.startswith("signal"):
                episode._custom_data[agent_id] = "low_level_IIC"

        print(4565432345532)
    
    def on_episode_start(self, *, episode, **kwargs):
        for agent_id in episode.agent_ids:
            if agent_id.startswith("signal"):
                episode.custom_data[agent_id] = "low_level_IIC"
    
    def on_episode_step(self, *, episode, **kwargs):
        infos = episode.get_infos(indices=-1)
        info = infos.get("high_level", {})
        if info.get("status") == "no-active":
            return

        for agent_id in episode.agent_ids:
            if agent_id.startswith("signal"):
                last_info = infos.get(agent_id, {})
                if "policy" in last_info:
                    episode._custom_data[agent_id] = last_info["policy"]
                    
'''

'''
import os
import csv
import json
from ray.rllib.env.base_env import BaseEnv
from ray.rllib.utils.typing import EpisodeType
from ray.rllib.evaluation.episode_v2 import EpisodeV2

class Lowlevel_Callbacks(RLlibCallback):
    def __init__(self, training_type=None):
        super().__init__()
        self.training_type = training_type

        self.log_dir = f"results/{self.training_type}"
        os.makedirs(self.log_dir, exist_ok=True)

        self.step_log_path = os.path.join(self.log_dir, "step_data.csv")
        self.episode_log_path = os.path.join(self.log_dir, "episode_summary.csv")

        if not os.path.exists(self.step_log_path):
            with open(self.step_log_path, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["episode", "step", "agent_id", "action", "reward", "observation", "demand_level"])

        if not os.path.exists(self.episode_log_path):
            with open(self.episode_log_path, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["episode", "total_travel_time", "total_one_dir_coord_path_travel_time",
                                 "total_two_dir_coord_path_travel_time",
                                 "branch_travel_time", "total_sum_reward", "total_reward_queue",
                                 "total_reward_vehwait" ,"demand_level"])
        self.step_buffer = []

    def on_episode_step(
        self,
        *,
        episode: Union[EpisodeType, EpisodeV2],
        env_runner: Optional["EnvRunner"] = None,
        metrics_logger: Optional[MetricsLogger] = None,
        env: Optional[gym.Env] = None,
        env_index: int,
        rl_module: Optional[RLModule] = None,
        # TODO (sven): Deprecate these args.
        worker: Optional["EnvRunner"] = None,
        base_env: Optional[BaseEnv] = None,
        policies: Optional[Dict[PolicyID, Policy]] = None,
        **kwargs,
    ) -> None:

        sim_env = env.envs[env_index]

        # 向下解包 wrapper，直到找到你自己的 MyEnv
        while hasattr(sim_env, "env"):
            sim_env = sim_env.env

        step = getattr(sim_env, "cur_sec", -1)
        ep_id = getattr(sim_env, "current_episode", -1)
        sim = getattr(sim_env, "cur_sim_dir", "unknown")

        for agent_id in episode.agent_ids:
            obs_list = episode.get_observations().get(agent_id, [])
            rew_list = episode.get_rewards().get(agent_id, [])
            act_list = episode.get_actions().get(agent_id, [])

            if obs_list and act_list and rew_list:
                obs = obs_list[-1]
                act = act_list[-1]
                rew = rew_list[-1]

                try:
                    obs_serialized = json.dumps(obs.tolist() if isinstance(obs, np.ndarray) else obs)
                except Exception:
                    obs_serialized = str(obs)

                row = [ep_id, step, agent_id, act, rew, obs_serialized, sim]
                self.step_buffer.append(row)

    def on_episode_end(
        self,
        *,
        episode: Union[EpisodeType, EpisodeV2],
        prev_episode_chunks: Optional[List[EpisodeType]] = None,
        env_runner: Optional["EnvRunner"] = None,
        metrics_logger: Optional[MetricsLogger] = None,
        env: Optional[gym.Env] = None,
        env_index: int,
        rl_module: Optional[RLModule] = None,
        # TODO (sven): Deprecate these args.
        worker: Optional["EnvRunner"] = None,
        base_env: Optional[BaseEnv] = None,
        policies: Optional[Dict[PolicyID, Policy]] = None,
        **kwargs,
    ) -> None:

        # 写入 step_data.csv
        if self.step_buffer:
            with open(self.step_log_path, "a", newline="") as f:
                writer = csv.writer(f)
                writer.writerows(self.step_buffer)
            self.step_buffer = []  # ✅ 清空缓存

        sim_env = env.envs[env_index]

        # 向下解包 wrapper，直到找到你自己的 MyEnv
        while hasattr(sim_env, "env"):
            sim_env = sim_env.env

        ep_id = getattr(sim_env, "current_episode", -1)
        demand_level = getattr(sim_env, "cur_sim_dir", "unknown")

        total_travel_time = getattr(sim_env, "total_travel_time", 0)
        total_one_dir_coord_path_travel_time = getattr(sim_env, "total_one_dir_coord_path_travel_time", 0)
        total_two_dir_coord_path_travel_time = getattr(sim_env, "total_two_dir_coord_path_travel_time", 0)
        branch_travel_time = total_travel_time - total_two_dir_coord_path_travel_time

        total_sum_reward = getattr(sim_env, "total_sum_reward", 0)
        total_reward_queue = getattr(sim_env, "total_reward_queue", 0)
        total_reward_vehwait = getattr(sim_env, "total_reward_vehwait", 0)


        row = [ep_id, total_travel_time, total_one_dir_coord_path_travel_time, total_two_dir_coord_path_travel_time,
               branch_travel_time,total_sum_reward,total_reward_queue,total_reward_vehwait, demand_level]

        with open(self.episode_log_path, "a", newline="") as f:
            csv.writer(f).writerow(row)
'''

