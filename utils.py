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
