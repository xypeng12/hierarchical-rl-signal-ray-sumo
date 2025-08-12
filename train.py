from ray import tune
from ray.rllib.algorithms.ppo import PPOConfig
from environment import MyEnv

from ray.rllib.core.rl_module.multi_rl_module import MultiRLModuleSpec
import os
import argparse
import warnings
from utils import generate_spec, policy_mapping_fn

warnings.filterwarnings("ignore", category = DeprecationWarning)
os.environ["RAY_DEDUP_LOGS_SKIP_REGEX"] = "Retrying in 1 seconds"

def generate_parser():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--train-policy",
        type=str,
        choices=["high_level", "low_level_GWC", "low_level_SCC", "low_level_IIC"],
        default="low_level_SCC",
        help="Specify which policy to train. Others will be frozen.",
    )

    parser.add_argument(
        "--use-gui",
        action="store_true",
        help="Whether to use SUMO GUI (sumo-gui)."
    )

    parser.add_argument(
        "--low-level-step-duration-sec",
        type=int,
        default=3,
        help="Duration (in seconds) of each low-level step."
    )

    parser.add_argument(
        "--high-level-decision-freq-steps",
        type=int,
        default=100,
        help="High-level policy chooses a new action every N low-level steps."
    )

    # 每个 episode 的总持续时间（秒）
    parser.add_argument(
        "--episode-duration-sec",
        type=int,
        default=3600,
        help="Duration of each episode in seconds."
    )
    return parser


if __name__ == "__main__":

    parser = generate_parser()
    cmd_args = parser.parse_args()  
    all_policies = ["low_level_IIC", "low_level_GWC"]
    low_level_policies = {"low_level_GWC", "low_level_SCC", "low_level_IIC"}

    for train_policy in all_policies:
        if train_policy == "high_level":
            cmd_args.episode_duration_sec = 58800

        args = argparse.Namespace(
            train_policy=train_policy,
            use_gui=cmd_args.use_gui,
            low_level_step_duration_sec=cmd_args.low_level_step_duration_sec,
            episode_duration_sec=cmd_args.episode_duration_sec,
        )
        print(f"\n🚦 Start training policy: {args.train_policy}")

        cls = MyEnv

        tune.register_env("env", lambda cfg: cls(config=cfg))


        env_config = {
            "use_gui": args.use_gui,
            "training_type": args.train_policy,
            "control_interval": args.low_level_step_duration_sec,
            "episode_length_sec": args.episode_duration_sec,
        }
        env_instance = cls(config=env_config)

        base_config = (
            PPOConfig()
            .environment(
                env="env",
                env_config=env_config,
            )
            .training(
                train_batch_size_per_learner=20000,
                minibatch_size=1024,
                num_epochs=20,
                clip_param=0.3,
                kl_coeff=0.2,
                vf_clip_param=1000.0,
                lr=[(0, 5e-4), (2e5, 1e-4), (5e5, 1e-5)],
                entropy_coeff=0.005,
            )
            .env_runners(rollout_fragment_length=400,
                         sample_timeout_s=300,
                         observation_filter = "MeanStdFilter")
        )


        # Configure a proper multi-agent setup for the hierarchical env.
        if args.train_policy == "high_level":

            GWC_spec, SCC_spec, IIC_spec, high_level_spec = generate_spec(env_instance,'high_level')

            for policy in low_level_policies:
                path = os.path.join("policies", policy, "learner_group", "learner", "rl_module", policy)
                abs_path = os.path.abspath(path)
                spec_path = f"file://{abs_path}"

                if policy=="low_level_GWC":
                    GWC_spec.load_state_path = spec_path
                elif policy=="low_level_SCC":
                    SCC_spec.load_state_path = spec_path
                else:
                    IIC_spec.load_state_path = spec_path

            base_config.multi_agent(
                policies = {"high_level","low_level_GWC","low_level_SCC","low_level_IIC"},
                policy_mapping_fn = policy_mapping_fn,
                policies_to_train = ["high_level"],
            )
            base_config.rl_module(
                rl_module_spec=MultiRLModuleSpec(
                    rl_module_specs={
                    "low_level_GWC": GWC_spec,
                    "low_level_SCC": SCC_spec,
                    "low_level_IIC": IIC_spec,
                    "high_level": high_level_spec,
                    },
                )
            )

        elif args.train_policy.startswith("low_level_"):
            spec = generate_spec(env_instance,args.train_policy)
            base_config.multi_agent(
                policies = {args.train_policy},
                policy_mapping_fn=lambda agent_id, episode, **kwargs: args.train_policy,
                policies_to_train=[args.train_policy],
            )
            base_config.rl_module(rl_module_spec = MultiRLModuleSpec(rl_module_specs = {args.train_policy:spec}))

        train = base_config.build_algo()

        if train_policy.startswith("low_level"):
            max_iteration = 300
            output_iteration_gap = 50
        else:
            max_iteration = 30
            output_iteration_gap = 10

        for i in range(max_iteration):
            train.train()
            print(f"[Iteration {i}]")
            if i%output_iteration_gap==0 and i!=0:
                save_dir = os.path.abspath(os.path.join("policies", args.train_policy, f"iteration{i}"))
                os.makedirs(save_dir, exist_ok=True)
                checkpoint = train.save(save_dir)


        save_dir = os.path.abspath(os.path.join("policies", args.train_policy))
        os.makedirs(save_dir, exist_ok=True)
        checkpoint = train.save(save_dir)


        print(f"Checkpoint saved to: {checkpoint.checkpoint.path}")

