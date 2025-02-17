import sys
import gymnasium as gym
from stable_baselines3 import DQN, HerReplayBuffer
from stable_baselines3.common.callbacks import CallbackList
from training.callback.stop_training_callback import StopTrainingCallback
from training.callback.intrinsic_reward_callback import RLeXploreWithOffPolicyRL
from powertrain_topology_rl.utils import *
from powertrain_topology_rl.env import PowertrainEnv_her, PowertrainEnv, PowertrainEnv_powerflow
from training.callback.wanb_callback import CustomWandbCallback
from stable_baselines3.common.env_checker import check_env
import numpy as np
import os
from training.feature_extractor import *
from powertrain_topology_rl.wrapper import *
from stable_baselines3.common.vec_env import DummyVecEnv
from rllte_fork.rllte.xplore.reward import ICM,RIDE,E3B
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.callbacks import EvalCallback, StopTrainingOnRewardThreshold
from stable_baselines3.common.vec_env import VecNormalize
from sb3_contrib import RecurrentPPO
from stable_baselines3.her.goal_selection_strategy import GoalSelectionStrategy

def training_dqn(args, component_library, component_class, component_type, wandb_callback, run, logger):
    best_dsm = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0],
       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
       [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0],
       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
       [0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0],
       [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
       [1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
       [0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])
       
    goal_selection_strategy = "future"
    if args.env_id == "powertrain_topology_rl/PowertrainEnv_her-v0":
        env_name = PowertrainEnv_her
    elif args.env_id == "powertrain_topology_rl/PowertrainEnv-v0":
        env_name = PowertrainEnv
    else:
        env_name = PowertrainEnv_powerflow
        
    if args.num_split > 1:
        envs = []
        for i in range (args.num_split):
            if args.max_components == 0:
                def env_fn():
                    return env_name(
                                    component_library=component_library,
                                    component_class=component_class,
                                    component_type=component_type,
                                    random_dsm=(i+1)*(1/args.num_split),
                                    desired_dsm=best_dsm
                                    )
                # env = gym.make(
                        # args.env_id,
                        # component_library=component_library,
                        # component_class=component_class,
                        # component_type=component_type,
                        # random_dsm=(i+1)*(1/args.num_split),
                        # #init_dsm=best_dsm,
                        # #desired_dsm=best_dsm
                # )
            else:
                def env_fn():
                    return env_name(
                                    component_library=component_library,
                                    component_class=component_class,
                                    component_type=component_type,
                                    random_dsm=(i+1)*(1/args.num_split),
                                    max_val=args.max_components
                                    )
                # env = gym.make(
                        # args.env_id,
                        # component_library=component_library,
                        # component_class=component_class,
                        # component_type=component_type,
                        # random_dsm=(i+1)*(1/args.num_split),
                        # max_val=args.max_components
                # )
            env_check = env_fn()
            check_env(env_check)
            env = make_vec_env(env_fn, args.n_envs)
            envs.append(env)
    else:
        if args.max_components == 0:
            def env_fn():
                return env_name(
                                component_library=component_library,
                                component_class=component_class,
                                component_type=component_type,
                                random_dsm=1,
                                desired_dsm=best_dsm
                                )
            # env = gym.make(
                    # args.env_id,
                    # component_library=component_library,
                    # component_class=component_class,
                    # component_type=component_type,
                    # random_dsm=1
            # )
        else:
            def env_fn():
                return env_name(
                                component_library=component_library,
                                component_class=component_class,
                                component_type=component_type,
                                random_dsm=1,
                                max_val=args.max_components
                                )
            # env = gym.make(
                        # args.env_id,
                        # component_library=component_library,
                        # component_class=component_class,
                        # component_type=component_type,
                        # random_dsm=1,
                        # max_val=args.max_components
                # )
        # check_env(env)
        env_check = env_fn()
        check_env(env_check)
        env = make_vec_env(env_fn, args.n_envs)
        
        envs = [env]
    
    if args.curiosity=="stratch":
        envs = [CuriosityWrapper(DummyVecEnv([lambda: env_])) for env_ in envs]
    if args.curiosity == "icm" or args.curiosity == "ride" or args.curiosity == "e3b":
        envs = [DummyVecEnv([lambda: env_]) for env_ in envs]
    
    
    # envs = [VecNormalize(env_, clip_reward=100.0) for env_ in envs]
    for idx, env in enumerate(envs):
            
        # check if model exist   
        if os.path.exists(args.local_model_save_name):
            model = DQN.load(args.local_model_save_name, env=env)
            logger.info("Model loaded.")

        else:
            if args.extract_features == "gnn":
                policy_kwargs = dict(
                        features_extractor_class=CustomCombinedExtractor_GNN,
                        )
            elif args.extract_features == "mlp":
                policy_kwargs = dict(
                        features_extractor_class=CustomCombinedExtractor_MLP,
                        )
            else:
                policy_kwargs = None
            
            if args.policy_type == "MultiInputPolicy":
                model = DQN(
                    args.policy_type,
                    env,
                    train_freq=(1, "episode"),
                    learning_rate=args.learning_rate,
                    policy_kwargs=policy_kwargs,
                    replay_buffer_class=HerReplayBuffer,
                    # Parameters for HER
                    replay_buffer_kwargs=dict(
                        n_sampled_goal=4,
                        goal_selection_strategy=goal_selection_strategy,
                    ),
                    verbose=1,
                    tensorboard_log=f"runs/{run.id}",
                    device="cpu" if args.policy_type == "MlpPolicy" else "auto"
                    )
            else:
                model = DQN(
                    args.policy_type,
                    env,
                    train_freq=(1, "episode"),
                    learning_rate=args.learning_rate,
                    policy_kwargs=policy_kwargs,
                    # replay_buffer_class=HerReplayBuffer,
                    # # Parameters for HER
                    # replay_buffer_kwargs=dict(
                        # n_sampled_goal=4,
                        # goal_selection_strategy=goal_selection_strategy,
                    # ),
                    verbose=1,
                    tensorboard_log=f"runs/{run.id}",
                    device="cpu" if args.policy_type == "MlpPolicy" else "auto"
                    )

        
        callback_list = [wandb_callback, CustomWandbCallback()]
        
        if args.stop_training_threshold == 1:
            # Separate evaluation env
            eval_env = gym.make(
                        args.env_id,
                        component_library=component_library,
                        component_class=component_class,
                        component_type=component_type,
                        random_dsm=(idx+1)*(1/args.num_split),
                        init_dsm=best_dsm
                )
            eval_env = DummyVecEnv([lambda: eval_env])
            # Stop training when the model reaches the reward threshold
            callback_on_best = StopTrainingOnRewardThreshold(reward_threshold=args.stop_training_rate, verbose=0)
            eval_callback = EvalCallback(eval_env,eval_freq=1000, callback_on_new_best=callback_on_best, verbose=0)
            callback_list.append(eval_callback)
            # stop_training_callback = StopTrainingCallback(reward_threshold=args.stop_training_rate,n_episodes=args.stop_training_n_episodes, verbose=1)
            # callback_list.append(stop_training_callback)
            # callback = CallbackList([stop_training_callback,wandb_callback, CustomWandbCallback()])
        if args.curiosity=="ride":
            
            irs = RIDE(env,obs_shape=env.observation_space.shape, device='cuda')
            callback_list.append(RLeXploreWithOffPolicyRL(irs))
        elif args.curiosity=="icm":
            irs = ICM(env,obs_shape=env.observation_space.shape, device='cuda')
            callback_list.append(RLeXploreWithOffPolicyRL(irs))
        elif args.curiosity=="e3b":
            irs = E3B(env,obs_shape=env.observation_space.shape, device='cuda')
            callback_list.append(RLeXploreWithOffPolicyRL(irs))
            
        callback = CallbackList(callback_list)
          
        # Train the model  
        model.learn(
            total_timesteps=args.total_timesteps // (args.num_split - idx), 
            callback=callback
            )
        logger.info("Training finished.")
        model.save(args.local_model_save_name)

    return model