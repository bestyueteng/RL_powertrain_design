import sys
import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CallbackList
from training.callback.stop_training_callback import StopTrainingCallback
from training.callback.intrinsic_reward_callback import RLeXploreWithOnPolicyRL
from powertrain_topology_rl.utils import *
from training.callback.wanb_callback import CustomWandbCallback
from stable_baselines3.common.env_checker import check_env
import numpy as np
import os
from training.feature_extractor import *
from powertrain_topology_rl.wrapper import *
from stable_baselines3.common.vec_env import DummyVecEnv
from rllte_fork.rllte.xplore.reward import ICM,RIDE,E3B, RND
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.callbacks import EvalCallback, StopTrainingOnRewardThreshold, CheckpointCallback
from stable_baselines3.common.vec_env import VecNormalize
from sb3_contrib import RecurrentPPO, MaskablePPO
from stable_baselines3.common.monitor import Monitor

def training_ppo(args, component_library, component_class, component_type, wandb_callback, run, logger):
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
    test_required_instances = [1, 2, 2, 2, 2, 2, 1, 1, 1]
       
    if args.activation == 'elu':
        activation_fn = th.nn.ELU
    elif args.activation == 'leakyrelu':
        activation_fn = th.nn.LeakyReLU
    else:
        activation_fn = th.nn.ReLU
        
    if args.num_split > 1:
        envs = []
        for i in range (args.num_split):
            env = gym.make(
                    args.env_id,
                    component_library=component_library,
                    component_class=component_class,
                    component_type=component_type,
                    random_dsm=(i+1)*(1/args.num_split)
                    # init_dsm = best_dsm
            )

            check_env(env)
            # env = make_vec_env(env, n_envs=8)
            envs.append(env)
    else:
        env = gym.make(
                args.env_id,
                component_library=component_library,
                component_class=component_class,
                component_type=component_type,
                random_dsm=1
        )

        # env = make_vec_env(env, n_envs=8)
        # check_env(env)
        envs = [env]
    
    if args.curiosity=="stratch":
         envs = [CuriosityWrapper(DummyVecEnv([lambda: env_])) for env_ in envs]
         # envs = [CuriosityWrapper(env_) for env_ in envs]
    if args.curiosity in ["icm", "ride","e3b", "rnd"]:
        envs = [DummyVecEnv([lambda: env_]) for env_ in envs]
        # envs = [CuriosityWrapper(env_) for env_ in envs]
    
    # envs = [VecNormalize(env_, clip_reward=100.0) for env_ in envs]
    for idx, env in enumerate(envs):
            
        # check if model exist   
        if os.path.exists(args.local_model_save_name):
            print("Continue training")
            if args.algorithm == "PPO":
                model = PPO.load(args.local_model_save_name, env=env)
                logger.info("Model loaded.")
            elif args.algorithm == "REPPO":
                model = RecurrentPPO.load(args.local_model_save_name, env=env)
                logger.info("Model loaded.")
            elif args.algorithm == "MASKPPO":
                model = RecurrentPPO.load(args.local_model_save_name, env=env)
                logger.info("Model loaded.")
        else:
            # Feature extractor module
            if args.extract_features == "gnn":
                features_extractor_class=CustomCombinedExtractor_GNN
            elif args.extract_features == "mlp":
                features_extractor_class=CustomCombinedExtractor_MLP
            else:
                features_extractor_class = None
                
            policy_kwargs = dict(features_extractor_class=features_extractor_class, activation_fn=activation_fn, net_arch=dict(pi=[64,128,128], vf=[64,128,128]))
            
            # Base algorithm
            if args.algorithm == "PPO":
                model = PPO(
                    args.policy_type,
                    env,
                    n_steps=args.n_steps,
                    ent_coef=args.ent_coef,
                    learning_rate=args.learning_rate,
                    policy_kwargs=policy_kwargs,
                    verbose=1,
                    tensorboard_log=f"runs/{run.id}",
                    device="cpu" if args.policy_type == "MlpPolicy" else "auto"
                    )
            elif args.algorithm == "REPPO":
                model = RecurrentPPO(
                    "MlpLstmPolicy",
                    env,
                    n_steps=args.n_steps,
                    ent_coef=args.ent_coef,
                    learning_rate=args.learning_rate,
                    policy_kwargs=policy_kwargs,
                    verbose=1,
                    tensorboard_log=f"runs/{run.id}",
                    device="cpu" if args.policy_type == "MlpPolicy" else "auto"
                    )
            elif args.algorithm == "MASKPPO":
                model = MaskablePPO(
                        args.policy_type,
                        env,
                        learning_rate=args.learning_rate,
                        policy_kwargs=policy_kwargs,
                        gamma=0.4, 
                        seed=32,
                        n_steps=args.n_steps,
                        ent_coef=args.ent_coef,
                        tensorboard_log=f"runs/{run.id}",
                        verbose=1,
                    )
        
        callback_list = [wandb_callback, CustomWandbCallback()]
        
        # Envaluation
        if args.stop_training_threshold == 1:
            # Separate evaluation env
             if args.env_id != "powertrain_topology_rl/PowertrainEnv_overall-v0":
                 eval_env = gym.make(
                             args.env_id,
                             component_library=component_library,
                             component_class=component_class,
                             component_type=component_type,
                             random_dsm=(idx+1)*(1/args.num_split),
                             init_dsm=best_dsm
                     )
             else:
                eval_env = gym.make(
                             args.env_id,
                             component_library=component_library,
                             component_class=component_class,
                             component_type=component_type,
                             random_dsm=(idx+1)*(1/args.num_split),
                             # init_dsm=best_dsm,
                             required_instances=test_required_instances
                     )
             monitored_eval_env = Monitor(eval_env)
             eval_env = DummyVecEnv([lambda: eval_env])
             
             # Stop training when the model reaches the reward threshold
             # callback_on_best = StopTrainingOnRewardThreshold(reward_threshold=args.stop_training_rate, verbose=1)
             # eval_callback = EvalCallback(eval_env,eval_freq=20000, callback_on_new_best=callback_on_best, verbose=1)
             # callback_list.append(eval_callback)
             stop_training_callback = StopTrainingCallback(reward_threshold=args.stop_training_rate,n_episodes=args.stop_training_n_episodes, verbose=1)
             callback_list.append(stop_training_callback)
             # callback = CallbackList([stop_training_callback,wandb_callback, CustomWandbCallback()])
             
        # Save checkpoint model
        checkpoint_callback = CheckpointCallback(
                              save_freq=100000,
                              save_path="./check_models/",
                              name_prefix=args.algorithm)
        callback_list.append(checkpoint_callback)
        
        # Curiosity model selection          
        if args.curiosity=="ride":
            irs = RIDE(env,obs_shape=env.observation_space.shape, device='cpu')
            callback_list.append(RLeXploreWithOnPolicyRL(irs))
        elif args.curiosity=="icm":
            irs = ICM(env,obs_shape=env.observation_space["DSM"].shape, device='cpu')
            callback_list.append(RLeXploreWithOnPolicyRL(irs))
        elif args.curiosity=="e3b":
            irs = E3B(env,obs_shape=env.observation_space.shape, device='cpu')
            callback_list.append(RLeXploreWithOnPolicyRL(irs))
        elif args.curiosity=="rnd":
            irs = RND(env,obs_shape=env.observation_space.shape, device='cpu')
            callback_list.append(RLeXploreWithOnPolicyRL(irs))
        callback = CallbackList(callback_list)
          
        # Train the model  
        model.learn(
            total_timesteps=args.total_timesteps, 
            callback=callback
            )
        logger.info("Training finished.")
        model.save(args.local_model_save_name)

    return model