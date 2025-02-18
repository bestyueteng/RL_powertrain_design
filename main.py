import os
import sys
import argparse
from powertrain_topology_rl.utils import LoggerWriter
from training import *
from testing import *


def parse_arguments():
    parser = argparse.ArgumentParser(description="Train RL agent for Powertrain Topology")
    parser.add_argument('--policy_type', type=str, default="MultiInputPolicy", help='Policy type for the RL algorithm')
    parser.add_argument('--algorithm', type=str, default="DQN", help='RL algorithm to use')
    parser.add_argument('--total_timesteps', type=int, default=25000, help='Total timesteps for training')
    parser.add_argument('--env_id', type=str, default="powertrain_topology_rl/PowertrainEnv-v0", help='Environment ID')
    parser.add_argument('--n_envs', type=int, default=4, help='Number of parallel environments')
    parser.add_argument('--wandb_project', type=str, default="powertrain_design", help='Wandb project name')
    parser.add_argument('--wandb_run_name', type=str, default=None, help='Wandb run name')
    parser.add_argument('--model_save_path', type=str, default="models/", help='Path to save the model')
    parser.add_argument('--log_file', type=str, default='output_log.txt', help='Log file name')
    parser.add_argument('--test_log_file', type=str, default='test_output_log.txt', help='Test Log file name')
    parser.add_argument('--exploration_final_eps', type=float, default=0.05, help='lower limit of epsilon')
    parser.add_argument('--learning_rate', type=float, default=0.0001, help='lr')
    parser.add_argument('--local_model_save_name', type=str, default="ppo", help='local model save')
    parser.add_argument('--train_freq', type=tuple[int, str], default=(5, "step"), help='Update the model every train_freq steps')
    parser.add_argument('--stop_training_threshold', type=int, default=0, help='stop_training_threshold')
    parser.add_argument('--stop_training_rate', type=float, default=0.7, help='stop_training_rate')
    parser.add_argument('--stop_training_n_episodes', type=int, default=10, help='stop_training_n_episodes')
    parser.add_argument('--num_split', type=int, default=1, help='training split')
    parser.add_argument('--max_components', type=int, default=0, help='max_components')
    parser.add_argument('--extract_features', type=str, default=None, help='extract_features')
    parser.add_argument('--curiosity', type=str, default=None, help='curiosity')
    parser.add_argument('--activation', type=str, default="relu", help='activation')
    parser.add_argument('--n_steps', type=int, default=256, help='n_steps')
    parser.add_argument('--ent_coef', type=float, default=0.0, help='ent_coef')

    return parser.parse_args()



def main():
    args = parse_arguments()
    
    logger = setup_logging(args.log_file)
    logger.info("Starting training script.")
    
    # Redirect print statements to logging
    sys.stdout = LoggerWriter(logger, logger.info)
    sys.stderr = LoggerWriter(logger, logger.error)
    
    # # Initialize Wandb
    run, wandb_callback = init_wanb(args)

    # Example data (as before)
    max_components = args.max_components
    if max_components == 0:
        component_library = {
            'Component number': [1, 2, 2, 2, 3, 4, 5, 5, 5],
            'Type of Components': [1, 1, 2, 3, 1, 1, 1, 2, 3],
            'Number of Instances': [1, 3, 4, 4, 4, 1, 1, 1, 1]
        }
    else:
        component_library = {
            'Component number': [1, 2, 2, 2, 3, 4, 5, 5, 5],
            'Type of Components': [1, 1, 2, 3, 1, 1, 1, 2, 3],
            'Number of Instances': [1, max_components, max_components, max_components, max_components, max_components, 1, 1, 1]
        }
    
    component_class = ["VehicleBody", "GearSystems", "EnergyConverters", "EnergyStorage", "Controller"]
    component_type = [
        "Vehicle body", ["Final Drive", "Simple transmission", "Multispeed gearbox"], 
        ["Electric motor 1"], 
        ["Battery"], 
        ["Torque Split", "Torque Coupler", "Electric Power Link"]
    ]
    
    if os.path.exists(args.local_model_save_name):
        os.remove(args.local_model_save_name)
    # training
    if args.algorithm in ["PPO", "REPPO", "MASKPPO"]:
        model = training_ppo(args, component_library, component_class, component_type, wandb_callback, run, logger)
    elif args.algorithm == "DQN":
        model = training_dqn(args, component_library, component_class, component_type, wandb_callback, run, logger)
    # elif args.algorithm == "rainbow":
        # training_rainbow(args, component_library, component_class, component_type, wandb_callback, run, logger)
    else:
        print("No supported algorithm")
        
    # Save the model
    model.save(os.path.join(args.model_save_path, run.id))

    logger.info("Training complete.")

    # testing
    if args.algorithm in ["PPO", "DQN", "REPPO", "MASKPPO"]:
        testing_model(args, component_library, component_class, component_type, model)

if __name__ == "__main__":
    main()

    # Restore stdout and stderr
    sys.stdout = sys.__stdout__
    sys.stderr = sys.__stderr__
