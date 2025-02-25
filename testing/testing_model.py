import gymnasium as gym
from powertrain_topology_rl.utils import *
import os
import sys
import argparse
from sb3_contrib import RecurrentPPO
from sb3_contrib.common.maskable.utils import get_action_masks

def testing_model(args, component_library, component_class, component_type, model):
    """
    Tests the model within the specified environment and logs actions to a file.

    Args:
        args: Arguments containing environment ID and other configurations.
        component_library: Library of components to use.
        component_class: Class of components.
        component_type: Type of components.
        model: The model to be tested.
    """
    log_file = args.test_log_file
    
    try:
        with open(log_file, 'w') as f:
            f.write("Model testing started.\n")
            
            if args.env_id == "powertrain_topology_rl/PowertrainEnv_powerflow-v0" or args.env_id == "powertrain_topology_rl/PowertrainEnv_powerflow_withmask-v0":
                test_env = gym.make(
                    args.env_id,
                    component_library=component_library,
                    component_class=component_class,
                    component_type=component_type,
                )
            elif args.env_id == "powertrain_topology_rl/PowertrainEnv_overall-v0":
                test_env = gym.make(
                    args.env_id,
                    component_library=component_library,
                    component_class=component_class,
                    component_type=component_type,
                    required_instances=[1, 2, 3, 3, 3, 1, 1, 1, 1],
                    performance_req = np.array([8, 200, 0.1993, 0.1, 50])
                )
            else:
                test_env = gym.make(
                    args.env_id,
                    component_library=component_library,
                    component_class=component_class,
                    component_type=component_type,
                )


            obs, _ = test_env.reset()
            f.write("Environment reset successfully.\n")
            
            if args.env_id != "powertrain_topology_rl/PowertrainEnv_powerflow_withmask-v0" and args.env_id != "powertrain_topology_rl/PowertrainEnv_overall-v0":
                for step in range(1, 50):
                    # Predict action using the model
                    action, _state = model.predict(obs, deterministic=True)
                    f.write(f"Step {step}: Action predicted - {action}\n")
                    
                    # Take a step in the environment
                    obs, reward, done, truncated, info = test_env.step(action)
                    f.write(f"Step {step}: Reward received - {reward}\n")
                    
                    if done or truncated:
                        f.write(f"Environment done at step {step}. Rendering and exiting.\n")
                        test_env.render()
                        break
            else:
                for step in range(1, 50):
                    # Predict action using the model
                    action_masks = get_action_masks(test_env)
                    action, _state = model.predict(obs, action_masks=action_masks)
                    f.write(f"Step {step}: Action predicted - {action}\n")
                    
                    # Take a step in the environment
                    obs, reward, done, truncated, info = test_env.step(action)
                    f.write(f"Step {step}: Reward received - {reward}\n")
                    
                    if done or truncated:
                        f.write(f"Environment done at step {step}. Rendering and exiting.\n")
                        test_env.render()
                        break
                    
    except Exception as e:
        with open(log_file, 'a') as f:
            f.write(f"An error occurred during testing: {e}\n")
    finally:
        with open(log_file, 'a') as f:
            f.write("Model testing completed.\n")

def parse_arguments():
    parser = argparse.ArgumentParser(description="Train RL agent for Powertrain Topology")
    parser.add_argument('--test_log_file', type=str, default='test_output_log.txt', help='Test Log file name')
    parser.add_argument('--algorithm', type=str, default="DQN", help='RL algorithm to use')
    
    return parser.parse_args()

if __name__ == "__main__":
    component_library = {
            'Component number': [1, 2, 2, 2, 3, 4, 5, 5, 5],
            'Type of Components': [1, 1, 2, 3, 1, 1, 1, 2, 3],
            'Number of Instances': [1, 1, 1, 1, 2, 1, 1, 1, 1]
        }
    component_class = ["VehicleBody", "GearSystems", "EnergyConverters", "EnergyStorage", "Controller"]
    component_type = [
        "Vehicle body", ["Final Drive", "Simple transmission", "Multispeed gearbox"], 
        ["Electric motor 1"], 
        ["Battery"], 
        ["Torque Split", "Torque Coupler", "Electric Power Link"]
    ]

    args = parse_arguments()
    
    if args.algorithm == "DQN":
        model_calss = DQN
    elif args.algorithm == "PPO":
        model_class = PPO
    elif args.algorithm == "REPPO":
        model_class = RecurrentPPO
        
    test_env = gym.make(
                "powertrain_topology_rl/PowertrainEnv-v0",
                component_library=component_library,
                component_class=component_class,
                component_type=component_type,
            )
    model = model_class.load(args.local_model_save_name, env=test_env)
    testing_model(args, component_library, component_class, component_type, model)
    
    
