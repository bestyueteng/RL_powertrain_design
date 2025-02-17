import os
os.environ['KMP_DUPLICATE_LIB_OK'] = 'True'
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
import numpy as np
# from powertrain_topology_rl.env.powertrain_env import PowertrainEnv
from powertrain_topology_rl.utils import *
import logging
import sys
import matplotlib.pyplot as plt
from stable_baselines3.common.env_util import make_vec_env
import gymnasium as gym
from stable_baselines3.common.callbacks import BaseCallback, CallbackList
from wandb.integration.sb3 import WandbCallback

# Define the LoggerWriter if not already defined in powertrain_topology_rl.utils
class LoggerWriter:
    def __init__(self, logger, log_method):
        self.logger = logger
        self.log_method = log_method

    def write(self, message):
        if message.strip() != "":
            self.log_method(message)

    def flush(self):
        pass

# Configure logging to write to a file
file_path = 'output_log.txt'
if os.path.exists(file_path):
    os.remove(file_path)
logging.basicConfig(filename='output_log.txt', level=logging.INFO)

# Redirect stdout to logging (wrap print statements)
sys.stdout = LoggerWriter(logging.getLogger(), logging.info)

# Example data (same as before)
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

# Define the environment creation function
def env_fn():
    return gym.make(
        "powertrain_topology_rl/PowertrainEnv-v0",
        component_library=component_library,
        component_class=component_class,
        component_type=component_type,
    )

# Check the environment (on a single instance)
env = env_fn()
check_env(env)


config = {
    "policy_type": "MultiInputPolicy",
    "total_timesteps": 1000,
    "env_id": "powertrain_topology_rl/PowertrainEnv-v0",
}

# Define the custom plotting callback
class PlottingCallback(BaseCallback):
    """
    Custom callback for plotting training rewards.
    """
    def __init__(self, verbose=0):
        super(PlottingCallback, self).__init__(verbose)
        self.episode_rewards = []
        self.episode_lengths = []

    def _on_step(self) -> bool:
        # Check if a new episode has started
        infos = self.locals.get('infos', [])
        for info in infos:
            if 'episode' in info.keys():
                episode_info = info['episode']
                self.episode_rewards.append(episode_info['r'])
                self.episode_lengths.append(episode_info['l'])
                # if self.verbose > 0:
                    # print(f"Episode {len(self.episode_rewards)}: Reward={episode_info['r']}, Length={episode_info['l']}")
        return True  # Returning False will stop training early

# Define the stop training callback
class StopTrainingCallback(BaseCallback):
    def __init__(self, reward_threshold: float, n_episodes: int = 200, verbose=0):
        super(StopTrainingCallback, self).__init__(verbose)
        self.reward_threshold = reward_threshold
        self.n_episodes = n_episodes
        self.episode_rewards = []

    def _on_step(self) -> bool:
        infos = self.locals.get('infos', [])
        for info in infos:
            if 'episode' in info.keys():
                episode_reward = info['episode']['r']
                self.episode_rewards.append(episode_reward)
                if len(self.episode_rewards) >= self.n_episodes:
                    avg_reward = np.mean(self.episode_rewards[-self.n_episodes:])
                    # if self.verbose > 0:
                    #     print(f"Average reward over last {self.n_episodes} episodes: {avg_reward}")
                    if avg_reward >= self.reward_threshold:
                        # if self.verbose > 0:
                        #     print(f"Average reward threshold of {self.reward_threshold} reached. Stopping training.")
                        return False
        return True


# Instantiate the PPO model
model = PPO(config["policy_type"], env, verbose=1)

# Instantiate the callbacks
plotting_callback = PlottingCallback(verbose=1)
stop_training_callback = StopTrainingCallback(reward_threshold=0.7, verbose=1)

# Combine both callbacks into a CallbackList
callback = CallbackList([plotting_callback, stop_training_callback])

# Train the model with the combined callbacks
model.learn(
    total_timesteps=config["total_timesteps"],
    callback=callback
)

print("Training finished.")

# Save the model
model.save("ppo_powertrain")

# Plotting the training results
def plot_training_results(callback):
    episodes = range(1, len(callback.episode_rewards) + 1)
    
    plt.figure(figsize=(12, 5))

    # Plot Episode Rewards
    plt.subplot(1, 2, 1)
    plt.plot(episodes, callback.episode_rewards, label='Episode Reward')
    plt.xlabel('Episode')
    plt.ylabel('Total Reward')
    plt.title('Training Episode Rewards')
    plt.legend()

    # Plot Episode Lengths
    plt.subplot(1, 2, 2)
    plt.plot(episodes, callback.episode_lengths, label='Episode Length', color='orange')
    plt.xlabel('Episode')
    plt.ylabel('Length')
    plt.title('Training Episode Lengths')
    plt.legend()

    plt.tight_layout()
    plt.show()

# Call the plotting function
plot_training_results(plotting_callback)

# Test the trained agent
test_env = env
obs, _ = test_env.reset()
done = False
for _ in range(2000):
    action, _states = model.predict(obs, deterministic=True)
    print("Action:", action)
    obs, reward, done, truncated, info = test_env.step(action)
    if np.any(done):
        test_env.render()
        break

# Restore stdout for console output
sys.stdout = sys.__stdout__
