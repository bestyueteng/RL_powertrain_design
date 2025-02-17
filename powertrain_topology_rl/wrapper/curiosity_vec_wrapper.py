import torch
import torch.nn as nn
import torch.optim as optim
import gymnasium as gym
from stable_baselines3.common.vec_env import VecEnv, VecEnvWrapper
import numpy as np

class CuriosityModule(nn.Module):
    def __init__(self, obs_dim, action_dim=1, hidden_dim=64):
        super(CuriosityModule, self).__init__()
        # State-action representation

        self.fc1 = nn.Linear(obs_dim + 1, hidden_dim)
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.fc3 = nn.Linear(hidden_dim, obs_dim)  # Predict the next state

        self.optimizer = optim.Adam(self.parameters(), lr=1e-3)

    def forward(self, x):

        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        predicted_next_state = self.fc3(x)
        return predicted_next_state

    def predict(self, curiosity_input_tensor):
        return self.forward(curiosity_input_tensor)

    def compute_intrinsic_reward(self, next_obs, predicted_next_obs):
        # Compute the squared prediction error (PEC)
        return torch.norm(next_obs - predicted_next_obs, p=2, dim=-1).mean()

class CuriosityWrapper(VecEnvWrapper):
    def __init__(self, env, intrinsic_reward_weight=0.5, curiosity_lr=3e-4):
        super(CuriosityWrapper, self).__init__(env)

        # Initialize Curiosity Module and Optimizer
        obs_dim = env.observation_space.shape[0]
        action_dim = 1
        self.curiosity_module = CuriosityModule(obs_dim=obs_dim, action_dim=action_dim)
        self.curiosity_optimizer = torch.optim.Adam(self.curiosity_module.parameters(), lr=curiosity_lr)

        # Intrinsic reward weight (how much influence curiosity has on exploration)
        self.intrinsic_reward_weight = intrinsic_reward_weight

    def reset(self, **kwargs):
        # Reset all environments
        obs = self.venv.reset(**kwargs)
        return obs

    def step_async(self, actions):
        # The VecEnv steps are handled asynchronously, but no specific action is needed here
        self.action = actions
        self.venv.step_async(actions)

    def step_wait(self):
        # Get observations, rewards, dones, truncated, and infos
        obs, reward, done, infos = self.venv.step_wait()

        # Extract only the "DSM" part of the observation
        obs_dsm = torch.tensor(obs, dtype=torch.float32)
        
        curiosity_input = np.append(obs_dsm, self.action)
        curiosity_input_tensor = torch.tensor(curiosity_input, dtype=torch.float32)
        # Extract the actions (convert to tensor if necessary)
        # action_tensor = torch.tensor(self.action, dtype=torch.float32)

        # Get predicted next state and calculate intrinsic reward
        next_obs = torch.tensor(obs, dtype=torch.float32)  # Use "DSM" for the next state
        predicted_next_obs = self.curiosity_module.predict(curiosity_input_tensor)
        
        # Compute the intrinsic reward (Prediction Error Curiosity)
        intrinsic_rewards = self.curiosity_module.compute_intrinsic_reward(next_obs, predicted_next_obs)

        # Combine intrinsic and extrinsic rewards
        if reward < 0:
            combined_rewards = reward
        else:
            combined_rewards = reward + self.intrinsic_reward_weight * intrinsic_rewards.detach().numpy()
            
        # Update curiosity module (using prediction error loss)
        loss = torch.nn.functional.mse_loss(predicted_next_obs, next_obs)
        self.curiosity_optimizer.zero_grad()
        loss.backward()
        self.curiosity_optimizer.step()
        
        # Return the observations, combined rewards, done flags, and truncated flags
        return obs, combined_rewards, done, infos

    def render(self, *args, **kwargs):
        return self.env.render(*args, **kwargs)