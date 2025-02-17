from stable_baselines3.common.callbacks import BaseCallback
import wandb
from wandb.integration.sb3 import WandbCallback
import os
import numpy as np
import torch as th 

# Custom callback to log additional metrics
class CustomWandbCallback(BaseCallback):
    def __init__(self, verbose=0):
        super(CustomWandbCallback, self).__init__(verbose)
        self.episode_rewards = []

    def _on_step(self):
        done = self.locals.get("dones")[0]
        rewards = self.locals.get("rewards")
        
        # Safely extract the reward
        if isinstance(rewards, th.Tensor):
            reward = rewards.item() if rewards.dim() == 0 else rewards[0].item()
        elif isinstance(rewards, np.ndarray):
            reward = rewards.item() if rewards.ndim == 0 else rewards[0]
        elif isinstance(rewards, (list, tuple)):
            reward = rewards[0] if len(rewards) > 0 else 0.0
        else:
            reward = rewards  # Assume scalar
            
        # reward = self.locals.get("rewards")[0]
        
        self.episode_rewards.append(reward)

        if done:
            episode_reward = sum(self.episode_rewards)
            episode_length = len(self.episode_rewards)
            wandb.log({
                "episode_reward": episode_reward,
                "episode_length": episode_length,
                "epsilon": getattr(self.model, 'exploration_rate', None),
            }, step=self.num_timesteps)
            self.episode_rewards = []

        return True
    

def init_wanb(args):
    
    run = wandb.init(
        project=args.wandb_project,
        config=args,
        name=args.wandb_run_name,
        sync_tensorboard=True,  # auto-upload sb3's tensorboard metrics
    )
    
    # Set up wandb callback
    wandb_callback = WandbCallback(
        gradient_save_freq=100,
        model_save_freq=5000,
        model_save_path=os.path.join(args.model_save_path, wandb.run.id),
        verbose=2,
    )

    return run, wandb_callback