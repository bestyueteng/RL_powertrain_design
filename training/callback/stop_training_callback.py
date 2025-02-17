from stable_baselines3.common.callbacks import BaseCallback
import numpy as np

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
                        print(f"Average reward threshold of {self.reward_threshold} reached. Stopping training.")
                        return False
        return True