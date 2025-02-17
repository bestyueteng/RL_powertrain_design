from stable_baselines3.common.base_class import BaseAlgorithm
from stable_baselines3.common.callbacks import BaseCallback
import torch as th

class RLeXploreWithOnPolicyRL(BaseCallback):
    """
    A custom callback for combining RLeXplore and on-policy algorithms from SB3.
    """
    def __init__(self, irs, verbose=0):
        super(RLeXploreWithOnPolicyRL, self).__init__(verbose)
        self.irs = irs
        self.buffer = None
        self.device = "cpu"
    def init_callback(self, model: BaseAlgorithm) -> None:
        super().init_callback(model)
        self.buffer = self.model.rollout_buffer

    def _on_step(self) -> bool:
        """
        This method will be called by the model after each call to `env.step()`.

        :return: (bool) If the callback returns False, training is aborted early.
        """

        observations = th.as_tensor(self.locals["obs_tensor"]["DSM"], device=self.device).float()
        actions = th.as_tensor(self.locals["actions"], device=self.device).float()
        rewards = th.as_tensor(self.locals["rewards"], device=self.device).float()
        dones = th.as_tensor(self.locals["dones"], device=self.device).float()
        next_observations = th.as_tensor(self.locals["new_obs"]["DSM"], device=self.device).float()


        # ===================== watch the interaction ===================== #
        self.irs.watch(observations, actions, rewards, dones, dones, next_observations)
        # ===================== watch the interaction ===================== #
        return True

    def _on_rollout_end(self) -> None:
        # ===================== compute the intrinsic rewards ===================== #
        # prepare the data samples
        obs = th.as_tensor(self.buffer.observations["DSM"], dtype=th.float32, device=self.device)
        # get the new observations
        new_obs = obs.clone()
        new_obs[:-1] = obs[1:]
        new_obs[-1] = th.as_tensor(self.locals["new_obs"]["DSM"], dtype=th.float32, device=self.device)
        actions = th.as_tensor(self.buffer.actions, dtype=th.float32, device=self.device)
        rewards = th.as_tensor(self.buffer.rewards, dtype=th.float32, device=self.device)
        dones = th.as_tensor(self.buffer.episode_starts, dtype=th.float32, device=self.device)
        # print(obs.shape, actions.shape, rewards.shape, dones.shape, new_obs.shape)
        # print(obs)
        # compute the intrinsic rewards
        intrinsic_rewards = self.irs.compute(
            samples=dict(observations=obs, actions=actions, 
                         rewards=rewards, terminateds=dones, 
                         truncateds=dones, next_observations=new_obs),
            sync=True)
        # add the intrinsic rewards to the buffer
        # intrinsic_rewards = 0.05*intrinsic_rewards
        self.buffer.advantages += intrinsic_rewards.cpu().numpy()
        self.buffer.returns += intrinsic_rewards.cpu().numpy()
        
class RLeXploreWithOffPolicyRL(BaseCallback):
    """
    A custom callback for combining RLeXplore and off-policy algorithms from SB3. 
    """
    def __init__(self, irs, verbose=0):
        super(RLeXploreWithOffPolicyRL, self).__init__(verbose)
        self.irs = irs
        self.buffer = None
        self.device = "cpu"
    def init_callback(self, model: BaseAlgorithm) -> None:
        super().init_callback(model)
        self.buffer = self.model.replay_buffer
        

    def _on_step(self) -> bool:
        """
        This method will be called by the model after each call to `env.step()`.

        :return: (bool) If the callback returns False, training is aborted early.
        """
        obs = th.as_tensor(self.locals['self']._last_obs,dtype=th.float32, device=self.device)
        actions = th.as_tensor(self.locals["actions"],dtype=th.float32, device=self.device)
        rewards = th.as_tensor(self.locals["rewards"],dtype=th.float32, device=self.device)
        dones = th.as_tensor(self.locals["dones"],dtype=th.float32, device=self.device)
        next_obs = th.as_tensor(self.locals["new_obs"],dtype=th.float32, device=self.device)

        # ===================== watch the interaction ===================== #
        self.irs.watch(obs, actions, rewards, dones, dones, next_obs)
        # ===================== watch the interaction ===================== #
        
        # ===================== compute the intrinsic rewards ===================== #
        intrinsic_rewards = self.irs.compute(samples={'observations':obs.unsqueeze(0), 
                                            'actions':actions.unsqueeze(0), 
                                            'rewards':rewards.unsqueeze(0),
                                            'terminateds':dones.unsqueeze(0),
                                            'truncateds':dones.unsqueeze(0),
                                            'next_observations':next_obs.unsqueeze(0)}, 
                                            sync=False)
        # ===================== compute the intrinsic rewards ===================== #

        try:
            # add the intrinsic rewards to the original rewards
            self.locals['rewards'] += intrinsic_rewards.cpu().numpy().squeeze()
            # update the intrinsic reward module
            replay_data = self.buffer.sample(batch_size=self.irs.batch_size)
            self.irs.update(samples={'observations': th.as_tensor(replay_data.observations).unsqueeze(1).to(self.device), # (n_steps, n_envs, *obs_shape)
                                     'actions': th.as_tensor(replay_data.actions).unsqueeze(1).to(self.device),
                                     'rewards': th.as_tensor(replay_data.rewards).to(self.device),
                                     'terminateds': th.as_tensor(replay_data.dones).to(self.device),
                                     'truncateds': th.as_tensor(replay_data.dones).to(self.device),
                                     'next_observations': th.as_tensor(replay_data.next_observations).unsqueeze(1).to(self.device)
                                     })
        except:
            pass

        return True

    def _on_rollout_end(self) -> None:
        pass