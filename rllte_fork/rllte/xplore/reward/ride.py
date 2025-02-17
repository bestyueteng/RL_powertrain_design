# =============================================================================
# MIT License

# Copyright (c) 2024 Reinforcement Learning Evolution Foundation

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
# =============================================================================


from typing import Dict, List, Optional

import numpy as np
import torch as th
import torch.nn.functional as F
from gymnasium.vector import VectorEnv
from torch import nn
from torch.utils.data import DataLoader, TensorDataset

from rllte.common.prototype import BaseReward
from rllte.common.utils import TorchRunningMeanStd
from .model import ForwardDynamicsModel, InverseDynamicsModel, ObservationEncoder


class RIDE(BaseReward):
    """RIDE: Rewarding Impact-Driven Exploration for Procedurally-Generated Environments.
        See paper: https://arxiv.org/pdf/2002.12292

    Args:
        envs (VectorEnv): The vectorized environments.
        device (str): Device (cpu, cuda, ...) on which the code should be run.
        beta (float): The initial weighting coefficient of the intrinsic rewards.
        kappa (float): The decay rate of the weighting coefficient.
        gamma (Optional[float]): Intrinsic reward discount rate, default is `None`.
        rwd_norm_type (str): Normalization type for intrinsic rewards from ['rms', 'minmax', 'none'].
        obs_norm_type (str): Normalization type for observations data from ['rms', 'none'].

        latent_dim (int): The dimension of encoding vectors.
        lr (float): The learning rate.
        batch_size (int): The batch size for training.
        k (int): Number of neighbors.
        kernel_cluster_distance (float): The kernel cluster distance.
        kernel_epsilon (float): The kernel constant.
        c (float): The pseudo-counts constant.
        sm (float): The kernel maximum similarity.
        update_proportion (float): The proportion of the training data used for updating the forward dynamics models.
        encoder_model (str): The network architecture of the encoder from ['mnih', 'pathak'].
        weight_init (str): The weight initialization method from ['default', 'orthogonal'].

    Returns:
        Instance of RIDE.
    """

    def __init__(
        self,
        envs: VectorEnv,
        obs_shape,
        device: str = "cpu",
        beta: float = 1.0,
        kappa: float = 0.0,
        gamma: float = None,
        rwd_norm_type: str = "rms",
        obs_norm_type: str = "none",
        latent_dim: int = 128,
        lr: float = 0.001,
        batch_size: int = 256,
        # episodic memory
        k: int = 10,
        kernel_cluster_distance: float = 0.008,
        kernel_epsilon: float = 0.0001,
        c: float = 0.001,
        sm: float = 8.0,
        update_proportion: float = 1.0,
        encoder_model: str = "mnih",
        weight_init: str = "orthogonal",
    ) -> None:
        super().__init__(envs, device, beta, kappa, gamma, rwd_norm_type, obs_norm_type)
        # build the encoder, inverse dynamics model and forward dynamics model
        self.obs_shape = obs_shape
        self.encoder = ObservationEncoder(
            obs_shape=self.obs_shape,
            latent_dim=latent_dim,
            encoder_model=encoder_model,
            weight_init=weight_init,
        ).to(self.device)
        self.im = InverseDynamicsModel(
            latent_dim=latent_dim,
            action_dim=self.policy_action_dim,
            encoder_model=encoder_model,
            weight_init=weight_init,
        ).to(self.device)
        self.fm = ForwardDynamicsModel(
            latent_dim=latent_dim,
            action_dim=self.policy_action_dim,
            encoder_model=encoder_model,
            weight_init=weight_init,
        ).to(self.device)
        # set the loss function
        if self.action_type == "Discrete":
            self.im_loss = nn.CrossEntropyLoss(reduction="none")
        else:
            self.im_loss = nn.MSELoss(reduction="none")
        # set the optimizers
        self.encoder_opt = th.optim.Adam(self.encoder.parameters(), lr=lr)
        self.im_opt = th.optim.Adam(self.im.parameters(), lr=lr)
        self.fm_opt = th.optim.Adam(self.fm.parameters(), lr=lr)
        # set the parameters
        self.batch_size = batch_size
        self.update_proportion = update_proportion
        self.k = k
        self.kernel_cluster_distance = kernel_cluster_distance
        self.kernel_epsilon = kernel_epsilon
        self.c = c
        self.sm = sm
        # build the episodic memory
        self.episodic_memory = [[] for _ in range(self.n_envs)]
        self.n_eps = [[] for _ in range(self.n_envs)]
        # rms for the intrinsic rewards
        self.dist_rms = TorchRunningMeanStd(shape=(1,), device=self.device)
        self.squared_distances = []
        # temporary buffers for intrinsic rewards and observations
        self.irs_buffer = []
        self.obs_buffer = []

    def pseudo_counts(
        self, embeddings: th.Tensor, memory: List[th.Tensor]
    ) -> th.Tensor:
        """Pseudo counts.

        Args:
            embeddings (th.Tensor): Encoded observations.
            memory (List[th.Tensor]): Episodic memory.

        Returns:
            Conut values.
        """
        memory = th.stack(memory)
        dist = (th.norm(embeddings - memory, p=2, dim=1).sort().values[: self.k]) ** 2
        self.squared_distances.append(dist)
        dist = dist / (self.dist_rms.mean + 1e-8)
        dist = th.maximum(dist - self.kernel_cluster_distance, th.zeros_like(dist))
        kernel = self.kernel_epsilon / (dist + self.kernel_epsilon)
        s = th.sqrt(kernel.sum()) + self.c

        if th.isnan(s) or s > self.sm:
            return 0.0
        else:
            return 1.0 / s

    def watch(
        self,
        observations: th.Tensor,
        actions: th.Tensor,
        rewards: th.Tensor,
        terminateds: th.Tensor,
        truncateds: th.Tensor,
        next_observations: th.Tensor,
    ) -> Optional[Dict[str, th.Tensor]]:
        """Watch the interaction processes and obtain necessary elements for reward computation.

        Args:
            observations (th.Tensor): Observations data with shape (n_envs, *obs_shape).
            actions (th.Tensor): Actions data with shape (n_envs, *action_shape).
            rewards (th.Tensor): Extrinsic rewards data with shape (n_envs).
            terminateds (th.Tensor): Termination signals with shape (n_envs).
            truncateds (th.Tensor): Truncation signals with shape (n_envs).
            next_observations (th.Tensor): Next observations data with shape (n_envs, *obs_shape).

        Returns:
            Feedbacks for the current samples.
        """
        with th.no_grad():
            # data shape of embeddings: (n_envs, latent_dim)
            observations = self.normalize(observations)
            embeddings = self.encoder(observations)
            for i in range(self.n_envs):
                if len(self.episodic_memory[i]) > 0:
                    # compute pseudo-counts
                    n_eps = self.pseudo_counts(
                        embeddings=embeddings[i].unsqueeze(0),
                        memory=self.episodic_memory[i],
                    )
                else:
                    n_eps = 0.0
                # store the pseudo-counts
                self.n_eps[i].append(n_eps)
                # update the episodic memory
                self.episodic_memory[i].append(embeddings[i])
                # clear the episodic memory if the episode is terminated or truncated
                if terminateds[i].item() or truncateds[i].item():
                    self.episodic_memory[i].clear()

    def compute(self, samples: Dict[str, th.Tensor], sync: bool = True) -> th.Tensor:
        """Compute the rewards for current samples.

        Args:
            samples (Dict[str, th.Tensor]): The collected samples. A python dict consists of multiple tensors,
                whose keys are ['observations', 'actions', 'rewards', 'terminateds', 'truncateds', 'next_observations'].
                For example, the data shape of 'observations' is (n_steps, n_envs, *obs_shape).
            sync (bool): Whether to update the reward module after the `compute` function, default is `True`.

        Returns:
            The intrinsic rewards.
        """
        super().compute(samples, sync)
        # get the number of steps and environments
        (n_steps, n_envs) = samples.get("next_observations").size()[:2]
        # get the observations and next observations
        obs_tensor = samples.get("observations").to(self.device)
        next_obs_tensor = samples.get("next_observations").to(self.device)
        # normalize the observations
        obs_tensor = self.normalize(obs_tensor)
        next_obs_tensor = self.normalize(next_obs_tensor)
        # compute the intrinsic rewards
        intrinsic_rewards = th.zeros(size=(n_steps, n_envs)).to(self.device)
        with th.no_grad():
            for i in range(self.n_envs):
                encoded_obs = self.encoder(obs_tensor[:, i])
                encoded_next_obs = self.encoder(next_obs_tensor[:, i])
                dist = F.mse_loss(encoded_obs, encoded_next_obs, reduction="none").mean(
                    dim=1
                )

                intrinsic_rewards[:, i] = dist.cpu()

        if sync:
            # get all the n_eps
            all_n_eps = [th.as_tensor(n_eps) for n_eps in self.n_eps]
            all_n_eps = th.stack(all_n_eps).T.to(self.device)
            # update the running mean and std of the squared distances
            flattened_squared_distances = th.cat(self.squared_distances, dim=0)
            self.dist_rms.update(flattened_squared_distances)
            self.squared_distances.clear()
            # flush the episodic memory of intrinsic rewards
            self.n_eps = [[] for _ in range(self.n_envs)]

            # update the reward module
            self.update(samples)

            # scale the intrinsic rewards
            return self.scale(intrinsic_rewards * all_n_eps)
        else:
            # TODO: first consider single environment for off-policy algorithms
            # compute the intrinsic rewards
            all_n_eps = [th.as_tensor(n_eps) for n_eps in self.n_eps]
            all_n_eps = th.stack(all_n_eps).T.to(self.device)
            intrinsic_rewards = intrinsic_rewards * all_n_eps
            # temporarily store the intrinsic rewards
            self.irs_buffer.append(intrinsic_rewards)
            if samples['truncateds'].item() or samples['terminateds'].item():
                # update the running mean and std of the squared distances
                flattened_squared_distances = th.cat(self.squared_distances, dim=0)
                self.dist_rms.update(flattened_squared_distances)
                self.squared_distances.clear()
                # update the running mean and std of the intrinsic rewards
                if self.rwd_norm_type == "rms":
                    self.rwd_norm.update(th.cat(self.irs_buffer))
                    self.irs_buffer.clear()
                if self.obs_norm_type == "rms":
                    self.obs_norm.update(th.cat(self.obs_buffer).cpu())
                    self.obs_buffer.clear()
            # flush the episodic memory of intrinsic rewards
            self.n_eps = [[] for _ in range(self.n_envs)]

            return (intrinsic_rewards / self.rwd_norm.std) * self.weight

    def update(self, samples: Dict[str, th.Tensor]) -> None:
        """Update the reward module if necessary.

        Args:
            samples (Dict[str, th.Tensor]): The collected samples same as the `compute` function.

        Returns:
            None.
        """
        # get the number of steps and environments
        (n_steps, n_envs) = samples.get("next_observations").size()[:2]
        # get the observations, actions and next observations
        obs_tensor = (
            samples.get("observations").to(self.device).view(-1, *self.obs_shape)
        )
        next_obs_tensor = (
            samples.get("next_observations").to(self.device).view(-1, *self.obs_shape)
        )
        # normalize the observations and next observations
        obs_tensor = self.normalize(obs_tensor)
        next_obs_tensor = self.normalize(next_obs_tensor)
        # apply one-hot encoding if the action type is discrete
        if self.action_type == "Discrete":
            actions_tensor = samples.get("actions").view(n_steps * n_envs)
            actions_tensor = F.one_hot(
                actions_tensor.long(), self.policy_action_dim
            ).float()
        else:
            actions_tensor = samples.get("actions").view(n_steps * n_envs, -1)
        # create the dataset and loader
        dataset = TensorDataset(obs_tensor, actions_tensor, next_obs_tensor)
        loader = DataLoader(dataset=dataset, batch_size=self.batch_size, shuffle=True)

        avg_im_loss = []
        avg_fm_loss = []
        # update the encoder, inverse dynamics model and forward dynamics model
        for _idx, batch_data in enumerate(loader):
            # get the batch data
            obs, actions, next_obs = batch_data
            obs, actions, next_obs = (
                obs.to(self.device),
                actions.to(self.device),
                next_obs.to(self.device),
            )
            # zero the gradients
            self.encoder_opt.zero_grad()
            self.im_opt.zero_grad()
            self.fm_opt.zero_grad()
            # encode the observations and next observations
            encoded_obs = self.encoder(obs)
            encoded_next_obs = self.encoder(next_obs)
            # get the predicted actions and next observations
            pred_actions = self.im(encoded_obs, encoded_next_obs)
            im_loss = self.im_loss(pred_actions, actions)
            pred_next_obs = self.fm(encoded_obs, actions)
            fm_loss = F.mse_loss(
                pred_next_obs, encoded_next_obs, reduction="none"
            ).mean(dim=-1)
            # use a random mask to select a subset of the training data
            mask = th.rand(len(im_loss), device=self.device)
            mask = (mask < self.update_proportion).type(th.FloatTensor).to(self.device)
            # get the masked losses
            im_loss = (im_loss * mask).sum() / th.max(
                mask.sum(), th.tensor([1], device=self.device, dtype=th.float32)
            )
            fm_loss = (fm_loss * mask).sum() / th.max(
                mask.sum(), th.tensor([1], device=self.device, dtype=th.float32)
            )
            # backward and update
            (im_loss + fm_loss).backward()
            self.encoder_opt.step()
            self.im_opt.step()
            self.fm_opt.step()
            avg_im_loss.append(im_loss.item())
            avg_fm_loss.append(fm_loss.item())

        self.metrics["loss"].append(
            [self.global_step, np.mean(avg_im_loss) + np.mean(avg_fm_loss)]
        )