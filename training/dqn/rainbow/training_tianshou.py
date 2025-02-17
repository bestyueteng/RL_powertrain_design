import gymnasium as gym
import torch
from torch.utils.tensorboard import SummaryWriter
import torch.nn as nn
import tianshou as ts
from tianshou.utils import TensorboardLogger
from tianshou.utils.net.common import Net
from powertrain_topology_rl.utils import *
from powertrain_topology_rl.env import PowertrainEnv
from collections.abc import Callable, Sequence
from typing import Any
from tianshou.utils.net.discrete import Actor, NoisyLinear
from tianshou.utils.net.common import NetBase

device = "cuda" if torch.cuda.is_available() else "cpu"

class DQN(NetBase[Any]):
    """Reference: Human-level control through deep reinforcement learning.

    For advanced usage (how to customize the network), please refer to
    :ref:`build_the_network`.

    This version has been modified to accept a 2D matrix observation (x*x)
    instead of an image with channels. All convolution-related code has been removed.
    """

    def __init__(
        self,
        obs_size: int,  # e.g., x*x if your input is x-by-x
        action_shape: Sequence[int] | int,
        device: str | int | torch.device = "cpu",
        features_only: bool = False,
        output_dim_added_layer: int | None = None,
        layer_init: Callable[[nn.Module], nn.Module] = lambda x: x,
    ) -> None:
        """
        :param obs_size: dimension of the flattened observation, i.e. x*x if your input is x-by-x.
        :param action_shape: shape (or integer) representing the number of actions.
        :param device: device to place the model on (e.g. "cpu", "cuda").
        :param features_only: if True, the network will output only the features before the final layer(s).
        :param output_dim_added_layer: if not None, adds a linear layer on top of the extracted features
                                       to match this dimension. Only used when `features_only` is True.
        :param layer_init: a function that initializes the layers (default is identity).
        """
        if not features_only and output_dim_added_layer is not None:
            raise ValueError(
                "Should not provide explicit output dimension using `output_dim_added_layer` "
                "when `features_only` is True."
            )
        super().__init__()
        self.device = device

        # Base MLP: flatten -> two linear layers (customize as you wish)
        self.net = nn.Sequential(
            layer_init(nn.Linear(obs_size, 128)),
            nn.ReLU(inplace=True),
            layer_init(nn.Linear(128, 128)),
            nn.ReLU(inplace=True),
        )

        with torch.no_grad():
            # Probe the output dimension of the base MLP
            base_output_dim = self.net(torch.zeros(1, obs_size)).shape[1]

        if not features_only:
            # Add final layer for Q-value outputs
            action_dim = int(np.prod(action_shape))
            self.net = nn.Sequential(
                self.net,
                layer_init(nn.Linear(base_output_dim, 512)),
                nn.ReLU(inplace=True),
                layer_init(nn.Linear(512, action_dim)),
            )
            self.output_dim = action_dim
        elif output_dim_added_layer is not None:
            # If features_only is True and we specify an extra layer
            self.net = nn.Sequential(
                self.net,
                layer_init(nn.Linear(base_output_dim, output_dim_added_layer)),
                nn.ReLU(inplace=True),
            )
            self.output_dim = output_dim_added_layer
        else:
            # If features_only is True and no extra layer is specified
            self.output_dim = base_output_dim

    def forward(
        self,
        obs: np.ndarray | torch.Tensor,
        state: Any | None = None,
        info: dict[str, Any] | None = None,
        **kwargs: Any,
    ) -> tuple[torch.Tensor, Any]:
        r"""Mapping: s -> Q(s, \*)."""
        # If obs is shape [B, x, x], flatten to [B, x*x]
        obs = torch.as_tensor(obs, device=self.device, dtype=torch.float32)
        obs = obs.view(obs.shape[0], -1)  # Flatten from (B, x, x) -> (B, x*x)
        return self.net(obs), state
    
class Rainbow(DQN):
    """Reference: Rainbow: Combining Improvements in Deep Reinforcement Learning.

    For advanced usage (how to customize the network), please refer to
    :ref:`build_the_network`.

    This version has been modified to accept a 2D matrix observation (x*x).
    """

    def __init__(
        self,
        obs_size: int,            # e.g., x*x if your input is x-by-x
        action_shape: Sequence[int],
        num_atoms: int = 51,
        noisy_std: float = 0.5,
        device: str | int | torch.device = "cpu",
        is_dueling: bool = True,
        is_noisy: bool = True,
    ) -> None:
        # Initialize a base DQN with features_only=True so we can build Rainbow layers on top
        super().__init__(
            obs_size=obs_size,
            action_shape=action_shape,
            device=device,
            features_only=True,  # We'll add distributional layers here
        )
        self.action_num = int(np.prod(action_shape))
        self.num_atoms = num_atoms
        self._is_dueling = is_dueling

        def linear(x: int, y: int) -> nn.Module:
            if is_noisy:
                return NoisyLinear(x, y, noisy_std)
            return nn.Linear(x, y)

        # Q network: outputs (action_num * num_atoms)
        self.Q = nn.Sequential(
            linear(self.output_dim, 512),
            nn.ReLU(inplace=True),
            linear(512, self.action_num * self.num_atoms),
        )

        # V network for dueling
        if self._is_dueling:
            self.V = nn.Sequential(
                linear(self.output_dim, 512),
                nn.ReLU(inplace=True),
                linear(512, self.num_atoms),
            )

        self.output_dim = self.action_num * self.num_atoms

    def forward(
        self,
        obs: np.ndarray | torch.Tensor,
        state: Any | None = None,
        info: dict[str, Any] | None = None,
        **kwargs: Any,
    ) -> tuple[torch.Tensor, Any]:
        r"""Mapping: s -> Z(s, \*), i.e., distribution over returns (num_atoms)."""
        # Use parent forward to get the base features
        obs_features, state = super().forward(obs, state, info, **kwargs)

        q = self.Q(obs_features)  # [B, action_num * num_atoms]
        q = q.view(-1, self.action_num, self.num_atoms)

        if self._is_dueling:
            v = self.V(obs_features)  # [B, num_atoms]
            v = v.view(-1, 1, self.num_atoms)
            # Combine advantage and value streams
            logits = q - q.mean(dim=1, keepdim=True) + v
        else:
            logits = q

        # Convert logits to probabilities for distributional RL
        probs = logits.softmax(dim=2)
        return probs, state
    
def training_rainbow(args, component_library, component_class, component_type, wandb_callback, run, logger):
    # Hyperparameters
    lr, epoch, batch_size = 2e-4, 1e6, 128
    train_num, test_num = 10, 10
    gamma, n_step, target_freq = 0.9, 10, 320
    buffer_size = 20000
    eps_train, eps_test = 0.1, 0.05
    step_per_epoch, step_per_collect = 20, 10

    # Initialize SummaryWriter and TensorboardLogger
    writer = SummaryWriter("logs/rainbow_dqn")
    writer.add_text("args", str(args))
    logger = TensorboardLogger(writer)

    # Create the environment
    env = PowertrainEnv(component_library, component_class, component_type)
    env = gym.wrappers.FlattenObservation(env)
    # Create vectorized environments for training and testing
    train_envs = ts.env.DummyVectorEnv([lambda: env for _ in range(train_num)])
    test_envs = ts.env.DummyVectorEnv([lambda: env for _ in range(test_num)])
    
    # Ensure the action space is discrete
    assert isinstance(env.action_space, gym.spaces.Discrete)
    
    print("env.observation_space.shape:", env.observation_space.shape)
    state_shape = env.observation_space.shape
    print("state_shape: ", state_shape[0])
    action_shape = env.action_space.n
    
    # Build the network with discrete distributional outputs (Rainbow DQN)
    net = Rainbow(
        obs_size=state_shape[0],
        action_shape=action_shape,
    )
    optim = torch.optim.Adam(net.parameters(), lr=lr)

    # Initialize the Rainbow policy
    policy: ts.policy.RainbowPolicy = ts.policy.RainbowPolicy(
        model=net,
        optim=optim,
        discount_factor=gamma,
        v_min=-5,
        v_max=70,
        action_space=env.action_space,
        estimation_step=n_step,
        target_update_freq=target_freq
    )
    
    # Create collectors for training and testing
    train_collector = ts.data.Collector(
        policy,
        train_envs,
        ts.data.VectorReplayBuffer(buffer_size, train_num),
        exploration_noise=True
    )
    test_collector = ts.data.Collector(policy, test_envs, exploration_noise=True)

    # Define a stopping function (optional, based on the reward threshold)
    def stop_fn(mean_rewards: float) -> bool:
        if env.spec:
            if not env.spec.reward_threshold:
                return False
            else:
                return mean_rewards >= env.spec.reward_threshold
        return False

    # Train the policy using Tianshou's off-policy trainer
    result = ts.trainer.OffpolicyTrainer(
        policy=policy,
        train_collector=train_collector,
        test_collector=test_collector,
        max_epoch=epoch,
        step_per_epoch=step_per_epoch,
        step_per_collect=step_per_collect,
        episode_per_test=test_num,
        batch_size=batch_size,
        update_per_step=1 / step_per_collect,
        train_fn=lambda epoch, env_step: policy.set_eps(eps_train),
        test_fn=lambda epoch, env_step: policy.set_eps(eps_test),
        stop_fn=stop_fn,
        verbose=False,
        logger=logger,
    ).run()
    print(f"Finished training in {result.timing.total_time} seconds")

    # Save and load the trained policy
    torch.save(policy.state_dict(), 'tianshou_dqn.pth')
    policy.load_state_dict(torch.load('tianshou_dqn.pth'))

    # Evaluate the policy
    policy.eval()
    policy.set_eps(eps_test)
    collector = ts.data.Collector(policy, env)
    collector.collect(n_episode=1, render=False)


if __name__ == "__main__":
    # Example dummy arguments and component definitions.
    # Replace these with your actual configuration.
    class DummyArgs:
        env_id = "YourEnvID"  # Replace with your actual environment ID

    args = DummyArgs()
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
    wandb_callback = None
    run = None
    logger = None

    training_rainbow(args, component_library, component_class, component_type, wandb_callback, run, logger)
