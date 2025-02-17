import gymnasium as gym
import torch
from torch.utils.tensorboard import SummaryWriter
import torch.nn as nn
import tianshou as ts
from tianshou.utils.space_info import SpaceInfo
from torch.utils.tensorboard import SummaryWriter
from tianshou.utils import TensorboardLogger
from tianshou.utils.net.common import Net
from powertrain_topology_rl.utils import *

device = "cuda" if torch.cuda.is_available() else "cpu"

# environments
component_library = {
        'Component number': [1, 2, 2, 2, 3, 4, 5, 5, 5],
        'Type of Components': [1, 1, 2, 3, 1, 1, 1, 2, 3],
        'Number of Instances': [1, 2, 2, 2, 2, 1, 1, 1, 1]
    }
    
component_class = ["VehicleBody", "GearSystems", "EnergyConverters", "EnergyStorage", "Controller"]
component_type = [
    "Vehicle body", ["Final Drive", "Simple transmission", "Multispeed gearbox"], 
    ["Electric motor 1"], 
    ["Battery"], 
    ["Torque Split", "Torque Coupler", "Electric Power Link"]
]

def main() -> None:
    lr, epoch, batch_size = 2e-4, 1e6, 64
    train_num, test_num = 10, 10
    gamma, n_step, target_freq = 0.9, 3, 320
    buffer_size = 20000
    eps_train, eps_test = 0.1, 0.05
    step_per_epoch, step_per_collect = 20, 10

    logger = ts.utils.TensorboardLogger(SummaryWriter("log/rainbow_dqn"))  # TensorBoard is supported!
    # For other loggers, see https://tianshou.readthedocs.io/en/master/tutorials/logger.html

    # You can also try SubprocVectorEnv, which will use parallelization
    train_envs = ts.env.DummyVectorEnv([lambda: gym.make(
            "powertrain_topology_rl/PowertrainEnv_tianshou-v0",
            component_library=component_library,
            component_class=component_class,
            component_type=component_type) for _ in range(train_num)])
    test_envs = ts.env.DummyVectorEnv([lambda: gym.make(
            "powertrain_topology_rl/PowertrainEnv_tianshou-v0",
            component_library=component_library,
            component_class=component_class,
            component_type=component_type) for _ in range(test_num)])
    env = gym.make(
            "powertrain_topology_rl/PowertrainEnv_tianshou-v0",
            component_library=component_library,
            component_class=component_class,
            component_type=component_type)
    
    from tianshou.utils.net.common import Net
    assert isinstance(env.action_space, gym.spaces.Discrete)
    
    print("env.observation_space.spaces: ", env.observation_space.shape)
    state_shape = env.observation_space.shape
    action_shape = env.action_space.n
    net = Net(state_shape=state_shape, action_shape=action_shape, hidden_sizes=[128, 128, 128],num_atoms=51)
    optim = torch.optim.Adam(net.parameters(), lr=lr)

    policy: ts.policy.RainbowPolicy = ts.policy.RainbowPolicy(
        model=net,
        optim=optim,
        discount_factor=gamma,
        v_min = -5,
        v_max=70,
        action_space=env.action_space,
        estimation_step=n_step,
        target_update_freq=target_freq
    )
    train_collector = ts.data.Collector(policy, train_envs, ts.data.VectorReplayBuffer(20000, 10), exploration_noise=True)
    test_collector = ts.data.Collector(policy, test_envs, exploration_noise=True)

    def stop_fn(mean_rewards: float) -> bool:
        if env.spec:
            if not env.spec.reward_threshold:
                return False
            else:
                return mean_rewards >= env.spec.reward_threshold
        return False

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
        logger=logger,
    ).run()
    print(f"Finished training in {result.timing.total_time} seconds")


    writer = SummaryWriter('/mech001.mnt/home/20223748/log/rainbow_dqn')
    logger = TensorboardLogger(writer)

    torch.save(policy.state_dict(), 'tianshou_dqn.pth')
    policy.load_state_dict(torch.load('tianshou_dqn.pth'))

    policy.eval()
    policy.set_eps(0.05)
    collector = ts.data.Collector(policy, env, exploration_noise=True)
    collector.collect(n_episode=1, render=False)


if __name__ == "__main__":
    main()