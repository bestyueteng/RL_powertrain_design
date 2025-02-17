from gymnasium.envs.registration import register

register(
    id="powertrain_topology_rl/PowertrainEnv-v0",
    entry_point="powertrain_topology_rl.env:PowertrainEnv",
)

register(
    id="powertrain_topology_rl/PowertrainEnv_powerflow-v0",
    entry_point="powertrain_topology_rl.env:PowertrainEnv_powerflow",
)

register(
    id="powertrain_topology_rl/PowertrainEnv_her-v0",
    entry_point="powertrain_topology_rl.env:PowertrainEnv_her",
)

register(
    id="powertrain_topology_rl/PowertrainEnv_overall-v0",
    entry_point="powertrain_topology_rl.env:PowertrainEnv_overall",
)

register(
    id="powertrain_topology_rl/PowertrainEnv_powerflow_withmask-v0",
    entry_point="powertrain_topology_rl.env:PowertrainEnv_powerflow_withmask",
)