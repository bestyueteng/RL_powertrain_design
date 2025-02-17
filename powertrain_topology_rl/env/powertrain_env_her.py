import numpy as np
import gymnasium as gym  # Changed to gymnasium
from gymnasium import spaces, Env
import random
from gymnasium.envs.registration import EnvSpec


# Import your utility functions.
from powertrain_topology_rl.utils import *
from powertrain_topology_rl.cp_init import *

class PowertrainEnv_her(gym.Env):
    
    metadata = {'render.modes': ['human']}
    spec = EnvSpec("PowertrainEnv_her-v0", "no-entry-point")
    
    def __init__(self,
                 component_library,
                 component_class,
                 component_type,
                 opt_method="test",
                 render_mode="human",
                 random_dsm=1,
                 init_dsm=None,
                 desired_dsm=None):
        """
        Parameters:
          - component_library, component_class, component_type, opt_method: as before.
          - random_dsm: controls the degree of randomness in the initial DSM.
          - init_dsm: a specific DSM to start with (optional).
          - desired_dsm: the target DSM matrix (2D numpy array) that defines success.
                         If None, the initial DSM will be used as the desired goal,
                         which might not be useful for training.
        """
        super(PowertrainEnv_her, self).__init__()
        self.render_mode = render_mode
        self.random_dsm = random_dsm

        # Process components and power types.
        self.components = structure_components(component_library, component_class, component_type)
        self.num_components = len(self.components)
        self.component_library = dict_to_matrix(component_library)
        self.component_class = component_class
        self.component_type = component_type
        self.power_types = get_power_types(component_library, component_class, component_type)
        self.opt_method = opt_method
        self.init_dsm = init_dsm
        
        # Since DSM is symmetric with zero diagonal, consider only upper triangle without diagonal.
        self.edge_indices = [(i, j) for i in range(1, self.num_components) for j in range(i+1, self.num_components)]
        self.num_possible_edges = len(self.edge_indices)
        
        # Mapping node to component classes, types, and power types.
        self.node_classes = {idx: class_name for idx, (class_name, _) in enumerate(self.components)}
        self.node_types = {idx: type_name for idx, (_, type_name) in enumerate(self.components)}
        self.power_in_types = {idx: power_in for idx, (power_in, _) in enumerate(self.power_types)}
        self.power_out_types = {idx: power_out for idx, (_, power_out) in enumerate(self.power_types)}

        # Action space: each action corresponds to adding an edge (by its index).
        self.action_space = spaces.Discrete(self.num_possible_edges)
        
        # Initialize DSM.
        self.initial_DSM = initialize_DSM(self.node_types, self.num_components)
        self.DSM = self.initial_DSM.copy()

        # Observation space: we now use the DSM matrix itself.
        # We assume DSM entries are 0 or 1.
        dsm_shape = self.DSM.shape
        self.observation_space = spaces.Dict({
            "observation": spaces.Box(low=0, high=1, shape=dsm_shape, dtype=np.int64),
            "achieved_goal": spaces.Box(low=0, high=1, shape=dsm_shape, dtype=np.int64),
            "desired_goal": spaces.Box(low=0, high=1, shape=dsm_shape, dtype=np.int64)
        })

        # History tracking (to prevent duplicates, if needed).
        self.old_DSM = []              # Previously encountered DSMs.
        self.old_action = None         # The last action applied.
        
        self.constraints = generate_constraints(self.node_classes, self.node_types, self.num_components)
        self.check_list = generate_check_list(self.node_classes, self.node_types)  # For power flow constraints.

        self.random_dsm_list = partial_init(self.components, self.power_types, self.random_dsm)
        
        # The desired DSM is provided externally. If none is provided, default to the initial DSM.
        self.desired_dsm = desired_dsm if desired_dsm is not None else self.initial_DSM.copy()
        print("self.desired_dsm: ", self.desired_dsm)
        
        self.reset()
         
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        # Initialize DSM.
        if self.random_dsm != 1:
            if self.init_dsm is None:
                idx = random.randint(0, len(self.random_dsm_list) - 1)
                self.DSM = np.array(self.random_dsm_list[idx])
            else:
                self.DSM = self.init_dsm.copy()
            # Randomly remove some connections.
            removed_any = False
            removable_connections = []
            for i in range(1, self.num_components):
                for j in range(i + 1, self.num_components):
                    if self.DSM[i][j] == 1:
                        removable_connections.append((i, j))
                        if random.random() <= self.random_dsm:
                            self.DSM[i][j] = 0
                            self.DSM[j][i] = 0
                            removed_any = True
            if not removed_any and removable_connections:
                i, j = random.choice(removable_connections)
                self.DSM[i][j] = 0
                self.DSM[j][i] = 0
            self.DSM = np.array(self.DSM)
        else:
            self.DSM = initialize_DSM(self.node_types, self.num_components)

        # Clear any previous history.
        self.old_DSM = []
        self.old_action = None

        obs = self._get_obs()
        # For HER, the achieved goal is the current DSM,
        # and the desired goal is the DSM matrix provided.
        return {
            "observation": obs,
            "achieved_goal": self.DSM.copy(),
            "desired_goal": self.desired_dsm.copy()
        }, {}
    
    def _get_obs(self):
        # Return the DSM matrix.
        return self.DSM.copy()
    
    def compute_reward(self, achieved_goal, desired_goal, info):
    
        # Check if we have batched input by testing if info is a dict.
        if isinstance(info, dict):
            # Single sample case.
            if info.get("invalid_action", True) or info.get("invalid_constraint", True):
                return -1
            if info.get("feasible", True) and info.get("evaluation_score") is not None:
                return info["evaluation_score"]
            # Fallback: negative mean squared error.
            mse = np.mean((achieved_goal - desired_goal) ** 2)
            return -mse
        else:
            # Batched case: assume info is an iterable of dictionaries.
            rewards = []
            for ag, dg, info_i in zip(achieved_goal, desired_goal, info):
                if info_i.get("invalid_action", True) or info_i.get("invalid_constraint", True):
                    rewards.append(-1)
                elif info_i.get("feasible", True) and info_i.get("evaluation_score") is not None:
                    rewards.append(info_i["evaluation_score"])
                else:
                    mse = np.mean((ag - dg) ** 2)
                    rewards.append(-mse)
            return np.array(rewards)


    def step(self, action):
        info = {
            'invalid_action':False,
            'invalid_constraint':False,
            'feasible':False,
            'evaluation_score':None,
        }
        
        truncated = False
        done = False
        achieved_goal = self.initial_DSM.copy()
        
        # Decode the action into an edge (from_node, to_node).
        from_node, to_node = self.edge_indices[action]
        
        # Prevent immediate repetition of the same action.
        if self.old_action == [from_node, to_node] or self.old_action == [to_node, from_node]:
            truncated = True
            info["invalid_action"] = True
            
        else:
            self.old_action = [from_node, to_node]
        
        # Check if the edge already exists.
        if self.DSM[from_node][to_node] == 1:
            info["invalid_action"] = True

        # Check power flow constraints.
        if not (self.power_in_types[from_node] == self.power_out_types[to_node] or
                self.power_out_types[from_node] == self.power_in_types[to_node]):
            info["invalid_action"] = True
        
        if info["invalid_action"]:
            
            truncated = True

        else:
            # Add the edge.
            self.DSM[from_node][to_node] = 1
            self.DSM[to_node][from_node] = 1
            
            achieved_goal = self.DSM.copy()
            
            # Check structural constraints.
            if check_constraints(self.DSM, self.constraints):
                truncated = True
                info["invalid_constraint"] = True
            
            else:
                # Check power flow feasibility.
                is_feasible = check_power_flow_completeness(self.DSM, self.check_list)
                if is_feasible:
                    info["feasible"] = True
                    # If feasible, evaluate the DSM
                    
                    # Check if it is a duprecated DSM
                    for idx, old_DSM in enumerate(self.old_DSM): 
                        is_duprecated = check_same_topology(self.DSM, old_DSM, self.node_classes, self.node_types, self.component_type)
                        
                        if is_duprecated:
                            print("Repeated DSM found.")
                            done = True
                            old_idx = idx
                            evaluation_score = self.old_evaluation_score[old_idx]
                            print("Evaluation score:", evaluation_score)
                            info["evaluation_score"] = eval_score
                            # reward = self.normalize_reward(reward)
                            break
                            

                        if not is_duprecated:
                            print("New DSM found.")
                            self.old_DSM.append(self.DSM.copy())
                            evaluation_score = self.evaluate_DSM()
                            info["evaluation_score"] = eval_score
                            self.old_evaluation_score.append(evaluation_score)
                            
                            # reward = self.normalize_reward(reward)
                            done = True  # Episode ends when a feasible topology is found
                            print("Evaluation score:", evaluation_score)
                else:
                    pass
                    
                    
        obs = self._get_obs()
        reward = self.compute_reward(achieved_goal, self.desired_dsm, info)

        return {
            "observation": obs.astype(np.int64),
            "achieved_goal": achieved_goal.astype(np.int64),
            "desired_goal": self.desired_dsm.copy().astype(np.int64)
        }, reward, done, truncated, info  
        
    def render(self):
        if self.render_mode == "human":
            print("Current DSM:")
            print(self.DSM)
        
    def evaluate_DSM(self):

        cost = get_performance(self.component_library,
                               self.DSM,
                               self.component_class,
                               self.component_type,
                               self.opt_method)
        return 1e6 / cost - 1