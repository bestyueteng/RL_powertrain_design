import numpy as np
import networkx as nx
import gymnasium as gym  # Changed to gymnasium
from gymnasium import spaces
from powertrain_topology_rl.utils import *
from powertrain_topology_rl.cp_init import *
import random

class PowertrainEnv(gym.Env):
    """
    Custom Environment for Vehicle Powertrain Topology Optimization with Symmetric DSM.
    """
    metadata = {'render.modes': ['human']}

    def __init__(self, component_library, component_class, component_type, opt_method="test", render_mode="human",random_dsm=1, init_dsm=None):
        super(PowertrainEnv, self).__init__()
        self.render_mode = render_mode
        self.random_dsm = random_dsm
        self.components = structure_components(component_library, component_class, component_type)
        self.num_components = len(self.components)
        
        self.component_library = dict_to_matrix(component_library)
        self.component_class = component_class
        self.component_type = component_type
        self.power_types = get_power_types(component_library, component_class, component_type)
        self.opt_method = opt_method
        self.init_dsm = init_dsm
        
        # Since DSM is symmetric and diagonal is zero, consider only upper triangle without diagonal
        self.edge_indices = [(i, j) for i in range(1, self.num_components) for j in range(i+1, self.num_components)] # Didn't change VB
        self.num_possible_edges = len(self.edge_indices)
        
        # Map nodes to their component classes
        self.node_classes = {idx: class_name for idx, (class_name, _) in enumerate(self.components)}

        # Map nodes to their component types
        self.node_types = {idx: type_name for idx, (_, type_name) in enumerate(self.components)}

        # Map nodes to their power types
        self.power_in_types = {idx: power_in for idx, (power_in, _) in enumerate(self.power_types)}
        self.power_out_types = {idx: power_out for idx, (_, power_out) in enumerate(self.power_types)}

        # Action space: For each possible edge, we can add or remove it
        self.num_actions = self.num_possible_edges  # Each edge has two actions: add or remove
        self.action_space = spaces.Discrete(self.num_actions) 
        
        # Initialize DSM
        self.initial_DSM = initialize_DSM(self.node_types,self.num_components)
        self.DSM = self.initial_DSM.copy()

        # Observation space: The flattened upper triangle of DSM without diagonal and the flattened component library
        lib_dim = len(self.component_library.flatten())

        # self.observation_space = spaces.Dict({
                                             # "DSM": spaces.Box(low=0, high=1, shape=(len(self.DSM.flatten()),),dtype=np.int32),
                                              # "Library": spaces.Box(low=0, high=5, shape=(lib_dim,), dtype=np.int32)
                                            # })
        self.observation_space = spaces.Box(low=0, high=1, shape=(len(self.DSM.flatten()),),dtype=np.int32)
        self.done_DSM = None
        self.old_DSM = []  # Placeholder for the DSM before applying the action
        self.old_evaluation_score = []
        self.old_action = None  # Placeholder for the last action applied

        self.constraints = generate_constraints(self.node_classes, self.node_types, self.num_components)
        self.check_list = generate_check_list(self.node_classes, self.node_types) # for power flow constraints

        # For recording
        self.num_of_motor = []
        self.num_of_power = []
        
        self.random_dsm_list = partial_init(self.components, self.power_types, self.random_dsm)
        self.reset()
         
    def reset(self,  *, seed=None, options=None):
        super().reset(seed=seed)
        if self.random_dsm != 1: # split training
            if self.init_dsm is None:
                idx = random.randint(0, len(self.random_dsm_list) - 1)
                self.DSM = np.array(self.random_dsm_list[idx])
            else:
                self.DSM = self.init_dsm.copy() # specify dsm for eval\
                # print("self.DSM Original: ", self.DSM)
                # print("self.init_dsm: ", self.init_dsm)
            # randomly remove some connections from the DSM matrix
            
            removed_any = False
            removable_connections = []  # Store removable (i, j) pairs

            for i in range(1, self.num_components):
                for j in range(i + 1, self.num_components):
                    if self.DSM[i][j] == 1:
                        removable_connections.append((i, j))
                        if random.random() <= self.random_dsm:
                            self.DSM[i][j] = 0
                            self.DSM[j][i] = 0
                            removed_any = True

            # Ensure at least one connection is removed
            if not removed_any and removable_connections:
                i, j = random.choice(removable_connections)
                self.DSM[i][j] = 0
                self.DSM[j][i] = 0
                        
            self.DSM = np.array(self.DSM)  
            # print("self.DSM: ", self.DSM)
        else:
            self.DSM = initialize_DSM(self.node_types,self.num_components)

        return self._get_obs(), {}
    
    def _get_obs(self):
        # DSM flattened
        DSM_flat = self.DSM.flatten()
        DSM_flat = np.array(DSM_flat, dtype=np.int32)
        # library flattened
        library_flat = self.component_library.flatten()
        library_flat = np.array(library_flat, dtype=np.int32)
        # State vector
        # obs = {"DSM": DSM_flat, "Library": library_flat}
        obs = DSM_flat
        return obs
    
    def step(self, action):
        truncated = False
        is_duprecated = False
        # Decode action into edge_index and add_remove
        # edge_idx, add_remove = divmod(action, 2)
        from_node, to_node = self.edge_indices[action]
        done = False
        reward = 0
        invalid_action = False
        
        if self.old_action == [from_node, to_node] or self.old_action == [to_node, from_node]:
            # Repeating the same action, assign a negative reward
            obs = self._get_obs()
            return obs, -1, False, True, {}
        else:
            self.old_action = [from_node, to_node] # Save the last action
            
        
        # Check if the edge already exists
        if self.DSM[from_node][to_node] == 1:
            invalid_action = True

        # Check if the edge is valid based on power flow constraints
        if self.power_in_types[from_node] == self.power_out_types[to_node] \
            or self.power_out_types[from_node] == self.power_in_types[to_node]:
            pass
        else:
            invalid_action = True
        
        if invalid_action:
            # Invalid action, assign a large negative reward
            reward = -1
            # Optionally, set done = True to end the episode
            truncated = True
            # Do not apply the action
        else:
            # Add an edge
            self.DSM[from_node][to_node] = 1
            self.DSM[to_node][from_node] = 1

            # Check constraints
            constraints_violated = check_constraints(self.DSM, self.constraints)

            if constraints_violated:
                reward = -1  
                truncated = True
            else:

                # Check power flow constraints
                is_feasible = check_power_flow_completeness(self.DSM, self.check_list)
                
                if is_feasible:
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
                            reward = evaluation_score
                            # reward = self.normalize_reward(reward)
                            break
                            

                    if not is_duprecated:
                        print("New DSM found.")
                        self.old_DSM.append(self.DSM.copy())
                        evaluation_score = self.evaluate_DSM()
                        reward = evaluation_score
                        self.old_evaluation_score.append(evaluation_score)
                        
                        # reward = self.normalize_reward(reward)
                        done = True  # Episode ends when a feasible topology is found
                        print("Evaluation score:", evaluation_score)
                        
                    
                else:
                    pass  
        
        obs = self._get_obs()
        return obs, reward, done, truncated, {}  
        
    def render(self):
        if self.render_mode == "human":
            # Print the DSM as a matrix
            print("Current DSM:")
            print(self.DSM)
        
    def evaluate_DSM(self):
        """
        Placeholder for the DSM evaluation function.
        Returns a score based on the DSM topology.
        """
        cost = get_performance(self.component_library, self.DSM, self.component_class, self.component_type, self.opt_method)
        return 1e6 / cost - 1
        
    def normalize_reward(self, reward):
        reward = reward - 30 / (100-30) # max evaluation is 100, min is 30
        return reward