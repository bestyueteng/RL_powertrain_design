import numpy as np
import networkx as nx
import gymnasium as gym  # Changed to gymnasium
from gymnasium import spaces
from powertrain_topology_rl.utils import *
from powertrain_topology_rl.cp_init import *
import random
import csv
import ast

class PowertrainEnv_overall(gym.Env):
    """
    A general env for powertrain topology optimization. The initial inputs are
    the components information and possible partial topology.
    """

    meradata = {'render.modes': ['human']}

    def __init__(self,component_library, 
                         component_class, 
                         component_type, 
                         opt_method="test", 
                         render_mode="human",
                         random_dsm=1, 
                         init_dsm=None,
                         required_instances=None,
                         performance_req=None):
        super(PowertrainEnv_overall, self).__init__()

        self.init_component_library_fix = component_library
        self.component_library_fix = dict_to_matrix(component_library)
        self.component_class_fix = component_class
        self.component_type_fix = component_type
        self.opt_method = opt_method
        self.render_mode = render_mode
        self.random_dsm = random_dsm
        self.init_partial_topo = init_dsm
        self.required_instances = required_instances
        self.performance_req = performance_req
        
        self.components = structure_components(component_library, component_class, component_type)
        self.max_components = len(self.components)
        self.num_of_instances = self.component_library_fix[2]

        self.init_DSM = np.zeros((self.max_components, self.max_components))

        # Map nodes to their component classes
        self.node_classes = {idx: class_name for idx, (class_name, _) in enumerate(self.components)}

        # Map nodes to their component types
        self.node_types = {idx: type_name for idx, (_, type_name) in enumerate(self.components)}

        # Map nodes to their power types
        self.power_types = get_power_types(component_library, component_class, component_type)
        self.power_in_types = {idx: power_in for idx, (power_in, _) in enumerate(self.power_types)}
        self.power_out_types = {idx: power_out for idx, (_, power_out) in enumerate(self.power_types)}

        # action space is the upper triangle of the DSM excepting the diagonal and the first row
        self.possible_actions = [(i, j) for i in range(1, self.max_components) for j in range(i+1, self.max_components)]
        self.action_space = spaces.Discrete(len(self.possible_actions))
        # print("shape of possible_actions: ", len(self.possible_actions))
        # observation space is the DSM and number of instances of each component
        self.observation_space = spaces.Dict({
            "DSM": spaces.MultiBinary(len(self.init_DSM.flatten())),
            "Library": spaces.Box(low=0, high=4, shape=(len(self.num_of_instances),), dtype=np.int8),
            "performance_req": spaces.Box(low=0, high=200, shape=(5,), dtype=np.float64)
        })

        self.done_DSM = None
        self.history_result = []  # Placeholder for the DSM before applying the action
        self.old_evaluation_score = []
        self.old_action = None  # Placeholder for the last action applied
        self.old_performance_req = None  # Placeholder for the last performance requirements
        
        self.constraints = generate_constraints(self.node_classes, self.node_types, self.max_components)
        self.check_list = generate_check_list(self.node_classes, self.node_types) # for power flow constraints
                
        self.reset()

    def reset(self, *, seed=None, options=None):
        self.DSM = self.init_DSM.copy()
        self.total_timestep = 0
        check = 0
        if self.performance_req is not None:
            self.performance_req = self.performance_req
        else:
            # randomize performance requirements
            self.performance_req = np.zeros(5)
            self._randomize_performance_req()
            # self.performance_req = np.array([8, 200, 0.1993, 0.1, 50])
        
        # Randomize num of instances
        if self.required_instances is None and self.init_partial_topo is None:
           
            self.new_components_library = randomize_instances(self.init_component_library_fix, start_index=1, max_val=3)
            self.valid_row = find_all_valid_rows(self.new_components_library, self.init_component_library_fix)
            self.invalid_row = [i for i in range(1, self.max_components) if i not in self.valid_row]
            self.new_components_library = dict_to_matrix(self.new_components_library)
            self.num_of_instances = self.new_components_library[2]
            
            self.random_dsm_list = partial_init(self.components, self.power_types, self.random_dsm, self.valid_row)
        
        else:
            check = 1
            self.new_components_library = self.init_component_library_fix.copy()
            
            if self.required_instances is not None:
                self.new_components_library['Number of Instances'] = self.required_instances
            
            self.valid_row = find_all_valid_rows(self.new_components_library, self.init_component_library_fix)
            self.invalid_row = [i for i in range(1, self.max_components) if i not in self.valid_row]
            self.new_components_library = dict_to_matrix(self.new_components_library)
            self.num_of_instances = self.new_components_library[2]
            
            self.random_dsm_list = partial_init(self.components, self.power_types, self.random_dsm, self.valid_row)
            
        # split training
        if self.random_dsm != 1: 
            if self.init_partial_topo is None:
                idx = random.randint(0, len(self.random_dsm_list) - 1)
                self.DSM = np.array(self.random_dsm_list[idx])
            else:
                self.DSM = np.zeros((self.max_components, self.max_components))
                for i in range (len(self.init_partial_topo)):
                    dsm_row = self.valid_row[i]
                    self.DSM[dsm_row][self.valid_row] = self.init_partial_topo[i]
                # print("self.DSM Original: ", self.DSM)
                # print("self.init_dsm: ", self.init_dsm)
            # randomly remove some connections from the DSM matrix
            
            removed_any = False
            removable_connections = []  # Store removable (i, j) pairs

            for i in range(1, self.max_components):
                for j in range(i + 1, self.max_components):
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
            self.DSM = initialize_DSM(self.node_types,self.max_components)
            if check == 1:
                print("check init done")
        return self._get_obs(), {}
    
    def _get_obs(self):
        # DSM flattened
        DSM_flat = self.DSM.flatten()
        DSM_flat = np.array(DSM_flat, dtype=np.int8)
        # library flattened
        library_flat = self.num_of_instances
        library_flat = np.array(library_flat, dtype=np.int8)
        # State vector
        obs = {"DSM": DSM_flat, "Library": library_flat, "performance_req": self.performance_req}
        return obs

    def action_masks(self):
        # Invalid actions are the actions not in valid rows
        self.invalid_actions = []
        for i in range(1, self.max_components):
            for j in range(i+1, self.max_components):
                if (i not in self.valid_row) or (j not in self.valid_row):
                    self.invalid_actions.append((i, j))
        
        # Invalid actions are actions meet power flow constraints

        for i in range(1, self.max_components):
            for j in range(i+1, self.max_components):
                if (self.power_in_types[i] == self.power_out_types[j]) or (self.power_out_types[i] == self.power_in_types[j]):
                    pass
                else:
                    self.invalid_actions.append((i, j))

        # Invalid actions are actions that already exist in the DSM
        for i in range(1, self.max_components):
            for j in range(i+1, self.max_components):
                if self.DSM[i, j] == 1:
                    self.invalid_actions.append((i, j))

        for action in range(len(self.possible_actions)):
            from_node, to_node = self.possible_actions[action]
            DSM_copy = self.DSM.copy()
            DSM_copy[from_node][to_node] = 1
            DSM_copy[to_node][from_node] = 1
            constraints_violated = check_constraints(DSM_copy, self.constraints)
            if constraints_violated:
                self.invalid_actions.append((from_node, to_node))
        # print("Invalid actions: ", self.invalid_actions)
        return [action not in self.invalid_actions for action in self.possible_actions]

    def step(self, action):
        truncated = False
        is_duprecated = False
        is_same_performance = False
        # Decode action into edge_index and add_remove
        # edge_idx, add_remove = divmod(action, 2)
        from_node, to_node = self.possible_actions[action]
        done = False
        reward = 0

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

                # Check if it is a duprecated DSM
                for idx, result in enumerate(self.history_result): 
                    old_DSM = result[0]
                    old_performance_req = result[1]
                    is_duprecated = check_same_topology(self.DSM, old_DSM, self.node_classes, self.node_types, self.component_type_fix)
                    is_same_performance = np.array_equal(self.performance_req, old_performance_req)
                    if is_duprecated and is_same_performance:
                        print("Repeated DSM found.")
                        done = True
                        old_idx = idx
                        evaluation_score = self.old_evaluation_score[old_idx]
                        print("Evaluation score:", evaluation_score)
                        reward = evaluation_score
                        # reward = self.normalize_reward(reward)
                        break
                
                        

                if not is_duprecated or not is_same_performance:
                    print("New DSM found.")
                    print("DSM:", self.DSM)
                    old_history = [self.DSM.copy(), self.performance_req.copy()]
                    self.history_result.append(old_history)
                    evaluation_score = self.evaluate_DSM()
                    reward = evaluation_score
                    self.old_evaluation_score.append(evaluation_score)
                    
                    # reward = self.normalize_reward(reward)
                    done = True  # Episode ends when a feasible topology is found
                    print("Evaluation score:", evaluation_score)
                
            else:
                # Intrinsic reward for adding an edge
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
        cost = get_performance(self.component_library_fix, self.DSM, self.component_class_fix, self.component_type_fix, self.performance_req, self.opt_method)
        return 1e6 / cost - 1
        
    def normalize_reward(self, reward):
        reward = reward - 30 / (100-30) # max evaluation is 100, min is 30
        return reward
    
    def _randomize_performance_req(self):
        """
        Randomize the performance requirements for the powertrain components.
        """
        # top speed (180, 200)
        self.performance_req[0] = random.randint(180, 200)
        # acceleration time (6,9)
        self.performance_req[1] = random.randint(6, 9)
        # gradability_ss (0.15, 0.1993)
        self.performance_req[2] = random.uniform(0.15, 0.1993)
        # gradability_fs (0.05, 0.15)
        self.performance_req[3] = random.uniform(0.05, 0.15)
        # gradability_fs_speed (40, 50)
        self.performance_req[4] = random.randint(40, 50)
