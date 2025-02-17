import numpy as np
import networkx as nx
import gymnasium as gym  # Changed to gymnasium
from gymnasium import spaces
from powertrain_topology_rl.utils import *
from powertrain_topology_rl.powerflow_utils import *
import random

class PowertrainEnv_powerflow_withmask(gym.Env):
    """
    Custom Environment for Vehicle Powertrain Topology Optimization with Symmetric DSM.
    """
    metadata = {'render.modes': ['human']}

    def __init__(self, component_library, component_class, component_type, opt_method="test", render_mode="human", random_dsm=1, init_dsm=None):
        super(PowertrainEnv_powerflow_withmask, self).__init__()
        self.component_library = dict_to_matrix(component_library)
        self.component_class = component_class
        self.component_type = component_type # flatten component type
        self.opt_method = opt_method
        self.render_mode = render_mode
        
        # ['VB', 'TS', 'FD', 'TC1', 'SGB', 'MGB','TC2', 'EM', 'EPL', 'B'] form 1 to 10
        self.num_type = 10
        self.num_path = 2
        
        # self.action_list = [(i, j) for i in range(1, self.num_type+1) for j in range(self.num_path)]
        # self.action_space = spaces.Discrete(len(self.action_list))  # [component, path], VB, BAT is mandatory
        self.action_space = spaces.Discrete(self.num_path*self.num_type)
        self.observation_space = spaces.Box(low=0, high=1, shape=(self.num_path*self.num_type,), dtype=np.int8)

        self.old_flows = []
        self.old_evaluation_score = []
        
        self.overall_step = self.num_type * self.num_path
        print("self.overall_step: ", self.overall_step)
        self.step_size = 0
        self.reset()
         
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.step_size = 0
        self.old_action = []
        self.used_powerflows = np.zeros((self.num_path, self.num_type), dtype=np.int8)
        # Create a (4, 9) array of zeros
        self.powerflows = np.zeros((self.num_path, self.num_type), dtype=np.int8)
    
        # For each path, place mandatory components. Example:
        # Suppose you want each path to start with [1 (VB), 8 (B)] in columns 0 and 1.
        # for path_id in range(4):
            # self.powerflows[path_id, 0] = 1  # VB
            # self.powerflows[path_id, 1] = 2   # TS
            # self.powerflows[path_id, -1] = 9  # B
        
        return self._get_obs(), {}
    
    def _get_obs(self):
    
        # for i in range(len(self.powerflows)):
            # path = self.powerflows[i]
            # self.powerflows[i] = np.sort(path, kind='mergesort')
            # self.powerflows[i] = np.concatenate((path[path != 0], path[path == 0]))
        
        # if self.powerflows.shape != (self.num_path, self.num_type):
            # obs = np.pad(self.powerflows, ((0, 0), (0, self.num_type - self.powerflows.shape[1])), mode='constant', constant_values=0)
        # else:
        obs = self.powerflows
        
        obs = np.sort(obs, axis=0)
        obs = np.array(obs.flatten(),dtype=np.int8)
        
        return obs
    def action_masks(self):
        # action_masks = np.ones(self.action_space.n)
        masks = np.ones(self.num_type*self.num_path)
        for i in range(self.num_type):
            for j in range(self.num_path):
                if self.powerflows[j,i] != 0:
                    masks[j*self.num_type+i] = 0

        for idx, path in enumerate(self.used_powerflows):
            # MGB and SBG cannot appear in one powerflow
            if np.any(path == 5):
                masks[idx*self.num_type+6] = 0
            if np.any(path == 6):
                masks[idx*self.num_type+5] = 0
        
        for idx, path in enumerate(self.used_powerflows):
            if np.any(path == 4):
                masks[idx*self.num_type+7] = 0
            if np.any(path == 7):
                masks[idx*self.num_type+4] = 0
        
        for action in self.old_action:
            masks[action] = 0
            

        return masks
    
    def step(self, action):
        reward = 0
        truncated = False
        done = False
        
        self.step_size += 1
        # component_id, path_id = self.action_list[action]
        path_id = action // self.num_type
        # print("self.step_size: ", self.step_size)
        # print("path_id: ", path_id)
        component_id = (action+1) % self.num_type
        if component_id == 0:
            component_id = 10
            
        if action in self.old_action:
            return self._get_obs(), -10, True, truncated, {}
        else:
            self.old_action.append(action)
        
        if self.powerflows[path_id,component_id-1] == 0:
            self.powerflows[path_id,component_id-1] = 1
            self.used_powerflows[path_id,component_id-1] = component_id
            # print("self.powerflows: ", self.powerflows)
            # print("self.used_powerflows: ", self.used_powerflows)
            # print("action: ", action)
        else:
            # self.powerflows[path_id,component_id-1] = 0
            # self.used_powerflows[path_id,component_id-1] = 0
            return self._get_obs(), -10, True, truncated, {} # cannot remove 
        
        if self.step_size >= self.overall_step:
            # print("truncated")
            truncated = True
            reward += -10
            self.step_size = 0
        
        # self.powerflows = np.sort(self.powerflows)
        # self.used_powerflows = np.sort(self.used_powerflows)

        for path in self.used_powerflows:
            # MGB and SBG cannot appear in one powerflow
            if np.any(path == 5) and np.any(path == 6):
                obs, _ = self.reset()
                return obs, -10, True, False, {}
                
        sorted_used_powerflow = self.used_powerflows.copy()
        for i in range(len(sorted_used_powerflow)):
            path = sorted_used_powerflow[i]
            sorted_used_powerflow[i] = np.sort(path, kind='mergesort')
            sorted_used_powerflow[i] = np.concatenate((path[path != 0], path[path == 0]))
            
        is_feasible = check_feasibility(sorted_used_powerflow, self.component_library)
        # print("is_feasible: ", is_feasible)
        # print("powerflows: ", sorted_used_powerflow)
        if is_feasible or truncated:
            # If feasible, evaluate the DSM
            if any([np.array_equal(sorted_used_powerflow, old_flow) for old_flow in self.old_flows]):
                print("Repeated DSM found.")
                done = True
                old_idx = [np.array_equal(sorted_used_powerflow, old_flow) for old_flow in self.old_flows].index(True)
                evaluation_score = self.old_evaluation_score[old_idx]
                print("Evaluation score:", evaluation_score)
                reward += evaluation_score
                # reward = self.normalize_reward(reward)
            else:
                print("New DSM found.")
                self.old_flows.append(sorted_used_powerflow.copy())
                self.num_components, self.DSM = map_flow_to_topology(sorted_used_powerflow)
                self.DSM = np.array(self.DSM) 
                self.component_library[2] = np.array(self.num_components)
                print("powerflow: ", self.powerflows)
                print("sorted_used_powerflow: ", sorted_used_powerflow)
                print("self.num_components: ", self.num_components)
                print("DSM: ", self.DSM)
                evaluation_score = self.evaluate_DSM()


                print("Evaluation score:", evaluation_score)
                
                reward += evaluation_score
                self.old_evaluation_score.append(evaluation_score)
                
                # reward = self.normalize_reward(reward)
                done = True  # Episode ends when a feasible topology is found
                
            
        else:
            for idx, path in enumerate(self.powerflows): 
                if sum([1 for elem in path if elem != 0]) < 4:
                    if path_id == idx:
                        reward += 1

            pass
        
        
            
        obs = self._get_obs()
        return obs, reward, done, truncated, {}  
        
    def render(self):
        if self.render_mode == "human":
            # Print the DSM as a matrix
            print("Current DSM:")
            print(self.DSM)
            print("Power flow: ")
            print(self.powerflows)
        
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