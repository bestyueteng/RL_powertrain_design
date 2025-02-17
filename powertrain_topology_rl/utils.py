import matlab.engine
import numpy as np
import networkx as nx
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.logger import Figure
import matplotlib.pyplot as plt
import random
import copy
import re

def structure_components(component_library, component_class, component_type):
    # Build the list of components
    components = []
    for i in range(len(component_library['Component number'])):
        class_idx = component_library['Component number'][i] - 1
        type_idx = component_library['Type of Components'][i] - 1
        num_instances = component_library['Number of Instances'][i]
        class_name = component_class[class_idx]
        type_name = component_type[class_idx][type_idx]
        for _ in range(num_instances):
            components.append((class_name, type_name))
    
    return components

def get_performance(component_library, DSM, component_class,component_type, opt_method='fminsearch',opt_tol=0.5):
    eng = matlab.engine.start_matlab()
    version = eng.version()
    print(matlab.engine.__file__)

    eng.addpath(r'/mech001.mnt/home/20223748/code_hpc/topology_to_model/', nargout=0)
    total_cost = eng.evaluation(component_library[0], component_library[1], component_library[2], DSM, component_class,component_type, opt_method,opt_tol)
    return total_cost


def dict_to_matrix(component_library):
        matrix = np.stack((component_library['Component number'], component_library['Type of Components'], component_library['Number of Instances']))
        return matrix

def initialize_DSM(node_types, num_components):
    init_DSM = np.zeros((num_components, num_components))

    # Connect VB to one of TSC
    tsc_idx = [idx for idx, cls in node_types.items() if cls == "Torque Split"]
    for idx in tsc_idx:
        init_DSM[0][idx] = 1
        init_DSM[idx][0] = 1
        break
    
    # init_DSM[0, 1] = 1 # VehicleBody -> GearSystems
    # init_DSM[1, 0] = 1 # GearSystems -> VehicleBody
    # init_DSM[1, 6] = 1 # GearSystems -> EnergyConverters
    # init_DSM[6, 1] = 1 # EnergyConverters -> GearSystems
    # init_DSM[6, 15] = 1 # EnergyConverters -> Controller
    # init_DSM[15, 6] = 1 # Controller -> EnergyConverters
    # init_DSM[15, 12] = 1 # Controller -> Battery
    # init_DSM[12, 15] = 1 # Battery -> Controller

    return init_DSM

def get_power_types(component_library, component_class, component_type):
    """
    Get the power types of the components in the library
    For Vehicle body, power-out is mechanical and no power-in
    For transmission models, the in and out power types are all mechanical
    For Motor, power-in is mechanical and power-out is electrical
    For Generator, power-in is electrical and power-out is mechanical
    For Fuel cell, power-in is electrical and power-out is chemical
    For Battery, power-in is electrical no power-out
    For Tank, power-in is chemical no power-out
    """

    power_types = []
    for i in range(len(component_library['Component number'])):
        class_idx = component_library['Component number'][i] - 1
        type_idx = component_library['Type of Components'][i] - 1
        num_instances = component_library['Number of Instances'][i]
        class_name = component_class[class_idx]
        type_name = component_type[class_idx][type_idx]
        for j in range(num_instances):
            if class_name == 'VehicleBody':
                power_types.append(('', 'mechanical'))
            if class_name == 'GearSystems':
                power_types.append(('mechanical', 'mechanical'))
            if class_name == 'EnergyConverters':
                if type_name == 'Electric generator':
                    power_types.append(('electrical', 'mechanical'))
                elif type_name == 'Electric motor 1':
                    power_types.append(('mechanical', 'electrical'))
                elif type_name == 'Fuel cell':
                    power_types.append(('electrical', 'chemical'))
            if class_name == 'EnergyStorage':
                if type_name == 'Battery':
                    power_types.append(('electrical', ''))
                elif type_name == 'Tank':
                    power_types.append(('chemical', ''))
            if class_name == 'Controller':
                if type_name == 'Electric Power Link':
                    power_types.append(('electrical', 'electrical'))
                elif type_name == 'Torque Coupler':
                    power_types.append(('mechanical', 'mechanical'))
                elif type_name == 'Torque Split':
                    power_types.append(('mechanical', 'mechanical'))

    return power_types

def generate_constraints(node_classes, node_types, num_components):
    constraints = []

    def constraint_1(DSM):
        # Except VB, controllers and storages, all used components should have at most two connections
        controller_indices = [idx for idx, cls in node_classes.items() if cls == "Controller" and any(DSM[idx])]
        energy_storage_indices = [idx for idx, cls in node_classes.items() if cls == "EnergyStorage" and any(DSM[idx])]
        VB_indices = [idx for idx, cls in node_classes.items() if cls == "VehicleBody" and any(DSM[idx])]
        for idx in range(num_components):
            if idx not in controller_indices and idx not in energy_storage_indices and idx not in VB_indices:
                if sum(DSM[idx]) > 2:
                    return False
        return True
    
    constraints.append(constraint_1)

    def constraint_2(DSM):
        # One storage can only have one connection
        energy_storage_indices = [idx for idx, cls in node_classes.items() if cls == "EnergyStorage" and any(DSM[idx])]
        
        for es_idx in energy_storage_indices:
            if sum(DSM[es_idx]) > 1:
                return False
        return True

    constraints.append(constraint_2)

    def constraint_3(DSM):
        # Two gearboxes except FD can not connect to each other
        gearbox_indices = [idx for idx, cls in node_types.items() if cls == "Simple transmission" or cls == "Multispeed gearbox" and any(DSM[idx])]
        for gb_idx1 in gearbox_indices:
            for gb_idx2 in gearbox_indices:
                if DSM[gb_idx1][gb_idx2]:
                    return False
        return True

    constraints.append(constraint_3)

    def constraint_4(DSM):
        # One Gear system cannot connect to more than one motor
        gear_system_indices = [idx for idx, cls in node_classes.items() if cls == "GearSystems" and any(DSM[idx])]
        motor_indices = [idx for idx, cls in node_types.items() if cls == "Electric motor 1" and any(DSM[idx])]
        for gs_idx in gear_system_indices:   
            if sum(DSM[gs_idx][motor_indices]) > 1:
                return False
        return True
    
    constraints.append(constraint_4)

    def constraint_5(DSM):
        # EPL cannot connect with other two controllers
        epl_indices = [idx for idx, cls in node_types.items() if cls == "Electric Power Link" and any(DSM[idx])]
        controller_indices = [idx for idx, cls in node_types.items() if cls == "Controller" and any(DSM[idx])]
        for epl_idx in epl_indices:
            if sum(DSM[epl_idx][controller_indices]) > 0:
                return False
        return True
    
    constraints.append(constraint_5)

    def constraint_6(DSM):
        # Gear System except FD cannot connect to two controllers
        gear_system_indices = [idx for idx, cls in node_types.items() if cls == "Simple transmission" or cls == "Multispeed gearbox" and any(DSM[idx])]
        controller_indices = [idx for idx, cls in node_classes.items() if cls == "Controller" and any(DSM[idx])]
        for gs_idx in gear_system_indices:
            if sum(DSM[gs_idx][controller_indices]) > 1:
                return False
        return True
    
    constraints.append(constraint_6)

    def constraint_7(DSM):
        # Motors should connect to at most two controllers or one gear system
        motor_indices = [idx for idx, cls in node_types.items() if cls == "Electric motor 1" and any(DSM[idx])]
        controller_indices = [idx for idx, cls in node_types.items() if cls == "Torque Split" or cls == "Torque Coupler" and any(DSM[idx])]
        gear_system_indices = [idx for idx, cls in node_classes.items() if cls == "GearSystems" and any(DSM[idx])]
        for motor_idx in motor_indices:
            if sum(DSM[motor_idx][controller_indices]) > 2 or sum(DSM[motor_idx][gear_system_indices]) > 1:
                return False
        return True
    
    constraints.append(constraint_7)

    def constraint_8(DSM):
        # Motor can not connect to two enerage storages or electric power links
        motor_indices = [idx for idx, cls in node_types.items() if cls == "Electric motor 1" and any(DSM[idx])]
        energy_storage_indices = [idx for idx, cls in node_classes.items() if cls == "EnergyStorage" and any(DSM[idx])]
        controller_indices = [idx for idx, cls in node_types.items() if cls == "Electric Power Link" and any(DSM[idx])]
        for motor_idx in motor_indices:
            if sum(DSM[motor_idx][energy_storage_indices]) + sum(DSM[motor_idx][controller_indices]) > 1:
                return False
        return True
    
    constraints.append(constraint_8)
    
    # def constraint_9(DSM):
    #     # TSC cannot connect to TC
    #     torque_split_indices = [idx for idx, cls in node_types.items() if cls == "Torque Split" and any(DSM[idx])]
    #     torque_coupler_indices = [idx for idx, cls in node_types.items() if cls == "Torque Coupler" and any(DSM[idx])]
    #     for ts_idx in torque_split_indices:
    #         if any(DSM[ts_idx][torque_coupler_indices]):
    #             return False
    #     return True
    
    # constraints.append(constraint_9)
    
    def constraint_10(DSM):
        # Components cannot connect to those have the same type
        for i in range(num_components):
            for j in range(num_components):
                if DSM[i][j]:
                    if node_types[i] == node_types[j]:
                        return False
        return True
    
    constraints.append(constraint_10)
    
    def constraint_11(DSM):
        # 2*final drive + gearbox + TC which connects to torque split should be 2 or 4
        final_drive_indices = [idx for idx, cls in node_types.items() if cls == "Final Drive" and any(DSM[idx])]
        gearbox_indices = [idx for idx, cls in node_classes.items() if cls == "GearSystems" and any(DSM[idx])]
        torque_split_indices = [idx for idx, cls in node_types.items() if cls == "Torque Split" and any(DSM[idx])]
        torque_coupler_indices = [idx for idx, cls in node_types.items() if cls == "Torque Coupler" and any(DSM[idx])]  
        num_fd = 0
        for fd_idx in final_drive_indices:
            num_fd += sum(DSM[fd_idx])
        num_gearbox = 0
        for gb_idx in gearbox_indices:
            if gb_idx not in final_drive_indices:
                # check if connected with torque split
                if any(DSM[gb_idx][torque_split_indices]):
                    num_gearbox += sum(DSM[gb_idx])
        num_tc = 0
        for tc_idx in torque_coupler_indices:
            if any(DSM[tc_idx][torque_split_indices]):
                num_tc += sum(DSM[tc_idx])

        if 2*num_fd + num_gearbox + num_tc == 2 or 2*num_fd + num_gearbox + num_tc == 4:
            return True
        else:
            return False
            
    constraints.append(constraint_11)
    
    def constraint_12(DSM):
        # if Simple and Multispeed gearbox have two connections, they should connect to one motor
        SM_gearbox_indices = [idx for idx, cls in node_types.items() if cls == "Simple transmission" or cls == "Multispeed gearbox" and any(DSM[idx])]
        motor_indices = [idx for idx, cls in node_types.items() if cls == "Electric motor 1" and any(DSM[idx])]
        for gb_idx in SM_gearbox_indices:
            if sum(DSM[gb_idx]) == 2:
                if sum(DSM[gb_idx][motor_indices]) != 1:
                    return False
        return True
    
    constraints.append(constraint_12)
    
    return constraints
    


def check_constraints(DSM, constraints):
    for idx, constraint in enumerate(constraints):
        if not constraint(DSM):
            return True
    return False


def generate_check_list(node_classes, node_types):
    check_list = []
    def check_1(DSM):
        # Motor should be connected to a energy storage or Electric Power Link
        motor_indices = [idx for idx, cls in node_types.items() if cls == "Electric motor 1" and any(DSM[idx])]
        energy_storage_indices = [idx for idx, cls in node_classes.items() if cls == "EnergyStorage" and any(DSM[idx])]
        controller_indices = [idx for idx, cls in node_types.items() if cls == "Electric Power Link" and any(DSM[idx])]
        for motor_idx in motor_indices:
            if not any(DSM[motor_idx][energy_storage_indices]) and not any(DSM[motor_idx][controller_indices]):
                return False
        return True

    check_list.append(check_1)

    def check_2(DSM):
        # Motor and Gear System should have 2 connections
        motor_indices = [idx for idx, cls in node_types.items() if cls == "Electric motor 1" and any(DSM[idx])]
        gear_system_indices = [idx for idx, cls in node_classes.items() if cls == "GearSystems" and any(DSM[idx])]
        for motor_idx in motor_indices:
            if sum(DSM[motor_idx]) != 2:
                return False
        for gs_idx in gear_system_indices:
            if sum(DSM[gs_idx]) != 2:
                return False
        return True
    
    check_list.append(check_2)

    def check_3(DSM):
        # Controller should connect to at least two components
        controller_indices = [idx for idx, cls in node_classes.items() if cls == "Controller" and any(DSM[idx])]
        for c_idx in controller_indices:
            if sum(DSM[c_idx]) < 2:
                return False
        return True
    
    check_list.append(check_3)

    def check_4(DSM):
        # Energy storage should connect to one component
        energy_storage_indices = [idx for idx, cls in node_classes.items() if cls == "EnergyStorage" and any(DSM[idx])]
        for es_idx in energy_storage_indices:
            if sum(DSM[es_idx]) != 1:
                return False
        return True
    
    check_list.append(check_4)

    def check_5(DSM):
        """
        Checks the completeness of the topology based on the new requirements:
        For all used components, there should be a path connecting from transmission to energy converter to energy storage.
        Paths that only include transmission and energy converter or only energy converter and energy storage are not considered complete.
        """
        # Build the undirected graph from the symmetric DSM
        G = nx.Graph()
        size = len(DSM)
        G.add_nodes_from(range(size))

        for i in range(size):
            for j in range(i+1, size):
                if DSM[i][j]:
                    G.add_edge(i, j)

        # Get indices of vehicle body and energy storages
        vehicle_body_indices = [idx for idx, cls in node_classes.items() if cls == "VehicleBody" ]
        energy_storage_indices = [idx for idx, cls in node_classes.items() if cls == "EnergyStorage"]

        # Get used indices of vehicle body and energy storages\
        vehicle_body_indices = [idx for idx in vehicle_body_indices if any(DSM[idx])]
        energy_storage_indices = [idx for idx in energy_storage_indices if any(DSM[idx])]


        # Get all used components (nodes with at least one connection)
        used_components = set()
        for i in range(size):
            if any(DSM[i]) or any(DSM[:, i]):
                used_components.add(i)

        # For each used component, check if there is a path from vehicle body to any energy storage that includes it
        for component in used_components:
            path_found = False
            for vb_idx in vehicle_body_indices:
                for es_idx in energy_storage_indices:
                    try:
                        paths = nx.all_simple_paths(G, source=vb_idx, target=es_idx)
                        for path in paths:
                            # Check if path includes at least one GearSystem and one EnergyConverter
                            has_gearsystem = any(node_classes[node] == 'GearSystems' for node in path)
                            has_energymotor = any(node_types[node] == 'Electric motor 1'  for node in path)
                            if has_gearsystem and has_energymotor:
                                if component in path:
                                    path_found = True
                                    break  # Found a path including the component
                        if path_found:
                            break
                    except nx.NetworkXNoPath:
                        continue
                if path_found:
                    break
            if not path_found:
                return False  # No path including this component
            
        return True

    check_list.append(check_5)

    def check_6(DSM):
        # FD should connect to TSC
        final_drive_indices = [idx for idx, cls in node_types.items() if cls == "Final Drive" and any(DSM[idx])]
        torque_split_indices = [idx for idx, cls in node_types.items() if cls == "Torque Split" and any(DSM[idx])]
        for ts_idx in torque_split_indices:
            if final_drive_indices is not None and final_drive_indices:
                if not any(DSM[ts_idx][final_drive_indices]):
                    return False
        return True
    
    check_list.append(check_6)

    def check_7(DSM):
        # Used gearbox should at least have one MGB
        mgb_indices = [idx for idx, cls in node_types.items() if cls == "Multispeed gearbox" and any(DSM[idx])]
        for mgb_idx in mgb_indices:
            if any(DSM[mgb_idx]):
                return True
        return False
    
    check_list.append(check_7)
    
    # def check_8(DSM):
        # # Torque Coupler should connect to TSC or FD
        # torque_coupler_indices = [idx for idx, cls in node_types.items() if cls == "Torque Coupler" and any(DSM[idx])]
        # torque_split_indices = [idx for idx, cls in node_types.items() if cls == "Torque Split" and any(DSM[idx])]
        # final_drive_indices = [idx for idx, cls in node_types.items() if cls == "Final Drive" and any(DSM[idx])]
        # for tc_idx in torque_coupler_indices:
        #     if not any(DSM[tc_idx][torque_split_indices]) and not any(DSM[tc_idx][final_drive_indices]):
        #         return False
        # return True
    
    # check_list.append(check_8)

    def check_9(DSM):
        # EPL should connect to at least one energy storage
        epl_indices = [idx for idx, cls in node_types.items() if cls == "Electric Power Link" and any(DSM[idx])]
        energy_storage_indices = [idx for idx, cls in node_classes.items() if cls == "EnergyStorage" and any(DSM[idx])]
        for epl_idx in epl_indices:
            if not any(DSM[epl_idx][energy_storage_indices]):
                return False
        return True
    
    check_list.append(check_9)
    
    return check_list


def check_power_flow_completeness(DSM, check_list):
    for check in check_list:
        if not check(DSM):
            return False
    return True

# Redirect stdout to logging (wrap print statements)
class LoggerWriter:
    def __init__(self, logger, level):
        self.logger = logger
        self.level = level

    def write(self, message):
        if message != '\n':  # Avoids logging empty newlines
            self.level(message)

    def flush(self):
        pass  # Required for file-like objects, but no action needed here
        
        
class FigureRecorderCallback(BaseCallback):
    def __init__(self, verbose=0):
        super().__init__(verbose)
        self.episode_rewards = []
        self.current_episode_reward = 0.0

    def _on_step(self):
        # Get the reward and done signal from the environment
        reward = self.locals['rewards']
        done = self.locals['dones']

        # Accumulate reward
        self.current_episode_reward = reward[0]  # Assuming a single environment

        if done[0]:
            # Episode finished
            self.episode_rewards.append(self.current_episode_reward)
            self.current_episode_reward = 0.0

            # Plot the episode rewards
            figure = plt.figure()
            plt.plot(self.episode_rewards)
            plt.xlabel('Episode')
            plt.ylabel('Total Reward')
            plt.title('Episode Rewards Over Time')

            # Log the figure to TensorBoard
            self.logger.record(
                "trajectory/figure",
                Figure(figure, close=True),
                exclude=("stdout", "log", "json", "csv"),
            )
            plt.close(figure)
        return True

def randomize_instances(component_lib, start_index=1, min_val=1, max_val=3):
    new_lib = copy.deepcopy(component_lib)

    # cannot change the last 4 components
    for i in range(start_index, len(new_lib['Number of Instances'])-4):
        new_lib['Number of Instances'][i] = random.randint(min_val, max_val)
    return new_lib

def find_all_valid_rows(new_lib, old_lib):
    new_instances = new_lib['Number of Instances']
    old_instances = old_lib['Number of Instances']
    row_count = 0
    valid_rows = []
    for idx, rows in enumerate(old_instances):
        # valid rows are from row_count to row_count + rows - new_instances[idx]
        for i in range(new_instances[idx]):
            valid_rows.append(row_count + i)
        row_count += rows
    return valid_rows
    
def make_graph(DSM):
    G = nx.Graph()
    size = len(DSM)
    G.add_nodes_from(range(size))

    for i in range(size):
        for j in range(i+1, size):
            if DSM[i][j]:
                G.add_edge(i, j)
    return G
    
def generate_lexicographic_ordering(DSM, node_classes, node_types, component_type):
    # enumerate the component type
    component_type = [item for sublist in component_type for item in sublist]
    
    char_list = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z']

    # map type to char
    char_node_types = []
    for idx, cls in node_types.items():
        char_node_types.append(char_list[component_type.index(cls)])
    

    # Build the undirected graph from the symmetric DSM
    G = nx.from_numpy_array(DSM)
    size = len(DSM)
    # Get indices of vehicle body and energy storages
    vehicle_body_indices = [idx for idx, cls in node_classes.items() if cls == "VehicleBody" ]
    energy_storage_indices = [idx for idx, cls in node_classes.items() if cls == "EnergyStorage"]

    # Get used indices of vehicle body and energy storages\
    vehicle_body_indices = [idx for idx in vehicle_body_indices if any(DSM[idx])]
    energy_storage_indices = [idx for idx in energy_storage_indices if any(DSM[idx])]


    # Get all used components (nodes with at least one connection)
    used_components = set()
    for i in range(size):
        if any(DSM[i]) or any(DSM[:, i]):
            used_components.add(i)

    all_paths = []
    for vb_idx in vehicle_body_indices:
        for es_idx in energy_storage_indices: 
            paths = nx.all_simple_paths(G, source=vb_idx, target=es_idx)
            for path in paths:
                all_paths.append(path)
    
   
    # generate a lexicographic ordering for the paths using the char_node_types
    char_path = []
    for path in all_paths:
        char_path.append(''.join([char_node_types[node] for node in path]))
    
    char_path.sort()
    

    return char_path

def check_same_topology(DSM1, DSM2, node_classes, node_types, component_type):
    char_path1 = generate_lexicographic_ordering(DSM1, node_classes, node_types, component_type)
    char_path2 = generate_lexicographic_ordering(DSM2, node_classes, node_types, component_type)

    return char_path1 == char_path2
    
def map_flow_to_topology(powerflows: np.ndarray):
    """
    Takes a 2D array of integer-coded powerflow paths and generates:
      1. A component_list (count of each component type).
      2. An adjacency matrix representing connectivity among components.
    """

    # 1) Filter out any path that has fewer than 4 non-zero elements.
    for idx, path in enumerate(powerflows):
        if np.count_nonzero(path) < 4:
            powerflows[idx] = 0

    # 2) Convert the numeric codes to string names.
    id_to_str = {
        1: 'VB',   # Vehicle Body
        2: 'TS',   # Torque Split
        3: 'FD',   # Final Drive
        4: 'TC',  # FD -> FC
        5: 'SGB',  # Single Gearbox
        6: 'MGB',  # Multi Gearbox
        7: 'TC',   # FC -> EM
        8: 'EM',   # Electric Motor
        9: 'EPL',  # ePowertrain
        10: 'BAT'     # Battery
    }

    powerflows_list = powerflows.tolist()

    for idx, path in enumerate(powerflows_list):
        powerflows_list[idx] = [id_to_str.get(val, 0) for val in path if val != 0]
    # 3) Initialize component counts.
    num_em  = 0
    num_fd  = 0
    num_sgb = 0
    num_mgb = 0
    num_epl = 0
    num_tc  = 0

    tc_detected  = False
 
    epl_detected = False

    # component_list holds the count of each component in order:
    # [VB, FD, SGB, MGB, EM, B, TS, TC, EPL]
    component_list = np.zeros(9, dtype=int)

    # 4) Process powerflows to rename and count components.
    for idx, path in enumerate(powerflows_list):

        for i in range(len(path) - 1):
            current_comp = path[i]
            next_comp    = path[i + 1]

            if current_comp.startswith("EM"):
                num_em += 1
                component_list[4] = num_em
                path[i] = f"EM{num_em}"

            # Handle FD -> TC transitions
            if current_comp.startswith("FD"):
                if next_comp.startswith("TC"):
                    if not tc_detected:
                        num_fd += 1
                        tc_detected = True
                        component_list[1] = num_fd  # FD count
                    # Rename FD with index (e.g., FD1, FD2, etc.)
                    path[i] = f"FD{num_fd}"
                else:
                    # If FD doesn't transition to TC, increment FD index
                    num_fd += 1
                    component_list[1] = num_fd
                    path[i] = f"FD{num_fd}"
            
            if current_comp.startswith("SGB"):
                if next_comp.startswith("TC"):
                    if not tc_detected:
                        num_sgb += 1
                        tc_detected = True
                        component_list[2] = num_sgb
                    
            if current_comp.startswith("MGB"):
                if next_comp.startswith("TC"):
                    if not tc_detected:
                        num_mgb += 1
                        tc_detected = True
                        component_list[3] = num_mgb
                        
            if current_comp.startswith("TS"):
                if next_comp.startswith("TC"):
                    if not tc_detected:
                        tc_detected = True
                        
            # Handle SGB -> EM transitions
            if current_comp.startswith("SGB") and next_comp.startswith("EM"):
                num_sgb += 1
                component_list[2] = num_sgb                
                path[i]   = f"SGB{num_sgb}"
                
            # Handle MGB -> EM transitions
            if current_comp.startswith("MGB") and next_comp.startswith("EM"):
                num_mgb += 1
                component_list[3] = num_mgb                
                path[i]   = f"MGB{num_mgb}"
                

            # Detect EPL usage
            if current_comp.startswith("EM") and next_comp == "EPL":
                epl_detected = True
            
            if current_comp.startswith("VB"):
                component_list[0] = 1
            if next_comp.startswith("BAT"): # battery always at last pos
                component_list[5] = 1
            if current_comp.startswith("TS"):
                component_list[6] = 1

    # If TC/EPL detected, update their counts.
    if tc_detected:
        num_tc = 1
        component_list[7] = num_tc
    
    if epl_detected:
        num_epl = 1
        component_list[8] = num_epl

    # 5) Build the adjacency matrix.
    order_map = {
        'VB':  0,
        'FD':  1,
        'SGB': 2,
        'MGB': 3,
        'EM':  4,
        'BAT': 5,
        'TS':  6,
        'TC':  7,
        'EPL': 8
    }

    # Collect all unique nodes.
    unique_nodes = set()
    for path in powerflows_list:
        unique_nodes.update(path)
    
    # A helper to parse the component names for sorting (e.g., FD2 -> prefix=FD, index=2).
    def parse_component(comp: str):
        match = re.match(r'([A-Za-z]+)(\d*)', comp)
        if not match:
            return (9999, 9999)
        prefix, suffix_str = match.groups()
        suffix_val = int(suffix_str) if suffix_str else 0
        return order_map.get(prefix, 9999), suffix_val

    # Sort the nodes based on (prefix_order, numeric_suffix).
    node_order = sorted(unique_nodes, key=parse_component)
    
    adjacency_matrix = np.zeros((len(node_order), len(node_order)), dtype=int)

    # A helper to get the index of a node in the sorted list.
    def get_index(node):
        return node_order.index(node)

    # Fill adjacency matrix (undirected).
    for path in powerflows_list:
        for i in range(len(path) - 1):
            start_idx = get_index(path[i])
            end_idx   = get_index(path[i + 1])
            adjacency_matrix[start_idx][end_idx] = 1
            adjacency_matrix[end_idx][start_idx] = 1

    return component_list, adjacency_matrix

    
