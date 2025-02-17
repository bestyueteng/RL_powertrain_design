import numpy as np
from powertrain_topology_rl.utils import *

def check_feasibility(powerflows,component_library):
    """
    Checks the feasibility of a set of powerflow paths based on several domain-specific constraints:

    1. Every (non-empty) path must contain:
       - VB (id 1)
       - TS (id 2) [manual constraint]
       - One form of transmission (SGB=5 or MGB=6)
       - EM (id 8)
       - Battery (id 10)

    2. A path cannot contain both FC1 (id 4) and TC2 (id 7) at the same time.

    3. If there are >=2 paths using EM (id 8), then every path must contain the ePowertrain link (id 9).

    4. If more than 2 paths use FD (id 3), at least one path must contain FC1 (id 4). 
       (In other words, "TC1 is needed if there are more than 2 paths using FD".)

    5. Maximum number of FD (id 3) is 2 (counting FD->TC collectively as one).

    6. 2*FD + (#gearboxes directly after TS) must be 2 or 4.

    :param powerflows: A list (or array) of paths. 
                       Each path is an array of component IDs in the order they appear.
    :return: Boolean indicating feasibility (True/False).
    """
    # Optional: not used in the logic, but kept for reference
    id_to_str = {
        1: 'VB',   # Vehicle Body
        2: 'TS',   # Torque Split
        3: 'FD',   # Final Drive
        4: 'FC1',  # FD -> FC
        5: 'SGB',  # Single Gearbox
        6: 'MGB',  # Multi Gearbox
        7: 'TC2',  # FC -> EM
        8: 'EM',   # Electric Motor
        9: 'EPL',  # ePowertrain
        10: 'B'    # Battery
    }
    
    # 1) Every path must have VB, TS, EM, Battery, and at least one gearbox (SGB or MGB).
    for path in powerflows:
        # Skip empty (all-zero) paths
        if not np.all(path == 0):

            # Check mandatory components
            has_vb    = (1 in path)
            has_ts    = (2 in path)
            has_em    = (8 in path)
            has_batt  = (10 in path)
            has_gear  = (6 in path)  # Transmission check
            
            if not (has_vb and has_ts and has_em and has_batt and has_gear):
                return False

    # 2) A path cannot have both FC1 (4) and TC2 (7)
    for path in powerflows:
        if 4 in path and 7 in path:
            return False

    # 3) If >=2 paths use EM, then every path must contain EPL (9).
    num_paths_with_em = sum(1 for path in powerflows if 8 in path)
    if num_paths_with_em >= 2:
        for path in powerflows:
            # Skip empty paths
            if np.all(path == 0):
                continue
            if 9 not in path:
                return False

    # 4) If more than 2 paths use FD, then at least one path must have FC1 (4)
       # (The comment "TC1 is needed if there are more than 2 paths using FD" 
        # implies at least one path must have 4 in it.)
    num_paths_with_fd = sum(1 for path in powerflows if 3 in path)
    if num_paths_with_fd > 2:
        # Check if any path has FC1 (4)
        if not any(4 in path for path in powerflows):
            return False

    # 5) Maximum FD usage is 2 (counting FD->TC collectively as one).
    num_fd = 0
    tc_detected = False
    for path in powerflows:
        if np.all(path == 0):
            continue
        
        # If FD is in path, check what comes after FD to decide counting
        if 3 in path:
            fd_indices = np.where(path == 3)[0]
            for fd_idx in fd_indices:
                # Guard against out-of-range
                if fd_idx + 1 < len(path):
                    after_fd = path[fd_idx + 1]
                    # FD -> SGB or FD -> MGB
                    if after_fd in (5, 6):
                        num_fd += 1
                    # FD -> FC1 (4) -> (TC)
                    elif after_fd == 4 and not tc_detected:
                        num_fd += 1
                        tc_detected = True
                else:
                    # FD is the last element in path; treat it as a used FD
                    num_fd += 1

    if num_fd > 2:
        return False

    # 6) 2*FD + (gearboxes directly after TS) must be 2 or 4
    num_gb_after_ts = 0
    for path in powerflows:
        if np.all(path == 0):
            continue

        # Check for single or multi gearbox that follows TS
        for gb_id in [5, 6]:
            if gb_id in path:
                gb_indices = np.where(path == gb_id)[0]
                for gb_idx in gb_indices:
                    if gb_idx - 1 >= 0 and path[gb_idx - 1] == 2:
                        num_gb_after_ts += 1

    # If 2*FD + GB->TS = 2 or 4, it's valid; else False
    if 2 * num_fd + num_gb_after_ts not in (2, 4):
        return False
    
    num_of_instances = component_library[2]
    components, _ = map_flow_to_topology(powerflows)
    for i in range(len(components)):
        if components[i] > num_of_instances[i]:
            # print("Component: ", i, " is not feasible.")
            return False
            
    return True
