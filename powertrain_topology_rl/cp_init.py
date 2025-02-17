from ortools.sat.python import cp_model
import numpy as np
from powertrain_topology_rl.utils import * 
import random

def partial_init(components, power_types, random_dsm, valid_row=None):
    component_type = [
        "Vehicle body", ["Final Drive", "Simple transmission", "Multispeed gearbox"], 
        ["Electric motor 1"], 
        ["Battery"], 
        ["Torque Split", "Torque Coupler", "Electric Power Link"]
    ]
    # ----------------------------
    # 1. Structure the Components
    # ----------------------------
    num_components = len(components)
    node_classes = {idx: class_name for idx, (class_name, _) in enumerate(components)}
    node_types = {idx: type_name for idx, (_, type_name) in enumerate(components)}
    
    # ----------------------------
    # 2. Instantiate the CP Model
    # ----------------------------
    model = cp_model.CpModel()

    # ----------------------------
    # 3. Define Decision Variables (DSM and Usage)
    # ----------------------------
    DSM = {}
    for i in range(num_components):
        for j in range(i + 1, num_components):
            DSM[i, j] = model.NewBoolVar(f'DSM_{i}_{j}')

    used = {}
    for i in range(num_components):
        used[i] = model.NewBoolVar(f'used_{i}')

    # ----------------------------
    # 4. Create Mappings from Indices to Classes and Types
    # ----------------------------
    node_classes = {idx: cls for idx, (cls, _) in enumerate(components)}
    node_types = {idx: typ for idx, (_, typ) in enumerate(components)}

    # Map nodes to their power types
    power_in_types = {idx: power_in for idx, (power_in, _) in enumerate(power_types)}
    power_out_types = {idx: power_out for idx, (_, power_out) in enumerate(power_types)}

    # ----------------------------
    # 5. Define Component Groups Based on Classes and Types
    # ----------------------------
    controller_indices = [idx for idx, cls in node_classes.items() if cls == "Controller"]
    energy_storage_indices = [idx for idx, cls in node_classes.items() if cls == "EnergyStorage"]
    VB_indices = [idx for idx, cls in node_classes.items() if cls == "VehicleBody"]
    gearbox_indices = [idx for idx, typ in node_types.items() if typ in ["Final Drive", "Simple transmission", "Multispeed gearbox"]]
    gear_system_indices = [idx for idx, cls in node_classes.items() if cls == "GearSystems"]
    electric_motor_indices = [idx for idx, typ in node_types.items() if typ == "Electric motor 1"]
    electric_power_link_indices = [idx for idx, typ in node_types.items() if typ == "Electric Power Link"]
    torque_split_indices = [idx for idx, typ in node_types.items() if typ == "Torque Split"]
    torque_coupler_indices = [idx for idx, typ in node_types.items() if typ == "Torque Coupler"]
    final_drive_indices = [idx for idx, typ in node_types.items() if typ == "Final Drive"]
    multispeed_gearbox_indices = [idx for idx, typ in node_types.items() if typ == "Multispeed gearbox"]
    SM_indices = [idx for idx, typ in node_types.items() if typ in ["Simple transmission", "Multispeed gearbox"]]

    # ----------------------------
    # 6. Link DSM Variables to Usage Variables
    # ----------------------------
    for i in range(num_components):
        for j in range(i + 1, num_components):
            # If DSM[i,j] is 1, then both i and j must be used
            model.Add(DSM[i, j] <= used[i])
            model.Add(DSM[i, j] <= used[j])

    # Ensure that used[i] is 1 if any DSM connections involve i
    for i in range(num_components):
        connected = []
        for j in range(num_components):
            if i < j:
                connected.append(DSM.get((i, j), None))
            elif j < i:
                connected.append(DSM.get((j, i), None))
        connected = [c for c in connected if c is not None]
        if connected:
            model.AddMaxEquality(used[i], connected)
        else:
            model.Add(used[i] == 0)

    # ----------------------------
    # 7. Enforce Required Components to be Used
    # ----------------------------
    # Enforce that all VehicleBody components must be used
    for vb_idx in VB_indices:
        model.Add(used[vb_idx] == 1)

    # Enforce that all Torque Split components must be used
    for ts_idx in torque_split_indices:
        model.Add(used[ts_idx] == 1)
    
    def get_DSM(i, j):
        if i == j:
            return None  # No self-connection
        return DSM[min(i, j), max(i, j)]
        
    # ----------------------------
    # 7+. Enforce Valid Rows Constraints
    # ----------------------------
    if valid_row is not None:
        # Convert valid_row to a set for faster lookup
        valid_row_set = set(valid_row)
        
        for i in range(num_components):
            if i not in valid_row_set:
                # Components not in valid_row should not be used
                model.Add(used[i] == 0)
                # And they should not have any connections
                for j in range(num_components):
                    if i < j:
                        model.Add(DSM[i, j] == 0)
                    elif j < i:
                        model.Add(DSM[j, i] == 0)
                            
    # ----------------------------
    # 8. Define Constraints (1 to 10)
    # ----------------------------

    # Helper function to get DSM variable
    def get_DSM(i, j):
        if i == j:
            return None  # No self-connection
        return DSM[min(i, j), max(i, j)]

    # Constraint 1: Except VB, controllers and storages, all used components should have at most two connections
    for idx in range(num_components):
        if idx not in controller_indices and idx not in energy_storage_indices and idx not in VB_indices:
            connections = []
            for j in range(num_components):
                if j != idx:
                    var = get_DSM(idx, j)
                    if var is not None:
                        connections.append(var)
            # Apply constraint only if the component is used
            model.Add(sum(connections) <= 2 * used[idx])

    # Constraint 2: One energy storage can only have one connection if used
    for es_idx in energy_storage_indices:
        connections = []
        for j in range(num_components):
            if j != es_idx:
                var = get_DSM(es_idx, j)
                if var is not None:
                    connections.append(var)
        model.Add(sum(connections) == used[es_idx])

    # Constraint 3: Two gearboxes except FD cannot connect to each other
    # Assuming "Final Drive" is a type within "GearSystems"
    for i in SM_indices:
        for j in SM_indices:
            if i != j:
                # Enforce no connection between gearboxes
                model.Add(get_DSM(i, j) == 0)

    # Constraint 4: One Gear system cannot connect to more than one motor
    for gs_idx in gear_system_indices:
        connections = []
        for m_idx in electric_motor_indices:
            var = get_DSM(gs_idx, m_idx)
            if var is not None:
                connections.append(var)
        # Apply constraint only if gear system is used
        model.Add(sum(connections) <= 1 * used[gs_idx])

    # Constraint 5: EPL cannot connect to other two controllers
    for i in controller_indices:
        for j in electric_power_link_indices:
            # Enforce no connection between controllers
            if i != j:
                model.Add(get_DSM(i, j) == 0)

    # Constraint 6: Gear System except FD cannot connect to two controllers
    for gs_idx in SM_indices:
        connections = []
        for c_idx in controller_indices:
            var = get_DSM(gs_idx, c_idx)
            if var is not None:
                connections.append(var)
        # Apply constraint only if gear system is used
        model.Add(sum(connections) <= 1 * used[gs_idx])

    # Constraint 7: Motors should connect to at most two controller or one gear system
    for motor_idx in electric_motor_indices:
        connections_controller = []
        connections_gear_system = []
        for c_idx in controller_indices:
            var = get_DSM(motor_idx, c_idx)
            if var is not None:
                connections_controller.append(var)
        for gs_idx in gear_system_indices:
            var = get_DSM(motor_idx, gs_idx)
            if var is not None:
                connections_gear_system.append(var)

        model.Add(sum(connections_controller) <= 2 * used[motor_idx])
        model.Add(sum(connections_gear_system) <= 1 * used[motor_idx])

    # Constraint 8: Motor cannot connect to two energy storages or electric power links
    for motor_idx in electric_motor_indices:
        connections_energy_storage = []
        connections_epl = []
        for es_idx in energy_storage_indices:
            var = get_DSM(motor_idx, es_idx)
            if var is not None:
                connections_energy_storage.append(var)
        for epl_idx in electric_power_link_indices:
            var = get_DSM(motor_idx, epl_idx)
            if var is not None:
                connections_epl.append(var)
        model.Add(sum(connections_energy_storage) + sum(connections_epl) <= 1 * used[motor_idx])

    # # Constraint 9: TSC cannot connect to TC
    # for ts_idx in torque_split_indices:
    #     for tc_idx in torque_coupler_indices:
    #         if ts_idx < tc_idx:
    #             model.Add(get_DSM(ts_idx, tc_idx) == 0)
    #         else:
    #             model.Add(get_DSM(tc_idx, ts_idx) == 0)

    # Constraint 10: Power in and out types should match for connected components
    for i in range(num_components):
        for j in range(i + 1, num_components):
            var_ij = get_DSM(i, j)
            var_ji = get_DSM(j, i)  # This should be the same variable as var_ij, just accessed consistently.

            if var_ij is not None:
                
                model.AddImplication(var_ij, power_in_types[i] == power_out_types[j] or power_out_types[i] == power_in_types[j])
    
    # Constraint 11: Simple and Multispeed gearbox should connect to one motor
    for gs_idx in SM_indices:
        connections = []
        for m_idx in electric_motor_indices:
            var = get_DSM(gs_idx, m_idx)
            if var is not None:
                connections.append(var)
        model.Add(sum(connections) == 1 * used[gs_idx])
    # Check 1: Motor should be connected to a energy storage or Electric Power Link if used
    for motor_idx in electric_motor_indices:
        connections_energy_storage = []
        connections_epl = []
        for es_idx in energy_storage_indices:
            var = get_DSM(motor_idx, es_idx)
            if var is not None:
                connections_energy_storage.append(var)
        for epl_idx in electric_power_link_indices:
            var = get_DSM(motor_idx, epl_idx)
            if var is not None:
                connections_epl.append(var)
        model.Add(sum(connections_energy_storage) + sum(connections_epl) >= 1 * used[motor_idx])

    # Check 2: Motor and Gear System should have two connections if used
    for motor_idx in electric_motor_indices:
        connections = []
        for j in range(num_components):
            if j != motor_idx:
                var = get_DSM(motor_idx, j)
                if var is not None:
                    connections.append(var)
        model.Add(sum(connections) == 2 * used[motor_idx])

    for gs_idx in gear_system_indices:
        connections = []
        for j in range(num_components):
            if j != gs_idx:
                var = get_DSM(gs_idx, j)
                if var is not None:
                    connections.append(var)
        model.Add(sum(connections) == 2 * used[gs_idx])

    # Check 3: Controller should connect to at least two components if used
    for c_idx in controller_indices:
        connections = []
        for j in range(num_components):
            if j != c_idx:
                var = get_DSM(c_idx, j)
                if var is not None:
                    connections.append(var)
        model.Add(sum(connections) >= 2 * used[c_idx])

    # Check 4: Energy storage should connect to one component if used
    for es_idx in energy_storage_indices:
        connections = []
        for j in range(num_components):
            if j != es_idx:
                var = get_DSM(es_idx, j)
                if var is not None:
                    connections.append(var)
        model.Add(sum(connections) == used[es_idx])

    # Check 6: Final Drive should connect to Torque Split if used
    for fd_idx in final_drive_indices:
        connections = []
        for ts_idx in torque_split_indices:
            var = get_DSM(fd_idx, ts_idx)
            if var is not None:
                connections.append(var)
        model.Add(sum(connections) >= 1 * used[fd_idx])

    # Check 7: At least one Multispeed Gearbox is used
    mgb_connections = []
    for mgb_idx in multispeed_gearbox_indices:
        for j in range(num_components):
            if j != mgb_idx:
                var = get_DSM(mgb_idx, j)
                if var is not None:
                    mgb_connections.append(var)
    model.Add(sum(mgb_connections) >= 1)

    # # Check 8: Torque Coupler should connect to TSC or FD if used
    # for tc_idx in torque_coupler_indices:
    #     connections_tsc = []
    #     connections_fd = []
    #     for ts_idx in torque_split_indices:
    #         var = get_DSM(tc_idx, ts_idx)
    #         if var is not None:
    #             connections_tsc.append(var)
    #     for fd_idx in final_drive_indices:
    #         var = get_DSM(tc_idx, fd_idx)
    #         if var is not None:
    #             connections_fd.append(var)
    #     model.Add(sum(connections_tsc) + sum(connections_fd) >= 1 * used[tc_idx])

    # Check 9: Electric Power Link should connect to at least one energy storage if used
    for epl_idx in electric_power_link_indices:
        connections = []
        for es_idx in energy_storage_indices:
            var = get_DSM(epl_idx, es_idx)
            if var is not None:
                connections.append(var)
        model.Add(sum(connections) >= 1 * used[epl_idx])

    # Additional Constraint: VehicleBody can only connect to Torque Split and must have exactly one connection if used
    for vb_idx in VB_indices:
        connections_non_ts = []
        for j in range(num_components):
            if j != vb_idx and node_types[j] != "Torque Split":
                var = get_DSM(vb_idx, j)
                if var is not None:
                    connections_non_ts.append(var)
        # Enforce that VehicleBody does not connect to non-Torque Split components
        model.Add(sum(connections_non_ts) == 0)
        
        connections_ts = []
        for ts_idx in torque_split_indices:
            var = get_DSM(vb_idx, ts_idx)
            if var is not None:
                connections_ts.append(var)
        # Enforce that VehicleBody is connected to exactly one Torque Split
        model.Add(sum(connections_ts) == 1 * used[vb_idx])
    
    # Additional Constraint: At least one Electric Power Link must be used
    # model.Add(sum(used[i] for i in electric_motor_indices) >= 1)
    
    # ----------------------------------------
    #  Constraint 11: 2 * (Used FD) + (#Gearboxes connected to TS) + (Used TC) ? {2, 4}
    # ----------------------------------------

    # 1) Create an integer expression for "2 * sum_fd + sum_gb_conn_ts".
    # sum_fd is the sum of "used" variables for Final Drive.
    sum_fd = sum(used[fd_idx] for fd_idx in final_drive_indices)
    
    # sum_gb_conn_ts is the count of gearboxes directly connected to a Torque Split component.
    # Because DSM[g, ts] is a BoolVar, summing them yields an IntVar expression.
    connections_g_ts = []
    for g in SM_indices:
        for ts in torque_split_indices:
            var = get_DSM(g, ts)
            if var is not None:
                connections_g_ts.append(var)
    sum_gb_conn_ts = sum(connections_g_ts)
    
    connections_tc_ts = []
    for tc in torque_coupler_indices:
        for ts in torque_split_indices:
            var = get_DSM(tc, ts)
            if var is not None:
                connections_tc_ts.append(var)
    sum_tc_conn_ts = sum(connections_tc_ts)
    
    sum_fd_g_tc_ts = 2 * sum_fd + sum_gb_conn_ts + sum_tc_conn_ts
    eq_2 = model.NewBoolVar("eq_2")
    eq_4 = model.NewBoolVar("eq_4")

    # model.Add(sum_fd_g_ts == 2)

    model.Add(sum_fd_g_tc_ts == 2).OnlyEnforceIf(eq_2)

    model.Add(sum_fd_g_tc_ts == 4).OnlyEnforceIf(eq_4)

    # Disjunction: eq_2 OR eq_4
    model.AddBoolOr([eq_2, eq_4])
    
    # ----------------------------
    # 11. Define the Solver and Solution Printer
    # ----------------------------
    class DSMPrinter(cp_model.CpSolverSolutionCallback):
        """
        Callback class to print and store all feasible DSM solutions.
        """
        def __init__(self, DSM, used, num_components, component_names):
            cp_model.CpSolverSolutionCallback.__init__(self)
            self.DSM = DSM
            self.used = used
            self.num_components = num_components
            self.solution_count = 0
            self.raw_solution_count = 0
            self.component_names = component_names
            self.DSM_list = []  # List to store all DSM matrices

        def on_solution_callback(self):
            self.raw_solution_count += 1
            
            DSM_matrix = []
            for i in range(self.num_components):
                row = []
                for j in range(self.num_components):
                    if i < j:
                        val = self.Value(self.DSM[i, j])
                    elif i == j:
                        val = 0  # No self-connections
                    else:
                        val = self.Value(self.DSM[j, i])
                    row.append(val)
                DSM_matrix.append(row)
                # Optionally, print component usage and connections
                used_status = self.Value(self.used[i])
                if used_status:
                    connections = []
                    for j in range(self.num_components):
                        if i < j:
                            val = self.Value(self.DSM[i, j])
                        elif j < i:
                            val = self.Value(self.DSM[j, i])
                        else:
                            val = 0  # No self-connections
                        if val:
                            connections.append(j)
                    connection_str = ', '.join([f"{j} ({self.component_names[j][0]}, {self.component_names[j][1]})" for j in connections])
                else:
                    pass
            
            
            # Append the DSM matrix to the list
            DSM_np = np.array(DSM_matrix)
            is_duplicate = False
            for idx, existing_matrix in enumerate(self.DSM_list):
                existing_np = np.array(existing_matrix)
                if check_same_topology(DSM_np, existing_np, node_classes, node_types, component_type):
                    is_duplicate = True
                    duplicate_idx = idx
                    break

            if not is_duplicate:
                # This is a unique solution, so store it and maybe print it
                self.DSM_list.append(DSM_matrix)
                self.solution_count += 1
            
            if is_duplicate:
                if random.random() <= 0.5: # make the diversity of dsm list
                    self.DSM_list[duplicate_idx] = DSM_matrix

    # Instantiate the solver
    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = 10.0
    
    #--- Randomization: set a random seed and use portfolio search ---
    random_seed = random.randint(1, 1000000)
    solver.parameters.random_seed = random_seed
    solver.parameters.search_branching = cp_model.PORTFOLIO_SEARCH
    
    # Instantiate the solution printer
    solution_printer = DSMPrinter(DSM, used, num_components, components)

    # Solve the Model and Collect Solutions
    status = solver.SearchForAllSolutions(model, solution_printer)
            
    return solution_printer.DSM_list
    # if status in [cp_model.OPTIMAL, cp_model.FEASIBLE]:
        # if solution_printer.DSM_list:
            # idx = random.randint(0, len(solution_printer.DSM_list) - 1)
            # DSM_matrix = solution_printer.DSM_list[idx]
            # return DSM_matrix
    # return None
    
    
    
    