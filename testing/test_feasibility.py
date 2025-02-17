from powertrain_topology_rl.utils import *

DSM =  np.array([
    [0., 0., 0., 0., 0., 0., 0., 1., 0., 0.],
    [0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
    [0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
    [0., 0., 0., 0., 0., 1., 0., 1., 0., 0.],
    [0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
    [0., 0., 0., 1., 0., 0., 1., 0., 0., 0.],
    [0., 0., 0., 0., 0., 1., 0., 0., 0., 0.],
    [1., 0., 0., 1., 0., 0., 0., 0., 0., 0.],
    [0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
    [0., 0., 0., 0., 0., 0., 0., 0., 0., 0.]
])

# Example data (same as before)
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

components = structure_components(component_library, component_class, component_type)
num_components = len(components)
# Map nodes to their component classes
node_classes = {idx: class_name for idx, (class_name, _) in enumerate(components)}

# Map nodes to their component types
node_types = {idx: type_name for idx, (_, type_name) in enumerate(components)}
constraints = generate_constraints(node_classes, node_types, num_components)
check_list = generate_check_list(node_classes, node_types) # for power flow constraints
constraints_violated = check_constraints(DSM, constraints)
is_feasible = check_power_flow_completeness(DSM, check_list)
print("Constraints violated:", constraints_violated)
print("Feasible topology:", is_feasible)