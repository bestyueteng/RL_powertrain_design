% Transmission: single gear * 1
%               multi gear * 3 (two motor, one generator)
% Motor:        Electric generator * 1
%               Electric motor * 2 (one B, one FC)
%               Fuel cell * 1
% Storage:      Battery * 1
%               Tank * 1

ComponentNumber = [1, 2, 2, 2, 3, 4, 5, 5, 5]; % 1: Vehicle Body; 2: Transmission; 3: Motor; 4: Storage; 5: Controller
TypeOfComponent = [1, 1, 2, 3, 1, 1, 1, 2, 3]; 
NumberOfInstances = [1, 1, 1, 1, 2, 1, 1, 1, 1];

component_library = table(ComponentNumber, TypeOfComponent, NumberOfInstances);

component_class = ["VehicleBody", "GearSystems", "EnergyConverters", "EnergyStorage", "Controller"];
component_type = {"Vehicle body", ["Final Drive", "Simple transmission", "Multispeed gearbox"], ...
    "Electric motor 1", "Battery", ["Torque Split", "Torque Coupler", "Electric Power Link"]};

DSM = [[0, 0, 0, 0, 0, 0, 0, 1, 0, 0]
[0, 0, 0, 0, 0, 0, 0, 1, 1, 0] 
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0] 
[0, 0, 0, 0, 0, 1, 0, 0, 1, 0] 
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0] 
[0, 0, 0, 1, 0, 0, 1, 0, 0, 0] 
[0, 0, 0, 0, 0, 1, 0, 0, 0, 0] 
[1, 1, 0, 0, 0, 0, 0, 0, 0, 0] 
[0, 1, 0, 1, 0, 0, 0, 0, 0, 0] 
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0]];
optimizing_method = 'test';
opt_tol = 0.5;
total_cost = evaluation(ComponentNumber,TypeOfComponent,NumberOfInstances,DSM,component_class,component_type,optimizing_method,opt_tol);



%% Current best

% DSM = [
%     0,0,0,0,0,0,0,0,0,0,0,1,0,0;
%     0,0,0,0,0,0,0,0,0,0,0,1,1,0;
%     0,0,0,0,0,0,0,1,0,0,0,1,0,0;
%     0,0,0,0,0,1,0,0,0,0,0,0,1,0;
%     0,0,0,0,0,0,0,0,1,0,0,1,0,0;
%     0,0,0,1,0,0,0,0,0,0,0,0,0,1;
%     0,0,0,0,0,0,0,0,0,0,1,0,1,0;
%     0,0,1,0,0,0,0,0,0,0,0,0,0,1;
%     0,0,0,0,0,0,0,0,0,0,0,0,0,1;
%     0,0,0,0,0,0,0,0,0,0,0,0,0,1;
%     0,0,0,0,0,0,1,0,0,0,0,0,0,0;
%     1,1,1,0,1,0,0,0,0,0,0,0,0,0;
%     0,1,0,1,0,0,1,0,0,0,0,0,0,0;
%     0,0,0,0,0,1,0,1,1,1,0,0,0,0;
% ];
