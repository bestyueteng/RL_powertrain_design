%% Test procedure  - gradient at specified speed
%
% Required inputs
%  1. Workspace with loaded model parameters
%  2. Model name as string in variable 'sys_name'
%  3. Speed at which to test gradient in [m/s] in variable 'v_gradtest'
%  Important: driving cycle block should be named 'Driving cycle'
%  Important: model should run without other errors, overloads and etc.
%
% Output
%  1. Vehicle top speed in [rad] in variable 'gradSpeed_test'
%  Important: result is viable only if overload message is displayed in the
%  command window

%% Load system
load_system(sys_name);

%% Save current cycle 
old_cycle = get_param(sys_name+ "/Driving cycle",'cyclenr');

%% Change driving cycle
set_param(sys_name+ "/Driving cycle",'cyclenr',23);

%% Set test speed
set_param(sys_name+ "/Driving cycle",'gradient_speed',num2str(v_gradtest*3.6));

%% Run model
results_gradSp = sim(sys_name,'SrcWorkspace','current');

%% Determine top speed
gradSpeed_test = results_gradSp.slope(end);

%% Load old driving cycle 
set_param(sys_name+ "/Driving cycle",'cyclenr',old_cycle); 

%% Clear data
clearvars results_gradSp;