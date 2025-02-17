%% Test procedure  - top speed
%
% Required inputs
%  1. Workspace with loaded model parameters
%  2. Model name as string in variable 'sys_name'
%  Important: driving cycle block should be named 'Driving cycle'
%  Important: model should run without other errors, overloads and etc.
%
% Output
%  1. Vehicle top speed in [m/s] in variable 'topSpeed_test'
%  Important: result is viable only if overspeed message is displayed in the
%  command window

%% Load system
load_system(sys_name);

%% Save current cycle 
old_cycle = get_param(sys_name+ "/Driving cycle",'cyclenr');

%% Change driving cycle
set_param(sys_name+ "/Driving cycle",'cyclenr',21);

%% Run model
results_topspeed = sim(sys_name,'SrcWorkspace','current');

%% Determine top speed
topSpeed_test = results_topspeed.v(end);

%% Load old driving cycle 
set_param(sys_name+ "/Driving cycle",'cyclenr',old_cycle); 

%% Clear data
clearvars results_topspeed;