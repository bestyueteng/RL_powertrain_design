% ==============
% TU/e QSS Toolbox
% Electric motor block initialization script
% ==============

%% Global variables
global h                    % Stepsize [s] from block "Driving Cycle"
            
%% Motor file name
if motortype == 1
    load('EM_data/motorstringPM');
    motornumber = motornumberPM;
    messageEM = 'PM';
elseif motortype == 2
    load('EM_data/motorstringAC');
    motornumber = motornumberAC;
    messageEM = 'AC';
end

motorstring = components;
clear('components');

%% Load data
load(char(strcat(motorstring(motornumber),'at')));
w_EM_upper = max(w_EM_max);                                 % Upper limit motor speed       [rad/s]

T_EM_col = scale_EM * T_EM_col;     % Scale motor torque
T_EM_max = scale_EM * T_EM_max;

theta_EM = inertia;
Pmax = T_EM_max(end)*w_EM_max(end);
messageEM = [messageEM, num2str(Pmax)];

%% Rename block and ToWorkspace variables
set_param(gcb, 'LinkStatus', 'none'); % break link with library
set_param([gcb, '/outw'],'VariableName',['w_EM',num2str(motornumberSim)]);
set_param([gcb, '/outT'],'VariableName',['T_EM',num2str(motornumberSim)]);
set_param([gcb, '/outP'],'VariableName',['P_EM',num2str(motornumberSim)]);
% set_param([gcb],'Name',['Electric motor ',num2str(motornumberSim)]);
