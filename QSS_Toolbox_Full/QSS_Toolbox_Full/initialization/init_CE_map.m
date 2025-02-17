% TU/e QSS Toolbox
% Combustion Engine block (based on consumption map) initialization script
% ==============

%% Global variables
global h                                    % Stepsize [s] from block "Driving Cycle"
% global eta_CE_map
% global w_CE_row
% global T_CE_col
% global w_CE_max
% global T_CE_max
% global intertia

%% Load data
if enginetype == 1
    load('CE_data/enginestringCI');
    enginenumber = enginenumberCI;
elseif enginetype == 2
    load('CE_data/enginestringSI');
    enginenumber = enginenumberSI;
end

enginestring = components;
clear('components');

load(char(strcat(enginestring(enginenumber),'at')));

T_CE_col   = T_CE_col.*scale_CE;    % Torque range                              [Nm]
T_CE_max   = T_CE_max.*scale_CE;    % Maximum torque                            [Nm]
V_CE_map   = V_CE_map .* scale_CE;  % Scale engine consumption 
w_CE_upper = max(w_CE_max);         % Upper limit engine speed                  [rad/s]
w_CE_idle = w_idle;
P_CE_idle = P_idle;
T_CE_idle  = P_CE_idle / w_CE_idle; % Torque at idle                            [Nm]

theta_CE = inertia;                 % engine inertia                            [kg*m^2]

%% Which engine type?
switch engine_type

case 1                                      % -> Otto
    H_u         = 42.7e6;                   % Bosch-Manual
    rho_f       = 0.745;                    % Bosch-Manual

case 2                                      % -> Diesel
    H_u         = 42.5e6;                   % Bosch-Manual
    rho_f       = 0.84;                     % Bosch-Manual
end
