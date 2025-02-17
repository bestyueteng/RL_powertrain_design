% ==============
% TU/e QSS Toolbox
% Electric generator block initialization script
% ==============

%% Global variables
global h                            % Stepsize [s] from block "Driving Cycle"

%% Generator file name
if gentype == 1
    load('GE_data/generatorstringPM');
    gennumber = gennumberPM;
    messageEG = 'PM';
elseif gentype == 2
    load('GE_data/generatorstringCE');
    gennumber = gennumberAC;
    messageEG = 'CE';
end

genstring = components;
clear('components');

%% Load data
load(char(strcat(genstring(gennumber),'at')));
w_EG_upper = max(w_EG_max);         % Upper limit generator speed       [rad/s]

T_EG_col = scale_EG * T_EG_col;     % Scale generator torque
T_EG_max = scale_EG * T_EG_max;
