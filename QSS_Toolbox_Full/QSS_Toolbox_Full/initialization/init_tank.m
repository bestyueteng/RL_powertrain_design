% ==============
% TU/e QSS Toolbox
% Tank block initialization script
% ==============

%% Global variables
global h                            % Stepsize [s] from block "Driving Cycle"

%% Select fuel type
switch fuel
    case 1                              % -> Gasoline
        H_u         = 42.7e6;           % Bosch-Manual
        rho_f       = 0.745;            % Bosch-Manual
    case 2                              % -> Diesel
        H_u         = 42.5e6;           % Bosch-Manual
        rho_f       = 0.84;             % Bosch-Manual
    case 3                              % -> Hydrogen
        H_u         = 142.9e6;          
        rho_f       = 0.089e-3;
end

%% Cold start losses
if (flag_cold_start == 1)           % -> Include cold start losses
    k_cs        = 1.15;
    
elseif (flag_cold_start == 0)       % -> Don't include cold start losses
    k_cs        = 1;
end

%% Other
Q_TK_IC         = 10;
liter2kwh       = H_u*rho_f/3.6e6;  % [l/100 km] -> [kWh/100 km]
