% ==============
% TU/e QSS Toolbox
% Supercapacitor block initialization script
% ==============

%% Global variables
global h                                                % Stepsize [s] from block "Driving Cycle"

%% Prepare data
C_SC			= C_SC * N_SC;                          % Total capacity                    [F]
P_SC_max		= P_SC_max * N_SC;                      % Total power                       [W]
R_SC			= R_SC / N_SC;                          % Total resistance                  [ohm]

Q_SC_IC         = (Q_SC_IC_rel/100) * C_SC * U_SC_max;  % Initial charge of each supercap   [C]
