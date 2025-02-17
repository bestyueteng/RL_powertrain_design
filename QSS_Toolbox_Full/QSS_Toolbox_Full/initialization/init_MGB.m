% TU/e QSS Toolbox
% Manual Gear Box (MGB) block initialization script
% ==============

%% Global variables
global h            % Stepsize [s] from block "Driving Cycle"
global nr_gearsGL
nr_gearsGL = nr_gears + 1;
set_param(gcb, 'LinkStatus', 'none'); % break link with library
set_param([gcb, '/ws_i'],'VariableName',['i',num2str(transnumberSim)]);
set_param([gcb, '/ws_w_MGB'],'VariableName',['w_MGB',num2str(transnumberSim)]);
set_param([gcb, '/ws_P_MGB'],'VariableName',['P_MGB',num2str(transnumberSim)]);
set_param([gcb, '/ws_T_MGB'],'VariableName',['T_MGB',num2str(transnumberSim)]);