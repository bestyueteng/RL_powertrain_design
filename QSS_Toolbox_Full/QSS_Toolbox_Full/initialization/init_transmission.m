% ==============
% TU/e QSS Toolbox
% Simple transmission block initialization script
% ==============

%% Global variables
global h            % Stepsize [s] from block "Driving Cycle"

%% Rename block and ToWorkspace variables
set_param(gcb, 'LinkStatus', 'none'); % break link with library

if transnumberSim > 10 % FD is also SBG, prevent repeat
    set_param([gcb, '/ws_w_trans'],'VariableName',['fd_w_trans',num2str(transnumberSim-10)]);
    set_param([gcb, '/ws_p_trans'],'VariableName',['fd_p_trans',num2str(transnumberSim-10)]);
    set_param([gcb, '/ws_t_trans'],'VariableName',['fd_t_trans',num2str(transnumberSim-10)]);
else
    set_param([gcb, '/ws_w_trans'],'VariableName',['w_trans',num2str(transnumberSim)]);
    set_param([gcb, '/ws_p_trans'],'VariableName',['p_trans',num2str(transnumberSim)]);
    set_param([gcb, '/ws_t_trans'],'VariableName',['t_trans',num2str(transnumberSim)]);
end