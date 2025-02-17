% ==============
% TU/e QSS Toolbox
% MGB control block initialization script
% ==============

%% Global variables
global nr_gearsGL
set_param(gcb, 'LinkStatus', 'none'); % break link with library
set_param([gcb, '/ws_g_p'],'VariableName',['g_p',num2str(ControlnumberSim)]);