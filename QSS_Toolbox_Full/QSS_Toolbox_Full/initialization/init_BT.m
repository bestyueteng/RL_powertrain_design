% TU/e QSS Toolbox
% Battery block initialization script
% ==============

%% Global variables
global batterymodel                         % Battery QP model

%% Load data
load('Battery_model.mat');
batterymodel = model;
set_param(gcb, 'LinkStatus', 'none'); % break link with library
set_param([gcb, '/ws_P_BT'],'VariableName',['P_BT',num2str(batnumberSim)]);
set_param([gcb, '/ws_SoC'],'VariableName',['SoC',num2str(batnumberSim)]);
set_param([gcb, '/ws_cons_BT'],'VariableName',['cons_BT',num2str(batnumberSim)]);