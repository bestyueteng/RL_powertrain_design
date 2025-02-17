function [omegas_m, torques_m, efficiencies_m, P_EM_max_m] = get_motor_data(motor_Data,scales)
    omegas_m = {};
    torques_m = {};
    efficiencies_m = {};
    P_EM_max_m = {};

    if ~iscell(motor_Data)
        motor_Data = {motor_Data};  % Convert to cell array
    end
    
    for i = length(motor_Data)
        filename = [motor_Data{i} '.mat'];
        motor_Data_m = load(filename);
        Scale_EM = scales(i);
        omegas_m{end+1} = motor_Data_m.w_EM_row; % Row values (speed)
        torques_m{end+1} = Scale_EM * motor_Data_m.T_EM_col; % Column values (torque)
        efficiencies_m{end+1} = motor_Data_m.eta_EM_mapM; % Efficiency matrix
        mask = efficiencies_m{end} > 1;
        efficiencies_m{end}(mask) = 1 ./ efficiencies_m{end}(mask);
        P_EM_max_m{end+1} = motor_Data_m.P_EM_max;
    end
end