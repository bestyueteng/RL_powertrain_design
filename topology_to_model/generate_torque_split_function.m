function generate_torque_split_function(modelName, block_name, num_tc)
    
    % Ensure the Simulink model is open
    open_system(modelName);

    % Check if the MATLAB Function block exists; if not, create it
    if ~bdIsLoaded(modelName)
        load_system(modelName);
    end

    % if ~exist_block(block_name)
    %     add_block('simulink/User-Defined Functions/MATLAB Function', block_name);
    % end

    % Generate the function script as a string
    funcScript = generateFunctionScript(num_tc);

    % Access the MATLABFunctionConfiguration object
    configObj = get_param(block_name, 'MATLABFunctionConfiguration');

    % Set the function script
    configObj.FunctionScript = funcScript;
end

function funcScript = generateFunctionScript(num_tc)
    % Generate the function signature based on num_tc
    outputs = {'w', 'dw', 't1', 't2', 't3', 't4', 't5'};
    outputList = strjoin(outputs(1:(2 + num_tc)), ', ');
    funcSignature = sprintf('function [%s] = torque_split_function(w_wheel, dw_wheel, t_wheel)\n', outputList);

    funcBody1 = [
    '    coder.extrinsic(''set_param'');', newline, ...
    '    coder.extrinsic(''get_param'');', newline, ...
    '    coder.extrinsic(''regexp'');', newline, ...
    '    w = w_wheel;', newline, ...
    '    dw = dw_wheel;', newline, ...
    '    t1 = 0;', newline, ...
    '    t2 = 0;', newline, ...
    '    t3 = 0;', newline, ...
    '    t4 = 0;', newline, ...
    '    t5 = 0;', newline, ...
    ];

    funcBody2 = [];
    for i = 1:num_tc
        fun_str = sprintf('%s = (t_wheel/0.98) / %s ; % Efficiency is 0.98\n', outputs{i+2}, int2str(num_tc));
        funcBody2 = [funcBody2, fun_str];
    end

    funcScript = [funcSignature, funcBody1, funcBody2, 'end'];
end