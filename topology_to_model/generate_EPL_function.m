function generate_EPL_function(modelName, block_name, power_in, power_out)
    
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
    funcScript = generateFunctionScript(power_out);

    % Access the MATLABFunctionConfiguration object
    configObj = get_param(block_name, 'MATLABFunctionConfiguration');

    % Set the function script
    configObj.FunctionScript = funcScript;
end

function funcScript = generateFunctionScript(power_out)
    % Generate the function signature based on power_out
    outputs = {'p1', 'p2', 'p3', 'p4', 'p5'};
    outputList = strjoin(outputs(1:power_out), ', ');
    funcSignature = sprintf('function [%s] = electric_power_list_function(power_in)\n', outputList);
    % Initialize the function body
    funcBody = '';
    % Insert your function body code, converted to a string
    % Note: We need to replace line breaks with '\n' and handle any special characters

    funcBody1 = [
    '    coder.extrinsic(''set_param'');', newline, ...
    '    coder.extrinsic(''get_param'');', newline, ...
    '    coder.extrinsic(''regexp'');', newline, ...
    '    P = 0;', newline, ...
    '    for i = 1:length(power_in)', newline, ...
    '        P = P + power_in(i);', newline, ...
    '    end', newline, ...
    '    power_split = (P/0.98); % Efficiency is 0.98', newline, ...  % Simply equal dvide the power. Need to change for other controls
    ];

    funcBody2 = [];
    for i = 1:power_out
        fun_str = sprintf(['%s = power_split/' [int2str(power_out)] ';\n'], outputs{i});
        funcBody2 = [funcBody2, fun_str];
    end

    funcScript = [funcSignature, funcBody1,funcBody2, 'end'];
end
%     syms add [1 power_in]
% 
%     P = 0;
%     for i = 1:length(add)
%         P = P + add(i);
%     end
% 
%     matlabFunctionBlock(block_name,P,'Vars',{add}, 'Outputs',{'Power'}, 'Optimize',false)
% end