% function generate_block(name)
% 
%     syms x y
%     r = sqrt(x^2 + y^2);
%     k = x;
%     q = y;
%     matlabFunctionBlock(name, r,k,q)
% end



function create_blocks(comclass, comtype, comconnectionsclass, comconnectionsinstance, modelName)
    num_tc = 0;
    num_bc = 0;
    num_bl = 0;
    num_fcl = 0;
    for i = 1:length(comclass)
        if comclass(i) == 'GearSystems'
            subclass = comconnectionsclass{i};
            % Only final drive will connect to the torque coupler
            if ismember('VehicleBody', subclass)
                if  ~ ismember('EnergyStorage', subclass)
                    for j = 1:length(subclass)
                        if subclass(j) == "EnergyConverters" || subclass(j) == "GearSystems"
                            num_tc = num_tc + 1;
                        end
                    end
                       
                elseif ismember('EnergyStorage', subclass)
                    num_bc = num_bc + 1;
                end
            end
        elseif comclass(i) == 'EnergyConverters'
            subclass = comconnectionsclass{i};
            if ismember('EnergyStorage', subclass) && comtype(i) ~= 'Fuel cell'
                
                num_bl = num_bl + 1;  
            end
            if ismember('EnergyStorage', subclass) && comtype(i) == 'Fuel cell'
                
                num_fcl = num_fcl + 1;  
            end
        end
    end
    
    if num_tc ~= 0
        num_tc
        block_name = [modelName '/Component_torque_coupler_'];
        add_block('simulink/User-Defined Functions/MATLAB Function', block_name);
        generate_torque_coupler_function(modelName, block_name,comtype, num_tc);
    end

    if num_bc ~= 0
        block_name = [modelName '/Component_battery_coupler_'];
        add_block('simulink/User-Defined Functions/MATLAB Function', block_name);
        generate_battery_coupler_function(modelName, block_name,comtype, num_bc);
    end

    % if num_bl ~= 0
    %     block_name = [modelName '/Battery_power_list_'];
    %     add_block('simulink/User-Defined Functions/MATLAB Function', block_name);
    %     generate_powerlist_function(block_name, num_bl);
    % 
    %     block_name = [modelName '/B_VectorConcatenate'];
    %     add_block('simulink/Commonly Used Blocks/Vector Concatenate', block_name);
    %     set_param(block_name,'NumInputs', num2str(num_bl));
    %     set_param(block_name,'Mode','Multidimensional array');
    %     set_param(block_name, 'ConcatenateDimension', num2str(num_bl));
    % 
    % end
    % 
    % if num_fcl ~= 0
    %     block_name = [modelName '/FuelCell_power_list_'];
    %     add_block('simulink/User-Defined Functions/MATLAB Function', block_name);
    %     generate_powerlist_function(block_name, num_bl);
    % 
    %     block_name = [modelName '/FC_VectorConcatenate'];
    %     add_block('simulink/Commonly Used Blocks/Vector Concatenate', block_name);
    %     set_param(block_name,'NumInputs', num2str(num_bl));
    %     set_param(block_name,'Mode','Multidimensional array');
    %     set_param(block_name, 'ConcatenateDimension', num2str(num_bl));
    % 
    % end
end
