function generate_blocks(comclass, comtype, modelName, DSM, comconnectionstype, comconnectionsinstance, component_type,componentStruct)

    add_block('DrivingCycle/Driving cycle', [modelName '/DC']);
    add_block('VehicleBody/Vehicle body', [modelName '/VB']);

    numComponents = length(comclass);
    num_mgb = 0;
    num_em = 0;
    num_bat = 0;
    num_sgb = 0;
    num_fd = 0;

    % Generate an empty struct to store component idx
    component_idx= struct();

    % Iterate through grouped data to populate the result
    for i = 1:length(component_type)
        subclass_type = {};
        subclass = component_type{i};
        if ischar(subclass)
            subclass_type{end+1} = subclass;
        else
            for j = 1:length(subclass)
                subclass_type{end+1} = subclass(j);
            end
        end
        for j = 1:length(subclass_type)
            type_name = strrep(subclass_type{j}, ' ', '');
            type_name = string(type_name);
            total_instances = 1;
            component_idx.(type_name) = total_instances;
        end
    end
    
    for i = 1:numComponents
        if ~all(DSM(i, :) == 0)
            if comclass(i) ~= 'VehicleBody' && comclass(i) ~= 'Controller' && comtype(i) ~= 'Final Drive'
                block_name = comclass(i)+'/'+comtype(i);
                type_name = strrep(comtype(i), ' ', '');
                idx = component_idx.(type_name);
                blockname = findBlock(componentStruct, i);
                component_name = [modelName '/' blockname];
                % component_name = strjoin(component_name, '');
                component_idx.(type_name) = component_idx.(type_name) + 1;
                add_block(block_name, component_name);
        
                if comtype(i) == 'Multispeed gearbox'
                    num_mgb = num_mgb + 1;

                    block_name = 'GearSystemsControl/Simple MGB controller';
                    add_block(block_name, [modelName '/MGB_controller_' num2str(idx)]);
                    
                    set_param(component_name,'transnumberSim',num2str(num_mgb));
                    set_param([modelName '/MGB_controller_' num2str(idx)],'ControlnumberSim',num2str(num_mgb));
                end

                if comtype(i) == 'Simple transmission'
                    num_sgb = num_sgb + 1;
                    set_param(component_name,'transnumberSim',num2str(num_sgb));
                end
                
                if comtype(i) == 'Electric motor 1'
                    num_em = num_em + 1;
                    set_param(component_name,'motornumberPM',num2str(5));
                    set_param(component_name,'motornumberSim',num2str(num_em));
                    % only for testing
                    set_param(component_name,'motortype','Induction (AC)');
                    set_param(component_name,'motornumberAC','75 kW');
                end
    
                if comtype(i) == 'Fuel cell'
                    set_param(component_name,'A_FC',num2str(1.9));
                end
    
                if comtype(i) == 'Tank'
                    set_param(component_name,'fuel','Hydrogen');
                end

                if comtype(i) == 'Battery'
                    num_bat = num_bat + 1;
                    set_param(component_name,'batnumberSim',num2str(num_bat));
                end
            end

            % FD will be a SGB
            if comtype(i) == 'Final Drive'
                num_fd = num_fd + 1;
                block_name = 'GearSystems/Simple transmission';
                % idx = component_idx.(type_name);
                blockname = findBlock(componentStruct, i);
                type_name = strrep(comtype(i), ' ', '');
                % component_name = [modelName '/' type_name '_' num2str(idx)];
                component_name = [modelName '/' blockname];
                % component_name = strjoin(component_name, '');
                
                component_idx.(type_name) = component_idx.(type_name) + 1;
                add_block(block_name, component_name);
                set_param(component_name,'transnumberSim',num2str(num_fd+10));
            end
            
            if comtype(i) == 'Electric Power Link'
                
                type_name = strrep(comtype(i), ' ', '');
                idx = component_idx.(type_name);
                % idx = findBlock(componentStruct, i);
                component_idx.(type_name) = component_idx.(type_name) + 1;

                % Get the number of in/out for the power link
                power_in = 0;
                power_out = 0;
                subclass = comconnectionstype{i}; % Get the type of connection components
                subconnection = comconnectionsinstance{i};
                for j = 1:length(subclass)
                    if subclass(j) == "EnergyConverters" % Motor will be power in, generator and fc will be power out
                        connected_com_id = subconnection(j);
                        if comtype(connected_com_id) == "Electric motor 1" 
                            power_in = power_in + 1;
                        else
                            power_out = power_out + 1;
                        end
                    elseif subclass(j) == "EnergyStorage"
                        power_out = power_out + 1;
                    end
                end
                
                % Generate the block
                block_name = [modelName '/ElectricPowerLink_' num2str(idx)];
                add_block('simulink/User-Defined Functions/MATLAB Function', block_name);
                generate_EPL_function(modelName, block_name, power_in, power_out);
        
                block_name = [modelName '/E_VectorConcatenate_' num2str(idx)];
                add_block('simulink/Commonly Used Blocks/Vector Concatenate', block_name);
                set_param(block_name,'NumInputs', num2str(power_in));
                set_param(block_name,'Mode','Multidimensional array');
                set_param(block_name, 'ConcatenateDimension', num2str(power_in));
                    
            end

            if comtype(i) == 'Torque Coupler'
                type_name = strrep(comtype(i), ' ', '');
                idx = component_idx.(type_name);
                % idx = findBlock(componentStruct, i);
                component_idx.(type_name) = component_idx.(type_name) + 1;

                torque_out = 0;
                subclass = comconnectionstype{i}; % Get the class of connection components
                subinstance = comconnectionsinstance{i};
                for j = 1:length(subclass)
                    if subclass(j) == "GearSystems"  
                        if comtype(subinstance(j)) ~= 'Final Drive' && ismember('EnergyConverters',comconnectionstype{subinstance(j)})  % no TC -> FD
                            torque_out = torque_out + 1;
                        end
                    end
                    if subclass(j) == "EnergyConverters"
                        torque_out = torque_out + 1;
                    end
                end

                 block_name = [modelName '/TorqueCoupler_' num2str(idx)];
                 add_block('simulink/User-Defined Functions/MATLAB Function', block_name);
                 generate_torque_coupler_function(modelName, block_name,comtype, torque_out, componentStruct);
            end

            if comtype(i) == 'Torque Split'
                type_name = strrep(comtype(i), ' ', '');
                idx = component_idx.(type_name);
                % idx = findBlock(componentStruct, i);
                component_idx.(type_name) = component_idx.(type_name) + 1;
                torque_out = 0;
                subclass = comconnectionstype{i}; % Get the type of connection components
                
                for j = 1:length(subclass)
                    if subclass(j) == "GearSystems" || subclass(j) == "EnergyConverters" 
                        torque_out = torque_out + 1;
                    end
                end

                 block_name = [modelName '/TorqueSplit_' num2str(idx)];
                 add_block('simulink/User-Defined Functions/MATLAB Function', block_name);
                 generate_torque_split_function(modelName, block_name, torque_out);
            end
                
            
            
            % rename_port([modelName '/Component_' num2str(i)],i,comclass(i)); % rename in/output ports
        end    
    end
  
end