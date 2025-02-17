function create_connections_old(comclass, comtype, comconnectionsclass, comconnectionsinstance, modelName, DSM)
    
    add_line(modelName, 'DC/1', 'VB/1', 'autorouting', 'smart');
    add_line(modelName, 'DC/2', 'VB/2', 'autorouting', 'smart');
    
    % Ports = get_param([modelName '/Battery_power_list_'], 'Ports');
    % numInPorts = Ports(1);

    num_gen = 0;
    num_motor = 0;
    num_motor_fc = 0;
    num_motor_b = 0;
    num_tc = 0;
    num_bc = 0;
    power_in = 0;
    power_out = 0;

    for i = 1:length(comclass)
        if ~all(DSM(i, :) == 0)
            switch comclass(i)
                case 'GearSystems'
                   
                    connections = comconnectionsclass{i};
                    if ismember('VehicleBody',connections) % Final Drive
                        
                        % Connect vehicle body

                        inputs = 3;
                        srcBlock = 'VB';
                        dstBlock = ['Component_' num2str(i)];
        
                        for q = 1:inputs
                            try
                                add_line(modelName, [srcBlock '/' num2str(q)], [dstBlock '/' num2str(q)], 'autorouting', 'smart');
                            catch ME
                                warning('Could not add line from %s to %s: %s', srcBlock, dstBlock, ME.message);
                            end
                        end
                        
                        % Connect FD and torque coupler
                        srcBlock = ['Component_' num2str(i)];
                        dstBlock = 'Component_torque_coupler_';
                        for q = 1:inputs
                            try
                                add_line(modelName, [srcBlock '/' num2str(q)], [dstBlock '/' num2str(q)], 'autorouting', 'on');
                            catch ME
                                warning('Could not add line from %s to %s: %s', srcBlock, dstBlock, ME.message);
                            end
                        end
                        
                        % Connect FD and its direct motor (Motor will be
                        % directly connected with torque coupler)
                        srcBlock = 'Component_torque_coupler_';
                        connectionsinstance = comconnectionsinstance{i};
                        for k = 1:length(connectionsinstance)
                            if comclass(connectionsinstance(k))=='EnergyConverters'
                                dstBlock = ['Component_' num2str(connectionsinstance(k))];
                                num_tc = num_tc + 1;
                                try
                                    add_line(modelName, [srcBlock '/' num2str(1)], [dstBlock '/' num2str(1)], 'autorouting', 'smart');
                                    add_line(modelName, [srcBlock '/' num2str(2)], [dstBlock '/' num2str(2)], 'autorouting', 'smart');
                                    add_line(modelName, [srcBlock '/' num2str(num_tc+2)], [dstBlock '/' num2str(3)], 'autorouting', 'smart');
                                catch ME
                                    warning('Could not add line from %s to %s: %s', srcBlock, dstBlock, ME.message);
                                end
                                
                            elseif comclass(connectionsinstance(k))=='GearSystems' 
                                dstBlock = ['Component_' num2str(connectionsinstance(k))];
                                num_tc = num_tc + 1;
                                try
                                    add_line(modelName, [srcBlock '/' num2str(1)], [dstBlock '/' num2str(2)], 'autorouting', 'smart');
                                    add_line(modelName, [srcBlock '/' num2str(2)], [dstBlock '/' num2str(3)], 'autorouting', 'smart');
                                    add_line(modelName, [srcBlock '/' num2str(num_tc+2)], [dstBlock '/' num2str(4)], 'autorouting', 'smart');
                                catch ME
                                    warning('Could not add line from %s to %s: %s', srcBlock, dstBlock, ME.message);
                                end
                            end
                        end
                        
                        % if comtype(i) == 'Multispeed gearbox'
                        %     try
                        %         add_line(modelName, ['MGB_controller_' num2str(i) '/' num2str(1)], [dstBlock '/' num2str(1)], 'autorouting', 'smart');
                        %         add_line(modelName, [srcBlock '/' num2str(1)], ['MGB_controller_' num2str(i) '/' num2str(1)], 'autorouting', 'smart');
                        %     catch ME
                        %         warning('Could not add line from %s to %s: %s', srcBlock, dstBlock, ME.message);
                        %     end
                        % end
        
                    else % Other transmission system
        
                        % if ismember('EnergyStorage', connections) % Generator side, not fixed
                        % 
                        %     num_bc = num_bc + 1;
                        % 
                        %     srcBlock = 'Component_battery_coupler_';
                        %     dstBlock = ['Component_' num2str(i)];
                        % 
                        %     try
                        %         add_line(modelName, [srcBlock '/' num2str(1)], [dstBlock '/' num2str(2)], 'autorouting', 'smart');
                        %         add_line(modelName, [srcBlock '/' num2str(2)], [dstBlock '/' num2str(3)], 'autorouting', 'smart');
                        %         add_line(modelName, [srcBlock '/' num2str(num_bc+2)], [dstBlock '/' num2str(4)], 'autorouting', 'smart');
                        %     catch ME
                        %         warning('Could not add line from %s to %s: %s', srcBlock, dstBlock, ME.message);
                        %     end
                        % 
                        % 
                        %     if comtype(i) == 'Multispeed gearbox'
                        %         try
                        %             add_line(modelName, ['MGB_controller_' num2str(i) '/' num2str(1)], [dstBlock '/' num2str(1)], 'autorouting', 'smart');
                        %             add_line(modelName, [srcBlock '/' num2str(3*(num_bc-1)+1)], ['MGB_controller_' num2str(i) '/' num2str(1)], 'autorouting', 'smart');
                        %         catch ME
                        %             warning('Could not add line from %s to %s: %s', srcBlock, dstBlock, ME.message);
                        %         end
                        %     end


                        if ismember('EnergyConverters', connections) % Motor side
                        
                            srcBlock = ['Component_' num2str(i)];
                            for k = 1:length(connectionsinstance)
                                if comtype(connectionsinstance(k))=='Energy Motor 1'
                                    dstBlock = ['Component_' num2str(connectionsinstance(k))];
                                    try
                                        add_line(modelName, [srcBlock '/' num2str(1)], [dstBlock '/' num2str(1)], 'autorouting', 'smart');
                                        add_line(modelName, [srcBlock '/' num2str(2)], [dstBlock '/' num2str(2)], 'autorouting', 'smart');
                                        add_line(modelName, [srcBlock '/' num2str(3)], [dstBlock '/' num2str(3)], 'autorouting', 'smart');
                                    catch ME
                                        warning('Could not add line from %s to %s: %s', srcBlock, dstBlock, ME.message);
                                    end
                                elseif comtype(connectionsinstance(k))=='Electric generator'
                                    dstBlock = ['Component_' num2str(connectionsinstance(k))];
                                    try
                                        add_line(modelName, [srcBlock '/' num2str(1)], [dstBlock '/' num2str(1)], 'autorouting', 'smart');
                                        add_line(modelName, [srcBlock '/' num2str(3)], [dstBlock '/' num2str(2)], 'autorouting', 'smart');
                                    catch ME
                                        warning('Could not add line from %s to %s: %s', srcBlock, dstBlock, ME.message);
                                    end
                                end
                            end


                            if comtype(i) == 'Multispeed gearbox'
                                srcBlock = 'Component_torque_coupler_';
                                dstBlock = ['Component_' num2str(i)];
                                try
                                    add_line(modelName, ['MGB_controller_' num2str(i) '/' num2str(1)], [dstBlock '/' num2str(1)], 'autorouting', 'smart');
                                    add_line(modelName, [srcBlock '/' num2str(1)], ['MGB_controller_' num2str(i) '/' num2str(1)], 'autorouting', 'smart');
                                catch ME
                                    warning('Could not add line from %s to %s: %s', srcBlock, dstBlock, ME.message);
                                end
                            end
                        end
                    end
    
                case 'EnergyConverters'
                    
                   
                    if comtype(i) == 'Electric generator' % not fixed
                        
                        % check and find if connected to a gearbox
                        connectionsinstance = comconnectionsinstance{i};
                        for k = 1:length(connectionsinstance)
                            if comclass(connectionsinstance(k))=='GearSystems'
                                srcBlock = ['Component_' num2str(connectionsinstance(k))];
                                break
                            end
                        end
        
                        dstBlock = ['Component_' num2str(i)];
        
                        
                        try
                            add_line(modelName, [srcBlock '/' num2str(1)], [dstBlock '/' num2str(1)], 'autorouting', 'smart');
                            add_line(modelName, [srcBlock '/' num2str(3)], [dstBlock '/' num2str(2)], 'autorouting', 'smart');
                        catch ME
                            warning('Could not add line from %s to %s: %s', srcBlock, dstBlock, ME.message);
                        end
                        
                        

                        
                        % srcBlock = ['Component_' num2str(i)];
                        % dstBlock = 'Battery_power_list_';
                        % try
                        %     add_line(modelName, [srcBlock '/' num2str(1)], [dstBlock '/' num2str(numInPorts-num_gen)], 'autorouting', 'smart');
                        % catch ME
                        %     warning('Could not add line from %s to %s: %s', srcBlock, dstBlock, ME.message);
                        % end
                        % num_gen = num_gen +1;
    
                    elseif comtype(i) == 'Electric motor 1'
                        
    
                        num_motor = num_motor + 1;
                        inputs = 3;
                        connectionsinstance = comconnectionsinstance{i};
                         % check if connected to a gearbox
                        % for k = 1:length(connectionsinstance)
                        %     if comclass(connectionsinstance(k))=='GearSystems'
                        %         srcBlock = ['Component_' num2str(connectionsinstance(k))];
                        %         break
                        %     end
                        % end
                        % 
                        % dstBlock = ['Component_' num2str(i)];
                        % for q = 1:inputs
                        %     try
                        %         add_line(modelName, [srcBlock '/' num2str(q)], [dstBlock '/' num2str(q)], 'autorouting', 'smart');
                        %     catch ME
                        %         warning('Could not add line from %s to %s: %s', srcBlock, dstBlock, ME.message);
                        %     end
                        % end
    
                        if  check_fc(connectionsinstance, comtype) ~= 0  % check if directly powered by FC
                            srcBlock = ['Component_' num2str(i)];
                            % Find the Fuel Cell it connects
                            for k = 1:length(connectionsinstance)
                                if comtype(connectionsinstance(k))=='Fuel cell'
                                    dstBlock = ['Component_' num2str(connectionsinstance(k))];
                                break
                                end
                            end

                            try
                                add_line(modelName, [srcBlock '/' num2str(1)], [dstBlock '/' num2str(1)], 'autorouting', 'smart');
                            catch ME
                                warning('Could not add line from %s to %s: %s', srcBlock, dstBlock, ME.message);
                            end
                        end

                        if check_bat(connectionsinstance, comtype) ~= 0 % check if directly powered by battery
                            srcBlock = ['Component_' num2str(i)];

                            % Find the Battery it connects
                            for k = 1:length(connectionsinstance)
                                if comtype(connectionsinstance(k))=='Battery'
                                    dstBlock = ['Component_' num2str(connectionsinstance(k))];
                                break
                                end
                            end

                            try
                                add_line(modelName, [srcBlock '/' num2str(1)], [dstBlock '/' num2str(1)], 'autorouting', 'smart');
                                add_line(modelName, ['DC' '/' num2str(3)], [dstBlock '/' num2str(2)], 'autorouting', 'smart');
                            catch ME
                                warning('Could not add line from %s to %s: %s', srcBlock, dstBlock, ME.message);
                            end
                        end
    
                            % num_motor_b = num_motor_b + 1;
                            % 
                            % srcBlock = ['Component_' num2str(i)];
                            % dstBlock = 'B_VectorConcatenate';
                            % try
                            %     add_line(modelName, [srcBlock '/' num2str(1)], [dstBlock '/' num2str(num_motor_b)], 'autorouting', 'smart');
                            % catch ME
                            %     warning('Could not add line from %s to %s: %s', srcBlock, dstBlock, ME.message);
                            % end
                            % num_gen = num_gen +1;
                         
    
    
                            
                    elseif comtype(i) == 'Fuel cell' 
                        
                        connectionsinstance = comconnectionsinstance{i};
                        for k = 1:length(connectionsinstance)
                            if comtype(connectionsinstance(k))=='Tank'
                                % check first motor needed
                                    dstBlock = ['Component_' num2str(connectionsinstance(k))]; 
                                break
                            end
                        end
    
                        srcBlock = ['Component_' num2str(i)];
                        try
                            add_line(modelName, [srcBlock '/' num2str(1)], [dstBlock '/' num2str(1)], 'autorouting', 'smart');
                            add_line(modelName, ['DC' '/' num2str(3)], [dstBlock '/' num2str(2)], 'autorouting', 'smart');
                        catch ME
                            warning('Could not add line from %s to %s: %s', srcBlock, dstBlock, ME.message);
                        end
                        
                    end

                % case 'EnergyStorage'
                %     % if comtype(i) == 'Battery'
                %     %     srcBlock = 'Battery_power_list_';
                %     %     dstBlock = ['Component_' num2str(i)];
                %     %     try
                %     %         add_line(modelName, [srcBlock '/' num2str(1)], [dstBlock '/' num2str(1)], 'autorouting', 'smart');
                %     %         add_line(modelName, ['DC' '/' num2str(3)], [dstBlock '/' num2str(2)], 'autorouting', 'smart');
                %     %     catch ME
                %     %         warning('Could not add line from %s to %s: %s', srcBlock, dstBlock, ME.message);
                %     %     end
                %     % 
                %     %     srcBlock = ['Component_' num2str(i)];
                %     %     dstBlock = 'Component_battery_coupler_';
                %     % 
                %     %     try
                %     %         add_line(modelName, [srcBlock '/' num2str(1)], [dstBlock '/' num2str(1)], 'autorouting', 'smart');
                %     %     catch ME
                %     %         warning('Could not add line from %s to %s: %s', srcBlock, dstBlock, ME.message);
                %     %     end
                %     % 
                %     %     srcBlock = 'B_VectorConcatenate';
                %     %     dstBlock = 'Battery_power_list_';
                %     % 
                %     %     try
                %     %         add_line(modelName, [srcBlock '/' num2str(1)], [dstBlock '/' num2str(1)], 'autorouting', 'smart');
                %     %     catch ME
                %     %         warning('Could not add line from %s to %s: %s', srcBlock, dstBlock, ME.message);
                %     %     end
                % 
                %     if comtype(i) == 'Tank' 
                %         % miss fuel cell power link
                %         connectionsinstance = comconnectionsinstance{i};
                %         for k = 1:length(connectionsinstance)
                %             if comtype(connectionsinstance(k))=='Fuel cell'
                %                srcBlock = ['Component_' num2str(connectionsinstance(k))]; 
                %                dstBlock = ['Component_' num2str(i)];
                %                try
                %                    add_line(modelName, [srcBlock '/' num2str(1)], [dstBlock '/' num2str(1)], 'autorouting', 'smart');
                %                    add_line(modelName, ['DC' '/' num2str(3)], [dstBlock '/' num2str(2)], 'autorouting', 'smart');
                %                catch ME
                %                    warning('Could not add line from %s to %s: %s', srcBlock, dstBlock, ME.message);
                %                end
                %             end
                %         end
                % 
                %     end
                % 
                %     if comtype(i) == 'Battery'
                %         % connect motor directly
                %         connectionsinstance = comconnectionsinstance{i};
                %         for k = 1:length(connectionsinstance)
                %             if comtype(connectionsinstance(k))=='Electric motor 1'
                %                srcBlock = ['Component_' num2str(connectionsinstance(k))]; 
                %                dstBlock = ['Component_' num2str(i)];
                %                try
                %                    add_line(modelName, [srcBlock '/' num2str(1)], [dstBlock '/' num2str(1)], 'autorouting', 'smart');
                %                    add_line(modelName, ['DC' '/' num2str(3)], [dstBlock '/' num2str(2)], 'autorouting', 'smart');
                %                catch ME
                %                    warning('Could not add line from %s to %s: %s', srcBlock, dstBlock, ME.message);
                %                end
                %             end
                %         end
                % 
                % 
                % 
                %         % connect EPL
                %     %     connectionsinstance = comconnectionsinstance{i};
                %     %     for k = 1:length(connectionsinstance)
                %     %         if comtype(connectionsinstance(k))=='Electric Power Link'
                %     %             srcBlock = ['Electric_Power_Link_' num2str(connectionsinstance(k))]; 
                %     %             dstBlock = ['Component_' num2str(i)];
                %     % 
                %     %             try
                %     %                 add_line(modelName, [srcBlock '/' num2str(1)], [dstBlock '/' num2str(1)], 'autorouting', 'smart');
                %     %                 add_line(modelName, ['DC' '/' num2str(3)], [dstBlock '/' num2str(2)], 'autorouting', 'smart');
                %     %             catch ME
                %     %                 warning('Could not add line from %s to %s: %s', srcBlock, dstBlock, ME.message);
                %     %             end
                %     % 
                %     %         end
                %     %     end
                %     end

                case "Controller"
                    if comtype(i) == 'Electric Power Link'
                        
                        % Fixed connection
                        srcBlock = ['E_VectorConcatenate_' num2str(i)];
                        dstBlock = ['Electric_Power_Link_' num2str(i)];

                        try
                            add_line(modelName, [srcBlock '/' num2str(1)], [dstBlock '/' num2str(1)], 'autorouting', 'smart');
                        catch ME
                            warning('Could not add line from %s to %s: %s', srcBlock, dstBlock, ME.message);
                        end
                        
                        % find all power in (motors)
                        connectionsinstance = comconnectionsinstance{i};
                        for k = 1:length(connectionsinstance)
                            if comtype(connectionsinstance(k))=='Electric motor 1'
                               power_in = power_in + 1;
                               srcBlock = ['Component_' num2str(connectionsinstance(k))]; 
                               dstBlock = ['E_VectorConcatenate_' num2str(i)];
                               try
                                    add_line(modelName, [srcBlock '/' num2str(1)], [dstBlock '/' num2str(power_in)], 'autorouting', 'smart');
                               catch ME
                                    warning('Could not add line from %s to %s: %s', srcBlock, dstBlock, ME.message);
                               end

                            elseif comtype(connectionsinstance(k)) == 'Electric generator' || comtype(connectionsinstance(k)) == 'Fuel cell'
                                 power_out = power_out + 1;
                                 srcBlock = ['Electric_Power_Link_' num2str(i)]; 
                                 dstBlock = ['Component_' num2str(connectionsinstance(k))];
                                 try
                                      add_line(modelName, [srcBlock '/' num2str(power_out)], [dstBlock '/' num2str(1)], 'autorouting', 'smart');
                                 catch ME
                                      warning('Could not add line from %s to %s: %s', srcBlock, dstBlock, ME.message);
                                 end
                            elseif comtype(connectionsinstance(k)) == 'Battery'

                                power_out = power_out + 1;
                                srcBlock = ['Electric_Power_Link_' num2str(i)]; 
                                dstBlock = ['Component_' num2str(connectionsinstance(k))];
                                try
                                     add_line(modelName, [srcBlock '/' num2str(power_out)], [dstBlock '/' num2str(1)], 'autorouting', 'smart');
                                     add_line(modelName, ['DC' '/' num2str(3)], [dstBlock '/' num2str(2)], 'autorouting', 'smart');
                                catch ME
                                     warning('Could not add line from %s to %s: %s', srcBlock, dstBlock, ME.message);
                                end
                            end
                        end
                    end
            end
        end
    end
end