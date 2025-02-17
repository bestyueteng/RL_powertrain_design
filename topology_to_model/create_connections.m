function create_connections(comclass, comtype, componentStruct, comconnectionsinstance, modelName, DSM, comconnectionstype)
    
    add_line(modelName, 'DC/1', 'VB/1', 'autorouting', 'smart');
    add_line(modelName, 'DC/2', 'VB/2', 'autorouting', 'smart');

    power_in = 0;
    power_out = 0;
    epl_idx = 0;
    mgb_idx = 0;
    TS_out = 0;
    TC_out = 0;
    for i = 1:length(comclass)
        if ~all(DSM(i, :) == 0)
            switch comclass(i)
                case 'VehicleBody'
                    % check and find if connected to a gearbox
                    connectionsinstance = comconnectionsinstance{i};
                    for k = 1:length(connectionsinstance)
                        if comclass(connectionsinstance(k))=='GearSystems'
                            srcBlock = 'VB';
                            dstBlock = findBlock(componentStruct, connectionsinstance(k));

                            % get the number of in ports for multi-gearbox
                            Ports = get_param([modelName '/' dstBlock], 'Ports');
                            numinPorts = Ports(1);
                            try
                                add_line(modelName, [srcBlock '/' num2str(1)], [dstBlock '/' num2str(numinPorts-2)], 'autorouting', 'smart');
                                add_line(modelName, [srcBlock '/' num2str(2)], [dstBlock '/' num2str(numinPorts-1)], 'autorouting', 'smart');
                                add_line(modelName, [srcBlock '/' num2str(3)], [dstBlock '/' num2str(numinPorts)], 'autorouting', 'smart');
                            catch ME
                                warning('Could not add line from %s to %s: %s', srcBlock, dstBlock, ME.message);
                            end
                        end
                    end
                    
                case 'GearSystems'
                   % check and find if connected to a Energy Converter
                   connectionsinstance = comconnectionsinstance{i};
                   for k = 1:length(connectionsinstance)
                       if comclass(connectionsinstance(k))=='EnergyConverters'
                           srcBlock = findBlock(componentStruct, i);
                           dstBlock = findBlock(componentStruct, connectionsinstance(k));
                           try
                                add_line(modelName, [srcBlock '/' num2str(1)], [dstBlock '/' num2str(1)], 'autorouting', 'smart');
                                add_line(modelName, [srcBlock '/' num2str(2)], [dstBlock '/' num2str(2)], 'autorouting', 'smart');
                                add_line(modelName, [srcBlock '/' num2str(3)], [dstBlock '/' num2str(3)], 'autorouting', 'smart');
                           catch ME
                                warning('Could not add line from %s to %s: %s', srcBlock, dstBlock, ME.message);
                           end

                       % For final drive, it can connect to other GB
                       elseif comclass(connectionsinstance(k))=='GearSystems' && comtype(i) == 'Final Drive'
                            srcBlock = findBlock(componentStruct, i);
                            dstBlock = findBlock(componentStruct, connectionsinstance(k));

                            % get the number of in ports for multi-gearbox
                            Ports = get_param([modelName '/' dstBlock], 'Ports');
                            numinPorts = Ports(1);
                            try
                                add_line(modelName, [srcBlock '/' num2str(1)], [dstBlock '/' num2str(numinPorts-2)], 'autorouting', 'smart');
                                add_line(modelName, [srcBlock '/' num2str(2)], [dstBlock '/' num2str(numinPorts-1)], 'autorouting', 'smart');
                                add_line(modelName, [srcBlock '/' num2str(3)], [dstBlock '/' num2str(numinPorts)], 'autorouting', 'smart');
                            catch ME
                                warning('Could not add line from %s to %s: %s', srcBlock, dstBlock, ME.message);
                            end
                       end
                   end
                    
                   % Add controller for MGB
                   if comtype(i) == 'Multispeed gearbox'
                        mgb_idx = mgb_idx + 1;
                        dstBlock = findBlock(componentStruct, i);
                        % find its connections
                        connectionsinstance = comconnectionsinstance{i};
                        for k = 1:length(connectionsinstance)
                            if comclass(connectionsinstance(k))=='GearSystems' 
                                srcBlock = findBlock(componentStruct, connectionsinstance(k));
                                try
                                    add_line(modelName, ['MGB_controller_' num2str(mgb_idx) '/' num2str(1)], [dstBlock '/' num2str(1)], 'autorouting', 'smart');
                                    add_line(modelName, [srcBlock '/' num2str(1)], ['MGB_controller_' num2str(mgb_idx) '/' num2str(1)], 'autorouting', 'smart');
                                catch ME
                                    warning('Could not add line from %s to %s: %s', srcBlock, dstBlock, ME.message);
                                end
                            elseif comclass(connectionsinstance(k)) == 'VehicleBody'
                                srcBlock = 'VB';
                                try
                                    add_line(modelName, ['MGB_controller_' num2str(mgb_idx) '/' num2str(1)], [dstBlock '/' num2str(1)], 'autorouting', 'smart');
                                    add_line(modelName, [srcBlock '/' num2str(1)], ['MGB_controller_' num2str(mgb_idx) '/' num2str(1)], 'autorouting', 'smart');
                                catch ME
                                    warning('Could not add line from %s to %s: %s', srcBlock, dstBlock, ME.message);
                                end
                            elseif comtype(connectionsinstance(k)) == 'Torque Coupler' 
                                ts_flag = 0;
                                for q = 1:length(connectionsinstance)
                                    if comtype(connectionsinstance(q)) == 'Torque Split' || comtype(connectionsinstance(q)) == 'Final Drive'
                                        ts_flag = 1;
                                    end
                                end

                                if ts_flag ==0
                                    srcBlock = findBlock(componentStruct, connectionsinstance(k));
                                    try
                                        add_line(modelName, ['MGB_controller_' num2str(mgb_idx) '/' num2str(1)], [dstBlock '/' num2str(1)], 'autorouting', 'smart');
                                        add_line(modelName, [srcBlock '/' num2str(1)], ['MGB_controller_' num2str(mgb_idx) '/' num2str(1)], 'autorouting', 'smart');
                                    catch ME
                                        warning('Could not add line from %s to %s: %s', srcBlock, dstBlock, ME.message);
                                    end
                                end

                            elseif comtype(connectionsinstance(k)) == 'Torque Split'
                                srcBlock = findBlock(componentStruct, connectionsinstance(k));
                                try
                                    add_line(modelName, ['MGB_controller_' num2str(mgb_idx) '/' num2str(1)], [dstBlock '/' num2str(1)], 'autorouting', 'smart');
                                    add_line(modelName, [srcBlock '/' num2str(1)], ['MGB_controller_' num2str(mgb_idx) '/' num2str(1)], 'autorouting', 'smart');
                                catch ME
                                    warning('Could not add line from %s to %s: %s', srcBlock, dstBlock, ME.message);
                                end
                            end
                        end
                   end

                case 'EnergyConverters'
                    connectionsinstance = comconnectionsinstance{i};
                    for k = 1:length(connectionsinstance)
                       if comclass(connectionsinstance(k))=='EnergyStorage'
                           srcBlock = findBlock(componentStruct, i);
                           dstBlock = findBlock(componentStruct, connectionsinstance(k));
                           try
                                add_line(modelName, [srcBlock '/' num2str(1)], [dstBlock '/' num2str(1)], 'autorouting', 'smart');
                                add_line(modelName, ['DC' '/' num2str(3)], [dstBlock '/' num2str(2)], 'autorouting', 'smart');
                           catch ME
                                warning('Could not add line from %s to %s: %s', srcBlock, dstBlock, ME.message);
                           end
                       end
                    end

                case "Controller"
                    if comtype(i) == 'Electric Power Link'
                        epl_idx = epl_idx+1;
                        % Fixed connection
                        dstBlock = findBlock(componentStruct, i);
                        srcBlock = ['E_VectorConcatenate_' num2str(epl_idx)];
                        

                        try
                            add_line(modelName, [srcBlock '/' num2str(1)], [dstBlock '/' num2str(1)], 'autorouting', 'smart');
                        catch ME
                            warning('Could not add line from %s to %s: %s', srcBlock, dstBlock, ME.message);
                        end
                        
                        % find all power in & out
                        connectionsinstance = comconnectionsinstance{i};
                        for k = 1:length(connectionsinstance)
                            if comtype(connectionsinstance(k))=='Electric motor 1'
                               power_in = power_in + 1;
                               srcBlock = findBlock(componentStruct, connectionsinstance(k)); 
                               dstBlock = ['E_VectorConcatenate_' num2str(epl_idx)];
                               try
                                    add_line(modelName, [srcBlock '/' num2str(1)], [dstBlock '/' num2str(power_in)], 'autorouting', 'smart');
                               catch ME
                                    warning('Could not add line from %s to %s: %s', srcBlock, dstBlock, ME.message);
                               end

                            elseif comtype(connectionsinstance(k)) == 'Electric generator' || comtype(connectionsinstance(k)) == 'Fuel cell'
                                 power_out = power_out + 1;
                                 srcBlock = findBlock(componentStruct, i);
                                 dstBlock = findBlock(componentStruct, connectionsinstance(k));
                                 try
                                      add_line(modelName, [srcBlock '/' num2str(power_out)], [dstBlock '/' num2str(1)], 'autorouting', 'smart');
                                 catch ME
                                      warning('Could not add line from %s to %s: %s', srcBlock, dstBlock, ME.message);
                                 end
                            elseif comtype(connectionsinstance(k)) == 'Battery'

                                power_out = power_out + 1;
                                srcBlock = findBlock(componentStruct, i);
                                dstBlock = findBlock(componentStruct, connectionsinstance(k));
                                try
                                     add_line(modelName, [srcBlock '/' num2str(power_out)], [dstBlock '/' num2str(1)], 'autorouting', 'smart');
                                     add_line(modelName, ['DC' '/' num2str(3)], [dstBlock '/' num2str(2)], 'autorouting', 'smart');
                                catch ME
                                     warning('Could not add line from %s to %s: %s', srcBlock, dstBlock, ME.message);
                                end
                            end
                        end
                    elseif comtype(i) == 'Torque Split'
                        
                        
                        % TS only for distributed drive
                        
                        % find all torque in
                        connectionsinstance = comconnectionsinstance{i};
                        % TS -> VB
                        
                        for k = 1:length(connectionsinstance)
                            if comclass(connectionsinstance(k)) == 'VehicleBody'
                                srcBlock = 'VB';
                                dstBlock = findBlock(componentStruct, i);
                                try
                                    add_line(modelName, [srcBlock '/' num2str(1)], [dstBlock '/' num2str(1)], 'autorouting', 'smart');
                                    add_line(modelName, [srcBlock '/' num2str(2)], [dstBlock '/' num2str(2)], 'autorouting', 'smart');
                                    add_line(modelName, [srcBlock '/' num2str(3)], [dstBlock '/' num2str(3)], 'autorouting', 'smart');
                               catch ME
                                    warning('Could not add line from %s to %s: %s', srcBlock, dstBlock, ME.message);
                                end
                            end
                        end

                        % find all torque out 
                        
                        connectionsinstance = comconnectionsinstance{i};
                        for k = 1:length(connectionsinstance)
                            if comclass(connectionsinstance(k)) == 'GearSystems'
                                TS_out = TS_out + 1;
                                srcBlock = findBlock(componentStruct, i);
                                dstBlock = findBlock(componentStruct, connectionsinstance(k));

                                % get the number of in ports for multi-gearbox
                                Ports = get_param([modelName '/' dstBlock], 'Ports');
                                numinPorts = Ports(1);
                                try
                                    add_line(modelName, [srcBlock '/' num2str(1)], [dstBlock '/' num2str(numinPorts-2)], 'autorouting', 'smart');
                                    add_line(modelName, [srcBlock '/' num2str(2)], [dstBlock '/' num2str(numinPorts-1)], 'autorouting', 'smart');
                                    add_line(modelName, [srcBlock '/' num2str(TS_out+2)], [dstBlock '/' num2str(numinPorts)], 'autorouting', 'smart');
                               catch ME
                                    warning('Could not add line from %s to %s: %s', srcBlock, dstBlock, ME.message);
                                end
                                
                            elseif comclass(connectionsinstance(k)) == 'EnergyConverters'
                                TS_out = TS_out + 1;
                                srcBlock = findBlock(componentStruct, i);
                                dstBlock = findBlock(componentStruct, connectionsinstance(k));
                                try
                                    add_line(modelName, [srcBlock '/' num2str(1)], [dstBlock '/' num2str(1)], 'autorouting', 'smart');
                                    add_line(modelName, [srcBlock '/' num2str(2)], [dstBlock '/' num2str(2)], 'autorouting', 'smart');
                                    add_line(modelName, [srcBlock '/' num2str(TS_out+2)], [dstBlock '/' num2str(3)], 'autorouting', 'smart');
                               catch ME
                                    warning('Could not add line from %s to %s: %s', srcBlock, dstBlock, ME.message);
                                end
                            % for TS
                            elseif comtype(connectionsinstance(k)) == 'Torque Coupler'
                                TS_out = TS_out + 1;
                                srcBlock = findBlock(componentStruct, i);
                                dstBlock = findBlock(componentStruct, connectionsinstance(k));
                                try
                                    add_line(modelName, [srcBlock '/' num2str(1)], [dstBlock '/' num2str(1)], 'autorouting', 'smart');
                                    add_line(modelName, [srcBlock '/' num2str(2)], [dstBlock '/' num2str(2)], 'autorouting', 'smart');
                                    add_line(modelName, [srcBlock '/' num2str(TS_out+2)], [dstBlock '/' num2str(3)], 'autorouting', 'smart');
                               catch ME
                                    warning('Could not add line from %s to %s: %s', srcBlock, dstBlock, ME.message);
                                end
                            end
                        end

                    elseif comtype(i) == 'Torque Coupler'
                        connectionsinstance = comconnectionsinstance{i};
                        % TC -> FD
                        for k = 1:length(connectionsinstance)
                            if comtype(connectionsinstance(k)) == 'Final Drive'
                                srcBlock = findBlock(componentStruct, connectionsinstance(k));
                                dstBlock = findBlock(componentStruct, i);
                                try
                                    add_line(modelName, [srcBlock '/' num2str(1)], [dstBlock '/' num2str(1)], 'autorouting', 'smart');
                                    add_line(modelName, [srcBlock '/' num2str(2)], [dstBlock '/' num2str(2)], 'autorouting', 'smart');
                                    add_line(modelName, [srcBlock '/' num2str(3)], [dstBlock '/' num2str(3)], 'autorouting', 'smart');
                               catch ME
                                    warning('Could not add line from %s to %s: %s', srcBlock, dstBlock, ME.message);
                                end

                            % If TSC/FD -> GB  , then GB -> TC
                            elseif comclass(connectionsinstance(k)) == 'GearSystems'
                                subinstance = comconnectionsinstance{connectionsinstance(k)};
                                for q = 1:length(subinstance)
                                    if comtype(subinstance(q))=='Torque Split' || comtype(subinstance(q)) == 'Final Drive'
                                        srcBlock = findBlock(componentStruct, connectionsinstance(k));
                                        dstBlock = findBlock(componentStruct, i);
                                        try
                                            add_line(modelName, [srcBlock '/' num2str(1)], [dstBlock '/' num2str(1)], 'autorouting', 'smart');
                                            add_line(modelName, [srcBlock '/' num2str(2)], [dstBlock '/' num2str(2)], 'autorouting', 'smart');
                                            add_line(modelName, [srcBlock '/' num2str(3)], [dstBlock '/' num2str(3)], 'autorouting', 'smart');
                                        catch ME
                                            warning('Could not add line from %s to %s: %s', srcBlock, dstBlock, ME.message);
                                        end
                                        break;
                                    end
                                end
                            end
                            
                        end
                        

                        % find all torque out 
                        
                        connectionsinstance = comconnectionsinstance{i};
                        for k = 1:length(connectionsinstance)
                            if comclass(connectionsinstance(k)) == 'GearSystems' && comtype(connectionsinstance(k)) ~= 'Final Drive'
                                ts_flag = 0;
                                subinstance = comconnectionsinstance{connectionsinstance(k)};
                                for q = 1:length(subinstance)
                                    if comtype(subinstance(q)) == 'Torque Split' || comtype(subinstance(q)) == 'Final Drive'
                                        ts_flag = 1;
                                        break;
                                    end
                                end

                                if ts_flag ==0
                                    TC_out = TC_out + 1;
                                    srcBlock = findBlock(componentStruct, i);
                                    dstBlock = findBlock(componentStruct, connectionsinstance(k));
    
                                    % get the number of in ports for multi-gearbox
                                    Ports = get_param([modelName '/' dstBlock], 'Ports');
                                    numinPorts = Ports(1);
                                    try
                                        add_line(modelName, [srcBlock '/' num2str(1)], [dstBlock '/' num2str(numinPorts-2)], 'autorouting', 'smart');
                                        add_line(modelName, [srcBlock '/' num2str(2)], [dstBlock '/' num2str(numinPorts-1)], 'autorouting', 'smart');
                                        add_line(modelName, [srcBlock '/' num2str(TC_out+2)], [dstBlock '/' num2str(numinPorts)], 'autorouting', 'smart');
                                   catch ME
                                        warning('Could not add line from %s to %s: %s', srcBlock, dstBlock, ME.message);
                                    end
                                end
                                
                            elseif comclass(connectionsinstance(k)) == 'EnergyConverters'
                                TC_out = TC_out + 1;
                                srcBlock = findBlock(componentStruct, i);
                                dstBlock = findBlock(componentStruct, connectionsinstance(k));
                                try
                                    add_line(modelName, [srcBlock '/' num2str(1)], [dstBlock '/' num2str(1)], 'autorouting', 'smart');
                                    add_line(modelName, [srcBlock '/' num2str(2)], [dstBlock '/' num2str(2)], 'autorouting', 'smart');
                                    add_line(modelName, [srcBlock '/' num2str(TC_out+2)], [dstBlock '/' num2str(3)], 'autorouting', 'smart');
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