function scaling (modelName, comtype, DSM, componentStruct)
    mgb_idx = 0;
    for i = 1:length(comtype)
        if ~all(DSM(i, :) == 0)
            switch comtype(i)
                % case 'Electric motor 1'
                    
                    % set_param([modelName, '/Component_', num2str(i)],'scale_EM',num2str(2));
                    % num_em = num_em + 1;
                    % if num_em == 1
                    %     set_param([modelName, '/Electric motor 1'],'scale_EM',num2str(model_scale{i}));
                    % else
                    %     set_param([modelName, '/Component_', num2str(i)],'scale_EM',num2str(model_scale{i}));
                    % end
                % case 'Fuel cell'
                %     set_param([modelName, '/Component_', num2str(i)],'N_FC',num2str(1));
                % case 'Battery'
                    % set_param([modelName, '/Component_', num2str(i)],'init_SoC',num2str(90));
                    % set_param([modelName, '/Component_', num2str(i)],'bt_Np',num2str(30));
                    % set_param([modelName, '/Component_', num2str(i)],'bt_Ns',num2str(88));

                    
                case 'Multispeed gearbox'
                    blockname = findBlock(componentStruct, i);
                    mgb_idx = mgb_idx + 1;
                    set_param([modelName, '/' blockname],'nr_gears',num2str(3));
                    set_param([modelName '/MGB_controller_' num2str(mgb_idx)],'nr_gears',num2str(3));
            end
        end
    end
end