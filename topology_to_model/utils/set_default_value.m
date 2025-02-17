function x_init = set_default_value(DSM,comtype,optimizing_method)
    num_of_gears_default_val = 3;
    scale_EM_default_val = 2;
    N_FC_default_val = 1;
    init_SoC_default_val = 90;
    gear_default_val = [5 3 1 1 1];
    speed_default_val = [100 300 500 500 1000];
    bt_Np_default_val = 40;
    bt_Ns_default_val = 68;
    switch optimizing_method
        case 'fminsearch'
            x_init = [];
            for i = 1:length(comtype)
                if ~all(DSM(i, :) == 0)
                    if comtype(i) == 'Multispeed gearbox'
                        num_of_gears_default = num_of_gears_default_val;
                        gear_default = gear_default_val(1:num_of_gears_default);
                        speed_default = speed_default_val(1:num_of_gears_default-1);
                        % x_init = [x_init num_of_gears_default gear_default speed_default];
                        x_init = [x_init gear_default speed_default];
                    end
                end
            end
            for i = 1:length(comtype)
                if ~all(DSM(i, :) == 0)
                    if comtype(i) == 'Simple transmission'
                        gear_default = 1;
                        x_init = [x_init gear_default];
                    end
                end
            end
            
            for i = 1:length(comtype)
                if ~all(DSM(i, :) == 0)
                    if comtype(i) == 'Electric motor 1'
                        scale_EM = scale_EM_default_val;
                        x_init = [x_init scale_EM];
                    end
                end
            end

            for i = 1:length(comtype)
                if ~all(DSM(i, :) == 0)
                    if comtype(i) == 'Fuel cell'
                        N_FC = N_FC_default_val;
                        x_init = [x_init N_FC];
                    end
                end
            end

            for i = 1:length(comtype)
                if ~all(DSM(i, :) == 0)
                    if comtype(i) == 'Battery'
                        init_SoC = init_SoC_default_val;
                        bt_Np = bt_Np_default_val;
                        bt_Ns = bt_Ns_default_val;
                        x_init = [x_init init_SoC bt_Np bt_Ns];
                    end
                end
            end

        case 'pso'
            lb = [];
            ub = [];
            x_init_lb = [];
            x_init_ub = [];
            for i = 1:length(comtype)
                if ~all(DSM(i, :) == 0)
                    if comtype(i) == 'Multispeed gearbox'
                        num_of_gears_default = num_of_gears_default_val;
                        lb_num_gears = num_of_gears_default;
                        ub_num_gears = num_of_gears_default + 2; % max 5 gears
                        
                        gear_default = gear_default_val(1:num_of_gears_default);

                        % Modify gear_default as per your conditions
                        lb_gear = gear_default - 5;  % Subtract 2 from each element
                        lb_gear(lb_gear < 1) = 1;  % Set elements less than 1 to 1
                        ub_gear = gear_default + 10;

                        speed_default = speed_default_val(1:num_of_gears_default-1);
                        lb_speed = speed_default - 50;
                        lb_speed(lb_speed <= 0) = 1;
                        ub_speed = speed_default + 50;

                        x_init_lb = [x_init_lb, lb_gear,lb_speed];
                        x_init_ub = [x_init_ub, ub_gear,ub_speed];

                    end
                end
            end
            for i = 1:length(comtype)
                if ~all(DSM(i, :) == 0)
                    if comtype(i) == 'Simple transmission'
                        gear_default = 1;
                        lb_gear = gear_default;
                        ub_gear = gear_default + 10;

                        x_init_lb = [x_init_lb,lb_gear];
                        x_init_ub = [x_init_ub,ub_gear];
                    end
                end
            end
            for i = 1:length(comtype)
                if ~all(DSM(i, :) == 0)
                    if comtype(i) == 'Electric motor 1'
                        scale_EM = scale_EM_default_val;
                        lb_scale = scale_EM - 2;
                        if lb_scale <= 1
                            lb_scale = 1;
                        end
                        ub_scale = scale_EM + 5;
                        x_init_lb = [x_init_lb,lb_scale];
                        x_init_ub = [x_init_ub,ub_scale];
                    end
                end
            end
            for i = 1:length(comtype)
                if ~all(DSM(i, :) == 0)
                    if comtype(i) == 'Fuel cell'
                        N_FC = N_FC_default_val;
                        lb_fc = N_FC;
                        ub_fc = N_FC + 50;
                        x_init_lb = [x_init_lb,lb_fc];
                        x_init_ub = [x_init_ub,ub_fc];
                    end
                end
            end
            for i = 1:length(comtype)
                if ~all(DSM(i, :) == 0)
                    if comtype(i) == 'Battery'
                        init_SoC = init_SoC_default_val;
                        bt_Np = bt_Np_default_val;
                        bt_Ns = bt_Ns_default_val;

                        lb_soc = init_SoC - 20;
                        ub_soc = init_SoC + 10;
                        lb_np = bt_Np - 10;
                        ub_np = bt_Np + 20;
                        lb_ns = bt_Ns - 10;
                        ub_ns = bt_Ns + 20;
                        
                        % Check if any values are less than 1 and set them to 1 if so
                        if lb_soc < 10
                            lb_soc = 10;
                        end
                        if ub_soc > 90
                            ub_soc = 90;
                        end
                        if lb_np < 1
                            lb_np = 1;
                        end
                        if ub_np < 1
                            ub_np = 1;
                        end
                        if lb_ns < 1
                            lb_ns = 1;
                        end
                        if ub_ns < 1
                            ub_ns = 1;
                        end
                        x_init_lb = [x_init_lb,lb_soc, lb_np, lb_ns];
                        x_init_ub = [x_init_ub,ub_soc, ub_np, ub_ns];
                    end
                end
            end

            x_init = {x_init_lb,x_init_ub};
        otherwise
            x_init = [];
            for i = 1:length(comtype)
                if ~all(DSM(i, :) == 0)
                    if comtype(i) == 'Multispeed gearbox'
                        num_of_gears_default = num_of_gears_default_val;
                        gear_default = gear_default_val(1:num_of_gears_default);
                        speed_default = speed_default_val(1:num_of_gears_default-1);
                        % x_init = [x_init num_of_gears_default gear_default speed_default];
                        x_init = [x_init gear_default speed_default];
                    end
                end
            end
            for i = 1:length(comtype)
                if ~all(DSM(i, :) == 0)
                    if comtype(i) == 'Simple transmission'
                        gear_default = 1;
                        x_init = [x_init gear_default];
                    end
                end
            end
            
            for i = 1:length(comtype)
                if ~all(DSM(i, :) == 0)
                    if comtype(i) == 'Electric motor 1'
                        scale_EM = scale_EM_default_val;
                        x_init = [x_init scale_EM];
                    end
                end
            end

            for i = 1:length(comtype)
                if ~all(DSM(i, :) == 0)
                    if comtype(i) == 'Fuel cell'
                        N_FC = N_FC_default_val;
                        x_init = [x_init N_FC];
                    end
                end
            end

            for i = 1:length(comtype)
                if ~all(DSM(i, :) == 0)
                    if comtype(i) == 'Battery'
                        init_SoC = init_SoC_default_val;
                        bt_Np = bt_Np_default_val;
                        bt_Ns = bt_Ns_default_val;
                        x_init = [x_init init_SoC bt_Np bt_Ns];
                    end
                end
            end
    end
end