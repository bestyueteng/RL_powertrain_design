function V_result = control_optimization(x,comtype,DSM,modelName,componentStruct)

try
%% Define Global variables
global N_sim
global ta mv
global x_result
global y_result
global z_result

%% Model name
sys_name = 'VehiclePowertrain';
load_system(sys_name);
%% Set constrains
req      = {8, 200, 0.1993, [0.1,50]};    % acc time [s], max speed [km/h], gradability_ss [%], gradability_speed [%, km/h]
acc_req   = req{1};    % acceleration time requirement [s]
vmax_req = req{2}/3.6; % top speed requirement [m/s]
grad_ss_req     = req{3};     % Gradability [%]
gradspeed_req = req{4};
gradspeed_grad_req = gradspeed_req(1);
gradspeed_speed_req = gradspeed_req(2);

penalty_cost = 1e6;
%% Set actual optimization values
num_var = 0; % MGB SGB EM FC B
num_of_gears = 3; % Fixed nr_of_gear for MGB
mgb_idx = 0;
% check type one by one following default sequence
for i = 1:length(comtype)
    if ~all(DSM(i, :) == 0)
        if comtype(i) == 'Multispeed gearbox'
            mgb_idx = mgb_idx + 1;
            for j = 1:num_of_gears
                num_var = num_var + 1;
                blockname = findBlock(componentStruct, i);
                set_param([modelName, '/' blockname],['i_' num2str(j)],num2str(x(num_var)));
            end

            for j = 1:num_of_gears-1
                num_var = num_var + 1;
                set_param([modelName '/MGB_controller_' num2str(mgb_idx)],['w_shift' num2str(j+1)],num2str(x(num_var)));
            end
        end
    end
end
for i = 1:length(comtype)
    if ~all(DSM(i, :) == 0)
        if comtype(i) == 'Simple transmission'
            num_var = num_var + 1;
            blockname = findBlock(componentStruct, i);
            set_param([modelName,'/' blockname],'gear_ratio',num2str(x(num_var)));
        end
    end
end
for i = 1:length(comtype)
    if ~all(DSM(i, :) == 0)
        if comtype(i) == 'Electric motor 1'
            num_var = num_var + 1;
            blockname = findBlock(componentStruct, i);
            set_param([modelName, '/' blockname],'scale_EM',num2str(x(num_var)));
        end
    end
end
for i = 1:length(comtype)
    if ~all(DSM(i, :) == 0)
        if comtype(i) == 'Fuel cell'
            num_var = num_var + 1;
            blockname = findBlock(componentStruct, i);
            set_param([modelName, '/' blockname],'N_FC',num2str(1));
        end
    end
end

for i = 1:length(comtype)
    if ~all(DSM(i, :) == 0)
        if comtype(i) == 'Battery'
            num_var = num_var + 1;
            blockname = findBlock(componentStruct, i);
            set_param([modelName, '/' blockname],'init_SoC',num2str(x(num_var)));
            num_var = num_var + 1;
            set_param([modelName, '/' blockname],'bt_Np',num2str(x(num_var)));
            num_var = num_var + 1;
            set_param([modelName, '/' blockname],'bt_Ns',num2str(x(num_var)));
        end
    end
end

%% Build matrix of optimization parameter values
x_result = [x_result; x'];
y_result = [y_result; [mv]'];
z_result = [z_result; [ta]'];

%% Component and vehicle massload_system(sys_name);
mbase = 1611;
mtr = 0;
mfc = 0;
mb = 0;
mem = 0;
Cm = 0;
Cf = 0;
Cb = 0;

for i = 1:length(comtype)
    if ~all(DSM(i, :) == 0)
        blockname = findBlock(componentStruct, i);
        m_block_name = [modelName '/' blockname];
        switch comtype(i)
            case 'Simple transmission'
                mtr = mtr+55;
            case 'Final Drive'
                mtr = mtr+55;
            case 'Multispeed gearbox'
                mtr = mtr + 50 + 5*num_of_gears;
            case 'Electric motor 1'
                [motor_Data, scales] = get_motor_scale_type(m_block_name);
                [omegas_m, torques_m, efficiencies_m, P_EM_max_m] = get_motor_data(motor_Data, scales);
                mem = mem + P_EM_max_m{1} * str2double(scales(1)) / 1400;
                Cm = Cm + 16*str2double(scales(1))*P_EM_max_m{1} / 1000;
            case 'Electric generator'
                % didn't consider
            case 'Battery'
                E_density = 250;  % battery energy density [Wh/kg]
                Np = get_param([modelName, '/' blockname],'bt_Np');
                Np = str2double(Np);
                Ns = get_param([modelName, '/' blockname],'bt_Ns');
                Ns = str2double(Ns);
                mb = mb + (Np*Ns*5.5*3.425)/E_density; 
                Cb = Cb + 200*Np*Ns*5.5*3.425 / 1000;

            case 'Fuel cell'
                
                P_fc = 0.5; % fuel cell power dencity (kw/kg)

                A_fc = get_param(m_block_name, 'A_FC'); % [m^2] size of a cell
                A_fc = str2double(A_fc); % Convert string to double

                V_fc = 0.535; % [V] cell voltage
                Ad_fc = 6000; % [A/m^2] current density

                n_fc = get_param(m_block_name, 'N_FC');
                n_fc = str2double(n_fc);
                Pfcmax = V_fc*n_fc*Ad_fc*A_fc / 1000; %[kw]
               
                mfc = mfc+Pfcmax/P_fc;
                Cf = Cf + 60*Pfcmax/1000;
        end
    end
end
m_total = mbase + mfc + mem + mtr + mb;       % vehicle mass [kg]
set_param([modelName '/VB'], 'm_f', num2str(m_total));

%% Simulation

options = simset('SrcWorkspace','current');

% Run Top Speed
set_param([modelName '/DC'], 'cyclenr', 'Test:    Top speed');
results_topspeed = sim(sys_name,[],options);
assignin('base','results_topspeed',results_topspeed);
if results_topspeed.v < vmax_req
    V_result = penalty_cost;
    disp("top speed");
else

    % Run Gradebility ss
    set_param([modelName '/DC'], 'cyclenr', 'Test:    Gradient (standstill)');
    % Add slope line
    set_param([modelName '/VB'], 'slope_bool', 'on'); 
    try
        add_line(modelName, ['DC' '/' num2str(3)], ['VB' '/' num2str(3)], 'autorouting', 'smart'); 
    catch ME
    end
    results_gradient_ss = sim(sys_name,[],options);
    assignin('base','results_gradient_ss',results_gradient_ss);
    results_slope = results_gradient_ss.slope;
    if  results_slope(end) < grad_ss_req
        V_result = penalty_cost;
        disp("Gradebility ss");
    else

        % Run Gradebility specify speed 
        set_param([modelName '/DC'], 'cyclenr', 'Test:    Gradient (specify speed)');
        set_param([modelName '/DC'], 'gradient_speed', num2str(gradspeed_speed_req));
        
        % Add slope line
        set_param([modelName '/VB'], 'slope_bool', 'on'); 
        try
            add_line(modelName, ['DC' '/' num2str(3)], ['VB' '/' num2str(3)], 'autorouting', 'smart'); 
        catch ME
        end
        results_gradient_speed = sim(sys_name,[],options);
        assignin('base','results_gradient_speed',results_gradient_speed);
        results_slope = results_gradient_speed.slope;
        if results_slope(end) < gradspeed_grad_req % Currently is 0.1993 as it's the maximum in the DC dataset
           V_result = penalty_cost;
           disp("Gradebility specify speed ");

        else
           
            % Check Acceleration
                

                % Run WLTP
                delete_line(modelName, ['DC' '/' num2str(3)], ['VB' '/' num2str(3)]); 
                set_param([modelName '/DC'], 'cyclenr', 'Europe: WLTP Class 3');
                set_param([modelName '/DC'], 'slope_bool_cycle', 'off');
                set_param([modelName '/VB'], 'slope_bool', 'off'); 
                results_wltp = sim(sys_name,[],options);
                assignin('base','results_wltp',results_wltp);
    
                % Object function
                V_liter = 0;
                cons_BT = 0;
                bat_idx = 0;
                fc_idx = 0;
    
                Cvb = 17000;              % Base vehicle cost
                Cv = Cm +Cb + Cf +Cvb;        % Depreciation cost
    
                % Get sum fuel
                for i = 1:length(comtype)
                    if comtype(i) == 'Fuel Cell'
                        if ~all(DSM(i, :) == 0)
                            fc_idx = fc_idx + 1;
                            para_string = ['V_liter' num2str(fc_idx)];
                            V = results_wltp.(para_string);
                            V_liter = V_liter + V(end); % [L/100km]
                        end
                    end
                end
                
                % Get sum Bat
                for i = 1:length(comtype)
                    if comtype(i) == 'Battery'
                        if ~all(DSM(i, :) == 0)
                            bat_idx = bat_idx + 1;
                            para_string = ['cons_BT' num2str(bat_idx)];
                            bat = results_wltp.(para_string);
                            cons_BT = cons_BT + bat(end); % [L/100km]
                        end
                    end
                end
                
                dis = results_wltp.x; %[m]
                % Check whether cycle could be finished exactly in N_sim computational steps;
                % if cycle duration is less than N_sim, set fuel consumption to infinite
                if length(dis) ~= 1801
                     disp("time")
                     V_result = penalty_cost;
                
                else
                    % disp("good")
                    total_liter = V_liter * 23.25/100; %[L]
                    
                    rho_h2 = 0.083; %[g/L]
                    
                    total_mass = rho_h2 * total_liter / 1000; %[kg]
                    % H1 = 33.33; % [kWh/kg]
                    % ef = total_mass * H1; %[kwh]
                    
                    dy = 20000; % [km]
                    y = 5;
                    cf = 12.95; %[euro/kg]
                    dc = 23.25;
                    
                    Ce_h2 = cf*total_mass*dy*y/dc;
                
                    Es = cons_BT * 23.25/100; %[kwh]
                    ce = 0.5; %[euro/kwh]
                    Ce_bt = ce*Es*dy*y/dc;
                
                    C = 0.5*Cv+Ce_bt+Ce_h2;
                    V_result = C;
                    V_result
                end
            
        end
    end
end
catch ME
    V_result = 1e6;
end
end