% ==============
% TU/e QSS Toolbox
% Driving cycle block initialization script
% ==============

%% Global variables
global h            % Stepsize [s] for all other blocks
global N_sim        % Number of computational steps required to complete the simulation of the cycle [-] for all other blocks
    
%% File names of the cycles
cyclestring = {'WLTP_Class1'
   'WLTP_Class2'
   'WLTP_Class3'
   'ECE_MAN'			
   'ECE_R15%2f04'
   'EUDC'
   'City'
   'NEDC'
   'VECTO_Delivery'
   'VECTO_LongHaul'
   'VECTO_Municipal'
   '10_15_MODE'
   '10_MODE'
   '10_MODE_3'
   '11_MODE'
   '11_MODE_4'
   '15_MODE'
   'CITY_I'
   'CITY_II'
   'FTP_75'
   'FTP_HIGHWAY'
   'topspeed'
   'gradient'};

%% Load data
if bool_combine
    data = load(char(cyclestring(cyclenr)));
    cycle_name = char(cyclestring(cyclenr));
    T_z = data.T_z;
    V_z = data.V_z;
    D_z = atan(data.D_z./100);
    G_z = data.G_z;
    clear data
    data = load(char(cyclestring(cyclenr2)));
    cycle_name = strcat(cycle_name,'+',char(cyclestring(cyclenr2)));
    data.T_z = data.T_z + T_z(end);
    T_z = cat(1, T_z, data.T_z);
    V_z = cat(1, V_z, data.V_z);
    D_z = cat(1, D_z, atan(data.D_z./100));
    G_z = cat(1, G_z, data.G_z);
    clear data
else
    if cyclenr == 24
        v_goal = (0:0.1:(gradient_speed/3.6))';
        t_goal = (1:(length(v_goal)))';
        g_goal = (zeros(size(v_goal)));
        d_goal = (zeros(size(v_goal)));

        data = load(char(cyclestring(23)));
        cycle_name = ['gradient at ',num2str(gradient_speed),'km/h'];
        data.V_z = cat(1, v_goal, ones(size(data.T_z))*gradient_speed/3.6);
        data.T_z = cat(1, t_goal, (data.T_z + t_goal(end)));
        data.G_z = cat(1, g_goal, data.G_z);
        data.D_z = cat(1, d_goal, data.D_z);
    else
        data = load(char(cyclestring(cyclenr)));
        cycle_name = char(cyclestring(cyclenr));

    end
    T_z = data.T_z;
    V_z = data.V_z;
    D_z = atan(data.D_z./100);
    G_z = data.G_z;
    clear data

end

cycle.speed = [];
cycle.alpha = [];
for i = 1:cyclerep
    cycle.speed = [cycle.speed V_z];
    cycle.alpha = [cycle.alpha D_z];
end
cycle.speed = cycle.speed(:);
stoptime = length(cycle.speed);
cycle.speed = timeseries(cycle.speed);
cycle.alpha = cycle.alpha(:);
cycle.alpha = timeseries(cycle.alpha);
if cyclerep > 1
    cycle_name = strcat(num2str(cyclerep),'x',cycle_name);
end

%% Prepare data (here: to plot a nice picture onto the block)
T_zplot = T_z;
V_zplot = V_z;

for nnn = length(T_zplot):round(length(T_zplot)*1.25)
   T_zplot(nnn+1) = T_zplot(nnn) + 1;
   V_zplot(nnn+1) = 0;
end

%% Store specific data
if autostop == 1
    set_param(char(bdroot(gcb)),'StopTime',num2str(stoptime));
end     

cl_par = struct('stepsize', stepsize, 'stoptime', stoptime);
set_param(gcb, 'UserData', cl_par);

%% Simulation parameters
h = stepsize; 
N_sim = stoptime;
    
