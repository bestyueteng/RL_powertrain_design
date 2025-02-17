function results = RunExamples(veh_type, pt_type)
%RunExamples(veh_type, pt_type) Runs an example vehicle 
% Options for veh_type: {'compact','midsize','SUV'}
% Options for pt_type: {'ICEV', 'HEV', 'BEV'}

%% Global variables
global N_sim

%% Model name
sys_name = pt_type;
load_system(sys_name);

%% Vehicle parameters
switch veh_type
    case 'compact'          % Toyota Yaris - Origianlly HEV/ICEV
        m0     = 850;       % Nominal vehicle mass [kg]
        dw     = 0.6996;    % wheel diameter [m]
        cd     = 0.29;      % air drag coefficient [-]
        Af     = 2.38;      % frontal area [m2]
        cr     = 0.0174;    % rolling resistance coefficient [-]
        f_r    = 0.6;       % reg. brake fraction, FWD [-]
    case 'midsize'          % Honda Accord - Origianlly HEV/ICEV
        m0     = 1250;      % Nominal vehicle mass [kg]
        dw     = 0.7189;    % wheel diameter [m]
        cd     = 0.23;      % air drag coefficient [-]
        Af     = 2.43;      % frontal area [m2]
        cr     = 0.008;     % rolling resistance coefficient [-]
        f_r    = 0.4;       % reg. brake fraction, RWD [-]
    case 'SUV'              % Volvo XC90 - Originally ICEV
        m0     = 2268-160;  % Nominal vehicle mass [kg]
        dw     = 0.7555;    % wheel diameter [m]
        cd     = 0.2409;    % air drag coefficient [-]
        Af     = 3.5662;    % frontal area [m2]
        cr     = 0.0023;    % rolling resistance coefficient [-]
        f_r    = 1;         % reg. brake fraction, AWD [-]
end
J_rot  = 5;                 % Rotating mass [%]

%% Gear box
e_gb    = 0.98;   % internal efficiency [-]
Ploss   = 300;    % stationary losses [W]
wem_min = 1;      % Minimum wheel speed beyond which losses are generated [rad/s]
switch pt_type
    case 'BEV'
        gamma = 3;                              % gear ratio [-]
        mtr   = 17;                             % 1spd transmission mass [kg]
    case 'HEV'
        gamma = [3.79, 2.12, 1.36, 1.03, 0.84]; % gear ratios [-]
        gamma_fd = 4;                           % final drive ratio [-]
        mtr    = 41;                            % 5spd transmission mass [kg]
    case 'ICEV'
        gamma = [3.79, 2.12, 1.36, 1.03, 0.84]; % gear ratios [-]
        gamma_fd = 4;                           % final drive ratio [-]
        mtr    = 41;                            % 5spd transmission mass [kg]
end

%% Power sources
J_em = 0.1;       % rotating inertia [kgm2]
Paux_em = 100;    % auxilary power losses [W]

Paux_ice = 0;
Tcutoff_ice = 5; % engine torque at fuel cutoff [Nm]
Pcutoff_ice = 0; % power at fuel cutoff [W]

switch pt_type
    case 'BEV'
        alpha_ice  = 0; % engine scaling factor [-] 
        switch veh_type
            case 'compact'  % (params extrapolated from HEV)
                alpha_em   = 2.73; % motor scaling factor [-]
                Np         = 55;   % number of cells in parallel
                Ns         = 52;   % number of cells in series
            case 'midsize'  % (params extrapolated from HEV)
                alpha_em   = 4.33; 
                Np         = 55;
                Ns         = 76;
            case 'SUV'      % (params extrapolated from ICEV)
                alpha_em   = 5.63; 
                Np         = 55;
                Ns         = 100;
        end
        set_param([sys_name, '/Battery'],'bt_Np',num2str(Np));
        set_param([sys_name, '/Battery'],'bt_Ns',num2str(Ns));
    case 'HEV'
        switch veh_type
            case 'compact'
                alpha_em   = 1.8; % motor scaling factor [-] - 60 kW
                alpha_ice  = .66; % engine scaling factor [-] - 67 kW
                Np         = 1;
                Ns         = 52;
            case 'midsize'
                alpha_em   = 4.06; % motor scaling factor [-]  - 134 kW
                alpha_ice  = 1.03; % engine scaling factor [-] - 106 kW
                Np         = 1;
                Ns         = 76;
            case 'SUV'      % (params extrapolated from ICEV)
                alpha_em   = 4.3; % motor scaling factor [-]
                alpha_ice  = 1.36; % engine scaling factor [-]
                Np         = 1;
                Ns         = 100;
        end
        set_param([sys_name, '/Battery'],'bt_Np',num2str(Np));
        set_param([sys_name, '/Battery'],'bt_Ns',num2str(Ns));
    case 'ICEV'
        alpha_em   = 0; % motor scaling factor [-]
        Np         = 0;
        Ns         = 0;
        switch veh_type
            case 'compact'
                alpha_ice  = 0.88; % engine scaling factor [-] - 90 kW
            case 'midsize'
                alpha_ice  = 1.4;  % engine scaling factor [-] - 143 kW
            case 'SUV'
                alpha_ice  = 1.82; % engine scaling factor [-] - 186 kW
        end
end

%% Battery
init_SoC  = 80;   % initial rel. state-of-charge [%]
SoC_low   = 10;   % lower limit for battery SoC
SoC_high  = 90;   % upper limit for battery SoC
E_density = 250;  % battery energy density [Wh/kg]
cell_cap  = 5.5;  % nominal cell capacity [Ah]
cell_vol  = 3.425;% nominal cell voltage [V]

%% Component and vehicle mass
P0_ice  = 102e3; % Unscaled engine power [W]
P0_em   = 33e3;  % Unscaled engine power [W]
Pmax_em = P0_em *alpha_em;
Pmax_ice = P0_ice*alpha_ice;

rho_em  = 1.4e3; % E-machine mass densith [kW/kg]
rho_ice = 1.5e3; % Engine mass densith [kW/kg]

mice   = Pmax_ice/rho_ice;                       % Engine mass [kg]
mem    = Pmax_em/rho_em;                         % E-machine mass [kg]
mb     = (Np*Ns*5.5*3.425)/E_density;            % battery mass [kg]
mcargo = 0;                                      % cargo mass [kg]
mv     = (m0 + mb + mice +  mem + mtr + mcargo); % vehicle mass [kg]

%% HEV parameters
switch veh_type
    case 'compact'
        Tsplit_em = .2;     % Amount of torque going to EM
    case 'midsize'
        Tsplit_em = .3;     % Amount of torque going to EM
    case 'SUV'
        Tsplit_em = .3;     % Amount of torque going to EM
end
w_shift = 500;

%% Simulation
options = simset('SrcWorkspace','current');
results = sim(sys_name,[],options);

end