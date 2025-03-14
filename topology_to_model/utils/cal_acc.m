function ta = cal_acc(sum_power, mv)

cr     = 0.017;     % rolling resistance coefficient [-]
cd     = 0.29;      % air drag coefficient [-]
Af     = 2.38;      % frontal area [m2]
dw     = 0.7;    % wheel diameter [m]

g = 9.81;        % Gravitation constant
rho = 1.18;      % Air density
kR = 1;       % weight distribution during acceleration
muP = 0.90;      % max. tyre-road adhesion coefficient
Ttmax = kR*muP*mv*g*dw/2;  % max. slip / tractive torque
Ptmax = sum_power*0.97;
vb = Ptmax./(Ttmax./(dw/2)); % base vehicle speed
vf = 100/3.6;                % final speed acceleration
ta = (mv*(vb^2+vf^2))./(2*(Ptmax - (2/3)*mv*g*cr*vf - (1/5)*rho*cd*Af*vf^3));

end