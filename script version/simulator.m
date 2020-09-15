% By D. Clifford 28/08/2020

% RATHER THAN USING ANY OF THE WELL ESTABLISHED OPTIMISATION METHODS TO
% TRIM THE AIRCRAFT I WONDERED IF IT COULD BE DONE QUICKLY WITH A BRUTE 
% FORCE APPROACH. IT TURNS OUT IT CAN (KIND OF), WHICH IS WHAT THIS
% MONSTROSITY OF A TRIM CALCULATOR IS.

% THE AERO DERIVATIVES AND A SIMPLE FLIGHT SIMULATOR USING A FOURTH ORDER
% RUNGE KUTTA TIME STEPPING METHOD ARE FOUND AFTER THE TRIM CALCULATIONS.

clear all

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% numerically determining the trimmed flight condition %%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% calling navion details
navion_details_six_DoF

%%%%%%%% first iteration - trim calculator %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
i = 1;

% initial angle of attack guess
alpha(i) = 0 * pi / 180;

% finding alpha_t at alpha to give zero aircraft moment
myfun = @three_DoF_aircraft_model;
fun = @(x) myfun(x, alpha(i));
alpha_t = fzero(fun, 0.0); % trimmed tailplane angle

% calculating net vertical force
[~, Fxb, Fzb] = three_DoF_aircraft_model(alpha_t, alpha);  

% total lift coefficient
CL_tot = (Fzb * cos(alpha) - Fxb * sin(alpha)) /...
       (0.5 * rho * Qinf^2 * S_w);
   
% total lift
L_tot(i) = (CL_tot * 0.5 * rho * Qinf^2 * S_w);

% net vertical force
netvertical(i) = L_tot(i) - W*cos(alpha(i));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%% second iteration - trim calculator %%%%%%%%
i = 2;

% new angle of attack guess
alpha(i) = alpha(i-1) + 0.0000001*netvertical(i - 1);

% finding alpha_t at new alpha to give zero aircraft moment
myfun=@three_DoF_aircraft_model;
fun=@(x) myfun(x, alpha(i));
alpha_t=fzero(fun,0.0); % trimmed tailplane angle

% calculating new net vertical force
[~, Fxb, Fzb] = three_DoF_aircraft_model(alpha_t, alpha(i));

% main wing lift coefficient
CL_tot(i) = (Fzb * cos(alpha(i)) - Fxb * sin(alpha(i))) /...
          (0.5 * rho * Qinf^2 * S_w);
     
% total lift
L_tot(i) = (CL_tot(i) * 0.5 * rho * Qinf^2 * S_w);

% net vertical force
netvertical(i) = L_tot(i) - W*cos(alpha(i));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%% third iteration and on - trim calculator %%%%%%%%
i = 3;

% exit condition if two consecutive net vertical forces are zero
while abs(netvertical(i - 1) - netvertical(i - 2)) > 1e-3

    % new freestream velocity guess
    alpha(i) = alpha(i-1) - 0.0000001*netvertical(i-1);
    
    % finding alpha_t at new Qinf to give zero aircraft moment
    myfun = @three_DoF_aircraft_model;
    fun = @(x) myfun(x, alpha(i));
    alpha_t = fzero(fun, 0.0); % trimmed tailplane angle
    
    % calculating new net vertical force
    [~, Fxb, Fzb] = three_DoF_aircraft_model(alpha_t, alpha(i));
    
    % main wing lift coefficient
    CL_tot(i) = (Fzb * cos(alpha(i)) - Fxb * sin(alpha(i))) /...
              (0.5 * rho * Qinf^2 * S_w);
        
    % total lift
    L_tot(i) = (CL_tot(i) * 0.5 * rho * Qinf^2 * S_w);
    
    % net vertical force
    netvertical(i) = L_tot(i) - W*cos(alpha(i));

    i = i + 1;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%% calculating trimmed flight forces %%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% trimmed angle of attack
alpha = alpha(i - 1);

% calculate pitching moment and body forces at trim
[MT, FxbT, FzbT] = three_DoF_aircraft_model(alpha_t, alpha);

% form the initial state vector (data for Navion aircraft)
x=zeros(1,6); xdot=zeros(1,6);
xe = 0.0;
ze = 0.0;
ub = Qinf * cos(alpha);
wb = Qinf * sin(alpha); % or negative in dynamics co-ordinates
qb = 0.0;
theta = alpha;

x(1) = xe;
x(2) = ze;
x(3) = ub;
x(4) = wb;
x(5) = qb;
x(6) = theta;

thrust = FxbT + m * g * sin(theta);

x; % initial state vector
[xdot, FxbrT, FzbrT, MbrT] = three_DoF_aircraft_simulator(x,...
                                                        alpha_t, thrust);

% for aero derivative calculations - u velocity perturbation
x1 = x;
du = 1;
x1(3) = x1(3) + du;

[~, Fxbr, Fzbr, Mbr] = three_DoF_aircraft_simulator(x1, alpha_t, thrust);

Xu = (Fxbr - FxbrT) / (du *  0.5 * rho * Qinf * S_w);
Zu = (Fzbr - FzbrT) / (du *  0.5 * rho * Qinf * S_w);
Mu = (Mbr - MbrT) / (du * 0.5 * rho * Qinf * S_w * c_w);


% for aero derivative calculations - w velocity perturbation
x2 = x;
dw = -1;
x2(4) = x2(4) + dw;

[~, Fxbr, Fzbr, Mbr] = three_DoF_aircraft_simulator(x2, alpha_t, thrust);

Xw = (Fxbr - FxbrT) / (dw * 0.5 * rho * Qinf * S_w);
Zw = (Fzbr - FzbrT) / (dw * 0.5 * rho * Qinf * S_w);
Mw = (Mbr - MbrT) / (dw * 0.5 * rho * Qinf * S_w * c_w);


% for aero derivative calculations - pitch rate perturbation
x3 = x;
dq = 1;
x3(5) = x3(5) + dq;

[~, Fxbr, Fzbr, Mbr] = three_DoF_aircraft_simulator(x3, alpha_t, thrust);

Zq = (Fzbr - FzbrT) / (dq * 0.5 * rho * Qinf * S_w * c_w);
Mq = (Mbr - MbrT) / (dq * 0.5 * rho * Qinf * S_w * c_w^2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% flight simulator %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% the flight simulator uses a fourth order runge kutta time stepping method
% this is sufficiently fast enough and does not lead to significant
% divergence as can be seen in the simple euler method for similarly sized
% time steps.

tic
dt = 0.1;
for i=1:1000
    i;
    time(i)=i*dt;
    [K1]=three_DoF_aircraft_simulator(x, alpha_t, thrust);
    [K2]=three_DoF_aircraft_simulator(x+(dt*0.5*K1), alpha_t, thrust);
    [K3]=three_DoF_aircraft_simulator(x+(dt*0.5*K2), alpha_t, thrust);
    [K4]=three_DoF_aircraft_simulator(x+(dt*K3), alpha_t, thrust);
    x=x+dt*(K1+(2*K2)+(2*K3)+K4)/6;
 
 xstore(i)=x(1);
 zstore(i)=x(2);
 ustore(i)=x(3);
 wstore(i)=x(4);
 Qstore(i)=sqrt(x(4)^2+x(5)^2+x(6)^2);
 qbstore(i)=x(5);
 thetastore(i)=x(6);


end
toc

figure

subplot(3,1,1)
plot(xstore,-zstore)
title('Flight Path')
xlabel('Xe (m)')
ylabel('Ze (m)')

subplot(3,2,3)
plot (time,ustore)
title('u Body Velocity')
xlabel('Time (s)')
ylabel('u (m/s)')

subplot(3,2,4)
plot(time,wstore)
title('w Body Velocity')
xlabel('Time (s)')
ylabel('u (m/s)')

subplot(3,2,5)
plot (time,qbstore)
title('Pitch Rate')
xlabel('Time (s)')
ylabel('Pitch Rate (rad/s)')

subplot(3,2,6)
plot (time,thetastore)
title('Theta')
xlabel('Time (s)')
ylabel('Theta (rad)')
