function[xdot, Fxbr, Fzbr, M] = three_DoF_aircraft_simulator(x,...
                                                           alpha_t, thrust)    

% By D. Clifford 28/08/2020
       
% This is a modified version of three_DoF_aircraft_model for flight
% simulation, taking in the aircraft's state vector, x and the longitudinal
% controls, alpha_t and thrust.

% THIS WAS THE INITIAL IMPLEMENTATION OF THE METHOD AND IS USED TO FIND THE
% AIRCRAFT'S STABILITY DERIVATIVES IN "simulator.m". A BETTER
% IMPLEMENTATION IS IN THE SIMULINK MODEL.

% *** YOU NEED AN AIRCRAFT DATA FILE FOR THIS WORK - BEFORE RUNNING THIS
% PROGRAM ENSURE THE DATA IN THE AIRCRAFT DATA FILE IS SAVED TO THE 
% WORKSPACE. AN EXAMPLE DATA FILE IS GIVEN AS "navion_details".

% call navion_details script
navion_details_six_DoF

% defining state vector entries
xe = x(1);
ze = x(2);
ub = x(3);
wb = x(4);
qb = x(5);
theta= x(6);
g=9.81;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%% aircraft details %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

lt = l - lw;                        % cg distance to tailplane ac
N_w = 4;                            % no. of main wing elements
N_tp_h = 1;                         % no. of horizontal tail plane elements
N_tp_v = 1;                         % no. vertical tail plane elements
N_f = 1;                            % no. of fuselage elements
Ntot = N_w + N_tp_h + N_tp_v + N_f; % total no. of vortex elements

% vortex element sizes
fact_w = sqrt(4 / (4 + 1));
fact_tp_h = sqrt(1 / (1 + 1));
fact_tp_v = sqrt(1 / (1 + 1));
fact_f = sqrt(1 / (1 + 1));

% length of the trailing vortices
large = 1.0e6;

% trailing vortex distance factors
bp = b_w * fact_w;
bp_f = b_f * fact_f;
dy = bp / N_w;
bp_tp_h = b_tp_h * fact_tp_h;
bp_tp_v = b_tp_v * fact_tp_h;

% control point to quarter chord dist
cp = 0.5 * c_w / fact_w;
cp_f = 0.5 * c_f / fact_f;
cp_tp_h = 0.5 * c_tp_h / fact_tp_h;
cp_tp_v = 0.5 * c_tp_v / fact_tp_v;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%% wing vortex coordinates %%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% generates horseshoe vortex coordinates starting at the port wing tip and 
% ending at the starboard wing tip.

% port wing
xa(:, 1) = [(3 * dy / 2) * tan(sweep_angle);...
            2 * - dy;...
            (2 * dy / cos(sweep_angle)) * tan(dihedral_angle)];
xb(:, 1) = [(3 * dy / 2) * tan(sweep_angle);...
            - dy;...
            (dy / cos(sweep_angle)) * tan(dihedral_angle)];
xa(:, 2) = xb(:, 1) - [dy * tan(sweep_angle); 0; 0];
xb(:, 2) = xa(:, 2) + [0.0;...
                       dy;...
                       - (dy / cos(sweep_angle)) * tan(dihedral_angle)];
                   
% starboard wing
xa(:, 3) = [(dy / 2) * tan(sweep_angle); 0.0; 0.0];
xb(:, 3) = [(dy / 2) * tan(sweep_angle);...
            dy;...
            (dy / cos(sweep_angle)) * tan(dihedral_angle)];
xa(:, 4) = xb(:, 3) + [dy * tan(sweep_angle); 0; 0];
xb(:, 4) = xa(:, 4) + [0.0;...
                       dy;...
                       (dy / cos(sweep_angle)) * tan(dihedral_angle)];

% coords of centre of horseshoe vortices
cc(:, :) = 0.5 * (xa(:, :) + xb(:, :));

% assigning wing coords to horseshoe vortices using the local chord for a
% straight tapered wing equation
cw = croot + (2 * abs(cc(2, :)) / b_w) * (ctip - croot);

% control point distance factors
cp = 0.5 * cw / fact_w;

% wing control point locations and normal vectors
for i = 1 : 4
    xc(:, i) = 0.5 * (xa(:, i) + xb(:, i)) +...
               [cos(alpha_ws) * cp(i); 0.0; sin(alpha_ws) * cp(i)];
end

% port wing normal vector
L1 = xb(:, 1) - xa(:, 1);
L2 = xc(:, 1) - xa(:, 1);

norm1 = cross(L1, L2);
mag1 = sqrt(norm1(1)^2+norm1(2)^2+norm1(3)^2);
unitnorm1 = norm1 / mag1;

n(:, 1) = unitnorm1 * -1;
n(:, 2) = unitnorm1 * -1;

% starboard wing normal vector
L3 = xb(:, 3) - xa(:, 3);
L4 = xc(:, 3) - xa(:, 3);

norm2 = cross(L3, L4);
mag2 = sqrt(norm2(1)^2+norm2(2)^2+norm2(3)^2);
unitnorm2 = norm2 / mag2;

n(:, 3) = unitnorm2 * -1;
n(:, 4) = unitnorm2 * -1;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%% horizontal stabiliser vortex coordinates %%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% coordinates of vortex segment on horizontal stabiliser
xa(:, 5) = [l; - 0.5 * bp_tp_h; 0.0];
xb(:, 5) = [l; + 0.5 * bp_tp_h; 0.0];
xc(:, 5) = 0.5 * (xa(:, 5) + xb(:, 5)) +...
                 [cos(alpha_t) * cp_tp_h; 0; sin(alpha_t) * cp_tp_h];

% tail plane normal vector
n(:, 5) = [sin(alpha_t); 0.0; cos(alpha_t)];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%% vertical stabiliser vortex coordinates %%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% coordinates of vortex segment on vertical stabiliser
xa(:, 6) = [l; 0.0; 0.0];
xb(:, 6) = [l; 0.0; bp_tp_v];
xc(:, 6) = 0.5 * (xa(:, 6) + xb(:, 6)) +...
                 [cp_tp_v; 0.0; 0.0];

% tail plane normal vector
n(:, 6) = [0.0; 1.0; 0.0];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%% fuselage vortex coordinates %%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% coordinates of vortex segment on fuselage
xa(:, 7) = [-lf; 0.0; -bp_f];
xb(:, 7) = [-lf; 0.0; 0.0];
xc(:, 7) = 0.5 * (xa(:, 7) + xb(:, 7)) + [cp_f; 0.0; 0.0];

% fuselage normal vector
n(:, 7) = [0.0; 1.0; 0.0];

Qinf = sqrt(ub^2 + wb^2);

% velocity vectors (relative to the aircraft datum line)
% wing panels from port wing tip to starboard wing tip
Q(:, 1) = [ub + (qb * (-3 * dy / 2 * (cos(sweep_angle))) * tan(dihedral_angle));...
           0.0;...
           wb - (qb * (-(cp(1) - (lw - (3 * dy / 2) * tan(sweep_angle)))))];

Q(:, 2) = [ub + (qb * (-dy / 2 *(cos(sweep_angle))) * tan(dihedral_angle));...
           0.0;...
           wb - (qb * (-(cp(2) - (lw - (dy / 2) * tan(sweep_angle)))))];

Q(:, 3) = [ub + (qb * (-dy / 2 *(cos(sweep_angle))) * tan(dihedral_angle));...
           0.0;...
           wb - (qb * (-(cp(3) - (lw - (dy / 2) * tan(sweep_angle)))))];
       
Q(:, 4) = [ub + (qb * (-3 * dy / 2 *(cos(sweep_angle))) * tan(dihedral_angle));...
           0.0;...
           wb - (qb * (-(cp(4) - (lw - (3 * dy / 2) * tan(sweep_angle)))))];
       
% horizontal stabiliser
Q(:, 5) = [ub; 0.0; wb - qb * (-lt - cp_tp_h)]; 

% vertical stabiliser
Q(:, 6) = [ub + (qb * (-bp_tp_v / 2)); 0.0; wb - (qb * (-lt - cp_tp_v))];

% fuselage
Q(:, 7) = [ub + (qb * (bp_f / 2)); 0.0; wb - (qb * (-(cp_f - (lf + lw))))];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%% solving circulation matrix %%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       
% set matrix and right hand side elements and solve for circulations
for i = 1 : 7
    for j = 1 : 7
        I = vfil(xa(:, j), xb(:, j), xc(:, i));
        I = I + vfil(xa(:, j) + [large; 0; 0], xa(:, j), xc(:, i));
        I = I + vfil(xb(:, j), xb(:, j) + [large; 0; 0], xc(:, i));
        A(i, j) = I(1) * n(1, i) + I(2) * n(2, i) + I(3) * n(3, i);
    end
    rhs(i)= - (Q(1, i) * n(1, i) + Q(2, i) * n(2, i) + Q(3, i) * n(3,i));
end

% solved circulation
gamma = A \ rhs';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%% calculating forces and local velocity %%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% force at centre of bound vortices
bc(:, :) = 0.5 * (xa(:, :) + xb(:, :));

% velocity vectors (relative to the aircraft datum line)
% wing panels from port wing tip to starboard wing tip
Q(:, 1) = [ub + (qb * (-3 * dy / 2 *(cos(sweep_angle))) * tan(dihedral_angle));...
           0.0;...
           wb - (qb * (-(- (lw - (3 * dy / 2) * tan(sweep_angle)))))];

Q(:, 2) = [ub + (qb * (-dy / 2 *(cos(sweep_angle))) * tan(dihedral_angle));...
           0.0;...
           wb - (qb * (-(- (lw - (dy / 2) * tan(sweep_angle)))))];

Q(:, 3) = [ub + (qb * (-dy / 2 *(cos(sweep_angle))) * tan(dihedral_angle));...
           0.0;...
           wb - (qb * (-(- (lw - (dy / 2) * tan(sweep_angle)))))];
       
Q(:, 4) = [ub + (qb * (-3 * dy / 2 *(cos(sweep_angle))) * tan(dihedral_angle));...
           0.0;...
           wb - (qb * (-(- (lw - (3 * dy / 2) * tan(sweep_angle)))))];
       
% horizontal stabiliser
Q(:, 5) = [ub; 0.0; wb - qb * (-lt)]; 

% vertical stabiliser
Q(:, 6) = [ub + (qb * (-bp_tp_v / 2)); 0.0; wb - (qb * (-lt))];

% fuselage
Q(:, 7) = [ub + (qb * (bp_f / 2)); 0.0; wb - (qb * (-(- (lf + lw))))];

for i = 1 : 7
    % finding the local velocity vector u on each load element i
    u(:, i) = Q (:, i);
    for j = 1 : 7
        u(:, i) = u(:, i) + vfil(xa(:, j), xb(:, j), bc(:, i)) * gamma(j);
        u(:, i) = u(:, i) + vfil(xa(:, j) +...
                  [large;0;0], xa(:,j), bc(:,i)) * gamma(j);
        u(:, i) = u(:, i) + vfil(xb(:, j), xb(:,j) +...
                  [large;0;0], bc(:,i)) * gamma(j);
    end
    % u cross s gives the direction of the force
    s = xb(:, i) - xa(:, i);
    Fx(i) = rho * (u(2, i) * s(3) - u(3, i) * s(2)) * gamma(i);
    Fz(i) = rho * (u(1, i) * s(2) - u(2, i) * s(1)) * gamma(i);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%% zero lift drag coefficient estimate %%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% viscosity of air at sea level
mu = 1.789e-5;

% reynolds number
Re_w = (rho * Qinf * (S_tot/b_w))/mu;

% skin friction coefficient 
Cfe_w = 0.00258 + 0.00102 * exp((-6.28e-9) * Re_w) +...
       0.00295 * exp((-2.01e-8) * Re_w);
   
% calculating zero lift drag coefficient
CD0 = Cfe_w * (S_tot/S_w);

% calculating zero lift drag
d0_w = CD0 * 0.5 * rho * Qinf^2 * S_w;
d0_tp_h = CD0 * (S_tp_h / S_w) * 0.5 * rho * Qinf^2 * S_tp_h;
d0_tp_v = CD0 * (S_tp_v / S_w) * 0.5 * rho * Qinf^2 * S_tp_v;
d0_f = CD0 * (S_f  / S_w) * 0.5 * rho * Qinf^2 * S_f;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%% outputs %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% pitching moment about aircraft cg
M = Fz(1) * (lw - bc(1, 1)) + Fz(2) * (lw - bc(1, 2)) +...
    Fz(3) * (lw - bc(1, 3)) + Fz(4) * (lw - bc(1, 4)) -...
    Fz(5) * lt;

% total z body force
Fzb = sum(Fz(1:Ntot));

% total x body force
Fxb = sum(Fx(1:Ntot)) + d0_w + d0_tp_h + d0_tp_v + d0_f;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% state vector %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% align forces with correct axis directions
Fxbr = - Fxb;
Fzbr = - Fzb;

% adding thrust component to x body force
Fxbr = Fxbr + thrust - m*g*sin(theta);
Fzbr = Fzbr + m*g*cos(theta);

xdot(1) = cos(theta) * ub + sin(theta) * wb;
xdot(2)= -sin(theta) * ub + cos(theta) * wb;

xdot(3) = (Fxbr / m) - (qb * wb);
xdot(4) = (Fzbr / m) + (qb * ub);
xdot(5) = M / Iyy;

xdot(6)= qb;


end
