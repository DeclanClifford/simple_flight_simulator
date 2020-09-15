% specifications and flight conditions of a Navion found in
% "Aircraft Stability and Control Data" by G. L. Teper, April 1969
% all units have been converted to metric

% NOTE - approximations of certain geometric data have been made using the
% aircraft sketch given by G. L. Teper

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%% flight conditions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Qinf = 53.6448;                       % free stream velocity
rho = 1.225;                          % free stream density
g = 9.81;                             % acc^ due to gravity

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%% navion specification %%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
m = 1246.319;
W = m * 9.81;                  % aircraft total weight

lambda = -1;                          % engine, 0 for jet, -1 for piston
Ixx = 1421;
Iyy = 4068;
Izz = 4787;
Ixz = 0;

lw = 0.17;                            % dist from AC to cg
l = 5;                                % dist from wing AC to tailplane AC
lf = 2;                               % dist from fuselage front to wing AC

croot = 2.07384;                      % wing root chord
ctip = 1.20253;                           % wing tip chord
b_w = 10.18032;                       % wing span
S_w = ((croot + ctip) / 2) * 2 * b_w / 2; % wing planform area
AR_w = (b_w^2) / S_w;                 % wing aspect ratio
h0 = 0.25;                            % dist from wing LE to AC / by c_w

sweep_angle = 0 * pi / 180;           % wing sweep angle
dihedral_angle = 5 * pi / 180;        % wing dihedral angle
alpha_ws = -5 * pi / 180;              % wing setting angle

b_f = 1.4;                            % fuselage height
c_f = 8.1;                            % fuselage length
S_f  = b_f * c_f;                     % fuselage area
AR_f = (b_f^2) / S_f;                 % fuselage aspect ratio

c_tp_h = 1;                           % tail plane chord
c_tp_v = 1.5;
AR_tp_h = 4;                          % tail plane aspect ratio
AR_tp_v = 1;

sigma = ctip / croot;                 % wing taper ratio
c_w = croot * (2 / 3) * ((1 + sigma + sigma^2) / (1 + sigma));   % wing mean aerodynamic chord
b_tp_h = c_tp_h * AR_tp_h;              % tail chord and tail span
b_tp_v = c_tp_v * AR_tp_v;
S_tp_h = b_tp_h * c_tp_h;                 % tail plane area
S_tp_v = b_tp_v * c_tp_v;

% an extremely rough approximation of total aircaft area (wetted area)
S_tot = S_w * 2 + S_f * 4 + S_tp_h + S_tp_v;
