% use this script to initialise the simulation's constants.


TF = 200;    % simulation time
dt = 1 / 60; % simulation time step


init_altitude = 2000;                           % initial altitude
lat0 = deg2rad(21);                             % initial latitude
long0 = deg2rad(157);                           % initial longitude
theta_initial = 0.177;                          % initial pitch angle
Qinf_initial = sqrt(55.9^2 + 10^2);             % free stream velocity
pitch_rate_initial = 0.0;                       % initial pitch rate
u_initial = Qinf_initial * cos(theta_initial);  % initial ub velocity
w_initial = Qinf_initial * sin(theta_initial);  % initial wb velocity

tail_deflection_initial = -0.177;               % initial tail deflection
throttle_initial = 0.584;                       % initial throttle setting

% initial state vector
x0 = [u_initial;
      w_initial;
      pitch_rate_initial;
      theta_initial];

% initial control vector
u_c = [tail_deflection_initial;
    throttle_initial];

% max negative elevator deflection
u_c1_min = -15*pi/180;
% max positive elevator deflection
u_c1_max = 15 * pi/180;

% minimum throttle
u_c2_min = 0;
% maximum throttle
u_c2_max = 1;
