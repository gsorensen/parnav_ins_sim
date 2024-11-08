function exOtterMSS()
% SIMotter is compatibel with MATLAB and GNU Octave (www.octave.org).
% This script simulates the Otter Uncrewed Surface Vehicle (USV) under 
% various control strategies to handle path following in the presence of 
% ocean currents. This script allows the user to select from several control
% methods and simulatesthe USV's performance using a cubic Hermite spline
% or straight-line paths.
%
% The simulation covers:
%   1. PID heading autopilot without path following.
%   2. Adaptive Line-of-Sight (ALOS) control for path following using
%      straight lines and waypoint switching.
%   3. Integral Line-of-Sight (ILOS) control for path following using
%      straight lines and waypoint switching.
%   4. ALOS control for path following using Hermite spline interpolation.
%
% Dependencies:
%   otter.m                 - Dynamics of the Otter USV
%   refModel.m              - Reference model for autopilot systems
%   ALOSpsi.m               - ALOS guidance algorithm for path following
%   ILOSpsi.m               - ILOS guidance algorithm for path following
%   hermiteSpline.m         - Cubic Hermite spline computations
%   crosstrackHermiteLOS.m  - Cross-track error and LOS guidance law for
%                             cubic Hermite splines
%   LOSobserver.m           - Observer for LOS guidance 
%   controlMethods.m        - Menu for choosing control law. 
%
% Simulink Models:
%   demoOtterUSVPathFollowingHeadingControl.slx
%   demoOtterUSVPathFollowingCourseControl.slx
%
% References:
%   T. I. Fossen and A. P. Aguiar (2024). A Uniform Semiglobal Exponential
%   Stable Adaptive Line-of-Sight (ALOS) Guidance Law for 3-D Path Following.
%   Automatica, 163, 111556. https://doi.org/10.1016/j.automatica.2024.111556
%
%   T. I. Fossen (2023). An Adaptive Line-of-sight (ALOS) Guidance Law for
%   Path Following of Aircraft and Marine Craft. IEEE Transactions on Control
%   Systems Technology, 31(6), 2887-2894. 
%   https://doi.org/10.1109/TCST.2023.3259819
%
% Author: Thor I. Fossen
% Date: 2021-04-25
% Revisions:
%   2023-10-14: Added heading autopilot and reference model.
%   2024-04-01: Integrated ALOS/ILOS path-following control algorithms for
%               straight-line paths and Hermite splines.
%   2024-04-21: Enhanced compatibility with GNU Octave.

clearvars;                                  % Clear variables from memory
close all;                                  % Close all figure windows
clear ALOSpsi ILOSpsi crosstrackHermiteLOS  % Clear persistent variables

%% USER INPUTS
h  = 0.01;                       % Sampling time [s]
N  = 20000;                      % Number of samples
T  = 700;
N = T / h;

% Load condition
mp = 25;                         % Payload mass (kg), maximum value 45 kg
rp = [0.05 0 -0.35]';            % Location of payload (m)

% Ocean current
V_c = 0.3;                       % Ocean current speed (m/s)
beta_c = deg2rad(30);            % Ocean current direction (rad)
%V_c = 0;                       % Ocean current speed (m/s)
%beta_c = deg2rad(0);            % Ocean current direction (rad)

% Waypoints
wpt.pos.x = [0 0   150 150 -100 -100 200]';
wpt.pos.y = [0 200 200 -50  -50  250 250]';

%mywpt.pos.x = [-15,-35,-55,-60,-50,-30, -20,        0 20 60 80 80 71 56                    30 -90 30 800  750 650 600 600 600 600]';
%mywpt.pos.y = [-15,-20,-15,  5, 26, 24,  12,     0 -20 -40 -60 -100 -127 -134 -140 -300 -500 -500  -350 -400 -600 -1500 -1550 -1900]';

posses = [
  -50 0
  -45 20
  -35 38
  -20 40
  -10 25
  0 0
  50 0
  50 300
  100 300
  100 400
  -100 400
  -100 -50
  150 -50
  150 400
]';
posses = [
  -100 0
  -95 -20
  -85 -38
  -70 -40
  -60 -25
  -50 0
  -45 20
  -35 38
  -20 40
  -10 25
  0 0
  50 0
  50 300
  100 300
  115 200
  135 200
  150 300
  150 450
]' + [50;0];

mywpt.pos.x = posses(1,:)';
mywpt.pos.y = posses(2,:)';

wpt = mywpt;

%target.pos = [600, -1850]';
target.pos = [150 435]'+ [50;0];

wayPoints = [wpt.pos.x wpt.pos.y]; 
%wayPoints = [mywpt.pos.x mywpt.pos.y];

% ALOS and ILOS parameters
Delta_h = 10;                    % Look-ahead distance
gamma_h = 0.001;                 % ALOS adaptive gain
kappa = 0.001;                   % ILOS integral gain

% Additional parameter for straight-line path following
R_switch = 5;                    % Radius of switching circle
K_f = 0.3;                       % LOS observer gain

% Initial heading, vehicle points towards next waypoint
psi0 = atan2(wpt.pos.y(2) - wpt.pos.y(1), wpt.pos.x(2) - wpt.pos.x(1));

% Additional parameters for Hermite spline path following
Umax = 2;                        % Maximum speed for Hermite spline LOS
idx_start = 1;                   % Initial index for Hermite spline
[w_path, x_path, y_path, dx, dy, pi_h, pp_x, pp_y, N_horizon] = ...
    hermiteSpline(wpt, Umax, h); % Compute Hermite spline for path following

% Otter USV input matrix
[~,~,M, B_prop] = otter();
Binv = inv(B_prop);              % Invert input matrix for control allocation

% PID heading autopilot parameters (Nomoto model: M(6,6) = T/K)
T = 1;                           % Nomoto time constant
K = T / M(6,6);                  % Nomoto gain constant

wn = 1.5;                        % Closed-loop natural frequency (rad/s)
zeta = 1.0;                      % Closed-loop relative damping factor (-)

Kp = M(6,6) * wn^2;                     % Proportional gain
Kd = M(6,6) * (2 * zeta * wn - 1/T);    % Derivative gain
Td = Kd / Kp;                           % Derivative time constant
Ti = 10 / wn;                           % Integral time constant

% Reference model parameters
wn_d = 1.0;                      % Natural frequency (rad/s)
zeta_d = 1.0;                    % Relative damping factor (-)
r_max = deg2rad(10.0);           % Maximum turning rate (rad/s)

% Propeller dynamics
T_n = 0.1;                       % Propeller time constant (s)
n = [0 0]';                      % Initial propeller speed, [n_left n_right]'

% Initial states
eta = [-50 0 0 0 0 psi0]';         % State vector, eta = [x y z phi theta psi]'
nu  = [0 0 0 0 0 0]';            % Velocity vector, nu  = [u v w p q r]'

z_psi = 0;                       % Integral state for heading control
z_spd = 0;                       % Integral state for speed control
psi_d = eta(6);                  % Initial desired heading
r_d = 0;                         % Initial desired rate of turn
a_d = 0;                         % Initial desired acceleration
u_d = 0;                         % Desired velocity (zero, we want to dock)
u_max = 1.5;
% Choose control method and display simulation options
methods = {'PID heading autopilot, no path following',...
           'ALOS path-following control using straight lines and waypoint switching',...
           'ILOS path-following control using straight lines and waypoint switching',...
           'ALOS path-following control using Hermite splines'};
ControlFlag = controlMethod(methods);
displayControlMethod(ControlFlag, R_switch, Delta_h);

%% MAIN LOOP
simdata = zeros(N+1, 15);        % Preallocate table for simulation data

for i = 1:N+1
    t = (i-1) * h;                  % Time (s)
    % Measurements with noise
    x = eta(1) + 0.01 * randn;      % Noisy x measurement
    y = eta(2) + 0.01 * randn;      % Noisy y measurement
    r = nu(6) + 0.001 * randn;      % Noisy rate of turn
    psi = eta(6) + 0.001 * randn;   % Noisy heading
    unow = norm(nu(1:2),2) + 0.001 * randn;   % Noisy speed

    % Guidance and control system
    switch ControlFlag
        case 1  % PID heading autopilot with reference model
            % Reference model, step input adjustments
            psi_ref = psi0;
            if t > 100; psi_ref = deg2rad(0); end
            if t > 500; psi_ref = deg2rad(-90); end

            % Reference model propagation
            [psi_d, r_d, a_d] = refModel(psi_d, r_d, a_d, psi_ref, r_max, ...
                                         zeta_d, wn_d, h, 1);

        case 2  % ALOS heading autopilot straight-line path following
           % wpt.pos.x
          %  wpt.pos.y
            psi_ref = ALOSpsi(x, y, Delta_h, gamma_h, h, R_switch, wpt);
            
            [psi_d, r_d] = LOSobserver(psi_d, r_d, psi_ref, h, K_f);

        case 3  % ILOS heading autopilot straight-line path following
            psi_ref = ILOSpsi(x, y, Delta_h, kappa, h, R_switch, wpt);
            [psi_d, r_d] = LOSobserver(psi_d, r_d, psi_ref, h, K_f);

        case 4  % ALOS heading autopilot, cubic Hermite spline interpolation
            [psi_ref, idx_start] = crosstrackHermiteLOS(w_path, x_path, ...
                y_path, dx, dy, pi_h, x, y, h, Delta_h, pp_x, pp_y, ...
                idx_start, N_horizon, gamma_h);
            [psi_d, r_d] = LOSobserver(psi_d, r_d, psi_ref, h, K_f);
    end
    

    % Computing desired speed based on distance to target
    dpos = [x; y] - target.pos;
    kappa = 1;
    v_d_n = -kappa * dpos / norm(dpos, 2);
    U_d = norm(v_d_n, 2);
    if U_d >= 0; U_d = min(u_max, U_d); else U_d = max(-u_max, U_d); end

    ztar = (x - target.pos(1))^2 + (y - target.pos(2))^2;
    
    if ztar > 7
        tau_X = 100;
    else
        disp("Inside circle of acceptance");
        tau_X = 75*((unow - U_d) + (1/100) * z_spd); 
    end

         tau_N = (T/K) * a_d + (1/K) * r_d - ...   % Yaw moment control
            Kp * (ssa(psi - psi_d) + ...      % Proportional term
            Td * (r - r_d) + (1/Ti) * z_psi); % Derivative and integral terms
    % PID heading (yaw moment) autopilot and forward thrust
   % tau_X = 100 * (unow - U_d);% Derivative and integral terms;   % Constant forward thrust


    % Control allocation
    u = Binv * [tau_X; tau_N];      % Compute control inputs for propellers
    n_c = sign(u) .* sqrt(abs(u));  % Convert to required propeller speeds

    % Store simulation data
    simdata(i, :) = [t, eta', nu', r_d, psi_d];

    % USV dynamics
    xdot = otter([nu; eta], n, mp, rp, V_c, beta_c);  % State derivatives

    % Euler's integration method (k+1)
    % nu = nu + h * xdot(1:6);                % Update velocity states
    % eta = eta + h * xdot(7:12);             % Update position states

    % Zero order hold (assuming small angular velocity)
    nu_prev     = nu;
    nu          = nu + h * xdot(1:6);                % Update velocity states
    eta_prev    = eta;     
    J           = eulerang(eta_prev(4),eta_prev(5),eta_prev(6));
    eta         = eta_prev + J*(nu + nu_prev)/2*h; 

    eta(4:6) = ssa(eta(4:6));
    
    n = n + h/T_n * (n_c - n);              % Update propeller speeds
    z_psi = z_psi + h * ssa(psi - psi_d);   % Update integral state
    z_spd = z_spd + h * (unow - U_d);       % Update integral state speed
end

%% PLOTS
scrSz = get(0, 'ScreenSize'); % Get screen dimensions
legendSize = 12;

% Simulation data structure
t = simdata(:,1);        % Time vector
eta = simdata(:,2:7);    % State vectors
nu  = simdata(:,8:13);   % Velocity vectors
r_d = simdata(:,14);     % Desired rate of turn
psi_d = simdata(:,15);   % Desired heading


user = compiler.UserInfo();
switch user.UserID
    case 'thb'
        [filepath,~,~] = fileparts(pwd);
        save_folder = fullfile( filepath, 'data');

        if ~exist( save_folder, 'dir')
            mkdir( save_folder )
        end
        save_file = fullfile( save_folder, 'otter_test_data.mat' );
        save(save_file, "t","h", "eta", "nu", "N");
    otherwise
        eta(:,6) = ssa(eta(:,6));
        save("~/ws/ntnu/parnav_ins_simulator/data/otter_test_data.mat", "t","h", "eta", "nu", "N");
end

% Plot positions
f1=figure(1);
set(gcf,'Position',[0.6*scrSz(3),0.2*scrSz(4), ...
    0.5*scrSz(4),0.6*scrSz(4)],'Visible','off');
hold on;
plot(eta(:,2),eta(:,1),'b');  % Plot vehicle position

% Control method specific plots
if ControlFlag == 1  % Heading autopilot
    legend('Vehicle position');
elseif any(ControlFlag == [2, 3])  % Straight line and circles
    plotStraightLinesAndCircles(wayPoints, R_switch);
else  % Hermite splines
   % plotHermiteSplines(y_path, x_path, wayPoints);
end

xlabel('East (m)', 'FontSize', 14);
ylabel('North (m)', 'FontSize', 14);
title('North-East Positions (m)', 'FontSize', 14);
axis equal;
grid on;
set(findall(gcf,'type','line'),'linewidth',2);
set(findall(gcf,'type','legend'),'FontSize',legendSize);
set(1,'Visible', 'on');  % Show figure

exportgraphics(f1, "trajectory_otter.pdf", "ContentType","vector");

% Plot velocities
figure(2);
if ~isoctave; set(gcf, 'Position', [1, 1, 0.3*scrSz(3), scrSz(4)]); end
subplot(611),plot(t,nu(:,1));
xlabel('Time (s)'),title('Surge velocity (m/s)'),grid on;
subplot(612),plot(t,nu(:,2));
xlabel('Time (s)'),title('Sway velocity (m/s)'),grid on;
subplot(613),plot(t,nu(:,3));
xlabel('Time (s)'),title('Heave velocity (m/s)'),grid on;
subplot(614),plot(t,rad2deg(nu(:,4)));
xlabel('Time (s)'),title('Roll rate (deg/s)'),grid on;
subplot(615),plot(t,rad2deg(nu(:,5)));
xlabel('Time (s)'),title('Pitch rate (deg/s)'),grid on;
subplot(616),plot(t,rad2deg(nu(:,6)),t,rad2deg(r_d));
xlabel('Time (s)'),title('Yaw rate (deg/s)'),grid on;
legend('r','r_d');
set(findall(gcf,'type','line'),'linewidth',2);
set(findall(gcf,'type','text'),'FontSize',14);
set(findall(gcf,'type','legend'),'FontSize',legendSize);

% Plot speed, heave position and Euler angles
figure(3);
if ~isoctave
    set(gcf, 'Position', [0.3*scrSz(3), 1, 0.3*scrSz(3), scrSz(4)]); 
end
subplot(511),plot(t, sqrt(nu(:,1).^2+nu(:,2).^2));
title('Speed (m/s)'),grid on;
subplot(512),plot(t,eta(:,3),'linewidth',2);
title('Heave position (m)'),grid on;
subplot(513),plot(t,rad2deg(eta(:,4)));
title('Roll angle (deg)'),grid on;
subplot(514),plot(t,rad2deg(eta(:,5)));
title('Pitch angle (deg)'),grid on;
subplot(515),plot(t,rad2deg(unwrap(eta(:,6))),t,rad2deg(unwrap(psi_d)));
xlabel('Time (s)'),title('Yaw angle (deg)'),grid on;
legend('\psi','\psi_d');
set(findall(gcf,'type','line'),'linewidth',2);
set(findall(gcf,'type','text'),'FontSize',14);
set(findall(gcf,'type','legend'),'FontSize',legendSize);

% Display the vehicle data and an image of the vehicle
vehicleData = {...
    'Length', '2.0 m',...
    'Beam', '1.08 m',...
    'Draft (no payload)', '13.4 cm',...
    'Draft (25 kg payload)', '19.5 cm',...    
    'Mass (no payload)', '55.0 kg',...
    'Max speed', '3.0 m/s',...
    'Max pos. propeller speed', '993 RPM',...
    'Max neg. propeller speed', '-972 RPM'};
displayVehicleData('Maritime Robotics Otter USV', vehicleData, 'otter.jpg', 4);

end

%% HELPER FUNCTIONS FOR PLOTTING
function plotStraightLinesAndCircles(wayPoints, R_switch)
    legendLocation = 'best';
    if isoctave; legendLocation = 'northeast'; end
    
    % Plot straight lines and circles for straight-line path following
    for idx = 1:length(wayPoints(:,1))-1
        if idx == 1
            plot([wayPoints(idx,2),wayPoints(idx+1,2)],[wayPoints(idx,1),...
                wayPoints(idx+1,1)], 'r--', 'DisplayName', 'Line');
        else
            plot([wayPoints(idx,2),wayPoints(idx+1,2)],[wayPoints(idx,1),...
                wayPoints(idx+1,1)], 'r--','HandleVisibility', 'off');
        end
    end

    theta = linspace(0, 2*pi, 100);
    for idx = 1:length(wayPoints(:,1))
        xCircle = R_switch * cos(theta) + wayPoints(idx,1);
        yCircle = R_switch * sin(theta) + wayPoints(idx,2);
        plot(yCircle, xCircle, 'k');
    end

    legend('Vehicle position','Straight-line path','Circle of acceptance',...
        'Location',legendLocation);
end

function plotHermiteSplines(y_path, x_path, wayPoints)
    legendLocation = 'best';
    if isoctave; legendLocation = 'northeast'; end

    % Plot Hermite spline paths for spline path following
    plot(y_path, x_path, 'r');
    plot(wayPoints(:, 2), wayPoints(:, 1), 'ko',...
        'MarkerFaceColor', 'g', 'MarkerSize',10);
    legend('Vehicle position','Hermite spline','Waypoints', ...
        'Location',legendLocation);
end

%% DISPLAY CONTROL METHOD
function displayControlMethod(ControlFlag, R_switch, Delta_h)
    disp('--------------------------------------------------------------------');
    disp('MSS toolbox: Otter USV');
    switch ControlFlag
        case 1
            disp('PID heading autopilot with reference feedforward');
        case 2
            disp(['ALOS path-following control using straight lines and ' ...
                'waypoint switching']);
            disp(['Circle of acceptance: R = ', num2str(R_switch), ' m']);
            disp(['Look-ahead distance: Delta_h = ', num2str(Delta_h), ' m']);
        case 3
            disp(['ILOS path-following control using straight lines and ' ...
                'waypoint switching']);
            disp(['Circle of acceptance: R =  ', num2str(R_switch), ' m']);
            disp(['Look-ahead distance: Delta_h = ', num2str(Delta_h), ' m']);
        case 4
            disp('ALOS path-following control using Hermite splines');
            disp(['Look-ahead distance: Delta_h = ', num2str(Delta_h), ' m']);
    end
    disp('--------------------------------------------------------------------');
    disp('Simulating...');
end
