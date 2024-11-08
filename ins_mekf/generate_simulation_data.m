clear all;

colour_palette = [
    hex2rgb("009e73"); % blue green
    hex2rgb("cc79a7"); % pale violet 
    hex2rgb("0072b2"); % blue
    hex2rgb("e69f00"); % orange
    hex2rgb("56b4e9"); % sky blue
    hex2rgb("d55e00"); % vermillion
    hex2rgb("f0e442"); % yellow
    hex2rgb("000000"); % black
];
colororder(colour_palette);

more_locator_test = false;

generate_for_otter = true;

generat_stand_still = false;
                
Hz = 100;
Ts = 1 / Hz;
if generate_for_otter == true
    load(sprintf("../data/otter_PVA_data_%dHz.mat", Hz));
elseif generat_stand_still == true
    load(sprintf("../data/standstill_PVA_data_%dHz.mat", Hz));
else
    load(sprintf("../data/PVA_data_%dHz.mat", Hz));
end
% Number of simulations
M = 100;

if generate_for_otter == false && generat_stand_still == false
locator_origins = [
    [-650, -650, 750];
    [650, -650, 750];
    [-650, 650,750];
    [650, 650, 750];
]';
locator_yaws = [
    0;
    0;
    0;
    0;
] * (pi/180);
elseif generat_stand_still == true
locator_origins = [
    [100 100 -10];
    [-100 100 0];
    [100 -100 -10];
    [-100 -100 0];
]';
locator_yaws = [
    0;
    0;
    0;
    0;
] * (pi/180);

else
locator_origins = [
    [190 470 -12];
%    [210 470 -8];
    [-40 20 -12];
%    [-60 0 -8];
]';
locator_yaws = [
    0;
    0;
   % 0;
 %   0;
] * (pi/180);

end

f=figure(1241223412); clf; hold on; grid on;
plot(pos(2, :), pos(1,:), "LineWidth", 1.5, "Color", colour_palette(3,:));
plot(locator_origins(2,1), locator_origins(1,1), "*", "LineWidth", 2.0,"Color", colour_palette(1,:));
plot(locator_origins(2,2), locator_origins(1,2), "*", "LineWidth", 2.0,"Color", colour_palette(2,:));

xlabel("East [m]", "Interpreter","latex","FontSize", 11); ylabel("North [m]", "Interpreter","latex","FontSize", 11);

xlim([min(pos(2,:))-20, max(pos(2,:))+20]);
ylim([min(pos(1,:))-30, max(pos(1,:))+20]);
text(pos(2,1)+5, pos(1,1), "Start", "Interpreter","latex","FontSize", 11);
text(pos(2,N)-6, pos(1,N)+5, "End", "Interpreter","latex","FontSize", 11);
%fill([50, 450],[-50,90],colour_palette(5,:));
%x = [-100 85 85 85 150 150 85 -100];
%y = [50   50 500 325 325 500 500 500];
%fill(y,x,[.7 .7 .7])
plot(locator_origins(2,:), locator_origins(1,:), "k--")
leg=legend("Vessel trajectory", "Locator 1", "Locator 2", "Locators  LOS", "Interpreter", "latex","FontSize", 11);
leg.Location = "southeast";
leg.NumColumns = 4;
name = sprintf("%s.pdf", "simulation_trajectory_updated_with_los");



exportgraphics(f, name,'ContentType','vector');

num_locators = size(locator_origins, 2);

if more_locator_test == true
    locator_origins = [locator_origins locator_origins locator_origins locator_origins locator_origins];
    locator_yaws = zeros(20,1);
    num_locators = size(locator_origins, 2);
end
%%
% 1 Spike
% 2 Multipath
err_counts = cell(1,M);
for iteration = 1 : M

fprintf("Generating run %d\n", iteration);

% Describe sim data
if generate_for_otter == true
    output_file = sprintf("../data/otter_simulation_data_%02d_%dHz", iteration, Hz);
elseif generat_stand_still == true
    output_file = sprintf("../data/standstill_simulation_data_%02d_%dHz", iteration, Hz);
else
    output_file = sprintf("../data/simulation_data_%02d_%dHz", iteration, Hz);
end
% Settings
add_noise = true;
if add_noise
    output_file = sprintf("%s_noisy", output_file);
end

add_bias = true;
if add_bias
    output_file = sprintf("%s_biased", output_file);
end

add_extra_noise = false;

add_sensor_faults = false;

aiding_Hz = 10;

output_file = sprintf("%s_aided_at_%dHz", output_file, aiding_Hz)
output_file_cpp = sprintf("%s_cpp.csv", output_file);

% Allocate the true state vectors
% TODO ADD DIMENSION M throughout to keep one file
t           = linspace(0, Ts*N, N);
q_nb        = zeros(4,N);
R_nb        = zeros(3,3,N);
delta_R_i_b = zeros(3,3,N - 1);
omega_ib_b  = zeros(3,N);
v_ib_i      = zeros(3,N);
a_ib_i      = zeros(3,N);
alpha_ib_b  = zeros(3,N);
f_ib_i      = zeros(3,N); 
f_ib_b      = zeros(3,N); 
b_acc       = zeros(3, N); 
b_ars       = zeros(3, N); 
g_0         = 9.81;
g_b_i       = [0 0 g_0]';

b_acc_const = [ -0.276839 -0.244186 0.337360 ]';
b_ars_const = [ -0.0028 0.0021 -0.0032 ]';

% TODO READ FROM FILE
% Change locator positions
% Define sectors, 
% Take locator position, add on what it believes robot position is (cartesian), should
% be robot position


% Allocate the measurement vectors
% Measurements for PARS and GNSS will be generated regardless of use case,
% so the same set of simulation data can be used in all cases
z_GNSS_pos  = zeros(3, N);
z_GNSS_pos_2  = zeros(3, N);
z_lever       = zeros(3, N);
z_GNSS_vel  = zeros(3, N);
z_PARS      = zeros(3 * size(locator_origins, 2), N);
n_PARS      = zeros(3 * size(locator_origins, 2), N);

z_comp      = zeros(1, N);
omega_meas  = zeros(3,N);
f_meas      = zeros(3,N); 

% Process noise covariance initialisation

% TODO MOVE TO TUNING FILE
vrw = 0.07; % m / s / sqrt(h) 
arw = 0.15; % deg per sqrt(h)
bias_instability_acc = 0.05; % milli g
bias_instability_ars = 0.5; % deg per hour
T_acc = 3600;
T_ars = 3600;

Q_v             = diag([1,1,1]) * (vrw/60)^2;
Q_q             = diag([1,1,1]) * ((arw * pi)/(60 * 180))^2;
Q_b_acc         = diag([1 1 1]) * sqrt((2/T_acc) * (bias_instability_acc * g_0/1000))^2;
Q_b_ars         = diag([1 1 1]) * sqrt((2/T_ars) * ((bias_instability_ars * pi)/(180 * 3600)))^2;

% Measurement noise covariance initialisation
R_GNSS_pos           = diag([1.5 1.5 3])^2;
R_GNSS_vel           = diag([0.2 0.2 0.4])^2;

sigma_rho   = 5; % if this is set to e.g., 10 or less, the initial estimates for z-positon is far worse than when not using PARS
sigma_Psi   = 7 * deg2rad(1) ;
sigma_alpha = 7 * deg2rad(1) ;
R_PARS      = diag([sigma_rho sigma_Psi sigma_alpha])^2;

% Initialise true state vectors
v_ib_i(:,1)         = vel(:,1); % Using the initial value from "true" velocity
a_ib_i(:,1)         = acc(:,1); 
q_nb(:,1)           = eulerAngles2quat( roll(1), pitch(1), yaw(1) );
R_nb(:,:,1)         = quat2rotMat_fast( q_nb(:,1) );

if add_bias
    b_acc(:, 1) = b_acc_const;
    b_ars(:, 1) = b_ars_const;
end

% Initialise sensor objects to generate measurements
gnss_sensor = GNSS(R_GNSS_pos, R_GNSS_vel, add_noise);
imu_sensor = IMU(Q_v, Q_q, Q_b_acc, Q_b_ars, T_acc, T_ars);

for idx = 1 : size(locator_origins, 2)
    locators(idx) = BLEPARS(sigma_rho, sigma_Psi, sigma_alpha,...
                            locator_origins(:,idx),...
                            locator_yaws(idx), ...
                            add_noise);
end

% Generate the correct measurements etc.
for k = 2 : N 
    q_nb(:,k)   = euler2q( roll(k), pitch(k), yaw(k) );
    R_nb(:,:,k) = quat2rotMat_fast(q_nb(:,k) );

    if add_bias
        b_acc(:, k) = fogm(b_acc(:, k - 1), Ts, Q_b_acc^(1/2), T_acc);
        b_ars(:, k) = fogm(b_ars(:, k - 1), Ts, Q_b_ars^(1/2), T_ars);
    end

    % Numerical differentiation (Groves App. J)
    v_ib_i(:,k) = (2 / Ts) * (pos(:, k) - pos(:, k - 1)) - v_ib_i(:, k - 1);
    a_ib_i(:,k) = (v_ib_i(:, k) - v_ib_i(:, k - 1)) * (1/Ts);

    % True IMU measurements
    f_ib_i(:, k) = a_ib_i(:, k) - g_b_i;
    f_ib_b(:, k) = R_nb(:, :, k)' * f_ib_i(:, k);


    delta_R_i_b(:, :, k - 1) = R_nb(:, :, k)' * R_nb(:, :, k - 1);
    mu = acos(0.5 * (trace(delta_R_i_b(:, :, k - 1)) - 1));

    if mu > 2e-5
        mu = (mu / (2 * sin(mu)));
    else
        mu = 0.5;
    end
    alpha_ib_b(:, k) = mu * [
        delta_R_i_b(2,3,k-1) - delta_R_i_b(3,2,k-1); 
        delta_R_i_b(3,1,k-1) - delta_R_i_b(1,3,k-1);
        delta_R_i_b(1,2,k-1) - delta_R_i_b(2,1,k-1)
    ];
    omega_ib_b(:, k) = alpha_ib_b(:, k) / Ts;
    
    % Use the IMU sensor object to create noisy, biased measurements
    [f_meas(:,k), omega_meas(:,k)] = ...
        imu_sensor.generate_measurement(f_ib_b(:,k), omega_ib_b(:,k),...
                                        b_acc(:, k - 1), b_ars(:, k - 1),...
                                        Ts, add_noise, add_bias);
    
    % Time step where we need aiding measurements
    if mod(k, aiding_Hz) == 0
        [z_GNSS_pos(:,k), ~, ~] = gnss_sensor.generate_pos_measurement(pos(:, k), Ts);
        [z_GNSS_pos_2(:,k), ~, ~] = gnss_sensor.generate_pos_measurement(pos(:, k), Ts, R_nb(:,:,k), [0 1 0]');
        z_lever(:, k) = R_nb(:,:,k) * [0 1 0]' + 0.05 * randn(3,1);
        [z_GNSS_vel(:,k), ~, ~] = gnss_sensor.generate_vel_measurement(v_ib_i(:, k), Ts);

        z_PARS_k = [];
        h_PARS_k = [];

        for idx = 1 : size(locators, 2)
            [~, z_idx, ~, ~] = locators(idx).generate_measurement([pos(:,k); zeros(12,1)]);     
            hval = locators(idx).h([pos(:,k); zeros(12,1)]);
            z_PARS_k = [z_PARS_k; z_idx];
            h_PARS_k = [h_PARS_k; hval];
        end

        z_PARS(:, k) = z_PARS_k;
        n_PARS(:, k) = z_PARS_k - h_PARS_k;
    end
end


% Sensor failures

% Chances out of 1000
chance_of_null_reading = 0; % Not used
chance_of_no_output = 0;
chance_of_repeated_reading = 0;
chance_of_abnormal_error_spike = 30;
chance_of_abnormal_error_mp = 30;

% Initialise sensor fault simulator
pars_sensor_fault = SensorFaultSimulator([0;0;0],...
                                         [sigma_rho; sigma_alpha; sigma_Psi ], ...
                                         chance_of_no_output, ...
                                         chance_of_null_reading, ...
                                         chance_of_repeated_reading, ...
                                         chance_of_abnormal_error_spike,...
                                         chance_of_abnormal_error_mp);
%%
e_PARS = zeros(size(z_PARS));
error_flags = zeros(size(z_PARS));
for idx = 1 : 3 :  3 * num_locators
    [e_PARS(idx:idx+2, :),  error_flags(idx:idx+2, :)] = pars_sensor_fault.roll_errors(z_PARS(idx:idx+2, :));
end
err_counts{iteration} = error_flags;
%%
% Random initialisation
u_pos = randn(3,1); u_pos = u_pos / norm(u_pos);
u_vel = randn(3,1); u_vel = u_vel / norm(u_vel);
u_att = randn(3,1); u_att = u_att / norm(u_att);
u_b_acc = randn(3,1); u_b_acc = u_b_acc / norm(u_b_acc);
u_b_ars = randn(3,1); u_b_ars = u_b_ars / norm(u_b_ars);

A_pos = 5;
A_vel = 1;
A_att = deg2rad(5); 
A_b_acc = 0.2;
A_b_ars = deg2rad(0.1);

if (~add_noise && ~add_bias)
first_pos = pos(:, 1);
first_vel = v_ib_i(:, 1);
first_att = q_nb(:,1);
first_b_acc = [0 0 0]';
first_b_ars = [0 0 0]';
else
first_pos = pos(:, 1) + A_pos * u_pos;
first_vel = v_ib_i(:, 1) + A_vel * u_vel;
if generat_stand_still == true
    A_att = deg2rad(45);
    yaw_offset = eul2quat([pi/4 0 0]);
    first_att=quatMulti(q_nb(:,1), yaw_offset'); first_att = first_att / norm(first_att);
else
first_att = quatMulti(q_nb(:,1), eul2quat(A_att .* u_att')'); first_att = first_att / norm(first_att);
end
first_b_acc = [0 0 0]';
first_b_ars = [0 0 0]';
end

%%
delta_z_gnss = z_GNSS_pos_2 - z_GNSS_pos;
%delta_z_gnss_trimmed = [nonzeros(delta_z_gnss(1,:))';
%                        nonzeros(delta_z_gnss(2,:))';
%                        nonzeros(delta_z_gnss(3,:))';];
yaw_est = zeros(1, size(delta_z_gnss, 2));
for k = 1 : size(delta_z_gnss,2)
    if mod(k, 10) == 0
        z_lever(:,k) = z_lever(:,k) / norm(z_lever(:,k));
        %delta_z_gnss(:,k) = delta_z_gnss(:,k) / norm(delta_z_gnss(:,k));
       % yaw_est(k) = atan2(delta_z_gnss(2,k),delta_z_gnss(1,k));
    end
end

%nonzeros(yaw_est) .* (180/pi)



%%
% Matlab saving
save(output_file, "pos", "v_ib_i", "q_nb", "f_meas", "omega_meas", ...
    "b_acc", "b_ars", "t", "z_GNSS_pos", "z_GNSS_vel", "Hz",...
    "aiding_Hz", "A_pos", "first_pos", "A_vel", "first_vel", "A_att",...
    "first_att", "A_b_acc", "first_b_acc", "A_b_ars", "first_b_ars",...
    "N", "num_locators", "z_PARS", "locator_origins", "locator_yaws", ...
    "Q_b_acc", "Q_b_ars", "g_b_i", "T_acc", "T_ars", "Q_v", "Q_q",...
    "sigma_rho", "sigma_alpha", "sigma_Psi", "R_PARS", "R_GNSS_pos",...
    "R_GNSS_vel", "R_GNSS_lev","roll", "pitch", "yaw", "e_PARS", "error_flags", "z_lever");


% C++ Saving
output_matrix = zeros(N, 53 + num_locators * 3 + 1 + 3 * num_locators + num_locators * 3 + 3);
for idx = 1 : N
    output_matrix(idx, :) = [
        1 pos(:, idx)' v_ib_i(:, idx)' q_nb(:, idx)' f_meas(:, idx)' omega_meas(:, idx)' b_acc(:, idx)' b_ars(:, idx)' t(idx) z_GNSS_pos(:, idx)'  z_GNSS_vel(:, idx)' Hz aiding_Hz...
        A_pos first_pos' A_vel first_vel' A_att first_att' A_b_acc first_b_acc' A_b_ars first_b_ars' num_locators z_PARS(:, idx)' reshape(locator_origins,[1 3 * num_locators]) e_PARS(:, idx)'  z_lever(:, idx)'
    ];     
end

writematrix(output_matrix, output_file_cpp);
%%
if iteration == M

end
%%
%figure(41); clf; hold on;

%bar(iteration, err_counts, 'stacked');
%legend(bar_legend)
%%
%{
%if iteration == M
load("../data/otter_simulation_data_88_100Hz_noisy_biased_aided_at_10Hz.mat")
f=figure(53125123); clf; hold on;
counter = 1;
locator_counter = 1;
titles = ["Range" "Elevation" "Azimuth"];
for idx = 1 : num_locators
    for measurement_idx = 1 : 3
        curr_idx = measurement_idx + 3 * (idx - 1);

        if measurement_idx == 1
            subplot(num_locators,3,curr_idx); hold on;
            plot(t(z_PARS(1,:)~=0), nonzeros((z_PARS(curr_idx, :)' + e_PARS(curr_idx,:)')) - nonzeros(z_PARS(curr_idx, :)'), "Color", colour_palette(1,:));
           % plot(t(z_PARS(1,:)~=0), nonzeros((z_PARS(curr_idx, :)')), "Color", colour_palette(2,:));
            ylabel("[m]")
        else
            subplot(num_locators, 3, curr_idx); hold on;
            plot(t(z_PARS(1,:)~=0), (180/pi).*(nonzeros((z_PARS(curr_idx, :)' + e_PARS(curr_idx,:)')) - nonzeros(z_PARS(curr_idx, :)')), "Color", colour_palette(1,:));
%            plot(t(z_PARS(1,:)~=0),nonzeros((180/pi).*(z_PARS(curr_idx, :)' + e_PARS(curr_idx,:)')), "Color", colour_palette(1,:));
 %           plot(t(z_PARS(1,:)~=0),nonzeros((180/pi).*z_PARS(curr_idx, :)'), "Color", colour_palette(2,:));
            ylabel("[deg]")
        end
        xlabel("Time [s]")
        leg=legend("Error due to fault"); leg.Location="best";
        title(titles(measurement_idx))
    end
end
exportgraphics(f, "zwitherrors.pdf",'ContentType','vector');
%end
%}
%%
% 88 was randomly chosen as illustration (from uniform int dist.)
if m == 88
f=figure(14123); clf; hold on;
tiled = tiledlayout(3,1);
load("../data/otter_simulation_data_88_100Hz_noisy_biased_aided_at_10Hz.mat")

for measurement_idx = 1 : 3
    nexttile; hold on;
    legendtext = strings(num_locators,1);
    count = 1;
    for idx = 1 : num_locators
       %disp("Loop run")
       curr_idx = measurement_idx + 3 * (idx - 1); 
       if measurement_idx == 1
            %subplot(num_locators,3,curr_idx); hold on;
            plot(t(z_PARS(1,:)~=0), nonzeros((z_PARS(curr_idx, :)' + e_PARS(curr_idx,:)')) - nonzeros(z_PARS(curr_idx, :)'));
            %ylabel("[m]")
       else
            %subplot(num_locators, 3, curr_idx); hold on;
            plot(t(z_PARS(1,:)~=0), (180/pi).*(nonzeros((z_PARS(curr_idx, :)' + e_PARS(curr_idx,:)')) - nonzeros(z_PARS(curr_idx, :)')));
           % ylabel("[deg]")
       end
       legendtext(count) = sprintf("Locator %d", idx);

        
       if measurement_idx == 1
           ylabel("Range [m]", "Interpreter", "latex")
           legend(legendtext, "Location","south", "NumColumns",num_locators, "Interpreter", "latex", "FontSize",12);
       elseif measurement_idx == 2
           ylabel("Elevation [deg]", "Interpreter", "latex")
       elseif measurement_idx == 3
           ylabel("Azimuth [deg]", "Interpreter", "latex")
       end

       count = count + 1;
    end
end
%title(tiled, "Measurement errors caused by sensor faults in run 88 of 100", "Interpreter", "latex");
xlabel(tiled, "Time [s]", "Interpreter", "latex");
exportgraphics(f, "z_faults.pdf",'ContentType','vector');
end
%%
%figure(21); clf; hold on;
%plot((e_PARS(2,z_PARS(2,:) ~= 0)'-nonzeros(z_PARS(2,:))).*(180/pi))
%plot((e_PARS(2,:)-(z_PARS(2,:))).*(180/pi), "*")
%%
%{
subplot(4,3,7); hold on;
plot(nonzeros((180/pi).*z_PARS(8, :)'));
subplot(4,3,8); hold on;
plot(nonzeros((180/pi).*z_PARS(9, :)'));
subplot(4,3,9); hold on;
plot(nonzeros((z_PARS(7, :)')));
subplot(4,3,10); hold on;
plot(nonzeros((180/pi).*z_PARS(11, :)'));
subplot(4,3,11); hold on;
plot(nonzeros((180/pi).*z_PARS(12, :)'));
subplot(4,3,12); hold on;
plot(nonzeros((z_PARS(10, :)')));

%%
figure(1232141); clf; hold on;
plot_idx_start  = 3;
plot_idx_end    = 200;
plot_idx_end    = length(pos);
timevec = t(plot_idx_start:plot_idx_end);
% pos 
subplot(3,4,1); 
    plot(timevec,pos(1,plot_idx_start:plot_idx_end));
    ylabel('North [m]')
    title('Pos NED')
    grid on;
subplot(3,4,5)    
    plot(timevec,pos(2,plot_idx_start:plot_idx_end));
    ylabel('East [m]')
    grid on;
subplot(3,4,9)    
    plot(timevec,pos(3,plot_idx_start:plot_idx_end));
    grid on;
    ylabel('Down [m]')
    xlabel('Time [s]')
%vel 
subplot(3,4,2); 
    plot(timevec,v_ib_i(1,plot_idx_start:plot_idx_end));
    ylabel('North [m]')
    title('Vel NED')
    grid on;
subplot(3,4,6)    
    plot(timevec,v_ib_i(2,plot_idx_start:plot_idx_end));
    ylabel('East [m]')
    grid on;
subplot(3,4,10)    
    plot(timevec,v_ib_i(3,plot_idx_start:plot_idx_end));
    grid on;
    ylabel('Down [m]')
    xlabel('Time [s]')
%acc 
subplot(3,4,3); 
    plot(timevec,a_ib_i(1,plot_idx_start:plot_idx_end));
    ylabel('North [m]')
    title('Acc NED')
    grid on;
subplot(3,4,7)    
    plot(timevec,a_ib_i(2,plot_idx_start:plot_idx_end));
    ylabel('East [m]')
    grid on;
subplot(3,4,11)    
    plot(timevec,a_ib_i(3,plot_idx_start:plot_idx_end));
    grid on;
    ylabel('Down [m]')
    xlabel('Time [s]')
%eulerang 
subplot(3,4,4); 
    plot( timevec,180/pi.*roll(plot_idx_start:plot_idx_end) );
    ylabel('Roll [deg]')
    title('Attitude')
    grid on;
subplot(3,4,8)    
    plot(timevec, 180/pi.*pitch(plot_idx_start:plot_idx_end) );
    ylabel('Pitch [deg]')
    grid on;
subplot(3,4,12)    
    plot(timevec, 180/pi.*yaw(plot_idx_start:plot_idx_end) );
    grid on;
    ylabel('Yaw [deg]')
    xlabel('Time [s]')


%}
end
%%
% Input M element cell array of dimensions (num_locators * 3) x N
% Output (error_types * num_locators) x M matrix 
% Parse error flags and identify how many errors per run there was for each
% kind of error for each locator. 
aggregated_error_counts = zeros(M, 2 * num_locators);
errlegendtext = string(2 * num_locators);
for m = 1 : M
    errs = err_counts{m};
    place_count = 1;
    locator_count = 1;

    for idx = 1 : 3 : 3 * num_locators
        num_spike_errors = size(nonzeros(errs(idx, :) ~= 0),1);
        num_multipath_errors = size(nonzeros((errs(idx+1, :) ~= 0 & errs(idx,:) == 0)),1);
        num_total_elev_error = size(nonzeros(errs(idx + 1, :) ~= 0),1);

        if num_total_elev_error ~= (num_spike_errors + num_multipath_errors)
            disp("Something is wrong, or there are more than two error kinds now");
        end

        nummeas = floor(N/10);

        aggregated_error_counts(m, place_count) = (num_spike_errors/nummeas)*100;
        errlegendtext(place_count) = sprintf("Spike errors for PARS locator #%d", locator_count);
        aggregated_error_counts(m, place_count + 1) = (num_multipath_errors/nummeas)*100;
        errlegendtext(place_count + 1) = sprintf("Multipath errors for PARS locator #%d", locator_count);
        place_count = place_count + 2;
        locator_count = locator_count + 1;
    end
end


save(sprintf("error_aggregate_after_%d_runs", M), "aggregated_error_counts", "errlegendtext","M")


%%


%{
f1=figure(41); clf; hold on;
colororder(colour_palette);
%bar(runs, aggregated_error_counts, "stacked"); 
bar(xvals, yvals, "stacked");
legend(errlegendtext, "NumColumns",2, "Location", "north");
ylim([0 4000])
%xticks(xvals); 
xlabel("Run")
ylabel("Error count");
exportgraphics(f1, "aggregated_errors_bar_chart.pdf",'ContentType','vector');

%}
%%
%{
f1=figure(99);
t = tiledlayout(4,1);
% Plot in tiles
nexttile, bar(xvals, yvals(:, 1), "FaceColor",colour_palette(1,:));
title(errlegendtext(1))
nexttile, bar(xvals, yvals(:, 2), "FaceColor",colour_palette(2,:));
title(errlegendtext(2))
nexttile, bar(xvals, yvals(:, 3), "FaceColor",colour_palette(3,:));
title(errlegendtext(3))
nexttile, bar(xvals, yvals(:, 4), "FaceColor",colour_palette(4,:));
title(errlegendtext(4))


% Specify common title, X and Y labels
xlabel(t, 'Run')
ylabel(t, '% of measurements affected by fault')
%title(t,sprintf("Measurements per run = %d",size(nonzeros(z_PARS(1,:))', 2)));
title(t,"Sensor faults overview")
exportgraphics(f1, "aggregated_errors_tiled.pdf",'ContentType','vector');
%%
%}


load("ins_mekf/error_aggregate_after_100_runs.mat");
runs = 1 : M;
errlegendtext = {
"Spike errors",...
"Multipath errors"};
xvals = 1 : M;
yvals = aggregated_error_counts;
f2=figure(98); clf; hold on;
tiled = tiledlayout(2,1,"TileSpacing","compact");
colororder(colour_palette)
%t = tiledlayout(4,1);
% Plot in tiles
nexttile; hold on; bar(xvals, yvals(:,1:2),"stacked"); ylabel("Locator 1", "Interpreter", "latex", "FontSize", 11)
legend(errlegendtext(1:2), "NumColumns",2, "Interpreter", "latex", "FontSize", 11)
ylim([0 36]); yticks(0:4:36);
nexttile; hold on; bar(xvals, yvals(:,3:4),"stacked");ylabel("Locator 2", "Interpreter", "latex", "FontSize", 11)
%legend(errlegendtext(3:4), "NumColumns",2, "Interpreter", "latex", "FontSize", 11)
ylim([0 36]); yticks(0:4:36);
ylabel(tiled,"\% of measurements affected", "Interpreter", "latex", "FontSize", 12)
xlabel(tiled, "Run", "Interpreter", "latex", "FontSize", 12)
exportgraphics(f2, "percentage_errors_tiled.pdf",'ContentType','vector');
%legend(errlegendtext(1:2));


%% Noise test

noise_process = n_PARS(:, z_PARS(1,:)~=0) + e_PARS(:, z_PARS(1,:)~=0);

normalrange1pdf = fitdist(noise_process(1,:)', "Normal")
normalazi1pdf = fitdist(noise_process(2,:)', "Normal")
normalele1pdf = fitdist(noise_process(3,:)', "Normal")

figure(22)
probplot( 'normal', [noise_process(1,:)'])
legend({'range','range samp.'},'location', 'best')
grid on;

figure(23)
probplot( 'normal', [noise_process(2,:)' noise_process(3,:)'])
legend({'azi', 'ele', 'azi samp.', 'ele samp.'},'location', 'best')
grid on;

figure(24);clf;
plot(normalrange1pdf);