%% Loading simulation data

clear all;

Hz = 100;
Ts = 1 / Hz;

% Simulate otter
run_otter = true;

% Standstill 
standstill = ~run_otter * false;

% Simulation settings
add_noise = true;
add_bias = true;
aiding_Hz = 10;

% Aiding measurements
% Currently 
use_pars = true;
use_range_only = false;
use_angles_only = false;
locators_to_use = 4; % With angles, incremental

use_gnss = false;
use_gnss_vel = false;
use_gnss_for_stabilisation = true;

% Simulate sensor faults
simulate_sensor_faults = true;
reject_outliers = true;

% Don't reject outliers if we're not simulating sensor faults
reject_outliers = reject_outliers * simulate_sensor_faults;

% Initialisation settings
initialise_at_true_att = false;
initialise_at_true_pos = false;
initialise_at_true_vel = false;
initialise_at_zero_bias = false; 

M = 100;
for m = 1 : M
fprintf("Running MEKF for m = %d\n", m);
% Fetching simulation data for given parameters

if run_otter == true
input_file = sprintf("../data/otter_simulation_data_%02d_%dHz", m, Hz);
elseif standstill == true
input_file = sprintf("../data/standstill_simulation_data_%02d_%dHz", m, Hz);
else
input_file = sprintf("../data/simulation_data_%02d_%dHz", m, Hz);

end

if add_noise
    input_file = sprintf("%s_noisy", input_file);
end

if add_bias
    input_file = sprintf("%s_biased", input_file);
end
input_file = sprintf("%s_aided_at_%dHz", input_file, aiding_Hz);
fprintf("Loading simulation data from %s\n", input_file);
load(input_file);


%% Run simulation
if num_locators < locators_to_use
    locators_to_use = num_locators;
end


% Estimates for biases in body frame
b_acc_hat = zeros(3, N); 
b_ars_hat = zeros(3, N); 

% Estimates for rot matrix, euler angles
R_nb_hat    = zeros(3,3,N);
roll_hat    = zeros(1,N);
pitch_hat   = zeros(1,N);
yaw_hat     = zeros(1,N);

% INS initialisation
p_ins = zeros(3,N);
v_ins = zeros(3,N);
q_ins = zeros(4,N);

% MEKF parameters
delta_x         = zeros(15, N); 
P_pred_pos      = diag([1 1 1]).* A_pos^2;
P_pred_vel      = diag([1 1 1]).* A_vel^2;
P_pred_att      = diag([1 1 1]).* A_att^2;
P_pred_b_acc    = diag([1 1 1]).* A_b_acc^2;
P_pred_b_ars    = diag([1 1 1]).* A_b_ars^2;

if initialise_at_true_pos
    p_ins(:, 1) = pos(:,1);
else
    p_ins(:, 1) = first_pos;
end

if initialise_at_true_vel
    v_ins(:, 1) = vel(:,1);
else
    v_ins(:, 1) = first_vel;
end

if initialise_at_true_att
    R_nb_hat(:,:,1)     = R_nb(:,:,1);
    [roll_hat(1), pitch_hat(1), yaw_hat(1)] = R2euler(R_nb_hat(:,:,1));
    q_ins(:,1) = q_nb(:,1); 
else
    q_ins(:, 1) = first_att;
    %res = quat2eul(first_att'); 
    R_nb_hat(:, :, 1) = quat2rotm(first_att');
    [roll_hat(1), pitch_hat(1), yaw_hat(1)] =  R2euler(R_nb_hat(:,:,1));
end

if ~initialise_at_zero_bias
    b_acc_hat (:, 1) = zeros(3,1); 
    b_gyro_hat(:, 1) = zeros(3,1);
end

% Quaternion normalisation
q_ins(:, 1) = q_ins(:, 1) / norm(q_ins(:, 1));

% Set initial covariance estimate to first predicted covariance
P_pred = zeros(15, 15, N); P_pred(:, :, 1) = blkdiag(P_pred_pos, P_pred_vel, P_pred_att, P_pred_b_acc, P_pred_b_ars);
P_est = zeros(15, 15, N); P_est (:, :, 1) = P_pred(:, :, 1);

% Initialise the INS sim and MEKF
simulator = INSMEKFSimulator(Ts, Q_b_acc, T_acc, Q_b_ars, T_ars, g_b_i);
mekf = MEKF(Ts, Q_v, Q_q, Q_b_acc, Q_b_ars, T_acc, T_ars);

% For easier access
num_locators = size(locator_origins, 2);

% Initialise locators
for idx = 1 : num_locators
    locators(idx) = BLEPARS(sigma_rho, sigma_Psi, sigma_alpha,...
                            locator_origins(:,idx),...
                            locator_yaws(idx), ...
                            add_noise);
end

% Initialise GNSS sensor
gnss_sensor = GNSS(R_GNSS_pos, R_GNSS_vel, add_noise);

t = linspace(0, Ts*N, N);

% Do not use GNSS vel without GNSS pos
use_gnss_vel = use_gnss * use_gnss_vel;

% Do not use range only without PARS
use_range_only = use_range_only * use_pars;

% Use compass measurements
use_compass_heading_meas = false;

% Use virtual height measurement
use_virtual_heading_meas = false;


z_PARS_corrupted = z_PARS + simulate_sensor_faults * e_PARS;

rejected_flags = zeros(size(z_PARS_corrupted));

%z_PARS_corruped = z_PARS;
%
% Setting aiding level and corresponding dimensions for v and S
% This is controlled by the booleans above
% 4 - GNSS + full PARS
% 3 - PARS range only
% 2 - PARS range + bearing (TODO ONE LOCATOR ONLY)
% 1 - GNSS only
% 0 - Nothing
aiding = 0;
N_meas = floor(N/aiding_Hz);

if run_otter == true
    output_file = sprintf("../results/otter/mekf/%02d_%dHz_aided_at_%dHz_by", m, Hz, aiding_Hz);
    output_file = sprintf("/Volumes/T9/02/results/otter/mekf/%02d_%dHz_aided_at_%dHz_by", m, Hz, aiding_Hz);
elseif standstill == true
    output_file = sprintf("../results/standstill/mekf/%02d_%dHz_aided_at_%dHz_by", m, Hz, aiding_Hz);
else
    output_file = sprintf("../results/dummy/mekf/%02d_%dHz_aided_at_%dHz_by", m, Hz, aiding_Hz);
end

if use_gnss == false && use_pars == false
    output_file = sprintf("%s_nothing", output_file);
    aiding = 0;
elseif use_range_only == true
    aiding = 3;
    v = zeros(num_locators + use_compass_heading_meas, N_meas);
    S = zeros(num_locators + use_compass_heading_meas, num_locators + use_compass_heading_meas, N_meas);
    output_file = sprintf("%s_PARS_range_only", output_file);
elseif use_gnss == true
    aiding = 1;
    v = zeros(3 + 3 * use_gnss_vel + use_compass_heading_meas + use_virtual_heading_meas, N_meas);
    S = zeros(3 + 3 * use_gnss_vel + use_compass_heading_meas + use_virtual_heading_meas, ...
              3 + 3 * use_gnss_vel + use_compass_heading_meas + use_virtual_heading_meas, N_meas);

    if use_gnss_vel
        output_file = sprintf("%s_GNSS_pos_and_vel", output_file);
    else
        output_file = sprintf("%s_GNSS_pos", output_file);
    end
elseif use_angles_only == true
    % TODO Compass
    aiding = 5;
    v = zeros(2 * locators_to_use + use_compass_heading_meas + use_virtual_heading_meas, N_meas);
    S = zeros(2 * locators_to_use + use_compass_heading_meas + use_virtual_heading_meas, 2 * locators_to_use + use_compass_heading_meas + use_virtual_heading_meas, N_meas);
    output_file = sprintf("%s_PARS_full", output_file);
elseif use_pars == true
    % TODO Compass
    aiding = 2;
    v = zeros(3 * locators_to_use + use_compass_heading_meas + use_virtual_heading_meas, N_meas);
    S = zeros(3 * locators_to_use + use_compass_heading_meas + use_virtual_heading_meas, 3 * locators_to_use + use_compass_heading_meas + use_virtual_heading_meas, N_meas);
    output_file = sprintf("%s_PARS_full", output_file);
end

if reject_outliers == true
    output_file = sprintf("%s_outlier_rejection", output_file);
elseif simulate_sensor_faults == true
    output_file = sprintf("%s_sensor_fault", output_file);
end


counter = 1; % measurement counter
outlier_counter = 0;
removed_measurements_at = [];
normalised_innovations_all = zeros(size(v));

false_positives = zeros(size(z_PARS));
missed_detections = zeros(size(z_PARS));

for j = 2 : N
    % Debias IMU measurements
    [w_ins, f_ins] = simulator.debias_imu_input(omega_meas(:,j),...
                                                f_meas(:, j),...
                                                b_ars_hat(:, j - 1),...
                                                b_acc_hat(:, j - 1));

    % Propagate INS                               
    [p_ins(:,j), v_ins(:,j), q_ins(:, j), R_nb_hat(:,:,j), roll_hat(j),...
        pitch_hat(j), yaw_hat(j), b_ars_hat(:,j), b_acc_hat(:,j)] = ...
            simulator.ins_propagate(p_ins(:, j - 1), v_ins(:, j - 1), ...
                                    q_ins(:, j - 1), b_acc_hat(:, j - 1), ...
                                    b_ars_hat(:, j - 1), w_ins, ...
                                    f_ins);


    % Filter prediction
    P_pred(:, :, j) = mekf.predict(P_est(:, :, j - 1),...
                                   f_ins,...
                                   w_ins,... 
                                   R_nb_hat(:, :, j - 1));

    if (mod(j, 10) == 0 && aiding > 0)
        x_pred = [p_ins(:,j); v_ins(:,j); roll_hat(j); pitch_hat(j); yaw_hat(j); b_acc_hat(:,j); b_ars_hat(:,j)]; 
        
        active_aiding = aiding;
        if use_gnss_for_stabilisation && j <= 15000
            active_aiding = 1;
        end

        % Predict measurements
        switch active_aiding
            % GNSS
            case 1
                %disp("GNSS aiding");
                z = z_GNSS_pos(:, j);
                H = gnss_sensor.H_pos_k();
                h = H * x_pred;
                R = R_GNSS_pos;
                angular_mask = zeros(1, 3);

                zt = z_lever(:, j) / norm(z_lever(:,j));
                Ht = [zeros(3, 6) -R_nb_hat(:,:,j) * Smtrx([0 1 0]') zeros(3, 6)];
                ht = R_nb_hat(:,:,j) * [0 1 0]';
                Rt = R_GNSS_lev;
                angular_maskt = zeros(1, 3);

                z = [z; zt];
                H = [H; Ht];
                h = [h; ht];
                R = blkdiag(R, Rt);
                angular_mask = [angular_mask angular_maskt];

            % PARS Full
            case 2
                %disp("PARS aiding");
                z = z_PARS_corrupted(:, j);
                h = [];
                H = [];
                R = [];
                angular_mask = [];
    
                for idx = 1 : locators_to_use
                    h_idx = locators(idx).h(x_pred);
                    H_idx = locators(idx).H(x_pred);
                    
                    % Add to total vector of PARS measurements
                    h = [h; h_idx];
                    H = [H; H_idx];
                    R = blkdiag(R, R_PARS);

                    % Add to mask used for SSA sanity check
                    angular_mask = [angular_mask 0 1 1];
                end
            % PARS range only
            case 3
                z_full = z_PARS_corrupted(1:3*locators_to_use, j);
                z = [];
                h = [];
                H = [];
                R = [];

                for idx = 1 : locators_to_use
                    h_idx = locators(idx).h(x_pred);
                    H_idx = locators(idx).H(x_pred);
   
                    % Add to total vector of PARS measurements
                    z = [z; z_full(1 + 3 * (idx - 1))];
                    h = [h; h_idx(1)];
                    H = [H; H_idx(1,:)];
                    R = blkdiag(R, R_PARS(1,1));
                end

                angular_mask = zeros(1, locators_to_use);
            % GNSS + PARS
            case 4
                
            % PARS Angles only
            case 5
                z_full = z_PARS_corrupted(:, j);
                z = [];
                h = [];
                H = [];
                R = [];
                angular_mask = [];
    
                for idx = 1 : locators_to_use
                    z = [z; z_full(2 + 3 * (idx - 1) : 3 + 3 * (idx - 1) )];
                    h_idx = locators(idx).h(x_pred);
                    H_idx = locators(idx).H(x_pred);
                    
                    % Add to total vector of PARS measurements
                    h = [h; h_idx(2:3)];
                    H = [H; H_idx(2:3, :)];
                    R = blkdiag(R, R_PARS(2:3,2:3));

                    % Add to mask used for SSA sanity check
                    %angular_mask = [angular_mask 0 1 1];
                    angular_mask = [angular_mask 1 1];
                end
        end
        
        mead_indices = size(z, 1);
        xk = x_pred;
        hf = h;
        Hf = H;
        Pk = P_pred(:, :, j);

        for meas_idx = 1 : mead_indices
           zk = z(meas_idx);
           hk = h(meas_idx);
           Hk = H(meas_idx,:); 
           Rk = R(meas_idx, meas_idx);
           [vk, Sk] = mekf.compute_innovation_nl(hk, Pk, zk, Hk, Rk, angular_mask);
            
           % Somethign smarter could be done c.f. Gustavson. The best
           % information available should be used first
           should_add_measurement = true;
           if reject_outliers == true && active_aiding ~= 1
               Tk = vk^2/Sk;
               should_add_measurement = Tk <= 3.841;
               rejected_flags(meas_idx, j) = ~should_add_measurement;

               if ~should_add_measurement && error_flags(meas_idx, j) == 0
                   false_positives(meas_idx, j) = 1;
               elseif should_add_measurement && error_flags(meas_idx, j) ~= 0
                   missed_detections(meas_idx, j) = 1;
               end
           end

           if should_add_measurement == true
               Wk = mekf.compute_kalman_gain(Pk, Hk, Sk);
               [dxk, Pk] = mekf.compute_correction(Pk, vk, Hk, Rk, Wk);

               % INS reset
               [p_ins(:,j), v_ins(:,j), q_ins(:, j),b_acc_hat(:,j), b_ars_hat(:,j)]... 
                    = simulator.ins_reset(p_ins(:,j),...
                                          v_ins(:,j),...
                                          q_ins(:, j),...
                                          b_acc_hat(:,j),...
                                          b_ars_hat(:,j),...
                                          dxk);
                eul = quat2eul(q_ins(:, j)');
                yaw_hat(j) = eul(1); pitch_hat(j) = eul(2); roll_hat(j) = eul(3);
                xk = [p_ins(:,j); v_ins(:,j); roll_hat(j); pitch_hat(j); yaw_hat(j); b_acc_hat(:,j); b_ars_hat(:,j)]; 

                % Recompute predicted measurements based on accepted meas.
                locator_counter = 1;
                for idx = 1 : 3 : 3 * locators_to_use
                    hf(idx:idx+2) = locators(locator_counter).h(xk);
                    Hf(idx:idx+2, :) = locators(locator_counter).H(xk);
                    locator_counter = locator_counter + 1;
                end
           end
            
           delta_x(:, j) = delta_x(:, j) + dxk;

           % Currently v and S aren't cell arrays, so we can't store GNSS
           % first and then PARS later after the stabilisation period
           if aiding ~= 1
               v(meas_idx, counter) = vk;
               S(meas_idx, meas_idx, counter) = Sk;
           end
        end

        P_est(:,:,j) = Pk;
        counter = counter + 1;
    else
        P_est(:, :, j) = P_pred(:, :, j);
    end
end
%%

outlier_rejection_stats = zeros(size(z_PARS,1), 2);

for meas_idx = 1 : size(z_PARS, 1)
    outlier_rejection_stats(meas_idx, 1) = size(nonzeros(false_positives(meas_idx,:)),1); 
    outlier_rejection_stats(meas_idx, 2) = size(nonzeros(missed_detections(meas_idx,:)),1); 
end

% TODO SPECIFYT WHAT OTHERWISE FIGURES ARE ALSO SAVED??????
fprintf("Saving MEKF run results to %s\n", output_file);
save(output_file);

end