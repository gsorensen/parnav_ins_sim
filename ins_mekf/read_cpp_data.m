colour_palette = [
    hex2rgb("0072b2"); % blue
    hex2rgb("e69f00"); % orange
    hex2rgb("009e73"); % blue green
    hex2rgb("cc79a7"); % pale violet 
    hex2rgb("56b4e9"); % sky blue
    hex2rgb("d55e00"); % vermillion
    hex2rgb("f0e442"); % yellow
    hex2rgb("000000"); % black
];


% Specify the filename of your CSV file
filename = 'cpp_test_data.csv';

% Read the entire CSV file
data = readmatrix(filename);

% Extract columns
t = data(:, 1); % Timestep

% Attitude error and covariance
att_err_x = data(:, 2);
att_err_y = data(:, 3);
att_err_z = data(:, 4);
att_cov_x = data(:, 5);
att_cov_y = data(:, 6);
att_cov_z = data(:, 7);

% Position error and covariance
pos_err_x = data(:, 8);
pos_err_y = data(:, 9);
pos_err_z = data(:, 10);
pos_cov_x = data(:, 11);
pos_cov_y = data(:, 12);
pos_cov_z = data(:, 13);

% Velocity error and covariance
vel_err_x = data(:, 14);
vel_err_y = data(:, 15);
vel_err_z = data(:, 16);
vel_cov_x = data(:, 17);
vel_cov_y = data(:, 18);
vel_cov_z = data(:, 19);

% Accelerometer bias error and covariance
acc_bias_err_x = data(:, 20);
acc_bias_err_y = data(:, 21);
acc_bias_err_z = data(:, 22);
acc_bias_cov_x = data(:, 23);
acc_bias_cov_y = data(:, 24);
acc_bias_cov_z = data(:, 25);

% Gyro bias error and covariance
gyro_bias_err_x = data(:, 26);
gyro_bias_err_y = data(:, 27);
gyro_bias_err_z = data(:, 28);
gyro_bias_cov_x = data(:, 29);
gyro_bias_cov_y = data(:, 30);
gyro_bias_cov_z = data(:, 31);

% Combine error terms into matrices
pos_err = [pos_err_x'; pos_err_y'; pos_err_z'];
att_err = [att_err_x'; att_err_y'; att_err_z'];
vel_err = [vel_err_x'; vel_err_y'; vel_err_z'];
acc_bias_err = [acc_bias_err_x'; acc_bias_err_y'; acc_bias_err_z'];
gyro_bias_err = [gyro_bias_err_x'; gyro_bias_err_y'; gyro_bias_err_z'];

% Combine covariance terms into matrices
pos_cov = [pos_cov_x'; pos_cov_y'; pos_cov_z'];
att_cov = [att_cov_x'; att_cov_y'; att_cov_z'];
vel_cov = [vel_cov_x'; vel_cov_y'; vel_cov_z'];
acc_bias_cov = [acc_bias_cov_x'; acc_bias_cov_y'; acc_bias_cov_z'];
gyro_bias_cov = [gyro_bias_cov_x'; gyro_bias_cov_y'; gyro_bias_cov_z'];

N = size(err, 2);

start_at_idx = 1;
end_at_idx = t(N);

plot_err_with_3sig_bounds(t, att_err .* (180/pi), att_cov .* (180/pi)^2, N, 1233, "Att error cpp", colour_palette, [start_at_idx, end_at_idx]);
plot_err_with_3sig_bounds(t, pos_err, pos_cov, N, 1234, "Position error cpp", colour_palette, [start_at_idx, end_at_idx]);
plot_err_with_3sig_bounds(t, vel_err, vel_cov, N, 1235, "Velocity error cpp", colour_palette, [start_at_idx, end_at_idx]);
plot_err_with_3sig_bounds(t, acc_bias_err, acc_bias_cov, N, 1236, "Acc bias error cpp", colour_palette, [start_at_idx, end_at_idx]);
plot_err_with_3sig_bounds(t, gyro_bias_err .* (180/pi), gyro_bias_cov .* (180/pi)^2, N, 1237, "Gyro bias error cpp", colour_palette, [start_at_idx, end_at_idx]);


function plot_err_with_3sig_bounds(t, x, P, N, fig_num, title, colours, xlims)
    P_px = reshape(P(1, :), [1, N]);
    P_py = reshape(P(2, :), [1, N]);
    P_pz = reshape(P(3, :), [1, N]);
    figure(fig_num); 
    %if fig_num ~= 4
    clf(fig_num); 
    %end
    x_ins = zeros(size(x));
    subplot(3,1,1); hold on;
    error = plot(t, x(1, :) - x_ins(1,:));
    upper = plot(t, 3 * sqrt(P_px(:)), "--");
    lower = plot(t, -3 * sqrt(P_px(:)), "--");
    xlim(xlims);
    legend(["Error" "3o-bounds"])
    lower.SeriesIndex = upper.SeriesIndex;
    subplot(3,1,2); hold on;
    error = plot(t, x(2, :) - x_ins(2,:));
    upper = plot(t, 3 * sqrt(P_py(:)), "--");
    lower = plot(t, -3 * sqrt(P_py(:)), "--");
    xlim(xlims);
    lower.SeriesIndex = upper.SeriesIndex;
    subplot(3,1,3); hold on;
    error = plot(t, x(3, :) - x_ins(3,:));
    upper = plot(t, 3 * sqrt(P_pz(:)), "--");
    lower = plot(t, -3 * sqrt(P_pz(:)), "--");
    xlim(xlims);
    lower.SeriesIndex = upper.SeriesIndex;
    sgtitle(title);
    colororder(colours);
end