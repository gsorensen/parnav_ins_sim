function [t, pos_est, vel_est,...
    att_est, acc_bias_est, gyro_bias_est, P_est]...
        = parse_cpp_data_file(cpp_file, simulation_case)

data = readmatrix(cpp_file);

t = data(:, 1)';
N = size(t, 2);
pos_est = data(:, 2:4)';
vel_est = data(:, 5:7)';
att_est = data(:, 8:10)';
acc_bias_est = data(:, 11:13)';
gyro_bias_est = data(:, 14:16)';
P_est = reshape(data(:, 17:241)', [15 15 N]);

O3 = zeros(3,3); I3 = eye(3);
Ms = [
  O3 I3 O3 O3 O3;
  O3 O3 I3 O3 O3;
  I3 O3 O3 O3 O3;
  O3 O3 O3 I3 O3;
  O3 O3 O3 O3 I3;
];

% Similarity transform since FGO has attitude first in covariance matrix
for i = 1 : N
    P_est(:, :, i) = Ms * P_est(:, :, i) * Ms';
end

if simulation_case == 1
% TODO PARSE Innovation when that is back in
end

end