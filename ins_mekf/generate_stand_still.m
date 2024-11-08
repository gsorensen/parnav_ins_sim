clear all

N = 10000;
Hz = 100;

% Position, velocity and acceleration
pos = zeros(3, 10000);
vel = zeros(3, 10000);
acc = zeros(3, 10000);

% Attitude
roll            = zeros(1, N);
pitch           = zeros(1, N);
yaw_amp         = deg2rad(25);
yaw             = yaw_amp * ones(1, N);

file_name = sprintf("../data/%s_%dHz", "standstill_PVA_data", Hz)

save(file_name,'pos','vel','acc', 'roll', 'pitch','yaw','N', 'Hz')
