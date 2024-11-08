% Generate orbit position
%{
r       = 500;
w_hor   = 0.4;
w_ver   = 0.1;
syms px(t) py(t) pz(t)

px(t) = r*cos(w_hor*t)*cos(w_ver*t); 
py(t) = r*sin(w_hor*t)*cos(w_ver*t);
pz(t) = -r*sin(w_ver*t);

vx(t) = diff( px, t);
vy(t) = diff( py, t);
vz(t) = diff( pz, t); 

ax(t) = diff( vx, t);
ay(t) = diff( vy, t);
az(t) = diff( vz, t); 
%}
%Hz = 100;
%Ts = 1/Hz;
clear all;
 
load("../data/otter_test_data.mat")

N = N + 1;
t = t';
pos = eta(:, 1:3)';
vel = zeros(3,N);
for idx = 1:N
    R_nb = Rzyx( ssa(eta(idx, 4)),ssa(eta(idx, 5)), ssa(eta(idx, 6)));
    vel(:, idx) = R_nb*nu(idx, 1:3)';
end
% vel = nu(:, 1:3)';
roll = eta(:, 4)';
pitch = eta(:, 5)';
yaw = (eta(:, 6))';
omega = nu(:, 4:6)';
acc = zeros(size(vel)); % We don't actually need this except for the first element which we keep zero
Hz = 1/h;
figure(77);
plot(pos(1,:), pos(2,:))

file_name = sprintf("../data/otter_%s_%dHz", "PVA_data", Hz)

figure(999);
subplot(2,1,1)
plot( pos(2, :), pos(1, :) );
xlabel('East [m]')
ylabel('North [m]')
grid on;

subplot(2,1,2)
plot(t, -pos(3, :));
ylabel('Up [m]')

grid on;

save(file_name,'pos','vel','acc', 'roll', 'pitch','yaw','N', 'Hz')
%{

% Time vector and constants
time_hor    = 60*5;% time horizon is seconds
time        = 0:Ts:time_hor;
N           = length(time);

% Position, velocity and acceleration
pos = double( [px(time); py(time); pz(time)] );  
vel = double( [vx(time); vy(time); vz(time)] ); 
acc = double( [ax(time); ay(time); az(time)] ); 

% Attitude
roll_pitch_amp  = deg2rad(5);
roll_pitch_w    = 0.1;
roll            = roll_pitch_amp * cos(roll_pitch_w * time);
pitch           = roll_pitch_amp * sin(roll_pitch_w * time);
yaw_amp         = deg2rad(25);
yaw_w           = 0.01;
yaw             = yaw_amp * cos(yaw_w * time);

%%
f=figure(1); clf; grid on
plot3(pos(1,:), pos(2,:), pos(3,:));
title("Agent trajectory visualiation"); grid on;
xlabel("East [m]"); ylabel("North [m]"); zlabel("Down [m]");
exportgraphics(f,"Trajectory.pdf",'ContentType','vector')
%%
file_name = sprintf("../data/%s_%dHz", "PVA_data", Hz)

save(file_name,'pos','vel','acc', 'roll', 'pitch','yaw','N', 'Hz')
%}