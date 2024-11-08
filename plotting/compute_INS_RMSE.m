function [pos_rmse,vel_rmse,att_rmse,acc_rmse,gyro_rmse] = ...
    compute_INS_RMSE(xtrue, xest)
%COMPUTE_RMSE 
%   xtrue, xest 15 x N matrices [pos, vel, att, acc, gyro]
    pos_rmse = rmse(xtrue(1:3),   xest(1:3), 2);
    vel_rmse = rmse(xtrue(4:6),   xest(4:6), 2);
    att_rmse = rmse(xtrue(7:9),   xest(7:9), 2);
    acc_rmse = rmse(xtrue(10:12), xest(10:12), 2);
    gyro_rmse = rmse(xtrue(13:15),xest(13:15), 2);
end