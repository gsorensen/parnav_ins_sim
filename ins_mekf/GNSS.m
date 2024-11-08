classdef GNSS
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here

    properties
        R_pos
        R_vel
        H_pos
        H_vel
        add_noise
    end

    methods
        function obj = GNSS(R_pos, R_vel, add_noise)
            obj.R_pos = R_pos;
            obj.R_vel = R_vel;
            obj.H_pos = [eye(3) zeros(3, 12)];
            obj.H_vel = [zeros(3,3) eye(3) zeros(3, 9)];
            obj.add_noise = add_noise;
        end

        function [z, H, R] = generate_pos_measurement(obj, p, Ts, Rmat, lever_arm)
            H = obj.H_pos;
            R = obj.R_pos;

            if nargin > 3
                p = p + Rmat * lever_arm;
            end

            z = p + R.^(1/2) * randn(3,1);

            % More likely than not, noise will be used
            if ~obj.add_noise
                z = p;
            end
        end

        function [z, H, R] = generate_vel_measurement(obj, v, Ts)
            H = obj.H_vel;
            R = obj.R_vel;
            z = v + R.^(1/2) * randn(3,1);

            % More likely than not, noise will be used
            if ~obj.add_noise
                z = v;
            end
        end

        function H = H_pos_k(obj)
            H = obj.H_pos;
        end

        function H = H_vel_k(obj)
            H = obj.H_vel;
        end
    end
end