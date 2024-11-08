classdef IMU
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here

    properties
        Q_v
        Q_q

        Q_b_acc
        Q_b_ars

        T_acc
        T_ars
    end

    methods
        function obj = IMU(Q_v, Q_q, Q_b_acc, Q_b_ars, T_acc, T_ars)
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            obj.Q_v = Q_v;
            obj.Q_q = Q_q;
            obj.Q_b_acc = Q_b_acc;
            obj.Q_b_ars = Q_b_ars;
            obj.T_acc = T_acc;
            obj.T_ars = T_ars;
        end

        function [f_meas, w_meas] = generate_measurement(obj, f, w,...
                                                         b_f, b_w, Ts,...
                                                         add_noise, ...
                                                         add_bias)
            f_meas = f;
            w_meas = w;

            if add_noise
               f_meas = f_meas + sqrt(1/Ts) * obj.Q_v^(1/2) * randn(3,1);
               w_meas = w_meas + sqrt(1/Ts) * obj.Q_q^(1/2) * randn(3,1);
            end

            use_wiener = false;
            if add_bias
                if use_wiener
                    % Wiener processes
                    b_f = b_f + Ts*sqrt(1/Ts) * obj.Q_b_acc^(1/2) * randn(3,1);
                    b_w = b_w + Ts*sqrt(1/Ts) * obj.Q_b_ars^(1/2) * randn(3,1); 
                else
                    % GM processes 
                    Phi_f   = exp(-Ts/obj.T_acc);
                    G_f     = obj.T_acc*obj.Q_b_acc^(1/2)*( eye(3) - expm((-Ts/obj.T_acc))*eye(3) );
                    w_f     = sqrt(1/Ts)* randn(3,1);
                    b_fg    = Phi_f*b_f + G_f*w_f;
                     
                    Phi_w   = exp(-Ts/obj.T_ars);
                    G_w     = obj.T_ars*obj.Q_b_ars^(1/2)*( eye(3) - expm((-Ts/obj.T_ars))*eye(3) );
                    w_w     = sqrt(1/Ts)* randn(3,1);
                    b_wg    = Phi_w*b_w + G_w*w_w;
                end

                f_meas = f_meas + b_fg;
                w_meas = w_meas + b_wg;
            end
        end
    end
end