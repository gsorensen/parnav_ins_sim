classdef MEKF
    %MEKF Summary of this class goes here
    %   Detailed explanation goes here

    properties
        Ts
        Q
        T_b_acc
        T_b_ars
        I15
    end

    methods
        function obj = MEKF(Ts, Q_v, Q_q, Q_b_acc, ...
                            Q_b_ars, T_b_acc, T_b_ars)
            obj.Ts = Ts;
            obj.Q = blkdiag(Q_v, Q_q, Q_b_acc, Q_b_ars);
            obj.T_b_acc = T_b_acc;
            obj.T_b_ars = T_b_ars;
            obj.I15 = eye(15);
        end

        function [Fd, Qd, F, G] = evaluate_f(obj, w, f, R)
            I3 = eye(3);
            O3 = zeros(3,3);

            % TODO Consider where to put these
            F_ = @(h, f, w, R) [....
                    O3 I3 O3    O3  O3;
                    O3 O3 -R*vec2skew(f)    -R  O3;
                    O3 O3 -vec2skew(w)      O3  -I3;
                    O3 O3 O3                -(1/obj.T_b_acc)*I3  O3;
                    O3 O3 O3                O3  -(1/obj.T_b_ars) * I3;];

            % hvis en IMU har 3x helt like sensor element så er F_i like grei å få for
            % pga en isotropisk støy, men hvis man f.eks. har noe ulik z-acc (som er
            % vanlig maritimt) så er det likegreit å bruke F_i som man får direkte fra
            % linearisieringen
            F_i = @(R, T_acc, T_ars) [O3 O3 O3 O3;
                   -R O3 O3 O3;
                   O3 -I3 O3 O3;
                   O3 O3 I3 O3;
                   O3 O3 O3 I3];

            F = F_(obj.Ts, f, w, R);
            G = F_i(R, obj.T_b_acc, obj.T_b_ars);

            % Might have to add to path
            [Fd, Qd] = calc_Phi_and_Qd_using_van_Loan( F, G, obj.Q, obj.Ts, 2);
        end

        function [P_pred_k] = predict(obj, P_est_k_1, f, w, R)
            % Obtain discrete-time system model and process noise
            % covariance
            [Fx, Qd] = obj.evaluate_f(w, f, R);

            P_pred_k = Fx * P_est_k_1 * Fx' + Qd;

            % Handle numerical errors, ensure P > 0
            P_pred_k = (P_pred_k + P_pred_k')/2;
        end

        function [v, S] = compute_innovation(~, x_pred, P_pred, z, H, R)
            % Innovation
            % QUESTION: Is it necessary here to have the nonlinear h(...)
            % model that exists for z_pars = [rho, Psi, alpha]?
            % See function below
            v = z - H * x_pred;

            % Innovation covariance
            S = H * P_pred * H' + R;
        end

        function [v, S] = compute_innovation_nl(~,h,P_pred, z, H, R, angular_mask)
            v = z - h;

            if nargin == 7
                for idx = 1 : size(v,1)
                    if angular_mask(idx) == 1
                        v(idx) = ssa(z(idx)) - h(idx);
                        if (abs(v(idx) - ssa(v(idx))) > 1e-3)
                         %   fprintf("v={%.2f} ssav={%.2f}\n", v(idx), ssa(v(idx)));
                            v(idx) = ssa(v(idx));
                        end
                    end
                end
            end
            % Innovation covariance
            S = H * P_pred * H' + R;
        end

        function W = compute_kalman_gain(~, P_pred, H, S)
            W = P_pred * H' / S;
        end

        function [dx, P] = compute_correction(obj, P_pred, v, H, R, W)
            dx = W * v;
            I15WH = obj.I15 - W*H;
            P = I15WH * P_pred * I15WH' + W * R * W';
        end

        function [dx, P_est, v, S] = correct(obj, x, P_pred, z, H, R)
            [v, S] = obj.compute_innovation(x, P_pred, z, H, R);

            W = obj.compute_kalman_gain(P_pred, H, S);

            [dx, P_est] = obj.compute_correction(P_pred, v, H, R, W);
        end
    end
end
