classdef INSMEKFSimulator
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here

    properties
        % Time step
        Ts

        % Bias parameters
        Q_b_acc
        Q_b_ars
        T_b_acc
        T_b_ars

        g
    end

    methods
        function obj = INSMEKFSimulator(Ts, Q_b_acc, T_b_acc, Q_b_ars, T_b_ars, g)
            obj.Ts = Ts;
            obj.Q_b_acc = Q_b_acc;
            obj.Q_b_ars = Q_b_ars;
            obj.T_b_acc = T_b_acc;
            obj.T_b_ars = T_b_ars;
            obj.g = g;
        end

        function [w_ins, f_ins] = debias_imu_input(~, w, f, b_w_hat, b_f_hat)
            w_ins = w - b_w_hat;
            f_ins = f - b_f_hat;
        end

        % Requires debiased IMU input
        function [p_k, v_k, q_k, R_k,...
                  roll_k, pitch_k, yaw_k,...
                  b_w_k, b_f_k] = ins_propagate(obj, p, v, q, b_f, b_w, w, f)
            % Propagate INS attitude
            [q_k, R_k, roll_k, pitch_k, yaw_k] = ...
                obj.ins_propagate_attitude(q, w);

            % Propagate INS velocity
            v_k = obj.ins_propagate_velocity(v, f, R_k);

            % Propagate INS position
            p_k = obj.ins_propagate_position(p, v, v_k);

            % Propagate INS biases
            [b_w_k, b_f_k] = obj.ins_propagate_bias(b_w, b_f);
        end

        function [p, v, q, b_f, b_w] = ins_reset(~, p, v, q, b_f, b_w, dx)
            p = p + dx(1:3);
            v = v + dx(4:6);
            b_f = b_f + dx(10:12);
            b_w = b_w + dx(13:15);
          
            a_g = dx(7:9);
            dq = (1 / sqrt (4 + a_g' * a_g)) * [2; a_g];
            q = quatMulti(q, dq);
            q = q / norm(q);
        end

        function [q_k, R_k, roll, pitch, yaw] = ins_propagate_attitude(obj, q_k_1, w)
            % jeg flyttet INS propagering før forreksjonen mtp. indeksingene du har
            % valgt så blir dette rettest mtp sanne sensorer. Viktig at tilstanden har rett indeks for
            % korrigering (du håndterte riktignok dette rett med at du brukte
            % indeks k-1 i korrigeringen)
            % INS propagation
            % q_ins(:, k) = (eye(4) + 0.5 * Ts * Omega(:, :, k )) * q_ins(:, k - 1); 
            % forward Euler er ikke spesielt bra. Like greit å bruke kvaterione
            % produkt. Man kunne også gjort det samme med rotasjonsmatrise ved å
            % bruke Rodrigues formelen i Groves Ch 5 (5.73)

            % Convert omega to alpha
            alpha = w * obj.Ts;
            alpha_norm = norm(alpha);

            % Compute increment
            u = alpha / alpha_norm;
            if alpha_norm > 1e-8
                d_q = [cos(alpha_norm/2); sin(alpha_norm/2) * u];
            else
                d_q = [1; alpha/2 ]; % Groves Tab. E.1, Alg. Order 1
            end
            
            % Propagate quaternion attitude
            q_k = quatMulti(q_k_1, d_q);

            % Normalisation
            q_k = q_k / norm(q_k);

            % Obtain rotmat attitude rep from quaternion
            R_k = quat2rotMat_fast(q_k);

            % Obtain Euler angles from rotmat
            [roll, pitch, yaw] = R2euler(R_k);
        end

        function [b_w_k, b_f_k] = ins_propagate_bias(obj, b_w_k_1, b_f_k_1)
            b_w_k = fogm(b_w_k_1, obj.Ts, obj.Q_b_ars^(1/2), obj.T_b_ars, false);
            b_f_k = fogm(b_f_k_1, obj.Ts, obj.Q_b_acc^(1/2), obj.T_b_acc, false);
        end

        function v_k = ins_propagate_velocity(obj, v_k_1, f, R)
            v_k = v_k_1 + obj.Ts * (R * f + obj.g);
        end

        function p_k = ins_propagate_position(obj, p_k_1, v_k_1, v_k)
            p_k = p_k_1 + (obj.Ts / 2) * (v_k + v_k_1);
        end
    end
end