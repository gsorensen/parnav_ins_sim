classdef BLEPARS
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here

    properties
        R % rho, Psi, alpha
        origin % Origin of the radio frame given in the navigation frame
        yaw_r % Yaw angle with respect to navigation frame
        Tn_r % Homogenous transformation matrix from {n} to {r}
        Tr_n % Homogenous transformation matrix from {r} to {n}
        add_noise
    end

    methods
        function obj = BLEPARS(sigma_rho, sigma_Psi, sigma_alpha, origin, yaw, add_noise)
            obj.R = diag([sigma_rho, sigma_Psi, sigma_alpha]).^2;
            obj.yaw_r = yaw;
            obj.origin = origin;

            Rmat = Rzyx(0, 0, yaw);
            u = -origin;
            
            obj.Tn_r = [Rmat u; 0 0 0 1];
            obj.Tr_n = [Rmat' -Rmat'*u; 0 0 0 1];

            if nargin == 5
                obj.add_noise = true;

            else
                obj.add_noise = add_noise;
            end
        end

        % NOTE: The literature used (i.e. Kristoffer's doctoral thesis)
        % represents radio mneasurements in the radio frame, I don't here
        % (similarly to what I did during my MSc), but I guess that is
        % would be very beneficial to instead handle this with a
        % radio-frame and a mapping/transformation to NED?
        %
        % I.e., each BLEPARS has its own frame and is at the origin. How to
        % do?
        %{
        function [is_valid, z, H, R] = generate_measurement_for_locators(obj, x, Ts)
            z = [];
            %zpredpars = [];
            H = [];
            R = [];
            is_valid = true;
        
            for j = 1 : size(obj.locators, 2)
                [is_measurement_valid, zi, Hi, Ri] = ...
                    obj.generate_measurement(x, obj.locators(:,j), Ts);
        
               % [is_measurement_valid, zpredi] = ...
                %    ble.predict_measurement(x_pred, locators_pos(:,j));
        
                if is_measurement_valid
                %if is_prediction_valid && is_measurement_valid
                    z = [z; zi];
                    H = [H; Hi];
                    R = blkdiag(R, Ri);
                    %zpredpars = [zpredpars; zpredi];
                    obj.num_measurements_taken = ...
                        obj.num_measurements_taken + 1;
                else
                    obj.num_measurements_failed = ...
                        obj.num_measurements_failed + 1;
                    is_valid = false;
                end
            end
        end
        %}
        function h_pred = predict_measurement_for_locators(obj, x)
            h_pred = [];

            %{
            for j = 1 : size(obj.locators, 2)
                [is_valid, h] = ...
                    obj.predict_measurement(x, obj.locators(:,j));

                if is_valid
                    h_pred = [h_pred; h];
                end
            end
            %}
        end

        function [is_valid, z_pred] = predict_measurement(obj, x_pred)
            z_pred = obj.h(x_pred(1:3));
            is_valid = isreal(z_pred);
        end

        function [is_valid, z, H, R] = generate_measurement(obj, x)
            z = obj.h(x(1:3)) + obj.R^(1/2) * randn(3,1);
            H = obj.H(x(1:3), size(x, 1));
            R = obj.R;
            is_valid = isreal(z) && isreal(H);

            if ~obj.add_noise
                z = obj.h(x(1:3));
            end
        end

        % Maps to navigation frame cartesian
        function p = map_to_cartesian_r(obj, rho, alpha, Psi)
            % Bias terms used when mapping to Cartesian
            var_Psi = obj.R(2,2);
            var_alpha = obj.R(3,3);
            b_Psi = exp(-var_Psi/2);
            b_alpha = exp(-var_alpha/2);

            % Eq. 14 in Okuhara et al 2021 ICUAS
            p = [
                (1/b_Psi)*(1/b_alpha)*rho*cos(Psi)*cos(alpha);
                (1/b_Psi)*(1/b_alpha)*rho*sin(Psi)*cos(alpha);
                -(1/b_alpha)*rho*sin(alpha)
            ];
        end
       
        function p = map_to_cartesian_n(obj, rho, alpha, Psi)
            p_r = obj.map_to_cartesian_r(rho, alpha, Psi);
            p_ = obj.Tr_n * [p_r; 1];
            p = p_(1:3);
        end


        % Helper, I just need an easy way to map to a given radio frame
        function p = map_to_radio_frame(obj, p_n)
            p_ = obj.Tn_r * [p_n; 1];
            p = p_(1:3);
        end

%    end

 %   methods(Static)
        function z = h(obj, x)
            % x user position in nav frame
            p_ = obj.Tn_r * [x(1:3); 1];
            p =  p_(1:3);
            px = p(1); py = p(2); pz = p(3);
            po = zeros(3,1); % TODO REWRITE EQUATIONS INSTEAD
            pox = po(1); poy = po(2); poz = po(3);
            
            rho = norm(p-po);
            rho_hor = norm(p(1:2)-po(1:2));
            Psi = atan2((py - poy),(px - pox)); 
            alpha = atan2(-(pz - poz), rho_hor);
            alpha_test = asin( -(pz-poz)/rho );

            if (abs(alpha - alpha_test) > 1e-6)
                disp("asin not equal to atan");
            end

            z = [rho; alpha; Psi];
        end

        function J = H(obj, x, dimx)
            if nargin ~= 3
                dimx = 15;
            end
            p_ = obj.Tn_r * [x(1:3); 1];
            p =  p_(1:3);
            px = p(1); py = p(2); pz = p(3);
            po = zeros(3,1); % TODO REWRITE EQUATIONS INSTEAD
            pox = po(1); poy = po(2); poz = po(3);

   
            H_rho = [-(2*pox - 2*px)/(2*((pox - px)^2 + (poy - py)^2 + (poz - pz)^2)^(1/2)), ...
                     -(2*poy - 2*py)/(2*((pox - px)^2 + (poy - py)^2 + (poz - pz)^2)^(1/2)), ...
                     -(2*poz - 2*pz)/(2*((pox - px)^2 + (poy - py)^2 + (poz - pz)^2)^(1/2)), ...
                      zeros(1, dimx - 3)];

            H_alpha = [((poz - pz)*(2*pox - 2*px))/(2*((pox - px)^2 + (poy - py)^2)^(3/2)*((poz - pz)^2/((pox - px)^2 + (poy - py)^2) + 1)),...
                       ((poz - pz)*(2*poy - 2*py))/(2*((pox - px)^2 + (poy - py)^2)^(3/2)*((poz - pz)^2/((pox - px)^2 + (poy - py)^2) + 1)),...
                        -1/(((pox - px)^2 + (poy - py)^2)^(1/2)*((poz - pz)^2/((pox - px)^2 + (poy - py)^2) + 1)),...
                        zeros(1, dimx - 3)];
            H_Psi = [(poy - py)/((pox - px)^2*((poy - py)^2/(pox - px)^2 + 1)),...
                     -1/((pox - px)*((poy - py)^2/(pox - px)^2 + 1)),...
                      0, zeros(1, dimx - 3)];
      
            J = [H_rho; H_alpha; H_Psi];
        end

    end
end