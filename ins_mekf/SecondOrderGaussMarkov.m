classdef SecondOrderGaussMarkov < handle
    properties
        dt
        xi
        w_d
        w_n
        q
        x
        A 
        B = [0; 1]
        S
        Phi
        P
    end
    
    methods
        function obj = SecondOrderGaussMarkov(dt, xi, w_n, q, tau, P0, N)
            obj.dt = dt;
            obj.xi = xi;
            obj.w_n = w_n;
            obj.w_d = w_n * sqrt(1 - xi^2);
       
            obj.q = q;
            obj.x = zeros(2, N);
            obj.A = [0 1; -obj.w_n^2 -2*xi*obj.w_n];
            obj.S = @(t) (q*tau/2) .* [
                tau^2 * ((1-exp(-2*t/tau)) -4*(1 - exp(-t/tau)) + 2*t/tau), tau * (1 - exp(-t/tau))^2;
                tau * (1 - exp(-t/tau))^2, (1 - exp(-2*t/tau))^2
            ]; 

            % Initialise constants
            obj.x(:, 1) = q .* randn(2,1);
            
            % Initialise covariance
            obj.P = zeros(2, 2, N);
            obj.P(:,:, 1) = P0;

            % Initialise transition matrix
            w_d = obj.w_d;

            if xi < 1
                phi_scale = @(t) exp(-xi * w_n * t) / w_d;
                phi11 = @(t) w_d * cos(w_d * t) + xi * w_n * sin(w_d * t);
                phi12 = @(t) sin(w_d * t);
                phi21 = @(t) -w_n^2*sin(w_d * t);
                phi22 = @(t)  w_d * cos(w_d * t) - xi * w_n * sin(w_d * t);
            elseif xi == 1
                phi_scale = @(t) 1; 
                phi11 = @(t) exp(-w_n * t) * (1 + w_n * t);
                phi12 = @(t) t * exp(-w_n * t);
                phi21 = @(t) -w_n^2 * t * exp(-w_n * t);
                phi22 = @(t) exp(-w_n * t) * (1 - w_n * t);
            else 
                phi_scale = @(t) exp(-xi * w_n * t) / w_d;
                phi11 = @(t) w_d * cosh(w_d * t) + xi * w_n * sinh(w_d * t);
                phi12 = @(t) sinh(w_d * t);
                phi21 = @(t) -w_n^2*sinh(w_d * t);
                phi22 = @(t)  w_d * cosh(w_d * t) - xi * w_n * sinh(w_d * t);
            end
            obj.Phi = @(t) phi_scale(t) * [phi11(t) phi12(t); phi21(t) phi22(t)];
        end

        function propagate(obj, idx)
            S_k = obj.S(obj.dt);
            Phi_k = obj.Phi(obj.dt);
            obj.P(:, :, idx) = Phi_k * obj.P(:, :, idx - 1)  * Phi_k' + S_k;
            obj.x(:, idx) = Phi_k * obj.x(:, idx - 1) + S_k .^ (1/2) * randn(2,1);
        end
    end
end

