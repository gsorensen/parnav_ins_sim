% First order Gauss Markov process
function bt = fogm(b, h, sigma, tau, add_noise)
    if nargin == 4 
        add_noise = true;
    end

    bt = exp(-h/tau) * b;
    
    if add_noise
        st = s(h, sigma, tau);
        bt = bt + st *randn(size(b));
        %bt = bt + st * (sqrt(1/h)*randn(size(b)));
    end

    % For first order Gauss Markov sample generation
    function st = s(h, sigma, tau)
        st = sigma * (tau * (eye(3) - expm((-h/tau)*eye(3)))); 
    end
end