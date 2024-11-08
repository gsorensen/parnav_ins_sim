function [ Phid, Qd ] = calc_Phi_and_Qd_using_van_Loan( F, G, Q, Ts, approx_order )
%CALC_PHId_AND_Qd_USING_VAN_LOAN Calulated the transition matirx PHId and the discrete
%time process noise covariance Qd such that the covariance can be
%propagated using P[k] = Phid[k]*P[k-1]*Phid'[k] + Qd[k] 

    dim_sys = size(F,1);
    A = [   -F              G*Q*G'; 
            zeros(dim_sys)  F';
        ]*Ts;
    
    if nargin < 5
        B       = expm( A ); 
    else
        B       = A^0;
        for k   = 1:approx_order
            B   = B + A^k/factorial(k);
        end
    end
      
    dim_A       = size(A,1);
    range_Phid  = dim_sys+1:dim_A;
    range_Qd    = 1:dim_sys;

    Phid        = B(     range_Phid,   range_Phid )';
    Qd          = Phid*B(  range_Qd,   range_Phid );
end