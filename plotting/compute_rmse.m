function RMSE = compute_rmse(xtrue, xest)
    N = size(xtrue, 2);
    D = size(xtrue, 1);
    start_idx = 25000;
    start_idx = 1;
    RMSE = zeros(D, 1);
    MAE = zeros(D, 1);
    
    if nargin == 2
    err = xtrue - xest;
    else
    err = xtrue;
    end

    for d = 1 : D
        RMSE(d) = sqrt((1/N) * sum(err(d,start_idx:N)*err(d,start_idx:N)'));
        MAE(d) = (1/N) * sum(abs(err(d,:)));
    end

    RMSE(7:9) = RMSE(7:9) .* (180/pi);
    RMSE(13:15) = RMSE(13:15) .* (180/pi);

    MAE(7:9) = MAE(7:9) .* (180/pi);
    MAE(13:15) = MAE(13:15) .* (180/pi);

    M = eye(15);
    M(2,:) = M(1,:)';
    M(1,:) = [0 1 0 0 0 0 0 0 0 0 0 0 0 0 0];
    
    RMSE = M * RMSE;

    %RMSE = MAE;
end

