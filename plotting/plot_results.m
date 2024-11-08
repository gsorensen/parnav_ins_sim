clear all; clc; %close all;
colour_palette = [
    hex2rgb("009e73"); % blue green
    hex2rgb("cc79a7"); % pale violet 
    hex2rgb("0072b2"); % blue
    hex2rgb("e69f00"); % orange
    hex2rgb("56b4e9"); % sky blue
    hex2rgb("d55e00"); % vermillion
    hex2rgb("f0e442"); % yellow
    hex2rgb("000000"); % black
];

colororder(colour_palette);

simulation_case = 2;
switch simulation_case
    case 0
        title_info = "GNSS";
    case 1
        title_info = "PARS range only";
    case 2
        title_info = "Full PARS";
end

run_otter = true;
run_standstill = ~run_otter * false;


% Choose what to do
ee3sigmabounds = true;
ee3sigmabounds_sensor_fault = true;
ee3sigmabounds_outlier_rejection = true;
ee3sigmabounds_huber = true;
ee3sigmabounds_tukey = true;
dop_plot = true;

cca_analysis = false;
save_cca_to_file = false;
filter_consistency = false;
do_compute_rmse = true;

Mt = 100;

fetch_data = false;

%vals = randi(100,10,1);
%%
if dop_plot == true
    locator_origins = [
        [190 470 -12];
        [-40 20 -12];
    ]';
    locator_yaws = [
        0;
        0;
    ] * (pi/180);
    
    sigma_rho   = 5;
    sigma_Psi   = 7 * deg2rad(1) ;
    sigma_alpha = 7 * deg2rad(1) ;
    
    for idx = 1 : size(locator_origins, 2)
        locators(idx) = BLEPARS(sigma_rho, sigma_Psi, sigma_alpha,...
                                locator_origins(:,idx),...
                                locator_yaws(idx), ...
                                true);
    end
    
    truestate = load(sprintf("../data/otter_PVA_data_%dHz.mat", 100));
    NEdop = zeros(1,truestate.N);
    Ddop = zeros(1,truestate.N);
    t = linspace(0, truestate.N/truestate.Hz, truestate.N);
    
    for k = 1 : truestate.N 
        if k > 15000
        H1 = locators(1).H(truestate.pos(:,k),3);
        H2 = locators(2).H(truestate.pos(:,k),3);
        H = [H1; H2];
        else
        H = eye(3);
        end
        Q = inv(H'*H);
        NEdop(k) = sqrt(trace(Q(1:2,1:2)));
        Ddop(k) = sqrt(trace(Q(3,3)));
    end
    dopfig=figure(141); clf; hold on; grid on;
    plot(t, NEdop, "Color",colour_palette(1,:));
    plot(t, Ddop, "Color",colour_palette(2,:));
    xlabel("Time [s]", "Interpreter","latex", "FontSize",14);
    ylabel("DOP", "Interpreter","latex", "FontSize",14);
    %yyaxis right; ylabel("True yaw angle [deg]", "Interpreter","latex", "FontSize", 14);
    %ax = gca;
   % ax.YColor = "k";
   % plot(t, xtruetot(9,:) .* (180/pi), "k--");
    xline(t(15000),':',"Loss-of-GNSS", "Interpreter","latex", "FontSize",11);
    legend(["HDOP", "VDOP", ""], "Interpreter","latex", "FontSize",11)
    xlim([0, t(truestate.N)]);
   % ylim([0 1])
    exportgraphics(dopfig, "dop_standalone.pdf",'ContentType','vector');
end
%%
if fetch_data == true
    for m = 1 : Mt
       % m = vals(val);
        disp("Fetching correct filenames...");
        [matlab_data_file, fixed_lag_data_file, isam2_data_file] = ...
            get_data_files_for_run(simulation_case, run_otter, run_standstill, 0, m);

        fprintf("Loading nominal data from Matlab (%s)...\n", matlab_data_file);
        load(matlab_data_file);
     
        % Initialise all the arrays
        if m == 1
            % True path doesn't change currently, but easier like this
            %{
            xtruetot = zeros(15, N, M);
            xmekftot = zeros(15, N, M);
            xmekftotsf = zeros(15, N, M);
            xmekftotnt = zeros(15, N, M);
            
            xfgotot = zeros(15, N, M);
            xfgototsf = zeros(15, N, M);
            xfgototnt = zeros(15, N, M);
            xfgotothb = zeros(15, N, M);
            xfgotottk = zeros(15, N, M);

            Pmekftot = zeros(15, 15, N, M);
            Pmekftotsf = zeros(15, 15, N, M);
            Pmekftotnt = zeros(15, 15, N, M);

            Pfgotot = zeros(15, 15, N, M);
            Pfgototsf = zeros(15, 15, N, M);
            Pfgototnt = zeros(15, 15, N, M);
            Pfgotothb = zeros(15, 15, N, M);
            Pfgotottk = zeros(15, 15, N, M);
            %}
            error_norms_mekf_tot = zeros(5, N, Mt);
            error_norms_fgo_tot = zeros(5, N, Mt);
            error_norms_mekf_totsf = zeros(5, N, Mt);
            error_norms_fgo_totsf = zeros(5, N, Mt);
            error_norms_mekf_totnt = zeros(5, N, Mt);
            error_norms_fgo_totnt = zeros(5, N, Mt);
            error_norms_fgo_tothb = zeros(5, N, Mt);
            error_norms_fgo_tottk = zeros(5, N, Mt);
            
            sigma3_mekf_tot = zeros(5, N, Mt);
            sigma3_fgo_tot = zeros(5, N, Mt);
            sigma3_mekf_totsf = zeros(5, N, Mt);
            sigma3_fgo_totsf = zeros(5, N, Mt);
            sigma3_mekf_totnt = zeros(5, N, Mt);
            sigma3_fgo_totnt = zeros(5, N, Mt);
            sigma3_fgo_tothb = zeros(5, N, Mt);
            sigma3_fgo_tottk = zeros(5, N, Mt);
            
            error_mekf_tot = zeros(15, N, Mt);
            error_fgo_tot = zeros(15, N,Mt);
            error_mekf_totsf = zeros(15,N, Mt);
            error_fgo_totsf = zeros(15, N,Mt);
            error_mekf_totnt = zeros(15,N, Mt);
            error_fgo_totnt = zeros(15,N, Mt);
            error_fgo_tothb = zeros(15,N, Mt);
            error_fgo_tottk = zeros(15,N, Mt);
            
            rmse_mekf_tot = zeros(15,1,Mt);
            rmse_fgo_tot = zeros(15,1,Mt);
            rmse_mekf_sf_tot = zeros(15,1,Mt);
            rmse_fgo_sf_tot = zeros(15,1,Mt);
            rmse_mekf_nt_tot = zeros(15,1,Mt);
            rmse_fgo_nt_tot = zeros(15,1,Mt);
            rmse_fgo_hb_tot = zeros(15,1,Mt);
            rmse_fgo_tk_tot = zeros(15,1,Mt);
        end

        fprintf("Loading data from C++ (%s)...\n", fixed_lag_data_file);
    
        [t, p_est_fgo, v_est_fgo, a_est_fgo, acc_est_fgo, gyro_est_fgo, P_est_fgo] = parse_cpp_data_file(fixed_lag_data_file, simulation_case);
       
        % Assign the desired variables from the nominal data
        xtrue = [pos;  v_ib_i;  roll; pitch;  yaw;   b_acc;  b_ars];
        xmekfins = [p_ins; v_ins; roll_hat; pitch_hat; yaw_hat; b_acc_hat; b_ars_hat];
        P_mekf = P_est;
        xfgo = [p_est_fgo; v_est_fgo; a_est_fgo; acc_est_fgo; gyro_est_fgo];
        
        % Save to total array
       % xtruetot(:,:,m) = xtrue;
        %xmekftot(:,:,m) = xmekfins;
        %Pmekftot(:,:,:,m) = P_mekf;
       % xfgotot(:,:,m) = xfgo;
        %Pfgotot(:,:,:,m) = P_est_fgo;
        xtruetot = xtrue;
        xmekftot = xmekfins;
        Pmekftot = P_mekf;
        xfgotot = xfgo;
        Pfgotot = P_est_fgo;
    
        if ee3sigmabounds_sensor_fault == true
            disp("Fetching sensor faults filenames...");
            [matlab_data_file_sf, fixed_lag_data_file_sf, ~] = ...
            get_data_files_for_run(simulation_case, run_otter, run_standstill, 1, m);
            load(matlab_data_file_sf);
            xmekfins_sf = [p_ins; v_ins; roll_hat; pitch_hat; yaw_hat; b_acc_hat; b_ars_hat];
            P_mekf_sf = P_est;
            [~, p_est_fgo_sf, v_est_fgo_sf, a_est_fgo_sf, acc_est_fgo_sf, gyro_est_fgo_sf, P_est_fgo_sf] = parse_cpp_data_file(fixed_lag_data_file_sf, simulation_case);
            xfgosf  = [p_est_fgo_sf; v_est_fgo_sf; a_est_fgo_sf; acc_est_fgo_sf; gyro_est_fgo_sf];

            % Save to total array
           % xmekftotsf(:,:,m) = xmekfins_sf;
           % xfgototsf(:,:,m) = xfgosf;
           % Pmekftotsf(:,:,:,m) = P_mekf_sf;
           % Pfgototsf(:,:,:,m) = P_est_fgo_sf;
            xmekftotsf = xmekfins_sf;
            xfgototsf = xfgosf;
            Pmekftotsf = P_mekf_sf;
            Pfgototsf = P_est_fgo_sf;
        end
    
        if ee3sigmabounds_outlier_rejection == true
            disp("Fetching natural test filenames...");
            [matlab_data_file_nt, fixed_lag_data_file_nt, ~] = ...
            get_data_files_for_run(simulation_case, run_otter, run_standstill, 2, m);
            load(matlab_data_file_nt);
            xmekfins_nt = [p_ins; v_ins; roll_hat; pitch_hat; yaw_hat; b_acc_hat; b_ars_hat];
            P_mekf_nt = P_est;
            [~, p_est_fgo_nt, v_est_fgo_nt, a_est_fgo_nt, acc_est_fgo_nt, gyro_est_fgo_nt, P_est_fgo_nt] = parse_cpp_data_file(fixed_lag_data_file_nt, simulation_case);
            xfgont  = [p_est_fgo_nt; v_est_fgo_nt; a_est_fgo_nt; acc_est_fgo_nt; gyro_est_fgo_nt];

            % Save to total array
           % xmekftotnt(:,:,m) = xmekfins_nt;
           % xfgototnt(:,:,m) = xfgont;
           % Pmekftotnt(:,:,:,m) = P_mekf_nt;
          %  Pfgototnt(:,:,:,m) = P_est_fgo_nt;
            xmekftotnt = xmekfins_nt;
            xfgototnt = xfgont;
            Pmekftotnt = P_mekf_nt;
            Pfgototnt = P_est_fgo_nt;
        end
    
        if ee3sigmabounds_huber == true
            disp("Fetching huber test filenames...");
            [~, fixed_lag_data_file_hb, ~] = ...
            get_data_files_for_run(simulation_case, run_otter, run_standstill, 3, m);
            [~, p_est_fgo_hb, v_est_fgo_hb, a_est_fgo_hb, acc_est_fgo_hb, gyro_est_fgo_hb, P_est_fgo_hb] = parse_cpp_data_file(fixed_lag_data_file_hb, simulation_case);
            xfgohb  = [p_est_fgo_hb; v_est_fgo_hb; a_est_fgo_hb; acc_est_fgo_hb; gyro_est_fgo_hb];

            %xfgotothb(:,:,m) = xfgohb;
            %Pfgotothb(:,:,:,m) = P_est_fgo_hb;
            xfgotothb = xfgohb;
            Pfgotothb = P_est_fgo_hb;
        end
    
        if ee3sigmabounds_tukey == true
            disp("Fetching tukey test filenames...");
            [~, fixed_lag_data_file_tk, ~] = ...
            get_data_files_for_run(simulation_case, run_otter, run_standstill, 4, m);
            [~, p_est_fgo_tk, v_est_fgo_tk, a_est_fgo_tk, acc_est_fgo_tk, gyro_est_fgo_tk, P_est_fgo_tk] = parse_cpp_data_file(fixed_lag_data_file_tk, simulation_case);
            xfgotk  = [p_est_fgo_tk; v_est_fgo_tk; a_est_fgo_tk; acc_est_fgo_tk; gyro_est_fgo_tk];

            %xfgotottk(:,:,m) = xfgotk;
            %Pfgotottk(:,:,:,m) = P_est_fgo_tk;
            xfgotottk = xfgotk;
            Pfgotottk = P_est_fgo_tk;
        end

    
    %for m = 1 : M
      %  m = vals(val);
        error_mekf = xtruetot - xmekftot;
        error_fgo = xtruetot - xfgotot;
        error_mekf_sf = xtruetot - xmekftotsf;
        error_fgo_sf = xtruetot - xfgototsf;
        error_mekf_nt = xtruetot - xmekftotnt;
        error_fgo_nt = xtruetot - xfgototnt;
        error_fgo_hb = xtruetot - xfgotothb;
        error_fgo_tk = xtruetot - xfgotottk;
    
        error_mekf_tot(:,:,m) = error_mekf;
        error_fgo_tot(:,:,m) = error_fgo;
        error_mekf_totsf(:,:,m) = error_mekf_sf;
        error_fgo_totsf(:,:,m) = error_fgo_sf;
        error_mekf_totnt(:,:,m) = error_mekf_nt;
        error_fgo_totnt(:,:,m) = error_fgo_nt;
        error_fgo_tothb(:,:,m) = error_fgo_hb;
        error_fgo_tottk(:,:,m) = error_fgo_tk;
    
        rmse_mekf_tot(:,:,m) = compute_rmse(error_mekf);
        rmse_fgo_tot(:,:,m)  = compute_rmse(error_fgo);
        rmse_mekf_sf_tot(:,:,m)  = compute_rmse(error_mekf_sf);
        rmse_fgo_sf_tot(:,:,m)  = compute_rmse(error_fgo_sf);
        rmse_mekf_nt_tot(:,:,m)  =compute_rmse(error_mekf_nt);
        rmse_fgo_nt_tot(:,:,m)  = compute_rmse(error_fgo_nt);
        rmse_fgo_hb_tot(:,:,m)  = compute_rmse(error_fgo_hb);
        rmse_fgo_tk_tot(:,:,m)  = compute_rmse(error_fgo_tk);
    
        for substatenum = 1 : 5
            offset = 3 * (substatenum - 1);
            start_id = 1 + offset;
            end_id = 3 + offset;
    
            %normstate = vecnorm(dx(start_id:end_id, :));
            error_norms_mekf_tot(substatenum, :, m) = vecnorm(error_mekf(start_id:end_id, :));
            error_norms_fgo_tot(substatenum, :, m) = vecnorm(error_fgo(start_id:end_id, :));
            error_norms_mekf_totsf(substatenum, :, m) = vecnorm(error_mekf_sf(start_id:end_id, :));
            error_norms_fgo_totsf(substatenum, :, m) = vecnorm(error_fgo_sf(start_id:end_id, :));
            error_norms_mekf_totnt(substatenum, :, m) = vecnorm(error_mekf_nt(start_id:end_id, :));
            error_norms_fgo_totnt(substatenum, :, m) = vecnorm(error_fgo_nt(start_id:end_id, :));
            error_norms_fgo_tothb(substatenum, :, m) = vecnorm(error_fgo_hb(start_id:end_id, :));
            error_norms_fgo_tottk(substatenum, :, m) = vecnorm(error_fgo_tk(start_id:end_id, :));
    
    
            for ele = 1 : size(sigma3_mekf_tot,2)
                sigma3_mekf_tot(substatenum, ele, m) = 3 * sqrt(trace(Pmekftot(start_id:end_id,start_id:end_id,ele)));
                sigma3_fgo_tot(substatenum, ele, m) = 3 * sqrt(trace(Pfgotot(start_id:end_id,start_id:end_id,ele)));
                sigma3_mekf_totsf(substatenum, ele, m) = 3 * sqrt(trace(Pmekftotsf(start_id:end_id,start_id:end_id,ele)));
                sigma3_fgo_totsf(substatenum, ele, m) = 3 * sqrt(trace(Pfgototsf(start_id:end_id,start_id:end_id,ele)));
                sigma3_mekf_totnt(substatenum, ele, m) = 3 * sqrt(trace(Pmekftotnt(start_id:end_id,start_id:end_id,ele)));
                sigma3_fgo_totnt(substatenum, ele, m) = 3 * sqrt(trace(Pfgototnt(start_id:end_id,start_id:end_id,ele)));
                sigma3_fgo_tothb(substatenum, ele, m) = 3 * sqrt(trace(Pfgotothb(start_id:end_id,start_id:end_id,ele)));
                sigma3_fgo_tottk(substatenum, ele, m) = 3 * sqrt(trace(Pfgotottk(start_id:end_id,start_id:end_id,ele)));
            end
        end
    end
    
    mean_error_norms_mekf_tot = mean(error_norms_mekf_tot, 3);
    mean_error_norms_fgo_tot = mean(error_norms_fgo_tot,3);
    mean_error_norms_mekf_totsf = mean(error_norms_mekf_totsf,3);
    mean_error_norms_fgo_totsf = mean(error_norms_fgo_totsf,3);
    mean_error_norms_mekf_totnt = mean(error_norms_mekf_totnt,3);
    mean_error_norms_fgo_totnt = mean(error_norms_fgo_totnt,3);
    mean_error_norms_fgo_tothb = mean(error_norms_fgo_tothb,3);
    mean_error_norms_fgo_tottk = mean(error_norms_fgo_tottk,3);
    
    mean_sigma3_mekf_tot = mean(sigma3_mekf_tot,3);
    mean_sigma3_fgo_tot = mean(sigma3_fgo_tot ,3);
    mean_sigma3_mekf_totsf = mean(sigma3_mekf_totsf,3);
    mean_sigma3_fgo_totsf = mean(sigma3_fgo_totsf,3);
    mean_sigma3_mekf_totnt = mean(sigma3_mekf_totnt,3);
    mean_sigma3_fgo_totnt = mean(sigma3_fgo_totnt,3);
    mean_sigma3_fgo_tothb = mean(sigma3_fgo_tothb,3);
    mean_sigma3_fgo_tottk = mean(sigma3_fgo_tottk,3);

    rmse_mekf = mean(rmse_mekf_tot, 3);
    rmse_fgo = mean(rmse_fgo_tot, 3);
    rmse_mekf_sf = mean(rmse_mekf_sf_tot, 3);
    rmse_fgo_sf = mean(rmse_fgo_sf_tot, 3);
    rmse_mekf_nt = mean(rmse_mekf_nt_tot, 3);
    rmse_fgo_nt = mean(rmse_fgo_nt_tot, 3);
    rmse_fgo_hb = mean(rmse_fgo_hb_tot, 3);
    rmse_fgo_tk = mean(rmse_fgo_tk_tot, 3);

    var_rmse_mekf = var(rmse_mekf_tot,1, 3);
    var_rmse_fgo = var(rmse_fgo_tot,1, 3);
    var_rmse_mekf_sf = var(rmse_mekf_sf_tot,1, 3);
    var_rmse_fgo_sf = var(rmse_fgo_sf_tot, 1,3);
    var_rmse_mekf_nt = var(rmse_mekf_nt_tot,1, 3);
    var_rmse_fgo_nt = var(rmse_fgo_nt_tot,1, 3);
    var_rmse_fgo_hb = var(rmse_fgo_hb_tot,1, 3);
    var_rmse_fgo_tk = var(rmse_fgo_tk_tot,1, 3);


    save(sprintf("/Volumes/T9/02/results/otter/processed/secondary_analysed_%0d.mat",Mt), ...
        "mean_error_norms_mekf_tot","mean_error_norms_fgo_tot", ...
        "mean_error_norms_mekf_totsf", ...
        "mean_error_norms_fgo_totsf", ...
        "mean_error_norms_mekf_totnt", ...
        "mean_error_norms_fgo_totnt", ...
        "mean_error_norms_fgo_tothb", ...
        "mean_error_norms_fgo_tottk", ...
        "mean_sigma3_mekf_tot", ...
        "mean_sigma3_fgo_tot", ...
        "mean_sigma3_mekf_totsf", ...
        "mean_sigma3_fgo_totsf", ...
        "mean_sigma3_mekf_totnt", ...
        "mean_sigma3_fgo_totnt", ...
        "mean_sigma3_fgo_tothb", ...
        "mean_sigma3_fgo_tottk", ...
        "rmse_mekf", ...
        "rmse_fgo", ...
        "rmse_mekf_sf", ...
        "rmse_fgo_sf", ...
        "rmse_mekf_nt", ...
        "rmse_fgo_nt", ...
        "rmse_fgo_hb", ...
        "rmse_fgo_tk", ...
        "var_rmse_mekf", ...
        "var_rmse_fgo", ...
        "var_rmse_mekf_sf", ...
        "var_rmse_fgo_sf", ...
        "var_rmse_mekf_nt", ...
        "var_rmse_fgo_nt", ...
        "var_rmse_fgo_hb", ...
        "var_rmse_fgo_tk", ...
        "xtruetot",...
        "Mt","N","t", "-v7.3");




   % save(sprintf("/Volumes/T9/01/results/data_for_analysis_%d.mat",M),"N", "M", "t", "xtruetot", ...
   %     "xmekftot", "xfgotot", "Pmekftot", "Pfgotot","xmekftotsf", ...
  %      "xfgototsf", "Pmekftotsf", "Pfgototsf","xmekftotnt", ...
  %      "xfgototnt", "Pmekftotnt", "Pfgototnt", ...
  %      "xfgotothb","Pfgotothb","xfgotottk","Pfgotottk", "-v7.3");
  %  save(sprintf("../results/data_for_analysis_%d.mat",M),"N", "M", "t", "xtruetot", ...
  %      "xmekftot", "xfgotot", "Pmekftot", "Pfgotot","xmekftotsf", ...
  %      "xfgototsf", "Pmekftotsf", "Pfgototsf","xmekftotnt", ...
  %      "xfgototnt", "Pmekftotnt", "Pfgototnt", ...
  %      "xfgotothb","Pfgotothb","xfgotottk","Pfgotottk", "-v7.3");
else
  load(sprintf("/Volumes/T9/02/results/otter/processed/analysed_%d.mat", Mt));
  %load(sprintf("/Volumes/T9/02/results/otter/processed/secondary_analysed_%d.mat", 1));

end


%%
if ee3sigmabounds == true  
    % Set 1 or 0 for which states to display in plot
    % 1. Pos
    % 2. Vel
    % 3. Att
    % 4. Acc bias
    % 5. Gyro bias
    plot_position = true;
    plot_velocity = false;
    plot_attitude = true;
    plot_acc_bias = false;
    plot_gyr_bias = false;
    
    plotting_mask = [plot_position plot_velocity plot_attitude plot_acc_bias plot_gyr_bias];
    axis_mask = [
      1 1 1;
      1 1 1;
      1 1 1;
      1 1 1;
      1 1 1;
    ];
    
    mekf_plot_title = sprintf("MEKF MSS case 3sigma error plot %s", title_info);
    fgo_plot_title =sprintf("Fixed-lag FGO MSS case 3sigma error plot %s", title_info);
    third_title = sprintf("Comparison of average error norm of %d runs", Mt);
    
    disp("Plotting estimation error results");

    ee_fig = figure(43); clf;
    legends = [];
  
    if ee3sigmabounds_sensor_fault == true
        legends = [legends "ESKF s.f. $3\sigma$" ];
        plot_mean_ee_3sigma(ee_fig, plotting_mask, t, mean_error_norms_mekf_totsf, mean_sigma3_mekf_totsf, 3, legends, false, true, Mt);
        legends = [legends "FGO s.f. $3\sigma$"];
        plot_mean_ee_3sigma(ee_fig, plotting_mask, t, mean_error_norms_fgo_totsf, mean_sigma3_fgo_totsf, 4, legends, false, true, Mt);
    end
   
    if ee3sigmabounds_outlier_rejection == true
        legends = [legends "ESKF n.t. $3\sigma$" ];
        plot_mean_ee_3sigma(ee_fig, plotting_mask, t, mean_error_norms_mekf_totnt, mean_sigma3_mekf_totnt, 5, legends, false, true, Mt);
        %legends = [legends "FGO n.t. $3\sigma$"];
        %plot_mean_ee_3sigma(ee_fig, plotting_mask, t, mean_error_norms_fgo_totnt, mean_sigma3_fgo_totnt, 6, legends, false, true, Mt);
    end

    if ee3sigmabounds_huber == true
        legends = [legends "FGO Huber-M $3\sigma$"];
        plot_mean_ee_3sigma(ee_fig, plotting_mask, t, mean_error_norms_fgo_tothb, mean_sigma3_fgo_tothb, 7, legends, false, true, Mt);
    end

    if ee3sigmabounds_tukey == true
        legends = [legends "FGO Tukey-M $3\sigma$"];
        plot_mean_ee_3sigma(ee_fig, plotting_mask, t, mean_error_norms_fgo_tottk, mean_sigma3_fgo_tottk, 8, legends, false, true, Mt);
    end

    legends = [legends "ESKF $3\sigma$" ];
    plot_mean_ee_3sigma(ee_fig, plotting_mask, t, mean_error_norms_mekf_tot, mean_sigma3_mekf_tot, 1, legends, false, true, Mt);
    legends = [legends "FGO $3\sigma$"];
    plot_mean_ee_3sigma(ee_fig, plotting_mask, t, mean_error_norms_fgo_tot, mean_sigma3_fgo_tot, 2, legends, false, true, Mt);
    
    if ee3sigmabounds_sensor_fault == true
        legends = [legends "ESKF s.f." ];
        plot_mean_ee_3sigma(ee_fig, plotting_mask, t, mean_error_norms_mekf_totsf, mean_sigma3_mekf_totsf, 3, legends, true, false, Mt);
        legends = [legends "FGO s.f."];
        plot_mean_ee_3sigma(ee_fig, plotting_mask, t, mean_error_norms_fgo_totsf, mean_sigma3_fgo_totsf, 4, legends, true, false, Mt);
    end
   
    if ee3sigmabounds_outlier_rejection == true
        legends = [legends "ESKF n.t." ];
        plot_mean_ee_3sigma(ee_fig, plotting_mask, t, mean_error_norms_mekf_totnt, mean_sigma3_mekf_totnt, 5, legends, true, false, Mt);
       % legends = [legends "FGO n.t."];
       % plot_mean_ee_3sigma(ee_fig, plotting_mask, t, mean_error_norms_fgo_totnt, mean_sigma3_fgo_totnt, 6, legends, true, false, Mt);
    end

    if ee3sigmabounds_huber == true
        legends = [legends "FGO Huber-M"];
        plot_mean_ee_3sigma(ee_fig, plotting_mask, t, mean_error_norms_fgo_tothb, mean_sigma3_fgo_tothb, 7, legends, true, false, Mt);
    end

    if ee3sigmabounds_tukey == true
        legends = [legends "FGO Tukey-M"];
        plot_mean_ee_3sigma(ee_fig, plotting_mask, t, mean_error_norms_fgo_tottk, mean_sigma3_fgo_tottk, 8, legends, true, false, Mt);
    end
   
    
    legends = [legends "ESKF" ];
    plot_mean_ee_3sigma(ee_fig, plotting_mask, t, mean_error_norms_mekf_tot, mean_sigma3_mekf_tot, 1, legends, true, false, Mt);
    legends = [legends "FGO"];
    plot_mean_ee_3sigma(ee_fig, plotting_mask, t, mean_error_norms_fgo_tot, mean_sigma3_fgo_tot, 2, legends, true, false, Mt);
    subplot(2,1,1); yyaxis right;  ylabel("True yaw angle [deg]", "Interpreter","latex", "FontSize", 14);
    legends = [legends '' ];
    
  %  [NEdop, NEDdop] = compute_dop(P_est);
  
    ax = gca;
    ax.YColor = "k";
    plot(t, xtruetot(9,:,1) .* (180/pi),"k--");
    subplot(2,1,1);
    xline(t(15000),':');
    legends = [legends ''];
    legend(legends);
    %set(lh,'position',[.1 .5 .1 .1])
    subplot(2,1,2); yyaxis right; ylabel("True yaw angle [deg]", "Interpreter","latex", "FontSize", 14);
    ax = gca;
    ax.YColor = "k";
    plot(t, xtruetot(9,:,1) .* (180/pi),"k--");
    subplot(2,1,2);
    xline(t(15000),':',"Loss-of-GNSS", "Interpreter","latex", "FontSize",14);

    %subplot(3,1,3); 
    %plot(t, NEdop,"b--", "DisplayName","NE DOP");
    %plot(t, Ddop,"r--", "DisplayName","NED DOP");  

    exportgraphics(ee_fig, "ee_norm_with_bounds_highlights_dop.pdf",'ContentType','vector');

    disp("Done");
end
%{
if cca_analysis == true
    disp("Plotting CCA analysis for MEKF run");
    k = plot_cca_for_all_substate_pairs(45, xmekfins);
    disp("Done");
    
    if save_cca_to_file == true
        filename = "cca.pdf";
        for i = 1:length(k)
            exportgraphics(k(i), filename, 'append', true);
        end
        close all
    end
end
%}
%%
variables = {
    'N [m]', 'E [m]', 'D [m]', 'N [m/s]', 'E [m/s]', 'D [m/s]',...
    'Roll [deg]', 'Pitch [deg]', 'Yaw [deg]', 'x [m/s^2]', 'y [m/s^2]', 'z [m/s^2]', ...
     'Roll rate [deg/s]', 'Pitch rate [deg/s]', 'Yaw rate [deg/s]'
};
rows = {
    'MEKF Nominal', 'MEKF Sensor fault', 'MEKF Natural test', 'FGO Nominal', 'FGO Sensor fault','FGO Natural test',...
    'FGO Huber-M', 'FGO Tukey-M'
};

rmse_table = table(rmse_mekf, rmse_mekf_sf, rmse_mekf_nt,rmse_fgo,  rmse_fgo_sf, rmse_fgo_nt, rmse_fgo_hb, rmse_fgo_tk, 'VariableNames',rows);
rmse_table.Properties.RowNames = variables'

%var_rmse_table = table(var_rmse_mekf, var_rmse_mekf_sf, var_rmse_mekf_nt, var_rmse_fgo,  var_rmse_fgo_sf, var_rmse_fgo_nt, var_rmse_fgo_hb, var_rmse_fgo_tk, 'VariableNames',rows);
%var_rmse_table.Properties.RowNames = variables'


% Combine identifiers and records
rmse = [rmse_mekf, rmse_mekf_sf, rmse_mekf_nt,rmse_fgo,  rmse_fgo_sf, rmse_fgo_nt, rmse_fgo_hb, rmse_fgo_tk]';
combined_data = [rows', num2cell(rmse)];

% Write to CSV file
output_file = 'input.csv';
fid = fopen(output_file, 'w');
for i = 1:size(combined_data, 1)
    fprintf(fid, '%s,', combined_data{i, 1});
    fprintf(fid, '%f,', combined_data{i, 2:end-1});
    fprintf(fid, '%f\n', combined_data{i, end});
end
fclose(fid);

disp(['CSV file written to ', output_file]);



%%
%{
if filter_consistency == true


    NEES = zeros(15, N);
    NEESfgo = zeros(15, N);

    for idx = 1 : N
        for state_idx = 1 : 15
            xval = xtrue(state_idx, idx) - xmekfins(state_idx, idx);
            xvalfgo = xtrue(state_idx, idx) - xfgo(state_idx, idx);

            if state_idx == 7 || state_idx == 8 ||state_idx == 9
                xval = ssa(xval);
                xvalfgo = ssa(xvalfgo);
            end

            Pval = P_est(state_idx, state_idx, idx);
            Pvalfgo = P_est_fgo(state_idx, state_idx, idx);

            NEES(state_idx, idx) = xval^2/Pval;
            NEESfgo(state_idx, idx) = xvalfgo^2/Pvalfgo;
        end
    end
    
    figure(114);clf;hold on;
    tiled = tiledlayout(5,3); title(tiled, "ANEES")
    
    for i = 1 : 15
        nexttile; hold on;
        plot((NEESfgo(i,:)));
        plot((NEES(i,:)));
        legend("FGO", "ESKF")
    end

    alpha = 0.025;
    scaledlower1=chi2inv(alpha,M)/(M);
    scaledupper1=chi2inv(1-alpha,M)/(M);
    ANEES = mean(NEES, 2);
    ANEESfgo = mean(NEESfgo, 2);
end
%}
%%
%{
norm_crosscov_pos_acc_bias = zeros(1,N);
norm_crosscov_pos_gyro_bias = zeros(1,N);
norm_crosscov_ori_acc_bias = zeros(1,N);
norm_crosscov_ori_gyro_bias = zeros(1,N);
norm_crosscov_vel_acc_bias = zeros(1,N);
norm_crosscov_vel_gyro_bias = zeros(1,N);
norm_crosscov_ori_vel = zeros(1,N);
norm_crosscov_pos_ori = zeros(1,N);


yawcorr = zeros(15, N);

for k = 1 : N
    norm_crosscov_pos_acc_bias(k) = norm(P_est(1:3,10:12,k));
    norm_crosscov_pos_gyro_bias(k) = norm(P_est(1:3,13:15,k));
    norm_crosscov_ori_acc_bias(k) = norm(P_est(7:9,10:12,k));
    norm_crosscov_ori_gyro_bias(k) = norm(P_est(7:9,13:15,k));
    norm_crosscov_vel_acc_bias(k) = norm(P_est(4:6,10:12,k));
    norm_crosscov_vel_gyro_bias(k) = norm(P_est(4:6,13:15,k));
    norm_crosscov_ori_vel(k) = norm(P_est(7:9,4:6,k));
    norm_crosscov_pos_ori(k) = norm(P_est(1:3,7:9,k));
    
    corrmat = corr(P_est(:,:,k));
    yawcorr(:,k) = (corrmat(9, :));
end

figure(42);clf;hold on;
subplot(4,2,1); hold on; title("Pos acc bias crosscov norm");
plot(t, norm_crosscov_pos_acc_bias);
subplot(4,2,2); hold on; title("Pos gyro bias crosscov norm");
plot(t, norm_crosscov_pos_gyro_bias);
subplot(4,2,3); hold on; title("Att acc bias crosscov norm");
plot(t, norm_crosscov_ori_acc_bias);
subplot(4,2,4); hold on; title("Att gyro bias crosscov norm");
plot(t, norm_crosscov_ori_gyro_bias);
subplot(4,2,5); hold on; title("Vel acc bias crosscov norm");
plot(t, norm_crosscov_vel_acc_bias);
subplot(4,2,6); hold on; title("Vel gyro bias crosscov norm");
plot(t, norm_crosscov_vel_gyro_bias);
subplot(4,2,7); hold on; title("Att vel crosscov norm");
plot(t, norm_crosscov_ori_vel);
subplot(4,2,8); hold on; title("Pos vel crosscov norm");
plot(t, norm_crosscov_ori_vel);

%%
figure(43); clf; hold on;
subplot(5,1,1); hold on; title("Yaw correlation with pos");
plot(t, yawcorr(1:3,:));
legend(["N" "E" "D"])
subplot(5,1,2); hold on; title("Yaw correlation with vel");
plot(t, yawcorr(4:6,:));
legend(["x" "y" "z"])
subplot(5,1,3); hold on; title("Yaw correlation with att");
plot(t, yawcorr(7:9,:));
legend(["Roll" "Pitch" "Yaw"])
subplot(5,1,4); hold on; title("Yaw correlation with acc bias");
plot(t, yawcorr(10:12,:));
legend(["x" "y" "z"])
subplot(5,1,5); hold on; title("Yaw correlation with gyro bias");
plot(t, yawcorr(13:15,:));
legend(["Roll rate" "Pitch rate" "Yaw rate"])

%% Correlation
Rcorr = zeros(size(P_est_fgo));
for k = 1 : N
    Rcorr(:,:,k) = corr(P_est_fgo(:,:,k));
end


%}


%%

%{
DOP = zeros(4,4,N);
PDOP = zeros(1, N);

for idx = 1 : N
    A = zeros(4,4);
    A(:,4) = ones(4,1);
    for locator_idx = 1 : size(locator_origins, 2)
        l = locator_origins(:, locator_idx);
        p = xtrue(1:3, idx);
        locator_range = sqrt((p(1) - l(1))^2 +  (p(2) - l(2))^2 + (p(3) - l(3))^2 );

        for cidx = 1 : 3
            A(locator_idx, cidx) = (p(cidx) - l(cidx)) / locator_range;
        end
    end

    DOP(:, :, idx) = inv(A' * A);
    PDOP(idx) = sqrt(DOP(1,1,idx) + DOP(2,2,idx)  + DOP(3,3,idx));
end
%%
figure(5413)
plot(PDOP)

%}

