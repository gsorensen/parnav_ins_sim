function [matlab_filename, ...
    fgo_fixed_lag_filename,...
    fgo_isam2_filename] = ...
    get_data_files_for_run(simulation_case, is_otter, is_standstill, outlier_state, m)

if is_otter == true
    matlab_prefix = sprintf("/Volumes/T9/02/results/otter/mekf/%02d_100Hz_aided_at_10Hz_by_", m);
    fixed_lag_prefix = sprintf("/Volumes/T9/02/results/otter/fgo_fixed_lag/%02d_100Hz_aided_at_10Hz_by_", m);
    isam2_prefix = sprintf("/Volumes/T9/02/results/otter/fgo_isam2/%02d_100Hz_aided_at_10Hz_by_", m);
    %matlab_prefix = sprintf("../results/otter/mekf/%02d_100Hz_aided_at_10Hz_by_", m);
    %fixed_lag_prefix = sprintf("../results/otter/fgo_fixed_lag/%02d_100Hz_aided_at_10Hz_by_", m);
    %isam2_prefix = sprintf("../results/otter/fgo_isam2/%02d_100Hz_aided_at_10Hz_by_", m);
elseif is_standstill == true
    matlab_prefix = sprintf("../results/standstill/mekf/%02d_100Hz_aided_at_10Hz_by_", m);
    fixed_lag_prefix = sprintf("../results/standstill/fgo_fixed_lag/%02d_100Hz_aided_at_10Hz_by_", m);
    isam2_prefix = sprintf("../results/standstill/fgo_isam2/%02d_100Hz_aided_at_10Hz_by_", m);
else
    matlab_prefix = sprintf("../results/dummy/mekf/%02d_100Hz_aided_at_10Hz_by_", m);
    fixed_lag_prefix = sprintf("../results/dummy/fgo_fixed_lag/%02d_100Hz_aided_at_10Hz_by_", m);
    isam2_prefix = sprintf("../results/dummy/fgo_isam2/%02d_100Hz_aided_at_10Hz_by_", m);
end

aiding_suffix = "";

switch simulation_case
    case 0
        aiding_suffix = "GNSS_pos";
    case 1
        aiding_suffix = "PARS_range_only";
    case 2
        aiding_suffix = "PARS_full";
end

outlier_suffix = "";

switch outlier_state
    case 0
    case 1
        outlier_suffix = "_sensor_fault";
    case 2
        outlier_suffix = "_outlier_rejection"; % Natural test
    case 3
        outlier_suffix = "_m_est_huber"; % NOTE: These are not implement for MEKF
    case 4
        outlier_suffix = "_m_est_tukey";
end


matlab_filename = sprintf("%s%s%s.mat", matlab_prefix, aiding_suffix, outlier_suffix);
fgo_fixed_lag_filename = sprintf("%s%s%s.csv", fixed_lag_prefix, aiding_suffix,outlier_suffix);
fgo_isam2_filename = sprintf("%s%s.csv", isam2_prefix, aiding_suffix);

end