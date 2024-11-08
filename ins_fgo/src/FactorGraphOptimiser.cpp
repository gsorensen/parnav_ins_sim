#include "FactorGraphOptimiser.hpp"
#include "OutlierRejection.hpp"
#include "PARS.hpp"
#include "PARSAzimuthFactor.hpp"
#include "PARSElevationFactor.hpp"
#include "PARSRangeFactor.hpp"
#include "utils.hpp"

#include <cstdint>
#include <fmt/core.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Quaternion.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/AttitudeFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/sam/RangeFactor.h>
#include <variant>

using gtsam::symbol_shorthand::B; // bias (ax, ay, az, gx, gy, gz)
using gtsam::symbol_shorthand::V; // velocity (xdot, ydot, zdot)
using gtsam::symbol_shorthand::X; // pose (x, y, z, r, p, y)

FactorGraphOptimiser::FactorGraphOptimiser(const std::uint64_t &time_steps) : m_N{time_steps}
{
}

auto FactorGraphOptimiser::increment_correction_count() -> void
{
    m_correction_count++;
}

auto FactorGraphOptimiser::initialise_preintegrated_IMU_measurement(const IMU &imu, bool use_combined_measurement)
    -> void
{
    // gtsam::imuBias::ConstantBias prior_imu_bias{imu.acc_bias.col(0), imu.gyro_bias.col(0)};
    gtsam::imuBias::ConstantBias prior_imu_bias{};
    boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> p = imu.params;

    if (use_combined_measurement)
    {
        m_imu_preintegrated = gtsam::PreintegratedCombinedMeasurements(p, prior_imu_bias);
    }
    else
    {
        m_imu_preintegrated = gtsam::PreintegratedImuMeasurements(p, prior_imu_bias);
    }
}

auto FactorGraphOptimiser::initialise_true_bias(const Eigen::Vector3d &initial_acc_bias,
                                                const Eigen::Vector3d &initial_gyro_bias) -> void
{
    m_true_bias = gtsam::imuBias::ConstantBias{initial_acc_bias, initial_gyro_bias};
}

auto FactorGraphOptimiser::initialise_optimiser(const OptimisationScheme &optimisation_scheme, const double &fixed_lag)
    -> void
{
    gtsam::ISAM2Params params;
    switch (optimisation_scheme)
    {
    case OptimisationScheme::ISAM2:
        fmt::print("Using ISAM2\n");
        params.relinearizeThreshold = 0.001;
        params.relinearizeSkip = 1;
        params.findUnusedFactorSlots = true;
        // params.setFactorization("QR");
        break;
    case OptimisationScheme::FixedLag:
        fmt::print("Using ISAM2 Fixed lag smoother\n");
        params.relinearizeThreshold = 0.001;
        params.relinearizeSkip = 1;
        params.findUnusedFactorSlots = true;
        // params.setFactorization("QR");
        break;
    case OptimisationScheme::LevenbergMarquardt:
        fmt::print("Using LM\n");
        break;
    default:
        break;
    }

    m_isam2 = gtsam::ISAM2(params);
    m_fixed_lag_smoother = gtsam::IncrementalFixedLagSmoother(fixed_lag, params);

    m_smoother_timestamps_maps[X(m_correction_count)] = 0.0;
    m_smoother_timestamps_maps[V(m_correction_count)] = 0.0;
    m_smoother_timestamps_maps[B(m_correction_count)] = 0.0;
}

auto FactorGraphOptimiser::initialise_priors(const InitialStateEstimate &est) -> void
{
    const gtsam::Point3 &prior_point{est.pos};
    gtsam::Rot3 prior_rotation = gtsam::Rot3::Quaternion(est.att[0], est.att[1], est.att[2], est.att[3]);
    gtsam::Pose3 prior_pose{prior_rotation, prior_point};
    const gtsam::Vector3 &prior_velocity{est.vel};
    gtsam::imuBias::ConstantBias prior_imu_bias{est.acc_bias, est.gyro_bias};

    gtsam::noiseModel::Diagonal::shared_ptr prior_pose_noise_model = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << est.A_att, est.A_att, est.A_att, est.A_pos, est.A_pos, est.A_pos).finished());

    gtsam::noiseModel::Diagonal::shared_ptr prior_velocity_noise_model =
        gtsam::noiseModel::Isotropic::Sigma(3, est.A_vel);

    gtsam::noiseModel::Diagonal::shared_ptr prior_bias_noise_model =
        gtsam::noiseModel::Isotropic::Sigmas((gtsam::Vector(6) << est.A_acc_bias, est.A_acc_bias, est.A_acc_bias,
                                              est.A_gyro_bias, est.A_gyro_bias, est.A_gyro_bias)
                                                 .finished());

    m_prev_estimate = gtsam::NavState{prior_pose, prior_velocity};

    m_initial_values.insert(X(m_correction_count), prior_pose);
    m_initial_values.insert(V(m_correction_count), prior_velocity);
    m_initial_values.insert(B(m_correction_count), prior_imu_bias);

    m_graph = gtsam::NonlinearFactorGraph{};
    m_graph.add(boost::make_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(m_correction_count), prior_pose,
                                                                     prior_pose_noise_model));
    m_graph.add(boost::make_shared<gtsam::PriorFactor<gtsam::Vector3>>(V(m_correction_count), prior_velocity,
                                                                       prior_velocity_noise_model));
    m_graph.add(boost::make_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(
        B(m_correction_count), prior_imu_bias, prior_bias_noise_model));
}

auto FactorGraphOptimiser::initialise_marginal_covariance() -> void
{
    gtsam::Marginals marginals_init{m_graph, m_initial_values};
    gtsam::Matrix66 P_X = marginals_init.marginalCovariance(X(m_correction_count));
    gtsam::Matrix33 P_V = marginals_init.marginalCovariance(V(m_correction_count));
    gtsam::Matrix66 P_B = marginals_init.marginalCovariance(B(m_correction_count));

    Matrix15d P0 = Eigen::MatrixXd::Zero(15, 15);
    P0.block<6, 6>(0, 0) = P_X;
    P0.block<3, 3>(6, 6) = P_V;
    P0.block<6, 6>(9, 9) = P_B;

    m_P_X = P_X;
    m_P_V = P_V;
    m_P_B = P_B;

    m_P.push_back(P0);
    m_P_k = P0;
    m_P_corr = P0;
}

auto FactorGraphOptimiser::predict_state(const Eigen::Vector3d &f, const Eigen::Vector3d &w, const double &dt) -> void
{
    m_output_time += dt;

    // Integrate measurement
    std::visit([f, w, dt](auto &&x) { x.integrateMeasurement(f, w, dt); }, m_imu_preintegrated);

    // Prediction
    gtsam::Vector3 pos_prev = m_prev_estimate.pose().translation();
    gtsam::Vector3 vel_prev = m_prev_estimate.velocity();
    gtsam::Rot3 rot_prev = m_prev_estimate.pose().rotation();

    gtsam::Vector3 ang_bias = m_prev_bias_estimate.gyroscope();
    gtsam::Vector3 acc_bias = m_prev_bias_estimate.accelerometer();

    gtsam::Vector3 inc_ang = (w - ang_bias) * dt;
    gtsam::Rot3 delta_rot = gtsam::Rot3::Expmap(inc_ang);
    gtsam::Rot3 rot_new = (rot_prev * delta_rot).normalized();

    gtsam::Vector3 gravity =
        std::visit([](auto &&x) -> gtsam::Vector3 { return x.params()->getGravity(); }, m_imu_preintegrated);
    gtsam::Vector3 acc_new = rot_new * (f - acc_bias) + gravity;
    gtsam::Vector3 vel_new = vel_prev + acc_new * dt;
    gtsam::Vector3 pos_new = pos_prev + (vel_new + vel_prev) * dt / 2;

    m_current_estimate = gtsam::NavState(rot_new, pos_new, vel_new);

    // Update covariance with the preintegrated measurement covariance
    m_P_k = m_P_corr + std::visit([](auto &&x) -> Matrix15d { return x.preintMeasCov(); }, m_imu_preintegrated);
}

auto FactorGraphOptimiser::allocate_innovation_containers() -> void
{
    m_v = std::vector<Eigen::VectorXd>(m_N / 10, Eigen::VectorXd{});
    m_S = std::vector<Eigen::MatrixXd>(m_N / 10, Eigen::MatrixXd{});
}

auto FactorGraphOptimiser::push_back_innovation_containers(const std::uint64_t &num_locators, const Aiding &aided_by)
    -> void
{
    uint64_t innov_dim{};
    if (aided_by == Aiding::PARSFull)
    {
        /// TODO: When testing this will be two because we are ignoring the
        /// range measurements
        innov_dim = num_locators * 3;
    }
    else if (aided_by == Aiding::PARSRangeOnly)
    {
        innov_dim = num_locators;
    }
    else
    {
        /// NOTE: GNSS, but not really caring about that at the moment
        innov_dim = 3;
    }

    m_v.emplace_back(innov_dim);
    m_S.emplace_back(innov_dim, innov_dim);
}

auto FactorGraphOptimiser::add_IMU_factor_to_graph() -> void
{
    gtsam::CombinedImuFactor imu_factor = {X(m_correction_count - 1),
                                           V(m_correction_count - 1),
                                           X(m_correction_count),
                                           V(m_correction_count),
                                           B(m_correction_count - 1),
                                           B(m_correction_count),
                                           std::get<gtsam::PreintegratedCombinedMeasurements>(m_imu_preintegrated)};

    // fmt::print("({}) Adding combined IMU factor to graph...\n", idx);
    m_graph.add(boost::make_shared<gtsam::CombinedImuFactor>(imu_factor));
}

auto FactorGraphOptimiser::add_GNSS_factor_to_graph(const GNSS &gnss, const std::int64_t &idx) -> void
{
    gtsam::noiseModel::Diagonal::shared_ptr correction_noise =
        gtsam::noiseModel::Diagonal::Variances((gnss.R.diagonal()));

    gtsam::GPSFactor gps_factor{X(m_correction_count), gnss.z_pos.col(idx), correction_noise};
    m_graph.add(gps_factor);
}

auto FactorGraphOptimiser::add_GNSS_att_factor_to_graph(const GNSS &gnss, const std::int64_t &idx) -> void
{
    gtsam::Point3 z_lev = gnss.z_lev.col(idx).normalized();
    gtsam::Unit3 nZ{z_lev.x(), z_lev.y(), z_lev.z()};
    static const gtsam::Unit3 bRef{0, 1, 0};

    gtsam::noiseModel::Diagonal::shared_ptr att_noise =
        gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(2) << 0.05, 0.05).finished());

    gtsam::Pose3AttitudeFactor att_factor{X(m_correction_count), nZ, att_noise, bRef};

    m_graph.add(att_factor);
}

auto FactorGraphOptimiser::add_PARS_locator_prior_to_graph(const PARSBeacon &locator) -> void
{
    // Create key based on locator ID
    gtsam::Symbol beacon_key('L', locator.id);

    // The locator needs a prior factor if not present
    if (!m_beacon_keys.contains(beacon_key))
    {
        // Set up the locator's loose noise, which is necessary for GTSAM
        gtsam::noiseModel::Diagonal::shared_ptr loose_noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.0001);

        gtsam::PriorFactor<gtsam::Point3> prior{beacon_key, locator.origin, loose_noise};
        m_graph.add(boost::make_shared<gtsam::PriorFactor<gtsam::Point3>>(prior));
        m_initial_values.insert(beacon_key, locator.origin);
        m_beacon_keys.insert(beacon_key);
    }
}

auto FactorGraphOptimiser::add_PARS_range_factor_to_graph(const PARSBeacon &locator, const std::int64_t &idx,
                                                          bool simulate_erorrs) -> void
{
    // Fetch the information from the locator
    double z = locator.z.col(idx)[0];
    if (simulate_erorrs)
    {
        z += locator.e.col(idx)[0];
    }

    double cov = locator.R.coeff(0, 0);

    // Set up the range measurement's noise model
    gtsam::noiseModel::Diagonal::shared_ptr noise_model =
        gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(1) << std::sqrt(cov)).finished());

    auto robust_noise_model =
        m_use_tukey ? gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Tukey::Create(m_tukey_parameter),
                                                        noise_model)
                    : gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(m_huber_parameter),
                                                        noise_model);

    // Create the factor
    PARS::RangeFactor factor = m_use_m_estimators
                                   ? PARS::RangeFactor{X(m_correction_count), robust_noise_model, z, locator.origin}
                                   : PARS::RangeFactor{X(m_correction_count), noise_model, z, locator.origin};

    const auto &[should_add_to_graph, v, S] =
        m_outlier_rejection.check_measurement_factor(factor, m_current_estimate.pose(), m_P_k.block(3, 3, 3, 3), cov);

    static bool reject_with_natural_test = m_do_outlier_rejection && !m_use_m_estimators;
    if (reject_with_natural_test && !should_add_to_graph)
    {
        return;
    }

    m_graph.add(boost::make_shared<PARS::RangeFactor>(factor));
}

auto FactorGraphOptimiser::add_PARS_azimuth_factor_to_graph(const PARSBeacon &locator, const std::int64_t &idx,
                                                            bool simulate_erorrs) -> void
{
    // Fetch the information from the locator
    double z = locator.z.col(idx)[2];
    if (simulate_erorrs)
    {
        z += locator.e.col(idx)[2];
    }
    double cov = locator.R.coeff(2, 2);

    // Set up the range measurement's noise model
    gtsam::noiseModel::Diagonal::shared_ptr noise_model =
        gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(1) << cov).finished());

    auto robust_noise_model =
        m_use_tukey ? gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Tukey::Create(m_tukey_parameter),
                                                        noise_model)
                    : gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(m_huber_parameter),
                                                        noise_model);

    // Create the factor
    PARS::AzimuthFactor factor = m_use_m_estimators
                                     ? PARS::AzimuthFactor{X(m_correction_count), robust_noise_model, z, locator.origin}
                                     : PARS::AzimuthFactor{X(m_correction_count), noise_model, z, locator.origin};

    const auto &[should_add_to_graph, v, S] =
        m_outlier_rejection.check_measurement_factor(factor, m_current_estimate.pose(), m_P_k.block(3, 3, 3, 3), cov);

    static bool reject_with_natural_test = m_do_outlier_rejection && !m_use_m_estimators;
    if (reject_with_natural_test && !should_add_to_graph)
    {
        return;
    }

    m_graph.add(boost::make_shared<PARS::AzimuthFactor>(factor));
}

auto FactorGraphOptimiser::add_PARS_elevation_factor_to_graph(const PARSBeacon &locator, const std::int64_t &idx,
                                                              bool simulate_erorrs) -> void
{
    // Fetch the information from the locator
    double z = locator.z.col(idx)[1];
    if (simulate_erorrs)
    {
        z += locator.e.col(idx)[1];
    }
    double cov = locator.R.coeff(1, 1);

    // Set up the range measurement's noise model
    gtsam::noiseModel::Diagonal::shared_ptr noise_model =
        gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(1) << cov).finished());

    auto robust_noise_model =
        m_use_tukey ? gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Tukey::Create(m_tukey_parameter),
                                                        noise_model)
                    : gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(m_huber_parameter),
                                                        noise_model);

    // Create the factor
    PARS::ElevationFactor factor =
        m_use_m_estimators ? PARS::ElevationFactor{X(m_correction_count), robust_noise_model, z, locator.origin}
                           : PARS::ElevationFactor{X(m_correction_count), noise_model, z, locator.origin};

    const auto &[should_add_to_graph, v, S] =
        m_outlier_rejection.check_measurement_factor(factor, m_current_estimate.pose(), m_P_k.block(3, 3, 3, 3), cov);

    static bool reject_with_natural_test = m_do_outlier_rejection && !m_use_m_estimators;
    if (reject_with_natural_test && !should_add_to_graph)
    {
        return;
    }

    m_graph.add(boost::make_shared<PARS::ElevationFactor>(factor));
}

auto FactorGraphOptimiser::add_all_PARS_factors_to_graph(const PARSBeacon &locator, const std::int64_t &idx,
                                                         bool simulate_erorrs) -> void
{
    add_PARS_range_factor_to_graph(locator, idx, simulate_erorrs);
    add_PARS_azimuth_factor_to_graph(locator, idx, simulate_erorrs);
    add_PARS_elevation_factor_to_graph(locator, idx, simulate_erorrs);
}

auto FactorGraphOptimiser::propagate_without_optimising() -> void
{
    /// TODO: Handle everything related to innovation and innovation covariance
    m_P_corr = m_P_k;
    m_P.push_back(m_P_k);
    m_prev_estimate = m_current_estimate;
}

auto FactorGraphOptimiser::optimise_isam2(const std::int64_t &idx, bool print_marginals) -> void
{
    m_isam2.update(m_graph, m_initial_values);
    m_result = m_isam2.calculateEstimate();

    m_P_X = m_isam2.marginalCovariance(X(m_correction_count));
    m_P_V = m_isam2.marginalCovariance(V(m_correction_count));
    m_P_B = m_isam2.marginalCovariance(B(m_correction_count));

    if (print_marginals)
    {
        gtsam::print(m_P_X, fmt::format("({}) Pose Covariance:", idx));
        gtsam::print(m_P_V, fmt::format("({}) Velocity Covariance:", idx));
        gtsam::print(m_P_B, fmt::format("({}) Bias Covariance:", idx));
    }

    m_graph.resize(0);
    m_initial_values.clear();
}

auto FactorGraphOptimiser::optimise_fixed_lag(const std::int64_t &idx, bool print_marginals) -> void
{

    // fmt::print("adding smoother timestamps\n");
    m_smoother_timestamps_maps[X(m_correction_count)] = m_output_time;
    m_smoother_timestamps_maps[V(m_correction_count)] = m_output_time;
    m_smoother_timestamps_maps[B(m_correction_count)] = m_output_time;

    m_fixed_lag_smoother.update(m_graph, m_initial_values, m_smoother_timestamps_maps);
    m_result = m_fixed_lag_smoother.calculateEstimate();

    m_P_X = m_fixed_lag_smoother.marginalCovariance(X(m_correction_count));
    m_P_V = m_fixed_lag_smoother.marginalCovariance(V(m_correction_count));
    m_P_B = m_fixed_lag_smoother.marginalCovariance(B(m_correction_count));

    if (print_marginals)
    {
        gtsam::print(m_P_X, fmt::format("({}) Pose Covariance:", idx));
        gtsam::print(m_P_V, fmt::format("({}) Velocity Covariance:", idx));
        gtsam::print(m_P_B, fmt::format("({}) Bias Covariance:", idx));
    }

    m_graph.resize(0);
    m_initial_values.clear();
    m_smoother_timestamps_maps.clear();
}

auto FactorGraphOptimiser::optimise_LM(const std::int64_t &idx, bool print_marginals) -> void
{
    gtsam::LevenbergMarquardtOptimizer optimizer(m_graph, m_initial_values);
    m_result = optimizer.optimize();

    gtsam::Marginals marginals{m_graph, m_result};
    gtsam::GaussianFactor::shared_ptr results = marginals.marginalFactor(X(m_correction_count));
    m_P_X = marginals.marginalCovariance(X(m_correction_count));
    m_P_V = marginals.marginalCovariance(V(m_correction_count));
    m_P_B = marginals.marginalCovariance(B(m_correction_count));

    if (print_marginals)
    {
        results->print();
        gtsam::print(m_P_X, fmt::format("({}) Pose Covariance:", idx));
        gtsam::print(m_P_V, fmt::format("({}) Velocity Covariance:", idx));
        gtsam::print(m_P_B, fmt::format("({}) Bias Covariance:", idx));
    }
}

auto FactorGraphOptimiser::optimise(const std::int64_t &idx, const OptimisationScheme &optimisation_scheme,
                                    bool print_marginals) -> void
{
    // fmt::print("({}) Insert prediction into values...\n", idx);
    m_initial_values.insert(X(m_correction_count), m_current_estimate.pose());
    m_initial_values.insert(V(m_correction_count), m_current_estimate.v());
    m_initial_values.insert(B(m_correction_count), m_prev_bias_estimate);

    // fmt::print("({}) Optimising...\n", idx);
    auto start_optimisation = std::chrono::system_clock::now();
    switch (optimisation_scheme)
    {
    case OptimisationScheme::ISAM2: {
        optimise_isam2(idx, print_marginals);
        break;
    }
    case OptimisationScheme::FixedLag: {
        optimise_fixed_lag(idx, print_marginals);
        break;
    }
    case OptimisationScheme::LevenbergMarquardt: {
        optimise_LM(idx, print_marginals);
        break;
    }
    default:
        fmt::print("If this appears, either a new unhandled scheme has been added "
                   "or something is very wrong\n");
    }

    m_P_k = Eigen::MatrixXd::Zero(15, 15);
    m_P_k.block<6, 6>(0, 0) = m_P_X;
    m_P_k.block<3, 3>(6, 6) = m_P_V;
    m_P_k.block<6, 6>(9, 9) = m_P_B;
    m_P_corr = m_P_k;
    m_P.push_back(m_P_k);
    // fmt::print("({}) Overriding preintegration and resetting prev_state...\n", idx);

    auto pose_corrected = m_result.at<gtsam::Pose3>(X(m_correction_count));
    const gtsam::Point3 &pos_corrected = pose_corrected.translation();
    const gtsam::Rot3 &rot_corrected = pose_corrected.rotation();
    const gtsam::Velocity3 &vel_corrected = m_result.at<gtsam::Vector3>(V(m_correction_count));

    m_prev_estimate = gtsam::NavState(rot_corrected, pos_corrected, vel_corrected);
    m_prev_bias_estimate = m_result.at<gtsam::imuBias::ConstantBias>(B(m_correction_count));

    // cout << "(" << i << ") Preintegration before reset \n";
    // imu_preintegrated_->print();

    // Reset the preintegration object
    std::visit([prev_bias = m_prev_bias_estimate](auto &&x) { x.resetIntegrationAndSetBias(prev_bias); },
               m_imu_preintegrated);

    // cout << "(" << i << ") Preintegration after reset \n";
    // imu_preintegrated_->print();
    auto end_optimisation = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_opt = end_optimisation - start_optimisation;
    // fmt::print("({}) Optmisation time elapsed: {} [s]\n", idx, elapsed_opt.count());
}

auto FactorGraphOptimiser::compute_errors(const std::int64_t &idx, const KinematicState &true_state, bool print_errors)
    -> void
{
    // Print out the position, orientation and velocity error for comparison +
    // bias values
    gtsam::Vector3 gtsam_position = m_prev_estimate.pose().translation();
    gtsam::Vector3 true_position = true_state.pos.col(idx);
    // fmt::print("True position: {} {} {}\n", true_position[0], true_position[1], true_position[2]);
    gtsam::Vector3 position_error = gtsam_position - true_position;
    m_current_position_error = position_error;

    m_position_estimate.push_back(gtsam_position);
    m_position_error.push_back(position_error);

    gtsam::Quaternion gtsam_quat = m_prev_estimate.pose().rotation().toQuaternion();
    gtsam::Quaternion true_quat = gtsam::Rot3::Quaternion(true_state.att.col(idx)[0], true_state.att.col(idx)[1],
                                                          true_state.att.col(idx)[2], true_state.att.col(idx)[3])
                                      .toQuaternion();

    gtsam::Rot3 true_att = gtsam::Rot3::Quaternion(true_state.att.col(idx)[0], true_state.att.col(idx)[1],
                                                   true_state.att.col(idx)[2], true_state.att.col(idx)[3]);

    gtsam::Rot3 est_att = m_prev_estimate.pose().rotation();

    // Quaternion quat_error           = gtsam_quat * true_quat.inverse();
    gtsam::Quaternion quat_error = gtsam_quat.inverse() * true_quat;
    quat_error.normalize();
    gtsam::Vector3 euler_angle_error{quat_error.x() * 2, quat_error.y() * 2, quat_error.z() * 2};
    gtsam::Rot3 test_error = gtsam::Rot3::Quaternion(quat_error.w(), quat_error.x(), quat_error.y(), quat_error.z());
    test_error.normalized();
    gtsam::Vector3 euler_angle_error_alt{test_error.roll(), test_error.pitch(), test_error.yaw()};

    gtsam::Vector3 euler_angle_error_alt_2{true_att.roll() - est_att.roll(), true_att.pitch() - est_att.pitch(),
                                           true_att.yaw() - est_att.yaw()};
    // ssa(ssa(true_att.yaw()) - est_att.yaw())};
    m_current_orientation_error = euler_angle_error_alt_2;

    m_orientation_estimate.push_back(m_prev_estimate.pose().rotation());
    // m_orientation_estimate.push_back(m_prev_estimate.pose().rotation());
    m_orientation_error.push_back(m_current_orientation_error);

    gtsam::Vector3 true_velocity = true_state.vel.col(idx);
    gtsam::Vector3 gtsam_velocity = m_prev_estimate.velocity();
    gtsam::Vector3 velocity_error = gtsam_velocity - true_velocity;
    m_current_velocity_error = velocity_error;

    m_velocity_estimate.push_back(gtsam_velocity);
    m_velocity_error.push_back(m_current_velocity_error);

    Eigen::Vector3d acc_error = (m_true_bias.accelerometer() - m_prev_bias_estimate.accelerometer()).transpose();
    Eigen::Vector3d gyro_error = (m_true_bias.gyroscope() - m_prev_bias_estimate.gyroscope()).transpose();

    /// NOTE: Done a bit differently to the other states
    Eigen::Vector3d acc_estimate = m_prev_bias_estimate.accelerometer().transpose();
    Eigen::Vector3d gyro_estimate = m_prev_bias_estimate.gyroscope().transpose();
    m_acc_bias_estimate.push_back(acc_estimate);
    m_acc_bias_error.push_back(acc_error);
    m_gyro_bias_estimate.push_back(gyro_estimate);
    m_gyro_bias_error.push_back(gyro_error);

    if (print_errors)
    {
        fmt::print(
            "({}) Pos err [m]: ({:.2f}, {:.2f}, {:.2f}) - Att err [deg]: ({:.2f}, {:.2f}, {:.2f})  - Vel err [m/s]: "
            "({:.2f}, {:.2f}, {:.2f})\n",
            idx, m_current_position_error.x(), m_current_position_error.y(), m_current_position_error.z(),
            m_current_orientation_error.x() * rad2deg(1), m_current_orientation_error.y() * rad2deg(1),
            m_current_orientation_error.z() * rad2deg(1), m_current_velocity_error.x(), m_current_velocity_error.y(),
            m_current_velocity_error.z());
    }
}

auto FactorGraphOptimiser::export_data_to_csv(const std::string &filename) const -> void
{
    std::ofstream output_file(filename);
    if (!output_file.is_open())
    {
        std::cerr << "Error opening file: " << filename << '\n';
        return;
    }

    double t = 0.0;
    for (int i = 0; i < m_N; i++)
    {
        t += 0.01; // NOTE Placement before/after
        Matrix15d P = m_P[i];
        Eigen::Vector3d pos = m_position_estimate[i];
        Eigen::Vector3d vel = m_velocity_estimate[i];
        gtsam::Rot3 att = m_orientation_estimate[i];
        Eigen::Vector3d acc_bias = m_acc_bias_estimate[i];
        Eigen::Vector3d gyro_bias = m_gyro_bias_estimate[i];
        output_file << fmt::format("{},", t);
        output_file << fmt::format("{},{},{},", pos.x(), pos.y(), pos.z());
        output_file << fmt::format("{},{},{},", vel.x(), vel.y(), vel.z());
        output_file << fmt::format("{},{},{},", att.roll(), att.pitch(), att.yaw());
        output_file << fmt::format("{},{},{},", acc_bias.x(), acc_bias.y(), acc_bias.z());
        output_file << fmt::format("{},{},{},", gyro_bias.x(), gyro_bias.y(), gyro_bias.z());

        for (int row_idx = 0; row_idx < 15; row_idx++)
        {
            for (int col_idx = 0; col_idx < 15; col_idx++)
            {
                output_file << fmt::format("{},", P(row_idx, col_idx));
            }
        }
        output_file << "\n";
    }

    // Close the file when done
    output_file.close();
}

auto FactorGraphOptimiser::set_debug_true_state(gtsam::Vector3 pos, gtsam::Vector4 att, gtsam::Vector3 vel) -> void
{
    gtsam::Point3 p{pos};
    gtsam::Rot3 rot = gtsam::Rot3::Quaternion(att[0], att[1], att[2], att[3]);
    gtsam::Velocity3 v{vel};
    gtsam::Pose3 pose{rot, p};
    gtsam::NavState state{pose, v};

    m_current_estimate = state;
}
