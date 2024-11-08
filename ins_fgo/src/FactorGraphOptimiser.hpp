#pragma once

#include "IMU.hpp"
#include "OutlierRejection.hpp"
#include "PARS.hpp"
#include "Types.hpp"
#include <Eigen/Core>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <variant>

enum class OptimisationScheme
{
    ISAM2,
    FixedLag,
    LevenbergMarquardt
};

enum class Aiding
{
    GNSS,
    PARSRangeOnly,
    PARSFull,
    GNSSPARS,
    Nothing
};

using Matrix15d = Eigen::Matrix<double, 15, 15>;

using PreintegratedMeasurement =
    std::variant<gtsam::PreintegratedImuMeasurements, gtsam::PreintegratedCombinedMeasurements>;

/// TODO: COMPUTE THE ERRORS

class FactorGraphOptimiser
{
  public:
    explicit FactorGraphOptimiser(const std::uint64_t &time_steps);
    auto increment_correction_count() -> void;
    auto initialise_preintegrated_IMU_measurement(const IMU &imu, bool use_combined_measurement) -> void;
    auto initialise_true_bias(const Eigen::Vector3d &initial_acc_bias, const Eigen::Vector3d &initial_gyro_bias)
        -> void;
    auto initialise_priors(const InitialStateEstimate &est) -> void;
    auto initialise_marginal_covariance() -> void;
    auto initialise_optimiser(const OptimisationScheme &optimisation_scheme, const double &fixed_lag) -> void;

    auto add_PARS_locator_prior_to_graph(const PARSBeacon &locator) -> void;
    auto add_GNSS_factor_to_graph(const GNSS &gnss, const std::int64_t &idx) -> void;
    auto add_GNSS_att_factor_to_graph(const GNSS &gnss, const std::int64_t &idx) -> void;

    auto add_PARS_range_factor_to_graph(const PARSBeacon &locator, const std::int64_t &idx,
                                        bool simulate_erorrs = false) -> void;
    auto add_PARS_azimuth_factor_to_graph(const PARSBeacon &locator, const std::int64_t &idx,
                                          bool simulate_erorrs = false) -> void;
    auto add_PARS_elevation_factor_to_graph(const PARSBeacon &locator, const std::int64_t &idx,
                                            bool simulate_erorrs = false) -> void;
    auto add_all_PARS_factors_to_graph(const PARSBeacon &locator, const std::int64_t &idx, bool simulate_erorrs = false)
        -> void;

    auto predict_state(const Eigen::Vector3d &f, const Eigen::Vector3d &w, const double &dt) -> void;
    auto allocate_innovation_containers() -> void;
    auto push_back_innovation_containers(const std::uint64_t &num_locators, const Aiding &aided_by) -> void;
    auto add_IMU_factor_to_graph() -> void;
    auto propagate_without_optimising() -> void;
    auto optimise(const std::int64_t &idx, const OptimisationScheme &optimisation_scheme, bool print_marginals) -> void;

    auto compute_errors(const std::int64_t &idx, const KinematicState &true_state, bool print_errors) -> void;

    auto export_data_to_csv(const std::string &filename) const -> void;

    auto set_debug_true_state(gtsam::Vector3 pos, gtsam::Vector4 att, gtsam::Vector3 vel) -> void;

    auto perform_outlier_rejection(bool state) -> void
    {
        m_do_outlier_rejection = state;
    };

    auto use_tukey(bool use_tukey) -> void
    {
        m_use_tukey = use_tukey;
    };
    auto use_m_estimators(bool state) -> void
    {
        m_use_m_estimators = state;
    };

  private:
    auto optimise_isam2(const std::int64_t &idx, bool print_marginals) -> void;
    auto optimise_fixed_lag(const std::int64_t &idx, bool print_marginals) -> void;
    auto optimise_LM(const std::int64_t &idx, bool print_marginals) -> void;

    std::uint64_t m_N;
    std::uint64_t m_correction_count = 0;
    double m_output_time = 0.0;
    gtsam::NonlinearFactorGraph m_graph;
    std::set<gtsam::Symbol> m_beacon_keys = {};

    gtsam::Values m_initial_values;
    gtsam::Values m_result;

    gtsam::NavState m_prev_estimate;
    gtsam::NavState m_current_estimate;
    gtsam::imuBias::ConstantBias m_prev_bias_estimate;
    gtsam::imuBias::ConstantBias m_true_bias;
    PreintegratedMeasurement m_imu_preintegrated;

    gtsam::ISAM2 m_isam2;
    gtsam::IncrementalFixedLagSmoother m_fixed_lag_smoother;
    gtsam::FixedLagSmoother::KeyTimestampMap m_smoother_timestamps_maps;
    Eigen::MatrixXd m_P_X;
    Eigen::MatrixXd m_P_V;
    Eigen::MatrixXd m_P_B;
    std::vector<Matrix15d> m_P = {};
    Matrix15d m_P_k = Eigen::MatrixXd::Zero(15, 15);
    Matrix15d m_P_corr = Eigen::MatrixXd::Zero(15, 15);

    std::vector<Eigen::VectorXd> m_v = {};
    std::vector<Eigen::MatrixXd> m_S = {};

    /// TODO: CLeanup this

    std::vector<Eigen::Vector3d> m_position_estimate = {};
    std::vector<Eigen::Vector3d> m_velocity_estimate = {};
    std::vector<gtsam::Rot3> m_orientation_estimate = {};
    std::vector<Eigen::Vector3d> m_acc_bias_estimate = {};
    std::vector<Eigen::Vector3d> m_gyro_bias_estimate = {};
    std::vector<Eigen::Vector3d> m_position_error = {};
    std::vector<Eigen::Vector3d> m_velocity_error = {};
    std::vector<Eigen::Vector3d> m_orientation_error = {};
    std::vector<Eigen::Vector3d> m_acc_bias_error = {};
    std::vector<Eigen::Vector3d> m_gyro_bias_error = {};
    Eigen::Vector3d m_current_position_error{};
    Eigen::Vector3d m_current_velocity_error{};
    Eigen::Vector3d m_current_orientation_error{};

    bool m_do_outlier_rejection = false;
    bool m_use_m_estimators = false;
    bool m_use_tukey = false;

    double m_huber_parameter = 1.345;
    double m_tukey_parameter = 4.6851;

    OutlierRejection m_outlier_rejection{3.841};
};
