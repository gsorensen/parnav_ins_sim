#include <algorithm>
#include <expected>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Quaternion.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/linearExceptions.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/PreintegrationParams.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

#include <chrono>
#include <stdexcept>
#include <string>

#include <fmt/core.h>
#include <variant>

#include "FactorGraphOptimiser.hpp"
#include "Types.hpp"
#include "utils.hpp"

auto aided_by_str(const Aiding &aided_by) -> const char *
{
    switch (aided_by)
    {
    case Aiding::GNSS:
        return "GNSS_pos";
    case Aiding::PARSRangeOnly:
        return "PARS_range_only";
    case Aiding::PARSFull:
        return "PARS_full";
    case Aiding::GNSSPARS:
        return "GNSS_PARS_full";
    case Aiding::Nothing:
        return "Nothing";
    }
}

auto scheme_str(const OptimisationScheme &scheme) -> const char *
{
    switch (scheme)
    {
    case OptimisationScheme::FixedLag:
        return "fixed_lag";
    case OptimisationScheme::ISAM2:
        return "isam2";
    case OptimisationScheme::LevenbergMarquardt:
        return "LM";
    }
}

auto get_filenames(const int &run, const Aiding &aided_by, const OptimisationScheme &optimisation_scheme,
                   const bool &is_running_otter) -> std::pair<std::string, std::string>
{
    /// NOTE: Change these to where data and results should be (absolute path
    /// with last forward slash)
    /// Results expect three subfolders to be present in the output folder: otter, standstill, dummy, depending on which
    /// program is run Furthermore, each subfolder should have a subfolder named "mekf" and "fgo_fixed_lag"
    static const std::string input_file_prefix = "/Users/ghms/ws/ntnu/parnav_ins_simulator/data/";
    //    static const std::string output_file_prefix = "/Users/ghms/ws/ntnu/parnav_ins_simulator/results/";
    static const std::string output_file_prefix = "/Volumes/T9/02/results/";

    std::string input_file = input_file_prefix;
    std::string output_file = output_file_prefix;

    const std::string aided = aided_by_str(aided_by);
    const std::string scheme = scheme_str(optimisation_scheme);

#ifdef OTTER
    input_file += fmt::format("otter_simulation_data_{:02d}_100Hz_noisy_biased_aided_at_10Hz_cpp.csv", run);
    output_file += fmt::format("otter/fgo_{}/{:02d}_100Hz_aided_at_10Hz_by_{}", scheme, run, aided);
#elifdef DUMMY
    input_file += fmt::format("simulation_data_{:02d}_100Hz_noisy_biased_aided_at_10Hz_cpp.csv", run);
    output_file += fmt::format("dummy/fgo_{}/{:02d}_100Hz_aided_at_10Hz_by_{}.csv", scheme, run, aided);
#elifdef SANITYCHECKING
    input_file += fmt::format("otter_simulation_data_{:02d}_100Hz_aided_at_10Hz_cpp.csv", run);
    output_file += fmt::format("fgo_{}/{:02d}_100Hz_aided_at_10Hz_by_{}.csv", scheme, run, aided);
#else
    input_file += fmt::format("standstill_simulation_data_{:02d}_100Hz_noisy_biased_aided_at_10Hz_cpp.csv", run);
    output_file += fmt::format("standstill/fgo_{}/{:02d}_100Hz_aided_at_10Hz_by_{}.csv", scheme, run, aided);
#endif

#ifdef USE_TUKEY
    output_file += fmt::format("_m_est_tukey.csv", scheme, run, aided);
#elifdef USE_HUBER
    output_file += fmt::format("_m_est_huber.csv", scheme, run, aided);
#elifdef OUTLIER_REJECTION
    output_file += fmt::format("_outlier_rejection.csv", scheme, run, aided);
#elifdef SIMULATE_FAULTS
    output_file += fmt::format("_sensor_fault.csv", scheme, run, aided);
#else
    output_file += fmt::format(".csv");
#endif

    return {input_file, output_file};
}

/// NOTE: Currently setting the tuning parameters manually here, for the values
/// that match the MEKF, could be an idea to store these in the CSV and parse
/// too
auto initialise_sensors(const RawSimulationData &sim_data, const SimulationParameters &sim_params)
    -> std::tuple<IMU, GNSS, std::vector<PARSBeacon>>
{
    double g0 = 9.81;

    double vrw = 0.07;
    double arw = 0.15;

    double bias_instability_acc = 0.05; // milli g
    double bias_instability_ars = 0.5;  // deg per hour
    double T_acc = 3600.0;
    double T_ars = 3600.0;

    double sigma_v = (vrw / 60.0);
    double sigma_q = (arw / 60.0) * (M_PI / 180.0);
    double sigma_b_acc = std::sqrt((2.0 / T_acc) * (bias_instability_acc * (g0 / 1000.0)));
    double sigma_b_ars = std::sqrt((2.0 / T_ars) * (bias_instability_ars / 3600.0) * (M_PI / 180.0));

    // noise scaling
    sigma_v *= 1;
    sigma_q *= 1;
    sigma_b_acc *= 1;
    sigma_b_ars *= 1;

    // Initialise the sensors and their measurements
    IMU imu = sim_data.parse_IMU_with_gravity(g0);
    imu.set_acc_noise_cov(sigma_v);
    imu.set_gyro_noise_cov(sigma_q);
    imu.set_bias_cov(sigma_b_acc, sigma_b_ars);
    imu.set_integration_error_cov(1e-10);

    GNSS gnss = sim_data.parse_GNSS_measurements();
    gnss.set_meas_noise_cov(1.5, 1.5, 3.0);

    double sigma_range = 5;
    double sigma_azimuth = 7 * deg2rad(1);
    double sigma_elevation = 7 * deg2rad(1);

    // Noise scaling
    sigma_range *= 1;
    sigma_azimuth *= 1;
    sigma_elevation *= 1;

    std::vector<PARSBeacon> locators = sim_data.parse_PARS_measurements(sim_params.num_locators);
    std::for_each(locators.begin(), locators.end(), [sigma_range, sigma_azimuth, sigma_elevation](auto &&locator) {
        locator.set_meas_noise_cov(sigma_range, sigma_azimuth, sigma_elevation);
    });

    return {imu, gnss, locators};
}

auto initialise_factor_graph_optimiser(const SimulationParameters &sim_params,
                                       const InitialStateEstimate &initial_state_estimate, const IMU &imu,
                                       const OptimisationScheme &optimisation_scheme,
                                       const bool &use_combined_measurement, const double &fixed_lag)
    -> FactorGraphOptimiser
{
    FactorGraphOptimiser optimiser{sim_params.N};
    optimiser.initialise_optimiser(optimisation_scheme, fixed_lag);
    optimiser.initialise_preintegrated_IMU_measurement(imu, use_combined_measurement);
    optimiser.initialise_true_bias(imu.acc_bias.col(0), imu.gyro_bias.col(0));
    optimiser.initialise_priors(initial_state_estimate);
    optimiser.initialise_marginal_covariance();
    //   optimiser.allocate_innovation_containers();

    return optimiser;
}

void run_optimisation(int run, const Aiding &aiding_scheme, const OptimisationScheme &optimisation_scheme,
                      bool run_otter, bool optimise, bool print_marginals, bool use_combined_measurement,
                      double fixed_lag)
{

    fmt::print("Starting run m = {}\n", run);

    const auto &[input_file, output_file] = get_filenames(run, aiding_scheme, optimisation_scheme, run_otter);

    // Parse the raw simulation data from a CSV file
    fmt::print("Reading data from file: {}\n", input_file);
    const RawSimulationData sim_data = read_simulation_data_from_CSV(input_file).value();

    // Fetch the simulation parameters, true state and initial estimates
    // from the Matlab system
    const SimulationParameters sim_params = sim_data.parse_simulation_parameters();
    const double dt = (1.0 / sim_params.freq);
    const KinematicState true_state = sim_data.parse_true_state();
    const InitialStateEstimate initial_state_estimate = sim_data.parse_initial_state_estimate();
    const auto &[imu, gnss, locators] = initialise_sensors(sim_data, sim_params);

    FactorGraphOptimiser optimiser = initialise_factor_graph_optimiser(
        sim_params, initial_state_estimate, imu, optimisation_scheme, use_combined_measurement, fixed_lag);

    const auto filtering_start_time = std::chrono::system_clock::now();

    optimiser.compute_errors(0, true_state, true);

#ifdef SIMULATE_FAULTS
    bool simulate_errors = true;
#else
    bool simulate_errors = false;
#endif

#ifdef OUTLIER_REJECTION
    optimiser.perform_outlier_rejection(true);
#endif

#ifdef USE_TUKEY
    optimiser.use_m_estimators(true);
    optimiser.use_tukey(true);
#elifdef USE_HUBER
    optimiser.use_m_estimators(true);
#endif
    //    fmt::print("Simulate errors: {}\n Outlier rejection: {}\n", simulate_errors, 0);

    std::uint64_t run_for = 30;
    run_for = sim_params.N; // Uncomment this line of you want to run for a
    // reduced numbed of iterations. Data won't be saved then
    for (int idx = 1; idx < run_for; idx++)
    {
        bool update_step = optimise && (idx + 1) % 10 == 0;

        optimiser.predict_state(imu.f.row(idx), imu.w.row(idx), dt);

        if (update_step)
        {
            optimiser.increment_correction_count();
            optimiser.add_IMU_factor_to_graph();

            Aiding active_aiding_scheme = idx < 15000 ? Aiding::GNSS : aiding_scheme;

            switch (active_aiding_scheme)
            {
            case Aiding::GNSS:
                optimiser.add_GNSS_factor_to_graph(gnss, idx);
                optimiser.add_GNSS_att_factor_to_graph(gnss, idx);
                break;
            case Aiding::PARSRangeOnly:
                std::for_each(locators.begin(), locators.end(), [&optimiser, idx, simulate_errors](auto &&locator) {
                    optimiser.add_PARS_range_factor_to_graph(locator, idx, simulate_errors);
                });
                break;
            case Aiding::PARSFull:
                std::for_each(locators.begin(), locators.end(), [&optimiser, idx, simulate_errors](auto &&locator) {
                    optimiser.add_all_PARS_factors_to_graph(locator, idx, simulate_errors);
                });

                break;
            case Aiding::GNSSPARS:
                optimiser.add_GNSS_factor_to_graph(gnss, idx);
                std::for_each(locators.begin(), locators.end(), [&optimiser, idx, simulate_errors](auto &&locator) {
                    optimiser.add_all_PARS_factors_to_graph(locator, idx, simulate_errors);
                });
                break;
            default:
                break;
            }

            optimiser.optimise(idx, optimisation_scheme, print_marginals);
        }
        else
        {
            optimiser.propagate_without_optimising();
        }

        // For speed, skip printing errors if it's in between update steps
        optimiser.compute_errors(idx, true_state, false);
    }

    if (run_for == sim_params.N)
    {
        optimiser.export_data_to_csv(output_file);
    }

    auto filtering_end_time = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = filtering_end_time - filtering_start_time;
    fmt::print("Elapsed time: {} [s] - Time horizon data: {} [s]\n", elapsed_seconds.count(),
               (static_cast<double>(sim_params.N) + 1) * (1.0 / sim_params.freq));
    fmt::print("Saved data to file: {}\n", output_file);

// Just to sanity check that we have the correct setting
#ifdef GTSAM_TANGENT_PREINTEGRATION
    fmt::print("Tangent preintegration\n");
#else
    fmt::print("Manifold preintegration\n");
#endif
}

#ifdef OTTER
const bool run_otter = true;
#else
const bool run_otter = false;
#endif

#ifdef FIXED_LAG
const auto optimisation_scheme = OptimisationScheme::FixedLag;
#elif ISAM
const auto optimisation_scheme = OptimisationScheme::ISAM2;
#elif LM
const auto optimisation_scheme = OptimisationScheme::LeveLevenbergMarquardt;
#endif

#ifdef RANGE_ONLY
const auto aiding_scheme = Aiding::PARSRangeOnly;
#elif FULL_PARS
const auto aiding_scheme = Aiding::PARSFull;
#elif GNSS_BENCHMARK
const auto aiding_scheme = Aiding::GNSS;
#elif SANITYCHECKING
const auto aiding_scheme = Aiding::Nothing;
#endif

#ifdef SANITYCHECKING
const bool optimise = false;
#else
const bool optimise = true;
#endif
const bool use_combined_measurement = true;
const bool print_marginals = false;
const double fixed_lag = 10.0; // fixed smoother lag
const int M = 100;

auto main(int /*argc*/, char * /*argv*/[]) -> int
{
    for (int run = 1; run <= M; run++)
    {
        /// Try except used because GTSAM under the hood may throw an exception
        try
        {
            run_optimisation(run, aiding_scheme, optimisation_scheme, run_otter, optimise, print_marginals,
                             use_combined_measurement, fixed_lag);
        }
        catch (gtsam::IndeterminantLinearSystemException &e)
        {
            std::cout << e.what() << '\n';
        }
        catch (std::invalid_argument &e)
        {
            std::cout << e.what() << '\n';
        }
        catch (std::runtime_error &e)
        {
            std::cout << e.what() << '\n';
        }
        catch (std::bad_variant_access &e)
        {
            std::cout << e.what() << '\n';
        }
        catch (std::bad_optional_access &e)
        {
            std::cout << e.what() << '\n';
        }
        catch (gtsam::InvalidMatrixBlock &e)
        {
            std::cout << e.what() << '\n';
        }
    }

    return 0;
}
