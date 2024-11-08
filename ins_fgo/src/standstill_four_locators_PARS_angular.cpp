#include "PARSAzimuthFactor.hpp"
#include "PARSElevationFactor.hpp"
#include "PARSRangeFactor.hpp"
#include "utils.hpp"
#include <Eigen/Core>
#include <fmt/core.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <random>

auto main() -> int
{
    using gtsam::symbol_shorthand::X; // pose (x, y, z, r, p, y)
    std::random_device rd{};
    std::mt19937 gen{rd()};

    const std::vector<double> sigma_vals = {5.0, 3.0, 1.0, 0.1};

    const bool add_noise = false;

    for (const auto &sigma_val : sigma_vals)
    {
        double sigma_azimuth = sigma_val;
        std::normal_distribution ran_azimuth{0.0, sigma_azimuth};

        double sigma_elevation = sigma_val;
        std::normal_distribution ran_elevation{0.0, sigma_elevation};

        const gtsam::Point3 &true_pos = gtsam::Point3{0, 0, 0};
        const gtsam::Rot3 &true_rot = gtsam::Rot3::RzRyRx(0, 0, 0.0 * M_PI / 180.0);
        const gtsam::Pose3 &true_pose{true_rot, true_pos};

        // Initialise prior
        const gtsam::Point3 &predicted_pos{10, 10, 2};
        const gtsam::Rot3 &predicted_rot = gtsam::Rot3::Identity();
        const gtsam::Pose3 &predicted_pose = gtsam::Pose3{predicted_rot, predicted_pos};
        gtsam::noiseModel::Diagonal::shared_ptr prior_pose_model =
            gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.01, 0.01, 0.01, 10, 10, 5).finished());
        gtsam::PriorFactor<gtsam::Pose3> prior_pose{X(0), predicted_pose, prior_pose_model};

        // Initialise graph
        gtsam::NonlinearFactorGraph graph{};
        graph.addPrior(X(0), predicted_pose, prior_pose_model);

        gtsam::Values initial_estimate{};
        initial_estimate.insert(X(0), predicted_pose);
        initial_estimate.print("Status before optimisation:\n");
        gtsam::Marginals initial_marginals{graph, initial_estimate};
        std::cout << initial_marginals.marginalCovariance(X(0)) << "\n";

        // Initialise locator origin and noise models
        const gtsam::Point3 locator_one_origin{81.4, 71.4, 16};
        const gtsam::Point3 locator_two_origin{-61.4, 91.4, -10};
        const gtsam::Point3 locator_three_origin{21.4, -71.4, 8};
        const gtsam::Point3 locator_four_origin{-61.4, -71.4, -10};

        gtsam::noiseModel::Diagonal::shared_ptr azimuth_noise =
            gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(1) << sigma_azimuth).finished());
        gtsam::noiseModel::Diagonal::shared_ptr elevation_noise =
            gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(1) << sigma_elevation).finished());

        const gtsam::Vector3 diff1 = true_pos - locator_one_origin;
        const gtsam::Vector3 diff2 = true_pos - locator_two_origin;
        const gtsam::Vector3 diff3 = true_pos - locator_three_origin;
        const gtsam::Vector3 diff4 = true_pos - locator_four_origin;

        double z_azi_one = ssa(std::atan2(diff1.y(), diff1.x()) + ran_azimuth(gen) * static_cast<double>(add_noise));
        double z_azi_two = ssa(std::atan2(diff2.y(), diff2.x()) + ran_azimuth(gen) * static_cast<double>(add_noise));
        double z_azi_three = ssa(std::atan2(diff3.y(), diff3.x()) + ran_azimuth(gen) * static_cast<double>(add_noise));
        double z_azi_four = ssa(std::atan2(diff4.y(), diff4.x()) + ran_azimuth(gen) * static_cast<double>(add_noise));

        double z_elev_one =
            ssa(std::atan2(-diff1.z(), diff1.head(2).norm()) + ran_elevation(gen) * static_cast<double>(add_noise));
        double z_elev_two =
            ssa(std::atan2(-diff2.z(), diff2.head(2).norm()) + ran_elevation(gen) * static_cast<double>(add_noise));
        double z_elev_three =
            ssa(std::atan2(-diff3.z(), diff3.head(2).norm()) + ran_elevation(gen) * static_cast<double>(add_noise));
        double z_elev_four =
            ssa(std::atan2(-diff4.z(), diff4.head(2).norm()) + ran_elevation(gen) * static_cast<double>(add_noise));

        graph.add(boost::make_shared<PARS::AzimuthFactor>(X(0), azimuth_noise, z_azi_one, locator_one_origin));
        graph.add(boost::make_shared<PARS::AzimuthFactor>(X(0), azimuth_noise, z_azi_two, locator_two_origin));
        graph.add(boost::make_shared<PARS::AzimuthFactor>(X(0), azimuth_noise, z_azi_three, locator_three_origin));
        graph.add(boost::make_shared<PARS::AzimuthFactor>(X(0), azimuth_noise, z_azi_four, locator_four_origin));

        graph.add(boost::make_shared<PARS::ElevationFactor>(X(0), elevation_noise, z_elev_one, locator_one_origin));
        graph.add(boost::make_shared<PARS::ElevationFactor>(X(0), elevation_noise, z_elev_two, locator_two_origin));
        graph.add(boost::make_shared<PARS::ElevationFactor>(X(0), elevation_noise, z_elev_three, locator_three_origin));
        graph.add(boost::make_shared<PARS::ElevationFactor>(X(0), elevation_noise, z_elev_four, locator_four_origin));

        gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_estimate);
        gtsam::Values result = optimizer.optimize();

        std::string output_str = fmt::format("Result with four PARS angular factors pairs with sigma {}  and {} rad:\n",
                                             sigma_azimuth, sigma_elevation);
        result.print(output_str);
        gtsam::Marginals marginals{graph, result};

        auto newpose = result.at<gtsam::Pose3>(X(0));
        gtsam::Point3 t = newpose.translation();
        gtsam::Matrix66 P = marginals.marginalCovariance(X(0));

        std::cout << P << "\n";

        double N3sigma = 3 * sqrt(P.coeff(4, 4));
        double E3sigma = 3 * sqrt(P.coeff(3, 3));
        double D3sigma = 3 * sqrt(P.coeff(5, 5));

        fmt::print("North estimate {:.4f} should be within 3sigma bounds +/- {:.4f} ({})\n", t[1], N3sigma,
                   t[1] > -N3sigma && t[1] < N3sigma);
        fmt::print("East estimate {:.4f} should be within 3sigma bounds +/- {:.4f} ({})\n", t[0], E3sigma,
                   t[0] > -E3sigma && t[0] < E3sigma);
        fmt::print("Down estimate {:.4f} should be within 3sigma bounds +/- {:.4f} ({})\n", t[2], D3sigma,
                   t[2] > -D3sigma && t[2] < D3sigma);
    }
    return 0;
}
