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
    //
    //
    std::random_device rd{};
    std::mt19937 gen{rd()};

    std::normal_distribution ran_range{0.0, 5.0};
    std::normal_distribution ran_azimuth{0.0, 0.02};
    std::normal_distribution ran_elevation{0.0, 0.02};

    const gtsam::Point3 &true_pos = gtsam::Point3{0, 0, 0};
    const gtsam::Rot3 &true_rot = gtsam::Rot3::RzRyRx(0, 0, 25.0 * M_PI / 180.0);
    const gtsam::Pose3 &true_pose{true_rot, true_pos};

    // Initialise prior
    const gtsam::Point3 &predicted_pos{10, 10, 2};
    const gtsam::Rot3 &predicted_rot = gtsam::Rot3::Identity();
    const gtsam::Pose3 &predicted_pose = gtsam::Pose3{predicted_rot, predicted_pos};
    gtsam::noiseModel::Diagonal::shared_ptr prior_pose_model =
        gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.01, 0.01, 0.01, 3, 3, 3).finished());
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
    const gtsam::Point3 locator_origin{71.4, 71.4, 10};
    gtsam::noiseModel::Diagonal::shared_ptr range_noise =
        gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(1) << 5.0).finished());
    gtsam::noiseModel::Diagonal::shared_ptr azimuth_noise =
        gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(1) << 0.02).finished());
    gtsam::noiseModel::Diagonal::shared_ptr elevation_noise =
        gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(1) << 0.02).finished());

    // Create a simple set of PARS measurements
    const gtsam::Vector3 diff = true_pos - locator_origin;
    double z_range = diff.norm() + ran_range(gen);
    double z_azimuth = ssa(std::atan2(diff.y(), diff.x()) + ran_azimuth(gen));
    double z_elevation = ssa(std::atan2(-diff.z(), diff.head(2).norm()) + ran_elevation(gen));

    graph.add(boost::make_shared<PARS::RangeFactor>(X(0), range_noise, z_range, locator_origin));
    graph.add(boost::make_shared<PARS::AzimuthFactor>(X(0), azimuth_noise, z_azimuth, locator_origin));
    graph.add(boost::make_shared<PARS::ElevationFactor>(X(0), elevation_noise, z_elevation, locator_origin));

    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_estimate);
    gtsam::Values result = optimizer.optimize();
    //    result.print("Final Result:\n");

    result.print("Result with one set of full PARS factors:\n");
    gtsam::Marginals marginals{graph, result};
    std::cout << marginals.marginalCovariance(X(0)) << "\n";

    return 0;
}
