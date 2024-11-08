#include "PARSRangeFactor.hpp"
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

    double sigma_range = 1;
    std::normal_distribution ran_range{0.0, sigma_range};

    const gtsam::Point3 &true_pos = gtsam::Point3{0, 0, 0};
    const gtsam::Rot3 &true_rot = gtsam::Rot3::RzRyRx(0, 0, 25.0 * M_PI / 180.0);
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
    const gtsam::Point3 locator_one_origin{71.4, 71.4, 14};
    const gtsam::Point3 locator_two_origin{-71.4, 71.4, 5};
    const gtsam::Point3 locator_three_origin{71.4, -71.4, 8};
    const gtsam::Point3 locator_four_origin{-71.4, -71.4, 12};

    gtsam::noiseModel::Diagonal::shared_ptr range_noise =
        gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(1) << sigma_range).finished());

    const gtsam::Vector3 diff1 = true_pos - locator_one_origin;
    const gtsam::Vector3 diff2 = true_pos - locator_two_origin;
    const gtsam::Vector3 diff3 = true_pos - locator_three_origin;
    const gtsam::Vector3 diff4 = true_pos - locator_four_origin;

    double z_range_one = diff1.norm();   // + ran_range(gen);
    double z_range_two = diff2.norm();   //+ ran_range(gen);
    double z_range_three = diff3.norm(); // + ran_range(gen);
    double z_range_four = diff4.norm();  // + ran_range(gen);

    graph.add(boost::make_shared<PARS::RangeFactor>(X(0), range_noise, z_range_one, locator_one_origin));
    graph.add(boost::make_shared<PARS::RangeFactor>(X(0), range_noise, z_range_two, locator_two_origin));
    graph.add(boost::make_shared<PARS::RangeFactor>(X(0), range_noise, z_range_three, locator_three_origin));
    graph.add(boost::make_shared<PARS::RangeFactor>(X(0), range_noise, z_range_four, locator_four_origin));

    // graph.add(
    //     boost::make_shared<PARS::RangeFactor>(X(0), range_noise, z_range_one + ran_range(gen), locator_one_origin));
    // graph.add(
    //     boost::make_shared<PARS::RangeFactor>(X(0), range_noise, z_range_one + ran_range(gen), locator_one_origin));
    // graph.add(
    //     boost::make_shared<PARS::RangeFactor>(X(0), range_noise, z_range_one + ran_range(gen), locator_one_origin));
    // graph.add(
    //     boost::make_shared<PARS::RangeFactor>(X(0), range_noise, z_range_one + ran_range(gen), locator_one_origin));
    // graph.add(
    //     boost::make_shared<PARS::RangeFactor>(X(0), range_noise, z_range_one + ran_range(gen), locator_one_origin));
    // graph.add(
    //     boost::make_shared<PARS::RangeFactor>(X(0), range_noise, z_range_two + ran_range(gen), locator_two_origin));
    // graph.add(
    //     boost::make_shared<PARS::RangeFactor>(X(0), range_noise, z_range_two + ran_range(gen), locator_two_origin));
    // graph.add(
    //     boost::make_shared<PARS::RangeFactor>(X(0), range_noise, z_range_two + ran_range(gen), locator_two_origin));
    // graph.add(
    //     boost::make_shared<PARS::RangeFactor>(X(0), range_noise, z_range_two + ran_range(gen), locator_two_origin));
    // graph.add(
    //     boost::make_shared<PARS::RangeFactor>(X(0), range_noise, z_range_two + ran_range(gen), locator_two_origin));
    // graph.add(
    //     boost::make_shared<PARS::RangeFactor>(X(0), range_noise, z_range_three + ran_range(gen),
    //     locator_three_origin));
    // graph.add(
    //     boost::make_shared<PARS::RangeFactor>(X(0), range_noise, z_range_three + ran_range(gen),
    //     locator_three_origin));
    // graph.add(
    //     boost::make_shared<PARS::RangeFactor>(X(0), range_noise, z_range_three + ran_range(gen),
    //     locator_three_origin));
    // graph.add(
    //     boost::make_shared<PARS::RangeFactor>(X(0), range_noise, z_range_three + ran_range(gen),
    //     locator_three_origin));
    // graph.add(
    //     boost::make_shared<PARS::RangeFactor>(X(0), range_noise, z_range_three + ran_range(gen),
    //     locator_three_origin));
    // graph.add(
    //     boost::make_shared<PARS::RangeFactor>(X(0), range_noise, z_range_four + ran_range(gen),
    //     locator_four_origin));
    // graph.add(
    //     boost::make_shared<PARS::RangeFactor>(X(0), range_noise, z_range_four + ran_range(gen),
    //     locator_four_origin));
    // graph.add(
    //     boost::make_shared<PARS::RangeFactor>(X(0), range_noise, z_range_four + ran_range(gen),
    //     locator_four_origin));
    // graph.add(
    //     boost::make_shared<PARS::RangeFactor>(X(0), range_noise, z_range_four + ran_range(gen),
    //     locator_four_origin));
    // graph.add(
    //     boost::make_shared<PARS::RangeFactor>(X(0), range_noise, z_range_four + ran_range(gen),
    //     locator_four_origin));

    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_estimate);
    gtsam::Values result = optimizer.optimize();

    std::string output_str = fmt::format("Result with four PARS range factors with sigma {} m:\n", sigma_range);
    result.print(output_str);
    gtsam::Marginals marginals{graph, result};
    std::cout << marginals.marginalCovariance(X(0)) << "\n";

    return 0;
}
