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

    std::normal_distribution ranx{0.0, 1.5};
    std::normal_distribution rany{0.0, 1.5};
    std::normal_distribution ranz{0.0, 3.0};

    const gtsam::Point3 &true_pos = gtsam::Point3{0, 0, 0};
    const gtsam::Rot3 &true_rot = gtsam::Rot3::RzRyRx(0, 0, 25.0 * M_PI / 180.0);
    const gtsam::Pose3 &true_pose{true_rot, true_pos};
    // const gtsam::Point3 &locator_origin = (gtsam::Vector3() << 71.4, 71.4, 10).finished();

    // Initialise prior
    const gtsam::Point3 &predicted_pos{10, 10, 2};
    const gtsam::Rot3 &predicted_rot = gtsam::Rot3::Identity();
    const gtsam::Pose3 &predicted_pose = gtsam::Pose3{predicted_rot, predicted_pos};
    gtsam::noiseModel::Diagonal::shared_ptr prior_pose_model =
        gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.01, 0.01, 0.01, 3, 3, 3).finished());
    gtsam::PriorFactor<gtsam::Pose3> prior_pose{X(0), predicted_pose, prior_pose_model};

    // Create a simple measurement
    gtsam::Vector3 z1 = true_pos;
    gtsam::Vector3 z2 = true_pos;
    gtsam::Vector3 z3 = true_pos;
    gtsam::Vector3 z4 = true_pos;
    gtsam::noiseModel::Diagonal::shared_ptr gps_noise =
        gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) << 1.5, 1.5, 3).finished());
    z1[0] += ranx(gen);
    z1[1] += rany(gen);
    z1[2] += ranz(gen);
    z2[0] += ranx(gen);
    z2[1] += rany(gen);
    z2[2] += ranz(gen);

    z3[0] += ranx(gen);
    z3[1] += rany(gen);
    z3[2] += ranz(gen);

    z4[0] += ranx(gen);
    z4[1] += rany(gen);
    z4[2] += ranz(gen);

    std::cout << "z1: " << z1 << "\n";
    std::cout << "z2: " << z2 << "\n";
    std::cout << "z3: " << z3 << "\n";
    std::cout << "z4: " << z4 << "\n";

    // Create a GPS factor
    gtsam::GPSFactor gps_factor1{X(0), z1, gps_noise};
    gtsam::GPSFactor gps_factor2{X(0), z2, gps_noise};
    gtsam::GPSFactor gps_factor3{X(0), z3, gps_noise};
    gtsam::GPSFactor gps_factor4{X(0), z4, gps_noise};

    // Initialise graph
    gtsam::NonlinearFactorGraph graph{};
    graph.addPrior(X(0), predicted_pose, prior_pose_model);

    gtsam::Values initial_estimate{};
    initial_estimate.insert(X(0), predicted_pose);
    initial_estimate.print("Status before optimisation:\n");
    gtsam::Marginals initial_marginals{graph, initial_estimate};
    std::cout << initial_marginals.marginalCovariance(X(0)) << "\n";

    graph.add(gps_factor1);
    graph.add(gps_factor2);
    graph.add(gps_factor3);
    graph.add(gps_factor4);

    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_estimate);
    gtsam::Values result = optimizer.optimize();
    //    result.print("Final Result:\n");

    result.print("Result with GPS factors:\n");
    gtsam::Marginals marginals{graph, result};
    std::cout << marginals.marginalCovariance(X(0)) << "\n";

    return 0;
}
