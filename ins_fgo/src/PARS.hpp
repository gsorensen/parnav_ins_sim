#pragma once

#include <Eigen/Core>
#include <gtsam/base/Matrix.h>
#include <gtsam/linear/NoiseModel.h>

struct PARSBeacon
{
    size_t id;
    Eigen::MatrixXd z;
    // Error
    Eigen::MatrixXd e;
    gtsam::Vector3 origin;
    double yaw = 0.0;
    gtsam::Matrix33 R;

    void set_meas_noise_cov(const double &sigma_range, const double &sigma_azimuth, const double &sigma_elevation);
} __attribute__((aligned(128)));
