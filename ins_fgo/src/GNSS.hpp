#pragma once

#include <Eigen/Core>
#include <gtsam/base/Matrix.h>

struct GNSS
{
    Eigen::MatrixXd z_pos;
    Eigen::MatrixXd z_vel;
    Eigen::MatrixXd z_lev;
    gtsam::Matrix33 R;

    void set_meas_noise_cov(const double &sigmaN, const double &sigmaE, const double &sigmaD);
} __attribute__((aligned(128)));
