#pragma once

#include <Eigen/Core>
#include <gtsam/navigation/CombinedImuFactor.h>

struct IMU
{
    Eigen::MatrixXd f;
    Eigen::MatrixXd w;
    Eigen::MatrixXd acc_bias;
    Eigen::MatrixXd gyro_bias;
    boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> params;

    void set_acc_noise_cov(const double &accel_noise_sigma);
    void set_gyro_noise_cov(const double &gyro_noise_sigma);
    void set_bias_cov(const double &accel_bias_rw_sigma, const double &gyro_bias_rw_sigma);
    void set_integration_error_cov(const double &integration_error_sigma);
} __attribute__((aligned(128)));
