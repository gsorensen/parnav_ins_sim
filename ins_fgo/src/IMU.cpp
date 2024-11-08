#include "IMU.hpp"

void IMU::set_acc_noise_cov(const double &accel_noise_sigma)
{
    params->setAccelerometerCovariance(gtsam::Matrix33::Identity() * pow(accel_noise_sigma, 2));
}

void IMU::set_gyro_noise_cov(const double &gyro_noise_sigma)
{
    params->setGyroscopeCovariance(gtsam::Matrix33::Identity() * pow(gyro_noise_sigma, 2));
}

void IMU::set_bias_cov(const double &accel_bias_rw_sigma, const double &gyro_bias_rw_sigma)
{
    gtsam::Matrix33 bias_acc_cov = gtsam::Matrix33::Identity() * pow(accel_bias_rw_sigma, 2);
    gtsam::Matrix33 bias_omega_cov = gtsam::Matrix33::Identity() * pow(gyro_bias_rw_sigma, 2);
    gtsam::Matrix66 bias_acc_omega_int = gtsam::Matrix::Identity(6, 6);
    bias_acc_omega_int.block<3, 3>(0, 0) = bias_acc_cov;
    bias_acc_omega_int.block<3, 3>(3, 3) = bias_omega_cov;

    params->setBiasAccCovariance(gtsam::Matrix33::Identity() * pow(accel_bias_rw_sigma, 2));
    params->setBiasOmegaCovariance(gtsam::Matrix33::Identity() * pow(gyro_bias_rw_sigma, 2));
    params->setBiasAccOmegaInit(bias_acc_omega_int);

    // params->biasAccCovariance = bias_acc_cov;     ///< continuous-time "Covariance" describing accelerometer
    //                                               ///< bias random walk
    // params->biasOmegaCovariance = bias_omega_cov; ///< continuous-time "Covariance" describing gyroscope
    //                                               ///< bias random walk
    //    params->biasAccOmegaInt = bias_acc_omega_int; ///< covariance of bias used as initial estimate.
}

void IMU::set_integration_error_cov(const double &integration_error_sigma)
{
    // gtsam::Matrix33 integration_error_cov =
    //     gtsam::Matrix33::Identity(3, 3) *
    //     pow(integration_error_sigma, 2); // error committed in integrating position from velocities
    // params->integrationCovariance = integration_error_cov;
    params->setIntegrationCovariance(gtsam::Matrix33::Identity() * pow(integration_error_sigma, 2));
}
