#include "GNSS.hpp"

void GNSS::set_meas_noise_cov(const double &sigmaN, const double &sigmaE, const double &sigmaD)
{
    R = (gtsam::Vector(3) << sigmaN * sigmaN, sigmaE * sigmaE, sigmaD * sigmaD).finished().asDiagonal();
}
