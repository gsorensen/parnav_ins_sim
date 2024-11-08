#include "PARS.hpp"
#include <iostream>

void PARSBeacon::set_meas_noise_cov(const double &sigma_range, const double &sigma_azimuth,
                                    const double &sigma_elevation)
{
    R = (gtsam::Vector(3) << sigma_range * sigma_range, sigma_elevation * sigma_elevation,
         sigma_azimuth * sigma_azimuth)
            .finished()
            .asDiagonal();

    std::cout << R << "\n";
}
