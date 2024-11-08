#pragma once

#include "PARSAzimuthFactor.hpp"
#include "PARSElevationFactor.hpp"
#include "PARSRangeFactor.hpp"
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose3.h>
#include <variant>

class OutlierRejection
{
  public:
    using Factor = std::variant<PARS::RangeFactor, PARS::AzimuthFactor, PARS::ElevationFactor>;
    explicit OutlierRejection(const double &threshold) : m_threshold{threshold}
    {
    }

    [[nodiscard]] auto check_measurement_factor(const Factor &factor, const gtsam::Pose3 &pose,
                                                const gtsam::Matrix33 &cov_pos, const double &meas_cov) const
        -> std::tuple<bool, double, double>;

  private:
    double m_threshold;
};
