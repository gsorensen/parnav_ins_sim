#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/types.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace PARS
{
using gtsam::symbol_shorthand::X;

class ElevationFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3>
{
  public:
    ElevationFactor(gtsam::Key j, const gtsam::SharedNoiseModel &model, double elevation, const gtsam::Point3 &l)
        : gtsam::NoiseModelFactor1<gtsam::Pose3>{model, j}, m_z{elevation}, m_l{l}
    {
    }
    auto evaluateError(const gtsam::Pose3 &p, boost::optional<gtsam::Matrix &> H1 = boost::none) const
        -> gtsam::Vector override;
    [[nodiscard]] auto h_c(const gtsam::Pose3 &p) const -> double;
    [[nodiscard]] auto H_c(const gtsam::Pose3 &p) const -> gtsam::Matrix13;
    [[nodiscard]] auto compute_innovation(const gtsam::Pose3 &p, const gtsam::Matrix33 &pred_cov, double meas_cov) const
        -> std::tuple<double, double>;

  private:
    double m_z;
    gtsam::Point3 m_l;
};
} // namespace PARS
