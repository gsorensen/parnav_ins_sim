#include "PARSAzimuthFactor.hpp"
#include "utils.hpp"

#include <cmath>
#include <fmt/core.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Quaternion.h>

namespace PARS
{
auto AzimuthFactor::evaluateError(const gtsam::Pose3 &p, boost::optional<gtsam::Matrix &> H1) const -> gtsam::Vector
{
    if (H1)
    {
        gtsam::Matrix13 J_c = H_c(p);
        gtsam::Matrix36 J_p = gtsam::Matrix(3, 6);
        gtsam::insertSub(J_p, p.rotation().matrix(), 0, 3);

        (*H1) = J_c * J_p;
    }

    gtsam::Vector1 error_vec{ssa(h_c(p) - ssa(m_z))};

    return error_vec;
}

auto AzimuthFactor::compute_innovation(const gtsam::Pose3 &p, const gtsam::Matrix33 &pred_cov, double meas_cov) const
    -> std::tuple<double, double>
{
    gtsam::Matrix13 H = H_c(p);

    double v = ssa(h_c(p) - ssa(m_z));
    double S = H * pred_cov * H.transpose() + meas_cov;

    return {v, S};
}

auto AzimuthFactor::h_c(const gtsam::Pose3 &p) const -> double
{
    gtsam::Vector3 p_rb_n_hat = p.translation() - m_l;

    return ssa(std::atan2(p_rb_n_hat.y(), p_rb_n_hat.x()));
}

auto AzimuthFactor::H_c(const gtsam::Pose3 &p) const -> gtsam::Matrix13
{
    const gtsam::Matrix33 &R_hat = p.rotation().matrix();
    gtsam::Vector3 p_rb_b_hat = R_hat.transpose() * (p.translation() - m_l);
    gtsam::Vector3 p_rb_n_hat = R_hat * p_rb_b_hat;
    double scale = 1.0 / (p_rb_n_hat.x() * p_rb_n_hat.x() + p_rb_n_hat.y() * p_rb_n_hat.y());
    gtsam::Matrix13 J_c = scale * (gtsam::Matrix(1, 3) << -p_rb_n_hat.y(), p_rb_n_hat.x(), 0.0).finished();

    return J_c;
}
} // namespace PARS
