#include "PARSRangeFactor.hpp"
#include <fmt/core.h>

namespace PARS
{
auto RangeFactor::evaluateError(const gtsam::Pose3 &p, boost::optional<gtsam::Matrix &> H1) const -> gtsam::Vector
{
    if (H1)
    {
        gtsam::Matrix13 J_c = H_c(p);
        gtsam::Matrix36 J_p = gtsam::Matrix(3, 6);
        gtsam::insertSub(J_p, p.rotation().matrix(), 0, 3);

        (*H1) = J_c * J_p;
    }

    gtsam::Vector1 error_vec{h_c(p) - m_z};

    return error_vec;
}

auto RangeFactor::compute_innovation(const gtsam::Pose3 &p, const gtsam::Matrix33 &pred_cov, double meas_cov) const
    -> std::tuple<double, double>
{
    gtsam::Matrix13 H = H_c(p);

    double v = h_c(p) - m_z;
    double S = H * pred_cov * H.transpose() + meas_cov;

    return {v, S};
}

auto RangeFactor::h_c(const gtsam::Pose3 &p) const -> double
{
    return (p.translation() - m_l).norm();
}

auto RangeFactor::H_c(const gtsam::Pose3 &p) const -> gtsam::Matrix13
{
    gtsam::Vector3 p_rb_n_hat = p.translation() - m_l;

    return p_rb_n_hat.transpose() / p_rb_n_hat.norm();
}
} // namespace PARS
