#include "OutlierRejection.hpp"

[[nodiscard]] auto OutlierRejection::check_measurement_factor(const Factor &factor, const gtsam::Pose3 &pose,
                                                              const gtsam::Matrix33 &cov_pos,
                                                              const double &meas_cov) const
    -> std::tuple<bool, double, double>
{
    const auto &[v, S] = std::visit(
        [pose, cov_pos, meas_cov](auto &&x) -> std::tuple<double, double> {
            return x.compute_innovation(pose, cov_pos, meas_cov);
        },
        factor);

    double natural_test = std::pow(v, 2) / S;
    bool add_factor = natural_test <= m_threshold;

    return {add_factor, v, S};
}
