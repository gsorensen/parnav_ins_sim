#include "Types.hpp"

auto RawSimulationData::parse_simulation_parameters() const -> SimulationParameters
{
    SimulationParameters params{};
    params.N = N;
    params.freq = static_cast<std::uint16_t>(data_matrix.col(30).transpose()[0]);
    params.aiding_freq = static_cast<std::uint16_t>(data_matrix.col(31).transpose()[0]);
    params.t = data_matrix.col(23).transpose();
    params.num_locators = static_cast<std::uint16_t>(data_matrix.col(53).transpose()[0]);

    return params;
}

auto RawSimulationData::parse_true_state() const -> KinematicState
{
    KinematicState state = {.pos = data_matrix.block(0, 1, N, 3).transpose(),
                            .vel = data_matrix.block(0, 4, N, 3).transpose(),
                            .att = data_matrix.block(0, 7, N, 4).transpose(),
                            .acc_bias = data_matrix.block(0, 17, N, 3).transpose(),
                            .gyro_bias = data_matrix.block(0, 20, N, 3).transpose()};

    return state;
}

auto RawSimulationData::parse_GNSS_measurements() const -> GNSS
{
    size_t lev_start_idx = data_matrix.cols() - 3;
    GNSS gnss = {.z_pos = data_matrix.block(0, 24, N, 3).transpose(),
                 .z_vel = data_matrix.block(0, 27, N, 3).transpose(),
                 .z_lev = data_matrix.block(0, lev_start_idx, N, 3).transpose(),
                 .R = gtsam::Matrix::Identity(3, 3)};

    return gnss;
}

auto RawSimulationData::parse_PARS_measurements(const std::uint16_t &num_locators) const -> std::vector<PARSBeacon>
{
    std::vector<PARSBeacon> locators{};

    for (size_t idx = 0; idx < num_locators; idx++)
    {
        size_t z_start_idx = 54 + 3 * idx;
        size_t origin_start_idx = 54 + num_locators * 3 + 3 * idx;
        size_t e_start_idx = 54 + num_locators * 3 + num_locators * 3 + 3 * idx;
        Eigen::MatrixXd z = data_matrix.block(0, z_start_idx, N, 3).transpose();
        Eigen::MatrixXd e = data_matrix.block(0, e_start_idx, N, 3).transpose();
        gtsam::Matrix33 R = gtsam::Matrix33::Identity();
        Eigen::MatrixXd origins = data_matrix.block(0, origin_start_idx, N, 3);
        Eigen::Vector3d origin = origins.row(0);

        PARSBeacon locator{.id = idx, .z = z, .e = e, .origin = origin, .R = R};
        locators.push_back(locator);
    }

    return locators;
}

auto RawSimulationData::parse_initial_state_estimate() const -> InitialStateEstimate
{
    InitialStateEstimate initial_estimate{
        .pos = data_matrix.block(0, 33, 1, 3).transpose(),
        .vel = data_matrix.block(0, 37, 1, 3).transpose(),
        .att = data_matrix.block(0, 41, 1, 4).transpose(),
        .acc_bias = data_matrix.block(0, 46, 1, 3).transpose(),
        .gyro_bias = data_matrix.block(0, 50, 1, 3).transpose(),
        .A_pos = data_matrix.col(32)[0],
        .A_vel = data_matrix.col(36)[0],
        .A_att = data_matrix.col(40)[0],
        .A_acc_bias = data_matrix.col(45)[0],
        .A_gyro_bias = data_matrix.col(49)[0],
    };

    return initial_estimate;
}

auto RawSimulationData::parse_IMU_with_gravity(const double &g) const -> IMU
{
    IMU imu{
        .f = data_matrix.block(0, 11, N, 3),
        .w = data_matrix.block(0, 14, N, 3),
        .acc_bias = data_matrix.block(0, 17, N, 3).transpose(),
        .gyro_bias = data_matrix.block(0, 20, N, 3).transpose(),
        .params = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedD(g),
    };

    /// TODO: Consider parsing all the tuning paramts from C++ here too

    return imu;
}

auto read_simulation_data_from_CSV(const std::string &filename) -> std::optional<RawSimulationData>
{
    std::ifstream data(filename);

    if (!data)
    {
        std::cerr << "Error: Could not open file " << filename << "\n";
        return std::nullopt;
    }
    std::string line;
    std::vector<double> values;
    int rows = 0;
    int testrows = 0;
    while (std::getline(data, line))
    {
        std::stringstream lineStream(line);
        std::string cell;
        while (std::getline(lineStream, cell, ','))
        {
            values.push_back(std::stod(cell));
            ++testrows;
        }
        ++rows;
    }

    RawSimulationData d{};
    d.data_matrix = Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        values.data(), rows, values.size() / rows);
    d.N = rows;

    return d;
}
