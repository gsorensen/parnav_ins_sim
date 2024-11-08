#pragma once

#include "GNSS.hpp"
#include "IMU.hpp"
#include "PARS.hpp"

#include <Eigen/Core>
#include <cstdint>
#include <fstream>
#include <iostream>

struct SimulationParameters
{
    std::uint16_t freq;
    std::uint16_t aiding_freq;
    std::uint8_t num_locators;
    Eigen::VectorXd t;
    std::uint64_t N;
} __attribute__((aligned(32)));

struct KinematicState
{
    Eigen::MatrixXd pos;
    Eigen::MatrixXd vel;
    Eigen::MatrixXd att;
    Eigen::MatrixXd acc_bias;
    Eigen::MatrixXd gyro_bias;
} __attribute__((aligned(128)));

struct InitialStateEstimate
{
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Vector4d att;
    Eigen::Vector3d acc_bias;
    Eigen::Vector3d gyro_bias;

    double A_pos;
    double A_vel;
    double A_att;
    double A_acc_bias;
    double A_gyro_bias;
} __attribute__((aligned(128)));

struct RawSimulationData
{
    std::uint64_t N;
    Eigen::MatrixXd data_matrix;

    [[nodiscard]] auto parse_simulation_parameters() const -> SimulationParameters;
    [[nodiscard]] auto parse_true_state() const -> KinematicState;
    [[nodiscard]] auto parse_GNSS_measurements() const -> GNSS;
    [[nodiscard]] auto parse_PARS_measurements(const std::uint16_t &num_locators) const -> std::vector<PARSBeacon>;
    [[nodiscard]] auto parse_initial_state_estimate() const -> InitialStateEstimate;
    [[nodiscard]] auto parse_IMU_with_gravity(const double &g) const -> IMU;
} __attribute__((aligned(32)));

[[nodiscard]] auto read_simulation_data_from_CSV(const std::string &filename) -> std::optional<RawSimulationData>;
