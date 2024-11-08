#pragma once

#include <Eigen/Core>

void print_vector(const Eigen::VectorXd &v, const std::string &name);

auto rad2deg(double rad) -> double;

auto deg2rad(double deg) -> double;

auto ssa(double angle) -> double;

auto custom_mod(double a, double n) -> double;
