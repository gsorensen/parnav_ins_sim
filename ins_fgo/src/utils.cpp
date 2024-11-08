#include "utils.hpp"
#include <fmt/core.h>
#include <iostream>

void print_vector(const Eigen::VectorXd &vec, const std::string &name)
{
    std::cout << name << " | X: " << vec.x() << " Y: " << vec.y() << " Z: " << vec.z() << "\n";
}

auto rad2deg(double rad) -> double
{
    return rad * 180 / M_PI;
}

auto deg2rad(double deg) -> double
{
    return deg / 180 * M_PI;
}

auto custom_mod(double a, double n) -> double
{
    return a - floor(a / n) * n;
}

auto ssa(double angle) -> double
{
    return custom_mod(angle + M_PI, 2 * M_PI) - M_PI;
}
