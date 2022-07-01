/*
Extended Kalman Filter class by Sam Tabor, 2013.
* http://diydrones.com/forum/topics/autonomous-soaring
* Set up for identifying thermals of Gaussian form, but could be adapted to other
* purposes by adapting the equations for the jacobians.
*/

#pragma once

#include <AP_Math/matrixN.h>
#include "Filter/Kalman.h"

#include <array>
#include <cstddef>
#include <tuple>

class ExtendedKalmanFilter {
public:
    static constexpr const uint8_t N = 4;

    ExtendedKalmanFilter();

    void reset(const VectorN<float,N> &x, const std::array<float,N> &p, const std::array<float,N> &q, float r);
    void update(float z, float Px, float Py, float driftX, float driftY);

    float operator[](std::size_t position);

private:
    // Extended Kalman filter: working on float data types, four estimated
    // states (thermal velocity, thermal radius, thermal center North sUAV
    // offset, and thermal center East sUAV offset), one observed output
    // (vertical air velocity variometer), no control input, two additional
    // update arguments (positions), and two additional prediction arguments
    // (wind drift North sUAV offset and wind drift East sUAV offset).
    using Ekf =
        Kalman<float, 4, 1, 0, std::tuple<float, float>,
                               std::tuple<float, float>>;

    Ekf _ekf;
};
