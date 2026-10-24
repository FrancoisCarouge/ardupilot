/*
Extended Kalman Filter class by Sam Tabor, 2013.
* http://diydrones.com/forum/topics/autonomous-soaring
* Set up for identifying thermals of Gaussian form, but could be adapted to other
* purposes by adapting the equations for the jacobians.
*/

#pragma once

#include <stdint.h>

#include <AP_Common/AP_Common.h>

class ExtendedKalmanFilter {
public:
    ExtendedKalmanFilter(void) {}

    CLASS_NO_COPY(ExtendedKalmanFilter);

    static constexpr const uint8_t N = 4;

    // state estimate: thermal strength (m/s), radius (m), north and east
    // position (m)
    float X[N] {};

    // reset the state to x, with the diagonal estimate covariance p and
    // process covariance q in the squared state units, and the measurement
    // variance r in (m/s)^2
    void reset(const float x[N], const float p[N], const float q[N], float r);

    // predict with the wind drift (m), then correct with the vertical air
    // velocity z (m/s) measured at the aircraft position Px, Py (m)
    void update(float z, float Px, float Py, float driftX, float driftY);

private:
    // the typed filter, defined in ExtendedKalmanFilter.cpp to keep its
    // expensive headers out of this one
    struct Thermal;
    Thermal *_thermal = nullptr;
};
