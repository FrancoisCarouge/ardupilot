#pragma once

#include <AP_Common/AP_Common.h>

#include <stdint.h>

/*
* This class implements a simple 1-D Kalman Filter to estimate the Relative body frame position of the lading target and its relative velocity
* position and velocity of the target is predicted using delta velocity
* The predictions are corrected periodically using the landing target sensor(or camera)
*/
class PosVelEKF {
public:
    PosVelEKF();
    ~PosVelEKF();

    CLASS_NO_COPY(PosVelEKF);

    // Initialize the covariance and state matrix
    // This is called when the landing target is located for the first time or it was lost, then relocated
    void init(float pos, float posVar, float vel, float velVar);

    // This functions runs the Prediction Step of the EKF
    // This is called at 400 hz
    void predict(float dt, float dVel, float dVelNoise);

    // fuse the new sensor measurement into the EKF calculations
    // This is called whenever we have a new measurement available
    void fusePos(float pos, float posVar);

    // Get the EKF state position
    float getPos() const;

    // Get the EKF state velocity
    float getVel() const;

    // get the normalized innovation squared
    float getPosNIS(float pos, float posVar);

private:
    // the typed Kalman filter, defined in PosVelEKF.cpp to keep its expensive
    // headers out of this one, and constructed in this storage on the first
    // init(), not with this object: no heap, and not during static
    // initialization, whose order the filter's default values depend on
    struct Filter;
    static constexpr size_t filter_size = 120;
    alignas(8) uint8_t _storage[filter_size];
    Filter *_filter = nullptr;
};
