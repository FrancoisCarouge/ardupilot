#pragma once

#include "Filter/Kalman.h"

#include <tuple>

/*
* This class implements a simple 1-D Extended Kalman Filter to estimate the Relative body frame postion of the lading target and its relative velocity
* position and velocity of the target is predicted using delta velocity
* The predictions are corrected periodically using the landing target sensor(or camera)
*/
class PosVelEKF
{
public:
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
    float getPos() const
    {
        return _ekf.x()(0);
    }

    // Get the EKF state velocity
    float getVel() const
    {
        return _ekf.x()(1);
    }

    // get the normalized innovation squared
    float getPosNIS(float pos, float posVar);

private:
    // Extended Kalman filter: working on float data types, two estimated states
    // (position and velocity), two observed outputs (position and velocity
    // delta), one control input (velocity delta), one additional update
    // argument (position uncertainty), and two additional prediction arguments
    // (time delta and velocity noise).
    using Ekf =
        Kalman<float, 2, 2, 1, std::tuple<float>, std::tuple<float, float>>;

    Ekf _ekf;
};
