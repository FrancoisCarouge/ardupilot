#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_LinearAlgebra/AP_LinearAlgebra_Quantity.h>

#include <stdint.h>

/*
* This class implements a simple 1-D Kalman Filter to estimate the Relative body frame position of the lading target and its relative velocity
* position and velocity of the target is predicted using delta velocity
* The predictions are corrected periodically using the landing target sensor(or camera)
*/
class PosVelEKF {
public:
    using Seconds = AP_LinearAlgebra::Units::Seconds;
    using Metres = AP_LinearAlgebra::Units::Metres;
    using MetresPerSecond = AP_LinearAlgebra::Units::MetresPerSecond;
    using SquareMetres = AP_LinearAlgebra::Units::SquareMetres;
    using SquareMetresPerSecondSquared = AP_LinearAlgebra::Units::SquareMetresPerSecondSquared;
    using Unitless = AP_LinearAlgebra::Units::Unitless;

    PosVelEKF();
    ~PosVelEKF();

    CLASS_NO_COPY(PosVelEKF);

    // Initialize the covariance and state matrix
    // This is called when the landing target is located for the first time or it was lost, then relocated
    void init(Metres pos, SquareMetres posVar, MetresPerSecond vel, SquareMetresPerSecondSquared velVar);

    // This functions runs the Prediction Step of the EKF
    // This is called at 400 hz
    void predict(Seconds dt, MetresPerSecond dVel, MetresPerSecond dVelNoise);

    // fuse the new sensor measurement into the EKF calculations
    // This is called whenever we have a new measurement available
    void fusePos(Metres pos, SquareMetres posVar);

    // Get the EKF state position
    Metres getPos() const;

    // Get the EKF state velocity
    MetresPerSecond getVel() const;

    // get the normalized innovation squared
    Unitless getPosNIS(Metres pos, SquareMetres posVar);

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
