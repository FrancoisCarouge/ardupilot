#include "PosVelEKF.h"

// Initialize the covariance and state matrix
// This is called when the landing target is located for the first time or it was lost, then relocated
void PosVelEKF::init(float pos, float posVar, float vel, float velVar)
{
    _ekf.x(pos, vel);
    _ekf.p(Ekf::estimate_uncertainty{{posVar, 0}, {0, velVar}});

    _ekf.f([](const Ekf::state &x, const float &dt,
    const float &dVelNoise, const Ekf::input &u) -> Ekf::state_transition {
        static_cast<void>(x);
        static_cast<void>(dVelNoise);
        static_cast<void>(u);
        return Ekf::state_transition{{1, dt}, {0, 1}};
    });
    _ekf.q([](const Ekf::state &x, const float &dt,
    const float &dVelNoise) -> Ekf::process_uncertainty {
        static_cast<void>(x);
        static_cast<void>(dt);
        return Ekf::process_uncertainty{{0, 0}, {0, dVelNoise * dVelNoise}};
    });
    _ekf.g(0, 1);
    _ekf.h(Ekf::output_model{{1, 0}, {0, 0}});
    _ekf.r([](const Ekf::state &x, const Ekf::output &u, const float & posVr) ->
    Ekf::output_uncertainty {
        static_cast<void>(x);
        static_cast<void>(u);
        return Ekf::output_uncertainty{{posVr, 0}, {0, 0}};
        });
}

// This functions runs the Prediction Step of the EKF
// This is called at 400 hz
void PosVelEKF::predict(float dt, float dVel, float dVelNoise)
{
    _ekf.predict(dt, dVelNoise, dVel);
}

// fuse the new sensor measurement into the EKF calculations
// This is called whenever we have a new measurement available
void PosVelEKF::fusePos(float pos, float posVar)
{
    _ekf.update(posVar, pos, 0);
}

// Returns normalized innovation squared
float PosVelEKF::getPosNIS(float pos, float posVar)
{
    const auto innovation_residual{pos - _ekf.x()(0)};
    const auto innovation_covariance{_ekf.p()(0) + posVar};

    return (innovation_residual * innovation_residual) / innovation_covariance;
}
