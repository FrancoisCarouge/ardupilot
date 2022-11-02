#include "ExtendedKalmanFilter.h"

#include "AP_Math/matrixN.h"
#include "Filter/Kalman.h"

#include <array>
#include <cmath>
#include <cstddef>

ExtendedKalmanFilter::ExtendedKalmanFilter()
{
  _ekf.transition([](const Ekf::state &x, const float &driftX,
                  const float &driftY) -> Ekf::state {
    const Ekf::state drifts{ 0, 0, -driftX, -driftY };
    return x + drifts;
  });

  _ekf.observation([](const Ekf::state &x, const float &Px,
                   const float &Py) -> Ekf::output {
    const auto exp{ std::exp(
        -(std::pow(x(2) - Px, 2.f) + std::pow(x(3) - Py, 2.f)) /
        std::pow(x(1), 2.f)) };
    return Ekf::output{ x(0) * exp };
  });

  _ekf.h([](const Ekf::state &x, const float &Px,
         const float &Py) -> Ekf::output_model {
    const auto exp{ std::exp(
        -(std::pow(x(2) - Px, 2.f) + std::pow(x(3) - Py, 2.f)) /
        std::pow(x(1), 2.f)) };
    const Ekf::output_model h{
      exp,
      2 * x(0) *
          ((std::pow(x(2) - Px, 2.f) + std::pow(x(3) - Py, 2.f)) /
           std::pow(x(1), 3.f)) *
          exp,
      -2 * (x(0) * (x(2) - Px) / std::pow(x(1), 2.f)) * exp,
      -2 * (x(0) * (x(3) - Py) / std::pow(x(1), 2.f)) * exp
    };
    return h;
  });
}

void ExtendedKalmanFilter::reset(const VectorN<float,N> &x, const std::array<float,N> &p, const std::array<float,N> &q, float r)
{
    _ekf.p(Ekf::estimate_uncertainty{{p[0], 0, 0, 0}, {0, p[1], 0, 0}, {0, 0, p[2], 0}, {0, 0, 0, p[3]}});
    _ekf.x(x[0], x[1], x[2], x[3]);
    _ekf.q(Ekf::process_uncertainty{{q[0], 0, 0, 0}, {0, q[1], 0, 0}, {0, 0, q[2], 0}, {0, 0, 0, q[3]}});
    _ekf.r(r);
}


void ExtendedKalmanFilter::update(float z, float Px, float Py, float driftX, float driftY)
{
    _ekf(driftX, driftY, Px, Py, z);
}

float ExtendedKalmanFilter::operator[](std::size_t position)
{
    return _ekf.x()(position);
}
