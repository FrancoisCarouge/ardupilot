/*
Extended Kalman Filter class by Sam Tabor, 2013.
* http://diydrones.com/forum/topics/autonomous-soaring
* Set up for identifying thermals of Gaussian form, but could be adapted to other
* purposes by adapting the equations for the jacobians.
*
* A typed Kalman filter: the thermal's strength in metres per second, its
* radius and position in metres, every covariance element in its own unit.
*/

#pragma once

#include <AP_Math/typed_linear_algebra.h>

class ExtendedKalmanFilter {
public:
    using Length = typed_linear_algebra::Length;
    using Speed = typed_linear_algebra::Speed;

    // The thermal: its strength, the updraft at its centre; its radius; its
    // position north and east.
    using State = typed_linear_algebra::Vector<Speed, Length, Length, Length>;
    using Covariance = typed_linear_algebra::Matrix<State, State>;
    // The variometer's output: the vertical air velocity at the aircraft.
    using Output = typed_linear_algebra::Vector<Speed>;
    using OutputVariance = typed_linear_algebra::Matrix<Output, Output>;
    // The variometer's variance, the single element of the output variance:
    // the filter of one output stores it as a quantity.
    using Variance = typed_linear_algebra::Element<OutputVariance, 0, 0>;
    using OutputModel = fcarouge::kalman_filter::internal::evaluate<
        fcarouge::kalman_filter::internal::quotient<Output, State>>;

    // The updraft model: its Jacobian by the thermal, H = ∂h/∂X, at the
    // aircraft's position.
    struct updraft_jacobian {
        OutputModel operator()(const State &x, const Length &px, const Length &py) const;
    };

    // The thermal drifts with the wind.
    struct drift {
        State operator()(const State &x, const Length &drift_x, const Length &drift_y) const;
    };

    // The updraft at the aircraft's position, h(X).
    struct updraft {
        Output operator()(const State &x, const Length &px, const Length &py) const;
    };

    using Filter = decltype(fcarouge::kalman{
        fcarouge::state{State{}}, fcarouge::output<Output>,
        fcarouge::estimate_uncertainty{Covariance{}}, fcarouge::process_uncertainty{Covariance{}},
        fcarouge::output_uncertainty{OutputVariance{}}, fcarouge::output_model{updraft_jacobian{}},
        fcarouge::transition{drift{}}, fcarouge::observation{updraft{}},
        fcarouge::update_types<Length, Length>, fcarouge::prediction_types<Length, Length>,
        fcarouge::simple_form, fcarouge::symmetrize});

    // Restarts the filter at a thermal x of covariance p, with the process
    // noise q, and the variance r of the variometer.
    void reset(const State &x, const Covariance &p, const Covariance &q, Variance r);

    // Fuses a variometer reading z at the aircraft's position (px, py), the
    // thermal having drifted with the wind by (drift_x, drift_y).
    void update(Speed z, Length px, Length py, Length drift_x, Length drift_y);

    Speed strength() const { return _filter.x().at<0>(); }
    Length radius() const { return _filter.x().at<1>(); }
    Length position_x() const { return _filter.x().at<2>(); }
    Length position_y() const { return _filter.x().at<3>(); }

private:
    Filter _filter{
        fcarouge::state{State{}}, fcarouge::output<Output>,
        fcarouge::estimate_uncertainty{Covariance{}}, fcarouge::process_uncertainty{Covariance{}},
        fcarouge::output_uncertainty{OutputVariance{}}, fcarouge::output_model{updraft_jacobian{}},
        fcarouge::transition{drift{}}, fcarouge::observation{updraft{}},
        fcarouge::update_types<Length, Length>, fcarouge::prediction_types<Length, Length>,
        fcarouge::simple_form, fcarouge::symmetrize};
};
