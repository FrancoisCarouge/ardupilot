/*
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  The EKF3 states and covariance as typed linear algebra, each state with
  its unit.

  These headers are expensive to compile: include this file only from the
  source files that implement a fusion, never from a header.
 */
#pragma once

#include <AP_LinearAlgebra/AP_LinearAlgebra.h>
#include <AP_LinearAlgebra/AP_LinearAlgebra_Units.h>
#include <AP_Math/ftype.h>

namespace NavEKF3_typed {

template <auto Reference>
using Quantity = AP_LinearAlgebra::Units::Quantity<Reference, ftype>;

constexpr auto metre_per_second = mp_units::si::metre / mp_units::si::second;

using Unitless = Quantity<mp_units::one>;
using Velocity = Quantity<metre_per_second>;
using Position = Quantity<mp_units::si::metre>;
using DeltaAngle = Quantity<mp_units::si::radian>;
using Field = Quantity<AP_LinearAlgebra::Units::gauss>;

// the 24 states, in the order of NavEKF3_core::state_elements: quaternion,
// velocity, position, delta angle bias, delta velocity bias, earth and body
// magnetic field, wind velocity
using State = AP_LinearAlgebra::ColumnVector<ftype,
      Unitless, Unitless, Unitless, Unitless,
      Velocity, Velocity, Velocity,
      Position, Position, Position,
      DeltaAngle, DeltaAngle, DeltaAngle,
      Velocity, Velocity, Velocity,
      Field, Field, Field,
      Field, Field, Field,
      Velocity, Velocity>;
using Covariance = decltype(State{} * fcarouge::transposed(State{}));

// typed views of the state and covariance arrays of the filter
using StateView = AP_LinearAlgebra::TypedView<State>;
using CovarianceView = AP_LinearAlgebra::TypedView<Covariance>;

// the Jacobian of an observation of type Observation with respect to the states
template <typename Observation>
using ObservationModel = decltype(AP_LinearAlgebra::ColumnVector<ftype, Observation> {} / State{});

} // namespace NavEKF3_typed
