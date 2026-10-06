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
  Single precision mp-units quantities used as typed matrix elements.
 */
#pragma once

#include <fcarouge/mp_units.hpp>
#include <mp-units/systems/si.h>

#include "AP_LinearAlgebra.h"

namespace AP_LinearAlgebra {
namespace Units {

template <auto Reference>
using Quantity = mp_units::quantity<Reference, float>;

using Unitless = Quantity<mp_units::one>;
using Metres = Quantity<mp_units::si::metre>;
using MetresPerSecond = Quantity<mp_units::si::metre / mp_units::si::second>;
using MetresPerSecondSquared = Quantity<mp_units::si::metre / mp_units::si::second / mp_units::si::second>;
using Radians = Quantity<mp_units::si::radian>;
using RadiansPerSecond = Quantity<mp_units::si::radian / mp_units::si::second>;
using Seconds = Quantity<mp_units::si::second>;

} // namespace Units
} // namespace AP_LinearAlgebra
