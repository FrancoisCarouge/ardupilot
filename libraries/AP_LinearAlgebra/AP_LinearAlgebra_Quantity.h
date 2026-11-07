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
  mp-units quantities of ArduPilot's units, in single precision unless
  another scalar type is given, without the typed linear algebra: light
  enough for the interfaces of the filters that use them.
 */
#pragma once

#include "AP_LinearAlgebra_MathsMacrosPush.h"
#include <mp-units/framework/quantity.h>
#include <mp-units/systems/si/units.h>
#include "AP_LinearAlgebra_MathsMacrosPop.h"

namespace AP_LinearAlgebra {
namespace Units {

template <auto Reference, typename T = float>
using Quantity = mp_units::quantity<Reference, T>;

using Unitless = Quantity<mp_units::one>;
using Metres = Quantity<mp_units::si::metre>;
using MetresPerSecond = Quantity<mp_units::si::metre / mp_units::si::second>;
using MetresPerSecondSquared = Quantity<mp_units::si::metre / mp_units::si::second / mp_units::si::second>;
using Radians = Quantity<mp_units::si::radian>;
using RadiansPerSecond = Quantity<mp_units::si::radian / mp_units::si::second>;
using Seconds = Quantity<mp_units::si::second>;
using SquareMetres = Quantity<mp_units::si::metre * mp_units::si::metre>;
using SquareMetresPerSecondSquared = Quantity<(mp_units::si::metre / mp_units::si::second) * (mp_units::si::metre / mp_units::si::second)>;

// the unit of ArduPilot's magnetic field measurements
inline constexpr struct milligauss final : mp_units::named_unit<"mG", mp_units::mag_power<10, -7> * mp_units::si::tesla> {} milligauss;
using Milligauss = Quantity<milligauss>;

// the unit of the magnetic field states of the EKFs
inline constexpr struct gauss final : mp_units::named_unit<"G", mp_units::mag_power<10, -4> * mp_units::si::tesla> {} gauss;

} // namespace Units
} // namespace AP_LinearAlgebra
