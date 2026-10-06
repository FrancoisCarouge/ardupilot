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
  Save and undefine the double precision maths macros of AP_HAL_Macros.h
  before including third-party C++ headers, whose standard library includes
  (such as <chrono>) use these names. Restore them with
  AP_LinearAlgebra_MathsMacrosPop.h. Included more than once on purpose.
 */
#pragma push_macro("sin")
#undef sin
#pragma push_macro("cos")
#undef cos
#pragma push_macro("tan")
#undef tan
#pragma push_macro("acos")
#undef acos
#pragma push_macro("asin")
#undef asin
#pragma push_macro("atan")
#undef atan
#pragma push_macro("atan2")
#undef atan2
#pragma push_macro("exp")
#undef exp
#pragma push_macro("pow")
#undef pow
#pragma push_macro("sqrt")
#undef sqrt
#pragma push_macro("log2")
#undef log2
#pragma push_macro("log10")
#undef log10
#pragma push_macro("ceil")
#undef ceil
#pragma push_macro("floor")
#undef floor
#pragma push_macro("round")
#undef round
#pragma push_macro("fmax")
#undef fmax
#pragma push_macro("log")
#undef log
#pragma push_macro("fabs")
#undef fabs
