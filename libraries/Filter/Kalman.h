#pragma once

#include <fcarouge/eigen/kalman.hpp>

template <typename State = double, typename Output = double,
          typename Input = void, typename UpdateTypes = fcarouge::eigen::empty_pack,
          typename PredictionTypes = fcarouge::eigen::empty_pack>
using Kalman =
    fcarouge::eigen::kalman<State, Output, Input, transpose, symmetrize, divide,
                     identity_matrix, UpdateTypes, PredictionTypes>;

/*//////////////////////////////////////////////////////////////////////////////
# WIP NOTES

## Useful Commands
CXXFLAGS="-std=c++23" sim_vehicle.py -v ArduCopter --console --map
astyle --options=Tools/CodeStyle/astylerc <file>

mode guided
arm throttle
takeoff 100
mode circle
param set circle_radius 2000
mode land
/*//////////////////////////////////////////////////////////////////////////////
