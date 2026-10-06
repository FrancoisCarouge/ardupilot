# Kalman and TypedLinearAlgebra in the ArduPilot fork: development plan

Repository: `ardupilot/` (branch `fcarouge`, remote `FrancoisCarouge/ardupilot`).
Libraries: [FrancoisCarouge/Kalman](https://github.com/FrancoisCarouge/Kalman) 0.5.4,
[FrancoisCarouge/TypedLinearAlgebra](https://github.com/FrancoisCarouge/TypedLinearAlgebra) 0.4.0.

## Decisions (2026-10-04)

| Topic | Decision |
|---|---|
| Toolchain | Fork-wide upgrade to `-std=gnu++26` and a C++26-capable GCC on every target |
| Units | mp-units as the TLA element types |
| Legacy code | Replaced outright, no compile-time fallback |
| EKF2/EKF3 | Full replacement |

## Objectives

1. Replace ArduPilot's Kalman filters with FrancoisCarouge Kalman (Phases 5, 7, 8).
2. Use FrancoisCarouge TypedLinearAlgebra for every heterogeneous linear algebra vector or matrix in
   ArduPilot: any vector, matrix, array, struct or argument list whose elements have different units or
   meanings and take part in linear algebra, including the ones not named as vectors (unions overlaying a
   struct on an array, parameter blocks reached through a `float *`, state triplets passed as separate
   arguments, polynomial coefficients, covariance and normal-equation matrices). The inventory below is
   kept up to date as more are found.
3. Find shortcomings, frictions and improvement opportunities in the FrancoisCarouge projects, as a
   real-world, safety-critical, embedded consumer. Every such finding is recorded in
   [FCAROUGE_FINDINGS.md](FCAROUGE_FINDINGS.md) with its evidence and a suggestion, and shared with the
   owner when found.

## Moving targets: upstream libraries and fork master

Kalman and TypedLinearAlgebra are developed in parallel with this branch (for example, both gained
commits between 2026-10-04 and 2026-10-05). This branch must be adjusted whenever they change:

- Pin each submodule to a tagged release, and record the pinned version here.
- On each new release: update the submodule in its own `modules:` commit, re-run the header probes
  (g++-14, clang-19, Arm GCC 15.2, `-Werror`), the AP_LinearAlgebra tests, the filter equivalence tests
  and the affected autotests, then adapt ArduPilot call sites in per-library commits.
- Changes ArduPilot needs (Phase 2 items) are made upstream first, then picked up through a release;
  library sources are never patched inside `modules/`.
- Record upstream changes that affect this plan (API, minimum compiler, new prerequisites) in the
  phase logs.

The fork's `master` branch (tracking ArduPilot) is also updated in parallel. This branch must follow it:

- Rebase `fcarouge` onto the updated `origin/master`; never merge (ArduPilot rejects merge commits).
  Once the branch is pushed, a rebase needs `git push --force-with-lease`.
- After each rebase: re-run `Tools/scripts/check_branch_conventions.py --base-branch origin/master`, the
  SITL and ChibiOS builds, `./waf check`, and the autotests of the filters touched so far.
- Expect conflicts where this branch edits shared infrastructure: `Tools/ardupilotwaf/boards.py` and
  `toolchain.py`, the installers, `.github/workflows/*` (new workflows also need the `setup-cxx26`
  step), and every filter replaced in later phases. New upstream code may need the C++26 fixes of
  Phase 1 again (volatile increments, hidden overloads).
- Re-check the inventory when upstream adds or changes a Kalman filter or heterogeneous linear algebra.

## Baseline findings

- ArduPilot compiles C++ with `-std=gnu++11` (`Tools/ardupilotwaf/boards.py`, `Board.configure_env`).
  Firmware toolchain: `gcc-arm-none-eabi-10-2020-q4` (GCC 10.2). ESP32: esp-idf v5.3 (GCC 13).
- Kalman requires C++23 (deducing `this`, `<print>`); TLA requires C++26.
- A 1x1 Kalman filter builds with g++-14, g++-15, clang++-21 under
  `-fno-exceptions -fno-rtti -Werror=shadow,undef,float-equal -Os` (117 bytes of code);
  g++-13 fails (`<print>` missing).
- `kalman_internal::function` allocates with `std::make_unique` and virtual dispatch: not acceptable
  after init in ArduPilot. Must be fixed upstream.
- Filters above 1x1 need a linear algebra backend; ArduPilot `MatrixN<T,N>` is square-only with 4 ops.
- Both libraries are Unlicense (GPLv3 compatible).

## Inventory

Kalman filters:

| Location | Shape (x x z x u) | State | Phase |
|---|---|---|---|
| `AP_Soaring/ExtendedKalmanFilter` | EKF 4x1x0 | m/s, m, m, m | 5 (pilot) |
| `AC_PrecLand/PosVelEKF` (x2 axes) | KF 2x1x1 | m, m/s; u m/s | 5 |
| `AP_Airspeed/Airspeed_Calibration` | EKF 3x1x0 | m/s, m/s, 1/sqrt(ratio) | 5 |
| `AP_NavEKF/EKFGSF_yaw` (bank of 5 + Gaussian sum) | EKF 3x2x0 | m/s, m/s, rad | 5 |
| `AP_Mount/SoloGimbalEKF` (disabled by default) | EKF, 9 states | rad, m/s, rad/s | 5 |
| `AP_NavEKF3` | 24-state EKF, sequential scalar fusion | mixed | 7 |
| `AP_NavEKF2` | 24-state EKF | mixed | 8 |

Heterogeneous linear algebra (TypedLinearAlgebra), surveyed 2026-10-06:

| Location | Heterogeneous vector or matrix | Form today | Phase |
|---|---|---|---|
| `AP_NavEKF3` `state_elements` / `statesArray` | 24 states: quaternion (1), velocity (m/s), position (m), gyro bias (rad per IMU step), accel bias (m/s per IMU step), earth and body field (Gauss), wind (m/s); 24x24 covariance `P` | union of a struct and `Vector24`; `Matrix24` | 7 |
| `AP_NavEKF3` `output_elements` | quaternion, velocity (m/s), position (m) | struct | 7 |
| `AP_NavEKF2` `statesArray` | 28 states, same groups | union with `Vector28` | 8 |
| `AP_NavEKF/EKFGSF_yaw` | per model `[vN m/s, vE m/s, yaw rad]` and 3x3 covariance | `ftype X[3]`, `ftype P[3][3]` | 5 |
| `AP_Soaring/ExtendedKalmanFilter` | `[strength m/s, radius m, x m, y m]` | done on the pilot branch | 5 |
| `AC_PrecLand/PosVelEKF` | `[pos m, vel m/s]`, 2x2 covariance | `float _state[2]`, `float _cov[3]` | 5 |
| `AP_Airspeed/Airspeed_Calibration` | `[wind N m/s, wind E m/s, 1/sqrt(ratio)]` and covariance | `Vector3f state`, `Matrix3f P` | 5 |
| `AP_Mount/SoloGimbalEKF` | 13 states: angle error (rad), velocity (m/s), gyro bias (rad/s), quaternion | `Vector13` and struct | 5 |
| `AP_Compass/CompassCalibrator` `param_t` | `[radius mGauss, offset mGauss x3, diag x3, offdiag x3, scale]`; Levenberg-Marquardt Jacobians and `JTJ` | struct reached as `float *` (`get_sphere_params()`, `get_ellipsoid_params()`) | 6 |
| `AP_AccelCal/AccelCalibrator` `param_u` | `[offset m/s^2 x3, diag x3, offdiag x3]`; Gauss-Newton normal equations | union of `param_t` and `VectorN<float, 9>` (type punning) | 6 |
| `AP_InertialSensor` temperature calibration, `AP_Math/polyfit.h` | polynomial coefficients per degree (unit/K, unit/K^2, unit/K^3); normal-equation matrix of powers of temperature | `AP_Vector3f coeff[3]`; `PolyFit::mat[order][order]` | 6 |
| `AP_Math/control.h` kinematic shaping | `[position, velocity, acceleration]` states (and jerk limits) | separate arguments of `update_pos_vel_accel*`, `shape_pos_vel_accel*`, `shape_angle_vel_accel` | 6 |
| `AP_Math/SCurve` segments | `[jerk, accel, vel, pos]` per segment | struct fields | 6 |
| `APM_Control/AP_AutoTune` `ATGains` | `[FF, P, I, D, IMAX]` gains of different units | struct, no vector arithmetic | 6, low value |

Excluded after review: `Location` (latitude, longitude, altitude: frame conversions, not linear algebra),
per-instance parameter arrays, rotation matrices (uniform), SITL physics models (not flight code).

Excluded (not Kalman filters / not flight code): TECS complementary filters, Variometer and `Filter/`
low-pass/notch filters, `AP_InertialNav`, SITL `SIM_*`.

## Phases

### Phase 0: Toolchain spike (go/no-go, 1-2 weeks)

- SITL with `-std=gnu++26` on g++-15 and clang-21, every vehicle, plus `./waf tests`: catalogue
  breakages (volatile compound assignment, implicit `this` capture, rewritten `operator==`
  ambiguities, enum arithmetic, aggregates with user-declared constructors, `u8` literals, `-Werror`).
- Arm GNU Toolchain 15.x `arm-none-eabi`: CubeOrange, MatekH743, one 1 MB F4; flash delta vs GCC 10.2;
  `<format>`/`<print>`/`<memory>` with newlib-nano.
- ESP32 (needs an esp-idf with GCC >= 14 or is dropped), QURT, Linux SBC cross toolchains.
- Exit: breakage catalogue, size deltas, list of targets that must be upgraded or dropped; owner decides.

### Phase 1: Land the fork-wide upgrade (2-4 weeks)

- Fix breakages, one commit per subsystem.
- `boards.py`, `chibios.py`, `esp32.py`, `Tools/environment_install`, CI images and workflows.
- Full autotest and size comparison.

### Phase 2: Upstream library prerequisites (2-3 weeks, in the FrancoisCarouge repositories)

- Kalman: non-allocating callable storage; optional `<format>`/`<print>`.
- For EKF3: scalar sequential fusion, state masking (`stateIndexLim`, wind/mag/accel-bias inhibition),
  innovation and S access, update rejection, overridable covariance prediction.
- mp-units: freestanding, `float` representation, contracts off, no formatting.
- Tagged releases.

### Phase 3: Dependencies (1 week)

- Submodules `modules/Kalman`, `modules/TypedLinearAlgebra`, `modules/mp-units`, pinned to tags.
- `Tools/ardupilotwaf/fcarouge.py` following the `littlefs.py` pattern.

### Phase 4: `AP_LinearAlgebra` library (2-3 weeks)

- Fixed-size MxN backend, no heap: `+ - *`, transpose, scalar, identity/zero, closed-form inverse for
  1x1-3x3, LDLT for symmetric S.
- mp-units aliases for ArduPilot quantities (m, m/s, m/s^2, rad, rad/s, gauss, unitless).
- gtest, gbenchmark, host-only Eigen reference test.

### Phase 5: Small filters, replaced outright

Equivalence trace captured from the old implementation before removal.

1. `AP_Soaring` EKF (pilot); remove `matrixN.*`. Autotests `Soaring`, `SoaringClimbRate`.
2. `AC_PrecLand/PosVelEKF`. Autotests `PrecisionLanding`, `PrecisionLoiterCompanion`.
3. `Airspeed_Calibration`. Autotest `AirspeedCal`.
4. `EKFGSF_yaw`. Autotests `GSF`, `GSF_reset`, `EKFYawResetLogged`; Replay.
5. `SoloGimbalEKF` (build only).

### Phase 6: TypedLinearAlgebra for the other heterogeneous vectors (3-5 weeks)

The non-Kalman entries of the TypedLinearAlgebra inventory, each with an equivalence test against the
current code: `CompassCalibrator` and `AccelCalibrator` parameter vectors, Jacobians and normal equations
(the accel calibrator's struct/array union is also type punning), IMU temperature calibration
coefficients and `PolyFit`, the `[position, velocity, acceleration]` states of the kinematic shaping
functions, and SCurve segments. Not blocked by F2: TypedLinearAlgebra and mp-units build for ChibiOS.
Keep looking for heterogeneous vectors not yet in the inventory.

### Phase 7: Full EKF3 replacement (largest phase, ~2-3 months)

- Typed 24-state vector, covariance prediction, then each fusion type (vel/pos, height, mag
  3-axis/declination/yaw, airspeed, sideslip, optical flow, range beacon, body/wheel odometry,
  GPS yaw, drag). Output predictor, ring buffers, multi-core lanes and reset logic stay outside the filter.
- Validation: Replay corpus (XKF* diff within tolerance), full autotest, H7/F4 CPU profiling,
  real flight tests by humans. Sensor-fusion changes are flagged for human review (AGENTS.md).

### Phase 8: EKF2

Same approach reusing EKF3 models; confirm port vs removal with the owner first.

### Phase 9: Cleanup and documentation

Remove `MatrixN`/`VectorN` once unused; library README.

## Standards (every phase)

- One subsystem per commit, `Subsystem: description`, prefix from `Tools/scripts/allowed_subsystems.py`;
  verify with `Tools/scripts/check_branch_conventions.py`. No merge or fixup commits.
- Do not modify submodule contents.
- No heap after init, no exceptions/RTTI, `float`/`ftype` not double.
- No parameter index or log format changes.
- GPLv3 headers on new files; astyle on changed code only.

## Phase 0 log

Working-tree changes (not committed): `Tools/ardupilotwaf/boards.py` and
`libraries/AP_HAL_ChibiOS/hwdef/common/chibios_board.mk` switched from `-std=gnu++11` to `-std=gnu++26`.
Builds use separate output directories (`build-cxx26-gcc15`, `build-cxx26-clang21`, `build-gcc10_cxx11`,
`build-gcc15_cxx11`, `build-gcc15_cxx26`); the original `build/` is untouched.

Toolchains used: host g++-15, clang++-21; Arm GNU Toolchain 15.2.Rel1; ArduPilot's
gcc-arm-none-eabi-10-2020-q4 (baseline). Submodules were initialized (only `waf` was checked out).

### SITL, g++-15, gnu++26

- copter, plane, rover, sub, heli, antennatracker, blimp: all build.
  (A combined `./waf copter plane rover ...` invocation produces Lua-binding link errors; building each
  vehicle separately is clean, so this is an invocation artefact, not C++26.)
- New warnings (2 kinds):
  - `-Wsized-deallocation`: `libraries/AP_Common/c++.cpp:77,82` define unsized `operator delete`
    only; C++14+ wants the sized overloads too.
  - `-Wvolatile`: `libraries/AP_ESC_Telem/AP_ESC_Telem.cpp:586` `telemdata.count++` on a volatile.
- `./waf check`: 69/72 pass. Failures `test_bitmask`, `test_math_double`, `test_rotations` are
  `EXPECT_EXIT` death tests ("failed to die"); they fail identically with gnu++11, so pre-existing
  in this environment, not caused by the standard change.

### Kalman headers on Arm GCC 15.2

1x1 Kalman probe, `-mcpu=cortex-m7 --specs=nano.specs -fno-exceptions -fno-rtti
-fsingle-precision-constant -Wdouble-promotion -Werror=shadow,undef,float-equal -Os`: compiles with
no warnings, 32 bytes of code. `<print>`/`<format>`/`<memory>` headers are available with newlib-nano.

### SITL, clang++-21, gnu++26

- All 7 vehicles build. `./waf check`: same 3 pre-existing death-test failures, 69/72 pass.
- Only standard-related warning: `-Wdeprecated-volatile` at `AP_ESC_Telem.cpp:586`. Other clang warnings
  (`-Wmain`, `-Wunused-private-field`, ...) are not standard-related.

### ChibiOS firmware (copter and plane)

All 18 builds succeed. Sizes in bytes (`arm-none-eabi-size`, text / bss):

| Board | Vehicle | GCC 10.2 gnu++11 (baseline) | GCC 15.2 gnu++11 | GCC 15.2 gnu++26 | text delta vs baseline | bss delta vs baseline |
|---|---|---|---|---|---|---|
| CubeOrange | copter | 1654052 / 568940 | 1640760 / 582268 | 1641768 / 581260 | -12284 | +12320 |
| CubeOrange | plane | 1648848 / 574320 | 1635748 / 587456 | 1636644 / 586560 | -12204 | +12240 |
| MatekH743 | copter | 1568388 / 392460 | 1555432 / 405452 | 1556452 / 404432 | -11936 | +11972 |
| MatekH743 | plane | 1562308 / 398716 | 1549484 / 411576 | 1550392 / 410668 | -11916 | +11952 |
| MatekF405 | copter | 874452 / 234960 | 868820 / 240632 | 869160 / 240292 | -5292 | +5332 |
| MatekF405 | plane | 926428 / 183160 | 920956 / 188668 | 921260 / 188364 | -5168 | +5204 |

- GCC 15 shrinks code by 5-13 kB; gnu++26 itself costs about +0.3-1 kB of code over gnu++11 on GCC 15.
- The apparent bss growth is not RAM. `arm-none-eabi-size` counts the `.crash_log` section, a flash
  region that takes the flash left over after `.text`, so smaller code means a larger crash log.
  Real RAM, from `objdump -h`:

  | Board copter | `.bss` GCC 10.2 | `.bss` GCC 15.2 | `.heap` GCC 10.2 | `.heap` GCC 15.2 |
  |---|---|---|---|---|
  | CubeOrange | 128952 | 129276 | 121952 | 121644 |
  | MatekF405 | 72240 | 72540 | 47832 | 47548 |

  The +300 B of `.bss` comes from the GCC toolchain, not the standard: it is newlib's static stdio
  `FILE` table `__sf` (+312 B), which is larger in the newlib shipped with Arm GCC 15.2. The heap
  shrinks by the same amount.
- New warnings with GCC 15 + gnu++26 (none with the GCC 10 baseline):
  - `-Wvolatile` (11): `AP_HAL_ChibiOS/shared_dma.cpp` (9 sites), `AP_HAL_ChibiOS/CrashDump_SD.cpp:1055`,
    `AP_ESC_Telem/AP_ESC_Telem.cpp:586`.
  - `-Wsized-deallocation` (2): `AP_Common/c++.cpp:77,82`.
  - From GCC 15 regardless of standard: `-Woverloaded-virtual=` at `AP_HAL/RCOutput.h:183` (`timer_tick`
    hidden), and ld "LOAD segment with RWX permissions".

### TypedLinearAlgebra headers

Probe: a minimal owning fixed-size `float` backend (`v[R*C]`, `operator()(i)`, `operator()(i, j)`,
`operator[]`, `+ - *`, scalar `* /`, `transpose()`, `Zero()`), a heterogeneous column vector
`[float, std::chrono::duration<float>]`, addition, scaling, `at<i>()` and a uniform matrix product.

- Builds with no warnings on host g++-15, clang++-21, and Arm GCC 15.2 for Cortex-M7 and Cortex-M4
  (`-fsingle-precision-constant -Wdouble-promotion -Werror=shadow,undef,float-equal -Os`):
  102 bytes of code, only external symbol `memset`. Including `<format>` pulls no formatting code
  into the object unless used.
- Adding `[float, s]` to `[s, float]` is rejected at compile time
  ("Matrix addition requires compatible element types").
- TLA's `std` backends use Kokkos' reference `std::linalg` and non-owning `std::mdspan` views; not usable
  for ArduPilot. The Phase 4 backend must be an owning fixed-size type like the probe's.

### Minimum compiler

Both probes (Kalman 1x1 at `6603b3f`, TLA at `32dad77`) build with g++-14, g++-15 and clang++-21;
g++-13 rejects `-std=gnu++26`. ArduPilot SITL copter builds at gnu++26 with g++-14 (same warnings as
g++-15). GCC 14 is therefore the floor for every target.

### SITL autotests (g++-15, gnu++26)

| Test | Result |
|---|---|
| `Plane.Soaring` | pass |
| `Plane.SoaringClimbRate` | fail, also fails at gnu++11 |
| `Plane.AirspeedCal` | pass |
| `Copter.PrecisionLanding` | pass |
| `Copter.GSF` | pass |
| `Copter.GSF_reset` | pass |
| `Copter.EKFYawResetLogged` | pass |

`SoaringClimbRate` fails with "VFR_HUD.climb diverged from SIM_STATE.vd by 29.4 m/s" as soon as soaring is
enabled, identically (29.2 m/s) with gnu++11. Resolved (2026-10-06): upstream added this test to reproduce a
known bug ("autotest: reproduce soaring climb rate reporting bug", f37bd41530, 2025-07-22) and lists it in
`disabled_tests` ("very bad sink rate"); running it by name bypasses that list. The bug is in the
total-energy variometer reading reported as `VFR_HUD.climb`, not in the soaring EKF, so it does not block
the Phase 5 pilot, which is validated with `Soaring`.

### Other targets

| Target | Current toolchain | C++26-capable? | Needed |
|---|---|---|---|
| ChibiOS (STM32) | gcc-arm-none-eabi 10.2 | no | Arm GNU Toolchain 15.2 (verified above) |
| ESP32 / ESP32-S3 | esp-idf `release/v5.3` (`Tools/scripts/esp32_get_idf.sh`), xtensa GCC 13.2 | no | esp-idf v5.4/v5.5 (GCC 14.2) or v6.0 (GCC 15.2); HAL port effort unknown |
| QURT | Hexagon SDK 4.1.0.4-lite, Hexagon Tools 8.4.05 clang | no | newer Hexagon SDK/Tools with C++26 clang, or drop QURT |
| Linux SBC | distro `arm-linux-gnueabihf` / `aarch64-linux-gnu` cross GCC | depends on distro | GCC >= 14 cross packages (not installed here, not tested) |
| SITL | host GCC/clang | yes with g++-14+ / clang-21 | CI images with g++-14+ |

### Phase 0 conclusion

Recommendation: **go** for Phase 1 on SITL, Linux and ChibiOS.

- The ArduPilot sources need only small fixes for gnu++26: 11 `-Wvolatile` sites, 2 missing sized
  `operator delete` overloads, plus the GCC 15 `-Woverloaded-virtual=` warning.
- Arm GCC 15.2 makes ChibiOS firmware 5-13 kB smaller; gnu++26 costs at most 1 kB back; static RAM +300 B.
- Both libraries compile cleanly for Cortex-M4/M7 under ArduPilot's float-only warning flags.

Owner decisions for Phase 1 (2026-10-05):

- ESP32: upgrade esp-idf to v6.0 (xtensa GCC 15.2).
- QURT: upgrade the Hexagon SDK to a release whose clang supports C++26 (proprietary SDK: the owner must
  provide it; it cannot be downloaded or tested in the development environment).
- Linux SBC: distro GCC 15 cross toolchains (`arm-linux-gnueabihf`, `aarch64-linux-gnu`) in CI.

Not done in Phase 0, needs a human:

- Hardware smoke test of a GCC 15 / gnu++26 firmware on at least one H7 and one F4 board.

Note: the Phase 0 toolchains, library clones and logs lived in a session scratchpad that has since been
cleared; the numbers above are the record. The `build-*` directories in the repository root are the
Phase 0 build outputs and can be deleted.

## Phase 1 log

### Source and build changes (working tree)

| Subsystem | Change |
|---|---|
| `AP_Common` | `c++.cpp`: add sized `operator delete(void *, size_t)` and `operator delete[](void *, size_t)` |
| `AP_ESC_Telem` | `AP_ESC_Telem.cpp`: volatile `count++` becomes `count = count + 1` |
| `AP_HAL_ChibiOS` | `shared_dma.cpp` (9 sites) and `CrashDump_SD.cpp`: same volatile increment rewrite |
| `AP_HAL_ChibiOS` | `RCOutput.h`: `using AP_HAL::RCOutput::timer_tick;` and, when serial LEDs are disabled, `using AP_HAL::RCOutput::serial_led_send;` so the base virtuals are not hidden |
| `hwdef` | `chibios_board.mk`: `-std=gnu++26` |
| `waf` | `boards.py`: `-std=gnu++26`; configure fails below g++ 14 or clang 19; ChibiOS floor g++ 14; `-Werror` whitelist gains Arm GCC 15.2.1; `--no-warn-rwx-segments` for ChibiOS links |
| `Tools` | `install-prereqs-ubuntu.sh`: Arm GNU Toolchain 15.2.rel1 from developer.arm.com (x86_64, aarch64) |

clang 18 fails on libstdc++ 15 `<format>` (immediate-function error) with both libraries; clang 19, 20 and
21 build them. Hence the clang 19 floor.

### Verification

- SITL g++-15: copter, plane build with no standard-related warnings; `./waf check` 69/72 (the same 3
  pre-existing death-test failures).
- SITL clang-21: copter builds with no standard-related warnings.
- Arm GCC 15.2: CubeOrange copter (with `-Werror` enabled), MatekF405 plane, `iomcu-dshot` iofirmware build
  with zero warnings, linker included.
- Configure with g++-13 or clang-18 stops with the new minimum-version message.

### Open

- CI: every compiling workflow uses ArduPilot's `ardupilot/ardupilot-dev-*:v0.2.0` images (Ubuntu 24.04:
  default g++ 13, clang 18 in the clang image, GCC 10.2 for ARM). Decision (2026-10-05): keep the images and
  add a setup step per job that apt-installs g++-14, clang-19 and noble's GCC 14 cross compilers, and
  fetches Arm GCC 15.2 (cached). Linux SBC CI uses noble's GCC 14 cross (no GCC 15 packages on noble),
  superseding the earlier GCC 15 choice.
- SITL `-Werror` whitelist must list the CI host compiler versions.
- Done: waf picks `g++-15`/`g++-14` for native builds when the default is older and CC/CXX are unset;
  the Ubuntu installer adds `g++-14` on noble. (Exporting CC/CXX was rejected: waf gives the `CXX`
  variable priority over cross toolchains.)
- Done: `.github/actions/setup-cxx26` installs g++-14 (and clang-19, g++-14-multilib, GCC 14 cross,
  cached Arm GCC 15.2 as `/opt/gcc-arm-none-eabi-15`) in every compiling job; ChibiOS matrices use GCC 15;
  scan-build uses clang-tools-19; environment tests drop jammy and bookworm; Arch installer uses Arm 15.2
  with its published SHA-256. actionlint 1.7.12 and zizmor (medium) clean. Not yet run in GitHub Actions.
- Done: macOS installer uses Arm 15.2 on Apple silicon (none published for Intel Macs, skipped there);
  openSUSE installer uses Arm 15.2. `Tools/scripts/configure-ci.sh` is unused legacy (Travis, clang 7)
  and left alone.
- Done: `armhf-musl` CI (navigator) uses the Bootlin armv7-eabihf musl stable 2026.08-1 toolchain
  (GCC 15.3) instead of the image's musl.cc GCC 11. Verified locally: `navigator` `sub` builds, static
  binary when configured with `--static`.
- Pre-existing, not caused by the upgrade: for musl builds the configure header probes (`endian.h`,
  `byteswap.h`, ...) fail to link because `-Wl,--wrap,malloc` meets the toolchain's `libstdc++.so`
  (same with the old musl.cc GCC 11 toolchain), so `AP_Common/missing/endian.h` is used and redefines
  `__BYTE_ORDER` (98 warnings). Also `build_ci.sh` passes `--static` to the build step, where it has no
  effect; it is a configure option.
- macOS: Apple clang floor set to 17 (Xcode 16.3, LLVM 19 based). Untested until `macos_build.yml` runs.
- Vagrant `initvagrant-autotest-server.sh` builds upstream ArduPilot's firmware server and clones upstream
  `master`; left unchanged.
- openSUSE: Arm 15.2 done; its Linaro GCC 7.5 `arm-linux-gnueabihf` download is still too old (no
  verified Tumbleweed replacement yet).
- Done: `test_wasm_plane` checked locally with emscripten 6.0.8 (clang 24, libc++): ArduPlane wasm builds
  at gnu++26 and `wasm_plane_smoke_test.mjs` passes (needs emsdk's Node 24; the host Node 18 cannot load
  the module). No workflow change needed.
- Done: ESP32 on ESP-IDF v6.0.3 (xtensa GCC 15.2). `esp32buzz` and `esp32s3empty` build plane and copter
  (the CI matrix). Changes: RC input (`RmtSigReader`) moved to the RMT RX driver; unused
  `SoftSigReaderRMT` removed; MCPWM capabilities via `MCPWM_LL_GET()`; explicit esp-idf components and
  `esp_hal_wdt` link order; newlib kept (`CONFIG_LIBC_NEWLIB=y`, v6.0 defaults to picolibc);
  `SPI2_HOST`/`SPI3_HOST`; `esp32_get_idf.sh` and `esp32_build.yml` on v6.0.3 (installed and cached in
  CI). `OSD.cpp` (legacy MCPWM/I2S) is behind `WITH_INT_OSD`, defined by no board, so it never compiles.
- ESP32 follow-ups:
  - RC input on the new RMT driver must be tested on hardware with a receiver before flight (human).
  - Port `I2CDevice` and `i2c_sw.c` from the legacy I2C driver (end-of-life in v6.0, removed in v7.0)
    to the I2C master driver.
  - Consider moving from newlib to picolibc.
  - New GCC 15 warning on xtensa: `-Walloc-size-larger-than` at `AP_Math/matrix_alg.cpp:38`
    (`NEW_NOTHROW T[n*n]` overflow check).
  - Pre-existing: `esp32_get_idf.sh` compares `git rev-parse HEAD` with the literal `'$COMMIT'`, so
    its commit check never matches.
- QURT Hexagon SDK upgrade (owner provides the SDK).

## Phase 1 CI results (PR 3)

First full CI run (2026-10-06): 87 jobs passed, 32 failed. Causes and fixes:

- Bootloaders (`Tools/AP_Bootloader`) not C++26 clean (volatile `--`, `register`): fixed; Phase 0 never
  built a bootloader.
- `setup-cxx26` ran `update-ccache-symlinks`, which removed the images' cross compiler ccache links:
  replaced by explicit links.
- Unit test and DDS containers ran as user 1001 and could not install compilers: run as root.
- ROS images (Ubuntu 22.04) lack `gcc-14`: installed from the `ubuntu-toolchain-r/test` PPA.
- scan-build's compiler wrappers used the default g++ 13: `CCC_CC`/`CCC_CXX` set to gcc-14/g++-14.
- Cygwin pinned `gcc-g++` 13.4: now 14.4.
- SPRacingH7 (H750 external flash) failed to link: Arm 15.2 newlib `.ARM.exidx` entries out of PREL31
  range; the C libraries' unwind entries are discarded in `common_extf_h750.ld`.
- armhf GCC 14.2 cross reported `-Werror=unused-value` on a braced `Vector2f` in `test_control.cpp`:
  parentheses.

## Phase 5 log

### AP_Soaring thermal EKF (pilot)

- `ExtendedKalmanFilter` reimplemented on Kalman with units: state `[m/s, m, m, m]`, output m/s, prediction
  and update arguments in metres. The public shape is kept (`X[4]`, `reset()`, `update()`); `reset()`
  takes plain arrays, so `AP_Soaring` no longer uses `MatrixN`/`VectorN`.
- Equivalence with the legacy filter over three thermals (600 steps, resets reusing the filter): maximum
  relative state difference 8e-6, despite the Joseph covariance update (F8). Autotest `Plane.Soaring`
  passes (thermal detected, climb to `SOAR_ALT_MAX`). SITL plane builds with g++ 15 and clang 20.
- Cortex-M4 cost: 4072 B code and 360 B RAM versus 1040 B and 0 B (F21).
- The Kalman headers are confined to `ExtendedKalmanFilter.cpp` (F22). The filter object is allocated
  once, on the first `reset()` (first thermal), because Kalman allocates its callables (F1); owner review
  requested.
- Blocked on ChibiOS by F2: the soaring changes are kept local until Kalman no longer requires `<print>`.

## Phase 6 log

### AP_AccelCal (first TypedLinearAlgebra-only replacement)

- New test `libraries/AP_AccelCal/tests/test_accel_calibrator.cpp`: synthetic samples from a sensor with known
  offsets, scale and cross-axis errors (6 and 12 orientations); passes on the original code and on the typed
  one (offsets within 1e-3 m/s/s, scale factors within 1e-4).
- The union of `param_t` and `VectorN<float, 9>` (type punning) is gone; the Gauss-Newton fit uses typed
  parameters `[offset m/s/s x3, diag x3 (, offdiag x3)]`, Jacobian, `JTJ`, `JTFI` and a typed division for the
  step. A deliberate unit mistake fails to compile (F27 for the message).
- Firmware: CubeOrange and MatekF405 copter build with `-Werror`, +2.4 kB flash, no RAM change. Two
  frictions fixed in ArduPilot: maths macros versus `<chrono>` (F29, macro push/pop in `AP_LinearAlgebra`),
  and stack frame size (F28, in-place accumulation with a type-checked unevaluated expression).
- Lesson for the soaring pilot: `std::exp` is also a macro victim on F4 boards; use `expf`.
- Backed out of `fcarouge` (revert commit) because TypedLinearAlgebra does not build with current libc++,
  which broke the WebAssembly CI (F30); kept on the local branch `fcarouge-accelcal-typed`, to re-apply after
  the upstream fix. The calibrator tests and the maths macro guard stay on `fcarouge`.

### Parked work (local branches, not pushed)

| Branch | Content | Waiting for |
|---|---|---|
| `fcarouge-soaring-pilot` | Kalman-based `AP_Soaring` thermal EKF | Kalman F2 (`<print>`) and F1 (heap callables) fixed upstream |
| `fcarouge-accelcal-typed` | TypedLinearAlgebra accelerometer calibration fit | TypedLinearAlgebra F30 (current libc++) fixed upstream |
