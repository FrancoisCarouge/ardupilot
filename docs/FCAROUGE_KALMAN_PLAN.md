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
4. Add strongly-typed units (mp-units quantities) where they are missing in ArduPilot's heterogeneous
   linear algebra vectors and Kalman filters (owner, 2026-10-14): every element, intermediate term and
   input of those computations carries its physical unit, not an opaque or unitless type, unless it is
   genuinely dimensionless. The audit below tracks the gaps.

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
| `AP_NavEKF3` AGL filter (`UpdateAglKf`, option, new upstream in 2026-10) | KF 2x1x1 | m, m/s; u m/s^2 | 5, done 2026-10-14 |
| `AP_NavEKF3` terrain offset (`EstimateTerrainOffset`) | EKF 1x1x0, two sequential observations | m | 5 |
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

1. `AP_Soaring` EKF (pilot); remove `matrixN.*`. Autotests `Soaring`, `SoaringClimbRate`. Done.
2. `AC_PrecLand/PosVelEKF`. Autotests `PrecisionLanding`, `PrecisionLoiterCompanion`. Done.
3. `Airspeed_Calibration`. Autotest `AirspeedCal`. Done.
4. `EKFGSF_yaw`. Autotests `GSF`, `GSF_reset`, `EKFYawResetLogged`; Replay. Done 2026-10-11 (Replay not run).
5. `SoloGimbalEKF` (build only). Next, see the end of the log.

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
- 2026-10-07, Kalman bumped to 2301ab7cd (F1 fixed): the pilot rebuilds unchanged; the filter object no
  longer references the throwing `operator new`/`delete`, its SITL object code drops from 70.9 kB to
  4.5 kB, and `Plane.Soaring` passes. The one allocation left is the pilot's own, on the first `reset()`,
  which keeps the Kalman headers out of `AP_Soaring` (F22); it could become inline storage.
- 2026-10-07, F2 worked around in ArduPilot: the ChibiOS placeholder `cstdio` declares `FILE`, `stdout`,
  `EOF`, `fwrite`, `putc` (and `fputc`, `fflush`, `stderr`) bound to undefined symbols, so `<print>`
  compiles, its unused inline functions are never emitted, and any call fails to link
  (`ap_chibios_stdio_streams_are_not_supported_*`). The pilot then builds and links for CubeOrange and
  MatekF405: +2460 B and +2452 B code, +104 B data against `fcarouge` (F21: was about 3 kB before F1).
- Still parked: the WebAssembly Plane build (emscripten 6.0.8, as in CI) fails in `objs/AP_Soaring` on
  libc++'s `<tuple>` (F30), as the accelerometer fit did.

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

### CompassCalibrator: what typing exposes

- `param_t` is reached through `float *` views (`get_sphere_params()`, `get_ellipsoid_params()`) that index
  across struct members (`radius` then `offset`; `offset`, `diag`, `offdiag`), relying on their layout.
- The Levenberg-Marquardt damping adds the same `_ellipsoid_lambda` to every diagonal element of the 9x9
  `JTJ`, whose diagonal units are 1 for the offset rows and mGauss^2 for the scale rows: the damping assumes
  1 = 1 mGauss^2, so the convergence depends on the field unit (working in Gauss would weaken the damping of
  the scale terms by 1e6 relative to the offsets). The sphere fit (all unitless Jacobian) is consistent.
  Marquardt's lambda * diag(JTJ) would be unit consistent. A typed port must make the assumption explicit
  (TypedLinearAlgebra lacks the diagonal operations to express either form directly, F31). Not changed:
  flagged for human review, as it alters calibration behaviour.
- Validation: autotests `Copter.FixedYawCalibration` then `Copter.SITLCompassCalibration`.

Port (branch `fcarouge-compasscal-typed`, parked on F30 like the accelerometer fit):

- `AP_LinearAlgebra` gains a `milligauss` unit (1e-7 T): mp-units has the gauss only in its high energy
  physics system.
- One Levenberg-Marquardt iteration, `run_fit<Fit>(lambda)`, replaces both fits. `SphereFit` (radius and
  offsets in mGauss, unitless Jacobian) and `EllipsoidFit` (offsets in mGauss, unitless scale factors;
  Jacobian unitless then mGauss) convert to and from `param_t`; the `float *` views are removed.
- The damping is kept as it was, added in place to each diagonal element and documented as unit
  inconsistent for the ellipsoid. A singular `JTJ` (non-finite step) leaves parameters and lambda unchanged,
  as `mat_inverse` failing did.
- Stack: the typed solve works on the stack where `mat_inverse` allocated on the heap. Copying `JTJ` for
  the second damped step peaked at 1608 B of the 2048 B `compasscal` thread; damping `JTJ` in place twice
  (lambda/10, then up to lambda) brings the ellipsoid fit to 1280 B (`run_fit` 648, `solve` 208,
  `solve_right` 424), from about 1000 B before. A `solve_right` that factors in place would save more.
- Results: both autotests pass on g++ 15 SITL; in two runs the fitted offsets stay within 0.01 mGauss of
  the simulated ones (baseline 0.013) with the same fitness range (0.034 to 0.039) and scale factors.
  CubeOrange and MatekF405 copter build with `-Werror`, 2.4 kB larger.
- Environment: the autotest needs pymavlink 2.4.50 (`MAG_CAL_FAILED_RESIDUALS_HIGH`); 2.4.49 fails it on
  the baseline too.

### IMU temperature calibration polynomial fit (`PolyFit`)

A lurking heterogeneous system: the moments `mat[i][j]` = sum x^(6-i-j) and sums `vec[i]` = sum y x^(3-i)
of the normal equations, and the coefficients in y/x^(3-i), all untyped.

- Baseline test first, on `fcarouge`: `AP_Math/tests/test_polyfit.cpp` (exact cubic recovered; three
  temperatures rejected).
- Port (branch `fcarouge-polyfit-typed`, parked on F30): x and y get dimensions of their own (`PolyFit`
  knows neither unit), the accumulation `M += t * transpose(t)`, `S += t * transpose(y)` and the solve are
  typed, `mat`/`vec` stay untyped storage in the header (value initialized now) so that the header does not
  include TypedLinearAlgebra.
- Singularity: elimination with partial pivoting left a 1e-14 pivot for three temperatures, accepting a
  rank-deficient fit; the 4x4 `mat_inverse` it replaces compared the determinant, in x^12, with
  FLT_EPSILON, a unit-dependent test. `AP_LinearAlgebra` gains `solve_right_positive_definite()` (Cholesky,
  in place, each pivot compared with the diagonal element of the same unit) and the typed
  `divide_positive_definite()`, which also checks the type consistency that TypedLinearAlgebra's division
  does not (F32). Candidate for the compass and accelerometer fits too (in place, less stack, normal
  matrices).
- `-fsingle-precision-constant` makes `1000.0` a float: TypedLinearAlgebra's mp-units plug-in rightly
  rejects storing it in a double matrix; double tests go in `DOUBLE_PRECISION_SOURCES`.
- Results: `test_polyfit` and `test_linear_algebra` pass (g++ 15); `Plane.IMUTempCal` passes on the
  baseline and on the port. Not yet done: firmware builds (size, stack), clang 20.

### Refresh 2026-10-09: upstream fixes, parked work folded in

- `fcarouge` rebased onto the fork's `master` (219 new commits; the only conflicts were upstream's
  zero-initialized `rmt_config_t` in the ESP32 RMT readers, which this branch replaces or removes).
- Submodules bumped: Kalman ae8146d7b, TypedLinearAlgebra c9a5c2b, mp-units 132b590d2. Kalman moved its
  customization points to `fcarouge::kalman_filter::internal`; `AP_LinearAlgebra_Kalman.h` follows.
- TypedLinearAlgebra c9a5c2b fixes F30: WebAssembly Plane (emscripten 6.0.8, as in CI) builds with the
  thermal EKF and the accelerometer, compass and polynomial fits. Nothing is parked any more: the four
  branches are folded into `fcarouge` (and kept, pushed, for reference).
- Validation of the folded work: SITL plane and copter build with g++ 15 and clang 20; `test_linear_algebra`
  (9), `test_polyfit` (2), `test_accel_calibrator` (2) pass with both; autotests `Plane.Soaring`,
  `Plane.IMUTempCal`, `Copter.FixedYawCalibration` and `Copter.SITLCompassCalibration` pass (compass offsets
  within 0.009 mGauss); CubeOrange and MatekF405 copter and plane build with `-Werror`. Code size against
  `fcarouge` before the fold: CubeOrange copter +312 B, plane +2760 B; MatekF405 copter +1984 B, plane
  +4424 B.
- clang 20 `-Wshadow` caught a test variable `a` shadowing mp-units' `a` unit symbol; renamed.
- F19 withdrawn (the setter takes `const element &`); the thermal EKF's output model now sets its
  elements from temporaries. F3 fixed. Other findings rechecked, see `FCAROUGE_FINDINGS.md`.
- Human review requested: the sensor calibrations (accelerometer, compass, IMU temperature) and the
  thermal EKF now run on the typed code; the compass damping keeps its unit-inconsistent behaviour.

### Next (as of 2026-10-09)

1. Watch PR 3 CI on the rebased branch.
2. Remove workarounds once their findings are fixed upstream (in-place accumulation for F28, `ElementType`
   dispatch for F20, consistency check for F32, the `cstdio` declarations for F2 if `<print>` becomes
   opt-in).
3. Continue the inventory: EKFGSF, EKF2/EKF3.

### Precision landing filter (`AC_PrecLand/PosVelEKF`)

- Kalman filter over typed `[position m, velocity m/s]`, input the velocity change, process noise from the
  prediction arguments, position measurement. The public interface is unchanged; the filter lives in
  fixed storage inside `PosVelEKF` (no heap, Kalman headers kept out of `AC_PrecLand.h`).
- Found in SITL, not by the unit tests: constructed with the global `AC_PrecLand`, the filter copied
  Kalman's `one<output_model>` before its dynamic initialization, a zero output model, so it never
  corrected (`Copter.PrecisionLanding` landed 52 m and 2.9 m off). F21 is a correctness finding; the
  filter is now constructed on the first `init()`, and `test_posvelekf` has a global filter case.
- Equivalence over 200 random 10 s runs (filters built at run time): positions within 3.8 um, velocities
  4.3e-6 m/s, NIS 5e-4 relative (Joseph form update, F8).
- Validation from a clean worktree: unit tests (g++ 15, clang 20), WebAssembly, `Copter.PrecisionLanding`
  and `PrecisionLoiterCompanion` pass (0.35 m from target, as before). CubeOrange copter +2320 B code,
  +40 B data; MatekF405 unchanged (no precision landing).

### Airspeed ratio calibration (`AP_Airspeed/Airspeed_Calibration`)

- Extended Kalman filter over typed `[wind N m/s, wind E m/s, scale]`, output true airspeed (m/s), callable
  H and observation of the ground velocity, identity transition with constant process noise. The
  Jacobian's `1/sqrt(SH1)` is in s/m: a first, untyped draft dropped that unit and the types caught it.
  Symmetrization, non-negative diagonal, state limits and the small airspeed guard are kept. `AP_Airspeed`
  uses accessors (`set_scale`, `get_state`, `get_variances`) instead of the members.
- Fixed storage, constructed on first use (F21). Filter sizes measured: 180 bytes here, 116 for the
  precision landing filter (its storage reduced from 160 to 120 bytes); F35.
- Equivalence over 100 random 300 s calibrations: scale within 1.8e-7, wind 1.4e-5 m/s, variances 1.4e-5
  relative. `test_airspeed_calibration` (global object, circle in wind) passes with the old and new filter,
  g++ 15 and clang 20; WebAssembly builds; `Plane.AIRSPEED_AUTOCAL` passes from a clean build, ratio resets
  1.242/3.307 and 1.245/3.279 against 1.242/3.306 before (run to run variation of the same order).
- Plane +2872 B code, +64 B data on CubeOrange; +2528 B, +64 B on MatekF405.

### Validation method

A long-lived build directory gave wrong results here: after header layout changes, objects that embed
`AC_PrecLand` were not all rebuilt, and failures (or passes) followed the stale objects. Autotests are
now run from a fresh worktree with a new build directory; on 2026-10-09 everything folded so far was
re-validated that way (the ten autotests listed above and in the 2026-10-09 refresh, all unit tests).
Firmware sizes measured earlier match fresh builds.

### SCurve segments (`AP_Math/SCurve`)

- Each segment's `[acceleration, velocity, position]` (m/s/s, m/s, m) is a typed vector: the kinematics
  at the segment start are carried over the elapsed time by a typed transition matrix (elements 1, s,
  s^2/2), plus the constant or raised cosine jerk contribution built from mp-units quantities. The
  decreasing jerk segment is the second half of the increasing one. The header is unchanged.
- Equivalence over a grid of jerks, durations and start states: rounding only, at most 0.49 mm in
  position at 4 km, 1.1e-5 m/s, 3.8e-6 m/s/s, jerk identical. `test_scurve` passes with g++ 15 and
  clang 20; WebAssembly Plane builds; autotests `Copter.CopterMission`, `WPArcs`, `SplineTerrain` and
  `NavDelay` pass. `Copter.FlyMissionTwice` fails identically before and after, and on the fork's
  unmodified `master` built with its own g++ 13: an upstream failure, not this work.
- CubeOrange and MatekF405 copter build with `-Werror`, 572 B larger. Finding F34 (no element list
  constructor for a heterogeneous matrix). Flight path code: human review requested.

### Kinematic shaping (`AP_Math/control`): assessed, not a TypedLinearAlgebra fit

The `[position, velocity, acceleration]` triples of `update_pos_vel_accel*`, `shape_*_vel_accel*` and
`shape_angle_vel_accel` are passed as separate arguments to scalar, nonlinear per-axis controllers
(`sqrt_controller`, limits, constraints); the only linear algebra is the two-line propagation in
`update_pos_vel_accel`. Position is `postype_t`, `double` on most builds, with `float` velocity and
acceleration, and a typed vector has one scalar type (F33): typing the triple would widen the 400 Hz
position loop to `double` for little checking. Units on these scalars (mp-units quantities, without
TypedLinearAlgebra) would be the fitting tool; left unchanged.

Environment notes: waf builds the directory of the last `configure` (its lock file), whatever `--out`
says, so configure before switching build directories; the SITL autotests need pymavlink 2.4.50; the
WebAssembly check uses emsdk 6.0.8, the CI version.

### Refresh 2026-10-11: CI triage, upstream updates

- PR 3 CI on ff49546737: 123 jobs passed, 92 failed. 89 of the failures are infrastructure: Docker Hub
  refused the container pulls (`toomanyrequests`, time-outs) in "Initialize containers"; they need a
  re-run, not a change. The other three jobs had two causes:
  - Arch Linux (GCC 16): mp-units enables its throwing constraints by itself when the compiler has
    constexpr exceptions, and they include a hosted-only header, which the freestanding configuration
    rejects. `fcarouge.py` now defines `MP_UNITS_API_THROWING_CONSTRAINTS=0` (F14). Reproduced with
    GCC 15 by forcing the macro to 1; GCC 16 itself is only available in CI.
  - clang scan-build: with clang 20 on the C++26 build, a suppressed finding in `GCS_Common.cpp` is no
    longer reported, and the job fails on stale suppressions; its clang 20 hash is dropped. Not run
    locally (CI will tell).
- The fork's `master` has not moved since the last rebase. Submodules bumped: Kalman ad6ea6dc9 (samples,
  tests and CI only), TypedLinearAlgebra 1b91a3678; mp-units unchanged.
- TypedLinearAlgebra 1b91a3678 verifies the types of a division (F32 fixed: `divide_positive_definite()`
  lost its own check and the element type helpers) and of conversions between typed matrices, which
  used to relabel the elements silently: every port so far builds unchanged under the new checks. Its
  in-place `matrix_rank_1_update` is what the normal-equation accumulations need, but it requires
  `<linalg>`, which no toolchain here has (F28 stays).
- Environment: the working tree had no submodule checked out. Cloning TypedLinearAlgebra downloads
  2.2 GB (F36); `git submodule update --init --reference <local clone> --dissociate` takes seconds. The
  Arm toolchain, emsdk and the Python packages of the autotests (numpy, MAVProxy, pymavlink 2.4.50) do
  not survive between sessions: a virtual environment and the Arm toolchain were set up again; emsdk
  was not, so the WebAssembly build of this work is left to CI.

### EKFGSF yaw estimator (`AP_NavEKF/EKFGSF_yaw`)

- Baseline test first: `AP_NavEKF/tests/test_ekfgsf_yaw.cpp` simulates a multicopter (thrust along the
  specific force) from rest, IMU at 400 Hz and GPS velocity at 10 Hz. Lesson: accelerations sustained
  longer than the 5 s tilt correction bias the estimator itself (9 degrees with 15 to 25 s periods), so
  the test uses 2 to 4 s periods; the previous code then finds the yaw within 3.2 degrees.
- Each of the five models is a Kalman filter over typed `[velocity N m/s, velocity E m/s, yaw rad]`,
  output the NE velocity, with a transition function (the delta velocity added in the earth frame). The
  scalar type is `ftype`, double or float: the `AP_LinearAlgebra::Units` aliases are float only, so the
  quantities are declared locally (to generalize before EKF3). What the library does not model is done
  around it (F7, F23): the yaw state is overwritten from the AHRS before each prediction; the Jacobian
  (`[1 0 -dvE; 0 1 dvN; 0 0 1]`, in (m/s)/rad) and the rotated process noise are set with `f()` and
  `q()` before `predict()`; the innovation gate and the 5-sigma compression are computed first, and
  the compression is passed through the measurement; variance limits and symmetry are applied on `p()`
  after each step. The Gaussian sum (weights, density) is unchanged scalar code on the stored innovation
  and its variance.
- What typing and the comparison exposed: the previous chi-square test ratio takes `+S01/det` for the off
  diagonal element of the inverse innovation variance; the inverse has `-S01/det`. Written as
  `transposed(y) / S * y`, the estimates diverged from the previous code (models with large innovations
  first, then weights and resets). The previous formula is kept, with a note in the code: flagged for
  human review, as it changes when innovations are compressed.
- Equivalence over 200 random 30 s flights (GPS noise, 40 m/s velocity spikes, restarts, airspeed
  compensation), previous and new estimators side by side: yaw within 6e-13 rad, weights within 1e-14,
  innovations within 5e-13 m/s in double precision; yaw within 3.4e-4 rad in single precision; the same
  clipped models and resets in both.
- Costs (F35, F37, F38). Memory: 58 scalars per filter instead of 18; the estimator, allocated per EKF
  core, grows from 1712 to 3592 bytes (double) and from 856 to 1824 (single). Stack on CubeOrange: the
  library's `update()` alone needs a 1416 byte frame, above the 1300 byte limit; it is called from a
  function of its own and the limit raised to 1500 for this file on ChibiOS with a double precision EKF
  (the EKF cores already raise theirs); a correction takes 1944 bytes of the 7168 byte main thread stack
  where it took 288, a prediction 872 where it took 96. A first version with the typed
  `H * P * transposed(H) + R` took 840 bytes more and was rewritten on quantities. Time on x86-64: five
  corrections 1203 ns against 271, five predictions with the attitude filters 1019 ns against 550.
  Flash: CubeOrange copter +7392 bytes, MatekF405 copter +6704.
- Validation: `test_ekfgsf_yaw` with g++ 15 and clang 20, double and single precision (the single
  precision SITL build needs two unrelated fixes in `SIM_ADSB.cpp` and `SIM_Scrimmage.cpp`, applied
  locally and not kept); CubeOrange and MatekF405 copter with `-Werror`; autotests `Copter.GSF`,
  `GSF_reset` and `EKFYawResetLogged` from a clean build. Not done: Replay, WebAssembly (CI), timing on
  a flight controller.
- Human review requested: sensor fusion code, the stack budget on double precision boards, and the test
  ratio sign.

### Solo gimbal EKF (`AP_Mount/SoloGimbalEKF`): assessed, not ported yet

- Structure: 9 error states (angle error rad, velocity m/s, delta angle bias rad) with the quaternion
  kept outside the filter, 380 lines of generated covariance prediction, sequential scalar fusion of
  the three velocity components (direct state observations) and of a compass heading (a row with three
  non-zero elements), and a reset of the angle error into the quaternion after each fusion. It reads
  the vehicle's AHRS, compass and clock through their singletons.
- It needs from the Kalman library what EKF3 will (F7): scalar updates of different output types
  (m/s, rad) on one state and covariance, and the stack and time of F37 and F38 at 9 states, in single
  precision.
- The model is available in closed form, so the generated covariance prediction can become
  `F P F' + Q` on typed matrices: `AP_NavEKF/Models/GimbalEstimatorExample/calcF.m` (12 states, the
  last three, delta velocity biases, unused here) and `calcQ.c`. F is the identity plus three 3x3
  blocks (angle error propagation, velocity from angle error, angle error from delta angle bias) and Q
  two blocks (delta angle noise, delta velocity noise rotated to NED): a use for typed blocks (F26).
- Validation is better than "build only": the autotest `Copter.MountSolo` runs it in SITL against the
  simulated gimbal, and passes on this branch (baseline, 2026-10-11). Because of the singletons, the
  equivalence is best measured in SITL with both implementations run side by side on the same inputs,
  rather than in a unit test.
- Found while reading, to settle before a port can be compared: the velocity fusion updates the
  covariance in place, `Cov[row][col] -= K[row] * Cov[stateIndex][col]`, so the rows after the fused
  state use a row already scaled by `1 - K[stateIndex]`, and are under-corrected; the symmetry forced
  afterwards averages the error into the upper triangle. On a random 9x9 covariance the result differs
  from `P - K H P` by 16% of the largest element before the symmetry and 8% after. The compass fusion
  buffers `H P` first and is not affected. `fixCovariance()` also skips the first diagonal element in
  its non-negative clamp. A library filter cannot reproduce the in-place update: flagged for human
  review, and the port will be compared against the previous code with this update corrected.

### Stack of the library filters at 9 and 24 states: a blocker for the next filters

Measured with a probe filter of N states and one output, for Cortex-M7 (Arm GCC 15.2, `-O2`,
`-fstack-usage`), on the pinned libraries (F38). Stack frame of `update()` / `predict()` in bytes:

| States | Single precision | Double precision |
|---|---|---|
| 3 | 680 / 304 | 1816 / 552 |
| 9 (gimbal EKF) | 4152 / 2296 | 8200 / 4560 |
| 24 (EKF3) | 28192 / 16152 | 56328 / 32280 |

- The flight controller's main thread has 7168 bytes of stack and ArduPilot allows 1300 bytes per function
  (2600 in the double precision EKF cores). Each scalar update of the gimbal EKF would take 4152 bytes,
  and EKF3 could not run at all: the dense update keeps about 12.5 state-sized matrices alive, the
  prediction 7.
- Experiment on a scratch copy of the Kalman header (not kept): naming the intermediates so that each
  statement holds few temporaries gives 2448 / 1968 at 9 states in single precision and 32896 / 27672 at
  24 in double; the simple covariance update instead of the Joseph form brings `update()` to 1432 and
  18864. Useful, not sufficient: the remaining frames are the matrices returned by value by every typed
  operation (F28).
- Consequence for the plan: the gimbal EKF and Phases 7 and 8 wait for upstream work on evaluation in
  place, in TypedLinearAlgebra (operations writing into an existing matrix, without `<linalg>`) and in
  Kalman (update and prediction written with them, scalar sequential updates, a replaceable covariance
  prediction). The small filters already ported (4 states and fewer) are within the limits, with the
  EKFGSF exception documented above.

### Next (as of 2026-10-11, superseded)

1. Watch PR 3 CI on the new commits; re-run the jobs that only failed on Docker Hub pulls.
2. Upstream, in the FrancoisCarouge repositories (Phase 2), in order of what blocks ArduPilot most:
   in-place typed operations (F28) and an update and prediction written with them (F38, F37), the simple
   covariance update as an option (F8), a two-stage update and scalar sequential updates (F7), callable F
   and Q with a transition function (F23), repository size (F36). The probe of the section above
   measures progress. Report the mp-units freestanding defect (F14) to mp-units.
3. `SoloGimbalEKF`, the last small filter and a rehearsal of EKF3: port it once its update fits the
   stack; see the assessments above. Its side-by-side SITL comparison can be prepared meanwhile.
4. Generalize `AP_LinearAlgebra::Units` to a scalar type parameter before EKF3 and EKF2.
5. Decide the EKFGSF test ratio sign with a maintainer.

### Refresh 2026-10-13: rebase, upstream updates, CI

- PR 3 CI on 79502ecf70 (2026-10-11): 119 jobs passed, 3 skipped, 1 failed. The Arch Linux (GCC 16),
  scan-build, unit test, WebAssembly and ChibiOS jobs pass, so the two fixes of 2026-10-11 are confirmed.
  The failure, `colcon build/test`, is a schema download from `download.ros.org` ("unreachable network").
  Flash cost of the whole branch against `master`, from the CI size table: copter +8.4 kB (MambaH743v4)
  to +12.7 kB (Durandal, MatekH743), plane +13.0 to +16.1 kB, rover +8.3 to +13.2 kB, 1 MB boards
  (MatekF405, KakuteF7, Pixhawk1-1M) +10.8 to +14.8 kB.
- `fcarouge` rebased onto the fork's `master` (102 new commits, no conflict), pushed with
  `--force-with-lease`; the previous head is kept locally as `backup/fcarouge-2026-10-11`.
- Submodules bumped: Kalman f897f39c8 (declared matrices owned by the declaration, declared output type
  honored, constrained constructor, `predict()` and `update()`), TypedLinearAlgebra ac4c3567a (`std::array`
  and Boost.Multi samples); mp-units unchanged. No finding is closed by them; the open ones were
  rechecked in the sources, and the stack probe gives the same frames (F38).
- Validation after the rebase and bumps, from a clean build: the seven typed unit tests; autotests
  `Copter.GSF`, `GSF_reset`, `EKFYawResetLogged`, `PrecisionLanding`, `PrecisionLoiterCompanion`,
  `FixedYawCalibration`, `SITLCompassCalibration`, `CopterMission`, `WPArcs`, `SplineTerrain`, `NavDelay`,
  `MountSolo`, `Plane.Soaring`, `IMUTempCal`, `AIRSPEED_AUTOCAL`; CubeOrange and MatekF405 copter and plane
  with `-Werror`.
- Inventory sweep (Kalman, covariance, innovation, Jacobian, least squares, matrix inverse, over the
  libraries and vehicles): nothing new. The optical flow calibration is a weighted average of per-sample
  scalars, not linear algebra, and is excluded.

### IMU temperature correction (`AP_InertialSensor_TCal::polynomial_eval`)

- The evaluation side of the temperature calibration, left out when `PolyFit` was typed: the
  coefficients of each axis (sensor/K, sensor/K^2, sensor/K^3, stored scaled by 1e6) form a typed
  3x3 matrix, a column per power, multiplied by the typed vector of the powers of the temperature
  difference. The sensor has a dimension of its own, as accelerometers and gyroscopes share the code;
  the parameter scale is the unit of the result (`mag_power<10, -6>` of the sensor unit), so the
  conversion to the sensor unit replaces the multiplication by `INV_SCALE_FACTOR`; the cast of the
  parameter array to `Vector3f *` is gone.
- mp-units requires temperature differences to be built with `delta<kelvin>()` (a deprecation warning
  otherwise): the typed code says that the argument is a difference, which the float did not.
- Equivalence over 200000 random coefficient sets and temperatures: at most 9e-8 from the Horner form,
  rounding. This runs for every IMU sample: 184 bytes of code, CubeOrange copter +92 bytes, same time
  within the resolution of the host measurement. `Plane.IMUTempCal` passes from a clean build.

### Cleanup: `MatrixN` removed

- With the thermal EKF on the Kalman library, `MatrixN` had no user: `AP_Math/matrixN.h` and
  `matrixN.cpp` are removed, with `VectorN::mult()`. `VectorN` stays (index checked arrays of the EKFs).
  The unused array types of `EKFGSF_yaw.h` are removed too.
- Also without a flight code user now: `mat_inverse()`, `mat_mul()` and `mat_identity()` of
  `AP_Math/matrix_alg.cpp`, whose inverse allocates on the heap; the calibrations that called them use the
  typed solves. Only `AP_Math/examples/matrix_alg` uses them, and they are not linked into the firmware.
  Left in place: removing a general utility and its example is the owner's call.
- `AP_LinearAlgebra::Units::Quantity` takes the scalar type as a second argument (float by default), used
  by the EKFGSF with `ftype`.

### EKF3 with TypedLinearAlgebra alone: a scalar fusion fits when the covariance is updated in place

A probe of one EKF3-like scalar fusion (appendix): 24 typed states of 6 units (quaternion, velocity,
position, delta angle bias, delta velocity bias, earth and body field, wind), a typed 1x24 output model,
`PHt = P * transposed(H)`, `S = H * PHt + R`, `K = PHt / S`, `x = x - K * innovation`, and the covariance
update `P = P - K * transposed(PHt)`.

| | Single precision | Double precision |
|---|---|---|
| Stack frame, covariance updated with typed operations | 9528 | 19048 |
| Stack frame, covariance updated in place on the storage | 608 | 1200 |

- Cortex-M7, Arm GCC 15.2 `-O2`, bytes. In place means a loop over the backend storage, the type of the
  expression `P - K * transposed(PHt)` being checked against the covariance type without evaluating it,
  the workaround of F28 used by the calibrations.
- Compile cost on the host (g++ 15 `-O2`): 3.2 s and 590 MB for the translation unit, 1.6 s and 400 MB
  of it for the headers: a typed 24x24 covariance of 576 element types is affordable (F22).
- Consequence: the fusion steps of EKF3 (and the gimbal EKF) can be typed now, without the Kalman
  library, keeping the generated covariance prediction; the Kalman library joins when F38 is solved.
  Done for the true airspeed, see the pilot below.
  What this needs in ArduPilot: a typed view over the existing `Matrix24 P` storage (a non-owning
  backend), the inhibited states (`stateIndexLim`, the gain mask), and a way to use the sparsity of H,
  five non-zero elements of 24 for the airspeed (F26).

### Phase 7 pilot: EKF3 true airspeed fusion (`NavEKF3_core::FuseAirspeed`)

- Side by side first: a typed computation of the Jacobian, innovation variance, gain and `K H P` ran
  beside the generated code in SITL and reproduced it (gain within 3e-13, the rest within 4e-16), on a
  copy of the covariance and then on a view of it.
- `AP_LinearAlgebra` gains `MatrixView`, a read only backend over storage owned elsewhere, and
  `TypedView<M>`: TypedLinearAlgebra composes over it unchanged, so the filter's `statesArray` and `P`
  arrays are used typed without a copy. `AP_NavEKF3_typed.h` declares the 24 typed states (quaternion
  unitless, velocity m/s, position m, delta angle bias rad, delta velocity bias m/s, earth and body field
  Gauss, wind m/s), the covariance, their views, and `ObservationModel<Observation>`; it is included by
  the fusion source file only, before that file's `P` macro.
- `FuseAirspeed()` now builds a typed 1x24 Jacobian, `PHt = P * transposed(H)`, `S = H * PHt + R`,
  `K = PHt / S`; the gain mask (a run time selection, applied when copying the gain from the backend
  storage), the consistency test and `FinishFusion()` are as before; `K H P` goes to the `KHP` array by a
  loop, the type of `K * (H * P)` being checked against the covariance type unevaluated (F28). The
  generated covariance prediction is untouched.
- What typing said: a first unit test of the view multiplied the covariance by a state vector, which
  TypedLinearAlgebra rejected, rightly: `P x` adds m^3 to m^3/s^2 (F10). The Jacobian elements for the
  velocities and the wind are unitless, as the types require.
- Equivalence in flight, the previous code computing beside the typed one on the same inputs: 38450
  fusions over `Plane.AIRSPEED_AUTOCAL`, `Soaring`, `Deadreckoning`, `DeadreckoningNoAirSpeed` and
  `EKFlaneswitch`, with the wind only, all states and all but wind gain masks: gains within 9e-12
  relative, innovation variance and `K H P` within 6e-16 (double precision; single precision not run).
- Costs: time per fusion on the host equal within noise (603 ns typed against 643 ns generated in one
  run, 1235 against 934 in another; an optimized build, `autotest.py --no-debug`: the default autotest
  build is `-O0` and made the typed code look 8 times slower). The dense products do 24 terms where the
  generated code did 5 (F26). Stack frame of `FuseAirspeed()`: 96 to 1248 bytes on CubeOrange (double,
  EKF limit 2600), 32 to 656 on MatekF405 (limit 1300). Flash: +200 and +260 bytes for plane.
- Validation: those five autotests and `Plane.AirspeedDrivers`, `Copter.GSF` from a clean build; g++ 15,
  clang 20, Arm GCC 15.2 with `-Werror`. Single precision (`autotest.py --ekf-single`, with the two
  SITL lines of 2026-10-11 patched locally): gains within 3e-6 of the largest gain of each fusion, the
  variance within 4e-7 and `K H P` within 2e-7, rounding; one more mask seen (magnetic field states
  inhibited). `Plane.Replay`, whose log replay includes the airspeed and wind bits, passes. Not done:
  timing on a flight controller, WebAssembly (CI). Human review requested: sensor fusion code.

### EKF3 sideslip fusion (`NavEKF3_core::FuseSideslip`)

- Same method and structure as the airspeed. The assumed zero sideslip is observed as the ratio of the
  lateral to the forward wind relative velocity, as its innovation is computed, so the observation is
  unitless (a radian would need an explicit conversion from the ratio in mp-units).
- The generated intermediate terms became typed: the body forward and lateral velocities (m/s), their
  inverse and inverse square (s/m, s^2/m^2), the quaternion products (unitless). The Jacobian elements
  then check as unitless for the quaternion and s/m for the velocity and the wind, without any change
  to the expressions: the generated code was consistent.
- Equivalence in flight, about 40000 fusions over the same five autotests, masks including none and all
  states: gains within 8e-14 of the largest gain, `K H P` within 4e-16 (double precision); 4e-5 and
  1.2e-7 in single precision.
- Costs: stack frame 248 to 1344 bytes on CubeOrange, 128 to 696 on MatekF405; flash -256 and +8 bytes
  for plane. g++ 15, clang 20, Arm GCC 15.2 with `-Werror`.
- Environment notes: `m.at<i, j>()` cannot be an argument of a function-like macro (gtest, `fmaxF`), the
  comma splits it; TypedLinearAlgebra's call form with index literals, `m(0_i, 1_i)`, can, and is the
  one to use there. clang `-Wshadow` rejects a variable named `H` where mp-units' unit symbols are
  visible (the henry).

### 2026-10-14: owner decisions applied, drag fusion

Owner decisions of 2026-10-14: remove the generic matrix helpers, fix the EKFGSF test ratio sign, keep the
toolchains between sessions, continue.

- Toolchains: the Arm GNU Toolchain 15.2 and the autotest Python environment (numpy, MAVProxy, pymavlink
  2.4.50) now live in the workspace, outside the repository, in `ardupilot/toolchains/`;
  `. ardupilot/env.sh` puts them on the path.
- CI regression after the rebase: the fork's `master` gave scan-build builds `-fno-builtin` (so that the
  clang analyser accepts the zero initialisation flag), and with it the gcc that compiles them refuses
  to evaluate the C library `pow`, `sqrt` and `cbrt` that mp-units calls in constant expressions;
  scan-build stopped in `AC_PrecLand`. Scan-build builds, analysed and never run, now get no zero
  initialisation flag. Checked locally with scan-build 20 and g++ 14.
- `mat_inverse()`, `mat_mul()`, `mat_identity()` (`AP_Math/matrix_alg.cpp`, with a heap allocating
  inverse) and their example are removed.
- EKFGSF test ratio: `-S_NE / det(S)` for the off diagonal element of the inverse innovation variance.
  Effect against the previous sign over 200 random simulated 30 s flights with spikes and restarts: the
  same yaw error after 15 s for the multicopter flights (RMS 10.0 degrees for both), 58.7 against 59.2
  degrees for the airspeed compensated ones (not physical in this simulation); the estimates differ
  where the compression differs. `Copter.GSF`, `GSF_reset`, `EKFYawResetLogged` pass.

### EKF3 drag fusion (`NavEKF3_core::FuseDragForces`)

- Same method as the airspeed and sideslip. The drag model is typed with the documented parameter units
  (ballistic coefficient kg/m^2, air density kg/m^3, momentum drag coefficient 1/s): the implied
  airspeed, the derivative of the specific force (1/s) and the predicted specific force (m/s^2) check.
  The Jacobian is in m/s^2 per quaternion unit and 1/s for velocity and wind; only the wind gains are
  kept, as before.
- The side-by-side check caught a port error: the previous code reads the states through references,
  so the second axis sees the wind just corrected by the first, which is what sequential fusion
  means; the first port copied them once before the loop (innovation variance 5e-4 relative off on
  the second axis). Fixed by reading the states per axis.
- Equivalence in flight, about 6000 fusions in `Copter.DeadReckoningInWind` and
  `GPSForYawWindEstimation` (mixed bluff body and momentum drag): gains within 4e-15, innovation
  variance, predicted specific force and `K H P` within 7e-16 (double); 2e-6, 4e-7, 2e-7, 3e-7
  (single). The bluff body only and momentum only models are not exercised by an autotest.
- Costs: stack frame 424 to 1432 bytes on CubeOrange (limit 2600), flash -840 bytes; 1 MB boards do not
  have drag fusion.
- mp-units `abs()` does not compile in the freestanding configuration (no `abs` for the representation
  without the hosted `<cmath>`); the absolute value is taken on the numerical value (F14).

### EKF3 3-axis magnetometer fusion (`NavEKF3_core::FuseMagnetometer`)

- The predicted body field, the common terms and the three Jacobians are typed (Gauss per quaternion
  unit, unitless for the earth field and body bias states, the field in Gauss).
- The previous code's structure is kept on purpose: the three innovation variances are computed once,
  from the states and covariance at entry, and each axis' gain uses the covariance already corrected by
  the previous axes with that earlier variance. The check generated for it compares the variances at
  entry and every axis against the live covariance.
- Equivalence in flight, about 21000 axis fusions in `Copter.CopterMission`, `SITLCompassCalibration`,
  `GSF_reset` and `MotorVibration`: variances within 9e-16, gains within 2e-14 of the largest gain,
  `K H P` within 2e-16 (double); 4e-7, 1.2e-5, 1.3e-7 (single). Only the all-but-wind gain mask occurred.
- Costs: stack frame 712 to 1752 bytes on CubeOrange (limit 2600), 328 to 912 on MatekF405 (limit
  1300); flash -1348 and -784 bytes.

### 2026-10-14 (afternoon): CI, library bumps, declination and yaw fusions

- PR 3 CI on db9fbcc200: 213 jobs passed, 2 failed. Scan-build (the fix of the morning) and WebAssembly
  pass. The failures are not this work: the Renode quadplane flight could not download its model data
  ("Name or service not known"), and `QuadPlane.QAUTOTUNE` ("Failed to level") failed in the
  pull-request run but passed in the push run of the same commit, and twice locally.
- Submodules bumped: Kalman 6b54da0bc (setters taking several values, printer), TypedLinearAlgebra
  db34d69db (`matrix_inf_norm`). The typed unit tests, copter and plane build unchanged.
- Declination fusion (`FuseDeclination`): observed in radians, Jacobian in rad/Gauss over the North and
  East field states; its conditioning test is kept as the previous denominator, `S * HK0^2`, against the
  same threshold. About 8000 fusions in four copter autotests: gains within 1.2e-14 of the largest,
  `K H P` within 9e-17 (double); 1.9e-6 and 3.4e-8 (single). Stack frame 64 to 1248 bytes on CubeOrange,
  56 to 640 on MatekF405; flash +392 and +352 bytes.
- Yaw fusion (`fuseEulerYaw`): the generated quaternion Jacobian and its singularity handling are kept,
  as there are no units to check in it; the observation model (rad per quaternion unit), variance and
  gain are typed, gains written up to `stateIndexLim` as before. About 6400 fusions with magnetometer,
  GPS and static yaw (not GSF, not external navigation): within 6e-16 (double), 2.3e-7 (single). Stack
  frame 288 to 1408 bytes on CubeOrange, 128 to 696 on MatekF405; flash +456 and +292 bytes.

### EKF3 velocity and position fusion (`NavEKF3_core::FuseVelPosNED`)

- Six direct observations (velocity N, E, D in m/s, position N, E, D in m), each an observation model
  selecting one state. The state index is a run time value; the typed fusion is a template on it,
  dispatched by a `switch` (six instantiations).
- `AP_LinearAlgebra` gains `row<i>()` and `column<j>()`, typed selections of a row or column (the gap of
  F26). Two lessons on the way:
  - Dense typed products with a selector H were folded by the host compiler, so selection did not
    change the host time; it matters on the target, where nothing guarantees the folding.
  - Building the selected vectors with TypedLinearAlgebra's element constructor cost 21.9 kB of flash
    on MatekF405 (about 1.2 kB of unrolled code per 24-element vector type, F39); copying the storage,
    the element types being the same, costs nothing: +2188 bytes for the whole fusion, against +2932 with
    the dense products.
- Equivalence in flight, about 88000 observations in `Copter.CopterMission`, `GPSGlitchLoiter`,
  `VisionPosition` and `DeadReckoningInWind`: variances identical, gains within 2.2e-16, `K H P` within
  4.4e-16.
- Costs: 191 ns against 62 ns per observation on the host (7 times slower with the element
  constructor); stack 992 bytes in the helper on CubeOrange, 512 on MatekF405; flash +2196 and +2188.

### EKF3 body frame velocity fusion (`NavEKF3_core::FuseBodyVel`)

- The 570 lines of generated scalar code for the three axes become typed expressions; the generated
  Jacobian expressions themselves are kept, on typed states, and check (m/s per quaternion unit,
  unitless for velocity). As in the previous code, the Jacobians use the states at entry and each axis
  the covariance corrected by the previous ones; gains 0 to 6 always apply, the others are masked.
- Equivalence in flight, about 2850 fusions in `Copter.BodyFrameOdom` and `Rover.WheelEncoders`, four
  gain masks: within 7e-16 (double), 3.6e-7 (single).
- Costs: stack frame 864 to 1480 bytes on CubeOrange; flash -3040 bytes; not on 1 MB boards.

### EKF3 AGL height filter on the Kalman library (`NavEKF3_core::UpdateAglKf`)

- Found during the Phase 7 work: upstream added an optional two-state filter of the height above ground
  and its rate (`EK3_OPTIONS` bit 3), hand written on scalar members, and the one-state terrain offset
  estimator sits beside it in the same file; both were missing from the inventory, now added.
- The AGL filter is a direct fit for the Kalman library, as the precision landing filter was: typed
  `[height m, velocity m/s]`, input the vertical acceleration, F and G from the time step, process noise
  from prediction arguments, rangefinder update with H = [1 0] and the library's Joseph form, which is
  what the expanded expressions computed. The decay, floor, caps, reset and outlier gate stay outside
  the filter. Fixed storage in the core, constructed when the variables are initialised.
- First flight: SITL crashed as soon as disarmed logging started, reading the filter of a core before
  its variables were initialised; the zero-allocated members used to read as zero, so the accessors now
  return zero until the filter exists.
- No autotest sets the option: checked with it added to the copter defaults for the run. The check ran
  the previous filter as a shadow on the same inputs, per core (a single shadow mixing the cores first
  reported large, false, differences). About 34000 steps and 6500 updates: height 6e-16, velocity
  4e-13, covariance 4e-15 (double); 3e-7, 1.2e-3 of near zero velocities, 3e-6 (single). Reset and
  outlier paths not exercised.
- Costs on CubeOrange: stack frame 72 to 1896 bytes (limit 2600), flash +6364 bytes, 320 bytes per core
  instead of 48 (F35). Not on 1 MB boards.

### Objective 4, units: audit (2026-10-14)

Where the typed code stood when the objective was added, and what is missing:

| Place | Units now | Gap | Action |
|---|---|---|---|
| Thermal EKF, precision landing, airspeed calibration, EKFGSF, AGL filter | physical units inside the filter | interface takes plain floats | see below |
| Accelerometer and compass calibrations | m/s/s, mGauss; scale and cross-axis factors unitless (correct) | none inside | none |
| SCurve | m, m/s, m/s/s, m/s/s/s | none | none |
| IMU temperature correction | an opaque "sensor" dimension | real units | done: m/s/s and rad/s per sensor, the sensor passed as an enum |
| `PolyFit` (temperature learning) | opaque x and y dimensions | real units | kept: a generic fit whose header is included by `AP_InertialSensor.h`; its callers' units would have to enter that header |
| EKF3 fusions ported so far | physical units for states, Jacobians, variances, gains; quaternion unitless (correct); sideslip observed as a ratio | inputs (noise variances, innovations) and the core's members stay plain | typed locally in each fusion; members stay plain (header cost) |
| EKF3 optical flow, range beacon, terrain offset, covariance prediction; gimbal EKF; EKF2 | none | everything | with their Phase 7/8 ports |

Typed interfaces, measured: a translation unit including mp-units' quantity and SI unit headers takes
0.6 s and 210 MB more (g++ 15 `-O2 -fsyntax-only`), with `AP_LinearAlgebra_Units.h` 1.7 s and 390 MB, 0.2 s
and 56 MB for `<cmath>` alone. The filter headers reach far: `PosVelEKF.h` reaches all 74 copter sources
through `AC_PrecLand.h` and `Copter.h`, `EKFGSF_yaw.h` every EKF2 and EKF3 source, `AP_Airspeed.h` is
included directly by 29 files. Units on these interfaces would add from tens of seconds to minutes of
CPU time per build; they need either a lighter quantity header (F22) or a filter kept behind an opaque
pointer so that its header does not reach the callers. Until then the units start inside the filter
source files, at the first line of each function.

### Next (as of 2026-10-14)

1. Phase 7, fusion by fusion: airspeed, sideslip, drag, 3-axis magnetometer, declination, yaw, velocity
   and position, body frame velocity done; next optical flow and range beacon. Then the one-state
   terrain offset filter (`EstimateTerrainOffset`) on the Kalman library, as the AGL filter now is.
   The side-by-side check is generated from the previous source by a script, and must compare what the
   previous code reads at each step, not only at entry.
2. EKF3 covariance prediction: stays generated until evaluation in place exists upstream.
3. Upstream (Phase 2), unchanged order: in-place typed operations (F28), update and prediction written
   with them (F38, F37), the simple covariance update (F8), a two-stage update and scalar sequential
   updates (F7), callable F and Q with a transition function (F23), typed blocks or sparse rows (F26),
   repository size (F36). Report the mp-units freestanding defects (F14) to mp-units.
4. `SoloGimbalEKF` and EKF2: the same typed fusions over views.
5. Decide with a maintainer: the gimbal EKF in-place covariance update.
6. Objective 4: try one typed filter interface behind an opaque pointer (the precision landing filter,
   used by `AC_PrecLand.cpp` only) and measure the build; extend if worthwhile.

## Appendix: stack probe

The probe behind the stack table: a filter of `PROBE_N` states of type `PROBE_T` and one output. From the
repository root, with the Arm GNU Toolchain on the path; the frames are read in `stack.su`.

```shell
arm-none-eabi-g++ -std=gnu++26 -O2 -mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb \
  -fno-exceptions -fno-rtti -fno-threadsafe-statics -fstack-usage -c stack.cpp \
  -DPROBE_N=9 -DPROBE_T=float \
  -DMP_UNITS_HOSTED=0 -DMP_UNITS_API_CONTRACTS=0 -DMP_UNITS_API_THROWING_CONSTRAINTS=0 \
  -Ilibraries -Imodules/Kalman/include -Imodules/TypedLinearAlgebra/include \
  -Imodules/TypedLinearAlgebra/support/mp_units \
  -isystem modules/mp-units/src/core/include -isystem modules/mp-units/src/systems/include
```

```cpp
// stack frames of a Kalman library filter of N states and one output
#include <AP_LinearAlgebra/AP_LinearAlgebra_Kalman.h>
#include <utility>

#ifndef PROBE_N
#define PROBE_N 9
#endif
#ifndef PROBE_T
#define PROBE_T float
#endif

namespace {
constexpr auto mps = mp_units::si::metre / mp_units::si::second;
using Q = mp_units::quantity<mps, PROBE_T>;
template <typename> struct StateOf;
template <size_t... I> struct StateOf<std::index_sequence<I...>> {
    template <size_t> using Element = Q;
    using type = AP_LinearAlgebra::ColumnVector<PROBE_T, Element<I>...>;
};
using State = StateOf<std::make_index_sequence<PROBE_N>>::type;
using Output = AP_LinearAlgebra::ColumnVector<PROBE_T, Q>;
using Covariance = AP_LinearAlgebra::OuterProduct<State, State>;
using OutputVariance = AP_LinearAlgebra::OuterProduct<Output, Output>;
using OutputModel = AP_LinearAlgebra::Quotient<Output, State>;

auto make_filter()
{
    using namespace fcarouge;
    return kalman{
        state{State{}},
        output<Output>,
        estimate_uncertainty{Covariance{}},
        process_uncertainty{Covariance{}},
        output_uncertainty{OutputVariance{}},
        output_model{[](const State &x) -> OutputModel {
            OutputModel h{};
            h.template at<0>(x.template at<1>() / x.template at<2>());
            h.template at<PROBE_N - 1>(x.template at<0>() / x.template at<1>());
            return h;
        }},
        transition{[](const State &x) -> State { return x; }},
        observation{[](const State &x) -> Output { return Output{x.template at<0>()}; }},
        update_types<>,
        prediction_types<>};
}
}

using Filter = decltype(make_filter());
extern Filter filter;
Filter filter{make_filter()};

__attribute__((noinline)) void probe_update(const Output &z)
{
    filter.update(z);
}

__attribute__((noinline)) void probe_predict()
{
    filter.predict();
}

unsigned probe_size()
{
    return sizeof(Filter);
}
```

## Appendix: EKF3 scalar fusion probe

Compiled as the stack probe, with `-DPROBE_T=double -DPROBE_STAGE=1` for the in-place covariance update
and `-DPROBE_STAGE=2` for the typed one (the Kalman include path is not needed).

```cpp
// a scalar fusion on an EKF3-like typed state: 24 states of 6 different units
#include <AP_LinearAlgebra/AP_LinearAlgebra.h>
#include <AP_LinearAlgebra/AP_LinearAlgebra_Units.h>

#ifndef PROBE_T
#define PROBE_T double
#endif

namespace {
using namespace mp_units;
template <auto R> using Q = quantity<R, PROBE_T>;
constexpr auto mps = si::metre / si::second;
using One = Q<one>;
using Vel = Q<mps>;
using Pos = Q<si::metre>;
using Ang = Q<si::radian>;
using Mag = Q<si::tesla>;
// quaternion, velocity, position, gyro bias, accel bias, earth field, body field, wind
using State = AP_LinearAlgebra::ColumnVector<PROBE_T,
    One, One, One, One, Vel, Vel, Vel, Pos, Pos, Pos, Ang, Ang, Ang, Vel, Vel, Vel,
    Mag, Mag, Mag, Mag, Mag, Mag, Vel, Vel>;
using Output = AP_LinearAlgebra::ColumnVector<PROBE_T, Vel>;
using Covariance = decltype(State{} * fcarouge::transposed(State{}));
using OutputModel = decltype(Output{} / State{});
}

extern State x;
extern Covariance P;
State x;
Covariance P;

#if PROBE_STAGE >= 1
// true airspeed like fusion: H is non zero for the velocities and the wind
__attribute__((noinline)) void fuse(const OutputModel &H, Vel innovation, Q<mps * mps> R)
{
    const auto PHt = P * fcarouge::transposed(H);
    const auto S = (H * PHt).at() + R;
    const auto K = PHt / S;
    x = State{x - K * innovation};
#if PROBE_STAGE >= 2
    // P = P - K * transposed(PHt), each typed operation returning a new matrix
    P = Covariance{P - K * fcarouge::transposed(PHt)};
#else
    // the same, in place on the storage, the types checked without evaluation
    static_assert(std::is_convertible_v<decltype(P - K * fcarouge::transposed(PHt)), Covariance>);
    for (size_t i = 0; i < 24; i++) {
        for (size_t j = 0; j < 24; j++) {
            P.data()(i, j) -= K.data()(i) * PHt.data()(j);
        }
    }
#endif
}
#endif
```
