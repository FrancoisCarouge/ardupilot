# Pull requests to propose upstream

Changes of the `fcarouge` branch that can go to ArduPilot `master` on their own, without the C++26
upgrade or the FrancoisCarouge libraries. Ordered by size, impact and risk, smallest first. The list
starts with five and grows as the work produces more; changes for the FrancoisCarouge libraries are
in [FCAROUGE_FINDINGS.md](FCAROUGE_FINDINGS.md). See [FCAROUGE_KALMAN_PLAN.md](FCAROUGE_KALMAN_PLAN.md)
for the work they come from.

Each entry was cherry-picked onto upstream `master` 90976d9fdf (2026-10-14) without conflict and
checked there: SITL with the default g++ 13 at `-std=gnu++11`, CubeOrange copter with the Arm GNU
Toolchain 15.2, the named tests run. Before opening one: rebase on the current `master`, search for
an open pull request on the same subject, follow `.github/PULL_REQUEST_TEMPLATE.md`, and state the
AI assistance (AGENTS.md).

2026-10-15: the branch rebased on `master` `b032bcbde3` without conflict; none of the five is upstream.

| # | Title | Commits on `fcarouge` | Size | Flight code | Status |
|---|---|---|---|---|---|
| 1 | AP_Math: construct the test vector with parentheses | 504c43f6b1 | 1 line, 1 file | no | proposed |
| 2 | Avoid deprecated increments of volatile | 6148e3042b, 675b10da4c | 12 lines, 3 files | yes, identical code | proposed |
| 3 | AP_Math: test polynomial fit | a154680822 | +49 lines, 1 file | no | proposed |
| 4 | AP_AccelCal: add calibrator tests | 276f2a4446 | +73 lines, 2 files | no | proposed |
| 5 | AP_NavEKF: test the EKFGSF yaw estimator | eabcf0b790 | +132 lines, 1 file | no | proposed |

## 1. AP_Math: construct the test vector with parentheses

- What: `test_control.cpp` builds a `Vector2f` with braces in an expression where the armhf GCC 14.2
  cross compiler reports `-Werror=unused-value`; parentheses construct the same vector.
- Why: keeps the unit tests building with GCC 14 cross compilers, which the fork's CI uses.
- Risk: test code only. Evidence: `test_control` passes on `master` (5 tests).

## 2. Avoid deprecated increments of volatile

- What: `x++` on a `volatile` becomes `x = x + 1` in `AP_ESC_Telem.cpp` (telemetry count),
  `shared_dma.cpp` (DMA contention statistics) and `CrashDump_SD.cpp` (a delay loop). Two commits, one
  per subsystem.
- Why: compound operations on `volatile` are deprecated in C++20 (`-Wvolatile`, `-Wdeprecated-volatile`
  in clang); the only standard-related warnings when building ArduPilot as C++20 or later.
- Risk: the same reads and writes. Evidence: CubeOrange object code of the three files is identical
  with and without the change (Arm GCC 15.2, `objdump -d`); SITL and CubeOrange copter build.

## 3. AP_Math: test polynomial fit

- What: a unit test of `PolyFit`, the fit used by the IMU temperature calibration: an exact cubic is
  recovered, and three temperatures (too few for the fit) are rejected.
- Why: `PolyFit` has no test; it feeds parameters that correct every IMU sample.
- Risk: test code only. Evidence: `test_polyfit` passes on `master` (2 tests).

## 4. AP_AccelCal: add calibrator tests

- What: unit tests of `AccelCalibrator` with synthetic samples from a sensor with known offsets, scale
  and cross-axis errors, for the 6 and 12 orientation fits; plus the tests' `wscript`.
- Why: the accelerometer calibration has no test; it sets the offsets and scale factors used in flight.
- Risk: test code only. Evidence: `test_accel_calibrator` passes on `master` (2 tests; offsets within
  1e-3 m/s/s, scale factors within 1e-4).

## 5. AP_NavEKF: test the EKFGSF yaw estimator

- What: a unit test of the Gaussian sum filter yaw estimator: a simulated multicopter flight (IMU at
  400 Hz, GPS velocity at 10 Hz), yaw found within 5 degrees after 20 s for constant and turning yaws,
  no estimate before velocity fusion, and an estimator constructed during static initialization.
- Why: the estimator behind the emergency yaw reset has no unit test, only autotests.
- Risk: test code only. Evidence: `test_ekfgsf_yaw` passes on `master` (4 tests).

## Not proposed, and why

- AP_HAL_ChibiOS: unhide RCOutput base class overloads (e5057fe9b1): upstream pull request #32357 (open)
  already handles `timer_tick`; the branch also unhides `serial_led_send` when serial LEDs are
  disabled. Propose that as a review comment on #32357 rather than a second pull request.
- AP_Common: add sized operator delete overloads (034e3bf0e6): does not compile on `master`, which
  builds as C++11 where sized deallocation does not exist ("no previous declaration"); it would need a
  `__cpp_sized_deallocation` guard first.
