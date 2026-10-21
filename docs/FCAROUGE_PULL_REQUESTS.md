# Pull requests to propose upstream

Changes of the `fcarouge` branch that can go to ArduPilot `master` on their own, without the C++26
upgrade or the FrancoisCarouge libraries. Ordered by size, impact and risk, smallest first. The list
starts with five and grows as the work produces more; changes for the FrancoisCarouge libraries are
in [FCAROUGE_FINDINGS.md](FCAROUGE_FINDINGS.md). See [FCAROUGE_KALMAN_PLAN.md](FCAROUGE_KALMAN_PLAN.md)
for the work they come from.

Validated 2026-10-18: all seven commits cherry-picked onto upstream `master` `f48fabc426` without
conflict, in a separate worktree; there, SITL copter and the unit tests build with upstream's default g++
at `-std=gnu++11`, CubeOrange copter builds with the Arm GNU Toolchain 15.2, and `test_control`,
`test_polyfit`, `test_accel_calibrator`, `test_ekfgsf_yaw` and `Copter.MountSolo` pass. Hashes change
with each rebase of `fcarouge`: the commit subjects identify the candidates. Before opening one: rebase
on the current `master`, search for an open pull request on the same subject, follow
`.github/PULL_REQUEST_TEMPLATE.md`, and state the AI assistance (AGENTS.md).

| # | Title | Commits on `fcarouge` (2026-10-18) | Size | Flight code | Status |
|---|---|---|---|---|---|
| 1 | AP_Math: construct the test vector with parentheses | c5919c63d7 | 1 line, 1 file | no | proposed |
| 2 | Avoid deprecated increments of volatile | 0666017502, 1fd3884f53 | 12 lines, 3 files | yes, identical code | proposed |
| 3 | AP_Math: test polynomial fit | a81c7ee3f3 | +49 lines, 1 file | no | proposed |
| 4 | AP_AccelCal: add calibrator tests | aa22d74954 | +73 lines, 2 files | no | proposed |
| 5 | AP_NavEKF: test the EKFGSF yaw estimator | cef5254563 | +132 lines, 1 file | no | proposed |
| 6 | AP_Mount: correct the gimbal EKF velocity covariance update | 1fd109bb10 | 12 lines, 1 file | yes, Solo gimbal only | proposed, owner agreed 2026-10-15 |

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
  1e-3 m/s/s, scale factors within 1e-4), and under ASan since 2026-10-20: the test now releases the
  calibrator's sample buffer, which the ASan unit test job reported as a leak.

## 5. AP_NavEKF: test the EKFGSF yaw estimator

- What: a unit test of the Gaussian sum filter yaw estimator: a simulated multicopter flight (IMU at
  400 Hz, GPS velocity at 10 Hz), yaw found within 5 degrees after 20 s for constant and turning yaws,
  no estimate before velocity fusion, and an estimator constructed during static initialization.
- Why: the estimator behind the emergency yaw reset has no unit test, only autotests.
- Risk: test code only. Evidence: `test_ekfgsf_yaw` passes on `master` (4 tests).

## 6. AP_Mount: correct the gimbal EKF velocity covariance update

- What: `SoloGimbalEKF::fuseVelocity()` corrected the covariance in place, so the rows after the observed
  velocity state used its already corrected row; the fix copies that row first, as `fuseCompass()` does.
  `fixCovariance()` also skipped the first diagonal element in its non-negative clamp.
- Why: P - K*H*P with the wrong H*P; on an arbitrary covariance the result is off by up to 16% of the
  largest element.
- Risk: flight code, but only with a Solo gimbal (`MNT1_TYPE` SoloGimbal), and the effect in flight is
  small: at most 8e-6 of the largest covariance element over `Copter.MountSolo`, which passes on
  `master` with the fix alone.
  Owner agreed (option 1, 2026-10-15); an upstream maintainer still has to.

## Not proposed, and why

- AP_NavEKF3: the beacon vertical offset counted twice in the range beacon Jacobian (upstream code, found
  2026-10-15, see the plan): with a height source other than the beacons, `FuseRngBcn` normalizes the
  horizontal Jacobian by a distance whose vertical part has the offset twice, while the predicted range
  has it once; up to 3.2% on the gains in `Copter.BeaconPosition`. A behaviour change in flight code:
  ask a maintainer whether `bcn_pd` should drop the second offset before writing a fix.

- AP_HAL_ChibiOS: unhide RCOutput base class overloads (e5057fe9b1): upstream pull request #32357 (open)
  already handles `timer_tick`; the branch also unhides `serial_led_send` when serial LEDs are
  disabled. Propose that as a review comment on #32357 rather than a second pull request.
- AP_Common: add sized operator delete overloads (034e3bf0e6): does not compile on `master`, which
  builds as C++11 where sized deallocation does not exist ("no previous declaration"); it would need a
  `__cpp_sized_deallocation` guard first.
