# eskf-vio: MSCKF visual-inertial odometry in C

15-state ESKF prediction at 200 Hz fused with MSCKF visual corrections from hand-rolled FAST-9 + Lucas-Kanade tracking and pinhole + distortion camera model. EuRoC MH_01: **3808 m --> 22.6 m** drift with IMU + camera only. Gyro bias norm recovered to ~1.3%.

Built from scratch as a learning vehicle. Not competitive with mature systems (OpenVINS-class filters reach sub-metre on this sequence) and not intended to be — yet.

## What's there

- **INS**, strapdown dead-reckoning: `gyro_to_q` (small-angle guard), accel to world frame, double integration
- **ESKF**, 15-state error propagation (pos 3, vel 3, att 3, ba 3, bg 3). Discrete error-state transition per Solà, exact per-tick rotation on the δθ diagonal, Q from ADIS16448 noise densities. `eskf_predict()` runs P <-- FPF^T + Q. Clone augmentation (6-dim P extension per pose), marginalization of oldest clone.
- **MSCKF**, Mourikis & Roumeliotis: linear multi-view triangulation (exact to 1e-15), per-observation Jacobians, null-space projection of feature states (Gram-Schmidt), Cholesky solve, chi-squared gating (5359 ok / 1691 rejected on MH_01)
- **VIO front-end**, FAST-9 + NMS + grid selection (~10k raw --> 231 spread corners), pyramidal KLT, forward-backward consistency gating, persistent feature IDs, dead-track management for MSCKF (max 12 obs per track)
- **Camera model**, pinhole + radial-tangential distortion, EuRoC cam0 calibration as compile-time constants (parameterisation planned), T_BS extrinsics

Every module is tested against known answers: Jacobians verified against finite differences to 1e-8, null-space annihilation and orthonormality, perfect-observation zero-correction, corrupted-clone recovery (>40% error reduction from a single track), outlier rejection with P untouched. 10 test binaries, ~70 checks, `make test` to run all of them.

## Architecture

Two-rate: ESKF prediction at IMU rate (200 Hz), MSCKF correction at camera rate (20 Hz). Error state injected into nominal via Solà right-multiplication: `q_true = q_nom ⊗  q(δθ)` for attitude, addition for pos/vel. Covariance reset Jacobian deferred (stage 5).

Clone poses augmented into `P` at each camera frame, marginalized when the window fills (MAX_CLONES=10). Null-space projection eliminates the 3D feature state from the measurement, leaving only pose residuals — the feature is never estimated, never stored.

## Results on EuRoC MH_01

Initialised from ground truth at t0 (pose, velocity, bias seed); no external input after that.

```
t [s]    pos err [m]  pred +- [m]  att err [deg]
0.0      0.00         0.00         nan
20.0     1.58         0.04         0.60
40.0     27.07        2.83         1.65
60.0     1.87         0.93         0.65
80.0     1.36         0.92         0.90
100.0    1.38         0.92         0.86
120.0    1.32         0.92         2.16
140.0    2.45         0.92         1.30
160.0    5.34         2.86         1.80
180.0    19.55        1.77         4.28
final: measured 22.6 m, predicted +-1.8 m
att 1-sigma: 1.053 deg
updates: 5359 ok, 1691 rejected
gyro bias est -0.00425 0.02139 0.07791 | true -0.00324 0.02134 0.07829
```

Dead-reckoning alone drifts 3808 m on the same sequence. Chi-squared gating rejects ~24% of tracks. The degradation windows (t≈40, t≈160–180) are under investigation; the filter remains ~12× overconfident on position, pending the consistency work in stage 5.

## Building

```bash
make test     # compile + run all unit tests
make run      # full EuRoC pipeline
make clean    # remove build artifacts
```

GCC + `libm`. Dataset at `~/datasets/euroc/MH_01_easy/` (or set `DATA_IMU`, `DATA_GT`, `DATA_CAM` in Makefile).

## References

- **Solà** — *Quaternion kinematics for the error-state Kalman filter* — ESKF propagation/update equations
- **Mourikis & Roumeliotis** — *A multi-state constraint Kalman filter for vision-aided inertial navigation* — MSCKF, null-space projection, clone poses

## Stage 5

Per-window diagnostics (accept rate, track length, overconfidence ratio), covariance reset Jacobian + Joseph-form update, FEJ (fixes spurious yaw-certainty), gyro-warm-started KLT, Jetson Orin Nano port, live camera pipeline.

