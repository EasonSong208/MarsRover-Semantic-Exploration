# PIDNet-S V3 stationary semantic deployment

Date: 2026-08-06

## Scope and safety

The test deployed only the project-owned observation and compatibility nodes. The
Dabai camera was already running. No camera driver, RTAB-Map, Nav2, chassis or arm
node was started or stopped, and no command topic or navigation goal was used.
Source was temporarily synchronized from the current uncommitted WSL worktree;
no commit or dependency installation was performed.

## Expected result

Load the five-class V3 checkpoint from SSD, publish header-preserving 640 x 360
`mono8` masks, feed the existing semantic fusion implementation, and run for
three minutes without inference errors or CUDA OOM.

## Actual result

- SSD model load: PASS; epoch 78, 453 inference tensors loaded, 26 training-only
  auxiliary tensors skipped.
- Target build/tests: PASS; one package built and 17 tests passed.
- Static inference: PASS; three non-single-class masks, values limited to 0-4,
  saved raw/mask/color/overlay output, steady FP16 inference about 34 ms.
- ROS interface: PASS; reliable/volatile `mono8`, 640 x 360, copied input header.
- Existing backend: PASS through the project-owned plain/structured XYZ adapter;
  a semantic cost grid was received. Navigation source remained unchanged.
- Continuous run: PASS for 180.03 seconds; average inference 49.93 ms, callback
  73.92 ms, output 11.73 Hz, estimated latest-frame drop 51.6%, no CUDA OOM or
  model error. Downstream cost/debug output covered over 175 seconds.
- Shutdown: PIDNet and compatibility nodes exited cleanly after the bounded run.

## Follow-up

Review SSD overlays, then test the real `/map_cropped` and map-frame fusion while
stationary. Do not start navigation or motion merely to perform that interface
check. The full command list and artifact paths are in `../pidnet_v3_deployment.md`.
