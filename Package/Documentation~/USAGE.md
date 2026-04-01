# OpenCvPlugin Usage Guide

## Setup

Add the package to your Unity project's `Packages/manifest.json` as a Git URL or local `file:` path. Unity will resolve the package and its native DLLs automatically — no manual file copying is required.

## API Overview

### OpenCvPluginNative

Low-level P/Invoke declarations and version checking. Call `VerifyVersion` on startup to catch DLL mismatches early.

### PnPNative

Raw P/Invoke declarations for the PnP solver. Most consumers should use `PnPSolver` instead.

### PnPSolver

The main entry point for pose estimation. Create one instance and reuse it across frames — it pre-allocates its working buffers on construction.

**Solve** takes flat `float[]` arrays of 3D object points (x,y,z,x,y,z,...) and 2D image points (x,y,x,y,...), plus a 3×3 camera intrinsic matrix in row-major order. Optional distortion coefficients may be provided; pass `null` to assume no distortion.

**Algorithm selection** — several OpenCV solver methods are exposed via `PnPNative` constants. `SOLVEPNP_SQPNP` is a good default for clean correspondences. `SOLVEPNP_ITERATIVE` is preferred when using temporal coherence (see below). `SOLVEPNP_EPNP` is faster and suitable as the RANSAC kernel.

**Temporal coherence** — when tracking a moving object across frames, pass `useExtrinsicGuess: true` to seed the solver with the previous frame's pose. This improves stability and convergence speed. Reset the pose (or pass `false`) when tracking is lost.

**RANSAC** — use `SolveRansac` when point correspondences may contain outliers (e.g. from feature matching). It exposes the standard OpenCV RANSAC parameters: iteration count, reprojection error threshold, and confidence.

**PnPSolver implements `IDisposable`** — dispose it when done to release native memory.

### GeometryUtils

Static helpers for coordinate system conversion:

- `RodriguesToUnityQuaternion` — converts an OpenCV Rodrigues rotation vector to a Unity `Quaternion`, accounting for the difference between OpenCV's right-handed Y-down coordinate system and Unity's left-handed Y-up system
- `TranslationToUnityPosition` — converts an OpenCV translation vector to a Unity `Vector3`
- `GetCameraMatrix` — computes a camera intrinsic matrix from a Unity `Camera`

## Sample

The **Basic Usage** sample (importable from the Package Manager) demonstrates version checking, basic PnP solving, temporal coherence, and Unity coordinate conversion.
