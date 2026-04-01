# Unity OpenCV Plugin

Lightweight native OpenCV integration for Unity. Currently provides PnP (Perspective-n-Point) pose estimation and geometry utilities for OpenCV–Unity coordinate conversion.

## Modules

- **PnPSolver** — estimate camera/object pose from 2D–3D point correspondences
- **GeometryUtils** — Rodrigues↔Quaternion conversion, OpenCV↔Unity coordinate transforms, camera matrix helpers

## Usage

See [USAGE.md](Documentation~/USAGE.md) for integration instructions and API reference.

To see a worked example, import the **Basic Usage** sample via the Package Manager.

## Platform Support

- Windows x64
- Unity 6+

## Source & Build

Native plugin source code and build instructions: [github.com/UoA-eResearch/unity-opencv-plugin](https://github.com/UoA-eResearch/unity-opencv-plugin)
