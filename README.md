# OpenCvPlugin

A lightweight native C++ plugin that wraps selected [OpenCV](https://opencv.org/) functions for use in Unity (or any C#/.NET host) via P/Invoke. It is **not** a general-purpose OpenCV binding — it exposes only the specific computer-vision functions needed by consuming projects, wrapped in a C-compatible API that is safe to call from managed code.

## Current Modules

| Module | Purpose | Key functions |
|--------|---------|---------------|
| **PnP** | Perspective-n-Point pose estimation — estimate camera/object pose from 2D-3D point correspondences | `OCP_PnP_Solve`, `OCP_PnP_SolveRansac`, `OCP_PnP_ProjectPoints` |

New modules (e.g. point-set alignment) can be added by following the same pattern — see *Adding a new module* below.

---

## Design Principles

1. **Stateless compute, persistent buffers.** Each module exposes a "context" (an opaque handle) that holds only pre-allocated working memory. No pose state or results are retained between calls — the caller (C#) owns all meaningful state and passes it in each call.

2. **Zero per-frame allocation.** Contexts pre-allocate their buffers when created. Per-call work reuses existing capacity. This avoids unpredictable GC or heap pressure during real-time use.

3. **Errors never crash the host.** Every exported function catches all C++ exceptions (OpenCV and otherwise) and returns an integer status code. A human-readable error message is always available via `OCP_<Module>_GetLastError`. The DLL will never bring down Unity.

4. **C# owns the data, C++ borrows it.** Point arrays, camera matrices, and output buffers are allocated in C# and passed as pinned pointers. The plugin reads/writes through those pointers during the call and never holds references afterward.

5. **Module isolation.** Each module lives in its own subfolder under `source/`, has its own context struct, and its own set of `OCP_<Module>_*` exports. The only shared code is the top-level header (`OpenCvPlugin.h`) which defines the export macro, status codes, and error-handling helpers.

---

## Project Structure

```
OpenCvPlugin/
  CMakeLists.txt          # Build configuration
  build_windows.bat       # One-click build script for Windows
  README.md               # This file
  source/
    OpenCvPlugin.h        # Shared: export macro, status codes, error macros
    OpenCvPlugin.cpp      # Shared: OCP_GetVersion()
    pnp/
      PnPContext.h        # PnP context struct definition
      PnPContext.cpp      # PnP exported functions
```

---

## Prerequisites

### OpenCV (pre-built)

This plugin links against pre-built OpenCV binaries. You do **not** need to compile OpenCV from source.

1. Download the Windows pre-built package from https://opencv.org/releases/
   - The current version used is **4.12.0**.
2. Extract it somewhere on your machine (e.g. `C:\dev\libs\opencv\opencv-4.12.0`).
3. The extracted folder structure looks like:
   ```
   opencv-4.12.0/
     build/
       x64/
         vc16/
           bin/          <-- contains opencv_world4120.dll (you'll need this later)
           lib/          <-- contains OpenCVConfig.cmake (CMake finds OpenCV here)
     sources/            <-- not needed for this plugin
   ```
4. Open `CMakeLists.txt` and update the `OpenCV_DIR` path to point to **the `lib` folder** inside your extraction:
   ```cmake
   set(OpenCV_DIR "C:/dev/libs/opencv/opencv-4.12.0/build/x64/vc16/lib")
   ```
   This is the folder that contains `OpenCVConfig.cmake`, which is how CMake discovers OpenCV's headers and libraries.

### Upgrading to a newer OpenCV version

1. Download the new pre-built package and extract it alongside (or replacing) the old one.
2. Update the `OpenCV_DIR` path in `CMakeLists.txt` to point to the new version's `lib` folder.
3. Rebuild (see below).
4. Copy the **new** `opencv_worldXYZ0.dll` into your Unity project's `Assets/Plugins/x86_64/` folder (see *Deploying to Unity*).

### Build Tools

- **CMake** 4.2 or later — https://cmake.org/download/
- **Visual Studio 2022** (or later) with the "Desktop development with C++" workload — the build script uses the VS 2022 compiler toolset (`v143`)

---

## Building

### Windows (the easy way)

Double-click `build_windows.bat`, or from a terminal:

```
build_windows.bat
```

This will:
1. Create a `build/` directory if it doesn't exist.
2. Run CMake to generate a Visual Studio solution.
3. Compile both Release and Debug configurations.

The output DLLs end up in:
- `build/Release/OpenCvPlugin.dll` (use this one)
- `build/Debug/OpenCvPlugin.dll` (larger, slower — only for debugging native crashes)

### Windows (manual / if the bat file doesn't work)

```
mkdir build
cd build
cmake .. -G "Visual Studio 18 2026" -T v143 -A x64
cmake --build . --config Release
```

If you have a different Visual Studio version, change the `-G` generator string. Common values:
- `"Visual Studio 17 2022"` for VS 2022
- `"Visual Studio 18 2026"` for VS 2026

You can find your version by opening Visual Studio and checking *Help > About*.

---

## Deploying to Unity

Unity needs **two** DLLs placed in your Unity project at `Assets/Plugins/x86_64/`:

1. **`OpenCvPlugin.dll`** — the plugin you just built (`build/Release/OpenCvPlugin.dll`)
2. **`opencv_world4120.dll`** — the OpenCV runtime library, found at:
   ```
   C:\dev\libs\opencv\opencv-4.12.0\build\x64\vc16\bin\opencv_world4120.dll
   ```
   (The `4120` in the filename corresponds to version 4.12.0. If you upgrade OpenCV, this filename changes.)

Without the OpenCV DLL present, Unity will fail to load the plugin at runtime with a `DllNotFoundException`.

> **Tip:** Only use the **Release** build of both DLLs. Debug builds are significantly slower and have different filenames (e.g. `opencv_world4120d.dll`).

---

## C API Conventions

All exported functions follow these conventions:

- **Naming:** `OCP_<Module>_<Action>` (e.g. `OCP_PnP_Solve`)
- **Return value:** `int` status code — see `OpenCvPlugin.h` for the `OCP_OK`, `OCP_ERROR_*` constants
- **Error messages:** after a non-OK return, call `OCP_<Module>_GetLastError(ctx)` to get a `const char*` describing what went wrong
- **Data ownership:** all input/output arrays are owned by the caller. The plugin never allocates memory that the caller must free.
- **Handle lifecycle:** `CreateContext` -> use -> `DestroyContext`. Forgetting to destroy leaks memory.

---

## Adding a New Module

1. Create a subfolder under `source/`, e.g. `source/alignment/`
2. Create `AlignmentContext.h` — define your context struct with a `char errorMsg[OCP_ERROR_MSG_SIZE]` field and any pre-allocated buffers
3. Create `AlignmentContext.cpp` — implement `OCP_Align_CreateContext`, `OCP_Align_DestroyContext`, your compute functions, and `OCP_Align_GetLastError`
4. Add both files to the `add_library()` call in `CMakeLists.txt`
5. Use the `OCP_TRY` / `OCP_CATCH` macros from `OpenCvPlugin.h` for error handling
6. On the C# side, create matching `AlignmentNative.cs` (DllImport declarations) and `AlignmentSolver.cs` (high-level wrapper)