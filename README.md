# OpenCvPlugin

A lightweight native C++ plugin that wraps selected [OpenCV](https://opencv.org/) functions for use in Unity via P/Invoke. It is not intended as a general-purpose OpenCV binding, though it may expand over time. Currently it covers PnP pose estimation; new modules can be added by following the same pattern — see *Adding a new module* below.

---

## Design Principles

1. **Stateless compute, persistent buffers.** Each module exposes a "context" (an opaque handle) that holds only pre-allocated working memory. No pose state or results are retained between calls — the caller (C#) owns all meaningful state and passes it in each call.

2. **Zero per-frame allocation.** Contexts pre-allocate their buffers when created. Per-call work reuses existing capacity. This avoids unpredictable GC or heap pressure during real-time use.

3. **Errors should not crash the host.** Every exported function catches all C++ exceptions (OpenCV and otherwise) and returns an integer status code. A human-readable error message is always available via `OCP_<Module>_GetLastError`. The intent is that exceptions are always contained within the DLL.

4. **C# owns the data, C++ borrows it.** Point arrays, camera matrices, and output buffers are allocated in C# and passed as pinned pointers. The plugin reads/writes through those pointers during the call and never holds references afterward.

5. **Module isolation.** Each module lives in its own subfolder under `source/`, has its own context struct, and its own set of `OCP_<Module>_*` exports. The only shared code is the top-level header (`OpenCvPlugin.h`) which defines the export macro, status codes, and error-handling helpers.

---

## Prerequisites

### OpenCV

This plugin links against pre-built OpenCV binaries. You do **not** need to compile OpenCV from source.

1. Download the Windows pre-built package from https://opencv.org/releases/ — the current version used is **4.12.0**.
2. Extract it somewhere convenient (e.g. `C:\dev\libs\opencv\opencv-4.12.0`).
3. Open `CMakeLists.txt` and update the `OpenCV_DIR` path to point to the `lib` folder in your extraction — this is the folder containing `OpenCVConfig.cmake`:
   ```cmake
   set(OpenCV_DIR "C:/dev/libs/opencv/opencv-4.12.0/build/x64/vc16/lib")
   ```

To upgrade OpenCV: download and extract the new version, update `OpenCV_DIR` in `CMakeLists.txt`, then rebuild and re-package.

### Build Tools

- **CMake** 4.2 or later — https://cmake.org/download/
- **Visual Studio 2022** (or later) with the "Desktop development with C++" workload

---

## Building

Run `build_windows.bat`. This configures a Visual Studio solution via CMake and compiles Release and Debug configurations. Output DLLs end up in `build/Release/` and `build/Debug/`.

---

## Packaging for Unity Consumers

Run `build_package.bat`. This builds the native DLL and copies both `OpenCvPlugin.dll` and `opencv_world4120.dll` into `Package/Plugins/x86_64/`.

Unity consumers receive these DLLs through the package dependency — they do not need to copy anything manually. See [Package/README.md](Package/README.md) for consumer setup, including how to reference this package in a Unity project.

---

## C API Conventions

All exported functions follow these conventions:

- **Naming:** `OCP_<Module>_<Action>` (e.g. `OCP_PnP_Solve`)
- **Return value:** `int` status code — see `OpenCvPlugin.h` for `OCP_OK`, `OCP_ERROR_*` constants
- **Error messages:** after a non-OK return, call `OCP_<Module>_GetLastError(ctx)` to get a `const char*` describing what went wrong
- **Data ownership:** all input/output arrays are owned by the caller; the plugin never allocates memory the caller must free
- **Handle lifecycle:** `CreateContext` → use → `DestroyContext`; forgetting to destroy leaks memory

---

## Adding a New Module

1. Create a subfolder under `source/`, e.g. `source/alignment/`
2. Create `AlignmentContext.h` — define your context struct with a `char errorMsg[OCP_ERROR_MSG_SIZE]` field and any pre-allocated buffers
3. Create `AlignmentContext.cpp` — implement `OCP_Align_CreateContext`, `OCP_Align_DestroyContext`, your compute functions, and `OCP_Align_GetLastError`
4. Add both files to the `add_library()` call in `CMakeLists.txt`
5. Use the `OCP_TRY` / `OCP_CATCH` macros from `OpenCvPlugin.h` for error handling
6. On the C# side, create matching `AlignmentNative.cs` (DllImport declarations) and `AlignmentSolver.cs` (high-level wrapper)
