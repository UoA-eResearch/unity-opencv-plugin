#pragma once

// ============================================================================
// OpenCvPlugin — Shared header
//
// Defines the export macro, status codes, handle types, and the error-handling
// wrapper macro used by all modules.
// ============================================================================

#include <cstring>   // strncpy
#include <exception>

// --- Export macro ------------------------------------------------------------
// Marks functions as exported from the shared library (DLL on Windows).

#if defined(_WIN32)
#  define OCP_EXPORT __declspec(dllexport)
#else
#  define OCP_EXPORT __attribute__((visibility("default")))
#endif

// --- Status codes -----------------------------------------------------------
// Returned by every exported function. Zero means success.
// Negative values are errors. Positive values are "soft" failures (e.g. the
// solver ran but couldn't find a solution).

#define OCP_OK                  0   // Success
#define OCP_SOLVE_FAILED        1   // OpenCV solver returned false / no solution found
#define OCP_ERROR_INVALID_ARGS -1   // Null pointer, bad count, unsupported parameter, etc.
#define OCP_ERROR_OPENCV       -2   // An OpenCV cv::Exception was caught
#define OCP_ERROR_UNKNOWN      -3   // A non-OpenCV C++ exception was caught

// --- Handle types -----------------------------------------------------------
// Opaque pointers passed between C# and C++. Each module defines its own.

typedef void* OcpPnPContext;

// --- Error-handling wrapper -------------------------------------------------
// Every exported function should use this macro around its body. It catches
// all C++ exceptions, writes a human-readable message into the context's
// errorMsg buffer, and returns the appropriate status code.
//
// Usage:
//
//   OCP_TRY(ctx)
//   {
//       // ... your code here ...
//       return OCP_OK;
//   }
//   OCP_CATCH(ctx)
//
// Requirements: `ctx` must be a pointer to a struct that has a
// `std::array<char, OCP_ERROR_MSG_SIZE>` member.

constexpr int OCP_ERROR_MSG_SIZE = 512;

// Helper: safely copy a message into a fixed-size error buffer.
inline void ocp_SetError(char* errorMsg, const char* msg) {
#if defined(_MSC_VER)
    strncpy_s(errorMsg, OCP_ERROR_MSG_SIZE, msg, _TRUNCATE);
#else
    std::strncpy(errorMsg, msg, OCP_ERROR_MSG_SIZE - 1);
    errorMsg[OCP_ERROR_MSG_SIZE - 1] = '\0';
#endif
}

// The try/catch wrapper, split into two macros so your code sits between them.
// We include opencv2/core.hpp here to catch cv::Exception; this is the only
// OpenCV dependency in this header.
#include <opencv2/core.hpp>

#define OCP_TRY(ctx)                                                          \
    try                                                                       \

// (your code block with { ... return OCP_OK; } goes here)

#define OCP_CATCH(ctx)                                                        \
    catch (const cv::Exception& e) {                                          \
        ocp_SetError((ctx)->errorMsg.data(), e.what());                       \
        return OCP_ERROR_OPENCV;                                              \
    }                                                                         \
    catch (const std::exception& e) {                                         \
        ocp_SetError((ctx)->errorMsg.data(), e.what());                       \
        return OCP_ERROR_UNKNOWN;                                             \
    }                                                                         \
    catch (...) {                                                              \
        ocp_SetError((ctx)->errorMsg.data(), "Unknown non-C++ exception");    \
        return OCP_ERROR_UNKNOWN;                                             \
    }