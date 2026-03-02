#pragma once

// ============================================================================
// PnP Module — Context definition and helpers
//
// The context holds only the per-instance error message buffer.
// It does NOT retain any solver state between calls — the caller owns rvec/tvec
// and passes them in/out each call.
// ============================================================================

#include "OpenCvPlugin.h"

#include <opencv2/core.hpp>

#include <array>

struct PnPContextInternal {
    // Per-context error message buffer, read via OCP_PnP_GetLastError.
    std::array<char, OCP_ERROR_MSG_SIZE> errorMsg;
};

// Wrap a read-only raw pointer as a cv::Mat header (zero-copy, zero-alloc).
// OpenCV's cv::Mat requires non-const void* even for read-only use (InputArray).
// This is safe: OpenCV's InputArray contract guarantees read-only access.
// UB would only occur if the data were modified, which does not happen.
inline cv::Mat matReadOnly(int rows, int cols, int type, const void* data) {
    return cv::Mat(rows, cols, type, const_cast<void*>(data));
}