#pragma once

// ============================================================================
// PnP Module — Context definition
//
// The context holds only pre-allocated working memory (scratch buffers).
// It does NOT retain any pose state between calls — the caller owns rvec/tvec
// and passes them in/out each call.
// ============================================================================

#include "OpenCvPlugin.h"

#include <opencv2/core.hpp>

#include <array>
#include <vector>

struct PnPContextInternal {
    // Pre-allocated scratch buffers for converting flat float arrays into
    // OpenCV point vectors. Reserved to maxPoints at creation time.
    std::vector<cv::Point3f> objPts;
    std::vector<cv::Point2f> imgPts;

    // Scratch buffer for projected points (used by ProjectPoints)
    std::vector<cv::Point2f> projPts;

    // Upper bound on point count, set at creation time.
    int maxPoints;

    // Per-context error message buffer, read via OCP_PnP_GetLastError.
    std::array<char, OCP_ERROR_MSG_SIZE> errorMsg;
};