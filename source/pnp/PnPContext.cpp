// ============================================================================
// PnP Module — Exported functions
// ============================================================================

#include "PnPContext.h"

#include <opencv2/calib3d.hpp>

#include <vector>   // std::vector (used locally by ProjectPoints)

// ---------------------------------------------------------------------------
// Helpers (internal, not exported)
// ---------------------------------------------------------------------------

// Validate common arguments for Solve.
// Returns OCP_OK if valid, or an error code (and fills errorMsg).
static int validateCommonArgs(PnPContextInternal* ctx,
                               const float* objectPoints,
                               const float* imagePoints,
                               int count,
                               const float* cameraMatrix,
                               int distCoeffCount,
                               const void* rvecInOut,
                               const void* tvecInOut)
{
    if (!objectPoints || !imagePoints || !cameraMatrix || !rvecInOut || !tvecInOut) {
        ocp_SetError(ctx->errorMsg.data(), "Null pointer passed for a required argument");
        return OCP_ERROR_INVALID_ARGS;
    }
    if (count <= 0) {
        ocp_SetError(ctx->errorMsg.data(), "Point count must be > 0");
        return OCP_ERROR_INVALID_ARGS;
    }
    if (distCoeffCount != 0 && distCoeffCount != 5) {
        ocp_SetError(ctx->errorMsg.data(), "distCoeffCount must be 0 or 5");
        return OCP_ERROR_INVALID_ARGS;
    }
    return OCP_OK;
}

// ---------------------------------------------------------------------------
// Exported functions
// ---------------------------------------------------------------------------

extern "C" {

// -- Lifecycle ---------------------------------------------------------------

OCP_EXPORT OcpPnPContext OCP_PnP_CreateContext()
{
    auto* ctx = new (std::nothrow) PnPContextInternal();
    if (!ctx) return nullptr;

    ctx->errorMsg[0] = '\0';

    return static_cast<OcpPnPContext>(ctx);
}

OCP_EXPORT void OCP_PnP_DestroyContext(OcpPnPContext handle)
{
    if (!handle) return;
    delete static_cast<PnPContextInternal*>(handle);
}

// -- Error query -------------------------------------------------------------

OCP_EXPORT const char* OCP_PnP_GetLastError(OcpPnPContext handle)
{
    if (!handle) return "Null context handle";
    auto* ctx = static_cast<PnPContextInternal*>(handle);
    return ctx->errorMsg.data();
}

// -- SolvePnP (wraps cv::solvePnP) -------------------------------------------

OCP_EXPORT int OCP_PnP_Solve(
    OcpPnPContext handle,
    const float* objectPoints,      // 3D points: [x,y,z, x,y,z, ...] (count * 3 floats)
    const float* imagePoints,       // 2D points: [u,v, u,v, ...]      (count * 2 floats)
    int count,                      // Number of point correspondences
    const float* cameraMatrix,      // 3x3 intrinsic matrix, row-major (9 floats)
    const float* distCoeffs,        // Distortion coefficients (may be null if distCoeffCount == 0)
    int distCoeffCount,             // 0 or 5
    int method,                     // cv::SolvePnPMethod enum value (0=ITERATIVE, 1=EPNP, 8=SQPNP)
    int useExtrinsicGuess,          // If non-zero, rvecInOut/tvecInOut are read as initial guess
    double* rvecInOut,              // In/Out: 3-double rotation vector (Rodrigues)
    double* tvecInOut               // In/Out: 3-double translation vector
)
{
    if (!handle) return OCP_ERROR_INVALID_ARGS;
    auto* ctx = static_cast<PnPContextInternal*>(handle);
    ctx->errorMsg[0] = '\0';

    int v = validateCommonArgs(ctx, objectPoints, imagePoints, count,
                                cameraMatrix, distCoeffCount, rvecInOut, tvecInOut);
    if (v != OCP_OK) return v;

    OCP_TRY(ctx)
    {
        // Zero-copy: wrap caller's pinned managed arrays as cv::Mat headers.
        // matReadOnly isolates the const_cast needed by cv::Mat's constructor.
        cv::Mat objMat  = matReadOnly(count, 1, CV_32FC3, objectPoints);
        cv::Mat imgMat  = matReadOnly(count, 1, CV_32FC2, imagePoints);
        cv::Mat camMat  = matReadOnly(3, 3, CV_32F, cameraMatrix);

        cv::Mat dist;
        if (distCoeffCount > 0 && distCoeffs) {
            dist = matReadOnly(1, distCoeffCount, CV_32F, distCoeffs);
        }

        // Wrap caller's double* buffers directly. cv::solvePnP writes results
        // in-place via OutputArray (no reallocation when shape/type match CV_64F).
        cv::Mat rvec(3, 1, CV_64F, rvecInOut);
        cv::Mat tvec(3, 1, CV_64F, tvecInOut);

        bool ok = cv::solvePnP(
            objMat, imgMat, camMat, dist,
            rvec, tvec,
            useExtrinsicGuess != 0,
            static_cast<cv::SolvePnPMethod>(method)
        );

        if (!ok) {
            ocp_SetError(ctx->errorMsg.data(), "solvePnP returned false (no solution)");
            return OCP_SOLVE_FAILED;
        }

        // Results already written to caller's rvecInOut/tvecInOut buffers.
        return OCP_OK;
    }
    OCP_CATCH(ctx)
}

// -- ProjectPoints (wraps cv::projectPoints) ---------------------------------

OCP_EXPORT int OCP_PnP_ProjectPoints(
    OcpPnPContext handle,
    const float* objectPoints,      // 3D points (count * 3 floats)
    int count,                      // Number of points
    const double* rvec,             // 3-double rotation vector (Rodrigues)
    const double* tvec,             // 3-double translation vector
    const float* cameraMatrix,      // 3x3 intrinsic matrix, row-major (9 floats)
    const float* distCoeffs,        // Distortion coefficients (may be null if distCoeffCount == 0)
    int distCoeffCount,             // 0 or 5
    float* projectedPointsOut       // Out: projected 2D points [u,v, u,v, ...] (count * 2 floats)
)
{
    if (!handle) return OCP_ERROR_INVALID_ARGS;
    auto* ctx = static_cast<PnPContextInternal*>(handle);
    ctx->errorMsg[0] = '\0';

    if (!objectPoints || !rvec || !tvec || !cameraMatrix || !projectedPointsOut) {
        ocp_SetError(ctx->errorMsg.data(), "Null pointer passed for a required argument");
        return OCP_ERROR_INVALID_ARGS;
    }
    if (count <= 0) {
        ocp_SetError(ctx->errorMsg.data(), "Point count must be > 0");
        return OCP_ERROR_INVALID_ARGS;
    }
    if (distCoeffCount != 0 && distCoeffCount != 5) {
        ocp_SetError(ctx->errorMsg.data(), "distCoeffCount must be 0 or 5");
        return OCP_ERROR_INVALID_ARGS;
    }

    OCP_TRY(ctx)
    {
        // Zero-copy: wrap caller's arrays as cv::Mat headers.
        // matReadOnly isolates the const_cast needed by cv::Mat's constructor.
        cv::Mat objMat  = matReadOnly(count, 1, CV_32FC3, objectPoints);
        cv::Mat rvecMat = matReadOnly(3, 1, CV_64F, rvec);
        cv::Mat tvecMat = matReadOnly(3, 1, CV_64F, tvec);
        cv::Mat camMat  = matReadOnly(3, 3, CV_32F, cameraMatrix);
        cv::Mat dist;
        if (distCoeffCount > 0 && distCoeffs) {
            dist = matReadOnly(1, distCoeffCount, CV_32F, distCoeffs);
        }

        std::vector<cv::Point2f> projPts;
        cv::projectPoints(objMat, rvecMat, tvecMat, camMat, dist, projPts);

        // Copy results to caller's flat array
        for (int i = 0; i < count; i++) {
            projectedPointsOut[i * 2]     = projPts[i].x;
            projectedPointsOut[i * 2 + 1] = projPts[i].y;
        }

        return OCP_OK;
    }
    OCP_CATCH(ctx)
}

} // extern "C"