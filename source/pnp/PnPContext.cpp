// ============================================================================
// PnP Module — Exported functions
// ============================================================================

#include "PnPContext.h"

#include <opencv2/calib3d.hpp>

#include <vector>   // std::vector (used locally by ProjectPoints, SolveRansac)

// ---------------------------------------------------------------------------
// Helpers (internal, not exported)
// ---------------------------------------------------------------------------

// Validate common arguments shared by Solve, SolveRansac, etc.
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

// -- SolvePnPRansac (wraps cv::solvePnPRansac) --------------------------------
// TODO: Refactor to match Solve pattern (zero-copy double* rvec/tvec)

OCP_EXPORT int OCP_PnP_SolveRansac(
    OcpPnPContext handle,
    const float* objectPoints,      // 3D points (count * 3 floats)
    const float* imagePoints,       // 2D points (count * 2 floats)
    int count,                      // Number of point correspondences
    const float* cameraMatrix,      // 3x3 intrinsic matrix, row-major (9 floats)
    const float* distCoeffs,        // Distortion coefficients (may be null if distCoeffCount == 0)
    int distCoeffCount,             // 0 or 5
    int method,                     // cv::SolvePnPMethod for the inner solver
    int useExtrinsicGuess,          // If non-zero, rvecInOut/tvecInOut are read as initial guess
    int iterationsCount,            // RANSAC iterations (e.g. 100)
    float reprojThreshold,          // Inlier threshold in pixels (e.g. 8.0)
    double confidence,              // Confidence level (e.g. 0.99)
    float* rvecInOut,               // In/Out: 3-float rotation vector
    float* tvecInOut,               // In/Out: 3-float translation vector
    int* inliersOut,                // Out: array to receive inlier indices (may be null)
    int* inlierCountOut,            // Out: number of inliers written (may be null)
    int maxInliers                  // Max capacity of inliersOut array
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
        // Zero-copy: wrap caller's arrays as cv::Mat headers
        cv::Mat objMat  = matReadOnly(count, 1, CV_32FC3, objectPoints);
        cv::Mat imgMat  = matReadOnly(count, 1, CV_32FC2, imagePoints);
        cv::Mat camMat  = matReadOnly(3, 3, CV_32F, cameraMatrix);
        cv::Mat dist;
        if (distCoeffCount > 0 && distCoeffs) {
            dist = matReadOnly(1, distCoeffCount, CV_32F, distCoeffs);
        }

        cv::Mat rvec, tvec;
        if (useExtrinsicGuess) {
            rvec = (cv::Mat_<double>(3, 1) << rvecInOut[0], rvecInOut[1], rvecInOut[2]);
            tvec = (cv::Mat_<double>(3, 1) << tvecInOut[0], tvecInOut[1], tvecInOut[2]);
        }

        cv::Mat inliersMat;

        bool ok = cv::solvePnPRansac(
            objMat, imgMat, camMat, dist,
            rvec, tvec,
            useExtrinsicGuess != 0,
            iterationsCount,
            reprojThreshold,
            confidence,
            inliersMat,
            static_cast<cv::SolvePnPMethod>(method)
        );

        if (!ok) {
            ocp_SetError(ctx->errorMsg.data(), "solvePnPRansac returned false (no solution)");
            return OCP_SOLVE_FAILED;
        }

        // Copy pose results back
        rvecInOut[0] = static_cast<float>(rvec.at<double>(0));
        rvecInOut[1] = static_cast<float>(rvec.at<double>(1));
        rvecInOut[2] = static_cast<float>(rvec.at<double>(2));

        tvecInOut[0] = static_cast<float>(tvec.at<double>(0));
        tvecInOut[1] = static_cast<float>(tvec.at<double>(1));
        tvecInOut[2] = static_cast<float>(tvec.at<double>(2));

        // Copy inlier indices to caller's buffer
        if (inlierCountOut) {
            int numInliers = inliersMat.rows;
            *inlierCountOut = numInliers;

            if (inliersOut && maxInliers > 0) {
                int toCopy = (numInliers < maxInliers) ? numInliers : maxInliers;
                for (int i = 0; i < toCopy; i++) {
                    inliersOut[i] = inliersMat.at<int>(i);
                }
            }
        }

        return OCP_OK;
    }
    OCP_CATCH(ctx)
}

// -- ProjectPoints (wraps cv::projectPoints) ---------------------------------
// TODO: Refactor to match Solve pattern (zero-copy double* rvec/tvec)

OCP_EXPORT int OCP_PnP_ProjectPoints(
    OcpPnPContext handle,
    const float* objectPoints,      // 3D points (count * 3 floats)
    int count,                      // Number of points
    const float* rvec,              // 3-float rotation vector
    const float* tvec,              // 3-float translation vector
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
        // Zero-copy: wrap caller's arrays as cv::Mat headers
        cv::Mat objMat  = matReadOnly(count, 1, CV_32FC3, objectPoints);
        cv::Mat rvecMat = (cv::Mat_<double>(3, 1) << rvec[0], rvec[1], rvec[2]);
        cv::Mat tvecMat = (cv::Mat_<double>(3, 1) << tvec[0], tvec[1], tvec[2]);
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