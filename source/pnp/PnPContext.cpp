// ============================================================================
// PnP Module — Exported functions
// ============================================================================

#include "PnPContext.h"

#include <opencv2/calib3d.hpp>

#include <cstdio>   // snprintf

// ---------------------------------------------------------------------------
// Helpers (internal, not exported)
// ---------------------------------------------------------------------------

// Copy flat float arrays into the context's pre-allocated OpenCV vectors.
// objPts: [x0,y0,z0, x1,y1,z1, ...]   imgPts: [u0,v0, u1,v1, ...]
static void copyPointsIn(PnPContextInternal* ctx,
                          const float* objectPoints,
                          const float* imagePoints,
                          int count)
{
    ctx->objPts.resize(count);
    ctx->imgPts.resize(count);
    for (int i = 0; i < count; i++) {
        ctx->objPts[i] = cv::Point3f(objectPoints[i * 3],
                                      objectPoints[i * 3 + 1],
                                      objectPoints[i * 3 + 2]);
        ctx->imgPts[i] = cv::Point2f(imagePoints[i * 2],
                                      imagePoints[i * 2 + 1]);
    }
}

// Validate common arguments shared by Solve and SolveRansac.
// Returns OCP_OK if valid, or an error code (and fills errorMsg).
static int validateCommonArgs(PnPContextInternal* ctx,
                               const float* objectPoints,
                               const float* imagePoints,
                               int count,
                               const float* cameraMatrix,
                               int distCoeffCount,
                               const float* rvecInOut,
                               const float* tvecInOut)
{
    if (!objectPoints || !imagePoints || !cameraMatrix || !rvecInOut || !tvecInOut) {
        ocp_SetError(ctx->errorMsg, "Null pointer passed for a required argument");
        return OCP_ERROR_INVALID_ARGS;
    }
    if (count <= 0) {
        ocp_SetError(ctx->errorMsg, "Point count must be > 0");
        return OCP_ERROR_INVALID_ARGS;
    }
    if (count > ctx->maxPoints) {
        snprintf(ctx->errorMsg, OCP_ERROR_MSG_SIZE,
                 "Point count %d exceeds context maximum %d", count, ctx->maxPoints);
        return OCP_ERROR_INVALID_ARGS;
    }
    if (distCoeffCount != 0 && distCoeffCount != 5) {
        ocp_SetError(ctx->errorMsg, "distCoeffCount must be 0 or 5");
        return OCP_ERROR_INVALID_ARGS;
    }
    return OCP_OK;
}

// ---------------------------------------------------------------------------
// Exported functions
// ---------------------------------------------------------------------------

extern "C" {

// -- Lifecycle ---------------------------------------------------------------

OCP_EXPORT OcpPnPContext OCP_PnP_CreateContext(int maxPoints)
{
    if (maxPoints <= 0) return nullptr;

    auto* ctx = new (std::nothrow) PnPContextInternal();
    if (!ctx) return nullptr;

    ctx->maxPoints = maxPoints;
    ctx->objPts.reserve(maxPoints);
    ctx->imgPts.reserve(maxPoints);
    ctx->projPts.reserve(maxPoints);
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
    return ctx->errorMsg;
}

// -- SolvePnP (wraps cv::solvePnPGeneric) ------------------------------------

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
    float* rvecInOut,               // In/Out: 3-float rotation vector (Rodrigues)
    float* tvecInOut,               // In/Out: 3-float translation vector
    float* reprojErrorOut           // Out: RMS reprojection error (may be null)
)
{
    if (!handle) return OCP_ERROR_INVALID_ARGS;
    auto* ctx = static_cast<PnPContextInternal*>(handle);
    ctx->errorMsg[0] = '\0';

    // Validate inputs
    int v = validateCommonArgs(ctx, objectPoints, imagePoints, count,
                                cameraMatrix, distCoeffCount, rvecInOut, tvecInOut);
    if (v != OCP_OK) return v;

    OCP_TRY(ctx)
    {
        // Copy points into pre-allocated vectors
        copyPointsIn(ctx, objectPoints, imagePoints, count);

        // Wrap caller's camera matrix and distortion as cv::Mat (no copy)
        cv::Mat camMat(3, 3, CV_32F, const_cast<float*>(cameraMatrix));
        cv::Mat dist;
        if (distCoeffCount > 0 && distCoeffs) {
            dist = cv::Mat(1, distCoeffCount, CV_32F, const_cast<float*>(distCoeffs));
        }

        // Prepare initial guess if requested
        // solvePnPGeneric expects double Mats for rvec/tvec guess
        cv::Mat rvecGuess, tvecGuess;
        if (useExtrinsicGuess) {
            rvecGuess = (cv::Mat_<double>(3, 1) << rvecInOut[0], rvecInOut[1], rvecInOut[2]);
            tvecGuess = (cv::Mat_<double>(3, 1) << tvecInOut[0], tvecInOut[1], tvecInOut[2]);
        }

        // Call solvePnPGeneric — returns all solutions with reprojection errors
        std::vector<cv::Mat> rvecs, tvecs;
        cv::Mat reprojErrors;

        int numSolutions = cv::solvePnPGeneric(
            ctx->objPts, ctx->imgPts, camMat, dist,
            rvecs, tvecs,
            useExtrinsicGuess != 0,
            static_cast<cv::SolvePnPMethod>(method),
            rvecGuess, tvecGuess,
            reprojErrors
        );

        if (numSolutions <= 0) {
            ocp_SetError(ctx->errorMsg, "solvePnPGeneric returned no solutions");
            return OCP_SOLVE_FAILED;
        }

        // Take the first (best) solution
        const cv::Mat& rvec = rvecs[0];
        const cv::Mat& tvec = tvecs[0];

        // Copy results back to caller's buffers (OpenCV PnP output is CV_64F)
        rvecInOut[0] = static_cast<float>(rvec.at<double>(0));
        rvecInOut[1] = static_cast<float>(rvec.at<double>(1));
        rvecInOut[2] = static_cast<float>(rvec.at<double>(2));

        tvecInOut[0] = static_cast<float>(tvec.at<double>(0));
        tvecInOut[1] = static_cast<float>(tvec.at<double>(1));
        tvecInOut[2] = static_cast<float>(tvec.at<double>(2));

        // Reprojection error (per-solution RMS)
        if (reprojErrorOut) {
            *reprojErrorOut = static_cast<float>(reprojErrors.at<double>(0));
        }

        return OCP_OK;
    }
    OCP_CATCH(ctx)
}

// -- SolvePnPRansac (wraps cv::solvePnPRansac) --------------------------------

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
        copyPointsIn(ctx, objectPoints, imagePoints, count);

        cv::Mat camMat(3, 3, CV_32F, const_cast<float*>(cameraMatrix));
        cv::Mat dist;
        if (distCoeffCount > 0 && distCoeffs) {
            dist = cv::Mat(1, distCoeffCount, CV_32F, const_cast<float*>(distCoeffs));
        }

        cv::Mat rvec, tvec;
        if (useExtrinsicGuess) {
            rvec = (cv::Mat_<double>(3, 1) << rvecInOut[0], rvecInOut[1], rvecInOut[2]);
            tvec = (cv::Mat_<double>(3, 1) << tvecInOut[0], tvecInOut[1], tvecInOut[2]);
        }

        cv::Mat inliersMat;

        bool ok = cv::solvePnPRansac(
            ctx->objPts, ctx->imgPts, camMat, dist,
            rvec, tvec,
            useExtrinsicGuess != 0,
            iterationsCount,
            reprojThreshold,
            confidence,
            inliersMat,
            static_cast<cv::SolvePnPMethod>(method)
        );

        if (!ok) {
            ocp_SetError(ctx->errorMsg, "solvePnPRansac returned false (no solution)");
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
        ocp_SetError(ctx->errorMsg, "Null pointer passed for a required argument");
        return OCP_ERROR_INVALID_ARGS;
    }
    if (count <= 0 || count > ctx->maxPoints) {
        snprintf(ctx->errorMsg, OCP_ERROR_MSG_SIZE,
                 "Point count %d out of valid range [1, %d]", count, ctx->maxPoints);
        return OCP_ERROR_INVALID_ARGS;
    }
    if (distCoeffCount != 0 && distCoeffCount != 5) {
        ocp_SetError(ctx->errorMsg, "distCoeffCount must be 0 or 5");
        return OCP_ERROR_INVALID_ARGS;
    }

    OCP_TRY(ctx)
    {
        // Build object points vector
        ctx->objPts.resize(count);
        for (int i = 0; i < count; i++) {
            ctx->objPts[i] = cv::Point3f(objectPoints[i * 3],
                                          objectPoints[i * 3 + 1],
                                          objectPoints[i * 3 + 2]);
        }

        cv::Mat rvecMat = (cv::Mat_<double>(3, 1) << rvec[0], rvec[1], rvec[2]);
        cv::Mat tvecMat = (cv::Mat_<double>(3, 1) << tvec[0], tvec[1], tvec[2]);
        cv::Mat camMat(3, 3, CV_32F, const_cast<float*>(cameraMatrix));
        cv::Mat dist;
        if (distCoeffCount > 0 && distCoeffs) {
            dist = cv::Mat(1, distCoeffCount, CV_32F, const_cast<float*>(distCoeffs));
        }

        ctx->projPts.resize(count);
        cv::projectPoints(ctx->objPts, rvecMat, tvecMat, camMat, dist, ctx->projPts);

        // Copy results to caller's flat array
        for (int i = 0; i < count; i++) {
            projectedPointsOut[i * 2]     = ctx->projPts[i].x;
            projectedPointsOut[i * 2 + 1] = ctx->projPts[i].y;
        }

        return OCP_OK;
    }
    OCP_CATCH(ctx)
}

} // extern "C"