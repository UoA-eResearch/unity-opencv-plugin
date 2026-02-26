using System;
using System.Runtime.InteropServices;

namespace OpenCvPlugin.PnP
{
    /// <summary>
    /// Low-level P/Invoke declarations for OpenCV PnP (Pose-n-Perspective) functions.
    /// All functions use opaque context handles and C-compatible signatures for stability.
    /// </summary>
    public static class PnPNative
    {
        private const string DLL_NAME = "OpenCvPlugin";

        // PnP solver flags matching OpenCV's cv::SolvePnPMethod
        public const int SOLVEPNP_ITERATIVE = 0;
        public const int SOLVEPNP_EPNP = 1;
        public const int SOLVEPNP_P3P = 2;
        public const int SOLVEPNP_DLS = 3;
        public const int SOLVEPNP_UPNP = 4;
        public const int SOLVEPNP_AP3P = 5;
        public const int SOLVEPNP_IPPE = 6;
        public const int SOLVEPNP_IPPE_SQUARE = 7;
        public const int SOLVEPNP_SQPNP = 8;

        /// <summary>
        /// Creates a PnP context with pre-allocated buffers for up to maxPoints points.
        /// This eliminates per-frame allocations - all math happens in pre-allocated memory.
        /// </summary>
        /// <param name="maxPoints">Maximum number of points the context will handle (4-1000 recommended)</param>
        /// <returns>Opaque context handle. Caller must call OCP_PnP_DestroyContext when done.</returns>
        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr OCP_PnP_CreateContext(int maxPoints);

        /// <summary>
        /// Destroys a PnP context and frees all associated memory.
        /// Safe to call with null/zero handle.
        /// </summary>
        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern void OCP_PnP_DestroyContext(IntPtr handle);

        /// <summary>
        /// Gets the last error message from the context.
        /// Returns pointer to context's internal error buffer (max 512 chars).
        /// Valid until next call on this context or context destruction.
        /// </summary>
        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr OCP_PnP_GetLastError(IntPtr handle);

        /// <summary>
        /// Solves Pose-n-Perspective problem using cv::solvePnPGeneric.
        /// Supports multiple solver algorithms and temporal coherence via extrinsic guess.
        /// </summary>
        /// <param name="handle">PnP context handle from OCP_PnP_CreateContext</param>
        /// <param name="objectPoints">3D points in object coordinates (x,y,z flat array, length = count*3)</param>
        /// <param name="imagePoints">2D points in image coordinates (x,y flat array, length = count*2)</param>
        /// <param name="count">Number of point correspondences (4-maxPoints)</param>
        /// <param name="cameraMatrix">Camera intrinsic matrix as 3x3 row-major array (9 floats): [fx,0,cx, 0,fy,cy, 0,0,1]</param>
        /// <param name="distCoeffs">Distortion coefficients [k1,k2,p1,p2,k3] or null for no distortion. Length must be 0 or 5.</param>
        /// <param name="distCoeffCount">Number of distortion coefficients (0 or 5)</param>
        /// <param name="method">Solver algorithm (SOLVEPNP_SQPNP, SOLVEPNP_ITERATIVE, etc.)</param>
        /// <param name="useExtrinsicGuess">If non-zero, uses rvecInOut/tvecInOut as initial guess (enables temporal coherence)</param>
        /// <param name="rvecInOut">Input/output rotation vector [rx,ry,rz] (Rodrigues). Use prior frame's rvec for temporal coherence.</param>
        /// <param name="tvecInOut">Input/output translation vector [tx,ty,tz]. Use prior frame's tvec for temporal coherence.</param>
        /// <param name="reprojErrorOut">Output array [1 element] for mean reprojection error in pixels</param>
        /// <returns>Status code: OCP_OK (0) on success, OCP_SOLVE_FAILED (1) if no solution, negative on error</returns>
        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern int OCP_PnP_Solve(
            IntPtr handle,
            [In] float[] objectPoints,
            [In] float[] imagePoints,
            int count,
            [In] float[] cameraMatrix,
            [In] float[] distCoeffs,
            int distCoeffCount,
            int method,
            int useExtrinsicGuess,
            [In, Out] float[] rvecInOut,
            [In, Out] float[] tvecInOut,
            [Out] float[] reprojErrorOut);

        /// <summary>
        /// Solves PnP with RANSAC outlier rejection using cv::solvePnPRansac.
        /// Returns inlier indices and inlier count for diagnostic purposes.
        /// </summary>
        /// <param name="handle">PnP context handle from OCP_PnP_CreateContext</param>
        /// <param name="objectPoints">3D points in object coordinates (x,y,z flat array, length = count*3)</param>
        /// <param name="imagePoints">2D points in image coordinates (x,y flat array, length = count*2)</param>
        /// <param name="count">Number of point correspondences (4-maxPoints)</param>
        /// <param name="cameraMatrix">Camera intrinsic matrix as 3x3 row-major array (9 floats): [fx,0,cx, 0,fy,cy, 0,0,1]</param>
        /// <param name="distCoeffs">Distortion coefficients [k1,k2,p1,p2,k3] or null for no distortion. Length must be 0 or 5.</param>
        /// <param name="distCoeffCount">Number of distortion coefficients (0 or 5)</param>
        /// <param name="method">Solver algorithm for RANSAC kernel (SOLVEPNP_EPNP, SOLVEPNP_ITERATIVE, etc.)</param>
        /// <param name="useExtrinsicGuess">If non-zero, uses rvecInOut/tvecInOut as initial guess</param>
        /// <param name="iterationsCount">RANSAC iterations (100-1000 recommended)</param>
        /// <param name="reprojThreshold">RANSAC inlier threshold in pixels (1.0-8.0 recommended)</param>
        /// <param name="confidence">RANSAC confidence level (0.90-0.999 recommended)</param>
        /// <param name="rvecInOut">Input/output rotation vector [rx,ry,rz] (Rodrigues)</param>
        /// <param name="tvecInOut">Input/output translation vector [tx,ty,tz]</param>
        /// <param name="inliersOut">Output buffer for inlier indices (caller-allocated, length = maxInliers)</param>
        /// <param name="inlierCountOut">Output array [1 element] for number of inliers found</param>
        /// <param name="maxInliers">Maximum capacity of inliersOut array (typically = point count)</param>
        /// <returns>Status code: OCP_OK (0) on success, OCP_SOLVE_FAILED (1) if no solution, negative on error</returns>
        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern int OCP_PnP_SolveRansac(
            IntPtr handle,
            [In] float[] objectPoints,
            [In] float[] imagePoints,
            int count,
            [In] float[] cameraMatrix,
            [In] float[] distCoeffs,
            int distCoeffCount,
            int method,
            int useExtrinsicGuess,
            int iterationsCount,
            float reprojThreshold,
            double confidence,
            [In, Out] float[] rvecInOut,
            [In, Out] float[] tvecInOut,
            [Out] int[] inliersOut,
            [Out] int[] inlierCountOut,
            int maxInliers);

        /// <summary>
        /// Projects 3D points to 2D using cv::projectPoints.
        /// Useful for manual reprojection error calculation or visualization.
        /// </summary>
        /// <param name="handle">PnP context handle from OCP_PnP_CreateContext</param>
        /// <param name="objectPoints">3D points in object coordinates (x,y,z flat array, length = count*3)</param>
        /// <param name="count">Number of points to project (1-maxPoints)</param>
        /// <param name="rvec">Rotation vector [rx,ry,rz] (Rodrigues)</param>
        /// <param name="tvec">Translation vector [tx,ty,tz]</param>
        /// <param name="cameraMatrix">Camera intrinsic matrix as 3x3 row-major array (9 floats): [fx,0,cx, 0,fy,cy, 0,0,1]</param>
        /// <param name="distCoeffs">Distortion coefficients [k1,k2,p1,p2,k3] or null for no distortion. Length must be 0 or 5.</param>
        /// <param name="distCoeffCount">Number of distortion coefficients (0 or 5)</param>
        /// <param name="imagePointsOut">Output 2D projected points (x,y flat array, caller-allocated, length = count*2)</param>
        /// <returns>Status code: OCP_OK (0) on success, negative on error</returns>
        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern int OCP_PnP_ProjectPoints(
            IntPtr handle,
            [In] float[] objectPoints,
            int count,
            [In] float[] rvec,
            [In] float[] tvec,
            [In] float[] cameraMatrix,
            [In] float[] distCoeffs,
            int distCoeffCount,
            [Out] float[] imagePointsOut);
    }
}
