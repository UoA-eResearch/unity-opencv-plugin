using System;
using System.Runtime.InteropServices;

namespace OpenCvPlugin.PnP
{
    /// <summary>
    /// PnP solver algorithms. Maps to OpenCV's cv::SolvePnPMethod.
    /// </summary>
    public enum PnPMethod
    {
        ITERATIVE = 0,      // Levenberg-Marquardt optimization (needs initial guess for best results)
        EPNP = 1,           // Efficient PnP (fast, no initial guess needed)
        P3P = 2,            // Perspective-3-Point (requires exactly 3 points)
        DLS = 3,            // Direct Least Squares
        UPNP = 4,           // Unified PnP
        AP3P = 5,           // Alternative P3P
        IPPE = 6,           // Infinitesimal Plane-based Pose Estimation
        IPPE_SQUARE = 7,    // IPPE for square markers
        SQPNP = 8           // SQPnP (fast and accurate, recommended default)
    }

    /// <summary>
    /// High-level managed wrapper for OpenCV PnP solving.
    /// Manages context lifecycle and provides GC-friendly solve calls via caller-owned buffers.
    /// Results (rvec/tvec) are written directly into caller-supplied double[3] arrays.
    /// Thread-safe per instance (do not share across threads).
    /// </summary>
    public class PnPSolver : IDisposable
    {
        private IntPtr _context;
        private bool _disposed;

        // Pre-allocated dummy buffer — avoids per-call GC allocation when distortion is unused.
        // Passed as a valid pointer with distCoeffCount=0 so the native side gets a non-null pointer.
        private static readonly float[] EmptyDistCoeffs = new float[1];

        /// <summary>
        /// Creates a PnP solver context.
        /// </summary>
        public PnPSolver()
        {
            _context = PnPNative.OCP_PnP_CreateContext();

            if (_context == IntPtr.Zero)
                throw new InvalidOperationException("Failed to create PnP context. Ensure OpenCvPlugin.dll is deployed.");
        }

        /// <summary>
        /// Solves PnP problem using specified algorithm.
        /// Results are written directly into the caller-supplied rvecInOut and tvecInOut buffers.
        /// For temporal coherence, pass the same double[3] arrays each call and set useExtrinsicGuess=true.
        /// </summary>
        /// <param name="objectPoints">3D points in object coordinates (x,y,z,x,y,z,...).
        /// For best performance, reuse the same array each frame rather than allocating new.</param>
        /// <param name="imagePoints">2D points in image coordinates (x,y,x,y,...). Must have same count as objectPoints.
        /// For best performance, reuse the same array each frame rather than allocating new.</param>
        /// <param name="cameraMatrix">Camera intrinsics as 3x3 row-major matrix: float[9] = [fx,0,cx, 0,fy,cy, 0,0,1]</param>
        /// <param name="rvecInOut">Caller-owned double[3] rotation vector (Rodrigues). Written on success.
        /// If useExtrinsicGuess=true, its current value is used as the initial guess.</param>
        /// <param name="tvecInOut">Caller-owned double[3] translation vector. Written on success.
        /// If useExtrinsicGuess=true, its current value is used as the initial guess.</param>
        /// <param name="distCoeffs">Distortion coefficients [k1,k2,p1,p2,k3] or null/empty for no distortion</param>
        /// <param name="method">Solver algorithm (default: SOLVEPNP_SQPNP - fast and accurate)</param>
        /// <param name="useExtrinsicGuess">If true, rvecInOut/tvecInOut are read as initial guess (temporal coherence)</param>
        /// <returns>True on success, false if solve failed (check GetLastError() for details)</returns>
        public bool Solve(
            float[] objectPoints,
            float[] imagePoints,
            float[] cameraMatrix,
            double[] rvecInOut,
            double[] tvecInOut,
            float[] distCoeffs = null,
            int method = PnPNative.SOLVEPNP_SQPNP,
            bool useExtrinsicGuess = false)
        {
            ThrowIfDisposed();

            // Validate inputs
            if (objectPoints == null || objectPoints.Length < 12)
                throw new ArgumentException("objectPoints must have at least 4 points (12 floats)", nameof(objectPoints));
            if (imagePoints == null || imagePoints.Length < 8)
                throw new ArgumentException("imagePoints must have at least 4 points (8 floats)", nameof(imagePoints));
            if (objectPoints.Length % 3 != 0)
                throw new ArgumentException("objectPoints length must be multiple of 3", nameof(objectPoints));
            if (imagePoints.Length % 2 != 0)
                throw new ArgumentException("imagePoints length must be multiple of 2", nameof(imagePoints));

            int objCount = objectPoints.Length / 3;
            int imgCount = imagePoints.Length / 2;
            if (objCount != imgCount)
                throw new ArgumentException("objectPoints and imagePoints must have same number of points");

            if (cameraMatrix == null || cameraMatrix.Length != 9)
                throw new ArgumentException("cameraMatrix must be float[9] = [fx,0,cx, 0,fy,cy, 0,0,1]", nameof(cameraMatrix));
            if (rvecInOut == null || rvecInOut.Length != 3)
                throw new ArgumentException("rvecInOut must be double[3]", nameof(rvecInOut));
            if (tvecInOut == null || tvecInOut.Length != 3)
                throw new ArgumentException("tvecInOut must be double[3]", nameof(tvecInOut));

            // Prepare distortion coefficients.
            // Note: We must pass a valid array pointer even if distCoeffCount=0
            // because the P/Invoke marshaler with [In] attribute can't handle null.
            // Using a pre-allocated dummy array when no distortion is specified.
            float[] distCoeffsToUse;
            int distCoeffCount;
            if (distCoeffs == null || distCoeffs.Length == 0)
            {
                distCoeffsToUse = EmptyDistCoeffs;
                distCoeffCount = 0;
            }
            else
            {
                distCoeffsToUse = distCoeffs;
                distCoeffCount = distCoeffs.Length;
            }

            if (distCoeffCount != 0 && distCoeffCount != 5)
                throw new ArgumentException("distCoeffs must be null, empty, or length 5", nameof(distCoeffs));

            // Call native function — results written directly into caller's rvecInOut/tvecInOut buffers
            int status = PnPNative.OCP_PnP_Solve(
                _context,
                objectPoints,
                imagePoints,
                objCount,
                cameraMatrix,
                distCoeffsToUse,
                distCoeffCount,
                method,
                useExtrinsicGuess ? 1 : 0,
                rvecInOut,
                tvecInOut);

            // Handle result
            if (status == OpenCvPluginNative.OCP_OK)
                return true;

            if (status == OpenCvPluginNative.OCP_SOLVE_FAILED)
                return false;  // Soft failure - no solution found

            // Hard error
            throw new InvalidOperationException($"PnP Solve failed: {OpenCvPluginNative.StatusToString(status)} - {GetLastError()}");
        }

        /// <summary>
        /// Projects 3D points to 2D using cv::projectPoints.
        /// Useful for manual reprojection error calculation or debugging.
        /// </summary>
        /// <param name="objectPoints">3D points in object coordinates (x,y,z,x,y,z,...)</param>
        /// <param name="cameraMatrix">Camera intrinsics as 3x3 row-major matrix: float[9] = [fx,0,cx, 0,fy,cy, 0,0,1]</param>
        /// <param name="rvec">Rotation vector double[3] (Rodrigues), e.g. from a previous Solve() call.</param>
        /// <param name="tvec">Translation vector double[3], e.g. from a previous Solve() call.</param>
        /// <param name="imagePointsOut">Output 2D points (caller allocates with length = objectPoints.Length * 2 / 3)</param>
        /// <param name="distCoeffs">Distortion coefficients [k1,k2,p1,p2,k3] or null/empty for no distortion</param>
        public void ProjectPoints(
            float[] objectPoints,
            float[] cameraMatrix,
            double[] rvec,
            double[] tvec,
            float[] imagePointsOut,
            float[] distCoeffs = null)
        {
            ThrowIfDisposed();

            // Validate inputs
            if (objectPoints == null || objectPoints.Length < 3)
                throw new ArgumentException("objectPoints must have at least 1 point (3 floats)", nameof(objectPoints));
            if (objectPoints.Length % 3 != 0)
                throw new ArgumentException("objectPoints length must be multiple of 3", nameof(objectPoints));

            int count = objectPoints.Length / 3;

            if (cameraMatrix == null || cameraMatrix.Length != 9)
                throw new ArgumentException("cameraMatrix must be float[9] = [fx,0,cx, 0,fy,cy, 0,0,1]", nameof(cameraMatrix));
            if (rvec == null || rvec.Length != 3)
                throw new ArgumentException("rvec must be double[3]", nameof(rvec));
            if (tvec == null || tvec.Length != 3)
                throw new ArgumentException("tvec must be double[3]", nameof(tvec));
            if (imagePointsOut == null || imagePointsOut.Length < count * 2)
                throw new ArgumentException("imagePointsOut must have length >= objectPoints.Length * 2 / 3", nameof(imagePointsOut));

            // Prepare distortion coefficients.
            // Note: We must pass a valid array pointer even if distCoeffCount=0
            // because the P/Invoke marshaler with [In] attribute can't handle null.
            float[] distCoeffsToUse;
            int distCoeffCount;
            if (distCoeffs == null || distCoeffs.Length == 0)
            {
                distCoeffsToUse = EmptyDistCoeffs;
                distCoeffCount = 0;
            }
            else
            {
                distCoeffsToUse = distCoeffs;
                distCoeffCount = distCoeffs.Length;
            }

            if (distCoeffCount != 0 && distCoeffCount != 5)
                throw new ArgumentException("distCoeffs must be null, empty, or length 5", nameof(distCoeffs));

            // Call native function — rvec/tvec passed directly (double precision, zero copy)
            int status = PnPNative.OCP_PnP_ProjectPoints(
                _context,
                objectPoints,
                count,
                rvec,
                tvec,
                cameraMatrix,
                distCoeffsToUse,
                distCoeffCount,
                imagePointsOut);

            if (status != OpenCvPluginNative.OCP_OK)
                throw new InvalidOperationException($"ProjectPoints failed: {OpenCvPluginNative.StatusToString(status)} - {GetLastError()}");
        }

        /// <summary>
        /// Gets the last error message from the native context.
        /// </summary>
        public string GetLastError()
        {
            if (_context == IntPtr.Zero)
                return "Context not initialized";

            IntPtr errorPtr = PnPNative.OCP_PnP_GetLastError(_context);
            if (errorPtr == IntPtr.Zero)
                return "Unknown error";

            return Marshal.PtrToStringAnsi(errorPtr) ?? "Failed to retrieve error message";
        }

        private void ThrowIfDisposed()
        {
            if (_disposed)
                throw new ObjectDisposedException(nameof(PnPSolver));
        }

        /// <summary>
        /// Releases native resources.
        /// </summary>
        public void Dispose()
        {
            if (!_disposed)
            {
                if (_context != IntPtr.Zero)
                {
                    PnPNative.OCP_PnP_DestroyContext(_context);
                    _context = IntPtr.Zero;
                }
                _disposed = true;
            }
            GC.SuppressFinalize(this);
        }

        ~PnPSolver()
        {
            Dispose();
        }
    }
}
