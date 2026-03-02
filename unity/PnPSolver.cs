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
    /// Manages context lifecycle, persistent pose state, and error handling.
    /// Thread-safe per instance (do not share across threads).
    /// </summary>
    public class PnPSolver : IDisposable
    {
        private IntPtr _context;
        private bool _disposed;

        // Pre-allocated buffers — avoids per-call GC allocations
        private static readonly float[] EmptyDistCoeffs = new float[1];
        private readonly int[] _inlierCountBuf = new int[1];

        // Persistent pose state for temporal coherence (double precision matches OpenCV's CV_64F)
        private readonly double[] _rvec = new double[3];  // Rotation vector (Rodrigues)
        private readonly double[] _tvec = new double[3];  // Translation vector

        // TODO: Remove when SolveRansac/ProjectPoints are refactored to use double[] rvec/tvec
        private readonly float[] _rvecFloat = new float[3];
        private readonly float[] _tvecFloat = new float[3];

        /// <summary>
        /// Last inlier count from SolveRansac().
        /// </summary>
        public int LastInlierCount { get; private set; }

        /// <summary>
        /// Current rotation vector (Rodrigues format, double precision).
        /// Updated by Solve/SolveRansac. Use GeometryUtils to convert to Unity Quaternion.
        /// </summary>
        public double[] RotationVector => _rvec;

        /// <summary>
        /// Current translation vector (double precision).
        /// Updated by Solve/SolveRansac. Use GeometryUtils to convert to Unity Vector3.
        /// </summary>
        public double[] TranslationVector => _tvec;

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
        /// Manually sets the current pose (rvec/tvec).
        /// Useful for providing initial guess for temporal coherence.
        /// </summary>
        public void SetPose(double[] rvec, double[] tvec)
        {
            if (rvec == null || rvec.Length != 3)
                throw new ArgumentException("rvec must be double[3]", nameof(rvec));
            if (tvec == null || tvec.Length != 3)
                throw new ArgumentException("tvec must be double[3]", nameof(tvec));

            Array.Copy(rvec, _rvec, 3);
            Array.Copy(tvec, _tvec, 3);
        }

        /// <summary>
        /// Resets the current pose to zero (no rotation, no translation).
        /// Call this when tracking is lost or starting a new sequence.
        /// </summary>
        public void ResetPose()
        {
            Array.Clear(_rvec, 0, 3);
            Array.Clear(_tvec, 0, 3);
        }

        /// <summary>
        /// Solves PnP problem using specified algorithm.
        /// Updates RotationVector and TranslationVector on success.
        /// </summary>
        /// <param name="objectPoints">3D points in object coordinates (x,y,z,x,y,z,...).
        /// For best performance, reuse the same array each frame rather than allocating new.</param>
        /// <param name="imagePoints">2D points in image coordinates (x,y,x,y,...). Must have same count as objectPoints.
        /// For best performance, reuse the same array each frame rather than allocating new.</param>
        /// <param name="cameraMatrix">Camera intrinsics as 3x3 row-major matrix: float[9] = [fx,0,cx, 0,fy,cy, 0,0,1]</param>
        /// <param name="distCoeffs">Distortion coefficients [k1,k2,p1,p2,k3] or null/empty for no distortion</param>
        /// <param name="method">Solver algorithm (default: SOLVEPNP_SQPNP - fast and accurate)</param>
        /// <param name="useExtrinsicGuess">If true, uses current RotationVector/TranslationVector as initial guess for temporal coherence</param>
        /// <returns>True on success, false if solve failed (check LastError for details)</returns>
        public bool Solve(
            float[] objectPoints,
            float[] imagePoints,
            float[] cameraMatrix,
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

            // Prepare distortion coefficients
            // Note: We must pass a valid array pointer even if distCoeffCount=0
            // because the P/Invoke marshaler with [In] attribute can't handle null.
            // Using a dummy single-element array when no distortion is specified.
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

            // Call native function — rvec/tvec written in-place (double precision, zero copy-back)
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
                _rvec,
                _tvec);

            // Handle result
            if (status == OpenCvPluginNative.OCP_OK)
                return true;

            if (status == OpenCvPluginNative.OCP_SOLVE_FAILED)
                return false;  // Soft failure - no solution found

            // Hard error
            throw new InvalidOperationException($"PnP Solve failed: {OpenCvPluginNative.StatusToString(status)} - {GetLastError()}");
        }

        /// <summary>
        /// Solves PnP with RANSAC outlier rejection.
        /// Updates RotationVector and TranslationVector on success.
        /// </summary>
        /// <param name="objectPoints">3D points in object coordinates (x,y,z,x,y,z,...).
        /// For best performance, reuse the same array each frame rather than allocating new.</param>
        /// <param name="imagePoints">2D points in image coordinates (x,y,x,y,...). Must have same count as objectPoints.
        /// For best performance, reuse the same array each frame rather than allocating new.</param>
        /// <param name="cameraMatrix">Camera intrinsics as 3x3 row-major matrix: float[9] = [fx,0,cx, 0,fy,cy, 0,0,1]</param>
        /// <param name="inlierIndices">Output buffer for inlier indices (caller allocates with length = point count). 
        /// Only first LastInlierCount elements are valid after call.</param>
        /// <param name="distCoeffs">Distortion coefficients [k1,k2,p1,p2,k3] or null/empty for no distortion</param>
        /// <param name="iterationsCount">RANSAC iterations (default: 100, range: 50-1000)</param>
        /// <param name="reprojectionError">RANSAC inlier threshold in pixels (default: 8.0, range: 1.0-10.0)</param>
        /// <param name="confidence">RANSAC confidence level (default: 0.99, range: 0.90-0.999)</param>
        /// <param name="method">Solver algorithm for RANSAC kernel (default: SOLVEPNP_EPNP)</param>
        /// <returns>True on success, false if solve failed (check LastError for details)</returns>
        public bool SolveRansac(
            float[] objectPoints,
            float[] imagePoints,
            float[] cameraMatrix,
            int[] inlierIndices,
            float[] distCoeffs = null,
            int iterationsCount = 100,
            float reprojectionError = 8.0f,
            double confidence = 0.99,
            int method = PnPNative.SOLVEPNP_EPNP)
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
            if (inlierIndices == null || inlierIndices.Length < objCount)
                throw new ArgumentException("inlierIndices must be int array with length >= point count", nameof(inlierIndices));

            // Prepare distortion coefficients
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

            // TODO: Remove float conversion when SolveRansac is refactored to use double[]
            _rvecFloat[0] = (float)_rvec[0]; _rvecFloat[1] = (float)_rvec[1]; _rvecFloat[2] = (float)_rvec[2];
            _tvecFloat[0] = (float)_tvec[0]; _tvecFloat[1] = (float)_tvec[1]; _tvecFloat[2] = (float)_tvec[2];

            // Call native function
            int status = PnPNative.OCP_PnP_SolveRansac(
                _context,
                objectPoints,
                imagePoints,
                objCount,
                cameraMatrix,
                distCoeffsToUse,
                distCoeffCount,
                method,
                0,  // useExtrinsicGuess = false for RANSAC
                iterationsCount,
                reprojectionError,
                confidence,
                _rvecFloat,
                _tvecFloat,
                inlierIndices,
                _inlierCountBuf,
                objCount);  // maxInliers

            // Copy results back to double precision
            _rvec[0] = _rvecFloat[0]; _rvec[1] = _rvecFloat[1]; _rvec[2] = _rvecFloat[2];
            _tvec[0] = _tvecFloat[0]; _tvec[1] = _tvecFloat[1]; _tvec[2] = _tvecFloat[2];
            LastInlierCount = _inlierCountBuf[0];

            // Handle result
            if (status == OpenCvPluginNative.OCP_OK)
                return true;

            if (status == OpenCvPluginNative.OCP_SOLVE_FAILED)
                return false;  // Soft failure - no solution found

            // Hard error
            throw new InvalidOperationException($"PnP SolveRansac failed: {OpenCvPluginNative.StatusToString(status)} - {GetLastError()}");
        }

        /// <summary>
        /// Projects 3D points to 2D using current pose (RotationVector/TranslationVector).
        /// Useful for manual reprojection error calculation or debugging.
        /// </summary>
        /// <param name="objectPoints">3D points in object coordinates (x,y,z,x,y,z,...)</param>
        /// <param name="cameraMatrix">Camera intrinsics as 3x3 row-major matrix: float[9] = [fx,0,cx, 0,fy,cy, 0,0,1]</param>
        /// <param name="imagePointsOut">Output 2D points (caller allocates with length = objectPoints.Length * 2 / 3)</param>
        /// <param name="distCoeffs">Distortion coefficients [k1,k2,p1,p2,k3] or null/empty for no distortion</param>
        public void ProjectPoints(
            float[] objectPoints,
            float[] cameraMatrix,
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
            if (imagePointsOut == null || imagePointsOut.Length < count * 2)
                throw new ArgumentException("imagePointsOut must have length >= objectPoints.Length * 2 / 3", nameof(imagePointsOut));

            // Prepare distortion coefficients
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

            // TODO: Remove float conversion when ProjectPoints is refactored to use double[]
            _rvecFloat[0] = (float)_rvec[0]; _rvecFloat[1] = (float)_rvec[1]; _rvecFloat[2] = (float)_rvec[2];
            _tvecFloat[0] = (float)_tvec[0]; _tvecFloat[1] = (float)_tvec[1]; _tvecFloat[2] = (float)_tvec[2];

            // Call native function
            int status = PnPNative.OCP_PnP_ProjectPoints(
                _context,
                objectPoints,
                count,
                _rvecFloat,
                _tvecFloat,
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
