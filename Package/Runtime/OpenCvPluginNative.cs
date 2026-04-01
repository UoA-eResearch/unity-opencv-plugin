using System;
using System.Runtime.InteropServices;

namespace OpenCvPlugin
{
    /// <summary>
    /// Core plugin infrastructure: version checking and status codes.
    /// </summary>
    public static class OpenCvPluginNative
    {
        private const string DLL_NAME = "OpenCvPlugin";

        // Status codes matching C++ definitions in OpenCvPlugin.h
        public const int OCP_OK = 0;
        public const int OCP_ERROR_INVALID_ARGS = -1;
        public const int OCP_ERROR_OPENCV = -2;
        public const int OCP_ERROR_UNKNOWN = -3;
        public const int OCP_SOLVE_FAILED = 1;

        /// <summary>
        /// Get plugin version as integer (major*10000 + minor*100 + patch).
        /// Expected: 10000 for version 1.0.0
        /// </summary>
        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern int OCP_GetVersion();

        /// <summary>
        /// Verifies plugin version matches expected version.
        /// Call this on startup to catch DLL version mismatches.
        /// </summary>
        public static void VerifyVersion(int expectedVersion = 10000)
        {
            int actualVersion = OCP_GetVersion();
            if (actualVersion != expectedVersion)
            {
                throw new InvalidOperationException(
                    $"OpenCvPlugin version mismatch: expected {expectedVersion}, got {actualVersion}. " +
                    "Ensure the native plugin version matches the C# wrapper version.");
            }
        }

        /// <summary>
        /// Converts status code to human-readable string.
        /// </summary>
        public static string StatusToString(int status)
        {
            switch (status)
            {
                case OCP_OK: return "OK";
                case OCP_ERROR_INVALID_ARGS: return "Invalid arguments";
                case OCP_ERROR_OPENCV: return "OpenCV error";
                case OCP_ERROR_UNKNOWN: return "Unknown error";
                case OCP_SOLVE_FAILED: return "Solve failed (no solution found)";
                default: return $"Unknown status code: {status}";
            }
        }

        /// <summary>
        /// Checks if status code indicates an error (not OK or SOLVE_FAILED which is a soft fail).
        /// </summary>
        public static bool IsError(int status)
        {
            return status < 0;
        }

        /// <summary>
        /// Checks if status code indicates success (OCP_OK).
        /// </summary>
        public static bool IsOk(int status)
        {
            return status == OCP_OK;
        }
    }
}
