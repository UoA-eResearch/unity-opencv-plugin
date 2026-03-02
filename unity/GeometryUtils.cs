using UnityEngine;

namespace OpenCvPlugin
{
    /// <summary>
    /// Utility functions for OpenCV-Unity integration:
    /// - Mathematical representation conversions (Rodrigues ↔ Quaternion)
    /// - Coordinate system transformations (OpenCV ↔ Unity)
    /// - Camera intrinsic helpers
    /// 
    /// <para><b>Coordinate System Conventions:</b></para>
    /// <list type="bullet">
    /// <item>OpenCV: Right-handed, Y-down, Z-forward (camera looks down +Z with +Y pointing down)</item>
    /// <item>Unity: Left-handed, Y-up, Z-forward (camera looks down +Z with +Y pointing up)</item>
    /// </list>
    /// 
    /// <para>Transformation between systems requires flipping Y positions and negating X/Z quaternion components.</para>
    /// </summary>
    public static class GeometryUtils
    {
        // ========================================
        // Mathematical Conversions
        // ========================================

        /// <summary>
        /// Converts Rodrigues rotation vector (angle-axis) to Quaternion.
        /// </summary>
        /// <param name="rvec">Rotation vector (Rodrigues format) [rx, ry, rz]</param>
        /// <returns>Quaternion representation of the rotation</returns>
        public static Quaternion RodriguesToQuaternion(float[] rvec)
        {
            if (rvec == null || rvec.Length != 3)
            {
                Debug.LogError("rvec must be float[3]");
                return Quaternion.identity;
            }

            return RodriguesToQuaternion(rvec[0], rvec[1], rvec[2]);
        }

        /// <summary>
        /// Converts Rodrigues rotation vector (angle-axis) to Quaternion.
        /// Accepts double[] to match OpenCV's native CV_64F precision.
        /// </summary>
        /// <param name="rvec">Rotation vector (Rodrigues format) [rx, ry, rz] as double[3]</param>
        /// <returns>Quaternion representation of the rotation</returns>
        public static Quaternion RodriguesToQuaternion(double[] rvec)
        {
            if (rvec == null || rvec.Length != 3)
            {
                Debug.LogError("rvec must be double[3]");
                return Quaternion.identity;
            }

            return RodriguesToQuaternion((float)rvec[0], (float)rvec[1], (float)rvec[2]);
        }

        private static Quaternion RodriguesToQuaternion(float rx, float ry, float rz)
        {
            // Calculate rotation angle (magnitude of rotation vector)
            float angle = Mathf.Sqrt(rx * rx + ry * ry + rz * rz);
            
            if (angle < 0.0001f)
                return Quaternion.identity;

            // Normalize axis
            Vector3 axis = new Vector3(rx / angle, ry / angle, rz / angle);

            // Convert to quaternion (angle is in radians, convert to degrees for Unity)
            return Quaternion.AngleAxis(angle * Mathf.Rad2Deg, axis);
        }

        /// <summary>
        /// Converts Quaternion to Rodrigues rotation vector (angle-axis).
        /// </summary>
        /// <param name="q">Quaternion to convert</param>
        /// <returns>Rodrigues vector [rx, ry, rz]</returns>
        public static float[] QuaternionToRodrigues(Quaternion q)
        {
            // Convert quaternion to angle-axis
            q.ToAngleAxis(out float angleDeg, out Vector3 axis);
            float angleRad = angleDeg * Mathf.Deg2Rad;

            // Rodrigues vector = axis * angle
            return new float[] { axis.x * angleRad, axis.y * angleRad, axis.z * angleRad };
        }

        // ========================================
        // Coordinate System Transformations
        // ========================================

        /// <summary>
        /// Converts rotation from OpenCV coordinate system to Unity coordinate system.
        /// </summary>
        /// <param name="opencvRotation">Rotation in OpenCV coordinates</param>
        /// <returns>Rotation in Unity coordinates</returns>
        public static Quaternion ConvertRotationOpenCVToUnity(Quaternion opencvRotation)
        {
            // Flip X and Z components to transform between coordinate systems
            return new Quaternion(-opencvRotation.x, opencvRotation.y, -opencvRotation.z, opencvRotation.w);
        }

        /// <summary>
        /// Converts rotation from Unity coordinate system to OpenCV coordinate system.
        /// </summary>
        /// <param name="unityRotation">Rotation in Unity coordinates</param>
        /// <returns>Rotation in OpenCV coordinates</returns>
        public static Quaternion ConvertRotationUnityToOpenCV(Quaternion unityRotation)
        {
            // Flip X and Z components to transform between coordinate systems
            return new Quaternion(-unityRotation.x, unityRotation.y, -unityRotation.z, unityRotation.w);
        }

        /// <summary>
        /// Converts position from OpenCV coordinate system to Unity coordinate system.
        /// </summary>
        /// <param name="opencvPosition">Position in OpenCV coordinates</param>
        /// <returns>Position in Unity coordinates</returns>
        public static Vector3 ConvertPositionOpenCVToUnity(Vector3 opencvPosition)
        {
            // Flip Y axis
            return new Vector3(opencvPosition.x, -opencvPosition.y, opencvPosition.z);
        }

        /// <summary>
        /// Converts position from Unity coordinate system to OpenCV coordinate system.
        /// </summary>
        /// <param name="unityPosition">Position in Unity coordinates</param>
        /// <returns>Position in OpenCV coordinates</returns>
        public static Vector3 ConvertPositionUnityToOpenCV(Vector3 unityPosition)
        {
            // Flip Y axis
            return new Vector3(unityPosition.x, -unityPosition.y, unityPosition.z);
        }

        // ========================================
        // Convenience Methods
        // ========================================

        /// <summary>
        /// Converts OpenCV Rodrigues rotation vector directly to Unity Quaternion.
        /// Performs both representation conversion and coordinate system transformation.
        /// </summary>
        /// <param name="rvec">OpenCV rotation vector (Rodrigues format) [rx, ry, rz]</param>
        /// <returns>Unity quaternion</returns>
        public static Quaternion RodriguesToUnityQuaternion(float[] rvec)
        {
            Quaternion opencvQuat = RodriguesToQuaternion(rvec);
            return ConvertRotationOpenCVToUnity(opencvQuat);
        }

        /// <summary>
        /// Converts OpenCV Rodrigues rotation vector directly to Unity Quaternion.
        /// Accepts double[] to match OpenCV's native CV_64F precision.
        /// </summary>
        /// <param name="rvec">OpenCV rotation vector (Rodrigues format) [rx, ry, rz] as double[3]</param>
        /// <returns>Unity quaternion</returns>
        public static Quaternion RodriguesToUnityQuaternion(double[] rvec)
        {
            Quaternion opencvQuat = RodriguesToQuaternion(rvec);
            return ConvertRotationOpenCVToUnity(opencvQuat);
        }

        /// <summary>
        /// Converts OpenCV translation vector directly to Unity position.
        /// Performs both array conversion and coordinate system transformation.
        /// </summary>
        /// <param name="tvec">OpenCV translation vector [tx, ty, tz]</param>
        /// <returns>Unity position vector</returns>
        public static Vector3 TranslationToUnityPosition(float[] tvec)
        {
            if (tvec == null || tvec.Length != 3)
            {
                Debug.LogError("tvec must be float[3]");
                return Vector3.zero;
            }

            Vector3 opencvPos = new Vector3(tvec[0], tvec[1], tvec[2]);
            return ConvertPositionOpenCVToUnity(opencvPos);
        }

        /// <summary>
        /// Converts OpenCV translation vector directly to Unity position.
        /// Accepts double[] to match OpenCV's native CV_64F precision.
        /// </summary>
        /// <param name="tvec">OpenCV translation vector [tx, ty, tz] as double[3]</param>
        /// <returns>Unity position vector</returns>
        public static Vector3 TranslationToUnityPosition(double[] tvec)
        {
            if (tvec == null || tvec.Length != 3)
            {
                Debug.LogError("tvec must be double[3]");
                return Vector3.zero;
            }

            Vector3 opencvPos = new Vector3((float)tvec[0], (float)tvec[1], (float)tvec[2]);
            return ConvertPositionOpenCVToUnity(opencvPos);
        }

        /// <summary>
        /// Applies OpenCV pose (rvec + tvec) to a Unity Transform.
        /// </summary>
        /// <param name="transform">Unity transform to update</param>
        /// <param name="rvec">OpenCV rotation vector [rx, ry, rz]</param>
        /// <param name="tvec">OpenCV translation vector [tx, ty, tz]</param>
        public static void ApplyPoseToTransform(Transform transform, float[] rvec, float[] tvec)
        {
            if (transform == null)
            {
                Debug.LogError("Transform is null");
                return;
            }

            transform.rotation = RodriguesToUnityQuaternion(rvec);
            transform.position = TranslationToUnityPosition(tvec);
        }

        /// <summary>
        /// Applies OpenCV pose (rvec + tvec) to a Unity Transform.
        /// Accepts double[] to match OpenCV's native CV_64F precision.
        /// </summary>
        /// <param name="transform">Unity transform to update</param>
        /// <param name="rvec">OpenCV rotation vector [rx, ry, rz] as double[3]</param>
        /// <param name="tvec">OpenCV translation vector [tx, ty, tz] as double[3]</param>
        public static void ApplyPoseToTransform(Transform transform, double[] rvec, double[] tvec)
        {
            if (transform == null)
            {
                Debug.LogError("Transform is null");
                return;
            }

            transform.rotation = RodriguesToUnityQuaternion(rvec);
            transform.position = TranslationToUnityPosition(tvec);
        }

        // ========================================
        // Camera Intrinsic Helpers
        // ========================================

        /// <summary>
        /// Computes OpenCV camera intrinsic matrix from Unity Camera.
        /// Returns 3x3 matrix in row-major format as float[9] = [fx,0,cx, 0,fy,cy, 0,0,1]
        /// Note: Assumes no lens distortion - for real AR/webcam applications,
        /// perform proper camera calibration and pass distortion coefficients separately.
        /// </summary>
        /// <param name="camera">Unity camera to extract intrinsics from</param>
        /// <returns>Camera matrix as float[9] in row-major order</returns>
        public static float[] GetCameraMatrix(Camera camera)
        {
            if (camera == null)
            {
                Debug.LogError("Camera is null");
                return new float[] { 800f, 0f, 640f, 0f, 800f, 360f, 0f, 0f, 1f }; // Fallback defaults
            }

            // Compute focal length from field of view
            // For vertical FOV: fy = (height/2) / tan(vFOV/2)
            float fovRad = camera.fieldOfView * Mathf.Deg2Rad;
            float focalLength = (camera.pixelHeight / 2.0f) / Mathf.Tan(fovRad / 2.0f);

            float fx = focalLength;
            float fy = focalLength;
            float cx = camera.pixelWidth / 2.0f;
            float cy = camera.pixelHeight / 2.0f;

            return new float[]
            {
                fx,  0f,  cx,
                0f,  fy,  cy,
                0f,  0f,  1f
            };
        }

        /// <summary>
        /// Computes OpenCV camera intrinsic matrix from explicit parameters.
        /// Useful when you know exact camera properties or want to override Unity camera settings.
        /// </summary>
        /// <param name="focalLengthPixels">Focal length in pixels (same for X and Y if square pixels)</param>
        /// <param name="imageWidth">Image width in pixels</param>
        /// <param name="imageHeight">Image height in pixels</param>
        /// <returns>Camera matrix as float[9] = [fx,0,cx, 0,fy,cy, 0,0,1]</returns>
        public static float[] GetCameraMatrix(float focalLengthPixels, int imageWidth, int imageHeight)
        {
            return new float[]
            {
                focalLengthPixels,  0f,                 imageWidth / 2.0f,
                0f,                 focalLengthPixels,  imageHeight / 2.0f,
                0f,                 0f,                 1f
            };
        }

        /// <summary>
        /// Computes OpenCV camera intrinsic matrix from separate focal lengths.
        /// Use this when pixels are not square or when you have calibrated fx != fy.
        /// </summary>
        /// <param name="fx">Focal length X in pixels</param>
        /// <param name="fy">Focal length Y in pixels</param>
        /// <param name="imageWidth">Image width in pixels</param>
        /// <param name="imageHeight">Image height in pixels</param>
        /// <returns>Camera matrix as float[9] = [fx,0,cx, 0,fy,cy, 0,0,1]</returns>
        public static float[] GetCameraMatrix(float fx, float fy, int imageWidth, int imageHeight)
        {
            return new float[]
            {
                fx,  0f,  imageWidth / 2.0f,
                0f,  fy,  imageHeight / 2.0f,
                0f,  0f,  1f
            };
        }

        /// <summary>
        /// Validates that a camera matrix has reasonable values.
        /// Useful for debugging camera calibration issues.
        /// </summary>
        /// <param name="cameraMatrix">Camera matrix as float[9] = [fx,0,cx, 0,fy,cy, 0,0,1]</param>
        /// <returns>True if matrix appears valid</returns>
        public static bool ValidateCameraMatrix(float[] cameraMatrix)
        {
            if (cameraMatrix == null || cameraMatrix.Length != 9)
            {
                Debug.LogError("Camera matrix must be float[9]");
                return false;
            }

            float fx = cameraMatrix[0];
            float fy = cameraMatrix[4];
            float cx = cameraMatrix[2];
            float cy = cameraMatrix[5];

            // Check for reasonable focal lengths (should be positive and typically > 100 pixels)
            if (fx <= 0 || fy <= 0)
            {
                Debug.LogError($"Invalid focal lengths: fx={fx}, fy={fy} (must be > 0)");
                return false;
            }

            if (fx < 50 || fy < 50)
            {
                Debug.LogWarning($"Unusually small focal lengths: fx={fx}, fy={fy} (typically > 100px)");
            }

            // Check for reasonable principal points (should be positive)
            if (cx <= 0 || cy <= 0)
            {
                Debug.LogError($"Invalid principal point: cx={cx}, cy={cy} (must be > 0)");
                return false;
            }

            // Check that the matrix has the expected structure (bottom row should be [0,0,1])
            if (cameraMatrix[6] != 0f || cameraMatrix[7] != 0f || cameraMatrix[8] != 1f)
            {
                Debug.LogWarning($"Unexpected values in bottom row: [{cameraMatrix[6]}, {cameraMatrix[7]}, {cameraMatrix[8]}], expected [0, 0, 1]");
            }

            return true;
        }
    }
}
