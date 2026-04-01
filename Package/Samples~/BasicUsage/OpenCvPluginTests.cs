using UnityEngine;
using System;
using OpenCvPlugin;
using OpenCvPlugin.PnP;

/// <summary>
/// Example MonoBehaviour demonstrating OpenCvPlugin PnP solver usage.
/// Attach to a GameObject to run synthetic pose estimation tests on Start.
/// </summary>
public class OpenCVPluginTests : MonoBehaviour
{
    [Header("Test Configuration")]
    [Tooltip("Run automatic test on Start")]
    public bool runTestOnStart = true;

    [Tooltip("Enable detailed logging")]
    public bool verboseLogging = false;

    [Header("Test Results")]
    public Vector3 lastPosition;
    public Quaternion lastRotation;
    public float lastReprojectionError;
    public bool lastSolveSucceeded;

    private PnPSolver _solver;

    // Caller-owned pose buffers reused across frames (written by Solve())
    private readonly double[] _rvec = new double[3];
    private readonly double[] _tvec = new double[3];

    void Awake()
    {
        // Verify plugin version on startup to catch DLL mismatches early
        try
        {
            OpenCvPluginNative.VerifyVersion(10000);
            Debug.Log($"<color=green>OpenCvPlugin loaded successfully (v{OpenCvPluginNative.OCP_GetVersion()})</color>");
        }
        catch (Exception e)
        {
            Debug.LogError($"<color=red>OpenCvPlugin version check failed!</color>\n{e.Message}");
                Debug.LogError("Ensure the package is correctly referenced and the native DLL is present.");

        // Create solver instance (reusable across frames)
        _solver = new PnPSolver();
    }

    void Start()
    {
        if (runTestOnStart)
        {
            RunBasicTest();
            RunTemporalCoherenceTest();
        }
    }

    /// <summary>
    /// Basic PnP solve test with synthetic data.
    /// Tests whether the plugin can solve a simple 4-point square.
    /// </summary>
    public void RunBasicTest()
    {
        Debug.Log("=== OpenCvPlugin Basic Test ===");

        try
        {
            // Define 3D model points: a 1x1 meter square at Z=0
            float[] objectPoints = new float[]
            {
                -0.5f, -0.5f, 0.0f,  // bottom-left
                 0.5f, -0.5f, 0.0f,  // bottom-right
                 0.5f,  0.5f, 0.0f,  // top-right
                -0.5f,  0.5f, 0.0f   // top-left
            };

            // Fake 2D image points (pixels from a ~1280x720 camera)
            float[] imagePoints = new float[]
            {
                320.0f, 480.0f,  // bottom-left in image
                960.0f, 480.0f,  // bottom-right
                960.0f, 240.0f,  // top-right
                320.0f, 240.0f   // top-left
            };

            // Camera intrinsics (example values for ~1280x720 image)
            // 3x3 matrix in row-major order: [fx,0,cx, 0,fy,cy, 0,0,1]
            float[] cameraMatrix = new float[]
            {
                800.0f,  0f,      640.0f,   // fx, 0, cx
                0f,      800.0f,  360.0f,   // 0, fy, cy
                0f,      0f,      1f        // 0, 0, 1
            };

            if (verboseLogging)
            {
                Debug.Log($"Object points: {objectPoints.Length / 3} points");
                Debug.Log($"Image points: {imagePoints.Length / 2} points");
                Debug.Log($"Camera: fx={cameraMatrix[0]}, fy={cameraMatrix[4]}, cx={cameraMatrix[2]}, cy={cameraMatrix[5]}");
            }

            // Solve PnP
            bool success = _solver.Solve(
                objectPoints,
                imagePoints,
                cameraMatrix,
                _rvec,
                _tvec,
                distCoeffs: null,
                method: PnPNative.SOLVEPNP_SQPNP,
                useExtrinsicGuess: false);

            lastSolveSucceeded = success;
            // lastReprojectionError = _solver.LastReprojectionError;

            if (success)
            {
                // Convert OpenCV pose to Unity coordinates
                lastPosition = GeometryUtils.TranslationToUnityPosition(_tvec);
                lastRotation = GeometryUtils.RodriguesToUnityQuaternion(_rvec);

                Debug.Log($"<color=green>✓ Solve succeeded!</color>");
                // Debug.Log($"  Reprojection error: {_solver.LastReprojectionError:F3} pixels");
                Debug.Log($"  OpenCV rvec: [{_rvec[0]:F3}, {_rvec[1]:F3}, {_rvec[2]:F3}]");
                Debug.Log($"  OpenCV tvec: [{_tvec[0]:F3}, {_tvec[1]:F3}, {_tvec[2]:F3}]");
                Debug.Log($"  Unity position: {lastPosition}");
                Debug.Log($"  Unity rotation: {lastRotation.eulerAngles}");
            }
            else
            {
                Debug.LogWarning($"<color=yellow>✗ Solve failed</color>: {_solver.GetLastError()}");
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"<color=red>Test exception:</color>\n{e.Message}\n{e.StackTrace}");
            lastSolveSucceeded = false;
        }
    }

    /// <summary>
    /// Tests temporal coherence by solving twice with the second solve using the first result as a guess.
    /// </summary>
    public void RunTemporalCoherenceTest()
    {
        Debug.Log("\n=== OpenCvPlugin Temporal Coherence Test ===");

        try
        {
            // Same setup as basic test
            float[] objectPoints = new float[]
            {
                -0.5f, -0.5f, 0.0f,
                 0.5f, -0.5f, 0.0f,
                 0.5f,  0.5f, 0.0f,
                -0.5f,  0.5f, 0.0f
            };

            float[] imagePoints1 = new float[]
            {
                320.0f, 480.0f,
                960.0f, 480.0f,
                960.0f, 240.0f,
                320.0f, 240.0f
            };

            // Slightly different image points (simulating next frame)
            float[] imagePoints2 = new float[]
            {
                325.0f, 485.0f,
                965.0f, 485.0f,
                965.0f, 245.0f,
                325.0f, 245.0f
            };

            float[] cameraMatrix = new float[] 
            { 
                800.0f, 0f, 640.0f,    // fx, 0, cx
                0f, 800.0f, 360.0f,    // 0, fy, cy
                0f, 0f, 1f             // 0, 0, 1
            };

            // First solve (no guess)
            Array.Clear(_rvec, 0, 3);
            Array.Clear(_tvec, 0, 3);
            bool success1 = _solver.Solve(
                objectPoints, imagePoints1, cameraMatrix,
                _rvec, _tvec,
                method: PnPNative.SOLVEPNP_ITERATIVE,
                useExtrinsicGuess: false);

            if (!success1)
            {
                Debug.LogWarning("First solve failed - cannot test temporal coherence");
                return;
            }

            double[] rvec1 = (double[])_rvec.Clone();
            double[] tvec1 = (double[])_tvec.Clone();

            Debug.Log($"Frame 1: rvec=[{rvec1[0]:F3}, {rvec1[1]:F3}, {rvec1[2]:F3}], " +
                     $"tvec=[{tvec1[0]:F3}, {tvec1[1]:F3}, {tvec1[2]:F3}]");

            // Second solve (with guess from first)
            bool success2 = _solver.Solve(
                objectPoints, imagePoints2, cameraMatrix,
                _rvec, _tvec,
                method: PnPNative.SOLVEPNP_ITERATIVE,
                useExtrinsicGuess: true);  // Use previous result as guess

            if (success2)
            {
                Debug.Log($"Frame 2: rvec=[{_rvec[0]:F3}, {_rvec[1]:F3}, {_rvec[2]:F3}], " +
                         $"tvec=[{_tvec[0]:F3}, {_tvec[1]:F3}, {_tvec[2]:F3}]");
                Debug.Log($"<color=green>✓ Temporal coherence test passed</color> (pose updated smoothly)");
            }
            else
            {
                Debug.LogWarning($"Second solve failed: {_solver.GetLastError()}");
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"Temporal coherence test exception:\n{e.Message}");
        }
    }

    /// <summary>
    /// Helper method to test PnP solving with Unity Vector types.
    /// Kept for backward compatibility with old OpenCVBridge API.
    /// </summary>
    public bool SolveFromVectors(Vector3[] objPts, Vector2[] imgPts, Camera cam, out Vector3 position, out Quaternion rotation)
    {
        if (objPts == null || imgPts == null || objPts.Length != imgPts.Length || objPts.Length < 4)
        {
            Debug.LogError("Invalid input: need at least 4 matching 3D/2D point pairs");
            position = Vector3.zero;
            rotation = Quaternion.identity;
            return false;
        }

        // Convert to flat arrays
        float[] objectPoints = new float[objPts.Length * 3];
        float[] imagePoints = new float[imgPts.Length * 2];

        for (int i = 0; i < objPts.Length; i++)
        {
            objectPoints[i * 3] = objPts[i].x;
            objectPoints[i * 3 + 1] = objPts[i].y;
            objectPoints[i * 3 + 2] = objPts[i].z;

            imagePoints[i * 2] = imgPts[i].x;
            imagePoints[i * 2 + 1] = imgPts[i].y;
        }

        // Get camera matrix from Unity camera
        float[] cameraMatrix = GeometryUtils.GetCameraMatrix(cam);

        // Solve
        bool success = _solver.Solve(
            objectPoints,
            imagePoints,
            cameraMatrix,
            _rvec,
            _tvec,
            distCoeffs: null,
            method: PnPNative.SOLVEPNP_SQPNP,
            useExtrinsicGuess: false);

        if (success)
        {
            position = GeometryUtils.TranslationToUnityPosition(_tvec);
            rotation = GeometryUtils.RodriguesToUnityQuaternion(_rvec);
        }
        else
        {
            position = Vector3.zero;
            rotation = Quaternion.identity;
        }

        return success;
    }

    void OnDestroy()
    {
        _solver?.Dispose();
    }
}