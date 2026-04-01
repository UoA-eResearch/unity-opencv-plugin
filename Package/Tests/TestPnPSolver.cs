using System;
using System.Collections;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;
using OpenCvPlugin;
using OpenCvPlugin.PnP;

/// <summary>
/// Unit tests for OpenCV PnP solver wrapper.
/// Tests cover round-trip pose recovery, projection, error handling, and edge cases.
/// </summary>
public class TestPnPSolver
{
    private PnPSolver _solver;
    private const float POSITION_TOLERANCE = 0.01f;  // 1cm tolerance for position
    private const float ROTATION_TOLERANCE = 1.0f;   // 1 degree tolerance for rotation

    // Caller-owned pose buffers — written by Solve(), persist across calls for temporal coherence.
    private readonly double[] _rvec = new double[3];
    private readonly double[] _tvec = new double[3];

    [SetUp]
    public void Setup()
    {
        // Create fresh solver for each test
        _solver = new PnPSolver();
        Array.Clear(_rvec, 0, 3);
        Array.Clear(_tvec, 0, 3);
    }

    [TearDown]
    public void TearDown()
    {
        _solver?.Dispose();
        _solver = null;
    }

    /// <summary>
    /// Round-trip test: Apply known pose → project points → solve PnP → verify recovered pose matches.
    /// This is the fundamental validation that the wrapper correctly interfaces with OpenCV.
    /// </summary>
    [Test]
    public void RoundTrip_KnownPose_RecoversPoseAccurately()
    {
        // Define 3D object points: a cube with 1m edges centered at origin
        float[] objectPoints = new float[]
        {
            -0.5f, -0.5f, -0.5f,  // vertex 0
             0.5f, -0.5f, -0.5f,  // vertex 1
             0.5f,  0.5f, -0.5f,  // vertex 2
            -0.5f,  0.5f, -0.5f,  // vertex 3
            -0.5f, -0.5f,  0.5f,  // vertex 4
             0.5f, -0.5f,  0.5f,  // vertex 5
             0.5f,  0.5f,  0.5f,  // vertex 6
            -0.5f,  0.5f,  0.5f,  // vertex 7
        };

        // Define camera intrinsics (1920x1080 camera, ~60deg FOV)
        float fx = 1000f, fy = 1000f;
        float cx = 960f, cy = 540f;
        float[] cameraMatrix = new float[]
        {
            fx,  0f,  cx,
            0f,  fy,  cy,
            0f,  0f,  1f
        };

        // Define ground truth pose: object at 2m distance, rotated 30deg around Y axis
        float[] groundTruthRvec = new float[] { 0f, 0.523599f, 0f };  // 30 degrees in radians around Y
        float[] groundTruthTvec = new float[] { 0f, 0f, 2f };         // 2m in front of camera

        // Convert ground-truth pose to double[] for ProjectPoints
        double[] gtRvec = { groundTruthRvec[0], groundTruthRvec[1], groundTruthRvec[2] };
        double[] gtTvec = { groundTruthTvec[0], groundTruthTvec[1], groundTruthTvec[2] };

        // Step 1: Use ground truth pose to project object points onto image plane
        float[] projectedImagePoints = new float[objectPoints.Length / 3 * 2];
        _solver.ProjectPoints(objectPoints, cameraMatrix, gtRvec, gtTvec, projectedImagePoints);

        // Step 2: Attempt to recover pose from projected points
        bool solved = _solver.Solve(
            objectPoints,
            projectedImagePoints,
            cameraMatrix,
            _rvec, _tvec,
            distCoeffs: null,
            method: PnPNative.SOLVEPNP_SQPNP,
            useExtrinsicGuess: false);

        // Step 3: Verify solve succeeded
        Assert.IsTrue(solved, $"PnP solve failed: {_solver.GetLastError()}");

        // Step 4: Verify recovered pose matches ground truth
        // Check translation (should be very close)
        Assert.AreEqual(groundTruthTvec[0], (float)_tvec[0], POSITION_TOLERANCE, "Translation X mismatch");
        Assert.AreEqual(groundTruthTvec[1], (float)_tvec[1], POSITION_TOLERANCE, "Translation Y mismatch");
        Assert.AreEqual(groundTruthTvec[2], (float)_tvec[2], POSITION_TOLERANCE, "Translation Z mismatch");

        // Check rotation (convert Rodrigues to angle-axis for comparison)
        float recoveredAngle = Mathf.Sqrt(
            (float)(_rvec[0] * _rvec[0] + _rvec[1] * _rvec[1] + _rvec[2] * _rvec[2])) * Mathf.Rad2Deg;
        float groundTruthAngle = Mathf.Sqrt(groundTruthRvec[0] * groundTruthRvec[0] +
                                            groundTruthRvec[1] * groundTruthRvec[1] +
                                            groundTruthRvec[2] * groundTruthRvec[2]) * Mathf.Rad2Deg;

        Assert.AreEqual(groundTruthAngle, recoveredAngle, ROTATION_TOLERANCE, "Rotation angle mismatch");

        Debug.Log($"Round-trip test passed!");
    }

    /// <summary>
    /// Tests basic PnP solving with minimal 4-point configuration (square planar object).
    /// </summary>
    [Test]
    public void BasicSolve_MinimalFourPoints_Succeeds()
    {
        // Define a simple square at Z=0
        float[] objectPoints = new float[]
        {
            -0.5f, -0.5f, 0f,
             0.5f, -0.5f, 0f,
             0.5f,  0.5f, 0f,
            -0.5f,  0.5f, 0f
        };

        // Synthetic image points for object at ~1.5m distance
        float[] imagePoints = new float[]
        {
            320f, 480f,
            960f, 480f,
            960f, 240f,
            320f, 240f
        };

        float[] cameraMatrix = new float[]
        {
            800f, 0f, 640f,
            0f, 800f, 360f,
            0f, 0f, 1f
        };

        bool solved = _solver.Solve(objectPoints, imagePoints, cameraMatrix, _rvec, _tvec);

        Assert.IsTrue(solved, "Basic 4-point solve should succeed");
        Assert.Greater(_tvec[2], 0.0, "Object should be in front of camera (positive Z)");
        Debug.Log($"Basic solve: tvec=[{_tvec[0]:F2}, {_tvec[1]:F2}, {_tvec[2]:F2}]");
    }

    // RansacSolve_WithOutliers_RejectsOutliers removed: solvePnPRansac has been removed from the plugin.

    /// <summary>
    /// Tests ProjectPoints function directly.
    /// </summary>
    [Test]
    public void ProjectPoints_WithKnownPose_ProjectsCorrectly()
    {
        // Simple 4-point square
        float[] objectPoints = new float[]
        {
            -0.5f, -0.5f, 0f,
             0.5f, -0.5f, 0f,
             0.5f,  0.5f, 0f,
            -0.5f,  0.5f, 0f
        };

        // Object at Z=2m, no rotation
        double[] rvecD = { 0.0, 0.0, 0.0 };
        double[] tvecD = { 0.0, 0.0, 2.0 };
        float[] cameraMatrix = new float[] { 800f, 0f, 640f, 0f, 800f, 360f, 0f, 0f, 1f };

        float[] projectedPoints = new float[8];
        _solver.ProjectPoints(objectPoints, cameraMatrix, rvecD, tvecD, projectedPoints);

        // Verify points are within image bounds (1280x720)
        for (int i = 0; i < 4; i++)
        {
            float x = projectedPoints[i * 2];
            float y = projectedPoints[i * 2 + 1];
            Assert.Greater(x, 0f, $"Point {i} X should be positive");
            Assert.Less(x, 1280f, $"Point {i} X should be within image width");
            Assert.Greater(y, 0f, $"Point {i} Y should be positive");
            Assert.Less(y, 720f, $"Point {i} Y should be within image height");
        }

        Debug.Log($"Projected points: ({projectedPoints[0]:F1}, {projectedPoints[1]:F1}), ...");
    }

    /// <summary>
    /// Tests error handling with invalid inputs (null, wrong sizes, etc).
    /// </summary>
    [Test]
    public void Solve_WithInvalidInputs_ThrowsAppropriateExceptions()
    {
        float[] validObjPts = new float[] { 0f, 0f, 0f, 1f, 0f, 0f, 1f, 1f, 0f, 0f, 1f, 0f };
        float[] validImgPts = new float[] { 100f, 100f, 200f, 100f, 200f, 200f, 100f, 200f };
        float[] validCamMat = new float[] { 800f, 0f, 640f, 0f, 800f, 360f, 0f, 0f, 1f };

        // Test null object points
        Assert.Throws<ArgumentException>(() =>
            _solver.Solve(null, validImgPts, validCamMat, _rvec, _tvec));

        // Test null image points
        Assert.Throws<ArgumentException>(() =>
            _solver.Solve(validObjPts, null, validCamMat, _rvec, _tvec));

        // Test null camera matrix
        Assert.Throws<ArgumentException>(() =>
            _solver.Solve(validObjPts, validImgPts, null, _rvec, _tvec));

        // Test wrong camera matrix size
        float[] badCamMat = new float[] { 800f, 800f, 640f, 360f }; // Only 4 elements instead of 9
        Assert.Throws<ArgumentException>(() =>
            _solver.Solve(validObjPts, validImgPts, badCamMat, _rvec, _tvec));

        // Test mismatched point counts
        float[] shortImgPts = new float[] { 100f, 100f, 200f, 100f }; // Only 2 points instead of 4
        Assert.Throws<ArgumentException>(() =>
            _solver.Solve(validObjPts, shortImgPts, validCamMat, _rvec, _tvec));

        // Test too few points (need at least 4)
        float[] tooFewObjPts = new float[] { 0f, 0f, 0f, 1f, 0f, 0f, 1f, 1f, 0f }; // Only 3 points
        float[] tooFewImgPts = new float[] { 100f, 100f, 200f, 100f, 200f, 200f }; // Only 3 points
        Assert.Throws<ArgumentException>(() =>
            _solver.Solve(tooFewObjPts, tooFewImgPts, validCamMat, _rvec, _tvec));

        Debug.Log("All error handling tests passed");
    }

    /// <summary>
    /// Tests edge case: exactly 4 points (minimum required).
    /// </summary>
    [Test]
    public void Solve_WithExactlyFourPoints_Succeeds()
    {
        float[] objectPoints = new float[] { 0f, 0f, 0f, 1f, 0f, 0f, 1f, 1f, 0f, 0f, 1f, 0f };
        float[] imagePoints = new float[] { 320f, 480f, 960f, 480f, 960f, 240f, 320f, 240f };
        float[] cameraMatrix = new float[] { 800f, 0f, 640f, 0f, 800f, 360f, 0f, 0f, 1f };

        bool solved = _solver.Solve(objectPoints, imagePoints, cameraMatrix, _rvec, _tvec);

        Assert.IsTrue(solved, "Should solve with exactly 4 points");
    }

    /// <summary>
    /// Tests edge case: many points (stress test).
    /// </summary>
    [Test]
    public void Solve_WithManyPoints_Succeeds()
    {
        // Generate 50 random points on a plane
        int numPoints = 50;
        float[] objectPoints = new float[numPoints * 3];
        float[] imagePoints = new float[numPoints * 2];
        
        System.Random rng = new System.Random(42); // Fixed seed for reproducibility
        for (int i = 0; i < numPoints; i++)
        {
            // Random points in 1x1 square at Z=0
            objectPoints[i * 3] = (float)(rng.NextDouble() - 0.5);
            objectPoints[i * 3 + 1] = (float)(rng.NextDouble() - 0.5);
            objectPoints[i * 3 + 2] = 0f;

            // Fake image points (would need projection in real test)
            imagePoints[i * 2] = (float)(640 + rng.NextDouble() * 100);
            imagePoints[i * 2 + 1] = (float)(360 + rng.NextDouble() * 100);
        }

        float[] cameraMatrix = new float[] { 800f, 0f, 640f, 0f, 800f, 360f, 0f, 0f, 1f };

        bool solved = _solver.Solve(objectPoints, imagePoints, cameraMatrix, _rvec, _tvec);

        Assert.IsTrue(solved, "Should handle 50 points");
        Debug.Log($"Solved with {numPoints} points, tvec=({_tvec[0]:F3}, {_tvec[1]:F3}, {_tvec[2]:F3})");
    }

    // SetPoseAndResetPose removed: SetPose/ResetPose have been removed from PnPSolver.
    // Callers now own the rvec/tvec buffers and manage them directly.

    /// <summary>
    /// Tests different solver methods (SQPNP, ITERATIVE, EPNP) all work.
    /// </summary>
    [Test]
    public void Solve_WithDifferentMethods_AllSucceed()
    {
        float[] objectPoints = new float[]
        {
            -0.5f, -0.5f, 0f, 0.5f, -0.5f, 0f, 0.5f, 0.5f, 0f, -0.5f, 0.5f, 0f
        };
        float[] imagePoints = new float[] { 320f, 480f, 960f, 480f, 960f, 240f, 320f, 240f };
        float[] cameraMatrix = new float[] { 800f, 0f, 640f, 0f, 800f, 360f, 0f, 0f, 1f };

        int[] methods = { 
            PnPNative.SOLVEPNP_SQPNP, 
            PnPNative.SOLVEPNP_ITERATIVE, 
            PnPNative.SOLVEPNP_EPNP 
        };
        string[] methodNames = { "SQPNP", "ITERATIVE", "EPNP" };

        for (int i = 0; i < methods.Length; i++)
        {
            Array.Clear(_rvec, 0, 3);
            Array.Clear(_tvec, 0, 3);
            bool solved = _solver.Solve(objectPoints, imagePoints, cameraMatrix, _rvec, _tvec, method: methods[i]);
            Assert.IsTrue(solved, $"Method {methodNames[i]} should succeed");
            Debug.Log($"{methodNames[i]}: tvec=({_tvec[0]:F3}, {_tvec[1]:F3}, {_tvec[2]:F3})");
        }
    }

    /// <summary>
    /// Tests GeometryUtils coordinate conversion functions.
    /// Validates separation of mathematical conversions vs coordinate system transformations.
    /// </summary>
    [Test]
    public void GeometryUtils_CoordinateConversion_WorksCorrectly()
    {
        // ===== Test 1: Pure mathematical conversion (NO coordinate system change) =====
        float[] rvec = new float[] { 0f, 0f, 0f }; // No rotation
        Quaternion q = GeometryUtils.RodriguesToQuaternion(rvec);
        Assert.AreEqual(Quaternion.identity, q, "Zero rotation should give identity quaternion");

        // Non-zero rotation: pure conversion without coordinate change
        float[] rvecX = new float[] { 1.5708f, 0f, 0f }; // 90 degrees around X axis (pi/2 radians)
        Quaternion qX = GeometryUtils.RodriguesToQuaternion(rvecX);
        float angleX;
        Vector3 axisX;
        qX.ToAngleAxis(out angleX, out axisX);
        Assert.AreEqual(90f, angleX, 0.1f, "Should be 90 degree rotation");
        Assert.AreEqual(1f, Mathf.Abs(axisX.x), 0.01f, "Axis should be X");
        Assert.AreEqual(0f, axisX.y, 0.01f, "Axis should be X");
        Assert.AreEqual(0f, axisX.z, 0.01f, "Axis should be X");

        // ===== Test 2: Coordinate system transformations =====
        // Position conversion (OpenCV → Unity)
        Vector3 opencvPos = new Vector3(1f, 2f, 3f);
        Vector3 unityPos = GeometryUtils.ConvertPositionOpenCVToUnity(opencvPos);
        Assert.AreEqual(1f, unityPos.x, "X should be unchanged");
        Assert.AreEqual(-2f, unityPos.y, "Y should be flipped");
        Assert.AreEqual(3f, unityPos.z, "Z should be unchanged");

        // Position conversion (Unity → OpenCV) - roundtrip
        Vector3 backToOpenCV = GeometryUtils.ConvertPositionUnityToOpenCV(unityPos);
        Assert.AreEqual(opencvPos.x, backToOpenCV.x, 0.001f, "Roundtrip X");
        Assert.AreEqual(opencvPos.y, backToOpenCV.y, 0.001f, "Roundtrip Y");
        Assert.AreEqual(opencvPos.z, backToOpenCV.z, 0.001f, "Roundtrip Z");

        // Rotation conversion (OpenCV → Unity)
        Quaternion opencvRot = Quaternion.Euler(30f, 45f, 60f);
        Quaternion unityRot = GeometryUtils.ConvertRotationOpenCVToUnity(opencvRot);
        Assert.AreNotEqual(opencvRot, unityRot, "Coordinate conversion should change rotation");

        // Rotation conversion (Unity → OpenCV) - roundtrip
        Quaternion backToOpenCVRot = GeometryUtils.ConvertRotationUnityToOpenCV(unityRot);
        Assert.AreEqual(opencvRot.x, backToOpenCVRot.x, 0.001f, "Roundtrip rotation X");
        Assert.AreEqual(opencvRot.y, backToOpenCVRot.y, 0.001f, "Roundtrip rotation Y");
        Assert.AreEqual(opencvRot.z, backToOpenCVRot.z, 0.001f, "Roundtrip rotation Z");
        Assert.AreEqual(opencvRot.w, backToOpenCVRot.w, 0.001f, "Roundtrip rotation W");

        // ===== Test 3: Convenience methods (combined operations) =====
        float[] tvec = new float[] { 1f, 2f, 3f };
        Vector3 pos = GeometryUtils.TranslationToUnityPosition(tvec);
        Assert.AreEqual(1f, pos.x);
        Assert.AreEqual(-2f, pos.y, "Y should be flipped for OpenCV→Unity conversion");
        Assert.AreEqual(3f, pos.z);

        Quaternion combinedQuat = GeometryUtils.RodriguesToUnityQuaternion(rvecX);
        Assert.AreNotEqual(qX, combinedQuat, "Combined method should apply coordinate transform");

        // ===== Test 4: Camera matrix generation =====
        Camera dummyCam = new GameObject("TestCam").AddComponent<Camera>();
        dummyCam.fieldOfView = 60f;

        float[] camMat = GeometryUtils.GetCameraMatrix(dummyCam);
        Assert.AreEqual(9, camMat.Length, "Camera matrix should be 9 elements");
        Assert.Greater(camMat[0], 0f, "fx should be positive");
        Assert.Greater(camMat[4], 0f, "fy should be positive");
        Assert.AreEqual(1f, camMat[8], "Bottom-right should be 1");

        UnityEngine.Object.DestroyImmediate(dummyCam.gameObject);

        Debug.Log("GeometryUtils tests passed - math conversions and coordinate transforms properly separated");
    }
}
