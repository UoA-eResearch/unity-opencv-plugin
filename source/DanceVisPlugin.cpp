#include <opencv2/opencv.hpp>
#include <vector>

// Standard export macro
#if defined(_WIN32)
#  define EXPORT_API __declspec(dllexport)
#else
#  define EXPORT_API
#endif

extern "C" {

    // The Bridge Function
    EXPORT_API void SolvePnPWrapper(
        float* objectPoints,    // 3D points (x,y,z...)
        float* imagePoints,     // 2D points (u,v...)
        int count,              // Number of points
        float* cameraMatrix,    // 9 floats
        float* distCoeffs,      // 5 floats
        float* rvecOut,         // Output: 3 floats
        float* tvecOut          // Output: 3 floats
    ) {
        // 1. Map raw C arrays to OpenCV Vectors (No deep copy, just wrappers where possible)
        std::vector<cv::Point3f> objPts;
        std::vector<cv::Point2f> imgPts;

        for (int i = 0; i < count; i++) {
            objPts.push_back(cv::Point3f(objectPoints[i * 3], objectPoints[i * 3 + 1], objectPoints[i * 3 + 2]));
            imgPts.push_back(cv::Point2f(imagePoints[i * 2], imagePoints[i * 2 + 1]));
        }

        // 2. Map Camera Matrix & Distortion
        // CV_32F ensures we treat the input floats correctly
        cv::Mat camMat = cv::Mat(3, 3, CV_32F, cameraMatrix);
        cv::Mat dist = cv::Mat(1, 5, CV_32F, distCoeffs);

        // 3. Prepare Output Containers
        cv::Mat rvec, tvec;

        // 4. Run SolvePnP
        // We use SOLVEPNP_ITERATIVE as a robust default
        cv::solvePnP(objPts, imgPts, camMat, dist, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);

        // 5. Copy results back to C#
        // Note: OpenCV PnP output is usually double (CV_64F), so we cast to float
        tvecOut[0] = (float)tvec.at<double>(0);
        tvecOut[1] = (float)tvec.at<double>(1);
        tvecOut[2] = (float)tvec.at<double>(2);

        rvecOut[0] = (float)rvec.at<double>(0);
        rvecOut[1] = (float)rvec.at<double>(1);
        rvecOut[2] = (float)rvec.at<double>(2);
    }
}