#include <android/log.h>
#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

extern "C" {
#include "apriltag.h"
#include "common/image_u8.h"
#include "tagStandard41h12.h"
}

#ifndef JNIEXPORT
#define JNIEXPORT __attribute__((visibility("default")))
#endif

#ifndef JNICALL
#define JNICALL
#endif

#define APRILTAG_LOGI(...) __android_log_print(ANDROID_LOG_INFO, "DJIAprilTag", __VA_ARGS__)
#define APRILTAG_LOGW(...) __android_log_print(ANDROID_LOG_WARN, "DJIAprilTag", __VA_ARGS__)

namespace
{
constexpr int kNoTargetTagId = -1;
constexpr int kRequiredOutputLength = 12;
constexpr int kRequiredPoseOutputLength = 12;
constexpr int kMaximumPoseCandidates = 2;
constexpr int kPoseCandidateStride = 13;
constexpr int kRequiredPoseCandidateOutputLength = kMaximumPoseCandidates * kPoseCandidateStride;
constexpr int kDetectorThreads = 2;
constexpr float kQuadDecimate = 2.0f;
constexpr float kDecodeSharpening = 0.25f;

std::mutex g_detectorMutex;
apriltag_detector_t* g_detector = nullptr;
apriltag_family_t* g_family = nullptr;
std::vector<uint8_t> g_grayscaleBuffer;
int g_targetTagId = 0;

void ResetOutput(float* outDetection, int outDetectionLength)
{
    if (outDetection == nullptr || outDetectionLength <= 0)
        return;

    std::fill(outDetection, outDetection + outDetectionLength, 0.0f);
    outDetection[0] = -1.0f;
}

void ResetPoseOutput(float* outPose, int outPoseLength)
{
    if (outPose == nullptr || outPoseLength <= 0)
        return;

    std::fill(outPose, outPose + outPoseLength, 0.0f);
}

void ResetPoseCandidateOutput(float* outPoseCandidates, int outPoseCandidatesLength)
{
    if (outPoseCandidates == nullptr || outPoseCandidatesLength <= 0)
        return;

    std::fill(outPoseCandidates, outPoseCandidates + outPoseCandidatesLength, 0.0f);
    for (int candidateIndex = 0; candidateIndex < kMaximumPoseCandidates; ++candidateIndex) {
        const auto errorOffset = candidateIndex * kPoseCandidateStride + kRequiredPoseOutputLength;
        if (errorOffset < outPoseCandidatesLength)
            outPoseCandidates[errorOffset] = FLT_MAX;
    }
}

void ReleaseDetectorLocked()
{
    if (g_detector != nullptr) {
        apriltag_detector_destroy(g_detector);
        g_detector = nullptr;
    }

    if (g_family != nullptr) {
        tagStandard41h12_destroy(g_family);
        g_family = nullptr;
    }

    g_grayscaleBuffer.clear();
}

bool EnsureDetectorLocked()
{
    if (g_detector != nullptr && g_family != nullptr)
        return true;

    ReleaseDetectorLocked();

    g_detector = apriltag_detector_create();
    g_family = tagStandard41h12_create();
    if (g_detector == nullptr || g_family == nullptr) {
        APRILTAG_LOGW("Failed to create AprilTag detector resources.");
        ReleaseDetectorLocked();
        return false;
    }

    apriltag_detector_add_family_bits(g_detector, g_family, 1);
    g_detector->nthreads = kDetectorThreads;
    g_detector->quad_decimate = kQuadDecimate;
    g_detector->quad_sigma = 0.0f;
    g_detector->decode_sharpening = kDecodeSharpening;
    g_detector->debug = 0;

    APRILTAG_LOGI("AprilTag detector ready (family=tagStandard41h12 targetId=%d).", g_targetTagId);
    return true;
}

double ComputeScore(const apriltag_detection_t* detection, bool isTargetTag)
{
    if (detection == nullptr)
        return -DBL_MAX;

    const double targetBias = isTargetTag ? 1000000.0 : 0.0;
    return targetBias + detection->decision_margin - (detection->hamming * 10.0);
}

float NormalizeCoordinate(double value, int dimension)
{
    if (dimension <= 0)
        return 0.0f;

    const auto normalized = static_cast<float>(value / static_cast<double>(dimension));
    return std::clamp(normalized, 0.0f, 1.0f);
}

bool IsFiniteMatrix(const cv::Mat& matrix)
{
    for (int row = 0; row < matrix.rows; ++row) {
        for (int column = 0; column < matrix.cols; ++column) {
            if (!std::isfinite(matrix.at<double>(row, column)))
                return false;
        }
    }
    return true;
}

int SolveAprilTagPoseCandidates(
    const apriltag_detection_t* detection,
    double fx,
    double fy,
    double cx,
    double cy,
    double tagSizeMeters,
    float* outPoseCandidates,
    int outPoseCandidatesLength)
{
    if (detection == nullptr || outPoseCandidates == nullptr || outPoseCandidatesLength < kPoseCandidateStride)
        return 0;

    const auto halfTagSize = tagSizeMeters * 0.5;
    // This order is mandated by OpenCV's SOLVEPNP_IPPE_SQUARE implementation.
    const std::vector<cv::Point3d> objectPoints = {
        {-halfTagSize, halfTagSize, 0.0},
        {halfTagSize, halfTagSize, 0.0},
        {halfTagSize, -halfTagSize, 0.0},
        {-halfTagSize, -halfTagSize, 0.0},
    };
    const std::vector<cv::Point2d> imagePoints = {
        {detection->p[0][0], detection->p[0][1]},
        {detection->p[1][0], detection->p[1][1]},
        {detection->p[2][0], detection->p[2][1]},
        {detection->p[3][0], detection->p[3][1]},
    };

    const cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) <<
        fx, 0.0, cx,
        0.0, fy, cy,
        0.0, 0.0, 1.0);
    const cv::Mat zeroDistortion = cv::Mat::zeros(4, 1, CV_64F);
    std::vector<cv::Mat> rotationVectors;
    std::vector<cv::Mat> translationVectors;

    try {
        if (cv::solvePnPGeneric(
                objectPoints,
                imagePoints,
                cameraMatrix,
                zeroDistortion,
                rotationVectors,
                translationVectors,
                false,
                cv::SOLVEPNP_IPPE_SQUARE) == 0)
        {
            return 0;
        }
    }
    catch (const cv::Exception&) {
        return 0;
    }

    auto candidateCount = 0;
    for (size_t index = 0; index < rotationVectors.size() && index < translationVectors.size(); ++index) {
        // Start each refinement from its own IPPE hypothesis. The Unity side
        // selects between the resulting poses in ARCore world space, so both
        // hypotheses must be retained rather than choosing OpenCV's first one.
        auto rotationVector = rotationVectors[index].clone();
        auto translationVector = translationVectors[index].clone();
        if (rotationVector.empty() || translationVector.empty() ||
            !IsFiniteMatrix(rotationVector) || !IsFiniteMatrix(translationVector) ||
            translationVector.at<double>(2, 0) <= 0.0)
        {
            continue;
        }

        try {
            cv::solvePnPRefineLM(objectPoints, imagePoints, cameraMatrix, zeroDistortion, rotationVector, translationVector);
        }
        catch (const cv::Exception&) {
            continue;
        }

        if (!IsFiniteMatrix(rotationVector) || !IsFiniteMatrix(translationVector) ||
            translationVector.at<double>(2, 0) <= 0.0)
        {
            continue;
        }

        std::vector<cv::Point2d> reprojectedPoints;
        cv::projectPoints(objectPoints, rotationVector, translationVector, cameraMatrix, zeroDistortion, reprojectedPoints);
        if (reprojectedPoints.size() != imagePoints.size())
            continue;

        auto squaredError = 0.0;
        for (size_t pointIndex = 0; pointIndex < imagePoints.size(); ++pointIndex) {
            const auto delta = reprojectedPoints[pointIndex] - imagePoints[pointIndex];
            squaredError += delta.dot(delta);
        }
        const auto rmsError = std::sqrt(squaredError / static_cast<double>(imagePoints.size()));
        if (!std::isfinite(rmsError) || rmsError > 12.0 || candidateCount >= kMaximumPoseCandidates ||
            (candidateCount + 1) * kPoseCandidateStride > outPoseCandidatesLength)
        {
            continue;
        }

        cv::Mat rotationMatrix;
        cv::Rodrigues(rotationVector, rotationMatrix);
        if (!IsFiniteMatrix(rotationMatrix))
            continue;

        const auto outputOffset = candidateCount * kPoseCandidateStride;
        outPoseCandidates[outputOffset] = static_cast<float>(translationVector.at<double>(0, 0));
        outPoseCandidates[outputOffset + 1] = static_cast<float>(translationVector.at<double>(1, 0));
        outPoseCandidates[outputOffset + 2] = static_cast<float>(translationVector.at<double>(2, 0));
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column)
                outPoseCandidates[outputOffset + 3 + row * 3 + column] = static_cast<float>(rotationMatrix.at<double>(row, column));
        }
        outPoseCandidates[outputOffset + kRequiredPoseOutputLength] = static_cast<float>(rmsError);
        ++candidateCount;
    }
    return candidateCount;
}

bool DetectAprilTagLocked(
    const uint8_t* rgbaBytes,
    int width,
    int height,
    float* outDetection,
    float* outPose,
    float* outPoseCandidates,
    int outPoseCandidatesLength,
    double fx,
    double fy,
    double cx,
    double cy,
    double tagSizeMeters)
{
    const auto pixelCount = static_cast<size_t>(width) * static_cast<size_t>(height);
    g_grayscaleBuffer.resize(pixelCount);

    for (size_t pixelIndex = 0, rgbaIndex = 0; pixelIndex < pixelCount; ++pixelIndex, rgbaIndex += 4) {
        const auto r = rgbaBytes[rgbaIndex];
        const auto g = rgbaBytes[rgbaIndex + 1];
        const auto b = rgbaBytes[rgbaIndex + 2];
        g_grayscaleBuffer[pixelIndex] = static_cast<uint8_t>((77 * r + 150 * g + 29 * b) >> 8);
    }

    image_u8_t image = {
        width,
        height,
        width,
        g_grayscaleBuffer.data()
    };

    zarray_t* detections = apriltag_detector_detect(g_detector, &image);
    if (detections == nullptr)
        return false;

    const auto detectionCount = zarray_size(detections);
    apriltag_detection_t* bestDetection = nullptr;
    auto bestScore = -DBL_MAX;

    for (int index = 0; index < detectionCount; ++index) {
        apriltag_detection_t* current = nullptr;
        zarray_get(detections, index, &current);
        if (current == nullptr)
            continue;

        const auto isTargetTag = g_targetTagId == kNoTargetTagId || current->id == g_targetTagId;
        if (!isTargetTag)
            continue;

        const auto score = ComputeScore(current, isTargetTag);
        if (score <= bestScore)
            continue;

        bestScore = score;
        bestDetection = current;
    }

    if (bestDetection == nullptr) {
        apriltag_detections_destroy(detections);
        return false;
    }

    outDetection[0] = static_cast<float>(bestDetection->id);
    outDetection[1] = NormalizeCoordinate(bestDetection->c[0], width);
    outDetection[2] = NormalizeCoordinate(bestDetection->c[1], height);
    outDetection[3] = NormalizeCoordinate(bestDetection->p[0][0], width);
    outDetection[4] = NormalizeCoordinate(bestDetection->p[0][1], height);
    outDetection[5] = NormalizeCoordinate(bestDetection->p[1][0], width);
    outDetection[6] = NormalizeCoordinate(bestDetection->p[1][1], height);
    outDetection[7] = NormalizeCoordinate(bestDetection->p[2][0], width);
    outDetection[8] = NormalizeCoordinate(bestDetection->p[2][1], height);
    outDetection[9] = NormalizeCoordinate(bestDetection->p[3][0], width);
    outDetection[10] = NormalizeCoordinate(bestDetection->p[3][1], height);
    outDetection[11] = static_cast<float>(bestDetection->decision_margin);

    if (outPose != nullptr || outPoseCandidates != nullptr) {
        float legacyPoseCandidates[kRequiredPoseCandidateOutputLength];
        auto* poseCandidates = outPoseCandidates != nullptr ? outPoseCandidates : legacyPoseCandidates;
        const auto poseCandidatesLength = outPoseCandidates != nullptr ? outPoseCandidatesLength : kRequiredPoseCandidateOutputLength;
        const auto poseCandidateCount = SolveAprilTagPoseCandidates(
            bestDetection, fx, fy, cx, cy, tagSizeMeters, poseCandidates, poseCandidatesLength);
        if (poseCandidateCount <= 0) {
            apriltag_detections_destroy(detections);
            return false;
        }

        if (outPose != nullptr)
            std::copy(poseCandidates, poseCandidates + kRequiredPoseOutputLength, outPose);
    }

    apriltag_detections_destroy(detections);
    return true;
}
} // namespace

extern "C" {

JNIEXPORT void JNICALL DJI_SetAprilTagTargetId(int tagId)
{
    std::lock_guard<std::mutex> lock(g_detectorMutex);
    g_targetTagId = tagId;
    APRILTAG_LOGI("AprilTag target id updated to %d.", g_targetTagId);
}

JNIEXPORT void JNICALL DJI_ReleaseAprilTagDetector()
{
    std::lock_guard<std::mutex> lock(g_detectorMutex);
    ReleaseDetectorLocked();
    APRILTAG_LOGI("AprilTag detector released.");
}

JNIEXPORT int JNICALL DJI_DetectAprilTagRgba32(
    const uint8_t* rgbaBytes,
    int width,
    int height,
    float* outDetection,
    int outDetectionLength)
{
    ResetOutput(outDetection, outDetectionLength);

    if (rgbaBytes == nullptr || width <= 0 || height <= 0 || outDetection == nullptr || outDetectionLength < kRequiredOutputLength)
        return 0;

    std::lock_guard<std::mutex> lock(g_detectorMutex);
    if (!EnsureDetectorLocked())
        return 0;

    return DetectAprilTagLocked(rgbaBytes, width, height, outDetection, nullptr, nullptr, 0, 0.0, 0.0, 0.0, 0.0, 0.0) ? 1 : 0;
}

JNIEXPORT int JNICALL DJI_DetectAprilTagPoseRgba32(
    const uint8_t* rgbaBytes,
    int width,
    int height,
    float fx,
    float fy,
    float cx,
    float cy,
    float tagSizeMeters,
    float* outDetection,
    int outDetectionLength,
    float* outPose,
    int outPoseLength)
{
    ResetOutput(outDetection, outDetectionLength);
    ResetPoseOutput(outPose, outPoseLength);

    if (rgbaBytes == nullptr || width <= 0 || height <= 0 || outDetection == nullptr || outDetectionLength < kRequiredOutputLength ||
        outPose == nullptr || outPoseLength < kRequiredPoseOutputLength || !std::isfinite(fx) || !std::isfinite(fy) ||
        !std::isfinite(cx) || !std::isfinite(cy) || !std::isfinite(tagSizeMeters) || fx <= 0.0f || fy <= 0.0f || tagSizeMeters <= 0.0f)
        return 0;

    std::lock_guard<std::mutex> lock(g_detectorMutex);
    if (!EnsureDetectorLocked())
        return 0;

    return DetectAprilTagLocked(rgbaBytes, width, height, outDetection, outPose, nullptr, 0, fx, fy, cx, cy, tagSizeMeters) ? 1 : 0;
}

JNIEXPORT int JNICALL DJI_DetectAprilTagPoseCandidatesRgba32(
    const uint8_t* rgbaBytes,
    int width,
    int height,
    float fx,
    float fy,
    float cx,
    float cy,
    float tagSizeMeters,
    float* outDetection,
    int outDetectionLength,
    float* outPoseCandidates,
    int outPoseCandidatesLength)
{
    ResetOutput(outDetection, outDetectionLength);
    ResetPoseCandidateOutput(outPoseCandidates, outPoseCandidatesLength);

    if (rgbaBytes == nullptr || width <= 0 || height <= 0 || outDetection == nullptr || outDetectionLength < kRequiredOutputLength ||
        outPoseCandidates == nullptr || outPoseCandidatesLength < kRequiredPoseCandidateOutputLength || !std::isfinite(fx) || !std::isfinite(fy) ||
        !std::isfinite(cx) || !std::isfinite(cy) || !std::isfinite(tagSizeMeters) || fx <= 0.0f || fy <= 0.0f || tagSizeMeters <= 0.0f)
    {
        return 0;
    }

    std::lock_guard<std::mutex> lock(g_detectorMutex);
    if (!EnsureDetectorLocked())
        return 0;

    DetectAprilTagLocked(
        rgbaBytes, width, height, outDetection, nullptr, outPoseCandidates, outPoseCandidatesLength, fx, fy, cx, cy, tagSizeMeters);
    auto candidateCount = 0;
    for (int candidateIndex = 0; candidateIndex < kMaximumPoseCandidates; ++candidateIndex) {
        if (outPoseCandidates[candidateIndex * kPoseCandidateStride + kRequiredPoseOutputLength] < FLT_MAX)
            ++candidateCount;
    }
    return candidateCount;
}

} // extern "C"
