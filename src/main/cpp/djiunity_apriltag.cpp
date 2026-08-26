#include <android/log.h>
#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <vector>

extern "C" {
#include "apriltag.h"
#include "common/image_u8.h"
#include "common/homography.h"
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

bool DetectAprilTagLocked(
    const uint8_t* rgbaBytes,
    int width,
    int height,
    float* outDetection,
    float* outPose,
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

    if (outPose != nullptr) {
        matd_t* pose = homography_to_pose(bestDetection->H, fx, fy, cx, cy);
        if (pose == nullptr) {
            apriltag_detections_destroy(detections);
            return false;
        }

        const auto tagHalfSizeMeters = tagSizeMeters * 0.5;
        outPose[0] = static_cast<float>(MATD_EL(pose, 0, 3) * tagHalfSizeMeters);
        outPose[1] = static_cast<float>(MATD_EL(pose, 1, 3) * tagHalfSizeMeters);
        outPose[2] = static_cast<float>(MATD_EL(pose, 2, 3) * tagHalfSizeMeters);
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column)
                outPose[3 + row * 3 + column] = static_cast<float>(MATD_EL(pose, row, column));
        }
        matd_destroy(pose);
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

    return DetectAprilTagLocked(rgbaBytes, width, height, outDetection, nullptr, 0.0, 0.0, 0.0, 0.0, 0.0) ? 1 : 0;
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

    return DetectAprilTagLocked(rgbaBytes, width, height, outDetection, outPose, fx, fy, cx, cy, tagSizeMeters) ? 1 : 0;
}

} // extern "C"
