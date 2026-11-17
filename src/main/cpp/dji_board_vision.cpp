#ifndef DJI_BOARD_POSE_REPLAY_TEST
#include <jni.h>
#include <android/log.h>
#endif

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <chrono>
#include <cstdio>
#include <cstdarg>
#include <exception>
#include <limits>
#include <mutex>
#include <sstream>
#include <iomanip>
#include <string>
#include <unordered_map>
#include <vector>

#ifndef DJI_BOARD_POSE_REPLAY_TEST
extern "C" {
#include "apriltag.h"
#include "apriltag_pose.h"
#include "common/image_u8.h"
#include "tagStandard41h12.h"
}
#else
extern "C" {
#include "apriltag/common/matd.h"
}
#endif

#ifndef DJI_APRILTAG_FOUR_TAG_MULTISTART_EXPERIMENT
#define DJI_APRILTAG_FOUR_TAG_MULTISTART_EXPERIMENT 0
#endif
thread_local bool g_quietPoseRefinement = false;
#ifdef DJI_BOARD_POSE_REPLAY_TEST
#define BOARD_LOGI(...) ((void)0)
#define BOARD_LOGW(...) ((void)0)
#define BOARD_LOGE(...) ((void)0)
#else
#define BOARD_LOGI(...) do { if (!g_quietPoseRefinement) __android_log_print(ANDROID_LOG_INFO, "DjiBoardVision", __VA_ARGS__); } while (0)
#define BOARD_LOGW(...) do { if (!g_quietPoseRefinement) __android_log_print(ANDROID_LOG_WARN, "DjiBoardVision", __VA_ARGS__); } while (0)
#define BOARD_LOGE(...) __android_log_print(ANDROID_LOG_ERROR, "DjiBoardVision", __VA_ARGS__)
#endif

namespace {
constexpr int kLayoutStride = 9; // id, size, board position xyz, board rotation xyzw
constexpr int kHeaderLength = 17;
constexpr int kMarkerOutputLength = 18; // id, margin, detected corners, projected corners
constexpr int kDetectorThreads = 2;
constexpr double kFiniteDifference = 1e-5;
constexpr int kMaxRefinementIterations = 20;
constexpr int kMaxRefinementStepReductions = 8;
constexpr int64_t kMaxFramePixels = 16 * 1024 * 1024;

enum class FailureCode : int {
    InvalidFrame = 1,
    InvalidCalibration = 2,
    InvalidDistortion = 3,
    InvalidLayout = 4,
    LumaLengthMismatch = 5,
    DetectorUnavailable = 6,
    DetectionUnavailable = 7,
    NoMarkers = 8,
    UnknownMarkerId = 9,
    InvalidCorners = 10,
    CorrespondenceMismatch = 11,
    PoseInitializationFailed = 12,
    PoseRefinementFailed = 13,
    JniException = 14,
    CppException = 15,
    UnknownException = 16,
};

struct Vec2 { double x; double y; };
struct Vec3 { double x; double y; double z; };
struct Quaternion { double x; double y; double z; double w; };
struct RigidPose { Quaternion q; Vec3 t; };
struct Distortion { double k1; double k2; double p1; double p2; double k3; };

struct MarkerDefinition {
    int id;
    // AprilTag's pose API defines tagsize at the detection-corner border
    // transition, not at the outside edge of the complete printed tile.
    double aprilTagDetectionSizeMeters;
    RigidPose boardFromMarker;
};

struct Correspondence {
    int markerId;
    int cornerIndex;
    Vec3 boardPoint;
    Vec2 imagePoint;
};

struct VisibleMarker {
    int id;
    double decisionMargin;
    std::array<Vec2, 4> detectedCorners;
    std::array<Vec2, 4> projectedCorners;
};

struct IndependentMarkerPose {
    int id;
    double tagPoseError;
    RigidPose cameraFromMarker;
    RigidPose cameraFromBoard;
    MarkerDefinition definition;
    Vec2 imageCenter;
    double imageQuadrilateralSide;
};

// Optional same-thread evidence only. Never changes initializer selection or solver inputs.
thread_local bool g_captureEvidenceEnabled = false;
thread_local std::string g_captureEvidence;
template <typename Function> void CaptureEvidence(Function function) noexcept {
    if (!g_captureEvidenceEnabled) return;
    try { function(); } catch (...) { g_captureEvidenceEnabled = false; g_captureEvidence.clear(); }
}
std::string CaptureNumber(double value) {
    if (!std::isfinite(value)) return "null";
    std::ostringstream out; out << std::setprecision(17) << value; return out.str();
}
std::string CapturePose(const RigidPose& p) {
    return "{\"position\":[" + CaptureNumber(p.t.x) + "," + CaptureNumber(p.t.y) + "," + CaptureNumber(p.t.z) +
        "],\"quaternionXYZW\":[" + CaptureNumber(p.q.x) + "," + CaptureNumber(p.q.y) + "," + CaptureNumber(p.q.z) + "," + CaptureNumber(p.q.w) + "]}";
}

#ifndef DJI_BOARD_POSE_REPLAY_TEST
std::mutex g_detectorMutex;
apriltag_detector_t* g_detector = nullptr;
apriltag_family_t* g_family = nullptr;
std::vector<uint8_t> g_luma;
uint64_t g_refinementDiagnosticFrame = 0;
#endif

double Dot(const Vec3& a, const Vec3& b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
double Norm(const Vec3& v) { return std::sqrt(Dot(v, v)); }
double Distance(const Vec2& a, const Vec2& b) { return std::hypot(a.x - b.x, a.y - b.y); }
Vec3 Add(const Vec3& a, const Vec3& b) { return {a.x+b.x, a.y+b.y, a.z+b.z}; }
Vec3 Scale(const Vec3& v, double s) { return {v.x*s, v.y*s, v.z*s}; }
Vec3 Subtract(const Vec3& a, const Vec3& b) { return Add(a, Scale(b, -1.0)); }

Quaternion Normalize(Quaternion q) {
    const double length = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
    return length > 1e-12 ? Quaternion{q.x/length, q.y/length, q.z/length, q.w/length} : Quaternion{0,0,0,1};
}

Quaternion Multiply(const Quaternion& a, const Quaternion& b) {
    return Normalize({
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w,
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z
    });
}

Quaternion Inverse(const Quaternion& q) { return {-q.x, -q.y, -q.z, q.w}; }

Vec3 Rotate(const Quaternion& qInput, const Vec3& v) {
    const Quaternion q = Normalize(qInput);
    const Vec3 u{q.x, q.y, q.z};
    const Vec3 uv{u.y*v.z-u.z*v.y, u.z*v.x-u.x*v.z, u.x*v.y-u.y*v.x};
    const Vec3 uuv{u.y*uv.z-u.z*uv.y, u.z*uv.x-u.x*uv.z, u.x*uv.y-u.y*uv.x};
    return Add(v, Add(Scale(uv, 2.0*q.w), Scale(uuv, 2.0)));
}

RigidPose Compose(const RigidPose& parentFromChild, const RigidPose& childFromGrandchild) {
    return {Multiply(parentFromChild.q, childFromGrandchild.q), Add(parentFromChild.t, Rotate(parentFromChild.q, childFromGrandchild.t))};
}

RigidPose Invert(const RigidPose& parentFromChild) {
    const Quaternion inverse = Inverse(parentFromChild.q);
    return {inverse, Scale(Rotate(inverse, parentFromChild.t), -1.0)};
}

double QuaternionAngularDifferenceDegrees(const Quaternion& aInput, const Quaternion& bInput) {
    const Quaternion a = Normalize(aInput);
    const Quaternion b = Normalize(bInput);
    const double dot = std::clamp(std::abs(a.x*b.x + a.y*b.y + a.z*b.z + a.w*b.w), 0.0, 1.0);
    return 2.0 * std::acos(dot) * 57.29577951308232;
}

Quaternion QuaternionFromRotationMatrix(const matd_t* r) {
    const double m00 = MATD_EL(r, 0, 0), m11 = MATD_EL(r, 1, 1), m22 = MATD_EL(r, 2, 2);
    const double trace = m00 + m11 + m22;
    Quaternion q{};
    if (trace > 0.0) {
        const double s = std::sqrt(trace + 1.0) * 2.0;
        q = {(MATD_EL(r,2,1)-MATD_EL(r,1,2))/s, (MATD_EL(r,0,2)-MATD_EL(r,2,0))/s, (MATD_EL(r,1,0)-MATD_EL(r,0,1))/s, 0.25*s};
    } else if (m00 > m11 && m00 > m22) {
        const double s = std::sqrt(1.0 + m00 - m11 - m22) * 2.0;
        q = {0.25*s, (MATD_EL(r,0,1)+MATD_EL(r,1,0))/s, (MATD_EL(r,0,2)+MATD_EL(r,2,0))/s, (MATD_EL(r,2,1)-MATD_EL(r,1,2))/s};
    } else if (m11 > m22) {
        const double s = std::sqrt(1.0 + m11 - m00 - m22) * 2.0;
        q = {(MATD_EL(r,0,1)+MATD_EL(r,1,0))/s, 0.25*s, (MATD_EL(r,1,2)+MATD_EL(r,2,1))/s, (MATD_EL(r,0,2)-MATD_EL(r,2,0))/s};
    } else {
        const double s = std::sqrt(1.0 + m22 - m00 - m11) * 2.0;
        q = {(MATD_EL(r,0,2)+MATD_EL(r,2,0))/s, (MATD_EL(r,1,2)+MATD_EL(r,2,1))/s, 0.25*s, (MATD_EL(r,1,0)-MATD_EL(r,0,1))/s};
    }
    return Normalize(q);
}

Quaternion SmallAngleQuaternion(const Vec3& radians) {
    const double angle = Norm(radians);
    if (angle < 1e-12) return {0, 0, 0, 1};
    const double half = angle * 0.5;
    const double scale = std::sin(half) / angle;
    return {radians.x*scale, radians.y*scale, radians.z*scale, std::cos(half)};
}

Vec2 DistortNormalized(const Vec2& undistorted, const Distortion& distortion) {
    const double x = undistorted.x;
    const double y = undistorted.y;
    const double r2 = x*x + y*y;
    const double radial = 1.0 + distortion.k1*r2 + distortion.k2*r2*r2 + distortion.k3*r2*r2*r2;
    return {
        x * radial + 2.0*distortion.p1*x*y + distortion.p2*(r2 + 2.0*x*x),
        y * radial + distortion.p1*(r2 + 2.0*y*y) + 2.0*distortion.p2*x*y
    };
}

Vec2 UndistortPixel(const Vec2& distortedPixel, double fx, double fy, double cx, double cy, const Distortion& distortion) {
    const Vec2 observed{(distortedPixel.x - cx) / fx, (distortedPixel.y - cy) / fy};
    Vec2 estimate = observed;
    for (int iteration = 0; iteration < 8; ++iteration) {
        const Vec2 distortedEstimate = DistortNormalized(estimate, distortion);
        estimate.x += observed.x - distortedEstimate.x;
        estimate.y += observed.y - distortedEstimate.y;
    }
    return {fx * estimate.x + cx, fy * estimate.y + cy};
}

bool Project(const RigidPose& cameraFromBoard, const Vec3& boardPoint, double fx, double fy, double cx, double cy, const Distortion& distortion, Vec2* output) {
    const Vec3 cameraPoint = Add(Rotate(cameraFromBoard.q, boardPoint), cameraFromBoard.t);
    if (cameraPoint.z <= 1e-5) return false;
    const Vec2 normalized = DistortNormalized({cameraPoint.x / cameraPoint.z, cameraPoint.y / cameraPoint.z}, distortion);
    *output = {fx * normalized.x + cx, fy * normalized.y + cy};
    return std::isfinite(output->x) && std::isfinite(output->y);
}

Vec3 CameraPointFromBoardPoint(const RigidPose& cameraFromBoard, const Vec3& boardPoint) {
    return Add(Rotate(cameraFromBoard.q, boardPoint), cameraFromBoard.t);
}

bool IsFinite(const Vec3& value) {
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

bool IsFinite(const Quaternion& value) {
    return std::isfinite(value.x) && std::isfinite(value.y) &&
           std::isfinite(value.z) && std::isfinite(value.w);
}

const char* DescribeProjectionFailure(const RigidPose& cameraFromBoard, const Vec3& boardPoint) {
    const Vec3 cameraPoint = CameraPointFromBoardPoint(cameraFromBoard, boardPoint);
    if (!IsFinite(cameraPoint)) return "CAMERA_POINT_NONFINITE";
    if (cameraPoint.z <= 1e-5) return "CAMERA_POINT_NONPOSITIVE_DEPTH";
    return "PROJECTED_PIXEL_NONFINITE";
}

void LogPose(const char* label, const RigidPose& pose) {
    const Vec3 xAxis = Rotate(pose.q, {1, 0, 0});
    const Vec3 yAxis = Rotate(pose.q, {0, 1, 0});
    const Vec3 zAxis = Rotate(pose.q, {0, 0, 1});
    BOARD_LOGI(
        "%s translation=[%.8f,%.8f,%.8f] quaternion=[%.8f,%.8f,%.8f,%.8f] "
        "rotationColumns=[[%.8f,%.8f,%.8f],[%.8f,%.8f,%.8f],[%.8f,%.8f,%.8f]]",
        label, pose.t.x, pose.t.y, pose.t.z, pose.q.x, pose.q.y, pose.q.z, pose.q.w,
        xAxis.x, xAxis.y, xAxis.z, yAxis.x, yAxis.y, yAxis.z, zAxis.x, zAxis.y, zAxis.z);
}

void LogBoardLayoutDefinitions(const std::unordered_map<int, MarkerDefinition>& layout) {
    BOARD_LOGI(
        "BOARD_LAYOUT_AUDIT convention=BOARD_ORIGIN_CENTER axes=+X_PRINT_RIGHT,+Y_PRINT_UP,+Z_OUTWARD "
        "cornerOrder=[(-half,+half),(+half,+half),(+half,-half),(-half,-half)]");
    for (const auto& entry : layout) {
        const MarkerDefinition& marker = entry.second;
        BOARD_LOGI(
            "BOARD_LAYOUT_MARKER id=%d aprilTagDetectionSizeMeters=%.8f centerBoard=[%.8f,%.8f,%.8f] "
            "boardFromMarkerQuaternion=[%.8f,%.8f,%.8f,%.8f]",
            marker.id, marker.aprilTagDetectionSizeMeters,
            marker.boardFromMarker.t.x, marker.boardFromMarker.t.y, marker.boardFromMarker.t.z,
            marker.boardFromMarker.q.x, marker.boardFromMarker.q.y,
            marker.boardFromMarker.q.z, marker.boardFromMarker.q.w);
    }
}

struct ImageTagGeometry {
    Vec2 center;
    double meanSideLength;
};

ImageTagGeometry CalculateImageTagGeometry(const std::array<Vec2, 4>& corners) {
    Vec2 center{};
    double perimeter = 0.0;
    for (int corner = 0; corner < 4; ++corner) {
        center.x += corners[corner].x;
        center.y += corners[corner].y;
        perimeter += Distance(corners[corner], corners[(corner + 1) % 4]);
    }
    return {{center.x * 0.25, center.y * 0.25}, perimeter * 0.25};
}

void LogIndependentMarkerPoseComparison(const std::vector<IndependentMarkerPose>& poses) {
    for (const auto& pose : poses) {
        BOARD_LOGI(
            "BOARD_INDEPENDENT_MARKER_POSE id=%d officialTagPoseError=%.8f "
            "equation=T_camera_board=T_camera_marker*T_marker_board",
            pose.id, pose.tagPoseError);
        LogPose("BOARD_INDEPENDENT_T_camera_marker", pose.cameraFromMarker);
        LogPose("BOARD_INDEPENDENT_T_board_marker", pose.definition.boardFromMarker);
        LogPose("BOARD_INDEPENDENT_T_marker_board", Invert(pose.definition.boardFromMarker));
        LogPose("BOARD_INDEPENDENT_T_camera_board", pose.cameraFromBoard);
    }

    for (size_t firstIndex = 0; firstIndex < poses.size(); ++firstIndex) {
        for (size_t secondIndex = firstIndex + 1; secondIndex < poses.size(); ++secondIndex) {
            const IndependentMarkerPose& first = poses[firstIndex];
            const IndependentMarkerPose& second = poses[secondIndex];
            const Vec3 detectedDeltaCamera = Subtract(second.cameraFromMarker.t, first.cameraFromMarker.t);
            const Vec3 detectedDeltaInFirstMarker = Rotate(
                Invert(first.cameraFromMarker).q, detectedDeltaCamera);
            const Vec3 layoutDeltaBoard = Subtract(
                second.definition.boardFromMarker.t, first.definition.boardFromMarker.t);
            const Vec3 layoutDeltaInFirstMarker = Rotate(
                Invert(first.definition.boardFromMarker).q, layoutDeltaBoard);
            const double detectedLength = Norm(detectedDeltaInFirstMarker);
            const double layoutLength = Norm(layoutDeltaInFirstMarker);
            const double directionCosine = detectedLength > 1e-8 && layoutLength > 1e-8
                ? std::clamp(Dot(detectedDeltaInFirstMarker, layoutDeltaInFirstMarker) /
                    (detectedLength * layoutLength), -1.0, 1.0)
                : std::numeric_limits<double>::quiet_NaN();
            const double directionErrorDegrees = std::isfinite(directionCosine)
                ? std::acos(directionCosine) * 57.29577951308232
                : std::numeric_limits<double>::quiet_NaN();
            const double boardRotationDifferenceDegrees = QuaternionAngularDifferenceDegrees(
                first.cameraFromBoard.q, second.cameraFromBoard.q);
            BOARD_LOGI(
                "BOARD_MARKER_PAIR firstId=%d secondId=%d detectedDeltaInFirstMarker=[%.8f,%.8f,%.8f] "
                "layoutDeltaInFirstMarker=[%.8f,%.8f,%.8f] detectedLength=%.8f layoutLength=%.8f "
                "lengthRatio=%.8f directionErrorDegrees=%.8f independentBoardRotationDeltaDegrees=%.8f",
                first.id, second.id,
                detectedDeltaInFirstMarker.x, detectedDeltaInFirstMarker.y, detectedDeltaInFirstMarker.z,
                layoutDeltaInFirstMarker.x, layoutDeltaInFirstMarker.y, layoutDeltaInFirstMarker.z,
                detectedLength, layoutLength,
                layoutLength > 1e-8 ? detectedLength / layoutLength : std::numeric_limits<double>::quiet_NaN(),
                directionErrorDegrees, boardRotationDifferenceDegrees);

            const double imageCenterDistance = Distance(first.imageCenter, second.imageCenter);
            const double imageTagSide = (first.imageQuadrilateralSide + second.imageQuadrilateralSide) * 0.5;
            const double imageGeometryRatio = imageTagSide > 1e-8
                ? imageCenterDistance / imageTagSide
                : std::numeric_limits<double>::quiet_NaN();
            const double boardTagSide =
                (first.definition.aprilTagDetectionSizeMeters + second.definition.aprilTagDetectionSizeMeters) * 0.5;
            const double boardGeometryRatio = boardTagSide > 1e-8
                ? layoutLength / boardTagSide
                : std::numeric_limits<double>::quiet_NaN();
            BOARD_LOGI(
                "BOARD_DIMENSIONLESS_GEOMETRY firstId=%d secondId=%d imageCenterDistancePx=%.8f "
                "imageTagQuadrilateralSidePx=%.8f IMAGE_GEOMETRY_RATIO=%.8f "
                "boardCenterDistanceMeters=%.8f declaredPoseQuadrilateralMeters=%.8f "
                "BOARD_GEOMETRY_RATIO=%.8f RATIO_ERROR=%.8f",
                first.id, second.id, imageCenterDistance, imageTagSide, imageGeometryRatio,
                layoutLength, boardTagSide, boardGeometryRatio,
                std::isfinite(imageGeometryRatio) && std::isfinite(boardGeometryRatio)
                    ? imageGeometryRatio - boardGeometryRatio
                    : std::numeric_limits<double>::quiet_NaN());
        }
    }
}

struct ReprojectionStatistics {
    bool valid = true;
    size_t projectedCount = 0;
    double rms = std::numeric_limits<double>::quiet_NaN();
    double maximumResidual = std::numeric_limits<double>::quiet_NaN();
    double minimumDepth = std::numeric_limits<double>::infinity();
    double maximumDepth = -std::numeric_limits<double>::infinity();
    std::string failureReason;
};

ReprojectionStatistics EvaluateReprojection(
    const RigidPose& cameraFromBoard,
    const std::vector<Correspondence>& correspondences,
    double fx, double fy, double cx, double cy, const Distortion& distortion,
    bool logCorrespondences,
    const char* correspondenceLogLabel = nullptr) {
    ReprojectionStatistics statistics{};
    double squaredError = 0.0;
    double maximumResidual = 0.0;
    for (const auto& observation : correspondences) {
        const Vec3 cameraPoint = CameraPointFromBoardPoint(cameraFromBoard, observation.boardPoint);
        if (IsFinite(cameraPoint)) {
            statistics.minimumDepth = std::min(statistics.minimumDepth, cameraPoint.z);
            statistics.maximumDepth = std::max(statistics.maximumDepth, cameraPoint.z);
        }

        Vec2 projection{};
        const bool projectionValid = Project(cameraFromBoard, observation.boardPoint, fx, fy, cx, cy, distortion, &projection);
        if (!projectionValid) {
            statistics.valid = false;
            if (statistics.failureReason.empty()) statistics.failureReason = DescribeProjectionFailure(cameraFromBoard, observation.boardPoint);
            if (logCorrespondences) {
                BOARD_LOGW(
                    "%s markerId=%d cornerIndex=%d objectBoard=[%.8f,%.8f,%.8f] "
                    "detectedPixel=[%.3f,%.3f] projectedPixel=INVALID cameraPoint=[%.8f,%.8f,%.8f] reason=%s",
                    correspondenceLogLabel ? correspondenceLogLabel : "BOARD_POSE_CORRESPONDENCE",
                    observation.markerId, observation.cornerIndex,
                    observation.boardPoint.x, observation.boardPoint.y, observation.boardPoint.z,
                    observation.imagePoint.x, observation.imagePoint.y,
                    cameraPoint.x, cameraPoint.y, cameraPoint.z,
                    DescribeProjectionFailure(cameraFromBoard, observation.boardPoint));
            }
            continue;
        }

        const double dx = observation.imagePoint.x - projection.x;
        const double dy = observation.imagePoint.y - projection.y;
        const double magnitude = std::hypot(dx, dy);
        squaredError += magnitude * magnitude;
        maximumResidual = std::max(maximumResidual, magnitude);
        ++statistics.projectedCount;
        if (logCorrespondences) {
            BOARD_LOGI(
                "%s markerId=%d cornerIndex=%d objectBoard=[%.8f,%.8f,%.8f] "
                "detectedPixel=[%.3f,%.3f] projectedPixel=[%.3f,%.3f] residual=[%.3f,%.3f] magnitude=%.3f cameraDepth=%.8f",
                correspondenceLogLabel ? correspondenceLogLabel : "BOARD_POSE_CORRESPONDENCE",
                observation.markerId, observation.cornerIndex,
                observation.boardPoint.x, observation.boardPoint.y, observation.boardPoint.z,
                observation.imagePoint.x, observation.imagePoint.y, projection.x, projection.y,
                dx, dy, magnitude, cameraPoint.z);
        }
    }

    if (statistics.projectedCount > 0) {
        statistics.rms = std::sqrt(squaredError / static_cast<double>(statistics.projectedCount));
        statistics.maximumResidual = maximumResidual;
    } else {
        statistics.valid = false;
        if (statistics.failureReason.empty()) statistics.failureReason = "NO_PROJECTABLE_CORRESPONDENCES";
    }
    if (!std::isfinite(statistics.minimumDepth) || !std::isfinite(statistics.maximumDepth)) {
        statistics.valid = false;
        if (statistics.failureReason.empty()) statistics.failureReason = "DEPTH_RANGE_NONFINITE";
    }
    return statistics;
}

ReprojectionStatistics LogInitialPoseReprojection(
    const RigidPose& cameraFromBoard,
    const std::vector<Correspondence>& correspondences,
    double fx, double fy, double cx, double cy, const Distortion& distortion,
    bool logCorrespondences) {
    const ReprojectionStatistics statistics = EvaluateReprojection(
        cameraFromBoard, correspondences, fx, fy, cx, cy, distortion, logCorrespondences,
        "BOARD_INITIAL_POSE_CORRESPONDENCE");
    BOARD_LOGI(
        "BOARD_INITIAL_POSE_REPROJECTION rms=%.6f maxResidual=%.6f correspondenceCount=%zu projectedCount=%zu "
        "minDepth=%.8f maxDepth=%.8f valid=%d reason=%s",
        statistics.rms, statistics.maximumResidual, correspondences.size(), statistics.projectedCount,
        statistics.minimumDepth, statistics.maximumDepth, statistics.valid ? 1 : 0,
        statistics.failureReason.empty() ? "NONE" : statistics.failureReason.c_str());
    return statistics;
}

bool ReportRefineFailure(
    std::string* failureReason,
    const char* exactReason,
    const char* sourceFunction,
    int sourceLine,
    bool /* logDetails */) {
    if (failureReason) *failureReason = exactReason;
    BOARD_LOGW(
        "BOARD_REFINE_FAILURE code=13 exactReason=%s sourceFunction=%s sourceLine=%d",
        exactReason, sourceFunction, sourceLine);
    return false;
}

RigidPose PerturbCameraPose(const RigidPose& pose, int parameter, double amount) {
    RigidPose result = pose;
    if (parameter < 3) {
        Vec3 delta{};
        if (parameter == 0) delta.x = amount;
        if (parameter == 1) delta.y = amount;
        if (parameter == 2) delta.z = amount;
        result.q = Multiply(SmallAngleQuaternion(delta), pose.q);
    } else {
        if (parameter == 3) result.t.x += amount;
        if (parameter == 4) result.t.y += amount;
        if (parameter == 5) result.t.z += amount;
    }
    return result;
}

bool SolveLinear6(double matrix[6][6], double vector[6], double output[6]) {
    for (int pivot = 0; pivot < 6; ++pivot) {
        int best = pivot;
        for (int row = pivot + 1; row < 6; ++row)
            if (std::abs(matrix[row][pivot]) > std::abs(matrix[best][pivot])) best = row;
        if (std::abs(matrix[best][pivot]) < 1e-12) return false;
        if (best != pivot) {
            for (int col = pivot; col < 6; ++col) std::swap(matrix[pivot][col], matrix[best][col]);
            std::swap(vector[pivot], vector[best]);
        }
        const double diagonal = matrix[pivot][pivot];
        for (int col = pivot; col < 6; ++col) matrix[pivot][col] /= diagonal;
        vector[pivot] /= diagonal;
        for (int row = 0; row < 6; ++row) {
            if (row == pivot) continue;
            const double factor = matrix[row][pivot];
            for (int col = pivot; col < 6; ++col) matrix[row][col] -= factor * matrix[pivot][col];
            vector[row] -= factor * vector[pivot];
        }
    }
    for (int index = 0; index < 6; ++index) output[index] = vector[index];
    return true;
}

bool RefineBoardPose(
    const std::vector<Correspondence>& correspondences,
    double fx, double fy, double cx, double cy, const Distortion& distortion,
    RigidPose* pose,
    double* rms,
    double* maximumResidual,
    std::string* failureReason,
    bool logDetails) {
    if (correspondences.size() < 4)
        return ReportRefineFailure(failureReason, "CORRESPONDENCE_COUNT_LT_4", __func__, __LINE__, logDetails);

    // residual = observed - projected and J = d(residual) / d(pose).  The
    // linearized least-squares system is J * delta = -residual, so the normal
    // equations use -(J^T * residual) on the right-hand side.
    const RigidPose initialPose = *pose;
    ReprojectionStatistics current = EvaluateReprojection(
        *pose, correspondences, fx, fy, cx, cy, distortion, false);
    if (!current.valid)
        return ReportRefineFailure(failureReason, current.failureReason.c_str(), __func__, __LINE__, logDetails);

    RigidPose bestPose = *pose;
    ReprojectionStatistics best = current;
    int acceptedSteps = 0;
    int rejectedSteps = 0;
    bool attemptedStep = false;

    for (int iteration = 0; iteration < kMaxRefinementIterations; ++iteration) {
        double hessian[6][6]{};
        double gradient[6]{};
        for (const auto& observation : correspondences) {
            Vec2 projection{};
            if (!Project(*pose, observation.boardPoint, fx, fy, cx, cy, distortion, &projection)) {
                return ReportRefineFailure(
                    failureReason, DescribeProjectionFailure(*pose, observation.boardPoint), __func__, __LINE__, logDetails);
            }
            const double residual[2] = {observation.imagePoint.x - projection.x, observation.imagePoint.y - projection.y};
            const double residualNorm = std::hypot(residual[0], residual[1]);
            const double huberWeight = residualNorm <= 4.0 ? 1.0 : 4.0 / residualNorm;
            double jacobian[2][6]{};
            for (int parameter = 0; parameter < 6; ++parameter) {
                Vec2 shifted{};
                const RigidPose perturbed = PerturbCameraPose(*pose, parameter, kFiniteDifference);
                if (!Project(perturbed, observation.boardPoint, fx, fy, cx, cy, distortion, &shifted)) {
                    return ReportRefineFailure(
                        failureReason, DescribeProjectionFailure(perturbed, observation.boardPoint), __func__, __LINE__, logDetails);
                }
                jacobian[0][parameter] = -(shifted.x - projection.x) / kFiniteDifference;
                jacobian[1][parameter] = -(shifted.y - projection.y) / kFiniteDifference;
            }
            for (int row = 0; row < 6; ++row) {
                for (int col = 0; col < 6; ++col)
                    hessian[row][col] += huberWeight * (jacobian[0][row]*jacobian[0][col] + jacobian[1][row]*jacobian[1][col]);
                gradient[row] += huberWeight * (jacobian[0][row]*residual[0] + jacobian[1][row]*residual[1]);
            }
        }
        for (int diagonal = 0; diagonal < 6; ++diagonal) hessian[diagonal][diagonal] += 1e-5;
        double step[6]{};
        for (double& value : gradient) value = -value;
        if (!SolveLinear6(hessian, gradient, step)) {
            BOARD_LOGW("REFINE_STEP_REJECTED iteration=%d reason=HESSIAN_SINGULAR_OR_ILL_CONDITIONED", iteration);
            ++rejectedSteps;
            break;
        }

        const double rotationStep = std::sqrt(step[0]*step[0] + step[1]*step[1] + step[2]*step[2]);
        const double translationStep = std::sqrt(step[3]*step[3] + step[4]*step[4] + step[5]*step[5]);
        if (logDetails || iteration < 3) {
            BOARD_LOGI(
                "REFINE_ITERATION iteration=%d rmsBefore=%.6f deltaRotation=%.9f deltaTranslation=%.9f",
                iteration, current.rms, rotationStep, translationStep);
        }

        bool accepted = false;
        for (int reduction = 0; reduction < kMaxRefinementStepReductions; ++reduction) {
            attemptedStep = true;
            const double alpha = std::ldexp(1.0, -reduction);
            RigidPose candidate = *pose;
            candidate.q = Normalize(Multiply(SmallAngleQuaternion({alpha*step[0], alpha*step[1], alpha*step[2]}), candidate.q));
            candidate.t = Add(candidate.t, {alpha*step[3], alpha*step[4], alpha*step[5]});
            if (!IsFinite(candidate.q) || !IsFinite(candidate.t)) {
                BOARD_LOGW("REFINE_ITERATION iteration=%d alpha=%.6f rmsBefore=%.6f deltaRotation=%.9f deltaTranslation=%.9f candidateRms=nan minCandidateDepth=nan",
                    iteration, alpha, current.rms, alpha * rotationStep, alpha * translationStep);
                BOARD_LOGW("REFINE_STEP_REJECTED iteration=%d alpha=%.6f reason=NONFINITE_RESULT", iteration, alpha);
                ++rejectedSteps;
                continue;
            }
            const ReprojectionStatistics candidateStatistics = EvaluateReprojection(
                candidate, correspondences, fx, fy, cx, cy, distortion, false);
            if (logDetails || iteration < 3) {
                BOARD_LOGI(
                    "REFINE_ITERATION iteration=%d alpha=%.6f rmsBefore=%.6f deltaRotation=%.9f deltaTranslation=%.9f "
                    "candidateRms=%.6f minCandidateDepth=%.8f",
                    iteration, alpha, current.rms, alpha * rotationStep, alpha * translationStep,
                    candidateStatistics.rms, candidateStatistics.minimumDepth);
            }
            if (!candidateStatistics.valid) {
                BOARD_LOGW(
                    "REFINE_STEP_REJECTED iteration=%d alpha=%.6f reason=%s candidateRms=nan minCandidateDepth=%.8f",
                    iteration, alpha, candidateStatistics.failureReason.c_str(), candidateStatistics.minimumDepth);
                ++rejectedSteps;
                continue;
            }
            if (!std::isfinite(candidateStatistics.rms) || !std::isfinite(candidateStatistics.maximumResidual)) {
                BOARD_LOGW("REFINE_STEP_REJECTED iteration=%d alpha=%.6f reason=NONFINITE_RESULT candidateRms=%.6f minCandidateDepth=%.8f",
                    iteration, alpha, candidateStatistics.rms, candidateStatistics.minimumDepth);
                ++rejectedSteps;
                continue;
            }
            if (candidateStatistics.rms >= current.rms) {
                BOARD_LOGW("REFINE_STEP_REJECTED iteration=%d alpha=%.6f reason=ERROR_INCREASE candidateRms=%.6f minCandidateDepth=%.8f",
                    iteration, alpha, candidateStatistics.rms, candidateStatistics.minimumDepth);
                ++rejectedSteps;
                continue;
            }

            *pose = candidate;
            current = candidateStatistics;
            if (current.rms < best.rms) {
                bestPose = *pose;
                best = current;
            }
            ++acceptedSteps;
            accepted = true;
            BOARD_LOGI("REFINE_STEP_ACCEPTED iteration=%d alpha=%.6f candidateRms=%.6f minCandidateDepth=%.8f",
                iteration, alpha, current.rms, current.minimumDepth);
            break;
        }
        if (!accepted) break;
        if (rotationStep + translationStep < 1e-6) break;
    }

    *pose = bestPose;
    *rms = best.rms;
    *maximumResidual = best.maximumResidual;
    if (acceptedSteps == 0 && attemptedStep) {
        BOARD_LOGW(
            "BOARD_REFINE_NO_IMPROVEMENT_USING_INITIAL_POSE initialRms=%.6f finalRms=%.6f rejectedStepCount=%d",
            EvaluateReprojection(initialPose, correspondences, fx, fy, cx, cy, distortion, false).rms,
            best.rms, rejectedSteps);
    }
    BOARD_LOGI(
        "BOARD_REFINE_SUMMARY initialRms=%.6f customFinalRms=%.6f minDepthBefore=%.8f minDepthAfter=%.8f "
        "acceptedStepCount=%d rejectedStepCount=%d convention=RESIDUAL_OBSERVED_MINUS_PROJECTED_J_EQUALS_DRESIDUAL "
        "normalEquation=JtJ_DELTA_EQUALS_MINUS_JtR finalSolver=CUSTOM_GAUSS_NEWTON_BACKTRACKING",
        EvaluateReprojection(initialPose, correspondences, fx, fy, cx, cy, distortion, false).rms,
        best.rms,
        EvaluateReprojection(initialPose, correspondences, fx, fy, cx, cy, distortion, false).minimumDepth,
        best.minimumDepth, acceptedSteps, rejectedSteps);
    return true;
}

#include "dji_board_pose_multistart.h"

#ifndef DJI_BOARD_POSE_REPLAY_TEST
thread_local int64_t g_multiStartFrameTimestampNs = 0;
thread_local int64_t g_multiStartSequence = 0;
thread_local int64_t g_multiStartGeneration = 0;

void ReleaseDetectorLocked() {
    if (g_detector) { apriltag_detector_destroy(g_detector); g_detector = nullptr; }
    if (g_family) { tagStandard41h12_destroy(g_family); g_family = nullptr; }
    g_luma.clear();
    g_refinementDiagnosticFrame = 0;
}

bool EnsureDetectorLocked() {
    if (g_detector && g_family) return true;
    ReleaseDetectorLocked();
    g_detector = apriltag_detector_create();
    g_family = tagStandard41h12_create();
    if (!g_detector || !g_family) { ReleaseDetectorLocked(); return false; }
    apriltag_detector_add_family_bits(g_detector, g_family, 1);
    g_detector->nthreads = kDetectorThreads;
    g_detector->quad_decimate = 2.0f;
    g_detector->decode_sharpening = 0.25;
    return true;
}

bool ParseLayout(JNIEnv* env, jfloatArray layoutArray, std::unordered_map<int, MarkerDefinition>* definitions, std::string* reason) {
    definitions->clear();
    if (!layoutArray) {
        *reason = "LAYOUT_NULL";
        return false;
    }
    const jsize length = env->GetArrayLength(layoutArray);
    if (env->ExceptionCheck()) {
        *reason = "LAYOUT_LENGTH_JNI_EXCEPTION";
        return false;
    }
    if (length <= 0 || length % kLayoutStride != 0) {
        *reason = "LAYOUT_LENGTH_INVALID";
        return false;
    }
    jfloat* values = env->GetFloatArrayElements(layoutArray, nullptr);
    if (!values) {
        *reason = "LAYOUT_ELEMENTS_UNAVAILABLE";
        return false;
    }
    bool valid = true;
    for (int offset = 0; offset < length; offset += kLayoutStride) {
        const double id = values[offset];
        const double size = values[offset + 1];
        if (!std::isfinite(id) || std::floor(id) != id || !std::isfinite(size) || size <= 0.0) {
            *reason = "LAYOUT_ID_OR_SIZE_INVALID";
            valid = false;
            break;
        }
        for (int index = offset + 2; index < offset + kLayoutStride; ++index) {
            if (!std::isfinite(values[index])) {
                *reason = "LAYOUT_NONFINITE_VALUE";
                valid = false;
                break;
            }
        }
        if (!valid) break;
        MarkerDefinition definition{};
        definition.id = static_cast<int>(id);
        definition.aprilTagDetectionSizeMeters = size;
        definition.boardFromMarker.t = {values[offset+2], values[offset+3], values[offset+4]};
        definition.boardFromMarker.q = Normalize({values[offset+5], values[offset+6], values[offset+7], values[offset+8]});
        if (definitions->find(definition.id) != definitions->end()) {
            *reason = "LAYOUT_DUPLICATE_MARKER_ID";
            valid = false;
            break;
        }
        definitions->emplace(definition.id, definition);
    }
    env->ReleaseFloatArrayElements(layoutArray, values, JNI_ABORT);
    if (env->ExceptionCheck()) {
        *reason = "LAYOUT_RELEASE_JNI_EXCEPTION";
        return false;
    }
    if (!valid || definitions->empty()) {
        if (reason->empty()) *reason = "LAYOUT_EMPTY";
        return false;
    }
    return true;
}

bool ParseDistortion(JNIEnv* env, jfloatArray distortionArray, Distortion* distortion, std::string* reason) {
    *distortion = {};
    if (!distortionArray) {
        *reason = "DISTORTION_NULL";
        return false;
    }
    const jsize length = env->GetArrayLength(distortionArray);
    if (env->ExceptionCheck()) {
        *reason = "DISTORTION_LENGTH_JNI_EXCEPTION";
        return false;
    }
    if (length != 5) {
        *reason = "DISTORTION_LENGTH_INVALID";
        return false;
    }
    jfloat values[5]{};
    env->GetFloatArrayRegion(distortionArray, 0, 5, values);
    if (env->ExceptionCheck()) {
        *reason = "DISTORTION_READ_JNI_EXCEPTION";
        return false;
    }
    for (const float value : values) {
        if (!std::isfinite(value)) {
            *reason = "DISTORTION_NONFINITE_VALUE";
            return false;
        }
    }
    *distortion = {values[0], values[1], values[2], values[3], values[4]};
    return true;
}

std::array<Vec3, 4> BoardMarkerCorners(const MarkerDefinition& marker) {
    const double half = marker.aprilTagDetectionSizeMeters * 0.5;
    // AprilTag's pose source defines p0=(-s,+s), p1=(+s,+s),
    // p2=(+s,-s), p3=(-s,-s). In pixel space this is +X right and +Y
    // print-down; ReferenceBoardDefinition supplies the rigid board-basis map.
    const std::array<Vec3, 4> markerCorners{{{-half, half, 0}, {half, half, 0}, {half, -half, 0}, {-half, -half, 0}}};
    std::array<Vec3, 4> boardCorners{};
    for (int index = 0; index < 4; ++index)
        boardCorners[index] = Add(marker.boardFromMarker.t, Rotate(marker.boardFromMarker.q, markerCorners[index]));
    return boardCorners;
}

RigidPose PoseFromOfficialTagPose(const apriltag_pose_t& pose) {
    return {QuaternionFromRotationMatrix(pose.R), {MATD_EL(pose.t,0,0), MATD_EL(pose.t,1,0), MATD_EL(pose.t,2,0)}};
}

jfloatArray CreateFailureResult(JNIEnv* env, FailureCode code, const char* reason) {
    CaptureEvidence([&] { g_captureEvidence += ",\"nativeStatus\":-" + std::to_string(static_cast<int>(code)) +
        ",\"failureReason\":\"" + reason + "\"}"; });
    BOARD_LOGW("DJI_BOARD_FRAME_REJECTED reason=%s code=%d", reason, static_cast<int>(code));
    jfloatArray result = env->NewFloatArray(kHeaderLength);
    if (!result) return nullptr;
    std::array<jfloat, kHeaderLength> values{};
    values[0] = -static_cast<jfloat>(code);
    env->SetFloatArrayRegion(result, 0, kHeaderLength, values.data());
    return result;
}
} // namespace

namespace {
jfloatArray DetectBoardLuma(
    JNIEnv* env,
    jbyteArray lumaArray,
    jint width,
    jint height,
    jfloat fx,
    jfloat fy,
    jfloat cx,
    jfloat cy,
    jfloatArray distortionArray,
    jfloatArray layoutArray) {
    const int64_t pixelCount = static_cast<int64_t>(width) * static_cast<int64_t>(height);
    if (!lumaArray || width <= 0 || height <= 0 || pixelCount <= 0 || pixelCount > kMaxFramePixels)
        return CreateFailureResult(env, FailureCode::InvalidFrame, "FRAME_DIMENSIONS_INVALID");

    std::unordered_map<int, MarkerDefinition> layout;
    std::string layoutReason;
    if (!ParseLayout(env, layoutArray, &layout, &layoutReason))
        return CreateFailureResult(env, FailureCode::InvalidLayout, layoutReason.c_str());

    Distortion distortion{};
    std::string distortionReason;
    if (!ParseDistortion(env, distortionArray, &distortion, &distortionReason))
        return CreateFailureResult(env, FailureCode::InvalidDistortion, distortionReason.c_str());

    const bool calibrationValid = std::isfinite(fx) && std::isfinite(fy) && std::isfinite(cx) && std::isfinite(cy) && fx > 0.0f && fy > 0.0f;
    if (!calibrationValid)
        return CreateFailureResult(env, FailureCode::InvalidCalibration, "CAMERA_INTRINSICS_INVALID");

    const jsize lumaLength = env->GetArrayLength(lumaArray);
    if (env->ExceptionCheck())
        return CreateFailureResult(env, FailureCode::JniException, "LUMA_LENGTH_JNI_EXCEPTION");
    if (static_cast<int64_t>(lumaLength) != pixelCount)
        return CreateFailureResult(env, FailureCode::LumaLengthMismatch, "LUMA_LENGTH_MISMATCH");

    BOARD_LOGI("BOARD_STAGE_01_FRAME_RECEIVED width=%d height=%d lumaBytes=%d", width, height, lumaLength);

    std::lock_guard<std::mutex> lock(g_detectorMutex);
    if (!EnsureDetectorLocked())
        return CreateFailureResult(env, FailureCode::DetectorUnavailable, "APRILTAG_DETECTOR_UNAVAILABLE");
    const uint64_t refinementFrame = ++g_refinementDiagnosticFrame;
    // Full corner/transform dumps are useful on first acquisition and periodically thereafter.
    const bool logRefinementDetails = refinementFrame <= 3 || refinementFrame % 10 == 0;
    if (logRefinementDetails) LogBoardLayoutDefinitions(layout);
    g_luma.resize(static_cast<size_t>(pixelCount));
    env->GetByteArrayRegion(lumaArray, 0, static_cast<jsize>(pixelCount), reinterpret_cast<jbyte*>(g_luma.data()));
    if (env->ExceptionCheck())
        return CreateFailureResult(env, FailureCode::JniException, "LUMA_COPY_JNI_EXCEPTION");
    image_u8_t image{width, height, width, g_luma.data()};
    zarray_t* detections = apriltag_detector_detect(g_detector, &image);
    if (!detections)
        return CreateFailureResult(env, FailureCode::DetectionUnavailable, "APRILTAG_DETECTION_UNAVAILABLE");

    std::vector<Correspondence> correspondences;
    std::vector<VisibleMarker> visible;
    std::unordered_map<int, apriltag_detection_t*> recognizedDetections;
    RigidPose initialCameraFromBoard{{0,0,0,1},{0,0,0}};
    bool hasInitialPose = false;
    apriltag_detection_t* initialDetection = nullptr;
    const MarkerDefinition* initialMarker = nullptr;
    const int count = zarray_size(detections);
    CaptureEvidence([&] {
        g_captureEvidence += ",\"decodedMarkersBeforeBoardValidation\":[";
        for (int index = 0; index < count; ++index) {
            if (index) g_captureEvidence += ",";
            apriltag_detection_t* detection = nullptr;
            zarray_get(detections, index, &detection);
            if (!detection) { g_captureEvidence += "null"; continue; }
            g_captureEvidence += "{\"id\":" + std::to_string(detection->id) + ",\"decisionMargin\":" + CaptureNumber(detection->decision_margin) + ",\"cornersXY\":[";
            for (int corner = 0; corner < 4; ++corner) {
                if (corner) g_captureEvidence += ",";
                g_captureEvidence += CaptureNumber(detection->p[corner][0]) + "," + CaptureNumber(detection->p[corner][1]);
            }
            g_captureEvidence += "]}";
        }
        g_captureEvidence += "]";
    });
    BOARD_LOGI("BOARD_STAGE_02_TAG_DETECTION_COMPLETE markerCount=%d", count);
    if (count <= 0) {
        apriltag_detections_destroy(detections);
        return CreateFailureResult(env, FailureCode::NoMarkers, "MARKER_COUNT_ZERO");
    }

    int unknownMarkerCount = 0;
    for (int index = 0; index < count; ++index) {
        apriltag_detection_t* detection = nullptr;
        zarray_get(detections, index, &detection);
        if (!detection) {
            apriltag_detections_destroy(detections);
            return CreateFailureResult(env, FailureCode::DetectionUnavailable, "NULL_APRILTAG_DETECTION");
        }
        const auto definitionIt = layout.find(detection->id);
        if (definitionIt == layout.end()) {
            unknownMarkerCount++;
            BOARD_LOGW("DJI_BOARD_MARKER_ID_REJECTED id=%d reason=NOT_IN_REFERENCE_BOARD_DEFINITION", detection->id);
            continue;
        }
        const MarkerDefinition& marker = definitionIt->second;
        recognizedDetections[marker.id] = detection;
        VisibleMarker observed{};
        observed.id = marker.id;
        observed.decisionMargin = detection->decision_margin;
        const auto boardCorners = BoardMarkerCorners(marker);
        for (int corner = 0; corner < 4; ++corner) {
            observed.detectedCorners[corner] = {detection->p[corner][0], detection->p[corner][1]};
            if (!std::isfinite(observed.detectedCorners[corner].x) || !std::isfinite(observed.detectedCorners[corner].y)) {
                apriltag_detections_destroy(detections);
                return CreateFailureResult(env, FailureCode::InvalidCorners, "DETECTED_CORNER_NONFINITE");
            }
            correspondences.push_back({marker.id, corner, boardCorners[corner], observed.detectedCorners[corner]});
            if (logRefinementDetails) {
                BOARD_LOGI(
                    "MARKER_CORNER_MAPPING id=%d detectorCorner=%d boardCorner=%d "
                    "detectedPixel=[%.3f,%.3f] boardPoint=[%.8f,%.8f,%.8f]",
                    marker.id, corner, corner,
                    observed.detectedCorners[corner].x, observed.detectedCorners[corner].y,
                    boardCorners[corner].x, boardCorners[corner].y, boardCorners[corner].z);
            }
        }
        if (!initialDetection) {
            initialDetection = detection;
            initialMarker = &marker;
        }
        visible.push_back(observed);
    }

    BOARD_LOGI("BOARD_STAGE_03_MARKER_IDS_VALIDATED detected=%d configured=%d unknown=%d", count, static_cast<int>(visible.size()), unknownMarkerCount);
    if (unknownMarkerCount > 0 || visible.empty()) {
        apriltag_detections_destroy(detections);
        return CreateFailureResult(env, FailureCode::UnknownMarkerId, "MARKER_ID_NOT_IN_REFERENCE_BOARD_DEFINITION");
    }

    const size_t objectPointCount = correspondences.size();
    const size_t imagePointCount = correspondences.size();
    BOARD_LOGI("BOARD_STAGE_04_CORRESPONDENCES_BUILT objectPoints=%zu imagePoints=%zu", objectPointCount, imagePointCount);
    if (objectPointCount != imagePointCount || objectPointCount < 4 || objectPointCount % 4 != 0) {
        apriltag_detections_destroy(detections);
        return CreateFailureResult(env, FailureCode::CorrespondenceMismatch, "CORRESPONDENCE_COUNT_INVALID");
    }

    BOARD_LOGI("BOARD_STAGE_05_CALIBRATION_VALIDATED fx=%.4f fy=%.4f cx=%.4f cy=%.4f distortion=[%.6f,%.6f,%.6f,%.6f,%.6f]", fx, fy, cx, cy, distortion.k1, distortion.k2, distortion.p1, distortion.p2, distortion.k3);

    // Each tag independently produces T_camera_marker. Converting each to the
    // shared board frame makes layout/corner convention errors observable before refinement.
    std::vector<IndependentMarkerPose> independentMarkerPoses;
    for (const auto& observed : visible) {
        const auto detectionIt = recognizedDetections.find(observed.id);
        const auto markerIt = layout.find(observed.id);
        if (detectionIt == recognizedDetections.end() || markerIt == layout.end()) continue;
        const apriltag_detection_t* detection = detectionIt->second;
        const MarkerDefinition& marker = markerIt->second;
        apriltag_detection_t undistortedDetection = *detection;
        for (int corner = 0; corner < 4; ++corner) {
            const Vec2 point = UndistortPixel(
                {detection->p[corner][0], detection->p[corner][1]}, fx, fy, cx, cy, distortion);
            undistortedDetection.p[corner][0] = point.x;
            undistortedDetection.p[corner][1] = point.y;
        }
        apriltag_detection_info_t info{&undistortedDetection, marker.aprilTagDetectionSizeMeters, fx, fy, cx, cy};
        apriltag_pose_t tagPose{};
        const double tagPoseError = estimate_tag_pose(&info, &tagPose);
        if (tagPose.R && tagPose.t) {
            const RigidPose cameraFromMarker = PoseFromOfficialTagPose(tagPose);
            const ImageTagGeometry imageGeometry = CalculateImageTagGeometry(observed.detectedCorners);
            independentMarkerPoses.push_back({
                marker.id,
                tagPoseError,
                cameraFromMarker,
                Compose(cameraFromMarker, Invert(marker.boardFromMarker)),
                marker,
                imageGeometry.center,
                imageGeometry.meanSideLength,
            });
        } else {
            BOARD_LOGW("BOARD_INDEPENDENT_MARKER_POSE_FAILED id=%d reason=OFFICIAL_APRILTAG_POSE_UNAVAILABLE", marker.id);
        }
        if (tagPose.R) matd_destroy(tagPose.R);
        if (tagPose.t) matd_destroy(tagPose.t);
    }
    if (logRefinementDetails) LogIndependentMarkerPoseComparison(independentMarkerPoses);
    CaptureEvidence([&] {
        g_captureEvidence += ",\"detectedMarkers\":[";
        bool separator = false;
        for (const auto& marker : visible) {
            if (separator) g_captureEvidence += ","; separator = true;
            g_captureEvidence += "{\"id\":" + std::to_string(marker.id) + ",\"decisionMargin\":" + CaptureNumber(marker.decisionMargin) + ",\"cornersXY\":[";
            for (int corner = 0; corner < 4; ++corner) {
                if (corner) g_captureEvidence += ",";
                g_captureEvidence += CaptureNumber(marker.detectedCorners[corner].x) + "," + CaptureNumber(marker.detectedCorners[corner].y);
            }
            g_captureEvidence += "]}";
        }
        g_captureEvidence += "],\"independentSelectedTagPoses\":[";
        separator = false;
        for (const auto& pose : independentMarkerPoses) {
            if (separator) g_captureEvidence += ","; separator = true;
            g_captureEvidence += "{\"markerId\":" + std::to_string(pose.id) + ",\"selectedTagPoseError\":" + CaptureNumber(pose.tagPoseError) +
                ",\"cameraFromMarker\":" + CapturePose(pose.cameraFromMarker) + ",\"cameraFromBoard\":" + CapturePose(pose.cameraFromBoard) + "}";
        }
        g_captureEvidence += "]";
    });

    // The official pose initializer accepts only K. Feed it corners undistorted
    // into the same detector pixel frame before joint refinement.
    if (initialDetection && initialMarker) {
        apriltag_detection_t undistortedDetection = *initialDetection;
        for (int corner = 0; corner < 4; ++corner) {
            const Vec2 point = UndistortPixel({initialDetection->p[corner][0], initialDetection->p[corner][1]}, fx, fy, cx, cy, distortion);
            undistortedDetection.p[corner][0] = point.x;
            undistortedDetection.p[corner][1] = point.y;
        }
        apriltag_detection_info_t info{&undistortedDetection, initialMarker->aprilTagDetectionSizeMeters, fx, fy, cx, cy};
        apriltag_pose_t tagPose{};
        const double tagPoseError = estimate_tag_pose(&info, &tagPose);
        if (tagPose.R && tagPose.t) {
            // T_camera_board = T_camera_marker * T_marker_board.
            const RigidPose cameraFromMarker = PoseFromOfficialTagPose(tagPose);
            const RigidPose markerFromBoard = Invert(initialMarker->boardFromMarker);
            initialCameraFromBoard = Compose(cameraFromMarker, markerFromBoard);
            hasInitialPose = true;
            CaptureEvidence([&] { g_captureEvidence += ",\"selectedInitializerMarkerId\":" + std::to_string(initialMarker->id) +
                ",\"initializerPose\":" + CapturePose(initialCameraFromBoard) + ",\"initializerTagPoseError\":" + CaptureNumber(tagPoseError); });
            BOARD_LOGI("DJI_BOARD_INITIAL_TAG_POSE error=%.6f markerId=%d", tagPoseError, initialMarker->id);
            if (logRefinementDetails) {
                BOARD_LOGI("BOARD_TRANSFORM_COMPOSITION equation=T_camera_board=T_camera_marker*T_marker_board markerId=%d", initialMarker->id);
                LogPose("T_camera_marker", cameraFromMarker);
                LogPose("T_board_marker", initialMarker->boardFromMarker);
                LogPose("T_marker_board", markerFromBoard);
                LogPose("T_camera_board_initial", initialCameraFromBoard);
            }
        }
        if (tagPose.R) matd_destroy(tagPose.R);
        if (tagPose.t) matd_destroy(tagPose.t);
    }

    RigidPose cameraFromBoard = initialCameraFromBoard;
    double rms = std::numeric_limits<double>::quiet_NaN();
    double maximumResidual = std::numeric_limits<double>::quiet_NaN();
    BOARD_LOGI("BOARD_STAGE_06_PNP_BEGIN correspondences=%zu hasInitialPose=%d", correspondences.size(), hasInitialPose ? 1 : 0);
    if (!hasInitialPose && !(DJI_APRILTAG_FOUR_TAG_MULTISTART_EXPERIMENT && visible.size() == 4)) {
        apriltag_detections_destroy(detections);
        return CreateFailureResult(env, FailureCode::PoseInitializationFailed, "OFFICIAL_APRILTAG_POSE_INITIALIZATION_FAILED");
    }
    const ReprojectionStatistics initialStatistics = LogInitialPoseReprojection(
        initialCameraFromBoard, correspondences, fx, fy, cx, cy, distortion, logRefinementDetails);
    const bool initialPoseFinite = IsFinite(initialCameraFromBoard.q) && IsFinite(initialCameraFromBoard.t);
    CaptureEvidence([&] { g_captureEvidence += ",\"initialPoseFinite\":" + std::string(initialPoseFinite ? "true" : "false") +
        ",\"initialRms\":" + CaptureNumber(initialStatistics.rms) + ",\"initialMinDepth\":" + CaptureNumber(initialStatistics.minimumDepth) +
        ",\"initialMaxDepth\":" + CaptureNumber(initialStatistics.maximumDepth); });
    if (logRefinementDetails) {
        BOARD_LOGI(
            "BOARD_REFINE_ALGORITHM name=CUSTOM_GAUSS_NEWTON finiteDifference=%.8f maxIterations=%d "
            "robustLoss=HUBER huberPixels=4.0 opencvLinked=false",
            kFiniteDifference, kMaxRefinementIterations);
        BOARD_LOGI(
            "BOARD_REFINE_INPUT objectPointCount=%zu imagePointCount=%zu objectPointUnits=METERS "
            "cameraMatrixType=CUSTOM_DOUBLE_3X3 cameraMatrixSize=3x3 distortionType=CUSTOM_DOUBLE_BROWN_CONRADY "
            "distortionSize=5 poseRotationType=QUATERNION_NOT_RVEC poseTranslationType=VEC3_NOT_TVEC "
            "initialQuaternion=[%.8f,%.8f,%.8f,%.8f] initialTvec=[%.8f,%.8f,%.8f] initialPoseFinite=%d "
            "cameraMatrixValid=1 distortionValid=1",
            correspondences.size(), correspondences.size(),
            initialCameraFromBoard.q.x, initialCameraFromBoard.q.y,
            initialCameraFromBoard.q.z, initialCameraFromBoard.q.w,
            initialCameraFromBoard.t.x, initialCameraFromBoard.t.y, initialCameraFromBoard.t.z,
            initialPoseFinite ? 1 : 0);
        BOARD_LOGI("BOARD_PNP_CANDIDATE_B algorithm=SOLVEPNP status=UNAVAILABLE reason=OPENCV_NOT_LINKED");
        BOARD_LOGI("BOARD_PNP_CANDIDATE_C algorithm=SOLVEPNP_IPPE status=UNAVAILABLE reason=OPENCV_NOT_LINKED");
        BOARD_LOGI("OPENCV_LM_REFINE_RMS=UNAVAILABLE reason=OPENCV_NOT_LINKED");
        BOARD_LOGI("OPENCV_VVS_REFINE_RMS=UNAVAILABLE reason=OPENCV_NOT_LINKED");
    }
    BOARD_LOGI(
        "BOARD_REFINE_BEGIN initialRvec=NOT_USED_QUATERNION initialTvec=[%.8f,%.8f,%.8f] "
        "initialRms=%.6f initialMaxResidual=%.6f initialMinDepth=%.8f initialMaxDepth=%.8f",
        initialCameraFromBoard.t.x, initialCameraFromBoard.t.y, initialCameraFromBoard.t.z,
        initialStatistics.rms, initialStatistics.maximumResidual,
        initialStatistics.minimumDepth, initialStatistics.maximumDepth);
    std::string refineFailureReason;
#if DJI_APRILTAG_FOUR_TAG_MULTISTART_EXPERIMENT
    MultiStartDiagnostics selectionDiagnostics;
    const bool poseValid = RefineBoardPoseSelection(
        DJI_APRILTAG_FOUR_TAG_MULTISTART_EXPERIMENT != 0, correspondences, visible,
        independentMarkerPoses, initialMarker ? initialMarker->id : -1, hasInitialPose, initialCameraFromBoard,
        fx, fy, cx, cy, distortion, &cameraFromBoard, &rms, &maximumResidual,
        &refineFailureReason, logRefinementDetails, &selectionDiagnostics);
    const auto compact = FormatMultiStartDiagnostics(selectionDiagnostics, visible,
        g_multiStartFrameTimestampNs, g_multiStartGeneration, g_multiStartSequence);
    BOARD_LOGI("BOARD_FOUR_TAG_MULTISTART %s", compact.data.data());
    CaptureEvidence([&] { g_captureEvidence += ",\"fourTagMultiStart\":" + std::string(compact.data.data()); });
#else
    const bool poseValid = RefineBoardPose(
        correspondences, fx, fy, cx, cy, distortion, &cameraFromBoard, &rms, &maximumResidual,
        &refineFailureReason, logRefinementDetails);
#endif
    if (!poseValid || !std::isfinite(rms) || !std::isfinite(maximumResidual)) {
        CaptureEvidence([&] { g_captureEvidence += ",\"refinementFailureDetail\":\"" + refineFailureReason + "\""; });
        const char* exactReason = !poseValid ? refineFailureReason.c_str() :
            (!std::isfinite(rms) ? "FINAL_RMS_NONFINITE" : "FINAL_MAX_RESIDUAL_NONFINITE");
        if (poseValid) {
            BOARD_LOGW(
                "BOARD_REFINE_FAILURE code=13 exactReason=%s sourceFunction=%s sourceLine=%d",
                exactReason, __func__, __LINE__);
        }
        const char* diagnosis = !initialStatistics.valid ||
                (!std::isfinite(initialStatistics.rms) || initialStatistics.rms > 10.0)
            ? "INITIAL_POSE_GEOMETRY_ERROR"
            : "REFINEMENT_NUMERIC_FAILURE";
        BOARD_LOGW(
            "BOARD_REFINE_FAILED exactReason=%s initialRms=%.6f initialMaxResidual=%.6f "
            "finalRms=%.6f finalMaxResidual=%.6f",
            exactReason, initialStatistics.rms, initialStatistics.maximumResidual, rms, maximumResidual);
        BOARD_LOGW("BOARD_REFINE_DIAGNOSIS=%s", diagnosis);
        apriltag_detections_destroy(detections);
        return CreateFailureResult(env, FailureCode::PoseRefinementFailed, "BOARD_POSE_REFINEMENT_FAILED");
    }
    BOARD_LOGI(
        "BOARD_REFINE_SUCCESS finalRvec=NOT_USED_QUATERNION finalQuaternion=[%.8f,%.8f,%.8f,%.8f] "
        "finalTvec=[%.8f,%.8f,%.8f] finalRms=%.6f maxResidual=%.6f",
        cameraFromBoard.q.x, cameraFromBoard.q.y, cameraFromBoard.q.z, cameraFromBoard.q.w,
        cameraFromBoard.t.x, cameraFromBoard.t.y, cameraFromBoard.t.z, rms, maximumResidual);
    BOARD_LOGI("CUSTOM_REFINE_RMS=%.6f", rms);
    CaptureEvidence([&] { g_captureEvidence += ",\"nativeStatus\":2,\"failureReason\":null,\"finalRefinedPose\":" + CapturePose(cameraFromBoard) +
        ",\"finalRms\":" + CaptureNumber(rms) + ",\"finalMaxResidual\":" + CaptureNumber(maximumResidual) + "}"; });
    BOARD_LOGI("BOARD_STAGE_07_PNP_SUCCESS position=[%.5f,%.5f,%.5f] rms=%.4f", cameraFromBoard.t.x, cameraFromBoard.t.y, cameraFromBoard.t.z, rms);
    if (poseValid) {
        for (auto& marker : visible) {
            const auto definition = layout.find(marker.id);
            const auto boardCorners = BoardMarkerCorners(definition->second);
            for (int corner = 0; corner < 4; ++corner) Project(cameraFromBoard, boardCorners[corner], fx, fy, cx, cy, distortion, &marker.projectedCorners[corner]);
        }
    }
    BOARD_LOGI("BOARD_STAGE_08_REPROJECTION_COMPLETE rms=%.4f maxResidual=%.4f", rms, maximumResidual);
    apriltag_detections_destroy(detections);

    const int length = kHeaderLength + static_cast<int>(visible.size()) * kMarkerOutputLength;
    jfloatArray result = env->NewFloatArray(length);
    if (!result) return nullptr;
    std::vector<jfloat> output(length, 0.0f);
    output[0] = poseValid ? 2.0f : (visible.empty() ? 0.0f : 1.0f);
    output[1] = static_cast<jfloat>(visible.size());
    output[2] = static_cast<jfloat>(correspondences.size());
    output[3] = static_cast<jfloat>(rms);
    output[4] = static_cast<jfloat>(maximumResidual);
    output[5] = static_cast<jfloat>(cameraFromBoard.t.x);
    output[6] = static_cast<jfloat>(cameraFromBoard.t.y);
    output[7] = static_cast<jfloat>(cameraFromBoard.t.z);
    const std::array<Vec3, 3> columns{{Rotate(cameraFromBoard.q, {1,0,0}), Rotate(cameraFromBoard.q, {0,1,0}), Rotate(cameraFromBoard.q, {0,0,1})}};
    output[8] = columns[0].x; output[9] = columns[1].x; output[10] = columns[2].x;
    output[11] = columns[0].y; output[12] = columns[1].y; output[13] = columns[2].y;
    output[14] = columns[0].z; output[15] = columns[1].z; output[16] = columns[2].z;
    int offset = kHeaderLength;
    for (const auto& marker : visible) {
        output[offset++] = static_cast<jfloat>(marker.id);
        output[offset++] = static_cast<jfloat>(marker.decisionMargin);
        for (const auto& corner : marker.detectedCorners) { output[offset++] = static_cast<jfloat>(corner.x); output[offset++] = static_cast<jfloat>(corner.y); }
        for (const auto& corner : marker.projectedCorners) { output[offset++] = static_cast<jfloat>(corner.x); output[offset++] = static_cast<jfloat>(corner.y); }
    }
    env->SetFloatArrayRegion(result, 0, length, output.data());
    return result;
}
} // namespace

extern "C" JNIEXPORT jfloatArray JNICALL
Java_com_sok9hu_djibridge_DjiBoardVisionNative_detectBoardLuma(
    JNIEnv* env,
    jclass,
    jbyteArray lumaArray,
    jint width,
    jint height,
    jfloat fx,
    jfloat fy,
    jfloat cx,
    jfloat cy,
    jfloatArray distortionArray,
    jfloatArray layoutArray) {
    CaptureEvidence([] { g_captureEvidence = "{\"alternativePlanarSolutions\":null,\"alternativeSolutionErrors\":null,\"alternativeEvidenceStatus\":\"OFFLINE_REPLAY_REQUIRED\",\"transformConvention\":\"T_camera_board maps Board to Camera\""; });
    try {
        return DetectBoardLuma(env, lumaArray, width, height, fx, fy, cx, cy, distortionArray, layoutArray);
    } catch (const std::exception& error) {
        BOARD_LOGE("DJI_BOARD_NATIVE_EXCEPTION type=std::exception message=%s", error.what());
        return CreateFailureResult(env, FailureCode::CppException, "CPP_EXCEPTION");
    } catch (...) {
        BOARD_LOGE("DJI_BOARD_NATIVE_EXCEPTION type=unknown");
        return CreateFailureResult(env, FailureCode::UnknownException, "UNKNOWN_CPP_EXCEPTION");
    }
}

extern "C" JNIEXPORT void JNICALL
Java_com_sok9hu_djibridge_DjiBoardVisionNative_setCaptureEvidenceEnabled(JNIEnv*, jclass, jboolean enabled) {
    g_captureEvidenceEnabled = enabled == JNI_TRUE;
    g_captureEvidence.clear();
}

extern "C" JNIEXPORT jstring JNICALL
Java_com_sok9hu_djibridge_DjiBoardVisionNative_getCaptureEvidenceJson(JNIEnv* env, jclass) {
    return env->NewStringUTF(g_captureEvidence.empty() ? "null" : g_captureEvidence.c_str());
}

extern "C" JNIEXPORT void JNICALL
Java_com_sok9hu_djibridge_DjiBoardVisionNative_releaseDetector(JNIEnv*, jclass) {
    std::lock_guard<std::mutex> lock(g_detectorMutex);
    ReleaseDetectorLocked();
}

extern "C" JNIEXPORT jboolean JNICALL
Java_com_sok9hu_djibridge_DjiBoardVisionNative_isFourTagMultiStartExperimentEnabled(JNIEnv*, jclass) {
    return DJI_APRILTAG_FOUR_TAG_MULTISTART_EXPERIMENT ? JNI_TRUE : JNI_FALSE;
}

extern "C" JNIEXPORT void JNICALL
Java_com_sok9hu_djibridge_DjiBoardVisionNative_setPoseSelectionContext(
    JNIEnv*, jclass, jlong frameTimestampNs, jlong generation, jlong sequence) {
    g_multiStartFrameTimestampNs = frameTimestampNs;
    g_multiStartGeneration = generation;
    g_multiStartSequence = sequence;
}
#else
} // namespace
#endif
