#include <jni.h>
#include <android/log.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <exception>
#include <limits>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

extern "C" {
#include "apriltag.h"
#include "apriltag_pose.h"
#include "common/image_u8.h"
#include "tagStandard41h12.h"
}

#define BOARD_LOGI(...) __android_log_print(ANDROID_LOG_INFO, "DjiBoardVision", __VA_ARGS__)
#define BOARD_LOGW(...) __android_log_print(ANDROID_LOG_WARN, "DjiBoardVision", __VA_ARGS__)
#define BOARD_LOGE(...) __android_log_print(ANDROID_LOG_ERROR, "DjiBoardVision", __VA_ARGS__)

namespace {
constexpr int kLayoutStride = 9; // id, size, board position xyz, board rotation xyzw
constexpr int kHeaderLength = 17;
constexpr int kMarkerOutputLength = 18; // id, margin, detected corners, projected corners
constexpr int kDetectorThreads = 2;
constexpr double kFiniteDifference = 1e-5;
constexpr int kMaxRefinementIterations = 20;
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
    double sizeMeters;
    RigidPose boardFromMarker;
};

struct Correspondence {
    Vec3 boardPoint;
    Vec2 imagePoint;
};

struct VisibleMarker {
    int id;
    double decisionMargin;
    std::array<Vec2, 4> detectedCorners;
    std::array<Vec2, 4> projectedCorners;
};

std::mutex g_detectorMutex;
apriltag_detector_t* g_detector = nullptr;
apriltag_family_t* g_family = nullptr;
std::vector<uint8_t> g_luma;

double Dot(const Vec3& a, const Vec3& b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
double Norm(const Vec3& v) { return std::sqrt(Dot(v, v)); }
Vec3 Add(const Vec3& a, const Vec3& b) { return {a.x+b.x, a.y+b.y, a.z+b.z}; }
Vec3 Scale(const Vec3& v, double s) { return {v.x*s, v.y*s, v.z*s}; }

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
    double* maximumResidual) {
    if (correspondences.size() < 4) return false;
    for (int iteration = 0; iteration < kMaxRefinementIterations; ++iteration) {
        double hessian[6][6]{};
        double gradient[6]{};
        for (const auto& observation : correspondences) {
            Vec2 projection{};
            if (!Project(*pose, observation.boardPoint, fx, fy, cx, cy, distortion, &projection)) return false;
            const double residual[2] = {observation.imagePoint.x - projection.x, observation.imagePoint.y - projection.y};
            const double residualNorm = std::hypot(residual[0], residual[1]);
            const double huberWeight = residualNorm <= 4.0 ? 1.0 : 4.0 / residualNorm;
            double jacobian[2][6]{};
            for (int parameter = 0; parameter < 6; ++parameter) {
                Vec2 shifted{};
                if (!Project(PerturbCameraPose(*pose, parameter, kFiniteDifference), observation.boardPoint, fx, fy, cx, cy, distortion, &shifted)) return false;
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
        if (!SolveLinear6(hessian, gradient, step)) return false;
        pose->q = Multiply(SmallAngleQuaternion({step[0], step[1], step[2]}), pose->q);
        pose->t = Add(pose->t, {step[3], step[4], step[5]});
        if (std::sqrt(step[0]*step[0] + step[1]*step[1] + step[2]*step[2] + step[3]*step[3] + step[4]*step[4] + step[5]*step[5]) < 1e-6) break;
    }
    double squaredError = 0.0;
    *maximumResidual = 0.0;
    for (const auto& observation : correspondences) {
        Vec2 projection{};
        if (!Project(*pose, observation.boardPoint, fx, fy, cx, cy, distortion, &projection)) return false;
        const double residual = std::hypot(observation.imagePoint.x-projection.x, observation.imagePoint.y-projection.y);
        squaredError += residual*residual;
        *maximumResidual = std::max(*maximumResidual, residual);
    }
    *rms = std::sqrt(squaredError / static_cast<double>(correspondences.size()));
    return true;
}

void ReleaseDetectorLocked() {
    if (g_detector) { apriltag_detector_destroy(g_detector); g_detector = nullptr; }
    if (g_family) { tagStandard41h12_destroy(g_family); g_family = nullptr; }
    g_luma.clear();
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
        definition.sizeMeters = size;
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
    const double half = marker.sizeMeters * 0.5;
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
    RigidPose initialCameraFromBoard{{0,0,0,1},{0,0,0}};
    bool hasInitialPose = false;
    apriltag_detection_t* initialDetection = nullptr;
    const MarkerDefinition* initialMarker = nullptr;
    const int count = zarray_size(detections);
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
            correspondences.push_back({boardCorners[corner], observed.detectedCorners[corner]});
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

    // The official pose initializer accepts only K. Feed it corners undistorted
    // into the same calibrated ImageReader pixel frame before joint refinement.
    if (initialDetection && initialMarker) {
        apriltag_detection_t undistortedDetection = *initialDetection;
        for (int corner = 0; corner < 4; ++corner) {
            const Vec2 point = UndistortPixel({initialDetection->p[corner][0], initialDetection->p[corner][1]}, fx, fy, cx, cy, distortion);
            undistortedDetection.p[corner][0] = point.x;
            undistortedDetection.p[corner][1] = point.y;
        }
        apriltag_detection_info_t info{&undistortedDetection, initialMarker->sizeMeters, fx, fy, cx, cy};
        apriltag_pose_t tagPose{};
        const double tagPoseError = estimate_tag_pose(&info, &tagPose);
        if (tagPose.R && tagPose.t) {
            // T_camera_board = T_camera_marker * T_marker_board.
            initialCameraFromBoard = Compose(PoseFromOfficialTagPose(tagPose), Invert(initialMarker->boardFromMarker));
            hasInitialPose = true;
            BOARD_LOGI("DJI_BOARD_INITIAL_TAG_POSE error=%.6f markerId=%d", tagPoseError, initialMarker->id);
        }
        if (tagPose.R) matd_destroy(tagPose.R);
        if (tagPose.t) matd_destroy(tagPose.t);
    }

    RigidPose cameraFromBoard = initialCameraFromBoard;
    double rms = std::numeric_limits<double>::quiet_NaN();
    double maximumResidual = std::numeric_limits<double>::quiet_NaN();
    BOARD_LOGI("BOARD_STAGE_06_PNP_BEGIN correspondences=%zu hasInitialPose=%d", correspondences.size(), hasInitialPose ? 1 : 0);
    if (!hasInitialPose) {
        apriltag_detections_destroy(detections);
        return CreateFailureResult(env, FailureCode::PoseInitializationFailed, "OFFICIAL_APRILTAG_POSE_INITIALIZATION_FAILED");
    }
    const bool poseValid = RefineBoardPose(correspondences, fx, fy, cx, cy, distortion, &cameraFromBoard, &rms, &maximumResidual);
    if (!poseValid || !std::isfinite(rms) || !std::isfinite(maximumResidual)) {
        apriltag_detections_destroy(detections);
        return CreateFailureResult(env, FailureCode::PoseRefinementFailed, "BOARD_POSE_REFINEMENT_FAILED");
    }
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
Java_com_sok9hu_djibridge_DjiBoardVisionNative_releaseDetector(JNIEnv*, jclass) {
    std::lock_guard<std::mutex> lock(g_detectorMutex);
    ReleaseDetectorLocked();
}
