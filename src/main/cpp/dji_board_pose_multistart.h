#pragma once

// Included after the existing board-pose math/refiner, inside its private namespace.
// The public JNI result layout and every downstream quality gate remain unchanged.
constexpr size_t kMaximumMultiStartHypotheses = 5; // original + four, if original is unavailable

struct MultiStartHypothesis {
    int markerId = -1;
    bool original = false;
    bool available = false;
    bool valid = false;
    RigidPose initial{};
    RigidPose refined{};
    double initialRms = std::numeric_limits<double>::quiet_NaN();
    double finalRms = std::numeric_limits<double>::quiet_NaN();
    double maximumResidual = std::numeric_limits<double>::quiet_NaN();
    int64_t refinementNs = 0;
    std::array<char, 72> reason{};
};

struct MultiStartDiagnostics {
    bool applied = false;
    std::array<MultiStartHypothesis, kMaximumMultiStartHypotheses> hypotheses{};
    size_t count = 0;
    int selected = -1;
    int second = -1;
    double originalSelectedAngleDeg = std::numeric_limits<double>::quiet_NaN();
    double originalSelectedTranslationM = std::numeric_limits<double>::quiet_NaN();
    double bestSecondAngleDeg = std::numeric_limits<double>::quiet_NaN();
    double bestSecondRmsGap = std::numeric_limits<double>::quiet_NaN();
    double maximumValidAngleDeg = 0;
    double closestDistinctRmsGap = std::numeric_limits<double>::infinity();
    bool numericalTie = false;
    bool multipleValidOrientationBasins = false;
};

bool IsUsableMultiStartPose(const RigidPose& pose) {
    const auto& q = pose.q;
    const double normSquared = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    return IsFinite(q) && IsFinite(pose.t) && std::isfinite(normSquared) && normSquared > 1e-24 &&
        std::isfinite(static_cast<float>(pose.t.x)) && std::isfinite(static_cast<float>(pose.t.y)) &&
        std::isfinite(static_cast<float>(pose.t.z));
}

bool IsCompleteFourTagCorrespondenceSet(
    const std::vector<VisibleMarker>& visible, const std::vector<Correspondence>& observations) {
    if (visible.size() != 4 || observations.size() != 16) return false;
    for (size_t i = 0; i < visible.size(); ++i) {
        for (size_t j = 0; j < i; ++j) if (visible[i].id == visible[j].id) return false;
        std::array<bool, 4> found{};
        for (const auto& observation : observations) {
            if (observation.markerId != visible[i].id) continue;
            const int corner = observation.cornerIndex;
            if (corner < 0 || corner >= 4 || found[corner]) return false;
            found[corner] = true;
        }
        for (bool present : found) if (!present) return false;
    }
    return true;
}

// RMS is published through the existing jfloat result. Equal published floats
// are a numerical tie, not a new image-quality/admission threshold.
bool MultiStartBetter(const MultiStartHypothesis& a, const MultiStartHypothesis& b) {
    const float left = static_cast<float>(a.finalRms);
    const float right = static_cast<float>(b.finalRms);
    if (left != right) return left < right;
    if (a.original != b.original) return a.original;
    return a.markerId < b.markerId;
}

template<class Refiner>
bool SelectFourTagPoseHypotheses(
    const std::vector<Correspondence>& observations,
    const std::vector<VisibleMarker>& visible,
    const std::vector<IndependentMarkerPose>& independent,
    int originalMarkerId, bool originalAvailable, const RigidPose& original,
    double fx, double fy, double cx, double cy, const Distortion& distortion,
    RigidPose* pose, double* rms, double* maximumResidual, std::string* failureReason,
    MultiStartDiagnostics* diagnostics, Refiner refine) {
    auto& d = *diagnostics;
    d = {};
    d.applied = true;
    if (!IsCompleteFourTagCorrespondenceSet(visible, observations)) {
        if (failureReason) *failureReason = "MULTISTART_INVALID_FOUR_TAG_CORRESPONDENCES";
        return false;
    }
    auto& first = d.hypotheses[d.count++];
    first.markerId = originalMarkerId;
    first.original = true;
    first.available = originalAvailable;
    first.initial = original;
    for (const auto& candidate : independent) {
        if (d.count == d.hypotheses.size()) break;
        if (candidate.id == originalMarkerId && originalAvailable) continue;
        bool recognized = false;
        for (const auto& marker : visible) recognized |= marker.id == candidate.id;
        if (!recognized) continue;
        // Ambiguous duplicate input IDs are not allowed to replace a valid original.
        int occurrences = 0;
        for (const auto& other : independent) occurrences += other.id == candidate.id;
        if (occurrences != 1) continue;
        auto& h = d.hypotheses[d.count++];
        h.markerId = candidate.id;
        h.available = true;
        h.initial = candidate.cameraFromBoard;
    }
    std::sort(d.hypotheses.begin()+1, d.hypotheses.begin()+d.count,
        [](const MultiStartHypothesis& a, const MultiStartHypothesis& b) { return a.markerId < b.markerId; });
    for (size_t i = 0; i < d.count; ++i) {
        auto& h = d.hypotheses[i];
        std::string reason;
        if (!h.available || !IsUsableMultiStartPose(h.initial)) {
            reason = "INVALID_INITIAL_POSE";
        } else {
            const auto initialStats = EvaluateReprojection(h.initial, observations, fx, fy, cx, cy, distortion, false);
            h.initialRms = initialStats.rms;
            h.refined = h.initial;
            if (!initialStats.valid) {
                reason = initialStats.failureReason;
            } else {
                const auto began = std::chrono::steady_clock::now();
                try {
                    h.valid = refine(&h.refined, &h.finalRms, &h.maximumResidual, &reason);
                } catch (...) {
                    h.valid = false;
                    reason = "REFINEMENT_EXCEPTION";
                }
                h.refinementNs = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::steady_clock::now()-began).count();
                if (h.valid) {
                    const auto finalStats = EvaluateReprojection(h.refined, observations, fx, fy, cx, cy, distortion, false);
                    h.valid = IsUsableMultiStartPose(h.refined) && finalStats.valid &&
                        std::isfinite(h.finalRms) && std::isfinite(h.maximumResidual) &&
                        std::isfinite(static_cast<float>(h.finalRms)) &&
                        std::isfinite(static_cast<float>(finalStats.rms)) &&
                        std::isfinite(static_cast<float>(finalStats.maximumResidual)) &&
                        h.finalRms >= 0 && h.maximumResidual >= 0;
                    if (h.valid) {
                        h.finalRms = finalStats.rms;
                        h.maximumResidual = finalStats.maximumResidual;
                    } else reason = "INVALID_REFINED_POSE";
                }
            }
        }
        std::snprintf(h.reason.data(), h.reason.size(), "%s", h.valid ? "VALID" : reason.c_str());
        if (h.valid && (d.selected < 0 || MultiStartBetter(h, d.hypotheses[d.selected])))
            d.selected = static_cast<int>(i);
    }
    if (d.selected < 0) {
        if (failureReason) *failureReason = "MULTISTART_NO_VALID_REFINED_POSE";
        return false;
    }
    const auto& selected = d.hypotheses[d.selected];
    for (size_t i = 0; i < d.count; ++i) {
        if (!d.hypotheses[i].valid) continue;
        if (static_cast<int>(i) != d.selected &&
            (d.second < 0 || MultiStartBetter(d.hypotheses[i], d.hypotheses[d.second])))
            d.second = static_cast<int>(i);
        for (size_t j = 0; j < i; ++j) {
            if (!d.hypotheses[j].valid) continue;
            const double angle = QuaternionAngularDifferenceDegrees(d.hypotheses[i].refined.q, d.hypotheses[j].refined.q);
            d.maximumValidAngleDeg = std::max(d.maximumValidAngleDeg, angle);
            // Diagnostic category only; never participates in selection or admission.
            if (angle > 10.0) {
                d.multipleValidOrientationBasins = true;
                d.closestDistinctRmsGap = std::min(d.closestDistinctRmsGap,
                    std::abs(d.hypotheses[i].finalRms-d.hypotheses[j].finalRms));
            }
        }
    }
    if (first.available && IsUsableMultiStartPose(first.initial)) {
        d.originalSelectedAngleDeg = QuaternionAngularDifferenceDegrees(
            first.valid ? first.refined.q : first.initial.q, selected.refined.q);
        d.originalSelectedTranslationM = Norm(Subtract(
            first.valid ? first.refined.t : first.initial.t, selected.refined.t));
    }
    if (d.second >= 0) {
        const auto& second = d.hypotheses[d.second];
        d.bestSecondAngleDeg = QuaternionAngularDifferenceDegrees(selected.refined.q, second.refined.q);
        d.bestSecondRmsGap = std::abs(second.finalRms-selected.finalRms);
        d.numericalTie = static_cast<float>(second.finalRms) == static_cast<float>(selected.finalRms);
    }
    *pose = selected.refined;
    *rms = selected.finalRms;
    *maximumResidual = selected.maximumResidual;
    return true;
}

bool RefineBoardPoseSelection(
    bool experimentEnabled, const std::vector<Correspondence>& observations,
    const std::vector<VisibleMarker>& visible, const std::vector<IndependentMarkerPose>& independent,
    int originalMarkerId, bool originalAvailable, const RigidPose& original,
    double fx, double fy, double cx, double cy, const Distortion& distortion,
    RigidPose* pose, double* rms, double* maximumResidual, std::string* failureReason,
    bool logDetails, MultiStartDiagnostics* diagnostics) {
    *diagnostics = {};
    if (!experimentEnabled || visible.size() != 4)
        return RefineBoardPose(observations, fx, fy, cx, cy, distortion,
            pose, rms, maximumResidual, failureReason, logDetails);
    const bool savedQuiet = g_quietPoseRefinement;
    struct Restore { bool value; ~Restore() { g_quietPoseRefinement = value; } } restore{savedQuiet};
    g_quietPoseRefinement = true;
    return SelectFourTagPoseHypotheses(observations, visible, independent,
        originalMarkerId, originalAvailable, original, fx, fy, cx, cy, distortion,
        pose, rms, maximumResidual, failureReason, diagnostics,
        [&](RigidPose* p, double* r, double* m, std::string* reason) {
            return RefineBoardPose(observations, fx, fy, cx, cy, distortion, p, r, m, reason, false);
        });
}

struct MultiStartJson {
    std::array<char, 3072> data{};
    size_t size = 0;
    void append(const char* format, ...) {
        if (size >= data.size()-1) return;
        va_list args;
        va_start(args, format);
        const int count = std::vsnprintf(data.data()+size, data.size()-size, format, args);
        va_end(args);
        if (count > 0) size = std::min(data.size()-1, size+static_cast<size_t>(count));
    }
    void number(const char* name, double value) {
        append("\"%s\":", name);
        if (std::isfinite(value)) append("%.17g", value); else append("null");
    }
};

MultiStartJson FormatMultiStartDiagnostics(const MultiStartDiagnostics& d,
    const std::vector<VisibleMarker>& visible, int64_t frameTimestampNs, int64_t generation, int64_t sequence) {
    MultiStartJson json;
    json.append("{\"experiment\":\"DJI_APRILTAG_FOUR_TAG_MULTISTART_EXPERIMENT\",\"enabled\":true,"
        "\"frameTimestampNs\":%lld,\"generation\":%lld,\"sequence\":%lld,\"applied\":%s,\"tagIds\":[",
        static_cast<long long>(frameTimestampNs), static_cast<long long>(generation), static_cast<long long>(sequence), d.applied ? "true" : "false");
    for (size_t i = 0; i < visible.size(); ++i) json.append("%s%d", i ? "," : "", visible[i].id);
    json.append("],\"originalInitializerId\":%d,\"selectedInitializerId\":%d,\"hypotheses\":[",
        d.count ? d.hypotheses[0].markerId : -1, d.selected >= 0 ? d.hypotheses[d.selected].markerId : -1);
    for (size_t i = 0; i < d.count; ++i) {
        const auto& h = d.hypotheses[i];
        json.append("%s{\"id\":%d,\"original\":%s,", i ? "," : "", h.markerId, h.original ? "true" : "false");
        json.number("initialRms", h.initialRms); json.append(",");
        json.number("finalRms", h.finalRms);
        json.append(",\"valid\":%s,\"refinementNs\":%lld,\"reason\":\"%s\"}",
            h.valid ? "true" : "false", static_cast<long long>(h.refinementNs), h.reason.data());
    }
    json.append("],"); json.number("originalSelectedAngleDeg", d.originalSelectedAngleDeg);
    json.append(","); json.number("originalSelectedTranslationM", d.originalSelectedTranslationM);
    json.append(","); json.number("bestSecondAngleDeg", d.bestSecondAngleDeg);
    json.append(","); json.number("bestSecondRmsGap", d.bestSecondRmsGap);
    json.append(","); json.number("maxValidAngleDeg", d.maximumValidAngleDeg);
    json.append(","); json.number("closestDistinctRmsGap", d.closestDistinctRmsGap);
    json.append(",\"numericalTie\":%s,\"multipleValidOrientationBasins\":%s,"
        "\"diagnosticOrientationSeparationDeg\":10,\"ambiguityMeaning\":\"NATIVE_VALID_NOT_PHYSICAL_CERTAINTY\"}",
        d.numericalTie ? "true" : "false", d.multipleValidOrientationBasins ? "true" : "false");
    return json;
}
