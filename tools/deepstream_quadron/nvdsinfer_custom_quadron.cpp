#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

#include "nvdsinfer_custom_impl.h"

namespace {

constexpr float kInputW = 1280.0f;
constexpr float kInputH = 1280.0f;
constexpr float kNmsThreshold = 0.50f;
constexpr int kKeepTopK = 20;

struct Candidate {
    float left;
    float top;
    float width;
    float height;
    float confidence;
    int class_id;
};

float intersection_over_union(const Candidate& a, const Candidate& b) {
    const float ax2 = a.left + a.width;
    const float ay2 = a.top + a.height;
    const float bx2 = b.left + b.width;
    const float by2 = b.top + b.height;

    const float ix1 = std::max(a.left, b.left);
    const float iy1 = std::max(a.top, b.top);
    const float ix2 = std::min(ax2, bx2);
    const float iy2 = std::min(ay2, by2);
    const float iw = std::max(0.0f, ix2 - ix1);
    const float ih = std::max(0.0f, iy2 - iy1);
    const float inter = iw * ih;
    const float union_area = a.width * a.height + b.width * b.height - inter;
    if (union_area <= 0.0f) {
        return 0.0f;
    }
    return inter / union_area;
}

float clampf(float value, float lo, float hi) {
    return std::max(lo, std::min(hi, value));
}

}  // namespace

extern "C" bool NvDsInferParseCustomQuadron(
    std::vector<NvDsInferLayerInfo> const& outputLayersInfo,
    NvDsInferNetworkInfo const& networkInfo,
    NvDsInferParseDetectionParams const& detectionParams,
    std::vector<NvDsInferObjectDetectionInfo>& objectList) {
    objectList.clear();
    if (outputLayersInfo.empty() || outputLayersInfo[0].buffer == nullptr) {
        return false;
    }

    const NvDsInferLayerInfo& layer = outputLayersInfo[0];
    int element_count = 1;
    for (int i = 0; i < layer.inferDims.numDims; ++i) {
        element_count *= layer.inferDims.d[i];
    }
    if (element_count <= 0 || element_count % 6 != 0) {
        return false;
    }

    const int rows = element_count / 6;
    const float* data = static_cast<const float*>(layer.buffer);
    const float threshold = detectionParams.perClassPreclusterThreshold.empty()
                                ? 0.10f
                                : detectionParams.perClassPreclusterThreshold[0];

    std::vector<Candidate> candidates;
    candidates.reserve(rows);
    for (int i = 0; i < rows; ++i) {
        const float* det = data + i * 6;
        const float object_conf = det[4];
        const float class_score = det[5];
        const float confidence = object_conf * class_score;
        if (confidence < threshold) {
            continue;
        }

        const float cx = det[0];
        const float cy = det[1];
        const float w = det[2];
        const float h = det[3];
        if (w <= 1.0f || h <= 1.0f) {
            continue;
        }

        Candidate candidate;
        candidate.left = clampf(cx - 0.5f * w, 0.0f, kInputW - 1.0f);
        candidate.top = clampf(cy - 0.5f * h, 0.0f, kInputH - 1.0f);
        const float right = clampf(cx + 0.5f * w, 0.0f, kInputW - 1.0f);
        const float bottom = clampf(cy + 0.5f * h, 0.0f, kInputH - 1.0f);
        candidate.width = right - candidate.left;
        candidate.height = bottom - candidate.top;
        candidate.confidence = confidence;
        candidate.class_id = 0;
        if (candidate.width > 1.0f && candidate.height > 1.0f) {
            candidates.push_back(candidate);
        }
    }

    std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
        return a.confidence > b.confidence;
    });

    std::vector<Candidate> selected;
    selected.reserve(std::min<int>(kKeepTopK, candidates.size()));
    for (const Candidate& candidate : candidates) {
        bool suppressed = false;
        for (const Candidate& kept : selected) {
            if (intersection_over_union(candidate, kept) > kNmsThreshold) {
                suppressed = true;
                break;
            }
        }
        if (suppressed) {
            continue;
        }
        selected.push_back(candidate);
        if (static_cast<int>(selected.size()) >= kKeepTopK) {
            break;
        }
    }

    for (const Candidate& candidate : selected) {
        NvDsInferObjectDetectionInfo object;
        std::memset(&object, 0, sizeof(object));
        object.classId = candidate.class_id;
        object.detectionConfidence = candidate.confidence;
        object.left = candidate.left;
        object.top = candidate.top;
        object.width = candidate.width;
        object.height = candidate.height;
        objectList.push_back(object);
    }

    return true;
}

CHECK_CUSTOM_PARSE_FUNC_PROTOTYPE(NvDsInferParseCustomQuadron);
