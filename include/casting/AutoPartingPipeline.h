#pragma once

#include "casting/Geometry.h"

#include <string>
#include <vector>

namespace casting {

struct DemoldEvaluation {
    Vector3 direction{};
    double score{};
    double visibilityRatio{};
    double undercutRatio{};
};

struct PartingLine {
    std::vector<Vector3> points;
};

struct PartingSurface {
    std::vector<Vector3> boundary;
};

struct SplitResult {
    Mesh upper;
    Mesh lower;
};

struct CoreRegion {
    std::vector<std::size_t> triangleIndices;
    Bounds bounds{};
};

struct InterferenceIssue {
    std::string message;
    double severity{};
};

struct AutoPartingSettings {
    double minTriangleArea = 1e-8;
    double draftAngleDegrees = 2.0;
    double undercutPenalty = 1.5;
    double visibilityPenalty = 0.5;
    double smoothingFactor = 0.3;
};

struct AutoPartingResult {
    Mesh cleanedMesh;
    DemoldEvaluation demold;
    PartingLine partingLine;
    PartingSurface partingSurface;
    SplitResult split;
    std::vector<CoreRegion> cores;
    std::vector<InterferenceIssue> issues;
};

class AutoPartingPipeline {
public:
    AutoPartingResult run(const Mesh& input, const AutoPartingSettings& settings = {});
};

}  // namespace casting
