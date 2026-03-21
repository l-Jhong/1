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

enum class ObstacleType {
    SlideCandidate,
    CoreCandidate,
    MultiDirection
};

struct ObstacleRegion {
    ObstacleType type = ObstacleType::MultiDirection;
    std::vector<std::size_t> triangleIndices;
    Bounds bounds{};
    Vector3 suggestedDirection{};
    double visibilityRatio{};
    double undercutRatio{};
};

struct SeparabilityReport {
    double score{};
    bool separable{};
    std::vector<ObstacleRegion> obstacles;
};

struct PartingSurfaceStage {
    std::string name;
    std::vector<Vector3> boundary;
};

struct ContourFace {
    std::vector<Vector3> boundary;
    Vector3 normal{};
    Vector3 centroid{};
    double area{};
};

struct MoldBlock {
    std::string role;
    Bounds bounds{};
    Bounds cavityBounds{};
    Vector3 pullDirection{};
    bool subtractPart = true;
};

struct MoldAssembly {
    Bounds overallBounds{};
    std::vector<MoldBlock> blocks;
};

struct StrategyOption {
    std::string name;
    double score{};
    std::vector<ObstacleType> resolved;
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
    double separabilityUndercutThreshold = 0.08;
    double moldClearance = 2.0;
};

struct AutoPartingResult {
    Mesh cleanedMesh;
    DemoldEvaluation demold;
    PartingLine partingLine;
    PartingSurface partingSurface;
    std::vector<PartingSurfaceStage> partingSurfaceStages;
    ContourFace maxContour;
    SeparabilityReport separability;
    SplitResult split;
    std::vector<CoreRegion> cores;
    MoldAssembly moldAssembly;
    std::vector<StrategyOption> strategies;
    std::vector<InterferenceIssue> issues;
};

class AutoPartingPipeline {
public:
    AutoPartingResult run(const Mesh& input, const AutoPartingSettings& settings = {});
};

}  // namespace casting
