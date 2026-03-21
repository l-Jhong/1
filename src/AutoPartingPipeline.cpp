#include "casting/AutoPartingPipeline.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <unordered_set>

namespace casting {
namespace {

struct EdgeKey {
    std::size_t a{};
    std::size_t b{};

    bool operator==(const EdgeKey& other) const {
        return a == other.a && b == other.b;
    }
};

struct EdgeKeyHasher {
    std::size_t operator()(const EdgeKey& key) const {
        return std::hash<std::size_t>{}(key.a) ^ (std::hash<std::size_t>{}(key.b) << 1U);
    }
};

constexpr double kPi = 3.14159265358979323846;
constexpr double kReferenceAxisAlignmentThreshold = 0.9;
constexpr double kOverlapTolerance = 1e-6;
constexpr double kUndercutWarningRatio = 0.2;
constexpr double kInvalidScore = -std::numeric_limits<double>::infinity();
constexpr double kClosureTolerance = 1e-6;
constexpr double kInternalCavityMarginRatio = 0.08;
constexpr double kStrategyObstaclePenalty = 0.2;
constexpr double kStrategyCorePenalty = 0.1;
constexpr double kStrategyCorePreferencePenalty = 0.05;

double degreesToRadians(double degrees) {
    return degrees * kPi / 180.0;
}

struct PlaneBasis {
    Vector3 origin;
    Vector3 normal;
    Vector3 axisU;
    Vector3 axisV;
};

PlaneBasis buildPlaneBasis(const Vector3& origin, const Vector3& normal) {
    PlaneBasis basis;
    basis.origin = origin;
    basis.normal = normalized(normal);
    Vector3 reference = (std::abs(basis.normal.x) < kReferenceAxisAlignmentThreshold)
                            ? Vector3{1.0, 0.0, 0.0}
                            : Vector3{0.0, 1.0, 0.0};
    basis.axisU = normalized(cross(basis.normal, reference));
    if (length(basis.axisU) <= std::numeric_limits<double>::epsilon()) {
        basis.axisU = {1.0, 0.0, 0.0};
    }
    basis.axisV = normalized(cross(basis.normal, basis.axisU));
    return basis;
}

Vector2 projectToPlane(const Vector3& point, const PlaneBasis& basis) {
    Vector3 offset = point - basis.origin;
    return {dot(offset, basis.axisU), dot(offset, basis.axisV)};
}

Vector3 liftFromPlane(const Vector2& point, const PlaneBasis& basis) {
    return basis.origin + basis.axisU * point.x + basis.axisV * point.y;
}

double polygonArea2D(const std::vector<Vector2>& polygon) {
    if (polygon.size() < 3) {
        return 0.0;
    }
    double area = 0.0;
    for (std::size_t i = 0; i < polygon.size(); ++i) {
        const Vector2& a = polygon[i];
        const Vector2& b = polygon[(i + 1) % polygon.size()];
        area += a.x * b.y - b.x * a.y;
    }
    return std::abs(area) * 0.5;
}

Mesh preprocessMesh(const Mesh& input, double minArea) {
    Mesh cleaned;
    cleaned.vertices = input.vertices;
    cleaned.triangles.reserve(input.triangles.size());
    Vector3 meshCentroid = computeCentroid(input);
    for (const auto& triangle : input.triangles) {
        if (triangle.v0 >= cleaned.vertices.size() || triangle.v1 >= cleaned.vertices.size() ||
            triangle.v2 >= cleaned.vertices.size()) {
            continue;
        }
        double area = triangleArea(input, triangle);
        if (area <= minArea) {
            continue;
        }
        Vector3 normal = triangleNormal(input, triangle);
        Vector3 centroid = triangleCentroid(input, triangle);
        Vector3 outward = centroid - meshCentroid;
        Triangle updated = triangle;
        if (dot(normal, outward) < 0.0) {
            std::swap(updated.v1, updated.v2);
        }
        cleaned.triangles.push_back(updated);
    }
    return cleaned;
}

DemoldEvaluation evaluateDemoldDirection(const Mesh& mesh, const Vector3& direction,
                                         double draftAngleDegrees, double undercutPenalty,
                                         double visibilityPenalty) {
    double totalArea = 0.0;
    double visibleArea = 0.0;
    double undercutArea = 0.0;
    double draftThreshold = std::sin(degreesToRadians(draftAngleDegrees));
    Vector3 dir = normalized(direction);
    for (const auto& triangle : mesh.triangles) {
        double area = triangleArea(mesh, triangle);
        totalArea += area;
        Vector3 normal = triangleNormal(mesh, triangle);
        double alignment = dot(normal, dir);
        if (alignment > 0.0) {
            visibleArea += area;
        }
        if (alignment < -draftThreshold) {
            undercutArea += area;
        }
    }
    if (totalArea <= std::numeric_limits<double>::epsilon()) {
        return {dir, kInvalidScore, 0.0, 0.0};
    }
    double visibilityRatio = visibleArea / totalArea;
    double undercutRatio = undercutArea / totalArea;
    double score = visibilityRatio - undercutPenalty * undercutRatio -
                   visibilityPenalty * (1.0 - visibilityRatio);
    return {dir, score, visibilityRatio, undercutRatio};
}

DemoldEvaluation selectBestDemoldDirection(const Mesh& mesh, const AutoPartingSettings& settings) {
    std::vector<Vector3> candidates = {
        {1.0, 0.0, 0.0},
        {-1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, -1.0, 0.0},
        {0.0, 0.0, 1.0},
        {0.0, 0.0, -1.0}
    };
    Vector3 averageNormal{};
    for (const auto& triangle : mesh.triangles) {
        averageNormal += triangleNormal(mesh, triangle) * triangleArea(mesh, triangle);
    }
    if (length(averageNormal) > std::numeric_limits<double>::epsilon()) {
        candidates.push_back(normalized(averageNormal));
    }
    DemoldEvaluation best{candidates.front(), kInvalidScore, 0.0, 0.0};
    for (const auto& candidate : candidates) {
        DemoldEvaluation eval =
            evaluateDemoldDirection(mesh, candidate, settings.draftAngleDegrees,
                                    settings.undercutPenalty, settings.visibilityPenalty);
        if (eval.score > best.score) {
            best = eval;
        }
    }
    return best;
}

PartingLine extractPartingLine(const Mesh& mesh, const Vector3& direction) {
    PartingLine line;
    if (mesh.triangles.empty()) {
        return line;
    }
    std::unordered_map<EdgeKey, std::vector<std::size_t>, EdgeKeyHasher> edgeMap;
    edgeMap.reserve(mesh.triangles.size() * 3);
    Vector3 dir = normalized(direction);
    std::vector<bool> frontFacing(mesh.triangles.size(), false);
    for (std::size_t index = 0; index < mesh.triangles.size(); ++index) {
        const auto& tri = mesh.triangles[index];
        frontFacing[index] = dot(triangleNormal(mesh, tri), dir) >= 0.0;
        std::array<std::size_t, 3> verts{tri.v0, tri.v1, tri.v2};
        for (std::size_t i = 0; i < 3; ++i) {
            std::size_t a = verts[i];
            std::size_t b = verts[(i + 1) % 3];
            EdgeKey key{std::min(a, b), std::max(a, b)};
            edgeMap[key].push_back(index);
        }
    }
    for (const auto& entry : edgeMap) {
        const auto& tris = entry.second;
        if (tris.size() != 2) {
            continue;
        }
        if (frontFacing[tris[0]] == frontFacing[tris[1]]) {
            continue;
        }
        const Vector3& a = mesh.vertices.at(entry.first.a);
        const Vector3& b = mesh.vertices.at(entry.first.b);
        line.points.push_back({(a.x + b.x) / 2.0, (a.y + b.y) / 2.0, (a.z + b.z) / 2.0});
    }
    if (line.points.size() < 3) {
        return line;
    }
    Vector3 centroid{};
    for (const auto& point : line.points) {
        centroid += point;
    }
    centroid = centroid / static_cast<double>(line.points.size());
    Vector3 normal = normalized(direction);
    Vector3 reference = (std::abs(normal.x) < kReferenceAxisAlignmentThreshold)
                            ? Vector3{1.0, 0.0, 0.0}
                            : Vector3{0.0, 1.0, 0.0};
    Vector3 axisU = normalized(cross(normal, reference));
    if (length(axisU) <= std::numeric_limits<double>::epsilon()) {
        axisU = {1.0, 0.0, 0.0};
    }
    Vector3 axisV = normalized(cross(normal, axisU));
    std::sort(line.points.begin(), line.points.end(),
              [&centroid, &axisU, &axisV](const Vector3& left, const Vector3& right) {
                  Vector3 leftOffset = left - centroid;
                  Vector3 rightOffset = right - centroid;
                  double leftAngle = std::atan2(dot(leftOffset, axisV), dot(leftOffset, axisU));
                  double rightAngle = std::atan2(dot(rightOffset, axisV), dot(rightOffset, axisU));
                  return leftAngle < rightAngle;
              });
    return line;
}

PartingSurface buildPartingSurface(const PartingLine& line, double smoothingFactor) {
    PartingSurface surface;
    surface.boundary = line.points;
    if (surface.boundary.size() < 3) {
        return surface;
    }
    std::vector<Vector3> smoothed = surface.boundary;
    const std::size_t count = surface.boundary.size();
    for (std::size_t i = 0; i < count; ++i) {
        const Vector3& prev = surface.boundary[(i + count - 1) % count];
        const Vector3& curr = surface.boundary[i];
        const Vector3& next = surface.boundary[(i + 1) % count];
        Vector3 average{(prev.x + curr.x + next.x) / 3.0,
                        (prev.y + curr.y + next.y) / 3.0,
                        (prev.z + curr.z + next.z) / 3.0};
        smoothed[i] = curr * (1.0 - smoothingFactor) + average * smoothingFactor;
    }
    surface.boundary.swap(smoothed);
    Vector3 closureDelta = surface.boundary.front() - surface.boundary.back();
    if (length(closureDelta) > kClosureTolerance) {
        surface.boundary.push_back(surface.boundary.front());
    }
    return surface;
}

std::vector<PartingSurfaceStage> buildPartingSurfaceStages(const PartingLine& line,
                                                           double smoothingFactor) {
    std::vector<PartingSurfaceStage> stages;
    PartingSurfaceStage raw;
    raw.name = "raw";
    raw.boundary = line.points;
    stages.push_back(raw);

    PartingSurface smoothed = buildPartingSurface(line, smoothingFactor);
    PartingSurfaceStage smoothStage;
    smoothStage.name = "smoothed";
    smoothStage.boundary = smoothed.boundary;
    stages.push_back(smoothStage);

    return stages;
}

ContourFace identifyMaxContour(const PartingLine& line, const Vector3& direction) {
    ContourFace contour;
    if (line.points.size() < 3) {
        return contour;
    }
    Vector3 centroid{};
    for (const auto& point : line.points) {
        centroid += point;
    }
    centroid = centroid / static_cast<double>(line.points.size());
    PlaneBasis basis = buildPlaneBasis(centroid, direction);
    std::vector<Vector2> projected;
    projected.reserve(line.points.size());
    for (const auto& point : line.points) {
        projected.push_back(projectToPlane(point, basis));
    }
    std::vector<Vector2> hull = computeConvexHull2D(projected);
    if (hull.size() < 3) {
        return contour;
    }
    contour.boundary.reserve(hull.size() + 1);
    for (const auto& point : hull) {
        contour.boundary.push_back(liftFromPlane(point, basis));
    }
    if (!contour.boundary.empty()) {
        Vector3 closureDelta = contour.boundary.front() - contour.boundary.back();
        if (length(closureDelta) > kClosureTolerance) {
            contour.boundary.push_back(contour.boundary.front());
        }
    }
    contour.area = polygonArea2D(hull);
    contour.centroid = centroid;
    contour.normal = normalized(direction);
    return contour;
}

SplitResult splitMesh(const Mesh& mesh, const Vector3& direction) {
    SplitResult split;
    split.upper.vertices = mesh.vertices;
    split.lower.vertices = mesh.vertices;
    Vector3 planePoint = computeCentroid(mesh);
    Vector3 dir = normalized(direction);
    for (const auto& triangle : mesh.triangles) {
        Vector3 centroid = triangleCentroid(mesh, triangle);
        double side = dot(centroid - planePoint, dir);
        if (side >= 0.0) {
            split.upper.triangles.push_back(triangle);
        } else {
            split.lower.triangles.push_back(triangle);
        }
    }
    return split;
}

std::vector<std::vector<std::size_t>> buildTriangleAdjacency(const Mesh& mesh) {
    std::unordered_map<EdgeKey, std::vector<std::size_t>, EdgeKeyHasher> edgeMap;
    edgeMap.reserve(mesh.triangles.size() * 3);
    for (std::size_t index = 0; index < mesh.triangles.size(); ++index) {
        const auto& tri = mesh.triangles[index];
        std::array<std::size_t, 3> verts{tri.v0, tri.v1, tri.v2};
        for (std::size_t i = 0; i < 3; ++i) {
            std::size_t a = verts[i];
            std::size_t b = verts[(i + 1) % 3];
            EdgeKey key{std::min(a, b), std::max(a, b)};
            edgeMap[key].push_back(index);
        }
    }
    std::vector<std::vector<std::size_t>> adjacency(mesh.triangles.size());
    for (const auto& entry : edgeMap) {
        const auto& tris = entry.second;
        for (std::size_t i = 0; i < tris.size(); ++i) {
            for (std::size_t j = i + 1; j < tris.size(); ++j) {
                adjacency[tris[i]].push_back(tris[j]);
                adjacency[tris[j]].push_back(tris[i]);
            }
        }
    }
    return adjacency;
}

std::vector<CoreRegion> detectCoreRegions(const Mesh& mesh, const Vector3& direction,
                                          double draftAngleDegrees) {
    std::vector<CoreRegion> regions;
    if (mesh.triangles.empty()) {
        return regions;
    }
    double draftThreshold = std::sin(degreesToRadians(draftAngleDegrees));
    Vector3 dir = normalized(direction);
    std::vector<bool> isUndercut(mesh.triangles.size(), false);
    for (std::size_t index = 0; index < mesh.triangles.size(); ++index) {
        Vector3 normal = triangleNormal(mesh, mesh.triangles[index]);
        isUndercut[index] = dot(normal, dir) < -draftThreshold;
    }
    auto adjacency = buildTriangleAdjacency(mesh);
    std::vector<bool> visited(mesh.triangles.size(), false);
    for (std::size_t index = 0; index < mesh.triangles.size(); ++index) {
        if (!isUndercut[index] || visited[index]) {
            continue;
        }
        CoreRegion region;
        std::vector<std::size_t> stack{index};
        visited[index] = true;
        Bounds bounds{};
        bounds.min = {std::numeric_limits<double>::max(),
                      std::numeric_limits<double>::max(),
                      std::numeric_limits<double>::max()};
        bounds.max = {std::numeric_limits<double>::lowest(),
                      std::numeric_limits<double>::lowest(),
                      std::numeric_limits<double>::lowest()};
        while (!stack.empty()) {
            std::size_t current = stack.back();
            stack.pop_back();
            region.triangleIndices.push_back(current);
            Vector3 centroid = triangleCentroid(mesh, mesh.triangles[current]);
            bounds.min.x = std::min(bounds.min.x, centroid.x);
            bounds.min.y = std::min(bounds.min.y, centroid.y);
            bounds.min.z = std::min(bounds.min.z, centroid.z);
            bounds.max.x = std::max(bounds.max.x, centroid.x);
            bounds.max.y = std::max(bounds.max.y, centroid.y);
            bounds.max.z = std::max(bounds.max.z, centroid.z);
            for (std::size_t neighbor : adjacency[current]) {
                if (isUndercut[neighbor] && !visited[neighbor]) {
                    visited[neighbor] = true;
                    stack.push_back(neighbor);
                }
            }
        }
        region.bounds = bounds;
        regions.push_back(region);
    }
    return regions;
}

DemoldEvaluation evaluateRegionDirection(const Mesh& mesh, const std::vector<std::size_t>& indices,
                                         const Vector3& direction, double draftAngleDegrees) {
    double totalArea = 0.0;
    double visibleArea = 0.0;
    double undercutArea = 0.0;
    double draftThreshold = std::sin(degreesToRadians(draftAngleDegrees));
    Vector3 dir = normalized(direction);
    for (std::size_t index : indices) {
        const auto& triangle = mesh.triangles.at(index);
        double area = triangleArea(mesh, triangle);
        totalArea += area;
        Vector3 normal = triangleNormal(mesh, triangle);
        double alignment = dot(normal, dir);
        if (alignment > 0.0) {
            visibleArea += area;
        }
        if (alignment < -draftThreshold) {
            undercutArea += area;
        }
    }
    if (totalArea <= std::numeric_limits<double>::epsilon()) {
        return {dir, kInvalidScore, 0.0, 0.0};
    }
    double visibilityRatio = visibleArea / totalArea;
    double undercutRatio = undercutArea / totalArea;
    double score = visibilityRatio - undercutRatio;
    return {dir, score, visibilityRatio, undercutRatio};
}

std::vector<Vector3> buildRegionCandidateDirections(const Vector3& demoldDirection,
                                                    const Vector3& averageNormal) {
    std::vector<Vector3> candidates = {
        {1.0, 0.0, 0.0},
        {-1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, -1.0, 0.0},
        {0.0, 0.0, 1.0},
        {0.0, 0.0, -1.0}
    };
    if (length(averageNormal) > std::numeric_limits<double>::epsilon()) {
        candidates.push_back(normalized(averageNormal));
    }
    Vector3 lateral = cross(demoldDirection, averageNormal);
    if (length(lateral) > std::numeric_limits<double>::epsilon()) {
        candidates.push_back(normalized(lateral));
        candidates.push_back(normalized(lateral * -1.0));
    }
    return candidates;
}

double angleBetween(const Vector3& left, const Vector3& right) {
    double denom = length(left) * length(right);
    if (denom <= std::numeric_limits<double>::epsilon()) {
        return 0.0;
    }
    double cosine = std::clamp(dot(left, right) / denom, -1.0, 1.0);
    return std::acos(cosine) * 180.0 / kPi;
}

Bounds expandBounds(const Bounds& bounds, double clearance) {
    Bounds expanded = bounds;
    expanded.min.x -= clearance;
    expanded.min.y -= clearance;
    expanded.min.z -= clearance;
    expanded.max.x += clearance;
    expanded.max.y += clearance;
    expanded.max.z += clearance;
    return expanded;
}

Bounds computeBoundsFromTriangles(const Mesh& mesh) {
    Bounds bounds{};
    if (mesh.triangles.empty()) {
        return computeBounds(mesh);
    }
    Vector3 minPoint{std::numeric_limits<double>::max(),
                     std::numeric_limits<double>::max(),
                     std::numeric_limits<double>::max()};
    Vector3 maxPoint{std::numeric_limits<double>::lowest(),
                     std::numeric_limits<double>::lowest(),
                     std::numeric_limits<double>::lowest()};
    for (const auto& triangle : mesh.triangles) {
        const Vector3& a = mesh.vertices.at(triangle.v0);
        const Vector3& b = mesh.vertices.at(triangle.v1);
        const Vector3& c = mesh.vertices.at(triangle.v2);
        minPoint.x = std::min({minPoint.x, a.x, b.x, c.x});
        minPoint.y = std::min({minPoint.y, a.y, b.y, c.y});
        minPoint.z = std::min({minPoint.z, a.z, b.z, c.z});
        maxPoint.x = std::max({maxPoint.x, a.x, b.x, c.x});
        maxPoint.y = std::max({maxPoint.y, a.y, b.y, c.y});
        maxPoint.z = std::max({maxPoint.z, a.z, b.z, c.z});
    }
    bounds.min = minPoint;
    bounds.max = maxPoint;
    return bounds;
}

SeparabilityReport evaluateSeparability(const Mesh& mesh, const DemoldEvaluation& demold,
                                        const std::vector<CoreRegion>& undercuts,
                                        const AutoPartingSettings& settings,
                                        std::vector<CoreRegion>* coreRegionsOut) {
    SeparabilityReport report;
    report.score = demold.score;
    report.separable = demold.undercutRatio <= settings.separabilityUndercutThreshold;
    if (!coreRegionsOut) {
        return report;
    }
    Bounds meshBounds = computeBounds(mesh);
    Vector3 meshSize = meshBounds.max - meshBounds.min;
    double internalMarginX = std::abs(meshSize.x) * kInternalCavityMarginRatio;
    double internalMarginY = std::abs(meshSize.y) * kInternalCavityMarginRatio;
    double internalMarginZ = std::abs(meshSize.z) * kInternalCavityMarginRatio;

    for (const auto& region : undercuts) {
        ObstacleRegion obstacle;
        obstacle.triangleIndices = region.triangleIndices;
        obstacle.bounds = region.bounds;

        Vector3 averageNormal{};
        for (std::size_t index : region.triangleIndices) {
            averageNormal += triangleNormal(mesh, mesh.triangles.at(index));
        }
        if (length(averageNormal) <= std::numeric_limits<double>::epsilon()) {
            averageNormal = demold.direction;
        }
        std::vector<Vector3> candidates =
            buildRegionCandidateDirections(demold.direction, averageNormal);

        bool foundSlide = false;
        DemoldEvaluation bestSlide{};
        double bestAngle = -1.0;
        DemoldEvaluation bestFallback{};
        double bestFallbackUndercut = std::numeric_limits<double>::max();
        for (const auto& candidate : candidates) {
            DemoldEvaluation eval = evaluateRegionDirection(mesh, region.triangleIndices, candidate,
                                                            settings.draftAngleDegrees);
            if (eval.undercutRatio < bestFallbackUndercut) {
                bestFallbackUndercut = eval.undercutRatio;
                bestFallback = eval;
            }
            if (eval.undercutRatio <= settings.separabilityUndercutThreshold) {
                double angle = angleBetween(candidate, demold.direction);
                if (angle > bestAngle) {
                    bestAngle = angle;
                    bestSlide = eval;
                    foundSlide = true;
                }
            }
        }

        if (foundSlide) {
            obstacle.type = ObstacleType::SlideCandidate;
            obstacle.suggestedDirection = bestSlide.direction;
            obstacle.visibilityRatio = bestSlide.visibilityRatio;
            obstacle.undercutRatio = bestSlide.undercutRatio;
        } else {
            bool internal =
                region.bounds.min.x > meshBounds.min.x + internalMarginX &&
                region.bounds.min.y > meshBounds.min.y + internalMarginY &&
                region.bounds.min.z > meshBounds.min.z + internalMarginZ &&
                region.bounds.max.x < meshBounds.max.x - internalMarginX &&
                region.bounds.max.y < meshBounds.max.y - internalMarginY &&
                region.bounds.max.z < meshBounds.max.z - internalMarginZ;
            if (internal) {
                obstacle.type = ObstacleType::CoreCandidate;
                obstacle.suggestedDirection = bestFallback.direction;
                obstacle.visibilityRatio = bestFallback.visibilityRatio;
                obstacle.undercutRatio = bestFallback.undercutRatio;
                coreRegionsOut->push_back(region);
            } else {
                obstacle.type = ObstacleType::MultiDirection;
                obstacle.suggestedDirection = bestFallback.direction;
                obstacle.visibilityRatio = bestFallback.visibilityRatio;
                obstacle.undercutRatio = bestFallback.undercutRatio;
            }
        }
        report.obstacles.push_back(obstacle);
    }
    return report;
}

MoldAssembly buildMoldAssembly(const SplitResult& split, const Vector3& direction,
                               double clearance) {
    MoldAssembly assembly;
    MoldBlock upper;
    upper.role = "UpperMold";
    upper.cavityBounds = computeBoundsFromTriangles(split.upper);
    upper.bounds = expandBounds(upper.cavityBounds, clearance);
    upper.pullDirection = normalized(direction);

    MoldBlock lower;
    lower.role = "LowerMold";
    lower.cavityBounds = computeBoundsFromTriangles(split.lower);
    lower.bounds = expandBounds(lower.cavityBounds, clearance);
    lower.pullDirection = normalized(direction * -1.0);

    assembly.blocks.push_back(upper);
    assembly.blocks.push_back(lower);
    assembly.overallBounds = expandBounds(upper.cavityBounds, clearance);
    Bounds lowerBounds = expandBounds(lower.cavityBounds, clearance);
    assembly.overallBounds.min.x = std::min(assembly.overallBounds.min.x, lowerBounds.min.x);
    assembly.overallBounds.min.y = std::min(assembly.overallBounds.min.y, lowerBounds.min.y);
    assembly.overallBounds.min.z = std::min(assembly.overallBounds.min.z, lowerBounds.min.z);
    assembly.overallBounds.max.x = std::max(assembly.overallBounds.max.x, lowerBounds.max.x);
    assembly.overallBounds.max.y = std::max(assembly.overallBounds.max.y, lowerBounds.max.y);
    assembly.overallBounds.max.z = std::max(assembly.overallBounds.max.z, lowerBounds.max.z);
    return assembly;
}

std::vector<StrategyOption> buildStrategyOptions(const SeparabilityReport& report,
                                                 std::size_t coreCount) {
    std::vector<StrategyOption> options;
    StrategyOption baseline;
    baseline.name = "DefaultMultiParting";
    baseline.score = report.score -
                     static_cast<double>(report.obstacles.size()) * kStrategyObstaclePenalty -
                     static_cast<double>(coreCount) * kStrategyCorePenalty;
    for (const auto& obstacle : report.obstacles) {
        baseline.resolved.push_back(obstacle.type);
    }
    options.push_back(baseline);

    StrategyOption conservative = baseline;
    conservative.name = "CorePreferred";
    conservative.score -= static_cast<double>(coreCount) * kStrategyCorePreferencePenalty;
    options.push_back(conservative);
    return options;
}

std::vector<InterferenceIssue> checkInterference(const Mesh& mesh, const SplitResult& split,
                                                 const PartingLine& line,
                                                 const DemoldEvaluation& evaluation) {
    std::vector<InterferenceIssue> issues;
    if (line.points.empty()) {
        issues.push_back({"Parting line extraction produced no boundary points.", 0.8});
    }
    Bounds upperBounds = computeBoundsFromTriangles(split.upper);
    Bounds lowerBounds = computeBoundsFromTriangles(split.lower);
    double overlapX = std::min(upperBounds.max.x, lowerBounds.max.x) -
                      std::max(upperBounds.min.x, lowerBounds.min.x);
    double overlapY = std::min(upperBounds.max.y, lowerBounds.max.y) -
                      std::max(upperBounds.min.y, lowerBounds.min.y);
    double overlapZ = std::min(upperBounds.max.z, lowerBounds.max.z) -
                      std::max(upperBounds.min.z, lowerBounds.min.z);
    if (overlapX > kOverlapTolerance && overlapY > kOverlapTolerance && overlapZ > kOverlapTolerance) {
        issues.push_back({"Upper/lower mold bounds overlap. Verify split plane and parting surface.",
                          0.6});
    }
    if (evaluation.undercutRatio > kUndercutWarningRatio) {
        issues.push_back({"Undercut ratio exceeds threshold, consider alternative demold direction.",
                          std::min(1.0, evaluation.undercutRatio)});
    }
    if (mesh.triangles.empty()) {
        issues.push_back({"Mesh has no valid triangles after preprocessing.", 1.0});
    }
    return issues;
}

}  // namespace

AutoPartingResult AutoPartingPipeline::run(const Mesh& input, const AutoPartingSettings& settings) {
    AutoPartingResult result;
    result.cleanedMesh = preprocessMesh(input, settings.minTriangleArea);
    result.demold = selectBestDemoldDirection(result.cleanedMesh, settings);
    result.partingLine = extractPartingLine(result.cleanedMesh, result.demold.direction);
    result.partingSurfaceStages = buildPartingSurfaceStages(result.partingLine,
                                                            settings.smoothingFactor);
    if (!result.partingSurfaceStages.empty()) {
        result.partingSurface.boundary = result.partingSurfaceStages.back().boundary;
    } else {
        result.partingSurface = buildPartingSurface(result.partingLine, settings.smoothingFactor);
    }
    result.maxContour = identifyMaxContour(result.partingLine, result.demold.direction);
    result.split = splitMesh(result.cleanedMesh, result.demold.direction);
    std::vector<CoreRegion> undercutRegions = detectCoreRegions(result.cleanedMesh,
                                                                result.demold.direction,
                                                                settings.draftAngleDegrees);
    result.separability = evaluateSeparability(result.cleanedMesh, result.demold, undercutRegions,
                                               settings, &result.cores);
    result.moldAssembly = buildMoldAssembly(result.split, result.demold.direction,
                                            settings.moldClearance);
    result.strategies = buildStrategyOptions(result.separability, result.cores.size());
    result.issues = checkInterference(result.cleanedMesh, result.split, result.partingLine,
                                      result.demold);
    return result;
}

}  // namespace casting
