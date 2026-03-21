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

double degreesToRadians(double degrees) {
    return degrees * kPi / 180.0;
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
        return {dir, -std::numeric_limits<double>::infinity(), 0.0, 0.0};
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
    DemoldEvaluation best{candidates.front(), -std::numeric_limits<double>::infinity(), 0.0, 0.0};
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
    std::sort(line.points.begin(), line.points.end(),
              [&centroid](const Vector3& left, const Vector3& right) {
                  double leftAngle = std::atan2(left.y - centroid.y, left.x - centroid.x);
                  double rightAngle = std::atan2(right.y - centroid.y, right.x - centroid.x);
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
    if (surface.boundary.front().x != surface.boundary.back().x ||
        surface.boundary.front().y != surface.boundary.back().y ||
        surface.boundary.front().z != surface.boundary.back().z) {
        surface.boundary.push_back(surface.boundary.front());
    }
    return surface;
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

std::vector<InterferenceIssue> checkInterference(const Mesh& mesh, const SplitResult& split,
                                                 const PartingLine& line,
                                                 const DemoldEvaluation& evaluation) {
    std::vector<InterferenceIssue> issues;
    if (line.points.empty()) {
        issues.push_back({"Parting line extraction produced no boundary points.", 0.8});
    }
    Bounds upperBounds = computeBounds(split.upper);
    Bounds lowerBounds = computeBounds(split.lower);
    if (upperBounds.min.x <= lowerBounds.max.x && upperBounds.max.x >= lowerBounds.min.x &&
        upperBounds.min.y <= lowerBounds.max.y && upperBounds.max.y >= lowerBounds.min.y &&
        upperBounds.min.z <= lowerBounds.max.z && upperBounds.max.z >= lowerBounds.min.z) {
        issues.push_back({"Upper/lower mold bounds overlap. Verify split plane and parting surface.",
                          0.6});
    }
    if (evaluation.undercutRatio > 0.2) {
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
    result.partingSurface = buildPartingSurface(result.partingLine, settings.smoothingFactor);
    result.split = splitMesh(result.cleanedMesh, result.demold.direction);
    result.cores = detectCoreRegions(result.cleanedMesh, result.demold.direction,
                                     settings.draftAngleDegrees);
    result.issues = checkInterference(result.cleanedMesh, result.split, result.partingLine,
                                      result.demold);
    return result;
}

}  // namespace casting
