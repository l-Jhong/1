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

Vector3 projectToBasis(const Vector3& point, const PlaneBasis& basis) {
    Vector3 offset = point - basis.origin;
    return {dot(offset, basis.axisU), dot(offset, basis.axisV), dot(offset, basis.normal)};
}

Vector3 liftFromBasis(const Vector3& point, const PlaneBasis& basis) {
    return basis.origin + basis.axisU * point.x + basis.axisV * point.y +
           basis.normal * point.z;
}

double distance2D(const Vector2& left, const Vector2& right) {
    Vector2 delta = left - right;
    return std::sqrt(delta.x * delta.x + delta.y * delta.y);
}

std::vector<Vector2> closeLoop2D(const std::vector<Vector2>& points) {
    std::vector<Vector2> closed = points;
    if (closed.size() < 2) {
        return closed;
    }
    if (distance2D(closed.front(), closed.back()) > kClosureTolerance) {
        closed.push_back(closed.front());
    }
    return closed;
}

std::vector<Vector3> closeLoopLocal(const std::vector<Vector3>& points) {
    std::vector<Vector3> closed = points;
    if (closed.size() < 2) {
        return closed;
    }
    Vector2 first{closed.front().x, closed.front().y};
    Vector2 last{closed.back().x, closed.back().y};
    if (distance2D(first, last) > kClosureTolerance) {
        closed.push_back(closed.front());
    }
    return closed;
}

std::vector<Vector2> smoothClosedCurve2D(const std::vector<Vector2>& points, double smoothingFactor) {
    if (points.size() < 3) {
        return points;
    }
    std::vector<Vector2> smoothed(points.size());
    std::size_t count = points.size();
    for (std::size_t i = 0; i < count; ++i) {
        const Vector2& prev = points[(i + count - 1) % count];
        const Vector2& curr = points[i];
        const Vector2& next = points[(i + 1) % count];
        Vector2 average{(prev.x + curr.x + next.x) / 3.0,
                        (prev.y + curr.y + next.y) / 3.0};
        smoothed[i] = curr * (1.0 - smoothingFactor) + average * smoothingFactor;
    }
    return smoothed;
}

std::vector<Vector3> smoothClosedCurve3D(const std::vector<Vector3>& points, double smoothingFactor) {
    if (points.size() < 3) {
        return points;
    }
    std::vector<Vector3> smoothed(points.size());
    std::size_t count = points.size();
    for (std::size_t i = 0; i < count; ++i) {
        const Vector3& prev = points[(i + count - 1) % count];
        const Vector3& curr = points[i];
        const Vector3& next = points[(i + 1) % count];
        Vector3 average{(prev.x + curr.x + next.x) / 3.0,
                        (prev.y + curr.y + next.y) / 3.0,
                        (prev.z + curr.z + next.z) / 3.0};
        smoothed[i] = curr * (1.0 - smoothingFactor) + average * smoothingFactor;
    }
    return smoothed;
}

std::vector<Vector2> extendCurve2D(const std::vector<Vector2>& points, double extension) {
    if (points.size() < 3 || extension <= 0.0) {
        return points;
    }
    Vector2 centroid{};
    for (const auto& point : points) {
        centroid += point;
    }
    centroid = centroid / static_cast<double>(points.size());
    std::vector<Vector2> extended(points.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
        Vector2 offset = points[i] - centroid;
        double length = std::sqrt(offset.x * offset.x + offset.y * offset.y);
        if (length <= std::numeric_limits<double>::epsilon()) {
            extended[i] = points[i];
            continue;
        }
        Vector2 direction{offset.x / length, offset.y / length};
        extended[i] = points[i] + direction * extension;
    }
    return extended;
}

std::vector<Vector3> extendLocalCurve(const std::vector<Vector3>& points, double extension) {
    if (points.size() < 3 || extension <= 0.0) {
        return points;
    }
    Vector2 centroid{};
    for (const auto& point : points) {
        centroid += Vector2{point.x, point.y};
    }
    centroid = centroid / static_cast<double>(points.size());
    std::vector<Vector3> extended(points.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
        Vector2 offset{points[i].x - centroid.x, points[i].y - centroid.y};
        double length = std::sqrt(offset.x * offset.x + offset.y * offset.y);
        if (length <= std::numeric_limits<double>::epsilon()) {
            extended[i] = points[i];
            continue;
        }
        Vector2 direction{offset.x / length, offset.y / length};
        extended[i] = {points[i].x + direction.x * extension,
                       points[i].y + direction.y * extension,
                       points[i].z};
    }
    return extended;
}
double orientation2D(const Vector2& a, const Vector2& b, const Vector2& c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

bool onSegment2D(const Vector2& a, const Vector2& b, const Vector2& c) {
    return std::min(a.x, b.x) - kClosureTolerance <= c.x &&
           c.x <= std::max(a.x, b.x) + kClosureTolerance &&
           std::min(a.y, b.y) - kClosureTolerance <= c.y &&
           c.y <= std::max(a.y, b.y) + kClosureTolerance;
}

bool segmentsIntersect2D(const Vector2& p1, const Vector2& p2,
                         const Vector2& q1, const Vector2& q2) {
    double o1 = orientation2D(p1, p2, q1);
    double o2 = orientation2D(p1, p2, q2);
    double o3 = orientation2D(q1, q2, p1);
    double o4 = orientation2D(q1, q2, p2);
    if ((o1 * o2) < 0.0 && (o3 * o4) < 0.0) {
        return true;
    }
    if (std::abs(o1) <= kClosureTolerance && onSegment2D(p1, p2, q1)) {
        return true;
    }
    if (std::abs(o2) <= kClosureTolerance && onSegment2D(p1, p2, q2)) {
        return true;
    }
    if (std::abs(o3) <= kClosureTolerance && onSegment2D(q1, q2, p1)) {
        return true;
    }
    if (std::abs(o4) <= kClosureTolerance && onSegment2D(q1, q2, p2)) {
        return true;
    }
    return false;
}

bool hasSelfIntersection2D(const std::vector<Vector2>& loop) {
    if (loop.size() < 4) {
        return false;
    }
    std::size_t count = loop.size() - 1;
    for (std::size_t i = 0; i < count; ++i) {
        Vector2 a1 = loop[i];
        Vector2 a2 = loop[(i + 1) % count];
        for (std::size_t j = i + 2; j < count; ++j) {
            if (j + 1 == i || (i == 0 && j + 1 == count)) {
                continue;
            }
            Vector2 b1 = loop[j];
            Vector2 b2 = loop[(j + 1) % count];
            if (segmentsIntersect2D(a1, a2, b1, b2)) {
                return true;
            }
        }
    }
    return false;
}

Bounds computeLocalBounds(const Mesh& mesh, const PlaneBasis& basis) {
    Bounds bounds{};
    bounds.min = {std::numeric_limits<double>::max(),
                  std::numeric_limits<double>::max(),
                  std::numeric_limits<double>::max()};
    bounds.max = {std::numeric_limits<double>::lowest(),
                  std::numeric_limits<double>::lowest(),
                  std::numeric_limits<double>::lowest()};
    for (const auto& vertex : mesh.vertices) {
        Vector3 local = projectToBasis(vertex, basis);
        bounds.min.x = std::min(bounds.min.x, local.x);
        bounds.min.y = std::min(bounds.min.y, local.y);
        bounds.min.z = std::min(bounds.min.z, local.z);
        bounds.max.x = std::max(bounds.max.x, local.x);
        bounds.max.y = std::max(bounds.max.y, local.y);
        bounds.max.z = std::max(bounds.max.z, local.z);
    }
    return bounds;
}

Bounds expandLocalBounds(const Bounds& bounds, double padding) {
    Bounds expanded = bounds;
    expanded.min.x -= padding;
    expanded.min.y -= padding;
    expanded.min.z -= padding;
    expanded.max.x += padding;
    expanded.max.y += padding;
    expanded.max.z += padding;
    return expanded;
}

Bounds localBoundsToWorld(const Bounds& local, const PlaneBasis& basis) {
    std::array<Vector3, 8> corners = {
        liftFromBasis({local.min.x, local.min.y, local.min.z}, basis),
        liftFromBasis({local.max.x, local.min.y, local.min.z}, basis),
        liftFromBasis({local.max.x, local.max.y, local.min.z}, basis),
        liftFromBasis({local.min.x, local.max.y, local.min.z}, basis),
        liftFromBasis({local.min.x, local.min.y, local.max.z}, basis),
        liftFromBasis({local.max.x, local.min.y, local.max.z}, basis),
        liftFromBasis({local.max.x, local.max.y, local.max.z}, basis),
        liftFromBasis({local.min.x, local.max.y, local.max.z}, basis)
    };
    Bounds bounds{};
    bounds.min = {std::numeric_limits<double>::max(),
                  std::numeric_limits<double>::max(),
                  std::numeric_limits<double>::max()};
    bounds.max = {std::numeric_limits<double>::lowest(),
                  std::numeric_limits<double>::lowest(),
                  std::numeric_limits<double>::lowest()};
    for (const auto& corner : corners) {
        bounds.min.x = std::min(bounds.min.x, corner.x);
        bounds.min.y = std::min(bounds.min.y, corner.y);
        bounds.min.z = std::min(bounds.min.z, corner.z);
        bounds.max.x = std::max(bounds.max.x, corner.x);
        bounds.max.y = std::max(bounds.max.y, corner.y);
        bounds.max.z = std::max(bounds.max.z, corner.z);
    }
    return bounds;
}

Bounds scaleLocalBounds(const Bounds& bounds, const Vector3& center, double scale) {
    Bounds scaled;
    scaled.min = center + (bounds.min - center) * scale;
    scaled.max = center + (bounds.max - center) * scale;
    return scaled;
}

Bounds extendBoundsAlongDirection(const Bounds& bounds, const Vector3& direction, double length) {
    Bounds extended = bounds;
    Vector3 dir = normalized(direction);
    if (dir.x >= 0.0) {
        extended.max.x += length * std::abs(dir.x);
    } else {
        extended.min.x -= length * std::abs(dir.x);
    }
    if (dir.y >= 0.0) {
        extended.max.y += length * std::abs(dir.y);
    } else {
        extended.min.y -= length * std::abs(dir.y);
    }
    if (dir.z >= 0.0) {
        extended.max.z += length * std::abs(dir.z);
    } else {
        extended.min.z -= length * std::abs(dir.z);
    }
    return extended;
}

bool boundsContains(const Bounds& outer, const Bounds& inner) {
    return inner.min.x >= outer.min.x && inner.min.y >= outer.min.y &&
           inner.min.z >= outer.min.z && inner.max.x <= outer.max.x &&
           inner.max.y <= outer.max.y && inner.max.z <= outer.max.z;
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

PartingSurface buildPartingSurface(const PartingLine& line, const Vector3& direction,
                                   double smoothingFactor, double extension,
                                   bool preferPlanarSurface, double nonPlanarDeviationRatio,
                                   std::vector<PartingSurfaceStage>* stages) {
    PartingSurface surface;
    if (line.points.size() < 3) {
        return surface;
    }

    // 1) 以分型线重心作为基准点，构建与脱模方向垂直的基准平面
    Vector3 centroid{};
    for (const auto& point : line.points) {
        centroid += point;
    }
    centroid = centroid / static_cast<double>(line.points.size());
    PlaneBasis basis = buildPlaneBasis(centroid, direction);

    if (stages) {
        stages->push_back({"raw", line.points});
    }

    // 2) 将分型线投影到基准平面，得到二维投影曲线
    std::vector<Vector2> projected;
    projected.reserve(line.points.size());
    std::vector<Vector3> localPoints;
    localPoints.reserve(line.points.size());
    double minW = std::numeric_limits<double>::max();
    double maxW = std::numeric_limits<double>::lowest();
    double minU = std::numeric_limits<double>::max();
    double maxU = std::numeric_limits<double>::lowest();
    double minV = std::numeric_limits<double>::max();
    double maxV = std::numeric_limits<double>::lowest();
    for (const auto& point : line.points) {
        Vector3 local = projectToBasis(point, basis);
        projected.push_back({local.x, local.y});
        localPoints.push_back(local);
        minW = std::min(minW, local.z);
        maxW = std::max(maxW, local.z);
        minU = std::min(minU, local.x);
        maxU = std::max(maxU, local.x);
        minV = std::min(minV, local.y);
        maxV = std::max(maxV, local.y);
    }
    projected = closeLoop2D(projected);
    if (stages) {
        PartingSurfaceStage projectedStage;
        projectedStage.name = "projected";
        for (const auto& point : projected) {
            projectedStage.boundary.push_back(liftFromPlane(point, basis));
        }
        stages->push_back(projectedStage);
    }

    double planeExtent = std::max(maxU - minU, maxV - minV);
    double deviationRatio = planeExtent <= std::numeric_limits<double>::epsilon()
                                ? 0.0
                                : (maxW - minW) / planeExtent;
    // 默认优先平面分型面，只有起伏超过阈值时才允许非平面
    bool allowNonPlanar = !preferPlanarSurface || deviationRatio > nonPlanarDeviationRatio;

    // 3) 对投影曲线做平滑处理，去除高频噪声
    std::vector<Vector2> smoothed = smoothClosedCurve2D(projected, smoothingFactor);
    if (stages) {
        PartingSurfaceStage smoothedStage;
        smoothedStage.name = "smoothed";
        for (const auto& point : smoothed) {
            smoothedStage.boundary.push_back(liftFromPlane(point, basis));
        }
        stages->push_back(smoothedStage);
    }

    // 4) 检查投影曲线是否存在自交，必要时以凸包进行自动修复
    std::vector<Vector2> repaired = smoothed;
    if (hasSelfIntersection2D(closeLoop2D(repaired))) {
        std::vector<Vector2> hull = computeConvexHull2D(repaired);
        if (hull.size() >= 3) {
            repaired = closeLoop2D(hull);
            if (stages) {
                PartingSurfaceStage repairedStage;
                repairedStage.name = "repaired";
                for (const auto& point : repaired) {
                    repairedStage.boundary.push_back(liftFromPlane(point, basis));
                }
                stages->push_back(repairedStage);
            }
        }
    }

    // 5) 沿法向的垂直平面内向外延伸，保证分型面覆盖范围
    std::vector<Vector2> extended = extendCurve2D(repaired, extension);
    extended = closeLoop2D(extended);
    if (stages) {
        PartingSurfaceStage extendedStage;
        extendedStage.name = "extended";
        for (const auto& point : extended) {
            extendedStage.boundary.push_back(liftFromPlane(point, basis));
        }
        stages->push_back(extendedStage);
    }

    // 6) 必要时保留非平面起伏，否则输出平面分型面
    if (allowNonPlanar) {
        std::vector<Vector3> localLoop = closeLoopLocal(localPoints);
        std::vector<Vector3> smoothedLocal = smoothClosedCurve3D(localLoop, smoothingFactor);
        std::vector<Vector3> extendedLocal = extendLocalCurve(smoothedLocal, extension);
        extendedLocal = closeLoopLocal(extendedLocal);
        if (stages) {
            PartingSurfaceStage nonPlanarStage;
            nonPlanarStage.name = "non_planar";
            for (const auto& point : extendedLocal) {
                nonPlanarStage.boundary.push_back(liftFromBasis(point, basis));
            }
            stages->push_back(nonPlanarStage);
        }
        surface.boundary.clear();
        surface.boundary.reserve(extendedLocal.size());
        for (const auto& point : extendedLocal) {
            surface.boundary.push_back(liftFromBasis(point, basis));
        }
        return surface;
    }

    // 7) 将二维边界抬升回三维，作为平面分型面边界
    surface.boundary.clear();
    surface.boundary.reserve(extended.size());
    for (const auto& point : extended) {
        surface.boundary.push_back(liftFromPlane(point, basis));
    }
    return surface;
}

std::vector<PartingSurfaceStage> buildPartingSurfaceStages(const PartingLine& line,
                                                           const Vector3& direction,
                                                           double smoothingFactor,
                                                           double extension,
                                                           bool preferPlanarSurface,
                                                           double nonPlanarDeviationRatio) {
    std::vector<PartingSurfaceStage> stages;
    buildPartingSurface(line, direction, smoothingFactor, extension, preferPlanarSurface,
                        nonPlanarDeviationRatio, &stages);
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

std::vector<InterferenceIssue> checkPartingSurfaceQuality(const PartingSurface& surface,
                                                          const Vector3& direction) {
    std::vector<InterferenceIssue> issues;
    if (surface.boundary.size() < 3) {
        issues.push_back({"Parting surface has insufficient boundary points.", 0.9});
        return issues;
    }

    Vector3 centroid{};
    for (const auto& point : surface.boundary) {
        centroid += point;
    }
    centroid = centroid / static_cast<double>(surface.boundary.size());
    PlaneBasis basis = buildPlaneBasis(centroid, direction);

    std::vector<Vector2> projected;
    projected.reserve(surface.boundary.size());
    for (const auto& point : surface.boundary) {
        projected.push_back(projectToPlane(point, basis));
    }

    if (distance2D(projected.front(), projected.back()) > kClosureTolerance) {
        issues.push_back({"Parting surface boundary is not closed.", 0.7});
    }

    std::vector<Vector2> loop = closeLoop2D(projected);
    if (hasSelfIntersection2D(loop)) {
        issues.push_back({"Parting surface boundary self-intersects after projection.", 0.8});
    }

    if (loop.size() >= 4) {
        double minAngle = 180.0;
        for (std::size_t i = 1; i + 1 < loop.size(); ++i) {
            Vector2 prev = loop[i - 1];
            Vector2 curr = loop[i];
            Vector2 next = loop[i + 1];
            Vector2 v1 = prev - curr;
            Vector2 v2 = next - curr;
            double denom = std::sqrt(v1.x * v1.x + v1.y * v1.y) *
                           std::sqrt(v2.x * v2.x + v2.y * v2.y);
            if (denom <= std::numeric_limits<double>::epsilon()) {
                continue;
            }
            double cosine = std::clamp((v1.x * v2.x + v1.y * v2.y) / denom, -1.0, 1.0);
            double angle = std::acos(cosine) * 180.0 / kPi;
            minAngle = std::min(minAngle, angle);
        }
        if (minAngle < 20.0) {
            issues.push_back({"Parting surface boundary has sharp corners.", 0.5});
        }
    }

    Vector3 accumulatedNormal{};
    for (std::size_t i = 1; i + 1 < surface.boundary.size(); ++i) {
        Vector3 v1 = surface.boundary[i] - surface.boundary[0];
        Vector3 v2 = surface.boundary[i + 1] - surface.boundary[0];
        accumulatedNormal += cross(v1, v2);
    }
    if (length(accumulatedNormal) > std::numeric_limits<double>::epsilon()) {
        Vector3 normal = normalized(accumulatedNormal);
        double alignment = std::abs(dot(normal, normalized(direction)));
        if (alignment < 0.95) {
            issues.push_back({"Parting surface normal is not aligned with demold direction.", 0.6});
        }
    }

    return issues;
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
        region.id = regions.size();
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

double boundsVolume(const Bounds& bounds) {
    double dx = std::abs(bounds.max.x - bounds.min.x);
    double dy = std::abs(bounds.max.y - bounds.min.y);
    double dz = std::abs(bounds.max.z - bounds.min.z);
    return dx * dy * dz;
}

std::vector<CoreRegion> filterCoreRegions(const std::vector<CoreRegion>& cores,
                                          const Bounds& meshBounds,
                                          const AutoPartingSettings& settings,
                                          std::vector<std::size_t>* keptIds) {
    std::vector<CoreRegion> filtered;
    if (cores.empty()) {
        return filtered;
    }
    double meshVolume = boundsVolume(meshBounds);
    if (meshVolume <= std::numeric_limits<double>::epsilon()) {
        return filtered;
    }
    struct CoreScore {
        CoreRegion region;
        double volume{};
    };
    std::vector<CoreScore> scored;
    scored.reserve(cores.size());
    for (const auto& core : cores) {
        double volume = boundsVolume(core.bounds);
        double ratio = volume / meshVolume;
        if (ratio < settings.minCoreVolumeRatio) {
            continue;
        }
        scored.push_back({core, volume});
    }
    std::sort(scored.begin(), scored.end(),
              [](const CoreScore& left, const CoreScore& right) {
                  return left.volume > right.volume;
              });
    std::size_t limit = settings.maxCoreCount == 0 ? scored.size()
                                                   : std::min(settings.maxCoreCount, scored.size());
    filtered.reserve(limit);
    if (keptIds) {
        keptIds->clear();
    }
    for (std::size_t i = 0; i < limit; ++i) {
        filtered.push_back(scored[i].region);
        if (keptIds) {
            keptIds->push_back(scored[i].region.id);
        }
    }
    return filtered;
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
                CoreRegion coreRegion = region;
                coreRegion.pullDirection = bestFallback.direction;
                coreRegionsOut->push_back(coreRegion);
                obstacle.coreId = coreRegion.id;
                obstacle.corePreferred = true;
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

MoldAssembly buildMoldAssembly(const Mesh& mesh, const PartingSurface& surface,
                               const Vector3& direction, const AutoPartingSettings& settings,
                               const std::vector<CoreRegion>& cores) {
    MoldAssembly assembly;

    // 1) 计算与脱模方向对齐的包围盒，用于生成铸型毛坯
    Vector3 planeOrigin = surface.boundary.empty() ? computeCentroid(mesh) : Vector3{};
    if (!surface.boundary.empty()) {
        for (const auto& point : surface.boundary) {
            planeOrigin += point;
        }
        planeOrigin = planeOrigin / static_cast<double>(surface.boundary.size());
    }
    PlaneBasis basis = buildPlaneBasis(planeOrigin, direction);
    Bounds partLocalBounds = computeLocalBounds(mesh, basis);
    Bounds blankLocalBounds = expandLocalBounds(partLocalBounds, settings.moldBlankPadding);

    // 2) 按收缩率放大产品模型，用于型腔减料
    Vector3 localCenter{(partLocalBounds.min.x + partLocalBounds.max.x) / 2.0,
                        (partLocalBounds.min.y + partLocalBounds.max.y) / 2.0,
                        (partLocalBounds.min.z + partLocalBounds.max.z) / 2.0};
    Bounds scaledLocalBounds = scaleLocalBounds(partLocalBounds, localCenter,
                                                1.0 + settings.shrinkageFactor);

    // 3) 以分型面所在平面切分毛坯，得到上下型
    double splitCoordinate = 0.0;
    Bounds upperBlankLocal = blankLocalBounds;
    Bounds lowerBlankLocal = blankLocalBounds;
    upperBlankLocal.min.z = std::max(upperBlankLocal.min.z, splitCoordinate);
    lowerBlankLocal.max.z = std::min(lowerBlankLocal.max.z, splitCoordinate);

    Bounds upperCavityLocal = scaledLocalBounds;
    Bounds lowerCavityLocal = scaledLocalBounds;
    upperCavityLocal.min.z = std::max(upperCavityLocal.min.z, splitCoordinate);
    lowerCavityLocal.max.z = std::min(lowerCavityLocal.max.z, splitCoordinate);

    MoldBlock upper;
    upper.role = "UpperMold";
    upper.bounds = localBoundsToWorld(upperBlankLocal, basis);
    upper.cavityBounds = localBoundsToWorld(upperCavityLocal, basis);
    upper.pullDirection = normalized(direction);

    MoldBlock lower;
    lower.role = "LowerMold";
    lower.bounds = localBoundsToWorld(lowerBlankLocal, basis);
    lower.cavityBounds = localBoundsToWorld(lowerCavityLocal, basis);
    lower.pullDirection = normalized(direction * -1.0);

    assembly.blocks.push_back(upper);
    assembly.blocks.push_back(lower);
    assembly.overallBounds = upper.bounds;
    assembly.overallBounds.min.x = std::min(assembly.overallBounds.min.x, lower.bounds.min.x);
    assembly.overallBounds.min.y = std::min(assembly.overallBounds.min.y, lower.bounds.min.y);
    assembly.overallBounds.min.z = std::min(assembly.overallBounds.min.z, lower.bounds.min.z);
    assembly.overallBounds.max.x = std::max(assembly.overallBounds.max.x, lower.bounds.max.x);
    assembly.overallBounds.max.y = std::max(assembly.overallBounds.max.y, lower.bounds.max.y);
    assembly.overallBounds.max.z = std::max(assembly.overallBounds.max.z, lower.bounds.max.z);

    // 4) 根据倒扣区域生成砂芯、芯头和芯座的几何范围
    for (const auto& core : cores) {
        CoreInsert insert;
        insert.pullDirection = length(core.pullDirection) > std::numeric_limits<double>::epsilon()
                                   ? core.pullDirection
                                   : direction;
        insert.bodyBounds = expandBounds(core.bounds, settings.moldClearance);
        insert.headBounds = extendBoundsAlongDirection(insert.bodyBounds, insert.pullDirection,
                                                       settings.coreHeadLength);
        insert.seatBounds = expandBounds(insert.headBounds, settings.coreSeatClearance);
        assembly.cores.push_back(insert);
    }

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
                                                 const PartingSurface& surface,
                                                 const DemoldEvaluation& evaluation,
                                                 const MoldAssembly& assembly) {
    std::vector<InterferenceIssue> issues;
    if (line.points.empty()) {
        issues.push_back({"Parting line extraction produced no boundary points.", 0.8});
    }
    std::vector<InterferenceIssue> surfaceIssues =
        checkPartingSurfaceQuality(surface, evaluation.direction);
    issues.insert(issues.end(), surfaceIssues.begin(), surfaceIssues.end());
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
    for (const auto& core : assembly.cores) {
        if (!boundsContains(assembly.overallBounds, core.seatBounds)) {
            issues.push_back({"Core seat exceeds mold blank bounds, adjust core head or padding.",
                              0.5});
        }
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
                                                            result.demold.direction,
                                                            settings.smoothingFactor,
                                                            settings.partingSurfaceExtension,
                                                            settings.preferPlanarSurface,
                                                            settings.nonPlanarDeviationRatio);
    if (!result.partingSurfaceStages.empty()) {
        result.partingSurface.boundary = result.partingSurfaceStages.back().boundary;
    } else {
        result.partingSurface = buildPartingSurface(result.partingLine, result.demold.direction,
                                                    settings.smoothingFactor,
                                                    settings.partingSurfaceExtension,
                                                    settings.preferPlanarSurface,
                                                    settings.nonPlanarDeviationRatio, nullptr);
    }
    result.maxContour = identifyMaxContour(result.partingLine, result.demold.direction);
    result.split = splitMesh(result.cleanedMesh, result.demold.direction);
    std::vector<CoreRegion> undercutRegions = detectCoreRegions(result.cleanedMesh,
                                                                result.demold.direction,
                                                                settings.draftAngleDegrees);
    result.separability = evaluateSeparability(result.cleanedMesh, result.demold, undercutRegions,
                                               settings, &result.cores);
    Bounds meshBounds = computeBounds(result.cleanedMesh);
    std::vector<std::size_t> keptCoreIds;
    // 根据体积占比与数量限制筛减砂芯，尽量减少砂芯数量
    result.cores = filterCoreRegions(result.cores, meshBounds, settings, &keptCoreIds);
    if (!keptCoreIds.empty()) {
        std::unordered_set<std::size_t> keptSet(keptCoreIds.begin(), keptCoreIds.end());
        for (auto& obstacle : result.separability.obstacles) {
            if (obstacle.type != ObstacleType::CoreCandidate || !obstacle.corePreferred) {
                continue;
            }
            if (keptSet.count(obstacle.coreId) == 0) {
                obstacle.type = ObstacleType::MultiDirection;
                obstacle.corePreferred = false;
            }
        }
    } else if (result.cores.empty()) {
        for (auto& obstacle : result.separability.obstacles) {
            if (obstacle.type == ObstacleType::CoreCandidate) {
                obstacle.type = ObstacleType::MultiDirection;
                obstacle.corePreferred = false;
            }
        }
    }
    result.moldAssembly = buildMoldAssembly(result.cleanedMesh, result.partingSurface,
                                            result.demold.direction, settings, result.cores);
    result.strategies = buildStrategyOptions(result.separability, result.cores.size());
    result.issues = checkInterference(result.cleanedMesh, result.split, result.partingLine,
                                      result.partingSurface, result.demold, result.moldAssembly);
    return result;
}

}  // namespace casting
