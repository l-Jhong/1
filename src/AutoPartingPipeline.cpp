#include "AutoPartingPipeline.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>
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
        // Section clustering tolerance for grouping parting-line points by local z.
        // The ratio scales with part thickness along demold direction; the minimum
        // value keeps grouping stable for very thin parts.
        constexpr double kSectionClusteringToleranceRatio = 0.05;
        constexpr double kMinSectionClusteringTolerance = 1e-4;
        constexpr double kSplitPlaneContainmentTolerance = 1e-6;
        constexpr double kSplitPlaneMinThicknessRatio = 1e-3;
        // Sampling slices used to estimate max contour; tuned to balance stability and runtime.
        constexpr std::size_t kContourSliceCount = 33;
        constexpr double kSliceTieAreaRatio = 0.03;
        constexpr double kSliceAreaEpsilon = 1e-8;
        constexpr double kSliceIntersectionTolerance = 1e-8;
        constexpr double kStraightShapeLengthRatioThreshold = 5.0;
        constexpr double kStraightShapePrimarySecondaryRatioThreshold = 2.0;
        constexpr double kStraightBaselineCurvatureDeg = 15.0;
        constexpr double kCurvedBaselineCurvatureDeg = 45.0;
        constexpr double kCastSteelMinWallThickness = 6.0;
        constexpr double kCastIronMinWallThickness = 8.0;
        constexpr double kThroughHoleNoCastDepthWidthRatio = 4.0;
        constexpr double kBlindHoleNoCastDepthWidthRatio = 3.0;
        constexpr double kMassProductionNoCastHoleDiameter = 12.0;
        constexpr double kBatchProductionNoCastHoleDiameter = 15.0;
        constexpr double kSmallBatchNoCastHoleDiameter = 30.0;
        constexpr int kSandCoreColorVariants = 6;
        constexpr int kSandCoreBaseLayer = 90;
        constexpr int kSandCoreLayerVariants = 10;
        constexpr std::size_t kVisualHashMultiplier = 2654435761U;

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
                ? Vector3{ 1.0, 0.0, 0.0 }
            : Vector3{ 0.0, 1.0, 0.0 };
            basis.axisU = normalized(cross(basis.normal, reference));
            if (length(basis.axisU) <= std::numeric_limits<double>::epsilon()) {
                basis.axisU = { 1.0, 0.0, 0.0 };
            }
            basis.axisV = normalized(cross(basis.normal, basis.axisU));
            return basis;
        }

        Vector2 projectToPlane(const Vector3& point, const PlaneBasis& basis) {
            Vector3 offset = point - basis.origin;
            return { dot(offset, basis.axisU), dot(offset, basis.axisV) };
        }

        Vector3 liftFromPlane(const Vector2& point, const PlaneBasis& basis) {
            return basis.origin + basis.axisU * point.x + basis.axisV * point.y;
        }

        Vector3 projectToBasis(const Vector3& point, const PlaneBasis& basis) {
            Vector3 offset = point - basis.origin;
            return { dot(offset, basis.axisU), dot(offset, basis.axisV), dot(offset, basis.normal) };
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
            Vector2 first{ closed.front().x, closed.front().y };
            Vector2 last{ closed.back().x, closed.back().y };
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
                Vector2 average{ (prev.x + curr.x + next.x) / 3.0,
                                (prev.y + curr.y + next.y) / 3.0 };
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
                Vector3 average{ (prev.x + curr.x + next.x) / 3.0,
                                (prev.y + curr.y + next.y) / 3.0,
                                (prev.z + curr.z + next.z) / 3.0 };
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
                Vector2 direction{ offset.x / length, offset.y / length };
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
                centroid += Vector2{ point.x, point.y };
            }
            centroid = centroid / static_cast<double>(points.size());
            std::vector<Vector3> extended(points.size());
            for (std::size_t i = 0; i < points.size(); ++i) {
                Vector2 offset{ points[i].x - centroid.x, points[i].y - centroid.y };
                double length = std::sqrt(offset.x * offset.x + offset.y * offset.y);
                if (length <= std::numeric_limits<double>::epsilon()) {
                    extended[i] = points[i];
                    continue;
                }
                Vector2 direction{ offset.x / length, offset.y / length };
                extended[i] = { points[i].x + direction.x * extension,
                               points[i].y + direction.y * extension,
                               points[i].z };
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

        bool pointInPolygon2D(const Vector2& point, const std::vector<Vector2>& polygon) {
            if (polygon.size() < 3) {
                return false;
            }
            bool inside = false;
            for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
                const Vector2& a = polygon[i];
                const Vector2& b = polygon[j];
                bool intersects = ((a.y > point.y) != (b.y > point.y)) &&
                    (point.x < (b.x - a.x) * (point.y - a.y) / ((b.y - a.y) + 1e-12) + a.x);
                if (intersects) {
                    inside = !inside;
                }
            }
            return inside;
        }

        std::size_t countPolygonIntersections2D(const std::vector<Vector2>& lhs,
            const std::vector<Vector2>& rhs) {
            if (lhs.size() < 2 || rhs.size() < 2) {
                return 0;
            }
            std::size_t intersections = 0;
            for (std::size_t i = 0; i + 1 < lhs.size(); ++i) {
                for (std::size_t j = 0; j + 1 < rhs.size(); ++j) {
                    if (segmentsIntersect2D(lhs[i], lhs[i + 1], rhs[j], rhs[j + 1])) {
                        ++intersections;
                    }
                }
            }
            return intersections;
        }

        Bounds computeLocalBounds(const Mesh& mesh, const PlaneBasis& basis) {
            Bounds bounds{};
            bounds.min = { std::numeric_limits<double>::max(),
                          std::numeric_limits<double>::max(),
                          std::numeric_limits<double>::max() };
            bounds.max = { std::numeric_limits<double>::lowest(),
                          std::numeric_limits<double>::lowest(),
                          std::numeric_limits<double>::lowest() };
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
            bounds.min = { std::numeric_limits<double>::max(),
                          std::numeric_limits<double>::max(),
                          std::numeric_limits<double>::max() };
            bounds.max = { std::numeric_limits<double>::lowest(),
                          std::numeric_limits<double>::lowest(),
                          std::numeric_limits<double>::lowest() };
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
            }
            else {
                extended.min.x -= length * std::abs(dir.x);
            }
            if (dir.y >= 0.0) {
                extended.max.y += length * std::abs(dir.y);
            }
            else {
                extended.min.y -= length * std::abs(dir.y);
            }
            if (dir.z >= 0.0) {
                extended.max.z += length * std::abs(dir.z);
            }
            else {
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

        std::vector<Vector2> deduplicatePoints2D(const std::vector<Vector2>& points, double tolerance) {
            std::vector<Vector2> unique;
            unique.reserve(points.size());
            for (const auto& point : points) {
                bool duplicated = false;
                for (const auto& existing : unique) {
                    if (distance2D(point, existing) <= tolerance) {
                        duplicated = true;
                        break;
                    }
                }
                if (!duplicated) {
                    unique.push_back(point);
                }
            }
            return unique;
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
                return { dir, kInvalidScore, 0.0, 0.0 };
            }
            double visibilityRatio = visibleArea / totalArea;
            double undercutRatio = undercutArea / totalArea;
            double score = visibilityRatio - undercutPenalty * undercutRatio -
                visibilityPenalty * (1.0 - visibilityRatio);
            return { dir, score, visibilityRatio, undercutRatio };
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
            DemoldEvaluation best{ candidates.front(), kInvalidScore, 0.0, 0.0 };
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
                std::array<std::size_t, 3> verts{ tri.v0, tri.v1, tri.v2 };
                for (std::size_t i = 0; i < 3; ++i) {
                    std::size_t a = verts[i];
                    std::size_t b = verts[(i + 1) % 3];
                    EdgeKey key{ std::min(a, b), std::max(a, b) };
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
                line.points.push_back({ (a.x + b.x) / 2.0, (a.y + b.y) / 2.0, (a.z + b.z) / 2.0 });
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
                ? Vector3{ 1.0, 0.0, 0.0 }
            : Vector3{ 0.0, 1.0, 0.0 };
            Vector3 axisU = normalized(cross(normal, reference));
            if (length(axisU) <= std::numeric_limits<double>::epsilon()) {
                axisU = { 1.0, 0.0, 0.0 };
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

            // 1) Use parting-line centroid as origin and build a reference plane perpendicular to demold direction.
            Vector3 centroid{};
            for (const auto& point : line.points) {
                centroid += point;
            }
            centroid = centroid / static_cast<double>(line.points.size());
            PlaneBasis basis = buildPlaneBasis(centroid, direction);

            if (stages) {
                stages->push_back({ "Original", line.points });
            }

            // 2) Project the parting line onto the reference plane to get a 2D curve.
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
                projected.push_back({ local.x, local.y });
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
                projectedStage.name = "Projected";
                for (const auto& point : projected) {
                    projectedStage.boundary.push_back(liftFromPlane(point, basis));
                }
                stages->push_back(projectedStage);
            }

            double planeExtent = std::max(maxU - minU, maxV - minV);
            double deviationRatio = planeExtent <= std::numeric_limits<double>::epsilon()
                ? 0.0
                : (maxW - minW) / planeExtent;
            // Prefer planar parting surface by default; allow non-planar only if deviation exceeds threshold.
            bool allowNonPlanar = !preferPlanarSurface || deviationRatio > nonPlanarDeviationRatio;

            // 3) Smooth the projected curve to remove high-frequency noise.
            std::vector<Vector2> smoothed = smoothClosedCurve2D(projected, smoothingFactor);
            if (stages) {
                PartingSurfaceStage smoothedStage;
                smoothedStage.name = "Smoothed";
                for (const auto& point : smoothed) {
                    smoothedStage.boundary.push_back(liftFromPlane(point, basis));
                }
                stages->push_back(smoothedStage);
            }

            // 4) Check whether projected curve self-intersects; repair with convex hull when needed.
            std::vector<Vector2> repaired = smoothed;
            if (hasSelfIntersection2D(closeLoop2D(repaired))) {
                std::vector<Vector2> hull = computeConvexHull2D(repaired);
                if (hull.size() >= 3) {
                    repaired = closeLoop2D(hull);
                    if (stages) {
                        PartingSurfaceStage repairedStage;
                        repairedStage.name = "Repaired";
                        for (const auto& point : repaired) {
                            repairedStage.boundary.push_back(liftFromPlane(point, basis));
                        }
                        stages->push_back(repairedStage);
                    }
                }
            }

            // 5) Extend outward in the normal-perpendicular plane to guarantee coverage.
            std::vector<Vector2> extended = extendCurve2D(repaired, extension);
            extended = closeLoop2D(extended);
            if (stages) {
                PartingSurfaceStage extendedStage;
                extendedStage.name = "Expanded";
                for (const auto& point : extended) {
                    extendedStage.boundary.push_back(liftFromPlane(point, basis));
                }
                stages->push_back(extendedStage);
            }

            // 6) Keep non-planar variation when needed; otherwise output planar parting surface.
            if (allowNonPlanar) {
                std::vector<Vector3> localLoop = closeLoopLocal(localPoints);
                std::vector<Vector3> smoothedLocal = smoothClosedCurve3D(localLoop, smoothingFactor);
                std::vector<Vector3> extendedLocal = extendLocalCurve(smoothedLocal, extension);
                extendedLocal = closeLoopLocal(extendedLocal);
                if (stages) {
                    PartingSurfaceStage nonPlanarStage;
                    nonPlanarStage.name = "Non-planar";
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

            // 7) Lift the 2D boundary back to 3D as the planar parting-surface boundary.
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

        // Builds the parting surface directly from the max contour face.
        //
        // Inputs:
        //   contour   – max contour face (convex hull of the parting-line projected onto
        //               the plane perpendicular to 'direction'); its boundary points all
        //               lie at z_local ≈ 0 in the plane whose normal is 'direction'.
        //   direction – demold / draft direction (used as the plane normal).
        //   extension – outward extension distance to give the surface adequate coverage.
        //   stages    – optional collector for intermediate boundary snapshots.
        //
        // Guarantee: every point in the returned surface.boundary lies on the flat plane
        // through contour.centroid with normal = normalized(direction).  This satisfies
        // the requirement that the parting surface must be perpendicular to the draft axis.
        PartingSurface buildPlanarPartingSurface(const ContourFace& contour,
            const Vector3& direction,
            double extension,
            std::vector<PartingSurfaceStage>* stages) {
            PartingSurface surface;
            if (contour.boundary.size() < 3) {
                return surface;
            }
            // Build a plane basis centred at the max-contour centroid with 'direction' as normal.
            PlaneBasis basis = buildPlaneBasis(contour.centroid, direction);

            if (stages) {
                stages->push_back({ "Max contour", contour.boundary });
            }

            // Project the contour boundary to the 2D plane (z ≈ 0 by construction).
            std::vector<Vector2> projected;
            projected.reserve(contour.boundary.size());
            for (const auto& point : contour.boundary) {
                projected.push_back(projectToPlane(point, basis));
            }
            projected = closeLoop2D(projected);

            // Extend outward so the parting surface fully covers the mold blank footprint.
            std::vector<Vector2> extended = extendCurve2D(projected, extension);
            extended = closeLoop2D(extended);

            if (stages) {
                PartingSurfaceStage extStage;
                extStage.name = "Expanded";
                for (const auto& point : extended) {
                    extStage.boundary.push_back(liftFromPlane(point, basis));
                }
                stages->push_back(extStage);
            }

            // Lift back to 3D — all points lie on the flat plane (local z = 0).
            surface.boundary.reserve(extended.size());
            for (const auto& point : extended) {
                surface.boundary.push_back(liftFromPlane(point, basis));
            }
            return surface;
        }

        ContourFace identifyMaxContour(const Mesh& mesh, const PartingLine& line, const Vector3& direction) {
            ContourFace contour;
            Vector3 basisOrigin{};
            if (!line.points.empty()) {
                for (const auto& point : line.points) {
                    basisOrigin += point;
                }
                basisOrigin = basisOrigin / static_cast<double>(line.points.size());
            }
            else {
                basisOrigin = computeCentroid(mesh);
            }
            PlaneBasis basis = buildPlaneBasis(basisOrigin, direction);

            std::vector<Vector3> localVertices;
            localVertices.reserve(mesh.vertices.size());
            double minW = std::numeric_limits<double>::max();
            double maxW = std::numeric_limits<double>::lowest();
            for (const auto& point : mesh.vertices) {
                Vector3 local = projectToBasis(point, basis);
                localVertices.push_back(local);
                minW = std::min(minW, local.z);
                maxW = std::max(maxW, local.z);
            }

            if (localVertices.empty()) {
                return contour;
            }

            std::vector<Vector2> bestHull;
            double bestArea = 0.0;
            double bestW = (minW + maxW) * 0.5;

            struct SliceCandidate {
                std::vector<Vector2> hull;
                double area = 0.0;
                double w = 0.0;
                double balance = std::numeric_limits<double>::max();
                int index = -1;
            };

            auto collectSliceCandidate = [&](double w, int index, SliceCandidate* out) -> bool {
                if (!out) {
                    return false;
                }
                std::vector<Vector2> intersections;
                intersections.reserve(mesh.triangles.size() * 2);
                for (const auto& triangle : mesh.triangles) {
                    const Vector3& a = localVertices.at(triangle.v0);
                    const Vector3& b = localVertices.at(triangle.v1);
                    const Vector3& c = localVertices.at(triangle.v2);
                    std::array<std::pair<Vector3, Vector3>, 3> edges = {
                        std::make_pair(a, b),
                        std::make_pair(b, c),
                        std::make_pair(c, a)
                    };
                    for (const auto& edge : edges) {
                        const Vector3& p0 = edge.first;
                        const Vector3& p1 = edge.second;
                        double d0 = p0.z - w;
                        double d1 = p1.z - w;
                        bool on0 = std::abs(d0) <= kSliceIntersectionTolerance;
                        bool on1 = std::abs(d1) <= kSliceIntersectionTolerance;
                        if (on0 && on1) {
                            intersections.push_back({ p0.x, p0.y });
                            intersections.push_back({ p1.x, p1.y });
                            continue;
                        }
                        if (on0) {
                            intersections.push_back({ p0.x, p0.y });
                            continue;
                        }
                        if (on1) {
                            intersections.push_back({ p1.x, p1.y });
                            continue;
                        }
                        if ((d0 < 0.0 && d1 > 0.0) || (d0 > 0.0 && d1 < 0.0)) {
                            double t = d0 / (d0 - d1);
                            intersections.push_back({
                                p0.x + (p1.x - p0.x) * t,
                                p0.y + (p1.y - p0.y) * t
                                });
                        }
                    }
                }
                intersections = deduplicatePoints2D(intersections, kSliceIntersectionTolerance * 10.0);
                if (intersections.size() < 3) {
                    return false;
                }
                std::vector<Vector2> hull = computeConvexHull2D(intersections);
                if (hull.size() < 3) {
                    return false;
                }
                std::vector<Vector2> closed = closeLoop2D(hull);
                if (hasSelfIntersection2D(closed)) {
                    return false;
                }
                double area = polygonArea2D(hull);
                if (area <= kSliceAreaEpsilon) {
                    return false;
                }
                std::size_t upperCount = 0;
                std::size_t lowerCount = 0;
                for (const auto& vertex : localVertices) {
                    if (vertex.z >= w) {
                        ++upperCount;
                    }
                    else {
                        ++lowerCount;
                    }
                }
                out->hull = std::move(hull);
                out->area = area;
                out->w = w;
                out->index = index;
                out->balance = std::abs(static_cast<double>(upperCount) - static_cast<double>(lowerCount));
                return true;
            };

            if (mesh.triangles.size() >= 3 &&
                (maxW - minW) > std::numeric_limits<double>::epsilon()) {
                double midW = (minW + maxW) * 0.5;
                SliceCandidate bestSlice;
                bool hasSlice = false;
                for (std::size_t i = 0; i < kContourSliceCount; ++i) {
                    double ratio = (kContourSliceCount == 1)
                        ? 0.0
                        : static_cast<double>(i) / static_cast<double>(kContourSliceCount - 1);
                    double w = minW + (maxW - minW) * ratio;
                    SliceCandidate candidate;
                    if (!collectSliceCandidate(w, static_cast<int>(i), &candidate)) {
                        continue;
                    }
                    if (!hasSlice) {
                        bestSlice = std::move(candidate);
                        hasSlice = true;
                        continue;
                    }
                    double areaTolerance =
                        std::max(bestSlice.area, candidate.area) * kSliceTieAreaRatio;
                    if (candidate.area > bestSlice.area + areaTolerance) {
                        bestSlice = std::move(candidate);
                        continue;
                    }
                    if (std::abs(candidate.area - bestSlice.area) <= areaTolerance) {
                        double candidateMidDist = std::abs(candidate.w - midW);
                        double bestMidDist = std::abs(bestSlice.w - midW);
                        if (candidateMidDist < bestMidDist - kSliceIntersectionTolerance) {
                            bestSlice = std::move(candidate);
                            continue;
                        }
                        if (std::abs(candidateMidDist - bestMidDist) <= kSliceIntersectionTolerance &&
                            candidate.balance < bestSlice.balance) {
                            bestSlice = std::move(candidate);
                        }
                    }
                }
                if (hasSlice) {
                    bestHull = bestSlice.hull;
                    bestArea = bestSlice.area;
                    bestW = std::clamp(bestSlice.w, minW, maxW);
                    contour.selectedFromSlice = true;
                    contour.selectedSliceIndex = bestSlice.index;
                    contour.selectedSliceW = bestW;
                }
            }

            if (bestHull.empty()) {
                contour.fallbackUsed = true;
                if (line.points.size() >= 3) {
                    struct SectionCluster {
                        std::vector<Vector2> uv;
                        double sumW = 0.0;
                    };

                    std::vector<Vector3> localPoints;
                    localPoints.reserve(line.points.size());
                    for (const auto& point : line.points) {
                        localPoints.push_back(projectToBasis(point, basis));
                    }

                    double wRange = maxW - minW;
                    double sectionTolerance = std::max(kMinSectionClusteringTolerance,
                        wRange * kSectionClusteringToleranceRatio);
                    std::vector<SectionCluster> clusters;
                    for (const auto& local : localPoints) {
                        bool assigned = false;
                        for (auto& cluster : clusters) {
                            double meanW = cluster.uv.empty()
                                ? local.z
                                : (cluster.sumW / static_cast<double>(cluster.uv.size()));
                            if (std::abs(local.z - meanW) <= sectionTolerance) {
                                cluster.uv.push_back({ local.x, local.y });
                                cluster.sumW += local.z;
                                assigned = true;
                                break;
                            }
                        }
                        if (!assigned) {
                            SectionCluster cluster;
                            cluster.uv.push_back({ local.x, local.y });
                            cluster.sumW = local.z;
                            clusters.push_back(cluster);
                        }
                    }

                    for (const auto& cluster : clusters) {
                        if (cluster.uv.size() < 3) {
                            continue;
                        }
                        std::vector<Vector2> hull = computeConvexHull2D(cluster.uv);
                        if (hull.size() < 3) {
                            continue;
                        }
                        double area = polygonArea2D(hull);
                        if (area > bestArea) {
                            bestArea = area;
                            bestHull = std::move(hull);
                            bestW = cluster.sumW / static_cast<double>(cluster.uv.size());
                        }
                    }
                }

                if (bestHull.empty()) {
                    SliceCandidate centerCandidate;
                    if (collectSliceCandidate((minW + maxW) * 0.5, -1, &centerCandidate)) {
                        bestHull = centerCandidate.hull;
                        bestArea = centerCandidate.area;
                        bestW = centerCandidate.w;
                    }
                }
            }

            if (bestHull.size() < 3) {
                return contour;
            }

            contour.boundary.reserve(bestHull.size() + 1);
            for (const auto& point : bestHull) {
                contour.boundary.push_back(liftFromBasis({ point.x, point.y, bestW }, basis));
            }
            if (!contour.boundary.empty()) {
                Vector3 closureDelta = contour.boundary.front() - contour.boundary.back();
                if (length(closureDelta) > kClosureTolerance) {
                    contour.boundary.push_back(contour.boundary.front());
                }
            }

            contour.area = bestArea;
            contour.centroid = {};
            for (const auto& point : contour.boundary) {
                contour.centroid += point;
            }
            contour.centroid = contour.centroid / static_cast<double>(contour.boundary.size());
            contour.normal = normalized(direction);
            contour.selectedSliceW = bestW;
            return contour;
        }

        std::vector<InterferenceIssue> checkPartingSurfaceQuality(const PartingSurface& surface,
            const Vector3& direction) {
            std::vector<InterferenceIssue> issues;
            if (surface.boundary.size() < 3) {
                issues.push_back({ "Insufficient parting-surface boundary points.", 0.9 });
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
                issues.push_back({ "Parting-surface boundary is not closed.", 0.7 });
            }

            std::vector<Vector2> loop = closeLoop2D(projected);
            if (hasSelfIntersection2D(loop)) {
                issues.push_back({ "Parting-surface boundary self-intersects after projection.", 0.8 });
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
                    issues.push_back({ "Parting-surface boundary has sharp corners.", 0.5 });
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
                    issues.push_back({ "Parting-surface normal is inconsistent with demold direction.", 0.6 });
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
                }
                else {
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
                std::array<std::size_t, 3> verts{ tri.v0, tri.v1, tri.v2 };
                for (std::size_t i = 0; i < 3; ++i) {
                    std::size_t a = verts[i];
                    std::size_t b = verts[(i + 1) % 3];
                    EdgeKey key{ std::min(a, b), std::max(a, b) };
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

        struct RegionSliceAnalysis {
            bool hasHook = false;
            double maxCurvatureDeg = 0.0;
            double lengthDiameterRatio = 0.0;
            double minWallThickness = 0.0;
            std::vector<double> segmentationPositions;
        };

        std::vector<Vector3> collectRegionVertices(const Mesh& mesh, const CoreRegion& region) {
            std::vector<Vector3> points;
            points.reserve(region.triangleIndices.size() * 3);
            for (std::size_t triIndex : region.triangleIndices) {
                const Triangle& tri = mesh.triangles.at(triIndex);
                points.push_back(mesh.vertices.at(tri.v0));
                points.push_back(mesh.vertices.at(tri.v1));
                points.push_back(mesh.vertices.at(tri.v2));
            }
            return points;
        }

        std::vector<Vector2> buildProjectedHull(const std::vector<Vector3>& points, const PlaneBasis& basis) {
            std::vector<Vector2> projected;
            projected.reserve(points.size());
            for (const auto& point : points) {
                projected.push_back(projectToPlane(point, basis));
            }
            projected = deduplicatePoints2D(projected, kSliceIntersectionTolerance * 10.0);
            if (projected.size() < 3) {
                return {};
            }
            std::vector<Vector2> hull = computeConvexHull2D(projected);
            if (hull.size() < 3) {
                return {};
            }
            return closeLoop2D(hull);
        }

        double rangeAlongDirection(const std::vector<Vector3>& points, const Vector3& direction) {
            if (points.empty()) {
                return 0.0;
            }
            Vector3 dir = normalized(direction);
            double minDot = std::numeric_limits<double>::max();
            double maxDot = std::numeric_limits<double>::lowest();
            for (const auto& point : points) {
                double projection = dot(point, dir);
                minDot = std::min(minDot, projection);
                maxDot = std::max(maxDot, projection);
            }
            return std::max(0.0, maxDot - minDot);
        }

        double estimateMinWallThickness(const Bounds& regionBounds, const Bounds& meshBounds) {
            double d0 = std::abs(regionBounds.min.x - meshBounds.min.x);
            double d1 = std::abs(meshBounds.max.x - regionBounds.max.x);
            double d2 = std::abs(regionBounds.min.y - meshBounds.min.y);
            double d3 = std::abs(meshBounds.max.y - regionBounds.max.y);
            double d4 = std::abs(regionBounds.min.z - meshBounds.min.z);
            double d5 = std::abs(meshBounds.max.z - regionBounds.max.z);
            return std::min({ d0, d1, d2, d3, d4, d5 });
        }

        RegionSliceAnalysis analyzeRegionSlices(const Mesh& mesh,
            const CoreRegion& region,
            const Vector3& sliceDirection,
            const Bounds& meshBounds) {
            RegionSliceAnalysis analysis;
            std::vector<Vector3> points = collectRegionVertices(mesh, region);
            if (points.size() < 3) {
                analysis.minWallThickness = estimateMinWallThickness(region.bounds, meshBounds);
                return analysis;
            }

            Vector3 centroid{};
            for (const auto& point : points) {
                centroid += point;
            }
            centroid = centroid / static_cast<double>(points.size());
            PlaneBasis basis = buildPlaneBasis(centroid, sliceDirection);

            struct SliceBin {
                double w{};
                std::vector<Vector2> uv;
            };

            double minW = std::numeric_limits<double>::max();
            double maxW = std::numeric_limits<double>::lowest();
            std::vector<Vector3> local;
            local.reserve(points.size());
            for (const auto& point : points) {
                Vector3 p = projectToBasis(point, basis);
                local.push_back(p);
                minW = std::min(minW, p.z);
                maxW = std::max(maxW, p.z);
            }
            double span = std::max(0.0, maxW - minW);
            std::size_t slices = std::clamp<std::size_t>(static_cast<std::size_t>(std::ceil(span / 2.5)), 4, 24);
            double step = span <= kClosureTolerance ? 1.0 : span / static_cast<double>(slices);

            std::vector<SliceBin> bins(slices);
            for (std::size_t i = 0; i < slices; ++i) {
                bins[i].w = minW + (static_cast<double>(i) + 0.5) * step;
            }
            for (const auto& p : local) {
                std::size_t index = 0;
                if (step > kClosureTolerance) {
                    double normalizedW = (p.z - minW) / step;
                    index = std::min<std::size_t>(slices - 1,
                        static_cast<std::size_t>(std::max(0.0, std::floor(normalizedW))));
                }
                bins[index].uv.push_back({ p.x, p.y });
            }

            struct SliceMetric {
                double w{};
                double area{};
                Vector2 center{};
            };
            std::vector<SliceMetric> metrics;
            metrics.reserve(slices);
            for (auto& bin : bins) {
                bin.uv = deduplicatePoints2D(bin.uv, kSliceIntersectionTolerance * 10.0);
                if (bin.uv.size() < 3) {
                    continue;
                }
                std::vector<Vector2> hull = computeConvexHull2D(bin.uv);
                if (hull.size() < 3) {
                    continue;
                }
                double area = polygonArea2D(hull);
                if (area <= kSliceAreaEpsilon) {
                    continue;
                }
                Vector2 center{};
                for (const auto& uv : hull) {
                    center += uv;
                }
                center = center / static_cast<double>(hull.size());
                metrics.push_back({ bin.w, area, center });
            }

            if (metrics.size() < 2) {
                analysis.lengthDiameterRatio = 0.0;
                analysis.minWallThickness = estimateMinWallThickness(region.bounds, meshBounds);
                return analysis;
            }

            for (std::size_t i = 1; i + 1 < metrics.size(); ++i) {
                if (metrics[i].area > metrics[i - 1].area * 1.05 &&
                    metrics[i].area > metrics[i + 1].area * 1.05) {
                    analysis.hasHook = true;
                    analysis.segmentationPositions.push_back(metrics[i].w);
                }
            }

            for (std::size_t i = 1; i + 1 < metrics.size(); ++i) {
                Vector2 v1 = metrics[i].center - metrics[i - 1].center;
                Vector2 v2 = metrics[i + 1].center - metrics[i].center;
                double norm = std::sqrt(v1.x * v1.x + v1.y * v1.y) *
                    std::sqrt(v2.x * v2.x + v2.y * v2.y);
                if (norm <= std::numeric_limits<double>::epsilon()) {
                    continue;
                }
                double cosine = std::clamp((v1.x * v2.x + v1.y * v2.y) / norm, -1.0, 1.0);
                double angle = std::acos(cosine) * 180.0 / kPi;
                analysis.maxCurvatureDeg = std::max(analysis.maxCurvatureDeg, angle);
            }

            double avgArea = 0.0;
            for (const auto& metric : metrics) {
                avgArea += metric.area;
            }
            avgArea /= static_cast<double>(metrics.size());
            double equivalentDiameter = 2.0 * std::sqrt(std::max(avgArea, 0.0) / kPi);
            double height = std::max(0.0, metrics.back().w - metrics.front().w);
            analysis.lengthDiameterRatio =
                height / std::max(equivalentDiameter, kClosureTolerance);
            analysis.minWallThickness = estimateMinWallThickness(region.bounds, meshBounds);
            return analysis;
        }

        bool isDirectionDistinct(const Vector3& lhs, const Vector3& rhs, double minAngleDegrees) {
            double denom = length(lhs) * length(rhs);
            if (denom <= std::numeric_limits<double>::epsilon()) {
                return false;
            }
            double cosine = std::clamp(dot(lhs, rhs) / denom, -1.0, 1.0);
            double angle = std::acos(cosine) * 180.0 / kPi;
            return angle >= minAngleDegrees;
        }

        double boundsVolume(const Bounds& bounds);

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
                std::vector<std::size_t> stack{ index };
                visited[index] = true;
                Bounds bounds{};
                bounds.min = { std::numeric_limits<double>::max(),
                              std::numeric_limits<double>::max(),
                              std::numeric_limits<double>::max() };
                bounds.max = { std::numeric_limits<double>::lowest(),
                              std::numeric_limits<double>::lowest(),
                              std::numeric_limits<double>::lowest() };
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

        struct NonCastDecision {
            bool castFeature = true;
            std::string reason;
            std::string recommendation;
        };

        double smallHoleThresholdByBatch(ProductionBatch batch) {
            switch (batch) {
            case ProductionBatch::MassProduction:
                return kMassProductionNoCastHoleDiameter;
            case ProductionBatch::BatchProduction:
                return kBatchProductionNoCastHoleDiameter;
            case ProductionBatch::SmallBatch:
                return kSmallBatchNoCastHoleDiameter;
            default:
                return kBatchProductionNoCastHoleDiameter;
            }
        }

        NonCastDecision evaluateNonCastFeature(const CoreRegion& region,
            double width,
            double depth,
            double length,
            double curvatureDeg,
            ProductionBatch batch) {
            NonCastDecision decision;
            double eps = std::max(kClosureTolerance, 1e-6);
            double depthWidth = depth / std::max(width, eps);
            double lengthWidth = length / std::max(width, eps);

            bool isOpen = region.openingCount >= 1;
            bool throughLike = region.openingCount >= 2;
            bool blindLike = region.openingCount == 1;
            bool nonRoundSlot = width < 20.0 && lengthWidth > 2.0;

            if (throughLike && depthWidth > kThroughHoleNoCastDepthWidthRatio) {
                decision.castFeature = false;
                decision.reason = "通孔长径比超过" +
                    std::to_string(kThroughHoleNoCastDepthWidthRatio) + "，按不铸出处理。";
            } else if (blindLike && depthWidth > kBlindHoleNoCastDepthWidthRatio) {
                decision.castFeature = false;
                decision.reason = "盲孔深径比超过" +
                    std::to_string(kBlindHoleNoCastDepthWidthRatio) + "，按不铸出处理。";
            } else if (isOpen && width < smallHoleThresholdByBatch(batch)) {
                decision.castFeature = false;
                decision.reason = "孔径低于当前生产批量的不铸出阈值。";
            } else if (nonRoundSlot && width < 15.0 && depth < 10.0) {
                decision.castFeature = false;
                decision.reason = "窄槽宽度和深度较小，按机械加工处理。";
            } else if (nonRoundSlot && width < 15.0 && lengthWidth > 10.0) {
                decision.castFeature = false;
                decision.reason = "细长槽长宽比过大，按不铸出处理。";
            } else if (curvatureDeg > 30.0 && isOpen && width < 20.0) {
                decision.castFeature = false;
                decision.reason = "弯曲孔/槽曲率大且不利于设置芯头，按不铸出处理。";
            }

            if (!decision.castFeature) {
                decision.recommendation = "该区域在毛坯中保留实体，并在后续机加工中开孔/开槽，附加机加工余量。";
            }
            return decision;
        }

        CoreRegion classifyCoreRegion(const Mesh& mesh,
            const CoreRegion& input,
            const Vector3& demoldDirection,
            CastingMaterial material,
            ProductionBatch batch,
            const Bounds& meshBounds,
            double meshVolume) {
            CoreRegion region = input;
            std::vector<Vector3> regionVertices = collectRegionVertices(mesh, region);
            if (regionVertices.empty()) {
                region.generateCore = false;
                region.basicType = CoreRegionBasicType::NoCore;
                return region;
            }

            Vector3 regionCentroid{};
            for (const auto& point : regionVertices) {
                regionCentroid += point;
            }
            regionCentroid = regionCentroid / static_cast<double>(regionVertices.size());

            PlaneBasis demoldBasis = buildPlaneBasis(regionCentroid, demoldDirection);
            std::vector<Vector2> regionHull = buildProjectedHull(regionVertices, demoldBasis);
            std::vector<Vector2> meshHull = buildProjectedHull(mesh.vertices, demoldBasis);

            bool insideProjection = false;
            std::size_t openingCount = 0;
            if (!regionHull.empty() && !meshHull.empty()) {
                std::size_t intersectionCount = countPolygonIntersections2D(regionHull, meshHull);
                openingCount = intersectionCount;
                bool allInside = true;
                for (const auto& p : regionHull) {
                    if (!pointInPolygon2D(p, meshHull)) {
                        allInside = false;
                        break;
                    }
                }
                insideProjection = allInside && intersectionCount == 0;
            }
            region.openingCount = openingCount;

            Vector3 avgNormal{};
            for (std::size_t triIndex : region.triangleIndices) {
                avgNormal += triangleNormal(mesh, mesh.triangles.at(triIndex));
            }
            avgNormal = normalized(avgNormal);
            Vector3 demold = normalized(demoldDirection);
            Vector3 sideDirection = avgNormal - demold * dot(avgNormal, demold);
            if (length(sideDirection) <= std::numeric_limits<double>::epsilon()) {
                Vector3 reference = std::abs(demold.x) < 0.95 ? Vector3{ 1.0, 0.0, 0.0 } : Vector3{ 0.0, 1.0, 0.0 };
                sideDirection = normalized(reference - demold * dot(reference, demold));
            } else {
                sideDirection = normalized(sideDirection);
            }

            if (!insideProjection) {
                region.basicType = CoreRegionBasicType::ExternalCore;
                region.pullDirection = sideDirection;
                region.undercutDepth = rangeAlongDirection(regionVertices, sideDirection);
            } else {
                region.basicType = CoreRegionBasicType::InternalCore;
                Vector3 size = region.bounds.max - region.bounds.min;
                double tolX = std::max(kClosureTolerance, std::abs(meshBounds.max.x - meshBounds.min.x) * 0.03);
                double tolY = std::max(kClosureTolerance, std::abs(meshBounds.max.y - meshBounds.min.y) * 0.03);
                double tolZ = std::max(kClosureTolerance, std::abs(meshBounds.max.z - meshBounds.min.z) * 0.03);
                struct OpeningCandidate {
                    bool touch = false;
                    double score = -1.0;
                    Vector3 direction{};
                };
                std::array<OpeningCandidate, 6> candidates{{
                    {std::abs(region.bounds.min.x - meshBounds.min.x) <= tolX, std::abs(size.x), {-1.0, 0.0, 0.0}},
                    {std::abs(region.bounds.max.x - meshBounds.max.x) <= tolX, std::abs(size.x), { 1.0, 0.0, 0.0}},
                    {std::abs(region.bounds.min.y - meshBounds.min.y) <= tolY, std::abs(size.y), {0.0, -1.0, 0.0}},
                    {std::abs(region.bounds.max.y - meshBounds.max.y) <= tolY, std::abs(size.y), {0.0,  1.0, 0.0}},
                    {std::abs(region.bounds.min.z - meshBounds.min.z) <= tolZ, std::abs(size.z), {0.0, 0.0, -1.0}},
                    {std::abs(region.bounds.max.z - meshBounds.max.z) <= tolZ, std::abs(size.z), {0.0, 0.0,  1.0}}
                }};
                OpeningCandidate best{};
                for (const auto& candidate : candidates) {
                    if (!candidate.touch) {
                        continue;
                    }
                    if (candidate.score > best.score) {
                        best = candidate;
                    }
                }
                Vector3 openingDirection = best.direction;
                if (length(openingDirection) <= std::numeric_limits<double>::epsilon()) {
                    if (std::abs(size.x) >= std::abs(size.y) && std::abs(size.x) >= std::abs(size.z)) {
                        openingDirection = { 1.0, 0.0, 0.0 };
                    } else if (std::abs(size.y) >= std::abs(size.x) && std::abs(size.y) >= std::abs(size.z)) {
                        openingDirection = { 0.0, 1.0, 0.0 };
                    } else {
                        openingDirection = { 0.0, 0.0, 1.0 };
                    }
                }
                region.pullDirection = normalized(openingDirection);
                region.cavityDepth = rangeAlongDirection(regionVertices, region.pullDirection);
            }

            Vector3 sliceDirection = region.basicType == CoreRegionBasicType::ExternalCore
                ? region.pullDirection
                : (length(region.pullDirection) > std::numeric_limits<double>::epsilon()
                    ? region.pullDirection
                    : demoldDirection);
            RegionSliceAnalysis slice = analyzeRegionSlices(mesh, region, sliceDirection, meshBounds);
            region.lengthDiameterRatio = slice.lengthDiameterRatio;
            region.minWallThickness = slice.minWallThickness;
            region.segmentationPositions = slice.segmentationPositions;
            Vector3 regionSize = region.bounds.max - region.bounds.min;
            double width = std::min({ std::abs(regionSize.x), std::abs(regionSize.y), std::abs(regionSize.z) });
            double length = std::max({ std::abs(regionSize.x), std::abs(regionSize.y), std::abs(regionSize.z) });
            double depth = region.basicType == CoreRegionBasicType::ExternalCore
                ? region.undercutDepth
                : region.cavityDepth;

            double depthThreshold = material == CastingMaterial::CastSteel ? 3.0 : 2.0;
            bool needCore = true;
            NonCastDecision nonCast = evaluateNonCastFeature(region, width, depth, length,
                slice.maxCurvatureDeg, batch);
            region.castFeature = nonCast.castFeature;
            region.nonCastReason = nonCast.reason;
            region.machiningRecommendation = nonCast.recommendation;
            region.nxColor = nonCast.castFeature ? kCoreColor : kNonCastFeatureColor;
            if (!nonCast.castFeature) {
                needCore = false;
            }

            if (region.basicType == CoreRegionBasicType::ExternalCore) {
                if (region.undercutDepth < depthThreshold) {
                    needCore = false;
                }
                region.detailType = CoreRegionDetailType::None;
            } else {
                if (region.openingCount == 0) {
                    region.detailType = CoreRegionDetailType::ClosedCavity;
                } else if (region.openingCount == 1) {
                    region.detailType = CoreRegionDetailType::BlindHole;
                    if (region.lengthDiameterRatio < 2.0) {
                        needCore = false;
                    }
                } else {
                    region.detailType = CoreRegionDetailType::ThroughHole;
                    if (region.lengthDiameterRatio < 3.0) {
                        needCore = false;
                    }
                }
            }

            if (meshVolume > std::numeric_limits<double>::epsilon()) {
                double ratio = boundsVolume(region.bounds) / meshVolume;
                if (ratio < 0.01) {
                    needCore = false;
                }
            }

            int directionalDemands = 0;
            if (slice.hasHook) {
                ++directionalDemands;
            }
            if (region.basicType == CoreRegionBasicType::ExternalCore &&
                isDirectionDistinct(region.pullDirection, demoldDirection, 25.0)) {
                ++directionalDemands;
            }

            bool ratioSplit = region.lengthDiameterRatio > 5.0;
            bool curvatureSplit = slice.maxCurvatureDeg > 45.0;
            bool hookSplit = slice.hasHook;
            bool multiDirectionSplit = directionalDemands >= 2;
            region.requiresSegmentation = ratioSplit || curvatureSplit || hookSplit || multiDirectionSplit;
            if (curvatureSplit && region.segmentationPositions.empty()) {
                region.segmentationPositions.push_back((region.bounds.min.x + region.bounds.max.x) * 0.5);
            }

            if (region.basicType == CoreRegionBasicType::NoCore) {
                needCore = false;
            }
            region.generateCore = needCore;
            if (!needCore) {
                region.basicType = CoreRegionBasicType::NoCore;
                region.detailType = CoreRegionDetailType::None;
            }
            return region;
        }

        std::vector<CoreRegion> classifyCoreRegions(const Mesh& mesh,
            const std::vector<CoreRegion>& regions,
            const Vector3& demoldDirection,
            const AutoPartingSettings& settings) {
            std::vector<CoreRegion> classified;
            if (regions.empty()) {
                return classified;
            }
            Bounds meshBounds = computeBounds(mesh);
            double meshVolume = boundsVolume(meshBounds);
            classified.reserve(regions.size());
            for (const auto& region : regions) {
                classified.push_back(classifyCoreRegion(mesh, region, demoldDirection,
                    settings.castingMaterial, settings.productionBatch, meshBounds, meshVolume));
            }

            std::vector<std::size_t> internalIds;
            for (const auto& region : classified) {
                if (region.generateCore && region.basicType == CoreRegionBasicType::InternalCore) {
                    internalIds.push_back(region.id);
                }
            }
            bool composite = internalIds.size() > 1;
            if (composite) {
                for (auto& region : classified) {
                    if (region.generateCore && region.basicType == CoreRegionBasicType::InternalCore) {
                        region.isComposite = true;
                        region.childRegionIds = internalIds;
                    }
                }
            }
            return classified;
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
                if (!core.generateCore) {
                    continue;
                }
                double volume = boundsVolume(core.bounds);
                double ratio = volume / meshVolume;
                if (ratio < settings.minCoreVolumeRatio) {
                    continue;
                }
                scored.push_back({ core, volume });
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
            const Vector3& direction, double draftAngleDegrees);
        Bounds expandBounds(const Bounds& bounds, double clearance);

        Vector3 boundsSize(const Bounds& bounds) {
            return {
                std::max(0.0, bounds.max.x - bounds.min.x),
                std::max(0.0, bounds.max.y - bounds.min.y),
                std::max(0.0, bounds.max.z - bounds.min.z)
            };
        }

        std::array<double, 3> sortedAxisLengths(const Bounds& bounds) {
            Vector3 size = boundsSize(bounds);
            std::array<double, 3> lengths{ size.x, size.y, size.z };
            std::sort(lengths.begin(), lengths.end());
            return lengths;
        }

        bool isNear(double value, double reference, double tolerance) {
            return std::abs(value - reference) <= tolerance;
        }

        struct RegionSurfaceContact {
            bool touchMinX = false;
            bool touchMaxX = false;
            bool touchMinY = false;
            bool touchMaxY = false;
            bool touchMinZ = false;
            bool touchMaxZ = false;
        };

        RegionSurfaceContact analyzeRegionSurfaceContact(const CoreRegion& region, const Bounds& meshBounds) {
            Vector3 meshSize = boundsSize(meshBounds);
            double tolX = std::max(kClosureTolerance, meshSize.x * 0.03);
            double tolY = std::max(kClosureTolerance, meshSize.y * 0.03);
            double tolZ = std::max(kClosureTolerance, meshSize.z * 0.03);
            RegionSurfaceContact contact;
            contact.touchMinX = isNear(region.bounds.min.x, meshBounds.min.x, tolX);
            contact.touchMaxX = isNear(region.bounds.max.x, meshBounds.max.x, tolX);
            contact.touchMinY = isNear(region.bounds.min.y, meshBounds.min.y, tolY);
            contact.touchMaxY = isNear(region.bounds.max.y, meshBounds.max.y, tolY);
            contact.touchMinZ = isNear(region.bounds.min.z, meshBounds.min.z, tolZ);
            contact.touchMaxZ = isNear(region.bounds.max.z, meshBounds.max.z, tolZ);
            return contact;
        }

        std::size_t countBoundaryConnections(const RegionSurfaceContact& contact) {
            return static_cast<std::size_t>(contact.touchMinX) +
                static_cast<std::size_t>(contact.touchMaxX) +
                static_cast<std::size_t>(contact.touchMinY) +
                static_cast<std::size_t>(contact.touchMaxY) +
                static_cast<std::size_t>(contact.touchMinZ) +
                static_cast<std::size_t>(contact.touchMaxZ);
        }

        Vector3 principalAxisDirection(const Bounds& bounds) {
            Vector3 size = boundsSize(bounds);
            if (size.x >= size.y && size.x >= size.z) {
                return { 1.0, 0.0, 0.0 };
            }
            if (size.y >= size.x && size.y >= size.z) {
                return { 0.0, 1.0, 0.0 };
            }
            return { 0.0, 0.0, 1.0 };
        }

        Vector3 fallbackPerpendicularDirection(const Vector3& demoldDirection) {
            const std::array<Vector3, 3> priorityAxes{ Vector3{1.0, 0.0, 0.0},
                                                       Vector3{0.0, 1.0, 0.0},
                                                       Vector3{0.0, 0.0, 1.0} };
            Vector3 demold = normalized(demoldDirection);
            for (const auto& axis : priorityAxes) {
                if (std::abs(dot(axis, demold)) < 0.95) {
                    return normalized(axis - demold * dot(axis, demold));
                }
            }
            return { 1.0, 0.0, 0.0 };
        }

        Vector3 averageRegionNormal(const Mesh& mesh, const CoreRegion& region) {
            Vector3 average{};
            for (std::size_t triIndex : region.triangleIndices) {
                average += triangleNormal(mesh, mesh.triangles.at(triIndex));
            }
            return normalized(average);
        }

        Vector3 openDirectionFromContact(const RegionSurfaceContact& contact, const Bounds& regionBounds) {
            Vector3 size = boundsSize(regionBounds);
            struct Candidate {
                bool touched = false;
                double score = -1.0;
                Vector3 direction{};
            };
            std::array<Candidate, 6> candidates{{
                {contact.touchMinX, size.x, {-1.0, 0.0, 0.0}},
                {contact.touchMaxX, size.x, { 1.0, 0.0, 0.0}},
                {contact.touchMinY, size.y, {0.0, -1.0, 0.0}},
                {contact.touchMaxY, size.y, {0.0,  1.0, 0.0}},
                {contact.touchMinZ, size.z, {0.0, 0.0, -1.0}},
                {contact.touchMaxZ, size.z, {0.0, 0.0,  1.0}}
            }};
            Candidate best{};
            for (const auto& candidate : candidates) {
                if (!candidate.touched) {
                    continue;
                }
                if (candidate.score > best.score) {
                    best = candidate;
                }
            }
            return normalized(best.direction);
        }

        bool isExternalRegion(const RegionSurfaceContact& contact) {
            return contact.touchMinX || contact.touchMaxX ||
                contact.touchMinY || contact.touchMaxY ||
                contact.touchMinZ || contact.touchMaxZ;
        }

        SandCoreTopology inferTopology(std::size_t boundaryConnectionCount) {
            if (boundaryConnectionCount >= 2) {
                return SandCoreTopology::ThroughHole;
            }
            if (boundaryConnectionCount == 1) {
                return SandCoreTopology::BlindHole;
            }
            return SandCoreTopology::ClosedCavity;
        }

        SandCoreShape inferShape(const Bounds& bounds) {
            std::array<double, 3> lengths = sortedAxisLengths(bounds);
            double minLength = std::max(lengths[0], kClosureTolerance);
            double midLength = std::max(lengths[1], kClosureTolerance);
            double maxLength = lengths[2];
            if (maxLength / minLength > kStraightShapeLengthRatioThreshold ||
                maxLength / midLength > kStraightShapePrimarySecondaryRatioThreshold) {
                return SandCoreShape::Straight;
            }
            return SandCoreShape::Curved;
        }

        double estimateCurvatureDegrees(const Bounds& bounds, SandCoreShape shape) {
            if (shape == SandCoreShape::Straight) {
                return kStraightBaselineCurvatureDeg;
            }
            std::array<double, 3> lengths = sortedAxisLengths(bounds);
            double minLength = std::max(lengths[0], kClosureTolerance);
            double maxLength = lengths[2];
            double compactness = std::clamp(minLength / std::max(maxLength, kClosureTolerance), 0.0, 1.0);
            return kCurvedBaselineCurvatureDeg + (1.0 - compactness) * 20.0;
        }

        SandCoreType inferSandCoreType(bool external,
            SandCoreTopology topology,
            SandCoreShape shape) {
            if (external) {
                return SandCoreType::ExternalSlide;
            }
            if (topology == SandCoreTopology::ThroughHole) {
                return SandCoreType::Runner;
            }
            if (shape == SandCoreShape::Curved) {
                return SandCoreType::Composite;
            }
            return SandCoreType::InternalCavity;
        }

        SandCoreGenerationMethod selectGenerationMethod(SandCoreType type, SandCoreShape shape) {
            if (type == SandCoreType::Runner || shape == SandCoreShape::Curved) {
                return SandCoreGenerationMethod::Sweep;
            }
            if (type == SandCoreType::InternalCavity || type == SandCoreType::Composite) {
                return SandCoreGenerationMethod::BooleanSubtract;
            }
            return SandCoreGenerationMethod::Extrude;
        }

        Vector3 computeSandCorePullDirection(const Mesh& mesh,
            const CoreRegion& region,
            const Vector3& demoldDirection,
            bool external,
            bool hasOpeningDirection,
            const Vector3& openingDirection) {
            if (hasOpeningDirection && length(openingDirection) > std::numeric_limits<double>::epsilon()) {
                return normalized(openingDirection);
            }

            if (external) {
                Vector3 avgNormal = averageRegionNormal(mesh, region);
                Vector3 demold = normalized(demoldDirection);
                Vector3 projected = avgNormal - demold * dot(avgNormal, demold);
                if (length(projected) > std::numeric_limits<double>::epsilon()) {
                    return normalized(projected);
                }
            } else {
                Vector3 axis = principalAxisDirection(region.bounds);
                if (length(axis) > std::numeric_limits<double>::epsilon()) {
                    return normalized(axis);
                }
            }

            return fallbackPerpendicularDirection(demoldDirection);
        }

        Bounds generateSandCoreGeometryBounds(const CoreRegion& region,
            const Vector3& pullDirection,
            SandCoreGenerationMethod method,
            const AutoPartingSettings& settings) {
            Bounds generated = region.bounds;
            if (method == SandCoreGenerationMethod::Extrude) {
                generated = extendBoundsAlongDirection(generated, pullDirection, settings.coreHeadLength * 0.6);
            } else if (method == SandCoreGenerationMethod::Sweep) {
                generated = expandBounds(generated, std::max(0.5, settings.moldClearance));
            } else {
                generated = expandBounds(generated, std::max(1.0, settings.moldClearance * 1.5));
            }
            return generated;
        }

        std::pair<Bounds, Bounds> splitBoundsAtMid(const Bounds& bounds) {
            Vector3 size = boundsSize(bounds);
            Bounds first = bounds;
            Bounds second = bounds;
            if (size.x >= size.y && size.x >= size.z) {
                double split = (bounds.min.x + bounds.max.x) * 0.5;
                first.max.x = split;
                second.min.x = split;
            } else if (size.y >= size.x && size.y >= size.z) {
                double split = (bounds.min.y + bounds.max.y) * 0.5;
                first.max.y = split;
                second.min.y = split;
            } else {
                double split = (bounds.min.z + bounds.max.z) * 0.5;
                first.max.z = split;
                second.min.z = split;
            }
            return { first, second };
        }

        CoreHeadSpec buildCoreHead(const Bounds& coreBounds,
            const Vector3& pullDirection,
            bool touchesExternalSurface,
            std::size_t boundaryConnectionCount,
            const AutoPartingSettings& settings,
            bool opposite = false) {
            Vector3 size = boundsSize(coreBounds);
            double coreHeight = std::max(size.z, kClosureTolerance);
            double coreLength = std::max({ size.x, size.y, size.z, kClosureTolerance });
            Vector3 direction = normalized(pullDirection);
            if (opposite) {
                direction = direction * -1.0;
            }

            CoreHeadSpec head;
            head.direction = direction;
            head.clearance = std::clamp(settings.coreSeatClearance, 0.3, 0.8);
            head.position = {
                (coreBounds.min.x + coreBounds.max.x) * 0.5 + direction.x * coreLength * 0.5,
                (coreBounds.min.y + coreBounds.max.y) * 0.5 + direction.y * coreLength * 0.5,
                (coreBounds.min.z + coreBounds.max.z) * 0.5 + direction.z * coreHeight * 0.5
            };

            if (direction.z > 0.6) {
                head.orientation = CoreHeadOrientation::VerticalUp;
                head.length = coreHeight * 0.15;
                head.draftAngleDegrees = 4.0;
            } else if (direction.z < -0.6) {
                head.orientation = CoreHeadOrientation::VerticalDown;
                head.length = coreHeight * 0.25;
                head.draftAngleDegrees = 2.0;
            } else {
                bool cantilever = touchesExternalSurface && boundaryConnectionCount <= 1;
                head.orientation = cantilever ? CoreHeadOrientation::HorizontalCantilever
                    : CoreHeadOrientation::HorizontalSupported;
                head.length = coreLength * (cantilever ? 0.6 : 0.3);
                head.draftAngleDegrees = cantilever ? 2.5 : 1.5;
            }

            head.diameter = std::max(std::min(size.x, size.y), kClosureTolerance);
            head.antiCompressionRing = head.diameter > coreLength * 0.35;
            return head;
        }

        SandCoreManufacturability evaluateSandCoreManufacturability(const SandCore& core,
            const Bounds& meshBounds,
            CastingMaterial material) {
            SandCoreManufacturability check;
            std::array<double, 3> lengths = sortedAxisLengths(core.geometryBounds);
            double minSize = std::max(lengths[0], kClosureTolerance);
            double maxSize = lengths[2];
            check.minWallThickness = minSize;
            check.slendernessRatio = maxSize / minSize;

            double minWallThreshold = kCastIronMinWallThickness;
            switch (material) {
            case CastingMaterial::CastSteel:
                minWallThreshold = kCastSteelMinWallThickness;
                break;
            case CastingMaterial::CastIron:
                minWallThreshold = kCastIronMinWallThickness;
                break;
            }
            check.minWallThicknessOk = check.minWallThickness >= minWallThreshold;
            check.slendernessOk = check.slendernessRatio <= 5.0;
            check.pullPathClear = boundsContains(expandBounds(meshBounds, maxSize), core.geometryBounds);

            if (!check.minWallThicknessOk) {
                check.messages.push_back("Minimum wall thickness is below material threshold.");
            }
            if (!check.slendernessOk) {
                check.messages.push_back("Aspect ratio exceeds 5.");
            }
            if (!check.pullPathClear) {
                check.messages.push_back("Core pull path may interfere with surrounding structures.");
            }
            return check;
        }

        void appendStageLog(SandCoreDiagnostics* diagnostics, const std::string& message) {
            if (diagnostics) {
                diagnostics->stageLogs.push_back(message);
            }
        }

        SandCore createSandCoreFromRegion(const Mesh& mesh,
            const CoreRegion& region,
            const Bounds& meshBounds,
            const Vector3& demoldDirection,
            const AutoPartingSettings& settings,
            std::size_t idSeed,
            bool allowSegmentation = true) {
            SandCore core;
            core.id = idSeed;
            core.nxBodyName = "SandCore_" + std::to_string(core.id);

            RegionSurfaceContact contact = analyzeRegionSurfaceContact(region, meshBounds);
            core.diagnostics.touchesExternalSurface = isExternalRegion(contact);
            core.diagnostics.boundaryConnectionCount = countBoundaryConnections(contact);
            core.diagnostics.topology = inferTopology(core.diagnostics.boundaryConnectionCount);
            core.diagnostics.shape = inferShape(region.bounds);

            std::array<double, 3> lengths = sortedAxisLengths(region.bounds);
            core.diagnostics.lengthWidthRatio = lengths[2] / std::max(lengths[0], kClosureTolerance);
            core.diagnostics.centerlineMaxCurvatureDeg =
                estimateCurvatureDegrees(region.bounds, core.diagnostics.shape);
            core.type = inferSandCoreType(core.diagnostics.touchesExternalSurface,
                core.diagnostics.topology,
                core.diagnostics.shape);
            appendStageLog(&core.diagnostics, "Region classification completed.");

            Vector3 openingDirection = openDirectionFromContact(contact, region.bounds);
            bool hasOpeningDirection = length(openingDirection) > std::numeric_limits<double>::epsilon();
            core.pullDirection = computeSandCorePullDirection(mesh, region, demoldDirection,
                core.diagnostics.touchesExternalSurface,
                hasOpeningDirection,
                openingDirection);
            appendStageLog(&core.diagnostics, "Core pull direction computed.");

            DemoldEvaluation pullEval = evaluateRegionDirection(mesh, region.triangleIndices, core.pullDirection,
                settings.draftAngleDegrees);
            core.diagnostics.hasUndercutAlongPull = pullEval.undercutRatio > settings.separabilityUndercutThreshold;
            core.diagnostics.generationMethod = selectGenerationMethod(core.type, core.diagnostics.shape);
            core.geometryBounds = generateSandCoreGeometryBounds(region, core.pullDirection,
                core.diagnostics.generationMethod, settings);
            appendStageLog(&core.diagnostics, "Geometry generation completed.");

            bool ratioSplit = core.diagnostics.lengthWidthRatio > 5.0;
            bool curvatureSplit = core.diagnostics.centerlineMaxCurvatureDeg > 45.0;
            bool undercutSplit = core.diagnostics.hasUndercutAlongPull;
            core.diagnostics.requiresSegmentation = undercutSplit || ratioSplit || curvatureSplit;
            appendStageLog(&core.diagnostics, "Split decision completed.");

            core.heads.push_back(buildCoreHead(core.geometryBounds, core.pullDirection,
                core.diagnostics.touchesExternalSurface,
                core.diagnostics.boundaryConnectionCount,
                settings, false));
            if (core.diagnostics.topology == SandCoreTopology::ThroughHole) {
                core.heads.push_back(buildCoreHead(core.geometryBounds, core.pullDirection,
                    core.diagnostics.touchesExternalSurface,
                    core.diagnostics.boundaryConnectionCount,
                    settings, true));
            }
            appendStageLog(&core.diagnostics, "Core head design completed.");

            core.manufacturability = evaluateSandCoreManufacturability(core, meshBounds, settings.castingMaterial);
            appendStageLog(&core.diagnostics, "Manufacturability check completed.");

            if (allowSegmentation && core.diagnostics.requiresSegmentation) {
                auto splitBounds = splitBoundsAtMid(core.geometryBounds);
                CoreRegion firstRegion = region;
                firstRegion.bounds = splitBounds.first;
                CoreRegion secondRegion = region;
                secondRegion.bounds = splitBounds.second;
                core.subCores.push_back(createSandCoreFromRegion(mesh, firstRegion, meshBounds,
                    demoldDirection, settings, idSeed * 10 + 1, false));
                core.subCores.push_back(createSandCoreFromRegion(mesh, secondRegion, meshBounds,
                    demoldDirection, settings, idSeed * 10 + 2, false));
                appendStageLog(&core.diagnostics, "Sub-cores generated by splitting.");
            }

            // Keep generated core bodies visually distinct while staying in a compact layer range.
            std::size_t visualKey = core.id * kVisualHashMultiplier;
            core.nxColor = region.nxColor + static_cast<int>(visualKey % kSandCoreColorVariants);
            core.nxLayer = kSandCoreBaseLayer + static_cast<int>(visualKey % kSandCoreLayerVariants);
            appendStageLog(&core.diagnostics, "NX output mapping completed.");
            return core;
        }

        std::vector<SandCore> generateSandCores(const Mesh& mesh,
            const Vector3& demoldDirection,
            const std::vector<CoreRegion>& regions,
            const AutoPartingSettings& settings) {
            std::vector<SandCore> sandCores;
            Bounds meshBounds = computeBounds(mesh);
            sandCores.reserve(regions.size());
            std::size_t id = 1;
            for (const auto& region : regions) {
                sandCores.push_back(createSandCoreFromRegion(mesh, region, meshBounds,
                    demoldDirection, settings, id++, true));
            }
            return sandCores;
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
                return { dir, kInvalidScore, 0.0, 0.0 };
            }
            double visibilityRatio = visibleArea / totalArea;
            double undercutRatio = undercutArea / totalArea;
            double score = visibilityRatio - undercutRatio;
            return { dir, score, visibilityRatio, undercutRatio };
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
            Vector3 minPoint{ std::numeric_limits<double>::max(),
                             std::numeric_limits<double>::max(),
                             std::numeric_limits<double>::max() };
            Vector3 maxPoint{ std::numeric_limits<double>::lowest(),
                             std::numeric_limits<double>::lowest(),
                             std::numeric_limits<double>::lowest() };
            for (const auto& triangle : mesh.triangles) {
                const Vector3& a = mesh.vertices.at(triangle.v0);
                const Vector3& b = mesh.vertices.at(triangle.v1);
                const Vector3& c = mesh.vertices.at(triangle.v2);
                minPoint.x = std::min({ minPoint.x, a.x, b.x, c.x });
                minPoint.y = std::min({ minPoint.y, a.y, b.y, c.y });
                minPoint.z = std::min({ minPoint.z, a.z, b.z, c.z });
                maxPoint.x = std::max({ maxPoint.x, a.x, b.x, c.x });
                maxPoint.y = std::max({ maxPoint.y, a.y, b.y, c.y });
                maxPoint.z = std::max({ maxPoint.z, a.z, b.z, c.z });
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
                }
                else {
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
                    }
                    else {
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

        void appendSandCoreInserts(const SandCore& core,
            const AutoPartingSettings& settings,
            std::vector<CoreInsert>* inserts) {
            if (!inserts) {
                return;
            }
            CoreInsert insert;
            insert.pullDirection = length(core.pullDirection) > std::numeric_limits<double>::epsilon()
                ? core.pullDirection
                : Vector3{ 0.0, 0.0, 1.0 };
            insert.bodyBounds = expandBounds(core.geometryBounds, settings.moldClearance);
            insert.headBounds = extendBoundsAlongDirection(insert.bodyBounds, insert.pullDirection,
                settings.coreHeadLength);
            insert.seatBounds = expandBounds(insert.headBounds, settings.coreSeatClearance);
            inserts->push_back(insert);
            for (const auto& subCore : core.subCores) {
                appendSandCoreInserts(subCore, settings, inserts);
            }
        }

        MoldAssembly buildMoldAssembly(const Mesh& mesh, const PartingSurface& surface,
            const Vector3& direction, const AutoPartingSettings& settings,
            const std::vector<SandCore>& sandCores,
            const std::vector<CoreRegion>& regions) {
            MoldAssembly assembly;

            // 1) Compute direction-aligned bounding box to generate mold blanks.
            Vector3 planeOrigin = surface.boundary.empty() ? computeCentroid(mesh) : Vector3{};
            if (!surface.boundary.empty()) {
                for (const auto& point : surface.boundary) {
                    planeOrigin += point;
                }
                planeOrigin = planeOrigin / static_cast<double>(surface.boundary.size());
            }
            PlaneBasis basis = buildPlaneBasis(planeOrigin, direction);
            Bounds partLocalBounds = computeLocalBounds(mesh, basis);
            double partThickness = std::max(0.0, partLocalBounds.max.z - partLocalBounds.min.z);
            double splitContainmentTolerance = std::max(kSplitPlaneContainmentTolerance,
                partThickness * kSplitPlaneMinThicknessRatio);
            // If split plane lies outside the part (with tolerance), pull it back along demold direction.
            if (partLocalBounds.min.z > splitContainmentTolerance ||
                partLocalBounds.max.z < -splitContainmentTolerance) {
                double shift = (partLocalBounds.min.z + partLocalBounds.max.z) * 0.5;
                planeOrigin = planeOrigin + basis.normal * shift;
                basis = buildPlaneBasis(planeOrigin, direction);
                partLocalBounds = computeLocalBounds(mesh, basis);
                partThickness = std::max(0.0, partLocalBounds.max.z - partLocalBounds.min.z);
                splitContainmentTolerance = std::max(kSplitPlaneContainmentTolerance,
                    partThickness * kSplitPlaneMinThicknessRatio);
            }
            Bounds blankLocalBounds = expandLocalBounds(partLocalBounds, settings.moldBlankPadding);

            // 2) Scale the product model by shrinkage for cavity subtraction.
            // Castings shrink during cooling, so cavity must be larger (scale factor = 1 + shrinkageFactor)
            // to match final dimensions after cooling.
            Vector3 localCenter{ (partLocalBounds.min.x + partLocalBounds.max.x) / 2.0,
                                (partLocalBounds.min.y + partLocalBounds.max.y) / 2.0,
                                (partLocalBounds.min.z + partLocalBounds.max.z) / 2.0 };
            Bounds scaledLocalBounds = scaleLocalBounds(partLocalBounds, localCenter,
                1.0 + settings.shrinkageFactor);

            // 3) Split blanks with the parting-surface plane into upper and lower molds.
            double splitCoordinate = 0.0;
            double splitLowerLimit = partLocalBounds.min.z + splitContainmentTolerance;
            double splitUpperLimit = partLocalBounds.max.z - splitContainmentTolerance;
            if (splitLowerLimit <= splitUpperLimit) {
                splitCoordinate = std::clamp(splitCoordinate, splitLowerLimit, splitUpperLimit);
            }
            else {
                splitCoordinate = (partLocalBounds.min.z + partLocalBounds.max.z) * 0.5;
            }
            Bounds upperBlankLocal = blankLocalBounds;
            Bounds lowerBlankLocal = blankLocalBounds;
            upperBlankLocal.min.z = std::max(upperBlankLocal.min.z, splitCoordinate);
            lowerBlankLocal.max.z = std::min(lowerBlankLocal.max.z, splitCoordinate);

            Bounds upperCavityLocal = scaledLocalBounds;
            Bounds lowerCavityLocal = scaledLocalBounds;
            upperCavityLocal.min.z = std::max(upperCavityLocal.min.z, splitCoordinate);
            lowerCavityLocal.max.z = std::min(lowerCavityLocal.max.z, splitCoordinate);

            MoldBlock upper;
            upper.role = "Upper mold";
            upper.bounds = localBoundsToWorld(upperBlankLocal, basis);
            upper.cavityBounds = localBoundsToWorld(upperCavityLocal, basis);
            upper.pullDirection = normalized(direction);

            MoldBlock lower;
            lower.role = "Lower mold";
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

            // 4) Final mold blank semantics: enclosing box minus scaled part minus all generated sand cores.
            for (const auto& core : sandCores) {
                appendSandCoreInserts(core, settings, &assembly.cores);
            }

            // 5) Store non-cast-hole regions for solid-fill and machining annotation.
            for (const auto& region : regions) {
                if (region.castFeature) {
                    continue;
                }
                MachiningRegion machining;
                machining.bounds = region.bounds;
                machining.nxColor = region.nxColor;
                machining.reason = region.nonCastReason;
                machining.recommendation = region.machiningRecommendation;
                assembly.machiningRegions.push_back(machining);
            }

            return assembly;
        }

        std::vector<StrategyOption> buildStrategyOptions(const SeparabilityReport& report,
            std::size_t coreCount) {
            std::vector<StrategyOption> options;
            StrategyOption baseline;
            baseline.name = "Default multi-parting";
            baseline.score = report.score -
                static_cast<double>(report.obstacles.size()) * kStrategyObstaclePenalty -
                static_cast<double>(coreCount) * kStrategyCorePenalty;
            for (const auto& obstacle : report.obstacles) {
                baseline.resolved.push_back(obstacle.type);
            }
            options.push_back(baseline);

            StrategyOption conservative = baseline;
            conservative.name = "Core-priority";
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
                issues.push_back({ "Parting-line extraction returned no boundary points.", 0.8 });
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
                issues.push_back({ "Upper/lower mold bounding boxes overlap; check split plane and parting surface.",
                                  0.6 });
            }
            if (evaluation.undercutRatio > kUndercutWarningRatio) {
                issues.push_back({ "Undercut ratio exceeds threshold; adjust demold direction.",
                                  std::min(1.0, evaluation.undercutRatio) });
            }
            if (mesh.triangles.empty()) {
                issues.push_back({ "No valid triangles remain after preprocessing.", 1.0 });
            }
            for (const auto& core : assembly.cores) {
                if (!boundsContains(assembly.overallBounds, core.seatBounds)) {
                    issues.push_back({ "Core seat exceeds mold stock bounds; adjust core head or allowance.",
                                      0.5 });
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
        // Identify the max contour first: it is the largest cross-section perpendicular to
        // the demold direction and serves as the authoritative reference for the parting surface.
        result.maxContour = identifyMaxContour(result.cleanedMesh, result.partingLine, result.demold.direction);
        // Build the parting surface from the max contour so it is a flat plane perpendicular
        // to the demold direction.  Fall back to the original approach if the contour is
        // degenerate (fewer than 3 boundary points).
        if (result.maxContour.boundary.size() >= 3) {
            result.partingSurface = buildPlanarPartingSurface(
                result.maxContour, result.demold.direction,
                settings.partingSurfaceExtension, &result.partingSurfaceStages);
        } else {
            result.partingSurfaceStages = buildPartingSurfaceStages(result.partingLine,
                result.demold.direction,
                settings.smoothingFactor,
                settings.partingSurfaceExtension,
                settings.preferPlanarSurface,
                settings.nonPlanarDeviationRatio);
            if (!result.partingSurfaceStages.empty()) {
                result.partingSurface.boundary = result.partingSurfaceStages.back().boundary;
            } else {
                result.partingSurface = buildPartingSurface(result.partingLine,
                    result.demold.direction,
                    settings.smoothingFactor,
                    settings.partingSurfaceExtension,
                    settings.preferPlanarSurface,
                    settings.nonPlanarDeviationRatio, nullptr);
            }
        }
        result.split = splitMesh(result.cleanedMesh, result.demold.direction);
        std::vector<CoreRegion> undercutRegions = detectCoreRegions(result.cleanedMesh,
            result.demold.direction,
            settings.draftAngleDegrees);
        std::vector<CoreRegion> regionClassifications = classifyCoreRegions(result.cleanedMesh,
            undercutRegions, result.demold.direction, settings);
        result.separability = evaluateSeparability(result.cleanedMesh, result.demold, undercutRegions,
            settings, &result.cores);
        if (result.cores.empty()) {
            result.cores = regionClassifications;
        } else {
            std::unordered_map<std::size_t, CoreRegion> classifiedById;
            for (const auto& core : regionClassifications) {
                classifiedById[core.id] = core;
            }
            for (auto& core : result.cores) {
                auto found = classifiedById.find(core.id);
                if (found != classifiedById.end()) {
                    core = found->second;
                }
            }
            std::unordered_set<std::size_t> knownIds;
            for (const auto& core : result.cores) {
                knownIds.insert(core.id);
            }
            for (const auto& core : regionClassifications) {
                if (core.generateCore && knownIds.count(core.id) == 0) {
                    result.cores.push_back(core);
                }
            }
        }
        Bounds meshBounds = computeBounds(result.cleanedMesh);
        std::vector<std::size_t> keptCoreIds;
        // Reduce sand-core candidates by volume ratio and count limits to minimize core count.
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
        }
        else if (result.cores.empty()) {
            for (auto& obstacle : result.separability.obstacles) {
                if (obstacle.type == ObstacleType::CoreCandidate) {
                    obstacle.type = ObstacleType::MultiDirection;
                    obstacle.corePreferred = false;
                }
            }
        }
        result.sandCores = generateSandCores(result.cleanedMesh, result.demold.direction,
            result.cores, settings);
        std::vector<InterferenceIssue> secondaryPartingIssues =
            checkPartingSurfaceQuality(result.partingSurface, result.demold.direction);
        for (const auto& issue : secondaryPartingIssues) {
            result.issues.push_back(
                { "Secondary parting-surface check after sand-core generation: " + issue.message,
                  issue.severity });
        }
        bool hasCriticalPartingIssue = std::any_of(secondaryPartingIssues.begin(),
            secondaryPartingIssues.end(),
            [](const InterferenceIssue& issue) {
                return issue.severity >= 0.85;
            });
        if (hasCriticalPartingIssue) {
            result.partingSurface = buildPartingSurface(result.partingLine,
                result.demold.direction,
                settings.smoothingFactor,
                settings.partingSurfaceExtension,
                settings.preferPlanarSurface,
                settings.nonPlanarDeviationRatio, nullptr);
            result.split = splitMesh(result.cleanedMesh, result.demold.direction);
        }
        result.moldAssembly = buildMoldAssembly(result.cleanedMesh, result.partingSurface,
            result.demold.direction, settings, result.sandCores, regionClassifications);
        result.strategies = buildStrategyOptions(result.separability, result.cores.size());
        std::vector<InterferenceIssue> interferenceIssues = checkInterference(result.cleanedMesh,
            result.split, result.partingLine, result.partingSurface, result.demold, result.moldAssembly);
        result.issues.insert(result.issues.end(), interferenceIssues.begin(), interferenceIssues.end());
        return result;
    }

}  // namespace casting
