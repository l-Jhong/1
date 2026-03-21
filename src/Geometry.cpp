#include "casting/Geometry.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace casting {

Vector3 operator+(const Vector3& left, const Vector3& right) {
    return {left.x + right.x, left.y + right.y, left.z + right.z};
}

Vector3 operator-(const Vector3& left, const Vector3& right) {
    return {left.x - right.x, left.y - right.y, left.z - right.z};
}

Vector3 operator*(const Vector3& value, double scale) {
    return {value.x * scale, value.y * scale, value.z * scale};
}

Vector3 operator/(const Vector3& value, double scale) {
    return {value.x / scale, value.y / scale, value.z / scale};
}

Vector3& operator+=(Vector3& left, const Vector3& right) {
    left.x += right.x;
    left.y += right.y;
    left.z += right.z;
    return left;
}

Vector2 operator+(const Vector2& left, const Vector2& right) {
    return {left.x + right.x, left.y + right.y};
}

Vector2 operator-(const Vector2& left, const Vector2& right) {
    return {left.x - right.x, left.y - right.y};
}

Vector2 operator*(const Vector2& value, double scale) {
    return {value.x * scale, value.y * scale};
}

Vector2 operator/(const Vector2& value, double scale) {
    return {value.x / scale, value.y / scale};
}

Vector2& operator+=(Vector2& left, const Vector2& right) {
    left.x += right.x;
    left.y += right.y;
    return left;
}

double dot(const Vector3& left, const Vector3& right) {
    return left.x * right.x + left.y * right.y + left.z * right.z;
}

Vector3 cross(const Vector3& left, const Vector3& right) {
    return {
        left.y * right.z - left.z * right.y,
        left.z * right.x - left.x * right.z,
        left.x * right.y - left.y * right.x
    };
}

double length(const Vector3& value) {
    return std::sqrt(dot(value, value));
}

Vector3 normalized(const Vector3& value) {
    double len = length(value);
    if (len <= std::numeric_limits<double>::epsilon()) {
        return {};
    }
    return value / len;
}

double cross(const Vector2& left, const Vector2& right) {
    return left.x * right.y - left.y * right.x;
}

Vector3 triangleNormal(const Mesh& mesh, const Triangle& triangle) {
    const Vector3& a = mesh.vertices.at(triangle.v0);
    const Vector3& b = mesh.vertices.at(triangle.v1);
    const Vector3& c = mesh.vertices.at(triangle.v2);
    return normalized(cross(b - a, c - a));
}

Vector3 triangleCentroid(const Mesh& mesh, const Triangle& triangle) {
    const Vector3& a = mesh.vertices.at(triangle.v0);
    const Vector3& b = mesh.vertices.at(triangle.v1);
    const Vector3& c = mesh.vertices.at(triangle.v2);
    return {(a.x + b.x + c.x) / 3.0, (a.y + b.y + c.y) / 3.0, (a.z + b.z + c.z) / 3.0};
}

double triangleArea(const Mesh& mesh, const Triangle& triangle) {
    const Vector3& a = mesh.vertices.at(triangle.v0);
    const Vector3& b = mesh.vertices.at(triangle.v1);
    const Vector3& c = mesh.vertices.at(triangle.v2);
    return 0.5 * length(cross(b - a, c - a));
}

Bounds computeBounds(const Mesh& mesh) {
    Bounds bounds{};
    if (mesh.vertices.empty()) {
        return bounds;
    }
    Vector3 minPoint{std::numeric_limits<double>::max(),
                     std::numeric_limits<double>::max(),
                     std::numeric_limits<double>::max()};
    Vector3 maxPoint{std::numeric_limits<double>::lowest(),
                     std::numeric_limits<double>::lowest(),
                     std::numeric_limits<double>::lowest()};
    for (const auto& vertex : mesh.vertices) {
        minPoint.x = std::min(minPoint.x, vertex.x);
        minPoint.y = std::min(minPoint.y, vertex.y);
        minPoint.z = std::min(minPoint.z, vertex.z);
        maxPoint.x = std::max(maxPoint.x, vertex.x);
        maxPoint.y = std::max(maxPoint.y, vertex.y);
        maxPoint.z = std::max(maxPoint.z, vertex.z);
    }
    bounds.min = minPoint;
    bounds.max = maxPoint;
    return bounds;
}

Vector3 computeCentroid(const Mesh& mesh) {
    if (mesh.vertices.empty()) {
        return {};
    }
    Vector3 sum{};
    for (const auto& vertex : mesh.vertices) {
        sum += vertex;
    }
    return sum / static_cast<double>(mesh.vertices.size());
}

std::vector<Vector2> computeConvexHull2D(const std::vector<Vector2>& points) {
    if (points.size() < 3) {
        return points;
    }
    std::vector<Vector2> sorted = points;
    std::sort(sorted.begin(), sorted.end(),
              [](const Vector2& left, const Vector2& right) {
                  if (left.x == right.x) {
                      return left.y < right.y;
                  }
                  return left.x < right.x;
              });
    sorted.erase(std::unique(sorted.begin(), sorted.end(),
                             [](const Vector2& left, const Vector2& right) {
                                 return left.x == right.x && left.y == right.y;
                             }),
                 sorted.end());
    if (sorted.size() < 3) {
        return sorted;
    }
    std::vector<Vector2> hull;
    hull.reserve(sorted.size() * 2);
    auto appendHull = [&hull](const Vector2& point) {
        while (hull.size() >= 2) {
            Vector2 a = hull[hull.size() - 2];
            Vector2 b = hull[hull.size() - 1];
            if (cross(b - a, point - b) > 0.0) {
                break;
            }
            hull.pop_back();
        }
        hull.push_back(point);
    };
    for (const auto& point : sorted) {
        appendHull(point);
    }
    std::size_t lowerSize = hull.size();
    for (auto it = sorted.rbegin(); it != sorted.rend(); ++it) {
        appendHull(*it);
    }
    if (hull.size() > 1) {
        hull.pop_back();
    }
    if (hull.size() > lowerSize) {
        hull.pop_back();
    }
    return hull;
}

}  // namespace casting
