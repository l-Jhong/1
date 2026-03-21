#pragma once

#include <cstddef>
#include <vector>

namespace casting {

struct Vector3 {
    double x{};
    double y{};
    double z{};
};

Vector3 operator+(const Vector3& left, const Vector3& right);
Vector3 operator-(const Vector3& left, const Vector3& right);
Vector3 operator*(const Vector3& value, double scale);
Vector3 operator/(const Vector3& value, double scale);
Vector3& operator+=(Vector3& left, const Vector3& right);

double dot(const Vector3& left, const Vector3& right);
Vector3 cross(const Vector3& left, const Vector3& right);
double length(const Vector3& value);
Vector3 normalized(const Vector3& value);

struct Triangle {
    std::size_t v0{};
    std::size_t v1{};
    std::size_t v2{};
};

struct Mesh {
    std::vector<Vector3> vertices;
    std::vector<Triangle> triangles;
};

struct Bounds {
    Vector3 min;
    Vector3 max;
};

Vector3 triangleNormal(const Mesh& mesh, const Triangle& triangle);
Vector3 triangleCentroid(const Mesh& mesh, const Triangle& triangle);
double triangleArea(const Mesh& mesh, const Triangle& triangle);
Bounds computeBounds(const Mesh& mesh);
Vector3 computeCentroid(const Mesh& mesh);

}  // namespace casting
