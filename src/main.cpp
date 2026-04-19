#include "casting/AutoPartingPipeline.h"

#include <iostream>

namespace {

casting::Mesh buildBoxMesh(double size) {
    double h = size / 2.0;
    casting::Mesh mesh;
    mesh.vertices = {
        {-h, -h, -h},
        {h, -h, -h},
        {h, h, -h},
        {-h, h, -h},
        {-h, -h, h},
        {h, -h, h},
        {h, h, h},
        {-h, h, h}
    };
    mesh.triangles = {
        {0, 1, 2}, {0, 2, 3},
        {4, 6, 5}, {4, 7, 6},
        {0, 4, 5}, {0, 5, 1},
        {1, 5, 6}, {1, 6, 2},
        {2, 6, 7}, {2, 7, 3},
        {3, 7, 4}, {3, 4, 0}
    };
    return mesh;
}

}  // namespace

int main() {
    casting::AutoPartingPipeline pipeline;
    casting::Mesh mesh = buildBoxMesh(10.0);
    casting::AutoPartingResult result = pipeline.run(mesh);

    std::cout << "Demold direction: (" << result.demold.direction.x << ", "
              << result.demold.direction.y << ", " << result.demold.direction.z << ")\n";
    std::cout << "Visibility ratio: " << result.demold.visibilityRatio << "\n";
    std::cout << "Undercut ratio: " << result.demold.undercutRatio << "\n";
    std::cout << "Parting line points: " << result.partingLine.points.size() << "\n";
    std::cout << "Parting surface stages: " << result.partingSurfaceStages.size() << "\n";
    std::cout << "Core regions: " << result.cores.size() << "\n";
    std::cout << "Sand cores: " << result.sandCores.size() << "\n";
    std::cout << "Separability: " << (result.separability.separable ? "separable" : "not separable")
              << " | Obstacles: " << result.separability.obstacles.size() << "\n";
    std::cout << "Max contour area: " << result.maxContour.area << "\n";
    std::cout << "Max contour source: "
              << (result.maxContour.selectedFromSlice ? "slice" : "fallback")
              << " | Slice index: " << result.maxContour.selectedSliceIndex
              << " | Local W: " << result.maxContour.selectedSliceW
              << " | Fallback used: " << (result.maxContour.fallbackUsed ? "yes" : "no") << "\n";
    std::cout << "Mold blocks: " << result.moldAssembly.blocks.size() << "\n";
    std::cout << "Core inserts: " << result.moldAssembly.cores.size() << "\n";
    if (!result.sandCores.empty()) {
        const auto& firstCore = result.sandCores.front();
        std::cout << "First sand core heads: " << firstCore.heads.size()
                  << " | sub-cores: " << firstCore.subCores.size() << "\n";
    }
    std::cout << "Interference issues: " << result.issues.size() << "\n";
    std::cout << "Visualization colors: Upper=" << casting::kUpperMoldColor
              << ", Lower=" << casting::kLowerMoldColor
              << ", Core=" << casting::kCoreColor << "\n";

    return 0;
}
