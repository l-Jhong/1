#include "casting/AutoPartingPipeline.h"
#include "casting/Geometry.h"

#include <iostream>

int main() {
    casting::AutoPartingPipeline pipeline;
    casting::Mesh mesh = casting::buildBoxMesh(10.0);
    casting::AutoPartingResult result = pipeline.run(mesh);

    // ---- 零件 (Part / Casting) ----
    std::cout << "=== 零件 (Part) ===\n";
    std::cout << "  Vertices: " << result.cleanedMesh.vertices.size()
              << " | Triangles: " << result.cleanedMesh.triangles.size() << "\n";
    std::cout << "  Demold direction: (" << result.demold.direction.x << ", "
              << result.demold.direction.y << ", " << result.demold.direction.z << ")"
              << " | Visibility: " << result.demold.visibilityRatio
              << " | Undercut: " << result.demold.undercutRatio << "\n";

    // ---- 分型面 (Parting Surface) ----
    std::cout << "=== 分型面 (Parting Surface) ===\n";
    std::cout << "  Boundary points: " << result.partingSurface.boundary.size() << "\n";
    std::cout << "  Parting line points: " << result.partingLine.points.size() << "\n";
    std::cout << "  Stages: " << result.partingSurfaceStages.size() << "\n";
    std::cout << "  Max contour area: " << result.maxContour.area
              << " | Source: " << (result.maxContour.selectedFromSlice ? "slice" : "fallback") << "\n";

    // ---- 上下模 (Upper / Lower Mold) ----
    std::cout << "=== 上下模 (Mold Halves) ===\n";
    for (const auto& block : result.moldAssembly.blocks) {
        std::cout << "  " << block.role
                  << " | Pull: (" << block.pullDirection.x << ", "
                  << block.pullDirection.y << ", " << block.pullDirection.z << ")"
                  << " | Bounds Z: [" << block.bounds.min.z << ", " << block.bounds.max.z << "]\n";
    }
    std::cout << "  Core inserts: " << result.moldAssembly.cores.size() << "\n";
    std::cout << "  Machining regions (non-cast): " << result.moldAssembly.machiningRegions.size() << "\n";

    // ---- 内外部砂芯 (Internal / External Sand Cores) ----
    std::cout << "=== 内外部砂芯 (Sand Cores) ===\n";
    std::size_t internalCount = 0;
    std::size_t externalCount = 0;
    for (const auto& core : result.sandCores) {
        if (core.type == casting::SandCoreType::ExternalSlide) {
            ++externalCount;
        } else {
            ++internalCount;
        }
    }
    std::cout << "  Total: " << result.sandCores.size()
              << " | Internal: " << internalCount
              << " | External: " << externalCount << "\n";
    if (!result.sandCores.empty()) {
        const auto& firstCore = result.sandCores.front();
        std::cout << "  First core heads: " << firstCore.heads.size()
                  << " | Sub-cores: " << firstCore.subCores.size() << "\n";
    }
    std::cout << "  Core regions detected: " << result.cores.size() << "\n";
    std::cout << "  Separability: "
              << (result.separability.separable ? "separable" : "not separable")
              << " | Obstacles: " << result.separability.obstacles.size() << "\n";

    // ---- Summary ----
    std::cout << "=== Summary ===\n";
    std::cout << "  Interference issues: " << result.issues.size() << "\n";
    std::cout << "  Visualization colors: upper=" << casting::kUpperMoldColor
              << ", lower=" << casting::kLowerMoldColor
              << ", core=" << casting::kCoreColor
              << ", non-cast=" << casting::kNonCastFeatureColor << "\n";

    return 0;
}
