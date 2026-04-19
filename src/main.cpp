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

    std::cout << "脱模方向: (" << result.demold.direction.x << ", "
              << result.demold.direction.y << ", " << result.demold.direction.z << ")\n";
    std::cout << "可见性比例: " << result.demold.visibilityRatio << "\n";
    std::cout << "倒扣比例: " << result.demold.undercutRatio << "\n";
    std::cout << "分型线点数: " << result.partingLine.points.size() << "\n";
    std::cout << "分型面阶段数: " << result.partingSurfaceStages.size() << "\n";
    std::cout << "芯区域数量: " << result.cores.size() << "\n";
    std::cout << "砂芯数量: " << result.sandCores.size() << "\n";
    std::cout << "可分离性: " << (result.separability.separable ? "可分离" : "不可分离")
              << " | 障碍数量: " << result.separability.obstacles.size() << "\n";
    std::cout << "最大轮廓面积: " << result.maxContour.area << "\n";
    std::cout << "最大轮廓来源: "
              << (result.maxContour.selectedFromSlice ? "切片" : "回退")
              << " | 切片索引: " << result.maxContour.selectedSliceIndex
              << " | 局部W: " << result.maxContour.selectedSliceW
              << " | 是否回退: " << (result.maxContour.fallbackUsed ? "是" : "否") << "\n";
    std::cout << "模具块数量: " << result.moldAssembly.blocks.size() << "\n";
    std::cout << "芯镶件数量: " << result.moldAssembly.cores.size() << "\n";
    if (!result.sandCores.empty()) {
        const auto& firstCore = result.sandCores.front();
        std::cout << "首个砂芯芯头数量: " << firstCore.heads.size()
                  << " | 子芯数量: " << firstCore.subCores.size() << "\n";
    }
    std::cout << "干涉问题数量: " << result.issues.size() << "\n";
    std::cout << "可视化颜色: 上模=" << casting::kUpperMoldColor
              << ", 下模=" << casting::kLowerMoldColor
              << ", 芯=" << casting::kCoreColor << "\n";

    return 0;
}
