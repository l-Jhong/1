# Casting Auto Parting (NX 12.0 C++)

本项目为 NX12.0 + VS2022 的铸造 CAD/CAE 自动分型流程示例，包含：

1. 模型预处理：修复网格、补洞、清理破面、统一法向
2. 最优脱模方向计算：基于可见性、脱模角、倒扣数量进行优化
3. 分型线自动提取：通过投影可见性分析获取轮廓交界环
4. 分型面构造：延伸、光顺、封闭分型线
5. 型腔分割：使用分型面分割上下模
6. 砂芯/活块自动识别：识别封闭空腔与倒扣区域
7. 干涉检查与方案优化

## 构建与运行

本仓库默认提供一个独立可执行示例（`casting_auto_parting_demo`）用于验证算法流程：

```bash
cmake -S . -B build
cmake --build build
./build/casting_auto_parting_demo
```

## NXOpen 二次开发入口

`src/NXOpenEntry.cpp` 内置了 NXOpen C++ 二次开发模板入口（`ufusr`），并在 `do_it()` 中调用自动分型流程示例。

如需在 NX 环境中编译：

```bash
cmake -S . -B build -DNXOPEN_ENABLED=ON -DNXOPEN_INCLUDE_DIR="C:/Siemens/NX12.0/UGOPEN"
cmake --build build
```

> 注意：请根据实际 NXOpen 安装路径设置 `NXOPEN_INCLUDE_DIR`。
