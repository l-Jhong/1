# Casting Auto Parting (NX 12.0 C++)

本项目为 NX 12.0 + VS2022 的铸造 CAD/CAE 自动分型流程示例，包含：

1. 模型预处理：修复网格、补洞、清理破面、统一法向
2. 最优脱模方向计算：基于可见性、脱模角、倒扣数量进行优化
3. 分型线自动提取：通过投影可见性分析获取轮廓交界环
4. 分型面构造：延伸、光顺、封闭分型线
5. 型腔分割：使用分型面分割上下模
6. 砂芯/活块自动识别：识别封闭空腔与倒扣区域
7. 干涉检查与方案优化

## 构建与运行（仅 VS2022）

本仓库已切换为 **VS2022 常规 C++ 工程**（不使用 CMake）。

1. 用 VS2022 打开解决方案：`casting_auto_parting.sln`
2. 选择 `x64` + `Debug` 或 `Release`
3. 右键 `casting_auto_parting_demo` 设为启动项目并编译运行

## NXOpen 二次开发入口

`src/NXOpenEntry.cpp` 内置了 NXOpen C++ 二次开发模板入口（`ufusr`），并在 `do_it()` 中调用自动分型流程示例。

如需在 NX 环境中编译（生成 NX 12.0 可用 DLL）：

1. 在 VS2022 中编译项目 `casting_auto_parting_nxopen`
2. 确认环境变量 `UGII_BASE_DIR` 指向 NX 安装目录（工程默认使用 `$(UGII_BASE_DIR)\\UGOPEN`）
3. 如路径不同，可在项目属性中调整：
   - `NXOPEN_INCLUDE_DIR`
   - `NXOPEN_LIB_DIR`

生成 DLL 文件名为：

```
casting_auto_parting_nxopen.dll
```
