# NX 12.0 二次开发示例项目

基于 **Siemens NX 12.0** 与 **C++17** 的 NX Open API 二次开发工程，演示如何在 NX 环境中查询实体几何信息（面数、边数、体积、表面积、质心）并输出到信息窗口。

---

## 目录结构

```
.
├── CMakeLists.txt              # CMake 构建脚本
├── NX12SecondaryDev.sln        # Visual Studio 2022 解决方案文件
├── NX12SecondaryDev.vcxproj    # Visual Studio 2022 项目文件
├── include/
│   ├── nx_utils.h              # NX Open 公用工具函数声明
│   └── body_info.h             # 实体信息查询接口声明
├── src/
│   ├── main.cpp                # DLL 入口（ufusr / ufusr_ask_unload）
│   ├── nx_utils.cpp            # NX Open 公用工具函数实现
│   └── body_info.cpp           # 实体信息查询实现
└── docs/
    └── build_guide.md          # 详细编译与部署说明
```

---

## 环境要求

| 组件 | 版本 |
|------|------|
| Siemens NX | 12.0 |
| Visual Studio | 2022（工具集 v143）或 2019（工具集 v142） |
| Windows SDK | 10.0 |
| C++ 标准 | C++17 |
| 目标平台 | x64 |

---

## 快速开始

### 方式一：Visual Studio 2022

1. 用 Visual Studio 打开 `NX12SecondaryDev.sln`。
2. 在 `NX12SecondaryDev.vcxproj` 中修改 `<NXRoot>` 属性，指向本机 NX 12.0 安装目录（默认 `C:\Program Files\Siemens\NX 12.0`）。
3. 选择 **x64 / Release**，按 **F7** 构建，输出 DLL 位于 `bin\x64\Release\`。

### 方式二：CMake

```bat
mkdir build && cd build
cmake .. -G "Visual Studio 17 2022" -A x64 -DNX_ROOT="C:/Program Files/Siemens/NX 12.0"
cmake --build . --config Release
```

---

## 在 NX 中加载运行

1. 打开 NX 12.0，载入任意含实体的部件（`.prt`）。
2. 菜单栏 → **文件** → **实用工具** → **执行NX打开程序（Execute NX Open Program）**。
3. 在弹出对话框中浏览选择编译好的 `NX12SecondaryDev.dll`。
4. 点击 **确定**，在 NX 信息窗口中查看输出。

---

## 功能说明

| 功能模块 | 文件 | 说明 |
|----------|------|------|
| DLL 入口 | `src/main.cpp` | `ufusr` / `ufusr_ask_unload` 回调 |
| 工具函数 | `src/nx_utils.cpp` | API 初始化、消息输出、对象遍历 |
| 实体信息 | `src/body_info.cpp` | 面数、边数、体积、表面积、质心查询 |

---

## 扩展开发

在此基础上可继续扩展：

- **特征创建**：调用 `UF_MODL_create_*` 系列函数创建拉伸、旋转等特征。
- **参数化建模**：读写表达式（`UF_MODL_ask_exp_tag`、`UF_MODL_set_exp_value`）。
- **装配操作**：使用 `uf_assem.h` 管理装配树。
- **制图标注**：调用 `uf_draw.h` 自动生成工程图与标注。
- **菜单集成**：编写 `.men` 菜单文件，将功能注册到 NX 菜单栏。

---

## 参考资料

- NX Open API 文档：`<NX_ROOT>\UGOPEN\NXOpenC++Reference\`
- UF 函数参考：`<NX_ROOT>\UGOPEN\uf.h` 及同目录各 `uf_*.h`
- Siemens 官方示例：`<NX_ROOT>\UGOPEN\SampleNXOpenApplications\`
