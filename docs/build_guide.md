# NX 12.0 二次开发编译与部署指南

## 1. 前置准备

### 1.1 安装 NX 12.0

确认 NX 12.0 已正确安装，并记录安装路径，例如：

```
C:\Program Files\Siemens\NX 12.0
```

### 1.2 确认 UGOPEN 目录

NX Open C++ API 头文件和库文件均位于：

```
<NX_ROOT>\UGOPEN\
```

主要文件：
- 头文件：`uf.h`、`uf_ui.h`、`uf_part.h`、`uf_modl.h` 等
- 导入库：`libufun.lib`、`libugopenint.lib`

---

## 2. Visual Studio 2022 编译

### 2.1 打开项目

双击打开 `NX12SecondaryDev.sln`。

### 2.2 修改 NX 安装路径

在 **解决方案资源管理器** 中右键项目 → **属性** → **配置属性** → **VC++ 目录**，
或直接编辑 `NX12SecondaryDev.vcxproj`，将：

```xml
<NXRoot>C:\Program Files\Siemens\NX 12.0</NXRoot>
```

改为本机实际路径。

### 2.3 选择目标配置

- **平台**：x64（NX 12.0 仅支持 64 位）
- **配置**：Release（正式部署）或 Debug（开发调试）

### 2.4 构建

菜单 **生成（Build）** → **生成解决方案（Build Solution）**，或按 `Ctrl+Shift+B`。

成功后，DLL 输出路径：
```
bin\x64\Release\NX12SecondaryDev.dll
```

---

## 3. CMake 编译

```bat
cd /d "<项目根目录>"
mkdir build && cd build

:: 配置
cmake .. -G "Visual Studio 17 2022" -A x64 ^
    -DNX_ROOT="C:/Program Files/Siemens/NX 12.0"

:: 编译 Release
cmake --build . --config Release
```

---

## 4. 在 NX 中运行

### 方式 A：执行 NX Open 程序

1. 打开 NX 12.0，加载一个含实体的 `.prt` 文件。
2. 菜单 → **文件(File)** → **实用工具(Utilities)** → **执行NX打开程序(Execute NX Open Program...)**。
3. 浏览选择 `NX12SecondaryDev.dll`，点击 **确定**。
4. 查看 NX **信息窗口** 中的输出结果。

### 方式 B：注册到菜单（推荐）

1. 创建菜单文件 `NX12SecondaryDev.men`（参考 NX Open 菜单编写规范）。
2. 将 DLL 和菜单文件放置到 NX 自定义目录，或通过 `UGII_CUSTOM_DIRECTORY_FILE` 环境变量指定。
3. 重启 NX，菜单项自动出现在指定位置。

---

## 5. 常见问题

| 问题 | 原因 | 解决方案 |
|------|------|----------|
| 找不到 `libufun.lib` | NX_ROOT 路径错误 | 检查 `<NXRoot>` 属性 |
| DLL 加载失败（0xC000007B） | 平台不匹配 | 确认使用 x64 配置 |
| 信息窗口无输出 | 部件未打开 | 先打开 `.prt` 文件再执行 |
| `UF_initialize()` 返回非零值 | NX 内部错误 | 检查 NX 授权是否有效 |
