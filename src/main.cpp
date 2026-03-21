/**
 * main.cpp
 * NX 12.0 二次开发 DLL 入口文件
 *
 * NX Open 应用程序通过以下两个回调函数与 NX 宿主进程通信：
 *   ufusr        – 用户选择菜单项或执行内部调试时被调用
 *   ufusr_ask_unload – NX 询问何时卸载该 DLL
 */

#include "nx_utils.h"
#include "body_info.h"

#include <uf.h>
#include <uf_ui.h>
#include <uf_part.h>
#include <uf_obj.h>
#include <uf_modl.h>

#include <string>
#include <sstream>

// -----------------------------------------------------------------------
// ufusr：NX 调用本 DLL 的主入口
// -----------------------------------------------------------------------
extern "C" void ufusr(char* /*param*/, int* returnCode, int /*paramLen*/)
{
    *returnCode = 0;

    // 初始化 NX Open API
    if (NXUtils::Initialize() != 0)
    {
        *returnCode = 1;
        return;
    }

    // 获取当前工作部件
    tag_t workPart = NULL_TAG;
    if (NXUtils::GetWorkPart(workPart) != 0)
    {
        NXUtils::PrintMessage("错误：未找到当前工作部件，请先打开一个部件文件。");
        NXUtils::Terminate();
        *returnCode = 1;
        return;
    }

    // 输出欢迎信息
    NXUtils::PrintMessage("=== NX 12.0 二次开发示例 ===");

    // 查询部件中所有实体（UF_solid_type / UF_solid_body_subtype）
    std::vector<tag_t> bodies;
    NXUtils::GetObjectsInPart(workPart, UF_solid_type, UF_solid_body_subtype, bodies);

    std::ostringstream oss;
    oss << "当前部件共有 " << bodies.size() << " 个实体。";
    NXUtils::PrintMessage(oss.str());

    // 对每个实体输出基本信息
    for (std::size_t i = 0; i < bodies.size(); ++i)
    {
        BodyInfo::PrintBodyInfo(bodies[i], static_cast<int>(i + 1));
    }

    NXUtils::PrintMessage("=== 处理完成 ===");

    NXUtils::Terminate();
}

// -----------------------------------------------------------------------
// ufusr_ask_unload：返回卸载策略
//   UF_UNLOAD_IMMEDIATELY  – 调用完立即卸载
//   UF_UNLOAD_UG_TERMINATE – NX 退出时卸载（默认）
//   UF_UNLOAD_EXPLICITLY   – 仅手动卸载
// -----------------------------------------------------------------------
extern "C" int ufusr_ask_unload()
{
    return UF_UNLOAD_IMMEDIATELY;
}
