#pragma once
/**
 * nx_utils.h
 * NX 12.0 二次开发公用工具函数声明
 */

#ifndef NX_UTILS_H
#define NX_UTILS_H

#include <string>
#include <vector>

// NX Open 头文件（由 UGOPEN 目录提供）
#include <uf.h>
#include <uf_ui.h>
#include <uf_obj.h>
#include <uf_part.h>
#include <uf_modl.h>
#include <uf_curve.h>
#include <uf_assem.h>
#include <uf_disp.h>
#include <uf_mtx.h>

namespace NXUtils
{
    /**
     * 初始化 NX Open API
     * @return 0 成功，非0 失败
     */
    int Initialize();

    /**
     * 终止 NX Open API
     */
    void Terminate();

    /**
     * 打印信息窗口消息
     * @param message 要显示的消息
     */
    void PrintMessage(const std::string& message);

    /**
     * 获取当前工作部件
     * @param partTag 输出参数：部件 tag
     * @return 0 成功，非0 失败
     */
    int GetWorkPart(tag_t& partTag);

    /**
     * 获取部件中所有实体对象的 tag 列表
     * @param partTag  部件 tag
     * @param objType  NX 对象类型（如 UF_solid_type）
     * @param objSubtype 子类型（如 UF_solid_body_subtype）
     * @param tags     输出：对象 tag 向量
     * @return 0 成功，非0 失败
     */
    int GetObjectsInPart(tag_t partTag, int objType, int objSubtype,
                         std::vector<tag_t>& tags);

    /**
     * 将笛卡尔坐标转换为字符串（调试用）
     * @param point  三维点坐标数组
     * @return 格式化字符串 "(x, y, z)"
     */
    std::string PointToString(const double point[3]);

} // namespace NXUtils

#endif // NX_UTILS_H
