/**
 * body_info.cpp
 * 实体（Body）信息查询与输出实现
 *
 * 使用 NX Open UF API 查询实体的面数、边数、体积、质心等属性，
 * 并将结果写入 NX 信息窗口。
 */

#include "body_info.h"
#include "nx_utils.h"

#include <uf.h>
#include <uf_ui.h>
#include <uf_modl.h>
#include <uf_obj.h>
#include <uf_disp.h>

#include <sstream>
#include <vector>
#include <iomanip>

namespace BodyInfo
{
    void PrintBodyInfo(tag_t bodyTag, int index)
    {
        std::ostringstream oss;
        oss << "\n--- 实体 #" << index << " ---";
        NXUtils::PrintMessage(oss.str());
        oss.str("");

        // ---------------------------------------------------------------
        // 1. 查询实体面（Face）列表（使用 UF_MODL_ask_body_faces）
        // ---------------------------------------------------------------
        uf_list_p_t faceList = nullptr;
        int faceCount = 0;
        if (UF_MODL_ask_body_faces(bodyTag, &faceList) == 0)
        {
            UF_MODL_ask_list_count(faceList, &faceCount);
            UF_MODL_delete_list(&faceList);
        }

        oss << "面数：" << faceCount;
        NXUtils::PrintMessage(oss.str());
        oss.str("");

        // ---------------------------------------------------------------
        // 2. 查询实体边（Edge）列表（通过 UF_MODL_ask_body_edges）
        // ---------------------------------------------------------------
        uf_list_p_t edgeList = nullptr;
        int edgeCount = 0;
        if (UF_MODL_ask_body_edges(bodyTag, &edgeList) == 0)
        {
            UF_MODL_ask_list_count(edgeList, &edgeCount);
            UF_MODL_delete_list(&edgeList);
        }
        oss << "边数：" << edgeCount;
        NXUtils::PrintMessage(oss.str());
        oss.str("");

        // ---------------------------------------------------------------
        // 3. 查询体积、表面积、质心（UF_MODL_ask_mass_props_3d）
        //    density = 1.0，精度 = 0.99
        // ---------------------------------------------------------------
        double density = 1.0;
        double accuracy = 0.99;
        double massProps[47] = {0.0};
        int units = 1; // 1 = mm/kg

        if (UF_MODL_ask_mass_props_3d(bodyTag, &density, &accuracy, units,
                                       massProps) == 0)
        {
            // massProps[0] = 体积, massProps[1] = 表面积
            // massProps[3..5] = 质心 (x, y, z)
            oss << std::fixed << std::setprecision(4);
            oss << "体积：" << massProps[0] << " mm³";
            NXUtils::PrintMessage(oss.str());
            oss.str("");

            oss << std::fixed << std::setprecision(4);
            oss << "表面积：" << massProps[1] << " mm²";
            NXUtils::PrintMessage(oss.str());
            oss.str("");

            double centroid[3] = {massProps[3], massProps[4], massProps[5]};
            oss << "质心：" << NXUtils::PointToString(centroid);
            NXUtils::PrintMessage(oss.str());
            oss.str("");
        }
        else
        {
            NXUtils::PrintMessage("（无法获取质量属性）");
        }
    }

} // namespace BodyInfo
