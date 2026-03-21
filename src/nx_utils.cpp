/**
 * nx_utils.cpp
 * NX 12.0 二次开发公用工具函数实现
 */

#include "nx_utils.h"

#include <uf.h>
#include <uf_ui.h>
#include <uf_obj.h>
#include <uf_part.h>

#include <cstdio>
#include <sstream>
#include <stdexcept>

namespace NXUtils
{
    int Initialize()
    {
        int errorCode = UF_initialize();
        return errorCode;
    }

    void Terminate()
    {
        UF_terminate();
    }

    void PrintMessage(const std::string& message)
    {
        UF_UI_open_listing_window();
        UF_UI_write_listing_window(message.c_str());
        UF_UI_write_listing_window("\n");
    }

    int GetWorkPart(tag_t& partTag)
    {
        partTag = UF_PART_ask_display_part();
        if (partTag == NULL_TAG)
        {
            return -1;
        }
        return 0;
    }

    int GetObjectsInPart(tag_t partTag, int objType, int objSubtype,
                         std::vector<tag_t>& tags)
    {
        tags.clear();

        int errorCode = 0;
        tag_t objectTag = NULL_TAG;

        // 使用 UF_OBJ_cycle_objs_in_part 遍历部件中所有指定类型的对象
        errorCode = UF_OBJ_cycle_objs_in_part(partTag, objType, &objectTag);
        while (errorCode == 0 && objectTag != NULL_TAG)
        {
            int type = 0, subtype = 0;
            UF_OBJ_ask_type_and_subtype(objectTag, &type, &subtype);

            if (subtype == objSubtype || objSubtype == -1)
            {
                tags.push_back(objectTag);
            }

            errorCode = UF_OBJ_cycle_objs_in_part(partTag, objType, &objectTag);
        }

        // Normal termination: objectTag became NULL_TAG (no more objects).
        // Only propagate errorCode if a genuine mid-cycle error occurred.
        return (objectTag == NULL_TAG) ? 0 : errorCode;
    }

    std::string PointToString(const double point[3])
    {
        std::ostringstream oss;
        oss << "(" << point[0] << ", " << point[1] << ", " << point[2] << ")";
        return oss.str();
    }

} // namespace NXUtils
