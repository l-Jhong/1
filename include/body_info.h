#pragma once
/**
 * body_info.h
 * 实体（Body）信息查询与输出接口声明
 */

#ifndef BODY_INFO_H
#define BODY_INFO_H

#include <uf.h>

namespace BodyInfo
{
    /**
     * 输出指定实体的基本几何信息到信息窗口
     * @param bodyTag  实体对象 tag
     * @param index    序号（从 1 开始，用于显示）
     */
    void PrintBodyInfo(tag_t bodyTag, int index);

} // namespace BodyInfo

#endif // BODY_INFO_H
