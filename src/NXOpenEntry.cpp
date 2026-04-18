#include "AutoPartingPipeline.h"

// Mandatory UF Includes
#include <uf.h>
#include <uf_curve.h>
#include <uf_object_types.h>
#include <uf_modl.h>
#include <uf_obj.h>

// Internal Includes
#include <NXOpen/ListingWindow.hxx>
#include <NXOpen/NXMessageBox.hxx>
#include <NXOpen/UI.hxx>

// Internal+External Includes
#include <NXOpen/Annotations.hxx>
#include <NXOpen/Assemblies_Component.hxx>
#include <NXOpen/Assemblies_ComponentAssembly.hxx>
#include <NXOpen/Body.hxx>
#include <NXOpen/BodyCollection.hxx>
#include <NXOpen/CurveCollection.hxx>
#include <NXOpen/Face.hxx>
#include <NXOpen/Line.hxx>
#include <NXOpen/NXException.hxx>
#include <NXOpen/NXObject.hxx>
#include <NXOpen/NXObjectManager.hxx>
#include <NXOpen/Part.hxx>
#include <NXOpen/PartCollection.hxx>
#include <NXOpen/Point3d.hxx>
#include <NXOpen/Session.hxx>
#include <NXOpen/Features_BooleanBuilder.hxx>
#include <NXOpen/Features_FeatureCollection.hxx>

// Std C++ Includes
#include <cstdio>
#include <iostream>
#include <limits>
#include <sstream>

using namespace NXOpen;
using std::string;
using std::exception;
using std::stringstream;
using std::endl;
using std::cout;
using std::cerr;

namespace {

NXOpen::Part* resolveWorkPart(BasePart* workPart) {
    if (NXOpen::Part* part = dynamic_cast<NXOpen::Part*>(workPart)) {
        return part;
    }
    NXOpen::Session* session = NXOpen::Session::GetSession();
    if (!session || !session->Parts()) {
        return nullptr;
    }
    return session->Parts()->Work();
}

casting::Mesh buildDemoMesh(double size) {
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

bool appendBoundingBoxMesh(tag_t bodyTag, casting::Mesh* mesh) {
    if (!mesh) {
        return false;
    }
    double box[6]{};
    if (UF_MODL_ask_bounding_box(bodyTag, box) != 0) {
        return false;
    }

    // 使用体的包围盒尺寸生成简化盒形网格，保证不同零件有不同的输入几何
    double xmin = box[0];
    double ymin = box[1];
    double zmin = box[2];
    double xmax = box[3];
    double ymax = box[4];
    double zmax = box[5];

    std::size_t base = mesh->vertices.size();
    mesh->vertices.push_back({xmin, ymin, zmin});
    mesh->vertices.push_back({xmax, ymin, zmin});
    mesh->vertices.push_back({xmax, ymax, zmin});
    mesh->vertices.push_back({xmin, ymax, zmin});
    mesh->vertices.push_back({xmin, ymin, zmax});
    mesh->vertices.push_back({xmax, ymin, zmax});
    mesh->vertices.push_back({xmax, ymax, zmax});
    mesh->vertices.push_back({xmin, ymax, zmax});

    mesh->triangles.push_back({base + 0, base + 1, base + 2});
    mesh->triangles.push_back({base + 0, base + 2, base + 3});
    mesh->triangles.push_back({base + 4, base + 6, base + 5});
    mesh->triangles.push_back({base + 4, base + 7, base + 6});
    mesh->triangles.push_back({base + 0, base + 4, base + 5});
    mesh->triangles.push_back({base + 0, base + 5, base + 1});
    mesh->triangles.push_back({base + 1, base + 5, base + 6});
    mesh->triangles.push_back({base + 1, base + 6, base + 2});
    mesh->triangles.push_back({base + 2, base + 6, base + 7});
    mesh->triangles.push_back({base + 2, base + 7, base + 3});
    mesh->triangles.push_back({base + 3, base + 7, base + 4});
    mesh->triangles.push_back({base + 3, base + 4, base + 0});
    return true;
}

// outSolidCount / outSheetCount 可为 nullptr，调用方不需要统计时传入 nullptr
casting::Mesh buildMeshFromWorkPart(BasePart* workPart,
                                    bool* outUsedSheetBodies = nullptr,
                                    int*  outSolidCount     = nullptr,
                                    int*  outSheetCount     = nullptr) {
    casting::Mesh mesh;
    if (outUsedSheetBodies) *outUsedSheetBodies = false;
    if (outSolidCount)      *outSolidCount      = 0;
    if (outSheetCount)      *outSheetCount      = 0;

    if (!workPart) {
        return mesh;
    }

    // 读取当前工作部件的体，生成用于分析的网格
    NXOpen::Part* part = resolveWorkPart(workPart);
    if (!part) {
        return mesh;
    }
    NXOpen::BodyCollection* bodyCollection = part->Bodies();
    if (!bodyCollection) {
        return mesh;
    }

    // 第一遍：仅处理实体（solid body）
    std::vector<NXOpen::Body*> sheetBodies;
    for (NXOpen::Body* body : *bodyCollection) {
        if (!body) {
            continue;
        }
        if (body->IsSolidBody()) {
            if (outSolidCount) (*outSolidCount)++;
            appendBoundingBoxMesh(body->Tag(), &mesh);
        } else {
            if (outSheetCount) (*outSheetCount)++;
            sheetBodies.push_back(body);
        }
    }

    // 第二遍：若无实体，回退到所有体（面片体，来自 STEP/STL 等中间格式）
    if (mesh.triangles.empty()) {
        for (NXOpen::Body* body : sheetBodies) {
            if (!body) {
                continue;
            }
            appendBoundingBoxMesh(body->Tag(), &mesh);
        }
        if (!mesh.triangles.empty() && outUsedSheetBodies) {
            *outUsedSheetBodies = true;
        }
    }

    // 如果仍未获取到任何网格，则退回到示例网格
    if (mesh.triangles.empty()) {
        mesh = buildDemoMesh(10.0);
    }
    return mesh;
}

bool applyBooleanFeature(NXOpen::Part* part,
                         NXOpen::Body* targetBody,
                         NXOpen::Body* toolBody,
                         NXOpen::Features::Feature::BooleanType operation) {
    if (!part || !targetBody || !toolBody) {
        return false;
    }

    NXOpen::Features::BooleanBuilder* booleanBuilder = nullptr;
    try {
        booleanBuilder = part->Features()->CreateBooleanBuilder(nullptr);
        booleanBuilder->SetOperation(operation);
        booleanBuilder->SetTarget(targetBody);
        booleanBuilder->SetTool(toolBody);
        booleanBuilder->SetRetainTarget(false);
        booleanBuilder->SetRetainTool(false);
        booleanBuilder->CommitFeature();
        booleanBuilder->Destroy();
        return true;
    } catch (...) {
        if (booleanBuilder) {
            booleanBuilder->Destroy();
        }
        return false;
    }
}

NXOpen::Body* createExtrudedRectangularSolid(const casting::Bounds& bounds, int color) {
    double edgeX = bounds.max.x - bounds.min.x;
    double edgeY = bounds.max.y - bounds.min.y;
    double edgeZ = bounds.max.z - bounds.min.z;
    if (edgeX <= 0.0 || edgeY <= 0.0 || edgeZ <= 0.0) {
        return nullptr;
    }

    UF_CURVE_line_t edges[4]{};
    edges[0].start_point[0] = bounds.min.x; edges[0].start_point[1] = bounds.min.y; edges[0].start_point[2] = bounds.min.z;
    edges[0].end_point[0]   = bounds.max.x; edges[0].end_point[1]   = bounds.min.y; edges[0].end_point[2]   = bounds.min.z;
    edges[1].start_point[0] = bounds.max.x; edges[1].start_point[1] = bounds.min.y; edges[1].start_point[2] = bounds.min.z;
    edges[1].end_point[0]   = bounds.max.x; edges[1].end_point[1]   = bounds.max.y; edges[1].end_point[2]   = bounds.min.z;
    edges[2].start_point[0] = bounds.max.x; edges[2].start_point[1] = bounds.max.y; edges[2].start_point[2] = bounds.min.z;
    edges[2].end_point[0]   = bounds.min.x; edges[2].end_point[1]   = bounds.max.y; edges[2].end_point[2]   = bounds.min.z;
    edges[3].start_point[0] = bounds.min.x; edges[3].start_point[1] = bounds.max.y; edges[3].start_point[2] = bounds.min.z;
    edges[3].end_point[0]   = bounds.min.x; edges[3].end_point[1]   = bounds.min.y; edges[3].end_point[2]   = bounds.min.z;

    tag_t edgeTags[4] = {NULL_TAG, NULL_TAG, NULL_TAG, NULL_TAG};
    for (int i = 0; i < 4; ++i) {
        if (UF_CURVE_create_line(&edges[i], &edgeTags[i]) != 0 || edgeTags[i] == NULL_TAG) {
            for (int j = 0; j < 4; ++j) {
                if (edgeTags[j] != NULL_TAG) {
                    UF_OBJ_delete_object(edgeTags[j]);
                }
            }
            return nullptr;
        }
    }

    uf_list_p_t sectionList = nullptr;
    if (UF_MODL_create_list(&sectionList) != 0 || !sectionList) {
        for (tag_t edgeTag : edgeTags) {
            UF_OBJ_delete_object(edgeTag);
        }
        return nullptr;
    }
    for (tag_t edgeTag : edgeTags) {
        UF_MODL_put_list_item(sectionList, edgeTag);
    }

    char taperAngle[] = "0";
    char startLimit[] = "0";
    char endLimit[64]{};
    int precision = std::numeric_limits<double>::max_digits10;
    std::snprintf(endLimit, sizeof(endLimit), "%.*g", precision, edgeZ);
    char* limits[2] = {startLimit, endLimit};
    double point[3] = {bounds.min.x, bounds.min.y, bounds.min.z};
    double direction[3] = {0.0, 0.0, 1.0};

    uf_list_p_t createdObjects = nullptr;
    int rc = UF_MODL_create_extruded(sectionList, taperAngle, limits, point, direction,
                                     UF_NULLSIGN, &createdObjects);
    UF_MODL_delete_list(&sectionList);
    for (tag_t edgeTag : edgeTags) {
        UF_OBJ_delete_object(edgeTag);
    }
    if (rc != 0 || !createdObjects) {
        return nullptr;
    }

    tag_t createdTag = NULL_TAG;
    if (UF_MODL_ask_list_item(createdObjects, 0, &createdTag) != 0 || createdTag == NULL_TAG) {
        UF_MODL_delete_list(&createdObjects);
        return nullptr;
    }

    tag_t bodyTag = NULL_TAG;
    if (UF_MODL_ask_feat_body(createdTag, &bodyTag) != 0 || bodyTag == NULL_TAG) {
        bodyTag = createdTag;
    }

    UF_MODL_delete_list(&createdObjects);
    if (bodyTag == NULL_TAG) {
        return nullptr;
    }

    UF_OBJ_set_color(bodyTag, color);
    return dynamic_cast<NXOpen::Body*>(NXOpen::NXObjectManager::Get(bodyTag));
}

}  // namespace

//------------------------------------------------------------------------------
// NXOpen C++ integration class
//------------------------------------------------------------------------------
class MyClass {
    // class members
public:
    static Session* theSession;
    static UI* theUI;

    MyClass();
    ~MyClass();

    void do_it();
    void print(const NXString&);
    void print(const string&);
    void print(const char*);
    void highlightPartingSurface(const std::vector<casting::Vector3>& boundary);
    void showMoldAssembly(const casting::MoldAssembly& assembly);

private:
    BasePart* workPart;
    BasePart* displayPart;
    NXMessageBox* mb;
    ListingWindow* lw;
    LogFile* lf;
};

//------------------------------------------------------------------------------
// Initialize static variables
//------------------------------------------------------------------------------
Session* (MyClass::theSession) = nullptr;
UI* (MyClass::theUI) = nullptr;

//------------------------------------------------------------------------------
// Constructor
//------------------------------------------------------------------------------
MyClass::MyClass() {
    // Initialize the NX Open C++ API environment
    MyClass::theSession = NXOpen::Session::GetSession();
    MyClass::theUI = UI::GetUI();
    mb = theUI->NXMessageBox();
    lw = theSession->ListingWindow();
    lf = theSession->LogFile();

    workPart = theSession->Parts()->BaseWork();
    displayPart = theSession->Parts()->BaseDisplay();
}

//------------------------------------------------------------------------------
// Destructor
//------------------------------------------------------------------------------
MyClass::~MyClass() {}

//------------------------------------------------------------------------------
// Print string to listing window or stdout
//------------------------------------------------------------------------------
void MyClass::print(const NXString& msg) {
    if (!lw->IsOpen()) {
        lw->Open();
    }
    lw->WriteLine(msg);
}
void MyClass::print(const string& msg) {
    if (!lw->IsOpen()) {
        lw->Open();
    }
    lw->WriteLine(msg);
}
void MyClass::print(const char* msg) {
    if (!lw->IsOpen()) {
        lw->Open();
    }
    lw->WriteLine(msg);
}

void MyClass::highlightPartingSurface(const std::vector<casting::Vector3>& boundary) {
    if (!workPart || boundary.size() < 2) {
        return;
    }
    for (std::size_t i = 0; i < boundary.size() - 1; ++i) {
        const auto& start = boundary[i];
        const auto& end = boundary[i + 1];
        NXOpen::Point3d p0(start.x, start.y, start.z);
        NXOpen::Point3d p1(end.x, end.y, end.z);
        NXOpen::Line* line = workPart->Curves()->CreateLine(p0, p1);
        if (line) {
            line->SetColor(186);
        }
    }
}

void MyClass::showMoldAssembly(const casting::MoldAssembly& assembly) {
    if (!workPart) {
        return;
    }
    // 通过“矩形轮廓 + 拉伸”生成与包围盒同尺寸的实体；若失败返回 nullptr。
    auto createBlock = [](const casting::Bounds& bounds, int color) -> NXOpen::Body* {
        return createExtrudedRectangularSolid(bounds, color);
    };

    // 使用不同颜色展示上、下模（仅显示，不输出文件）
    // 只有在包围盒成功生成后，才可对该 tag 进行后续布尔运算
    NXOpen::Body* upperBody = nullptr;
    NXOpen::Body* lowerBody = nullptr;
    if (assembly.blocks.size() >= 2) {
        upperBody = createBlock(assembly.blocks[0].bounds, casting::kUpperMoldColor);
        lowerBody = createBlock(assembly.blocks[1].bounds, casting::kLowerMoldColor);
    }

    // 仅在上下模包围盒均已成功生成的前提下，才创建砂芯几何（避免对空体执行布尔运算）
    if (!upperBody || !lowerBody) {
        return;
    }

    // 按收缩率放大的零件型腔（cavityBounds）从上下模坯料中布尔减去，得到实际铸型。
    // 统一通过 NXOpen Boolean Builder：目标体 + 工具体 + 操作类型（Subtract）。
    NXOpen::Part* part = resolveWorkPart(workPart);
    if (!part) {
        return;
    }

    if (assembly.blocks[0].subtractPart) {
        NXOpen::Body* upperCavity = createBlock(assembly.blocks[0].cavityBounds, 0);
        if (upperCavity) {
            applyBooleanFeature(part, upperBody, upperCavity,
                                NXOpen::Features::Feature::BooleanTypeSubtract);
        }
    }
    if (assembly.blocks[1].subtractPart) {
        NXOpen::Body* lowerCavity = createBlock(assembly.blocks[1].cavityBounds, 0);
        if (lowerCavity) {
            applyBooleanFeature(part, lowerBody, lowerCavity,
                                NXOpen::Features::Feature::BooleanTypeSubtract);
        }
    }

    for (const auto& core : assembly.cores) {
        createBlock(core.bodyBounds, casting::kCoreColor);
        createBlock(core.headBounds, casting::kCoreHeadColor);
    }
}
//------------------------------------------------------------------------------
// Do something
//------------------------------------------------------------------------------
void MyClass::do_it() {
    casting::AutoPartingPipeline pipeline;

    // 优先使用当前零件的几何数据，避免不同零件得到相同结果
    bool usedSheetBodies = false;
    int  solidCount      = 0;
    int  sheetCount      = 0;
    casting::Mesh mesh = buildMeshFromWorkPart(workPart, &usedSheetBodies,
                                               &solidCount, &sheetCount);

    // 打印体类型统计，帮助用户确认当前零件的几何状态
    stringstream bodyStats;
    bodyStats << "Body stats — Solid: " << solidCount << " | Sheet: " << sheetCount;
    print(bodyStats.str());

    // 若当前零件只含面片体（来自 STEP/STL 等中间格式），给出友好提示
    if (usedSheetBodies) {
        print("Warning: no solid bodies found. Analysis is running on sheet bodies "
              "(imported from STEP/STL). For full mold operations, please convert "
              "them to solids using the Sew or Thicken command first.");
    }

    casting::AutoPartingResult result = pipeline.run(mesh);

    stringstream stream;
    stream << "Demold direction: (" << result.demold.direction.x << ", "
           << result.demold.direction.y << ", " << result.demold.direction.z << ")";
    print(stream.str());

    stream.str("");
    stream.clear();
    stream << "Visibility ratio: " << result.demold.visibilityRatio
           << " | Undercut ratio: " << result.demold.undercutRatio;
    print(stream.str());

    stream.str("");
    stream.clear();
    stream << "Parting line points: " << result.partingLine.points.size()
           << " | Core regions: " << result.cores.size();
    print(stream.str());

    stream.str("");
    stream.clear();
    stream << "Separability: " << (result.separability.separable ? "separable" : "not separable")
           << " | Obstacles: " << result.separability.obstacles.size()
           << " | Max contour area: " << result.maxContour.area;
    print(stream.str());

    highlightPartingSurface(result.partingSurface.boundary);
    showMoldAssembly(result.moldAssembly);
}

//------------------------------------------------------------------------------
// Entry point(s) for unmanaged internal NXOpen C/C++ programs
//------------------------------------------------------------------------------
//  Explicit Execution
extern "C" DllExport void ufusr(char* parm, int* returnCode, int rlen) {
    if (UF_initialize() != 0) {
        UI::GetUI()->NXMessageBox()->Show("UF Error", NXOpen::NXMessageBox::DialogTypeError,
                                          "UF_initialize failed.");
        return;
    }
    try {
        // Create NXOpen C++ class instance
        MyClass* theMyClass;
        theMyClass = new MyClass();
        theMyClass->do_it();
        delete theMyClass;
    } catch (const NXException& e1) {
        UI::GetUI()->NXMessageBox()->Show("NXException", NXOpen::NXMessageBox::DialogTypeError,
                                          e1.Message());
    } catch (const exception& e2) {
        UI::GetUI()->NXMessageBox()->Show("Exception", NXOpen::NXMessageBox::DialogTypeError,
                                          e2.what());
    } catch (...) {
        UI::GetUI()->NXMessageBox()->Show("Exception", NXOpen::NXMessageBox::DialogTypeError,
                                          "Unknown Exception.");
    }
    UF_terminate();
}

//------------------------------------------------------------------------------
// Unload Handler
//------------------------------------------------------------------------------
extern "C" DllExport int ufusr_ask_unload() {
    return static_cast<int>(NXOpen::Session::LibraryUnloadOptionImmediately);
}
