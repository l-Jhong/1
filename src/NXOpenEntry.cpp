#include "casting/AutoPartingPipeline.h"

// Mandatory UF Includes
#include <uf.h>
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
#include <NXOpen/Part.hxx>
#include <NXOpen/PartCollection.hxx>
#include <NXOpen/Point3d.hxx>
#include <NXOpen/Session.hxx>

// Std C++ Includes
#include <array>
#include <cstdio>
#include <iostream>
#include <sstream>

using namespace NXOpen;
using std::string;
using std::exception;
using std::stringstream;
using std::endl;
using std::cout;
using std::cerr;

namespace {

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
    double minCorner[3]{};
    double maxCorner[3]{};
    if (UF_MODL_ask_bounding_box(bodyTag, minCorner, maxCorner) != 0) {
        return false;
    }

    // 使用体的包围盒生成一个近似网格，保证不同零件有不同的输入几何
    casting::Vector3 origin{minCorner[0], minCorner[1], minCorner[2]};
    casting::Vector3 axisU{1.0, 0.0, 0.0};
    casting::Vector3 axisV{0.0, 1.0, 0.0};
    casting::Vector3 axisW{0.0, 0.0, 1.0};
    double distances[3] = {maxCorner[0] - minCorner[0],
                           maxCorner[1] - minCorner[1],
                           maxCorner[2] - minCorner[2]};

    std::size_t base = mesh->vertices.size();
    mesh->vertices.push_back(origin);
    mesh->vertices.push_back(origin + axisU * distances[0]);
    mesh->vertices.push_back(origin + axisU * distances[0] + axisV * distances[1]);
    mesh->vertices.push_back(origin + axisV * distances[1]);
    mesh->vertices.push_back(origin + axisW * distances[2]);
    mesh->vertices.push_back(origin + axisU * distances[0] + axisW * distances[2]);
    mesh->vertices.push_back(origin + axisU * distances[0] + axisV * distances[1] +
                             axisW * distances[2]);
    mesh->vertices.push_back(origin + axisV * distances[1] + axisW * distances[2]);

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

casting::Mesh buildMeshFromWorkPart(BasePart* workPart) {
    casting::Mesh mesh;
    if (!workPart) {
        return mesh;
    }

    // 读取当前工作部件的实体，生成用于分析的网格
    NXOpen::Part* part = dynamic_cast<NXOpen::Part*>(workPart);
    if (!part) {
        return mesh;
    }
    std::vector<NXOpen::Body*> bodies = part->Bodies()->GetArray();
    for (NXOpen::Body* body : bodies) {
        if (!body) {
            continue;
        }
        appendBoundingBoxMesh(body->Tag(), &mesh);
    }

    // 如果没有成功获取实体网格，则退回到示例网格
    if (mesh.triangles.empty()) {
        mesh = buildDemoMesh(10.0);
    }
    return mesh;
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
    auto createBlock = [](const casting::Bounds& bounds, int color) {
        double edgeX = bounds.max.x - bounds.min.x;
        double edgeY = bounds.max.y - bounds.min.y;
        double edgeZ = bounds.max.z - bounds.min.z;
        if (edgeX <= 0.0 || edgeY <= 0.0 || edgeZ <= 0.0) {
            return;
        }
        double corner[3] = {bounds.min.x, bounds.min.y, bounds.min.z};
        std::array<std::array<char, 32>, 3> edgeBuffers{};
        std::snprintf(edgeBuffers[0].data(), edgeBuffers[0].size(), "%.6f", edgeX);
        std::snprintf(edgeBuffers[1].data(), edgeBuffers[1].size(), "%.6f", edgeY);
        std::snprintf(edgeBuffers[2].data(), edgeBuffers[2].size(), "%.6f", edgeZ);
        std::array<char*, 3> edges = {edgeBuffers[0].data(),
                                      edgeBuffers[1].data(),
                                      edgeBuffers[2].data()};
        tag_t blockTag = NULL_TAG;
        if (UF_MODL_create_block1(corner, UF_POSITIVE, edges.data(), &blockTag) == 0 &&
            blockTag != NULL_TAG) {
            UF_OBJ_set_color(blockTag, color);
        }
    };

    // 使用不同颜色展示上、下模和砂芯（仅显示，不输出文件）
    if (assembly.blocks.size() >= 2) {
        createBlock(assembly.blocks[0].bounds, casting::kUpperMoldColor);
        createBlock(assembly.blocks[1].bounds, casting::kLowerMoldColor);
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
    casting::Mesh mesh = buildMeshFromWorkPart(workPart);
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
