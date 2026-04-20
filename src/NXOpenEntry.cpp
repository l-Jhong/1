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
#include <algorithm>
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

#ifndef DEBUG_ENVELOPE_SUBTRACTION
#define DEBUG_ENVELOPE_SUBTRACTION 0
#endif

namespace {

NXOpen::Body* createExtrudedRectangularSolid(const casting::Bounds& bounds, int color, int layer = -1);

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

    // Build a simplified box mesh from body bounds so each part yields unique input geometry.
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

// outSolidCount / outSheetCount can be nullptr when the caller does not need counts.
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

    // Read bodies from the current work part and build analysis mesh.
    NXOpen::Part* part = resolveWorkPart(workPart);
    if (!part) {
        return mesh;
    }
    NXOpen::BodyCollection* bodyCollection = part->Bodies();
    if (!bodyCollection) {
        return mesh;
    }

    // First pass: process solid bodies only.
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

    // Second pass: if no solids exist, fall back to all bodies (sheet bodies from STEP/STL etc.).
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

    // If no mesh can still be built, fall back to a demo mesh.
    if (mesh.triangles.empty()) {
        mesh = buildDemoMesh(10.0);
    }
    return mesh;
}

bool applyBooleanFeature(NXOpen::Part* part,
                         NXOpen::Body* targetBody,
                         NXOpen::Body* toolBody,
                         NXOpen::Features::Feature::BooleanType operation,
                         bool retainTool = true) {
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
        booleanBuilder->SetRetainTool(retainTool);
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

casting::Bounds askBodyBounds(tag_t bodyTag) {
    double box[6]{};
    casting::Bounds bounds{};
    if (UF_MODL_ask_bounding_box(bodyTag, box) != 0) {
        return bounds;
    }
    bounds.min = {box[0], box[1], box[2]};
    bounds.max = {box[3], box[4], box[5]};
    return bounds;
}

bool isBoundsInside(const casting::Bounds& inner, const casting::Bounds& outer, double tolerance) {
    return inner.min.x >= outer.min.x - tolerance &&
           inner.min.y >= outer.min.y - tolerance &&
           inner.min.z >= outer.min.z - tolerance &&
           inner.max.x <= outer.max.x + tolerance &&
           inner.max.y <= outer.max.y + tolerance &&
           inner.max.z <= outer.max.z + tolerance;
}

bool isBoundsOverlapping(const casting::Bounds& left, const casting::Bounds& right, double tolerance) {
    return left.max.x >= right.min.x - tolerance && left.min.x <= right.max.x + tolerance &&
           left.max.y >= right.min.y - tolerance && left.min.y <= right.max.y + tolerance &&
           left.max.z >= right.min.z - tolerance && left.min.z <= right.max.z + tolerance;
}

NXOpen::Body* createExtractionEnvelope(const casting::Bounds& bounds,
                                       const casting::Vector3& pullDir,
                                       double padding) {
    casting::Bounds expanded = bounds;
    expanded.min.x -= padding;
    expanded.min.y -= padding;
    expanded.min.z -= padding;
    expanded.max.x += padding;
    expanded.max.y += padding;
    expanded.max.z += padding;
    // TODO: Build a strict envelope from maxContour projection with side walls parallel to pullDir.
    (void)pullDir;
    return createExtrudedRectangularSolid(expanded, casting::kCoreColor, casting::kSandCoreBaseLayer);
}

std::vector<NXOpen::Body*> extractInternalVolumes(NXOpen::Body* body) {
    std::vector<NXOpen::Body*> internalBodies;
    if (!body) {
        return internalBodies;
    }

    NXOpen::Part* part = dynamic_cast<NXOpen::Part*>(body->OwningPart());
    if (!part || !part->Bodies()) {
        return internalBodies;
    }

    // TODO: Implement topological closed-cavity extraction (UF_MODL_ask_body_boundaries / UF_MODL_ask_face_loops).
    casting::Bounds moldBounds = askBodyBounds(body->Tag());
    constexpr double kBoundsContainmentTolerance = 1e-4;
    for (NXOpen::Body* candidate : *part->Bodies()) {
        if (!candidate || candidate == body || !candidate->IsSolidBody()) {
            continue;
        }
        casting::Bounds candidateBounds = askBodyBounds(candidate->Tag());
        if (!isBoundsOverlapping(candidateBounds, moldBounds, kBoundsContainmentTolerance)) {
            continue;
        }
        if (isBoundsInside(candidateBounds, moldBounds, kBoundsContainmentTolerance)) {
            internalBodies.push_back(candidate);
        }
    }
    return internalBodies;
}

std::vector<NXOpen::Body*> extractInternalCoresManually(NXOpen::Part* part, NXOpen::Body* intermediateBody) {
    std::vector<NXOpen::Body*> internalCores;
    if (!part || !intermediateBody) {
        return internalCores;
    }

    // 中文提示：请用户在 NX 中手动选择封闭空腔的全部内表面。
    NXOpen::UI* ui = NXOpen::UI::GetUI();
    if (ui && ui->NXMessageBox()) {
        ui->NXMessageBox()->Show(
            "内部砂芯提取",
            NXOpen::NXMessageBox::DialogTypeInformation,
            "当前为框架实现：请后续接入面选择对话框与缝合成体流程。");
    }

    // TODO: 使用 SelectionManager / UF_UI_select_with_class_dialog 弹出面选择对话框，
    //       将用户选中的 Face 收集为 faceTags。
    // TODO: 对 faceTags 执行缝合（sew）或抽取为片体，再尝试加厚/封闭生成实体 Body。
    // TODO: 将生成的内部砂芯实体加入 internalCores 返回。
    (void)part;
    (void)intermediateBody;
    return internalCores;
}

std::vector<NXOpen::Body*> splitIntoConnectedBodies(NXOpen::Body* body) {
    std::vector<NXOpen::Body*> connectedBodies;
    if (!body) {
        return connectedBodies;
    }

    // TODO: 连通域分离框架
    // 1) 使用 UF_MODL_ask_body_faces 获取 body 的所有面。
    // 2) 基于“共享边/共享顶点”构建面邻接图。
    // 3) 对邻接图执行 BFS/DFS，得到多个连通分量。
    // 4) 每个连通分量调用 NX/UF 的抽取几何接口生成独立 Body。
    // 5) 返回所有独立 Body；若无法分离则返回原始 body。
    connectedBodies.push_back(body);
    return connectedBodies;
}

std::vector<tag_t> collectSolidBodyTags(BasePart* workPart) {
    std::vector<tag_t> tags;
    NXOpen::Part* part = resolveWorkPart(workPart);
    if (!part) {
        return tags;
    }
    NXOpen::BodyCollection* bodyCollection = part->Bodies();
    if (!bodyCollection) {
        return tags;
    }
    for (NXOpen::Body* body : *bodyCollection) {
        if (body && body->IsSolidBody()) {
            tags.push_back(body->Tag());
        }
    }
    return tags;
}

NXOpen::Body* createExtrudedRectangularSolid(const casting::Bounds& bounds, int color, int layer) {
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
    if (layer > 0) {
        UF_OBJ_set_layer(bodyTag, layer);
    }
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
    void showSandCores(const std::vector<casting::SandCore>& sandCores);
    void showSandCoreRecursive(const casting::SandCore& sandCore);

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
    NXOpen::Part* part = resolveWorkPart(workPart);
    if (!part) {
        return;
    }
    std::vector<tag_t> partSolidTags = collectSolidBodyTags(workPart);
    NXOpen::Body* partBody = nullptr;
    if (!partSolidTags.empty()) {
        partBody = dynamic_cast<NXOpen::Body*>(NXOpen::NXObjectManager::Get(partSolidTags.front()));
    }

    // Build a solid matching the bounds via "rectangular profile + extrusion"; return nullptr on failure.
    auto createBlock = [](const casting::Bounds& bounds, int color) -> NXOpen::Body* {
        return createExtrudedRectangularSolid(bounds, color);
    };

    // Display upper/lower molds with different colors (display only, no file export).
    // Continue boolean operations on the tag only when bound solids are created successfully.
    NXOpen::Body* upperBody = nullptr;
    NXOpen::Body* lowerBody = nullptr;
    if (assembly.blocks.size() >= 2) {
        upperBody = createBlock(assembly.blocks[0].bounds, casting::kUpperMoldColor);
        lowerBody = createBlock(assembly.blocks[1].bounds, casting::kLowerMoldColor);
    }

    // Create sand-core geometry only when both upper/lower mold blanks were created.
    if (!upperBody || !lowerBody) {
        return;
    }

    // 中文步骤1：基于 cavityBounds 扩展 20mm，构建提取包容盒。
    NXOpen::Body* envelopeBody = nullptr;
    NXOpen::Body* intermediateBody = nullptr;
    if (!assembly.blocks.empty()) {
        casting::Bounds envelopeBounds = assembly.blocks[0].cavityBounds;
        constexpr double kEnvelopePadding = 20.0;
        envelopeBounds.min.x -= kEnvelopePadding;
        envelopeBounds.min.y -= kEnvelopePadding;
        envelopeBounds.min.z -= kEnvelopePadding;
        envelopeBounds.max.x += kEnvelopePadding;
        envelopeBounds.max.y += kEnvelopePadding;
        envelopeBounds.max.z += kEnvelopePadding;
        envelopeBody = createExtrudedRectangularSolid(
            envelopeBounds, casting::kCoreColor, casting::kSandCoreBaseLayer);
    }

    // 中文步骤2：执行 envelopeBody - partBody，得到 intermediateBody。
    if (envelopeBody && partBody) {
        if (applyBooleanFeature(part, envelopeBody, partBody,
                                NXOpen::Features::Feature::BooleanTypeSubtract, true)) {
            intermediateBody = envelopeBody;
        }
    }

    // 中文步骤3：手动提取内部砂芯，并从 intermediateBody 中减去（保留砂芯工具体）。
    std::vector<NXOpen::Body*> internalCores;
    std::vector<NXOpen::Body*> externalCores;
    if (intermediateBody) {
        internalCores = extractInternalCoresManually(part, intermediateBody);
        for (NXOpen::Body* coreBody : internalCores) {
            if (!coreBody) {
                continue;
            }
            UF_OBJ_set_color(coreBody->Tag(), casting::kCoreColor);
            UF_OBJ_set_layer(coreBody->Tag(), casting::kSandCoreBaseLayer);
            applyBooleanFeature(part, intermediateBody, coreBody,
                                NXOpen::Features::Feature::BooleanTypeSubtract, true);
        }

        // 中文步骤4：剩余体按连通域分离，作为外部砂芯。
        externalCores = splitIntoConnectedBodies(intermediateBody);
        for (NXOpen::Body* coreBody : externalCores) {
            if (!coreBody) {
                continue;
            }
            UF_OBJ_set_color(coreBody->Tag(), casting::kCoreColor);
            UF_OBJ_set_layer(coreBody->Tag(), casting::kSandCoreBaseLayer);
        }
    }

    // 中文步骤5：先从上下模毛坯中减去内部/外部砂芯（保留砂芯工具体）。
    for (NXOpen::Body* moldBody : {upperBody, lowerBody}) {
        for (NXOpen::Body* internalCore : internalCores) {
            applyBooleanFeature(part, moldBody, internalCore,
                                NXOpen::Features::Feature::BooleanTypeSubtract, true);
        }
        for (NXOpen::Body* externalCore : externalCores) {
            applyBooleanFeature(part, moldBody, externalCore,
                                NXOpen::Features::Feature::BooleanTypeSubtract, true);
        }
    }

    // 中文步骤6：最后执行 upperBody - partBody 与 lowerBody - partBody（不保留工具体）。
    if (partBody) {
        // 先保留一次工具体，确保同一零件体可用于两次减法；最后一次不保留。
        applyBooleanFeature(part, upperBody, partBody,
                            NXOpen::Features::Feature::BooleanTypeSubtract, true);
        applyBooleanFeature(part, lowerBody, partBody,
                            NXOpen::Features::Feature::BooleanTypeSubtract, false);
    }

    // 中文步骤7：清理临时体 envelopeBody / intermediateBody。
    std::vector<tag_t> tempBodyTags;
    if (envelopeBody) {
        tempBodyTags.push_back(envelopeBody->Tag());
    }
    if (intermediateBody) {
        tempBodyTags.push_back(intermediateBody->Tag());
    }
    std::sort(tempBodyTags.begin(), tempBodyTags.end());
    tempBodyTags.erase(std::unique(tempBodyTags.begin(), tempBodyTags.end()), tempBodyTags.end());
    for (tag_t bodyTag : tempBodyTags) {
        UF_OBJ_delete_object(bodyTag);
    }
}

void MyClass::showSandCoreRecursive(const casting::SandCore& sandCore) {
    createExtrudedRectangularSolid(
        sandCore.geometryBounds, sandCore.nxColor, sandCore.nxLayer);

    for (const auto& head : sandCore.heads) {
        casting::Bounds headBounds = sandCore.geometryBounds;
        double span = std::max(1.0, head.length);
        headBounds.min.x = head.position.x - span * 0.25;
        headBounds.max.x = head.position.x + span * 0.25;
        headBounds.min.y = head.position.y - span * 0.25;
        headBounds.max.y = head.position.y + span * 0.25;
        headBounds.min.z = head.position.z - span * 0.25;
        headBounds.max.z = head.position.z + span * 0.25;
        createExtrudedRectangularSolid(headBounds, casting::kCoreHeadColor, sandCore.nxLayer);
    }

    for (const auto& subCore : sandCore.subCores) {
        showSandCoreRecursive(subCore);
    }
}

void MyClass::showSandCores(const std::vector<casting::SandCore>& sandCores) {
    if (!workPart) {
        return;
    }
    for (const auto& sandCore : sandCores) {
        showSandCoreRecursive(sandCore);
    }
}
//------------------------------------------------------------------------------
// Do something
//------------------------------------------------------------------------------
void MyClass::do_it() {
    casting::AutoPartingPipeline pipeline;

    // Prefer geometry from the current part to avoid identical results across different parts.
    bool usedSheetBodies = false;
    int  solidCount      = 0;
    int  sheetCount      = 0;
    casting::Mesh mesh = buildMeshFromWorkPart(workPart, &usedSheetBodies,
                                               &solidCount, &sheetCount);

    // Print body-type statistics to help confirm current part geometry state.
    stringstream bodyStats;
    bodyStats << "Body stats - solids: " << solidCount << " | sheets: " << sheetCount;
    print(bodyStats.str());

    // Show a friendly hint when the current part contains sheet bodies only.
    if (usedSheetBodies) {
        print("Warning: no solid body found; analysis will run on sheet bodies (from STEP/STL). "
              "For full mold operations, convert to solid first (for example, sew or thicken).");
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
           << " | Core regions: " << result.cores.size()
           << " | Sand cores: " << result.sandCores.size();
    print(stream.str());

    stream.str("");
    stream.clear();
    stream << "Separability: " << (result.separability.separable ? "separable" : "not separable")
           << " | Obstacles: " << result.separability.obstacles.size()
           << " | Max contour area: " << result.maxContour.area;
    print(stream.str());

    stream.str("");
    stream.clear();
    stream << "Contour source: " << (result.maxContour.selectedFromSlice ? "slice" : "fallback")
           << " | Slice index: " << result.maxContour.selectedSliceIndex
           << " | Local W: " << result.maxContour.selectedSliceW
           << " | Used fallback: " << (result.maxContour.fallbackUsed ? "yes" : "no");
    print(stream.str());

    highlightPartingSurface(result.partingSurface.boundary);
    showMoldAssembly(result.moldAssembly);
    showSandCores(result.sandCores);
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
        UI::GetUI()->NXMessageBox()->Show("NX Exception", NXOpen::NXMessageBox::DialogTypeError,
                                          e1.Message());
    } catch (const exception& e2) {
        UI::GetUI()->NXMessageBox()->Show("Exception", NXOpen::NXMessageBox::DialogTypeError,
                                          e2.what());
    } catch (...) {
        UI::GetUI()->NXMessageBox()->Show("Exception", NXOpen::NXMessageBox::DialogTypeError,
                                          "Unknown exception.");
    }
    UF_terminate();
}

//------------------------------------------------------------------------------
// Unload Handler
//------------------------------------------------------------------------------
extern "C" DllExport int ufusr_ask_unload() {
    return static_cast<int>(NXOpen::Session::LibraryUnloadOptionImmediately);
}
