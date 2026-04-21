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
#include <NXOpen/Features_TransformBuilder.hxx>

// Std C++ Includes
#include <cstdio>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <set>
#include <sstream>
#include <utility>

using namespace NXOpen;
using std::string;
using std::exception;
using std::stringstream;
using std::endl;
using std::cout;
using std::cerr;

#define DEBUG_ENVELOPE_SUBTRACTION
#define USE_SIMPLE_ENVELOPE_SUBTRACTION

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

void computeZRangeAlongDirection(const casting::Mesh& mesh,
                                 const casting::Vector3& direction,
                                 double& outMinZ,
                                 double& outMaxZ) {
    outMinZ =  std::numeric_limits<double>::max();
    outMaxZ = -std::numeric_limits<double>::max();
    for (const auto& v : mesh.vertices) {
        double proj = v.x * direction.x + v.y * direction.y + v.z * direction.z;
        outMinZ = std::min(outMinZ, proj);
        outMaxZ = std::max(outMaxZ, proj);
    }
    if (outMinZ > outMaxZ) {
        outMinZ = 0.0;
        outMaxZ = 0.0;
    }
}

NXOpen::Body* cloneBody(NXOpen::Body* original) {
    if (!original) {
        return nullptr;
    }
    NXOpen::Session* session = NXOpen::Session::GetSession();
    NXOpen::Part* part = session
        ? dynamic_cast<NXOpen::Part*>(session->Parts()->Work())
        : nullptr;
    if (!part) {
        return nullptr;
    }

    // Snapshot existing body tags so we can identify the newly cloned body.
    std::set<tag_t> preTags;
    for (NXOpen::Body* b : *part->Bodies()) {
        preTags.insert(b->Tag());
    }

    NXOpen::Features::TransformBuilder* builder = nullptr;
    try {
        builder = part->Features()->CreateTransformBuilder(nullptr);
        builder->ObjectToTransform()->Add(original);
        builder->SetMovementMethod(
            NXOpen::Features::TransformBuilder::MovementMethodDelta);
        builder->SetOperationOption(
            NXOpen::Features::TransformBuilder::OperationOptionCopy);
        builder->CommitFeature();
        builder->Destroy();
        builder = nullptr;
    } catch (...) {
        if (builder) { builder->Destroy(); }
        return nullptr;
    }

    // Return the first body that appeared after the clone operation.
    for (NXOpen::Body* b : *part->Bodies()) {
        if (preTags.find(b->Tag()) == preTags.end()) {
            return b;
        }
    }
    return nullptr;
}

NXOpen::Body* createExactContourExtrusion(const std::vector<casting::Vector3>& boundary,
                                          const casting::Vector3& pullDir,
                                          double heightMin,
                                          double heightMax) {
    if (boundary.size() < 3) {
        return nullptr;
    }
    double len = heightMax - heightMin;
    if (len <= 0.0) {
        return nullptr;
    }

    // Determine whether the loop is already closed (first == last point).
    std::size_t n = boundary.size();
    bool alreadyClosed = (boundary.front().x == boundary.back().x &&
                          boundary.front().y == boundary.back().y &&
                          boundary.front().z == boundary.back().z);
    std::size_t nLines = alreadyClosed ? n - 1 : n;
    if (nLines < 3) {
        return nullptr;
    }

    // Create one line segment per boundary edge.
    std::vector<tag_t> edgeTags(nLines, NULL_TAG);
    for (std::size_t i = 0; i < nLines; ++i) {
        const auto& p0 = boundary[i];
        const auto& p1 = boundary[(i + 1) % n];
        UF_CURVE_line_t seg{};
        seg.start_point[0] = p0.x; seg.start_point[1] = p0.y; seg.start_point[2] = p0.z;
        seg.end_point[0]   = p1.x; seg.end_point[1]   = p1.y; seg.end_point[2]   = p1.z;
        if (UF_CURVE_create_line(&seg, &edgeTags[i]) != 0 || edgeTags[i] == NULL_TAG) {
            for (tag_t t : edgeTags) {
                if (t != NULL_TAG) UF_OBJ_delete_object(t);
            }
            return nullptr;
        }
    }

    uf_list_p_t sectionList = nullptr;
    if (UF_MODL_create_list(&sectionList) != 0 || !sectionList) {
        for (tag_t t : edgeTags) UF_OBJ_delete_object(t);
        return nullptr;
    }
    for (tag_t t : edgeTags) {
        UF_MODL_put_list_item(sectionList, t);
    }

    // Normalize pull direction.
    double mag = std::sqrt(pullDir.x * pullDir.x + pullDir.y * pullDir.y + pullDir.z * pullDir.z);
    double dir[3] = {0.0, 0.0, 1.0};
    if (mag > 1e-9) {
        dir[0] = pullDir.x / mag;
        dir[1] = pullDir.y / mag;
        dir[2] = pullDir.z / mag;
    }

    char taperAngle[] = "0";
    char startLimit[] = "0";
    char endLimit[64]{};
    int precision = std::numeric_limits<double>::max_digits10;
    std::snprintf(endLimit, sizeof(endLimit), "%.*g", precision, len);
    char* limits[2] = {startLimit, endLimit};
    double point[3] = {boundary[0].x, boundary[0].y, boundary[0].z};

    uf_list_p_t createdObjects = nullptr;
    int rc = UF_MODL_create_extruded(sectionList, taperAngle, limits, point, dir,
                                     UF_NULLSIGN, &createdObjects);
    UF_MODL_delete_list(&sectionList);
    for (tag_t t : edgeTags) UF_OBJ_delete_object(t);

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
    return dynamic_cast<NXOpen::Body*>(NXOpen::NXObjectManager::Get(bodyTag));
}

std::vector<NXOpen::Body*> extractInternalVolumes(NXOpen::Body* body) {
    (void)body;
    return {};
}

std::vector<NXOpen::Body*> extractInternalCoresManually(NXOpen::Part* part, NXOpen::Body* intermediateBody) {
    std::vector<NXOpen::Body*> internalCores;
    if (!part || !intermediateBody) {
        return internalCores;
    }

    // Prompt user to manually select all inner faces of a closed cavity.
    NXOpen::UI* ui = NXOpen::UI::GetUI();
    if (ui && ui->NXMessageBox()) {
        ui->NXMessageBox()->Show(
            "Internal Core Extraction",
            NXOpen::NXMessageBox::DialogTypeInformation,
            "Scaffold only: integrate face-picking dialog and sewing-to-solid workflow.");
    }

    // TODO: Use SelectionManager / UF_UI_select_with_class_dialog to pick faces into faceTags.
    // TODO: Sew/extract faceTags into sheets, then close/thicken into solid bodies.
    // TODO: Append generated internal core bodies to internalCores.
    (void)part;
    (void)intermediateBody;
    return internalCores;
}

std::vector<NXOpen::Body*> splitIntoConnectedBodies(NXOpen::Body* body) {
    (void)body;
    return {};
}

std::pair<NXOpen::Body*, NXOpen::Body*> splitBodyByPartingSurface(NXOpen::Body* body,
                                                                  const casting::Vector3& planeOrigin,
                                                                  const casting::Vector3& planeNormal) {
    if (!body) {
        return {nullptr, nullptr};
    }

    // Retrieve the active work part for boolean operations.
    NXOpen::Session* session = NXOpen::Session::GetSession();
    NXOpen::Part* part = session
        ? dynamic_cast<NXOpen::Part*>(session->Parts()->Work())
        : nullptr;
    if (!part) {
        return {body, nullptr};
    }

    // Normalize the plane normal.
    double mag = std::sqrt(planeNormal.x * planeNormal.x +
                           planeNormal.y * planeNormal.y +
                           planeNormal.z * planeNormal.z);
    double nx = 0.0, ny = 0.0, nz = 1.0;
    if (mag > 1e-9) {
        nx = planeNormal.x / mag;
        ny = planeNormal.y / mag;
        nz = planeNormal.z / mag;
    }

    // Compute half-extent large enough to cover the body plus margin.
    casting::Bounds bodyBounds = askBodyBounds(body->Tag());
    double diag = std::sqrt(
        std::pow(bodyBounds.max.x - bodyBounds.min.x, 2.0) +
        std::pow(bodyBounds.max.y - bodyBounds.min.y, 2.0) +
        std::pow(bodyBounds.max.z - bodyBounds.min.z, 2.0));
    double halfExtent = std::max(diag * 2.0, 1000.0);

    // Upper half-space box: centre displaced from planeOrigin along +normal.
    casting::Bounds upperBounds;
    upperBounds.min = {planeOrigin.x + nx * halfExtent - halfExtent,
                       planeOrigin.y + ny * halfExtent - halfExtent,
                       planeOrigin.z + nz * halfExtent - halfExtent};
    upperBounds.max = {planeOrigin.x + nx * halfExtent + halfExtent,
                       planeOrigin.y + ny * halfExtent + halfExtent,
                       planeOrigin.z + nz * halfExtent + halfExtent};

    // Lower half-space box: centre displaced from planeOrigin along -normal.
    casting::Bounds lowerBounds;
    lowerBounds.min = {planeOrigin.x - nx * halfExtent - halfExtent,
                       planeOrigin.y - ny * halfExtent - halfExtent,
                       planeOrigin.z - nz * halfExtent - halfExtent};
    lowerBounds.max = {planeOrigin.x - nx * halfExtent + halfExtent,
                       planeOrigin.y - ny * halfExtent + halfExtent,
                       planeOrigin.z - nz * halfExtent + halfExtent};

    NXOpen::Body* upperBox = createExtrudedRectangularSolid(upperBounds, 0);
    NXOpen::Body* lowerBox = createExtrudedRectangularSolid(lowerBounds, 0);

    // Clone the original body twice so we can intersect each clone with its
    // respective half-space box (do NOT use UF_MODL_split_body).
    NXOpen::Body* upperClone = cloneBody(body);
    NXOpen::Body* lowerClone = cloneBody(body);

    NXOpen::Body* upperResult = nullptr;
    NXOpen::Body* lowerResult = nullptr;

    // Intersect upper clone with upper-side box (tool consumed, retainTool=false).
    if (upperClone && upperBox) {
        if (applyBooleanFeature(part, upperClone, upperBox,
                                NXOpen::Features::Feature::BooleanTypeIntersect, false)) {
            upperResult = upperClone;
        } else {
            UF_OBJ_delete_object(upperClone->Tag());
            UF_OBJ_delete_object(upperBox->Tag());
        }
    } else {
        if (upperClone) { UF_OBJ_delete_object(upperClone->Tag()); }
        if (upperBox)   { UF_OBJ_delete_object(upperBox->Tag());   }
    }

    // Intersect lower clone with lower-side box.
    if (lowerClone && lowerBox) {
        if (applyBooleanFeature(part, lowerClone, lowerBox,
                                NXOpen::Features::Feature::BooleanTypeIntersect, false)) {
            lowerResult = lowerClone;
        } else {
            UF_OBJ_delete_object(lowerClone->Tag());
            UF_OBJ_delete_object(lowerBox->Tag());
        }
    } else {
        if (lowerClone) { UF_OBJ_delete_object(lowerClone->Tag()); }
        if (lowerBox)   { UF_OBJ_delete_object(lowerBox->Tag());   }
    }

    return {upperResult, lowerResult};
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

// Convenience wrapper around createExtrudedRectangularSolid for callers that
// only need to supply bounds and optionally a display color.
NXOpen::Body* createBox(const casting::Bounds& bounds, int color = 0) {
    return createExtrudedRectangularSolid(bounds, color);
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
    void showMoldAssembly(const casting::MoldAssembly& assembly,
                          const casting::ContourFace& maxContour,
                          const casting::Mesh& mesh);
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

void MyClass::showMoldAssembly(const casting::MoldAssembly& assembly,
                               const casting::ContourFace& maxContour,
                               const casting::Mesh& mesh) {
    if (!workPart) {
        return;
    }
    NXOpen::Part* part = resolveWorkPart(workPart);
    if (!part) {
        return;
    }
    std::vector<tag_t> partSolidTags = collectSolidBodyTags(workPart);
    std::vector<NXOpen::Body*> partToolBodies;
    partToolBodies.reserve(partSolidTags.size());
    for (tag_t bodyTag : partSolidTags) {
        NXOpen::Body* body = dynamic_cast<NXOpen::Body*>(NXOpen::NXObjectManager::Get(bodyTag));
        if (body) {
            partToolBodies.push_back(body);
        }
    }
    NXOpen::Body* partBody = partToolBodies.empty() ? nullptr : partToolBodies[0];

#ifdef USE_SIMPLE_ENVELOPE_SUBTRACTION
    // -----------------------------------------------------------------------
    // Simplified sand-core extraction: envelope-subtraction workflow
    // -----------------------------------------------------------------------

    // 1) Compute precise part bounding box from mesh vertices (no expansion).
    casting::Bounds partBounds{};
    if (!mesh.vertices.empty()) {
        partBounds.min = partBounds.max = mesh.vertices[0];
        for (const auto& v : mesh.vertices) {
            partBounds.min.x = std::min(partBounds.min.x, v.x);
            partBounds.min.y = std::min(partBounds.min.y, v.y);
            partBounds.min.z = std::min(partBounds.min.z, v.z);
            partBounds.max.x = std::max(partBounds.max.x, v.x);
            partBounds.max.y = std::max(partBounds.max.y, v.y);
            partBounds.max.z = std::max(partBounds.max.z, v.z);
        }
    }

    // 2) Generate precise envelope B from partBounds (no padding).
    NXOpen::Body* bodyB = createBox(partBounds);

    // 3) C = B - partBody (keep tool so partBody is preserved for later use).
    NXOpen::Body* bodyC = nullptr;
    if (bodyB && partBody) {
        if (applyBooleanFeature(part, bodyB, partBody,
                                NXOpen::Features::Feature::BooleanTypeSubtract, true)) {
            bodyC = bodyB;
        }
    }

    // 4) Extract internal cores (closed cavities) from C.
    std::vector<NXOpen::Body*> internalCores;
    if (bodyC) {
        internalCores = extractInternalVolumes(bodyC);
    }

    // 5) Subtract internal cores from C, then split remaining C into external cores.
    std::vector<NXOpen::Body*> externalCores;
    if (bodyC) {
        for (NXOpen::Body* coreBody : internalCores) {
            if (coreBody) {
                applyBooleanFeature(part, bodyC, coreBody,
                                    NXOpen::Features::Feature::BooleanTypeSubtract, false);
            }
        }
        externalCores = splitIntoConnectedBodies(bodyC);
    }

    // 6) Generate mold blank A: merge bounds of blocks[0] and blocks[1].
    NXOpen::Body* bodyA = nullptr;
    if (assembly.blocks.size() >= 2) {
        casting::Bounds mergedBounds = assembly.blocks[0].bounds;
        const casting::Bounds& blk1 = assembly.blocks[1].bounds;
        mergedBounds.min.x = std::min(mergedBounds.min.x, blk1.min.x);
        mergedBounds.min.y = std::min(mergedBounds.min.y, blk1.min.y);
        mergedBounds.min.z = std::min(mergedBounds.min.z, blk1.min.z);
        mergedBounds.max.x = std::max(mergedBounds.max.x, blk1.max.x);
        mergedBounds.max.y = std::max(mergedBounds.max.y, blk1.max.y);
        mergedBounds.max.z = std::max(mergedBounds.max.z, blk1.max.z);
        bodyA = createBox(mergedBounds);
    }
    if (!bodyA) {
        return;
    }

    // 7) A minus all internal and external cores (keep tool bodies for later use).
    for (NXOpen::Body* coreBody : internalCores) {
        if (coreBody) {
            applyBooleanFeature(part, bodyA, coreBody,
                                NXOpen::Features::Feature::BooleanTypeSubtract, true);
        }
    }
    for (NXOpen::Body* coreBody : externalCores) {
        if (coreBody) {
            applyBooleanFeature(part, bodyA, coreBody,
                                NXOpen::Features::Feature::BooleanTypeSubtract, true);
        }
    }

    // 8) A minus partBody (do not retain tool).
    if (partBody) {
        applyBooleanFeature(part, bodyA, partBody,
                            NXOpen::Features::Feature::BooleanTypeSubtract, false);
    }

    // 9) Parting surface info: origin = maxContour.centroid, normal = blocks[0].pullDirection.
    casting::Vector3 partingOrigin = maxContour.centroid;
    casting::Vector3 partingNormal = {0.0, 0.0, 1.0};
    if (!assembly.blocks.empty()) {
        partingNormal = assembly.blocks[0].pullDirection;
        if (std::abs(partingNormal.x) + std::abs(partingNormal.y) + std::abs(partingNormal.z) < 1e-9) {
            partingNormal = {0.0, 0.0, 1.0};
        }
    }

    // 10) Split A into upper and lower mold bodies.
    auto moldSplit = splitBodyByPartingSurface(bodyA, partingOrigin, partingNormal);
    NXOpen::Body* upperBody = moldSplit.first;
    NXOpen::Body* lowerBody = moldSplit.second;
    if (upperBody) {
        UF_OBJ_set_color(upperBody->Tag(), casting::kUpperMoldColor);
    }
    if (lowerBody) {
        UF_OBJ_set_color(lowerBody->Tag(), casting::kLowerMoldColor);
    }

    // 11) Split each core by parting surface; set color and layer on both halves.
    auto splitAndColorCore = [&](NXOpen::Body* coreBody) {
        if (!coreBody) {
            return;
        }
        auto coreSplit = splitBodyByPartingSurface(coreBody, partingOrigin, partingNormal);
        if (coreSplit.first) {
            UF_OBJ_set_color(coreSplit.first->Tag(), casting::kCoreColor);
            UF_OBJ_set_layer(coreSplit.first->Tag(), casting::kSandCoreBaseLayer);
        }
        if (coreSplit.second) {
            UF_OBJ_set_color(coreSplit.second->Tag(), casting::kCoreColor);
            UF_OBJ_set_layer(coreSplit.second->Tag(), casting::kSandCoreBaseLayer);
        }
    };
    for (NXOpen::Body* coreBody : internalCores) {
        splitAndColorCore(coreBody);
    }
    for (NXOpen::Body* coreBody : externalCores) {
        splitAndColorCore(coreBody);
    }

    // 12) Delete intermediate bodies B and C (optional cleanup).
    // Note: bodyC == bodyB when the boolean subtraction succeeded (B is modified
    // in-place and then aliased as C).  Deleting via bodyC is therefore sufficient
    // to release the underlying NX body in both cases.
    if (bodyC) {
        UF_OBJ_delete_object(bodyC->Tag());
    } else if (bodyB) {
        // Boolean failed; bodyB was never aliased, so delete it separately.
        UF_OBJ_delete_object(bodyB->Tag());
    }

#else
    // -----------------------------------------------------------------------
    // Legacy path (kept as fallback when USE_SIMPLE_ENVELOPE_SUBTRACTION is
    // not defined).
    // -----------------------------------------------------------------------

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

#ifdef DEBUG_ENVELOPE_SUBTRACTION
    // 1) Pull direction and parting-plane origin from maxContour.
    casting::Vector3 pullDir{0.0, 0.0, 1.0};
    casting::Vector3 partingOrigin = maxContour.centroid;
    if (!assembly.blocks.empty()) {
        pullDir = assembly.blocks[0].pullDirection;
    }
    if (std::abs(pullDir.x) + std::abs(pullDir.y) + std::abs(pullDir.z) < 1e-9) {
        pullDir = {0.0, 0.0, 1.0};
    }

    // 2) Compute part Z range along pull direction from mesh vertices.
    double minZ = 0.0;
    double maxZ = 0.0;
    computeZRangeAlongDirection(mesh, pullDir, minZ, maxZ);

    // 3) Compute mesh vertex bounding box and build envelope A (expanded 20 mm in XY).
    NXOpen::Body* envelopeBodyA = nullptr;
    if (!mesh.vertices.empty()) {
        casting::Bounds meshBounds{};
        meshBounds.min = mesh.vertices[0];
        meshBounds.max = mesh.vertices[0];
        for (const auto& v : mesh.vertices) {
            meshBounds.min.x = std::min(meshBounds.min.x, v.x);
            meshBounds.min.y = std::min(meshBounds.min.y, v.y);
            meshBounds.min.z = std::min(meshBounds.min.z, v.z);
            meshBounds.max.x = std::max(meshBounds.max.x, v.x);
            meshBounds.max.y = std::max(meshBounds.max.y, v.y);
            meshBounds.max.z = std::max(meshBounds.max.z, v.z);
        }
        constexpr double kEnvelopePadding = 20.0;
        casting::Bounds envelopeBounds = meshBounds;
        envelopeBounds.min.x -= kEnvelopePadding;
        envelopeBounds.min.y -= kEnvelopePadding;
        envelopeBounds.max.x += kEnvelopePadding;
        envelopeBounds.max.y += kEnvelopePadding;
        envelopeBounds.min.z = minZ - 0.5;
        envelopeBounds.max.z = maxZ + 0.5;
        envelopeBodyA = createExtrudedRectangularSolid(envelopeBounds, 0, casting::kSandCoreBaseLayer);
    }

    // 4) Create two identical precise contour extrusions B1 and B2 from maxContour.boundary.
    NXOpen::Body* contourB1 = createExactContourExtrusion(maxContour.boundary, pullDir, minZ, maxZ);
    NXOpen::Body* contourB2 = createExactContourExtrusion(maxContour.boundary, pullDir, minZ, maxZ);

    // 5) A - B1 (not retain tool) → A becomes the excess part (returned to mold).
    NXOpen::Body* excessBody = nullptr;
    if (envelopeBodyA && contourB1) {
        if (applyBooleanFeature(part, envelopeBodyA, contourB1,
                                NXOpen::Features::Feature::BooleanTypeSubtract, false)) {
            excessBody = envelopeBodyA;
        }
    }

    // 6) B2 - partBody (retain tool) → B2 becomes the intermediate body for core extraction.
    NXOpen::Body* intermediateBody = nullptr;
    if (contourB2 && partBody) {
        if (applyBooleanFeature(part, contourB2, partBody,
                                NXOpen::Features::Feature::BooleanTypeSubtract, true)) {
            intermediateBody = contourB2;
        }
    }

    std::vector<NXOpen::Body*> allCores;
    if (intermediateBody) {
        // 7) Extract internal cores from intermediate body.
        std::vector<NXOpen::Body*> internalCores = extractInternalVolumes(intermediateBody);

        // 8) Subtract internal cores from intermediate body to isolate external volume.
        for (NXOpen::Body* coreBody : internalCores) {
            if (!coreBody) {
                continue;
            }
            applyBooleanFeature(part, intermediateBody, coreBody,
                                NXOpen::Features::Feature::BooleanTypeSubtract, false);
        }

        // Separate remaining external volume into connected bodies (external cores).
        std::vector<NXOpen::Body*> externalCores = splitIntoConnectedBodies(intermediateBody);
        allCores.insert(allCores.end(), internalCores.begin(), internalCores.end());
        allCores.insert(allCores.end(), externalCores.begin(), externalCores.end());
    }

    // 9) Apply color and layer to all cores.
    for (NXOpen::Body* coreBody : allCores) {
        if (!coreBody) {
            continue;
        }
        UF_OBJ_set_color(coreBody->Tag(), casting::kCoreColor);
        UF_OBJ_set_layer(coreBody->Tag(), casting::kSandCoreBaseLayer);
    }

    // 10) Split excess body by parting surface and unite each half back into upper/lower mold.
    if (excessBody) {
        auto excessSplit = splitBodyByPartingSurface(excessBody, partingOrigin, pullDir);
        if (excessSplit.first) {
            applyBooleanFeature(part, upperBody, excessSplit.first,
                                NXOpen::Features::Feature::BooleanTypeUnite, false);
        }
        if (excessSplit.second) {
            applyBooleanFeature(part, lowerBody, excessSplit.second,
                                NXOpen::Features::Feature::BooleanTypeUnite, false);
        }
    }

    // 11) Split each core by parting surface, subtract from corresponding mold (retain core tools),
    //     then delete the original unsplit core body.
    for (NXOpen::Body* coreBody : allCores) {
        if (!coreBody) {
            continue;
        }
        auto splitCore = splitBodyByPartingSurface(coreBody, partingOrigin, pullDir);
        if (splitCore.first) {
            applyBooleanFeature(part, upperBody, splitCore.first,
                                NXOpen::Features::Feature::BooleanTypeSubtract, true);
        }
        if (splitCore.second) {
            applyBooleanFeature(part, lowerBody, splitCore.second,
                                NXOpen::Features::Feature::BooleanTypeSubtract, true);
        }
        UF_OBJ_delete_object(coreBody->Tag());
    }

    // 12) Final: subtract part body from upper and lower mold blanks (do not retain tool).
    if (partBody) {
        applyBooleanFeature(part, upperBody, partBody,
                            NXOpen::Features::Feature::BooleanTypeSubtract, false);
        applyBooleanFeature(part, lowerBody, partBody,
                            NXOpen::Features::Feature::BooleanTypeSubtract, false);
    }
#else
    // Baseline logic: subtract part body from mold blanks directly (do not retain tool).
    if (partBody) {
        applyBooleanFeature(part, upperBody, partBody,
                            NXOpen::Features::Feature::BooleanTypeSubtract, false);
        applyBooleanFeature(part, lowerBody, partBody,
                            NXOpen::Features::Feature::BooleanTypeSubtract, false);
    }
#endif  // DEBUG_ENVELOPE_SUBTRACTION

#endif  // USE_SIMPLE_ENVELOPE_SUBTRACTION
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
    showMoldAssembly(result.moldAssembly, result.maxContour, mesh);
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
