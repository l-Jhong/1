#include "casting/AutoPartingPipeline.h"

// Mandatory UF Includes
#include <uf.h>
#include <uf_curve.h>
#include <uf_object_types.h>
#include <uf_modl.h>
#include <uf_obj.h>
#include <uf_ui.h>

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
#include <cmath>
#include <iostream>
#include <limits>
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
        mesh = casting::buildBoxMesh(10.0);
    }
    return mesh;
}

NXOpen::Body* applyBooleanFeature(NXOpen::Part* part,
                                   NXOpen::Body* targetBody,
                                   NXOpen::Body* toolBody,
                                   NXOpen::Features::Feature::BooleanType operation,
                                   bool retainTool = true) {
    if (!part || !targetBody || !toolBody) {
        return nullptr;
    }

    NXOpen::Features::BooleanBuilder* booleanBuilder = nullptr;
    try {
        booleanBuilder = part->Features()->CreateBooleanBuilder(nullptr);
        booleanBuilder->SetOperation(operation);
        booleanBuilder->SetTarget(targetBody);
        booleanBuilder->SetTool(toolBody);
        booleanBuilder->SetRetainTarget(false);
        booleanBuilder->SetRetainTool(retainTool);
        NXOpen::NXObject* committed = booleanBuilder->CommitFeature();
        booleanBuilder->Destroy();
        booleanBuilder = nullptr;
        if (!committed) {
            return nullptr;
        }
        tag_t featTag = committed->Tag();
        tag_t bodyTag = NULL_TAG;
        if (UF_MODL_ask_feat_body(featTag, &bodyTag) != 0 || bodyTag == NULL_TAG) {
            return nullptr;
        }
        return dynamic_cast<NXOpen::Body*>(NXOpen::NXObjectManager::Get(bodyTag));
    } catch (...) {
        if (booleanBuilder) {
            booleanBuilder->Destroy();
        }
        return nullptr;
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
    // TODO: Build a strict envelope from maxContour projection with side walls parallel to pullDir.
    (void)pullDir;
    return createExtrudedRectangularSolid(casting::expandBounds(bounds, padding),
                                          casting::kCoreColor, casting::kSandCoreBaseLayer);
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

    // Get the bounding box of the original body.
    double box[6]{};
    if (UF_MODL_ask_bounding_box(original->Tag(), box) != 0) {
        return nullptr;
    }

    // Build a box that fully encloses the original (with padding to avoid
    // tangency issues at the boundary during the boolean operation).
    const double kPad = 1.0;
    casting::Bounds bounds;
    bounds.min = {box[0] - kPad, box[1] - kPad, box[2] - kPad};
    bounds.max = {box[3] + kPad, box[4] + kPad, box[5] + kPad};
    NXOpen::Body* boxBody = createExtrudedRectangularSolid(bounds, 0);
    if (!boxBody) {
        return nullptr;
    }

    // Boolean Intersect: boxBody (target) ∩ original (tool, retained).
    // Because the box fully contains the original, the result equals the
    // original's shape, giving us an independent copy while the original
    // body is preserved intact (retainTool = true).
    NXOpen::Session* session = NXOpen::Session::GetSession();
    NXOpen::Part* part = session
        ? dynamic_cast<NXOpen::Part*>(session->Parts()->Work())
        : nullptr;
    if (!part) {
        return nullptr;
    }

    NXOpen::Body* cloned = applyBooleanFeature(part, boxBody, original,
                                               NXOpen::Features::Feature::BooleanTypeIntersect, true);
    return cloned;
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

casting::Mesh buildDemoMesh(double size) {
    return casting::buildBoxMesh(size);
}

// Returns true when the mesh contains at least one face whose outward normal makes an
// angle greater than (90 + angleThresholdDeg) degrees with the pull direction.
// Such a face is "back-facing" beyond the threshold and indicates an external undercut.
bool hasExternalUndercut(const casting::Mesh& mesh,
                         const casting::Vector3& pullDir,
                         double angleThresholdDeg = 5.0) {
    if (mesh.triangles.empty()) {
        return false;
    }

    constexpr double kPi = 3.14159265358979323846;

    // Normalize pull direction.
    double mag = casting::length(pullDir);
    casting::Vector3 pullNorm = (mag > 1e-9) ? casting::normalized(pullDir)
                                             : casting::Vector3{0.0, 0.0, 1.0};

    for (const auto& tri : mesh.triangles) {
        casting::Vector3 n = casting::triangleNormal(mesh, tri);
        double nLen = casting::length(n);
        if (nLen < 1e-12) {
            continue;
        }
        n = n / nLen;
        double dotVal = casting::dot(n, pullNorm);
        // Clamp to [-1, 1] to avoid NaN from acos.
        dotVal = std::max(-1.0, std::min(1.0, dotVal));
        double angleDeg = std::acos(dotVal) * 180.0 / kPi;
        if (angleDeg > 90.0 + angleThresholdDeg) {
            return true;
        }
    }
    return false;
}

std::vector<NXOpen::Body*> extractInternalVolumes(NXOpen::Body* body) {
    // Stub: internal void detection is not yet implemented.
    (void)body;
    return {};
}

// Initialization callback for UF_UI_select_with_class_dialog: restrict selection to solid faces.
static int selectFaceInit(UF_UI_selection_p_t sel, void* /*userData*/) {
    UF_UI_mask_t mask = { UF_solid_type, UF_solid_face_subtype, 0 };
    return UF_UI_set_sel_mask(sel, UF_UI_SEL_MASK_CLEAR_AND_ENABLE_SPECIFIC, 1, &mask);
}

// Prompts the user to select solid faces interactively, then sews them into a solid body.
// Returns the resulting solid body, or nullptr if selection was cancelled or sewing failed.
NXOpen::Body* manuallySelectAndSewFaces(const char* prompt) {
    uf_list_p_t faceList = nullptr;
    if (UF_MODL_create_list(&faceList) != 0 || !faceList) {
        return nullptr;
    }

    int numFaces = 0;
    while (true) {
        int response = 0;
        tag_t objTag = NULL_TAG;
        int rc = UF_UI_select_with_class_dialog(
            const_cast<char*>(prompt),
            const_cast<char*>("Select Face"),
            selectFaceInit,
            nullptr,
            &response,
            &objTag);
        if (rc != 0 || response == UF_UI_CANCEL || objTag == NULL_TAG) {
            break;
        }
        UF_MODL_put_list_item(faceList, objTag);
        ++numFaces;
    }

    if (numFaces == 0) {
        UF_MODL_delete_list(&faceList);
        return nullptr;
    }

    constexpr double kSewTolerance = 0.01;

    // Extract face tags from the list into an array for the array-based sew API.
    int listCount = 0;
    UF_MODL_ask_list_count(faceList, &listCount);
    std::vector<tag_t> faceArr(listCount);
    for (int i = 0; i < listCount; ++i) {
        UF_MODL_ask_list_item(faceList, i, &faceArr[i]);
    }
    UF_MODL_delete_list(&faceList);

    tag_t sewnTag = NULL_TAG;
    int numSewErrors = 0;
    tag_t* badEdges = nullptr;
    int rc = UF_MODL_create_sew(listCount, faceArr.data(), kSewTolerance,
                                &sewnTag, &numSewErrors, &badEdges);
    if (badEdges) { UF_free(badEdges); }
    if (rc != 0 || sewnTag == NULL_TAG) {
        return nullptr;
    }
    return dynamic_cast<NXOpen::Body*>(NXOpen::NXObjectManager::Get(sewnTag));
}

std::vector<NXOpen::Body*> splitIntoConnectedBodies(NXOpen::Body* body) {
    // TODO: not yet implemented — split multi-lump body into individual connected bodies
    (void)body;
    return {};
}

std::pair<NXOpen::Body*, NXOpen::Body*> splitBodyByPartingSurface(NXOpen::Body* body,
                                                                  const casting::Vector3& planeOrigin,
                                                                  const casting::Vector3& planeNormal) {
    if (!body) {
        return {nullptr, nullptr};
    }

    // Normalize the plane normal.
    double mag = std::sqrt(planeNormal.x * planeNormal.x +
                           planeNormal.y * planeNormal.y +
                           planeNormal.z * planeNormal.z);
    double normal[3] = {0.0, 0.0, 1.0};
    if (mag > 1e-9) {
        normal[0] = planeNormal.x / mag;
        normal[1] = planeNormal.y / mag;
        normal[2] = planeNormal.z / mag;
    }
    double origin[3] = {planeOrigin.x, planeOrigin.y, planeOrigin.z};

    // Clone the body so we can trim the original for the upper half
    // and the clone for the lower half.
    NXOpen::Body* lowerBody = cloneBody(body);

    tag_t planTag = NULL_TAG;
    if (UF_MODL_create_plane(origin, normal, &planTag) != 0 || planTag == NULL_TAG) {
        // Fallback: cannot create plane, route whole body to upper side.
        return {body, lowerBody};
    }

    // Trim original body: keep the positive-normal side (upper mold).
    tag_t trimmedUpper = NULL_TAG;
    UF_MODL_trim_body(body->Tag(), planTag, 1, &trimmedUpper);

    // Trim cloned body: keep the negative-normal side (lower mold).
    tag_t trimmedLower = NULL_TAG;
    if (lowerBody) {
        UF_MODL_trim_body(lowerBody->Tag(), planTag, 0, &trimmedLower);
    }

    UF_OBJ_delete_object(planTag);

    NXOpen::Body* upperResult = (trimmedUpper != NULL_TAG)
        ? dynamic_cast<NXOpen::Body*>(NXOpen::NXObjectManager::Get(trimmedUpper)) : body;
    NXOpen::Body* lowerResult = (trimmedLower != NULL_TAG && lowerBody)
        ? dynamic_cast<NXOpen::Body*>(NXOpen::NXObjectManager::Get(trimmedLower)) : lowerBody;

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
    // 1) Pull direction and parting-plane origin.
    casting::Vector3 pullDir{0.0, 0.0, 1.0};
    casting::Vector3 partingOrigin = maxContour.centroid;
    if (!assembly.blocks.empty()) {
        pullDir = assembly.blocks[0].pullDirection;
    }
    if (std::abs(pullDir.x) + std::abs(pullDir.y) + std::abs(pullDir.z) < 1e-9) {
        pullDir = {0.0, 0.0, 1.0};
    }

    // 2) Compute Z range along pull direction.
    double minZ = 0.0;
    double maxZ = 0.0;
    computeZRangeAlongDirection(mesh, pullDir, minZ, maxZ);

    // 3) Build initial envelope A from mesh bounding box (XY +20 mm, Z from projection range).
    NXOpen::Body* envelopeBodyA = nullptr;
    if (!mesh.vertices.empty()) {
        constexpr double kEnvelopePadding = 20.0;
        casting::Bounds meshBounds = casting::computeBounds(mesh);
        casting::Bounds envelopeBounds = casting::expandBounds(meshBounds, kEnvelopePadding);
        envelopeBounds.min.z = minZ - 0.5;
        envelopeBounds.max.z = maxZ + 0.5;
        envelopeBodyA = createExtrudedRectangularSolid(envelopeBounds, 0, casting::kSandCoreBaseLayer);
    }

    // 4) Create two identical precise contour extrusions B1 and B2.
    NXOpen::Body* contourB1 = createExactContourExtrusion(maxContour.boundary, pullDir, minZ, maxZ);
    NXOpen::Body* contourB2 = createExactContourExtrusion(maxContour.boundary, pullDir, minZ, maxZ);

    // 5) A - B1 (not retain tool) → excessBody; B1 is consumed.
    NXOpen::Body* excessBody = nullptr;
    if (envelopeBodyA && contourB1) {
        excessBody = applyBooleanFeature(part, envelopeBodyA, contourB1,
                                         NXOpen::Features::Feature::BooleanTypeSubtract, false);
    }

    // 6) B2 - partBody (retain tool) → intermediateBody for core extraction.
    NXOpen::Body* intermediateBody = nullptr;
    if (contourB2 && partBody) {
        intermediateBody = applyBooleanFeature(part, contourB2, partBody,
                                               NXOpen::Features::Feature::BooleanTypeSubtract, true);
    }

    // 7) Undercut detection.
    bool undercutDetected = hasExternalUndercut(mesh, pullDir);

    // 8) Extract internal cores (stub, returns empty list).
    std::vector<NXOpen::Body*> internalCores = extractInternalVolumes(intermediateBody);

    // 9) Conditionally extract external cores when undercut is present.
    std::vector<NXOpen::Body*> externalCores;
    if (undercutDetected && intermediateBody) {
        // Subtract internal cores from intermediate body before splitting into external cores.
        for (NXOpen::Body* ic : internalCores) {
            if (ic && intermediateBody) {
                intermediateBody = applyBooleanFeature(part, intermediateBody, ic,
                                                       NXOpen::Features::Feature::BooleanTypeSubtract, false);
            }
        }
        externalCores = splitIntoConnectedBodies(intermediateBody);
    }

    // 10) Merge all cores; set color and layer.
    std::vector<NXOpen::Body*> allCores;
    allCores.insert(allCores.end(), internalCores.begin(), internalCores.end());
    allCores.insert(allCores.end(), externalCores.begin(), externalCores.end());
    for (NXOpen::Body* coreBody : allCores) {
        if (!coreBody) {
            continue;
        }
        UF_OBJ_set_color(coreBody->Tag(), casting::kCoreColor);
        UF_OBJ_set_layer(coreBody->Tag(), casting::kSandCoreBaseLayer);
    }

    // 11) Split excess body and unite each half with the corresponding mold blank.
    if (excessBody) {
        auto excessSplit = splitBodyByPartingSurface(excessBody, partingOrigin, pullDir);
        if (excessSplit.first && upperBody) {
            upperBody = applyBooleanFeature(part, upperBody, excessSplit.first,
                                            NXOpen::Features::Feature::BooleanTypeUnite, false);
        }
        if (excessSplit.second && lowerBody) {
            lowerBody = applyBooleanFeature(part, lowerBody, excessSplit.second,
                                            NXOpen::Features::Feature::BooleanTypeUnite, false);
        }
    }

    // 12) Split each core by parting surface; subtract each half from the corresponding mold
    //     (retain tool), then delete the original core body.
    for (NXOpen::Body* coreBody : allCores) {
        if (!coreBody) {
            continue;
        }
        auto splitCore = splitBodyByPartingSurface(coreBody, partingOrigin, pullDir);
        if (splitCore.first && upperBody) {
            upperBody = applyBooleanFeature(part, upperBody, splitCore.first,
                                            NXOpen::Features::Feature::BooleanTypeSubtract, true);
        }
        if (splitCore.second && lowerBody) {
            lowerBody = applyBooleanFeature(part, lowerBody, splitCore.second,
                                            NXOpen::Features::Feature::BooleanTypeSubtract, true);
        }
        UF_OBJ_delete_object(coreBody->Tag());
    }

    // 13) Final: subtract part body from both mold blanks (do not retain tool).
    if (partBody) {
        if (upperBody) {
            upperBody = applyBooleanFeature(part, upperBody, partBody,
                                            NXOpen::Features::Feature::BooleanTypeSubtract, false);
        }
        if (lowerBody) {
            lowerBody = applyBooleanFeature(part, lowerBody, partBody,
                                            NXOpen::Features::Feature::BooleanTypeSubtract, false);
        }
    }

    // 14) Clean up intermediate body.
    if (intermediateBody) {
        UF_OBJ_delete_object(intermediateBody->Tag());
    }
#else
    // Baseline logic: subtract part body from mold blanks directly (do not retain tool).
    if (partBody) {
        upperBody = applyBooleanFeature(part, upperBody, partBody,
                                        NXOpen::Features::Feature::BooleanTypeSubtract, false);
        lowerBody = applyBooleanFeature(part, lowerBody, partBody,
                                        NXOpen::Features::Feature::BooleanTypeSubtract, false);
    }
#endif
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

    {
        stringstream s;
        s << "Demold direction: (" << result.demold.direction.x << ", "
          << result.demold.direction.y << ", " << result.demold.direction.z << ")";
        print(s.str());
    }
    {
        stringstream s;
        s << "Visibility ratio: " << result.demold.visibilityRatio
          << " | Undercut ratio: " << result.demold.undercutRatio;
        print(s.str());
    }
    {
        stringstream s;
        s << "Parting line points: " << result.partingLine.points.size()
          << " | Core regions: " << result.cores.size()
          << " | Sand cores: " << result.sandCores.size();
        print(s.str());
    }
    {
        stringstream s;
        s << "Separability: " << (result.separability.separable ? "separable" : "not separable")
          << " | Obstacles: " << result.separability.obstacles.size()
          << " | Max contour area: " << result.maxContour.area;
        print(s.str());
    }
    {
        stringstream s;
        s << "Contour source: " << (result.maxContour.selectedFromSlice ? "slice" : "fallback")
          << " | Slice index: " << result.maxContour.selectedSliceIndex
          << " | Local W: " << result.maxContour.selectedSliceW
          << " | Used fallback: " << (result.maxContour.fallbackUsed ? "yes" : "no");
        print(s.str());
    }

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
