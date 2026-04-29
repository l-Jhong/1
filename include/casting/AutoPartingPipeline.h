#pragma once

#include "Geometry.h"

#include <string>
#include <vector>

namespace casting {

constexpr int kUpperMoldColor = 186;
constexpr int kLowerMoldColor = 112;
constexpr int kCoreColor = 10;
constexpr int kCoreHeadColor = 25;
constexpr int kNonCastFeatureColor = 33;
// Base layer number for sand-core visualization in NXOpen.
constexpr int kSandCoreBaseLayer = 90;

struct DemoldEvaluation {
    Vector3 direction{};
    double score{};
    double visibilityRatio{};
    double undercutRatio{};
};

struct PartingLine {
    std::vector<Vector3> points;
};

struct PartingSurface {
    std::vector<Vector3> boundary;
};

struct SplitResult {
    Mesh upper;
    Mesh lower;
};

enum class CoreRegionBasicType {
    NoCore,
    ExternalCore,
    InternalCore
};

enum class CoreRegionDetailType {
    None,
    ThroughHole,
    BlindHole,
    ClosedCavity
};

enum class UndercutType {
    Reverse,
    Side,
    Complex
};

struct CoreRegion {
    std::size_t id{};
    std::vector<std::size_t> triangleIndices;
    Bounds bounds{};
    Vector3 pullDirection{};
    UndercutType undercutType = UndercutType::Reverse;
    CoreRegionBasicType basicType = CoreRegionBasicType::NoCore;
    CoreRegionDetailType detailType = CoreRegionDetailType::None;
    bool generateCore = true;
    bool castFeature = true;
    std::string nonCastReason;
    std::string machiningRecommendation;
    bool requiresSegmentation = false;
    bool isComposite = false;
    std::size_t openingCount = 0;
    double undercutDepth = 0.0;
    double cavityDepth = 0.0;
    double lengthDiameterRatio = 0.0;
    double minWallThickness = 0.0;
    std::vector<double> segmentationPositions;
    std::vector<std::size_t> childRegionIds;
    int nxColor = kCoreColor;
};

enum class CastingMaterial {
    CastSteel,
    CastIron
};

enum class ProductionBatch {
    MassProduction,
    BatchProduction,
    SmallBatch
};

enum class SandCoreType {
    ExternalSlide,
    InternalCavity,
    Runner,
    Composite
};

enum class SandCoreTopology {
    ThroughHole,
    BlindHole,
    ClosedCavity
};

enum class SandCoreShape {
    Straight,
    Curved
};

enum class SandCoreGenerationMethod {
    Extrude,
    Sweep,
    BooleanSubtract
};

enum class CoreHeadOrientation {
    VerticalUp,
    VerticalDown,
    HorizontalSupported,
    HorizontalCantilever
};

struct CoreHeadSpec {
    Vector3 position{};
    Vector3 direction{};
    double length{};
    double draftAngleDegrees{};
    double clearance{};
    double diameter{};
    bool antiCompressionRing = false;
    CoreHeadOrientation orientation = CoreHeadOrientation::VerticalUp;
};

struct SandCoreDiagnostics {
    bool touchesExternalSurface = false;
    std::size_t boundaryConnectionCount = 0;
    double lengthWidthRatio = 0.0;
    double centerlineMaxCurvatureDeg = 0.0;
    bool hasUndercutAlongPull = false;
    bool requiresSegmentation = false;
    SandCoreTopology topology = SandCoreTopology::ClosedCavity;
    SandCoreShape shape = SandCoreShape::Straight;
    SandCoreGenerationMethod generationMethod = SandCoreGenerationMethod::Extrude;
    std::vector<std::string> stageLogs;
};

struct SandCoreManufacturability {
    bool minWallThicknessOk = true;
    bool slendernessOk = true;
    bool pullPathClear = true;
    double minWallThickness = 0.0;
    double slendernessRatio = 0.0;
    std::vector<std::string> messages;
};

struct SandCore {
    std::size_t id{};
    SandCoreType type = SandCoreType::InternalCavity;
    Bounds geometryBounds{};
    Vector3 pullDirection{};
    std::vector<CoreHeadSpec> heads;
    std::vector<SandCore> subCores;
    SandCoreDiagnostics diagnostics;
    SandCoreManufacturability manufacturability;
    int nxColor = kCoreColor;
    int nxLayer = 91;
    std::string nxBodyName;
};

enum class ObstacleType {
    SlideCandidate,
    CoreCandidate,
    MultiDirection
};

struct ObstacleRegion {
    ObstacleType type = ObstacleType::MultiDirection;
    std::vector<std::size_t> triangleIndices;
    Bounds bounds{};
    Vector3 suggestedDirection{};
    double visibilityRatio{};
    double undercutRatio{};
    std::size_t coreId{};
    bool corePreferred = false;
};

struct SeparabilityReport {
    double score{};
    bool separable{};
    std::vector<ObstacleRegion> obstacles;
};

struct PartingSurfaceStage {
    std::string name;
    std::vector<Vector3> boundary;
};

struct ContourFace {
    std::vector<Vector3> boundary;
    Vector3 normal{};
    Vector3 centroid{};
    double area{};
    bool selectedFromSlice = false;
    int selectedSliceIndex = -1;
    double selectedSliceW = 0.0;
    bool fallbackUsed = false;
};

struct MoldBlock {
    std::string role;
    Bounds bounds{};
    Bounds cavityBounds{};
    Vector3 pullDirection{};
    bool subtractPart = true;
};

struct CoreInsert {
    Bounds bodyBounds{};
    Bounds headBounds{};
    Bounds seatBounds{};
    Vector3 pullDirection{};
};

struct MachiningRegion {
    Bounds bounds{};
    int nxColor = kNonCastFeatureColor;
    std::string reason;
    std::string recommendation;
};

struct MoldAssembly {
    Bounds overallBounds{};
    std::vector<MoldBlock> blocks;
    std::vector<CoreInsert> cores;
    std::vector<MachiningRegion> machiningRegions;
};

struct StrategyOption {
    std::string name;
    double score{};
    std::vector<ObstacleType> resolved;
};

struct InterferenceIssue {
    std::string message;
    double severity{};
};

struct AutoPartingSettings {
    // Length-related values use model units (typically mm in casting workflows).
    // Area value in squared model units (typically mm²).
    double minTriangleArea = 1e-8;
    // Angles are expressed in degrees.
    double draftAngleDegrees = 2.0;
    double undercutPenalty = 1.5;
    double visibilityPenalty = 0.5;
    double smoothingFactor = 0.3;
    double separabilityUndercutThreshold = 0.08;
    double partingSurfaceExtension = 80.0;
    double moldBlankPadding = 80.0;
    double moldClearance = 2.0;
    double shrinkageFactor = 0.012;
    double coreHeadLength = 25.0;
    double coreSeatClearance = 0.3;
    bool preferPlanarSurface = true;
    double nonPlanarDeviationRatio = 0.08;
    std::size_t maxCoreCount = 1;
    double minCoreVolumeRatio = 0.01;
    // Material used by manufacturability checks (minimum wall-thickness thresholds).
    CastingMaterial castingMaterial = CastingMaterial::CastIron;
    ProductionBatch productionBatch = ProductionBatch::BatchProduction;
};

struct AutoPartingResult {
    Mesh cleanedMesh;
    DemoldEvaluation demold;
    PartingLine partingLine;
    PartingSurface partingSurface;
    std::vector<PartingSurfaceStage> partingSurfaceStages;
    ContourFace maxContour;
    SeparabilityReport separability;
    SplitResult split;
    std::vector<CoreRegion> cores;
    std::vector<SandCore> sandCores;
    MoldAssembly moldAssembly;
    std::vector<StrategyOption> strategies;
    std::vector<InterferenceIssue> issues;
};

class AutoPartingPipeline {
public:
    AutoPartingResult run(const Mesh& input, const AutoPartingSettings& settings = {});
};

}  // namespace casting
