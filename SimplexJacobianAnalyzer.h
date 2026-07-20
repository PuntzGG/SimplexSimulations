#pragma once

#include <array>
#include <complex>
#include <optional>
#include <string>

#include "SimplexDynamicModel.h"
#include "SimplexState.h"

struct ReducedJacobian final
{
    double dxByDx = 0.0;
    double dxByDy = 0.0;
    double dyByDx = 0.0;
    double dyByDy = 0.0;

    [[nodiscard]] bool IsFinite() const noexcept;
    [[nodiscard]] double FrobeniusNorm() const noexcept;
};

struct SimplexEigenpair final
{
    std::complex<double> eigenvalue{};
    std::array<std::complex<double>, 2> reducedEigenvector{};
    std::array<std::complex<double>, 3> simplexEigenvector{};
    bool hasEigenvector = false;

    [[nodiscard]] bool IsReal(double tolerance = 1e-10) const noexcept;
};

enum class JacobianAnalysisStatus
{
    Available,
    AvailableWithConvergenceWarning,
    NonsmoothSwitchingSurface,
    EvaluationFailure,
    InvalidInput
};

enum class StabilityClassification
{
    AttractingNode,
    RepellingNode,
    Saddle,
    SpiralSink,
    SpiralSource,
    CenterLike,
    NonHyperbolicOrInconclusive
};

struct SimplexJacobianSettings final
{
    double finiteDifferenceStep = 1e-5;
    double boundaryTolerance = 1e-8;
    double convergenceRelativeTolerance = 5e-3;
    double eigenvalueRelativeTolerance = 1e-8;

    [[nodiscard]] bool IsComputable() const noexcept;
};

struct SimplexJacobianAnalysis final
{
    JacobianAnalysisStatus status = JacobianAnalysisStatus::InvalidInput;
    std::optional<ReducedJacobian> jacobian;
    std::array<SimplexEigenpair, 2> eigenpairs{};
    StabilityClassification classification =
        StabilityClassification::NonHyperbolicOrInconclusive;
    bool isBoundaryConstrained = false;
    bool isRepeatedEigenvalue = false;
    bool isDefective = false;
    double finiteDifferenceRelativeError = 0.0;
    std::string message;
};

[[nodiscard]] const char* JacobianAnalysisStatusName(
    JacobianAnalysisStatus status
) noexcept;
[[nodiscard]] const char* StabilityClassificationName(
    StabilityClassification classification
) noexcept;

class SimplexJacobianAnalyzer final
{
public:
    [[nodiscard]] SimplexJacobianAnalysis Analyze(
        const SimplexDynamicModel& dynamics,
        const SimplexState& state,
        const SimplexJacobianSettings& settings = {}
    ) const;

    [[nodiscard]] static SimplexJacobianAnalysis AnalyzeMatrix(
        const ReducedJacobian& jacobian,
        double eigenvalueRelativeTolerance = 1e-8
    );
};
