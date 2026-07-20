#include "SimplexJacobianAnalyzer.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
    struct TangentDirection final
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    [[nodiscard]] std::optional<SimplexDerivative> EvaluateChecked(
        const SimplexDynamicModel& dynamics,
        const SimplexState& state
    )
    {
        const auto derivative = dynamics.Evaluate(state);
        if (!derivative.has_value()
            || !derivative->IsFinite()
            || !derivative->IsTangent()) {
            return std::nullopt;
        }
        return derivative;
    }

    [[nodiscard]] std::optional<SimplexState> Perturb(
        const SimplexState& state,
        const TangentDirection& direction,
        double scale
    )
    {
        return SimplexState::TryCreate(
            state.X() + scale * direction.x,
            state.Y() + scale * direction.y,
            state.Z() + scale * direction.z,
            1e-10
        );
    }

    [[nodiscard]] std::optional<ReducedJacobian> EstimateInterior(
        const SimplexDynamicModel& dynamics,
        const SimplexState& state,
        double requestedStep,
        double boundaryTolerance
    )
    {
        const double step = std::min({
            requestedStep,
            (state.X() - boundaryTolerance) * 0.5,
            (state.Y() - boundaryTolerance) * 0.5,
            (state.Z() - boundaryTolerance) * 0.5
        });
        if (!std::isfinite(step) || step <= 0.0) {
            return std::nullopt;
        }

        constexpr TangentDirection xDirection{ 1.0, 0.0, -1.0 };
        constexpr TangentDirection yDirection{ 0.0, 1.0, -1.0 };
        const auto positiveXState = Perturb(state, xDirection, step);
        const auto negativeXState = Perturb(state, xDirection, -step);
        const auto positiveYState = Perturb(state, yDirection, step);
        const auto negativeYState = Perturb(state, yDirection, -step);
        if (!positiveXState.has_value()
            || !negativeXState.has_value()
            || !positiveYState.has_value()
            || !negativeYState.has_value()) {
            return std::nullopt;
        }

        const auto positiveX = EvaluateChecked(dynamics, *positiveXState);
        const auto negativeX = EvaluateChecked(dynamics, *negativeXState);
        const auto positiveY = EvaluateChecked(dynamics, *positiveYState);
        const auto negativeY = EvaluateChecked(dynamics, *negativeYState);
        if (!positiveX.has_value()
            || !negativeX.has_value()
            || !positiveY.has_value()
            || !negativeY.has_value()) {
            return std::nullopt;
        }

        const ReducedJacobian result{
            (positiveX->dx - negativeX->dx) / (2.0 * step),
            (positiveY->dx - negativeY->dx) / (2.0 * step),
            (positiveX->dy - negativeX->dy) / (2.0 * step),
            (positiveY->dy - negativeY->dy) / (2.0 * step)
        };
        return result.IsFinite()
            ? std::optional<ReducedJacobian>{ result }
            : std::nullopt;
    }

    [[nodiscard]] std::array<TangentDirection, 2> BoundaryBasis(
        const SimplexState& state,
        double tolerance
    )
    {
        const bool zeroX = state.X() <= tolerance;
        const bool zeroY = state.Y() <= tolerance;
        const bool zeroZ = state.Z() <= tolerance;

        if (zeroY && zeroZ) {
            return {
                TangentDirection{ -1.0, 1.0, 0.0 },
                TangentDirection{ -1.0, 0.0, 1.0 }
            };
        }
        if (zeroX && zeroZ) {
            return {
                TangentDirection{ 1.0, -1.0, 0.0 },
                TangentDirection{ 0.0, -1.0, 1.0 }
            };
        }
        if (zeroX && zeroY) {
            return {
                TangentDirection{ 1.0, 0.0, -1.0 },
                TangentDirection{ 0.0, 1.0, -1.0 }
            };
        }
        if (zeroZ) {
            return {
                TangentDirection{ 1.0 - state.X(), -state.Y(), 0.0 },
                TangentDirection{ -state.X(), -state.Y(), 1.0 }
            };
        }
        if (zeroY) {
            return {
                TangentDirection{ 1.0 - state.X(), 0.0, -state.Z() },
                TangentDirection{ -state.X(), 1.0, -state.Z() }
            };
        }

        return {
            TangentDirection{ 0.0, 1.0 - state.Y(), -state.Z() },
            TangentDirection{ 1.0, -state.Y(), -state.Z() }
        };
    }

    [[nodiscard]] std::optional<ReducedJacobian> EstimateBoundary(
        const SimplexDynamicModel& dynamics,
        const SimplexState& state,
        double step,
        double boundaryTolerance
    )
    {
        if (!std::isfinite(step) || step <= 0.0 || step >= 1.0) {
            return std::nullopt;
        }

        const auto base = EvaluateChecked(dynamics, state);
        const auto basis = BoundaryBasis(state, boundaryTolerance);
        const auto firstState = Perturb(state, basis[0], step);
        const auto secondState = Perturb(state, basis[1], step);
        if (!base.has_value()
            || !firstState.has_value()
            || !secondState.has_value()) {
            return std::nullopt;
        }

        const auto first = EvaluateChecked(dynamics, *firstState);
        const auto second = EvaluateChecked(dynamics, *secondState);
        if (!first.has_value() || !second.has_value()) {
            return std::nullopt;
        }

        const double d00 = basis[0].x;
        const double d10 = basis[0].y;
        const double d01 = basis[1].x;
        const double d11 = basis[1].y;
        const double determinant = d00 * d11 - d01 * d10;
        if (!std::isfinite(determinant)
            || std::abs(determinant) <= 1e-14) {
            return std::nullopt;
        }

        const double y00 = (first->dx - base->dx) / step;
        const double y10 = (first->dy - base->dy) / step;
        const double y01 = (second->dx - base->dx) / step;
        const double y11 = (second->dy - base->dy) / step;

        // J * D = Y, so J = Y * inverse(D).
        const ReducedJacobian result{
            (y00 * d11 - y01 * d10) / determinant,
            (-y00 * d01 + y01 * d00) / determinant,
            (y10 * d11 - y11 * d10) / determinant,
            (-y10 * d01 + y11 * d00) / determinant
        };
        return result.IsFinite()
            ? std::optional<ReducedJacobian>{ result }
            : std::nullopt;
    }

    [[nodiscard]] double MatrixDifferenceNorm(
        const ReducedJacobian& left,
        const ReducedJacobian& right
    ) noexcept
    {
        return std::sqrt(
            std::pow(left.dxByDx - right.dxByDx, 2.0)
            + std::pow(left.dxByDy - right.dxByDy, 2.0)
            + std::pow(left.dyByDx - right.dyByDx, 2.0)
            + std::pow(left.dyByDy - right.dyByDy, 2.0)
        );
    }

    [[nodiscard]] SimplexEigenpair MakeEigenpair(
        const ReducedJacobian& matrix,
        std::complex<double> eigenvalue,
        double tolerance
    )
    {
        std::complex<double> first = matrix.dxByDy;
        std::complex<double> second = eigenvalue - matrix.dxByDx;
        double norm = std::sqrt(std::norm(first) + std::norm(second));
        if (norm <= tolerance) {
            first = eigenvalue - matrix.dyByDy;
            second = matrix.dyByDx;
            norm = std::sqrt(std::norm(first) + std::norm(second));
        }
        if (norm <= tolerance) {
            return SimplexEigenpair{ eigenvalue, {}, {}, false };
        }

        first /= norm;
        second /= norm;
        return SimplexEigenpair{
            eigenvalue,
            { first, second },
            { first, second, -first - second },
            true
        };
    }
}

bool ReducedJacobian::IsFinite() const noexcept
{
    return std::isfinite(dxByDx)
        && std::isfinite(dxByDy)
        && std::isfinite(dyByDx)
        && std::isfinite(dyByDy);
}

double ReducedJacobian::FrobeniusNorm() const noexcept
{
    return std::sqrt(
        dxByDx * dxByDx
        + dxByDy * dxByDy
        + dyByDx * dyByDx
        + dyByDy * dyByDy
    );
}

bool SimplexEigenpair::IsReal(double tolerance) const noexcept
{
    if (!hasEigenvector || !std::isfinite(tolerance) || tolerance < 0.0) {
        return false;
    }

    return std::abs(eigenvalue.imag()) <= tolerance
        && std::abs(reducedEigenvector[0].imag()) <= tolerance
        && std::abs(reducedEigenvector[1].imag()) <= tolerance;
}

bool SimplexJacobianSettings::IsComputable() const noexcept
{
    return std::isfinite(finiteDifferenceStep)
        && std::isfinite(boundaryTolerance)
        && std::isfinite(convergenceRelativeTolerance)
        && std::isfinite(eigenvalueRelativeTolerance)
        && finiteDifferenceStep > 0.0
        && finiteDifferenceStep < 0.25
        && boundaryTolerance >= 0.0
        && boundaryTolerance < 0.25
        && convergenceRelativeTolerance > 0.0
        && eigenvalueRelativeTolerance > 0.0;
}

const char* JacobianAnalysisStatusName(
    JacobianAnalysisStatus status
) noexcept
{
    switch (status) {
    case JacobianAnalysisStatus::Available:
        return "Available";
    case JacobianAnalysisStatus::AvailableWithConvergenceWarning:
        return "Available with finite-difference warning";
    case JacobianAnalysisStatus::NonsmoothSwitchingSurface:
        return "Classical Jacobian undefined at switching surface";
    case JacobianAnalysisStatus::EvaluationFailure:
        return "Vector-field evaluation failed";
    case JacobianAnalysisStatus::InvalidInput:
        return "Invalid analysis input";
    }

    return "Unknown";
}

const char* StabilityClassificationName(
    StabilityClassification classification
) noexcept
{
    switch (classification) {
    case StabilityClassification::AttractingNode:
        return "Attracting node";
    case StabilityClassification::RepellingNode:
        return "Repelling node";
    case StabilityClassification::Saddle:
        return "Saddle";
    case StabilityClassification::SpiralSink:
        return "Spiral sink";
    case StabilityClassification::SpiralSource:
        return "Spiral source";
    case StabilityClassification::CenterLike:
        return "Center-like";
    case StabilityClassification::NonHyperbolicOrInconclusive:
        return "Non-hyperbolic / inconclusive";
    }

    return "Unknown";
}

SimplexJacobianAnalysis SimplexJacobianAnalyzer::Analyze(
    const SimplexDynamicModel& dynamics,
    const SimplexState& state,
    const SimplexJacobianSettings& settings
) const
{
    SimplexJacobianAnalysis result;
    if (!state.IsValid() || !settings.IsComputable()) {
        result.message = "The state or finite-difference settings are invalid.";
        return result;
    }

    if (!dynamics.IsClassicallyDifferentiableAt(state)) {
        result.status = JacobianAnalysisStatus::NonsmoothSwitchingSurface;
        result.message =
            "Classical Jacobian undefined at this Best Response switching surface.";
        return result;
    }

    const bool boundary = state.X() <= settings.boundaryTolerance
        || state.Y() <= settings.boundaryTolerance
        || state.Z() <= settings.boundaryTolerance;
    result.isBoundaryConstrained = boundary;

    const auto coarse = boundary
        ? EstimateBoundary(
            dynamics,
            state,
            settings.finiteDifferenceStep,
            settings.boundaryTolerance
        )
        : EstimateInterior(
            dynamics,
            state,
            settings.finiteDifferenceStep,
            settings.boundaryTolerance
        );
    const auto fine = boundary
        ? EstimateBoundary(
            dynamics,
            state,
            settings.finiteDifferenceStep * 0.5,
            settings.boundaryTolerance
        )
        : EstimateInterior(
            dynamics,
            state,
            settings.finiteDifferenceStep * 0.5,
            settings.boundaryTolerance
        );
    if (!coarse.has_value() || !fine.has_value()) {
        result.status = JacobianAnalysisStatus::EvaluationFailure;
        result.message = "Could not evaluate a valid finite-difference stencil.";
        return result;
    }

    result = AnalyzeMatrix(*fine, settings.eigenvalueRelativeTolerance);
    result.isBoundaryConstrained = boundary;
    result.finiteDifferenceRelativeError = MatrixDifferenceNorm(
        *coarse,
        *fine
    ) / std::max(1.0, fine->FrobeniusNorm());
    if (result.finiteDifferenceRelativeError
        > settings.convergenceRelativeTolerance) {
        result.status =
            JacobianAnalysisStatus::AvailableWithConvergenceWarning;
        result.message =
            "The h and h/2 finite-difference estimates did not fully converge.";
    }
    else if (boundary) {
        result.message =
            "One-sided feasible stencil; classification is simplex-constrained.";
    }
    else {
        result.message =
            "Central reduced-chart stencil with z = 1 - x - y.";
    }

    return result;
}

SimplexJacobianAnalysis SimplexJacobianAnalyzer::AnalyzeMatrix(
    const ReducedJacobian& jacobian,
    double eigenvalueRelativeTolerance
)
{
    SimplexJacobianAnalysis result;
    if (!jacobian.IsFinite()
        || !std::isfinite(eigenvalueRelativeTolerance)
        || eigenvalueRelativeTolerance <= 0.0) {
        result.message = "The reduced Jacobian is invalid.";
        return result;
    }

    result.status = JacobianAnalysisStatus::Available;
    result.jacobian = jacobian;
    const double trace = jacobian.dxByDx + jacobian.dyByDy;
    const double determinant = jacobian.dxByDx * jacobian.dyByDy
        - jacobian.dxByDy * jacobian.dyByDx;
    const std::complex<double> discriminant{
        trace * trace - 4.0 * determinant,
        0.0
    };
    const std::complex<double> root = std::sqrt(discriminant);
    const std::complex<double> lambda0 = 0.5 * (trace + root);
    const std::complex<double> lambda1 = 0.5 * (trace - root);
    const double matrixScale = 1.0 + jacobian.FrobeniusNorm();
    const double tolerance = eigenvalueRelativeTolerance * matrixScale;

    result.isRepeatedEigenvalue =
        std::abs(lambda0 - lambda1) <= tolerance;
    if (result.isRepeatedEigenvalue) {
        const std::complex<double> lambda = 0.5 * (lambda0 + lambda1);
        const bool scalarMatrix =
            std::abs(jacobian.dxByDy) <= tolerance
            && std::abs(jacobian.dyByDx) <= tolerance
            && std::abs(jacobian.dxByDx - lambda.real()) <= tolerance
            && std::abs(jacobian.dyByDy - lambda.real()) <= tolerance;
        if (scalarMatrix) {
            result.eigenpairs[0] = SimplexEigenpair{
                lambda,
                { 1.0, 0.0 },
                { 1.0, 0.0, -1.0 },
                true
            };
            result.eigenpairs[1] = SimplexEigenpair{
                lambda,
                { 0.0, 1.0 },
                { 0.0, 1.0, -1.0 },
                true
            };
        }
        else {
            result.eigenpairs[0] = MakeEigenpair(jacobian, lambda, tolerance);
            result.eigenpairs[1].eigenvalue = lambda;
            result.isDefective = true;
            result.message =
                "Repeated eigenvalue with only one independent eigenvector.";
        }
    }
    else {
        result.eigenpairs[0] = MakeEigenpair(jacobian, lambda0, tolerance);
        result.eigenpairs[1] = MakeEigenpair(jacobian, lambda1, tolerance);
    }

    const double real0 = lambda0.real();
    const double real1 = lambda1.real();
    const bool complexPair = std::abs(lambda0.imag()) > tolerance
        || std::abs(lambda1.imag()) > tolerance;
    if (complexPair) {
        if (real0 < -tolerance) {
            result.classification = StabilityClassification::SpiralSink;
        }
        else if (real0 > tolerance) {
            result.classification = StabilityClassification::SpiralSource;
        }
        else {
            result.classification = StabilityClassification::CenterLike;
        }
    }
    else if ((real0 < -tolerance && real1 > tolerance)
             || (real0 > tolerance && real1 < -tolerance)) {
        result.classification = StabilityClassification::Saddle;
    }
    else if (real0 < -tolerance && real1 < -tolerance) {
        result.classification = StabilityClassification::AttractingNode;
    }
    else if (real0 > tolerance && real1 > tolerance) {
        result.classification = StabilityClassification::RepellingNode;
    }
    else {
        result.classification =
            StabilityClassification::NonHyperbolicOrInconclusive;
    }

    return result;
}
