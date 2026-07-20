#include "EquilibriumInspector.h"

void EquilibriumInspector::Clear() noexcept
{
    selectedIndex_.reset();
    equilibrium_.reset();
    payoffs_.reset();
    analysis_ = SimplexJacobianAnalysis{};
    dynamicsKind_ = DynamicsKind::Custom;
}

bool EquilibriumInspector::Select(
    std::size_t index,
    const std::vector<SimplexEquilibrium>& equilibria,
    const SimplexDynamicModel& dynamics,
    const SimplexJacobianSettings& settings
)
{
    if (index >= equilibria.size()) {
        return false;
    }

    const SimplexEquilibrium candidate = equilibria[index];
    const SimplexJacobianAnalysis candidateAnalysis = analyzer_.Analyze(
        dynamics,
        candidate.state,
        settings
    );

    selectedIndex_ = index;
    equilibrium_ = candidate;
    payoffs_ = dynamics.Payoffs(candidate.state);
    analysis_ = candidateAnalysis;
    dynamicsKind_ = dynamics.Kind();
    return true;
}

bool EquilibriumInspector::HasSelection() const noexcept
{
    return selectedIndex_.has_value() && equilibrium_.has_value();
}

std::size_t EquilibriumInspector::SelectedIndex() const noexcept
{
    return selectedIndex_.value_or(0U);
}

DynamicsKind EquilibriumInspector::SelectedDynamicsKind() const noexcept
{
    return dynamicsKind_;
}

const SimplexEquilibrium& EquilibriumInspector::Equilibrium() const noexcept
{
    return *equilibrium_;
}

const SimplexJacobianAnalysis&
EquilibriumInspector::Analysis() const noexcept
{
    return analysis_;
}

const std::optional<StrategyPayoffs>&
EquilibriumInspector::Payoffs() const noexcept
{
    return payoffs_;
}
