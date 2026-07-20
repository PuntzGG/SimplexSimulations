#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include "DynamicsKind.h"
#include "SimplexDynamicModel.h"
#include "SimplexEquilibriumFinder.h"
#include "SimplexJacobianAnalyzer.h"
#include "StrategyPayoffs.h"

class EquilibriumInspector final
{
public:
    void Clear() noexcept;

    [[nodiscard]] bool Select(
        std::size_t index,
        const std::vector<SimplexEquilibrium>& equilibria,
        const SimplexDynamicModel& dynamics,
        const SimplexJacobianSettings& settings = {}
    );

    [[nodiscard]] bool HasSelection() const noexcept;
    [[nodiscard]] std::size_t SelectedIndex() const noexcept;
    [[nodiscard]] DynamicsKind SelectedDynamicsKind() const noexcept;
    [[nodiscard]] const SimplexEquilibrium& Equilibrium() const noexcept;
    [[nodiscard]] const SimplexJacobianAnalysis& Analysis() const noexcept;
    [[nodiscard]] const std::optional<StrategyPayoffs>& Payoffs() const noexcept;

private:
    SimplexJacobianAnalyzer analyzer_;
    std::optional<std::size_t> selectedIndex_;
    std::optional<SimplexEquilibrium> equilibrium_;
    std::optional<StrategyPayoffs> payoffs_;
    SimplexJacobianAnalysis analysis_;
    DynamicsKind dynamicsKind_ = DynamicsKind::Custom;
};
