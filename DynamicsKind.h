#pragma once

#include <cstdint>
#include <string_view>

enum class DynamicsKind : std::uint8_t
{
    Logit,
    EqualSplitBestResponse,
    Replicator,
    Custom
};

struct DynamicsCapabilities final
{
    bool hasResponseTarget = false;
    bool isGloballySmooth = false;
    bool hasClassicalJacobian = false;
    bool canHaveBoundaryEquilibria = false;
};

[[nodiscard]] constexpr std::string_view DynamicsName(
    DynamicsKind kind
) noexcept
{
    switch (kind) {
    case DynamicsKind::Logit:
        return "Logit";
    case DynamicsKind::EqualSplitBestResponse:
        return "Equal-split Best Response";
    case DynamicsKind::Replicator:
        return "Replicator";
    case DynamicsKind::Custom:
        return "Custom";
    }

    return "Unknown";
}

[[nodiscard]] constexpr DynamicsCapabilities CapabilitiesFor(
    DynamicsKind kind
) noexcept
{
    switch (kind) {
    case DynamicsKind::Logit:
        return DynamicsCapabilities{ true, true, true, false };
    case DynamicsKind::EqualSplitBestResponse:
        return DynamicsCapabilities{ true, false, false, true };
    case DynamicsKind::Replicator:
        return DynamicsCapabilities{ false, true, true, true };
    case DynamicsKind::Custom:
        return DynamicsCapabilities{};
    }

    return DynamicsCapabilities{};
}
