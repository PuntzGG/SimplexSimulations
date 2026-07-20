#include "DynamicsFactory.h"

#include "LogitDynamics.h"
#include "ReplicatorDynamics.h"

std::unique_ptr<SimplexDynamicModel> CreateDynamics(
    DynamicsKind kind,
    const OpggParameters& parameters,
    const BestResponseSettings& bestResponseSettings
)
{
    if (!parameters.IsComputable() || !bestResponseSettings.IsComputable()) {
        return nullptr;
    }

    switch (kind) {
    case DynamicsKind::Logit:
        return std::make_unique<LogitDynamics>(parameters);
    case DynamicsKind::EqualSplitBestResponse:
        return std::make_unique<BestResponseDynamics>(
            parameters,
            bestResponseSettings
        );
    case DynamicsKind::Replicator:
        return std::make_unique<ReplicatorDynamics>(parameters);
    case DynamicsKind::Custom:
        return nullptr;
    }

    return nullptr;
}
