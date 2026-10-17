#pragma once

#include "osm/GeoBbox.h"

#include <string>

namespace osm_net {

// Shared Overpass QL builder (Stage 1 highway filter + type=restriction relations).
// Keep in sync with scripts/fetch_osm_sample.ps1.
struct OverpassQueryOptions {
    int timeoutSeconds = 25;
};

std::string buildRoadNetworkQuery(const GeoBbox& bbox, const OverpassQueryOptions& options = {});

// Highway regex used by the query (documentation / tests).
inline constexpr const char* kHighwayRegex =
    "^(motorway|trunk|primary|secondary|tertiary|unclassified|residential|living_street|service)$";

} // namespace osm_net
