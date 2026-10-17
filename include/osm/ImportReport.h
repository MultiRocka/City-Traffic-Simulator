#pragma once

#include <cstdint>
#include <cstddef>
#include <string>
#include <vector>

using namespace std;

// Provenance categories for tag → MapDescription interpretation (Stage 3).
// Values are short human-readable notes (not a structured key/value schema).

struct ImportReport {
    string sourceFile;

    size_t inputNodeCount = 0;
    size_t inputWayCount = 0;
    size_t inputRelationCount = 0;

    size_t builtIntersectionCount = 0;
    size_t builtRoadCount = 0;
    size_t builtLaneCount = 0; // sum of per-direction laneCount on built roads
    size_t builtCrossingCount = 0;
    size_t signalizedIntersectionCount = 0;

    // Tag / policy accounting (mutually intended categories).
    vector<string> readFromTags;   // value taken directly from a supported tag
    vector<string> inferred;       // derived from tags with a documented rule
    vector<string> defaultsApplied; // simulation / importer default (assumption)
    vector<string> unsupported;    // important tags seen but not enforced

    vector<string> skipped;        // ways/nodes/segments skipped with reasons
    vector<int64_t> missingNodeRefs;
    vector<int64_t> unsupportedRelationIds;

    size_t graphComponentCount = 0;
    string graphComponentsNote;
    string routingLimitsNote;
    string incompletenessNote;

    vector<string> warnings;
};
