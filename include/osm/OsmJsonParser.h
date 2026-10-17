#pragma once

#include "osm/OsmDocument.h"

#include <cstdint>
#include <string>
#include <vector>

using namespace std;

// Result of parsing an Overpass JSON document (not GeoJSON).
// ok=false → error is set; document may be empty/partial and should not be used.
// Relations are stored on OsmDocument::relations; Stage 3 classifies supported
// turn restrictions vs unsupported (via-way / conditional / incomplete).
// unsupportedRelationIds remains for backward-compat callers (usually empty after parse).
// missingNodeRefs lists way node ids that were not present as node elements.
struct OsmParseResult {
    bool ok = false;
    string error;
    OsmDocument document;
    vector<int64_t> unsupportedRelationIds;
    vector<int64_t> missingNodeRefs;
};

class OsmJsonParser {
public:
    // Read Overpass JSON from a file path (working-directory relative or absolute).
    static OsmParseResult parseFile(const string& path);

    // Parse Overpass JSON text already in memory.
    static OsmParseResult parseString(const string& jsonText);
};
