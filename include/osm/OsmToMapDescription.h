#pragma once

#include "core/MapDescription.h"
#include "core/Vector2D.h"
#include "osm/ImportReport.h"
#include "osm/OsmDocument.h"
#include "osm/OsmJsonParser.h"

#include <string>
#include <vector>

using namespace std;

// Stage 3: OsmDocument → MapDescription + ImportReport.
// Does not invent connections from crossing geometries without a shared OSM node.
// Simulation attachments (generators / phases) are OsmScenarioBuilder (Stage 4).

struct OsmLocalProjection {
    double originLatDeg = 0.0;
    double originLonDeg = 0.0;

    // Equirectangular meters relative to origin (testable).
    Vector2D project(double latDeg, double lonDeg) const;

    static OsmLocalProjection fromLatLon(double latDeg, double lonDeg);
    static OsmLocalProjection fromDocumentBBoxCenter(const OsmDocument& document);
};

struct OsmToMapResult {
    bool ok = false;
    string error;
    MapDescription map;
    ImportReport report;
};

class OsmToMapDescription {
public:
    // Convert an already-parsed document. Optional parse metadata fills relation /
    // missing-ref fields on the report (from OsmJsonParser).
    static OsmToMapResult convert(
        const OsmDocument& document,
        const string& sourceFile = {},
        const vector<int64_t>& unsupportedRelationIds = {},
        const vector<int64_t>& missingNodeRefs = {}
    );

    // Parse Overpass JSON then convert. Parse failures remain at the parse layer
    // (ok=false, error from OsmJsonParser; map empty).
    static OsmToMapResult convertFile(const string& path);
    static OsmToMapResult convertParseResult(const OsmParseResult& parsed, const string& sourceFile);

    // Polyline length in meters (sum of consecutive segment lengths).
    static double polylineLengthMeters(const vector<Vector2D>& points);
};
