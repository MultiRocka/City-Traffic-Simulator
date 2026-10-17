#include "osm/OsmToMapDescription.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

using namespace std;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kMetersPerDegLat = 111132.92;
constexpr double kMetersPerDegLonEquator = 111319.9;
constexpr double kDefaultMaxSpeedKmh = 50.0;
constexpr double kKmhToMs = 1000.0 / 3600.0;
constexpr double kMphToMs = 1609.344 / 3600.0;

void pushUnique(vector<string>& list, const string& note) {
    if (note.empty()) {
        return;
    }
    if (find(list.begin(), list.end(), note) == list.end()) {
        list.push_back(note);
    }
}

string toLowerCopy(string value) {
    for (char& c : value) {
        if (c >= 'A' && c <= 'Z') {
            c = static_cast<char>(c - 'A' + 'a');
        }
    }
    return value;
}

bool isCarHighway(const string& highway) {
    static const unordered_set<string> kAllowed = {
        "motorway", "motorway_link",
        "trunk", "trunk_link",
        "primary", "primary_link",
        "secondary", "secondary_link",
        "tertiary", "tertiary_link",
        "unclassified", "residential", "living_street", "service",
        "road"
    };
    return kAllowed.count(highway) > 0;
}

bool looksLikeUnsupportedImportantTag(const string& key) {
    // Bare access / vehicle / motor_* are enforced via the car access hierarchy.
    // Remaining: turn:lanes*, access:* subkeys, and any *conditional*.
    if (key == "turn:lanes" || key == "turn:lanes:forward" || key == "turn:lanes:backward") {
        return true;
    }
    if (key.rfind("access:", 0) == 0) {
        return true;
    }
    if (key.find("conditional") != string::npos) {
        return true;
    }
    return false;
}

enum class CarAccessClass {
    Open,
    DestinationOnly,
    Forbidden,
    UnsupportedRestricted
};

string resolveCarAccessValue(const map<string, string>& tags) {
    // More specific wins: access → vehicle → motor_vehicle → motorcar.
    string value;
    auto apply = [&](const char* key) {
        const string raw = OsmDocument::tagOrEmpty(tags, key);
        if (!raw.empty()) {
            value = toLowerCopy(raw);
        }
    };
    apply("access");
    apply("vehicle");
    apply("motor_vehicle");
    apply("motorcar");
    return value;
}

bool accessValueLooksConditional(const string& value) {
    return value.find('@') != string::npos || value.find("yes @") != string::npos;
}

CarAccessClass classifyCarAccess(const map<string, string>& tags, string& resolvedValue) {
    for (const auto& tag : tags) {
        if (tag.first.find("conditional") != string::npos) {
            resolvedValue = toLowerCopy(tag.second);
            return CarAccessClass::UnsupportedRestricted;
        }
    }

    resolvedValue = resolveCarAccessValue(tags);
    if (resolvedValue.empty()) {
        return CarAccessClass::Open;
    }
    if (accessValueLooksConditional(resolvedValue)) {
        return CarAccessClass::UnsupportedRestricted;
    }
    if (resolvedValue == "yes" || resolvedValue == "permissive" || resolvedValue == "designated" ||
        resolvedValue == "official" || resolvedValue == "public") {
        return CarAccessClass::Open;
    }
    if (resolvedValue == "no" || resolvedValue == "private" || resolvedValue == "military") {
        return CarAccessClass::Forbidden;
    }
    if (resolvedValue == "destination" || resolvedValue == "customers" ||
        resolvedValue == "delivery" || resolvedValue == "agricultural" ||
        resolvedValue == "forestry" || resolvedValue == "permit") {
        return CarAccessClass::DestinationOnly;
    }
    if (resolvedValue == "unknown") {
        return CarAccessClass::UnsupportedRestricted;
    }
    // Unrecognized non-empty value: conservative — do not treat as free transit.
    return CarAccessClass::UnsupportedRestricted;
}

bool exceptExemptsCars(const string& exceptRaw) {
    string except = toLowerCopy(exceptRaw);
    for (char& c : except) {
        if (c == ',' || c == '|') {
            c = ';';
        }
    }
    string token;
    for (size_t i = 0; i <= except.size(); ++i) {
        if (i == except.size() || except[i] == ';') {
            if (token == "motorcar" || token == "motor_vehicle" || token == "vehicle" ||
                token == "psv" || token == "bus" || token == "taxi" || token == "car") {
                return true;
            }
            token.clear();
        } else if (except[i] != ' ') {
            token.push_back(except[i]);
        }
    }
    return false;
}

bool isSupportedNoRestriction(const string& restriction) {
    return restriction == "no_left_turn" || restriction == "no_right_turn" ||
           restriction == "no_straight_on" || restriction == "no_u_turn" ||
           restriction == "no_entry" || restriction == "no_exit";
}

bool isSupportedOnlyRestriction(const string& restriction) {
    return restriction == "only_left_turn" || restriction == "only_right_turn" ||
           restriction == "only_straight_on" || restriction == "only_u_turn";
}

struct SpeedParse {
    bool ok = false;
    double metersPerSecond = 0.0;
    string detail; // for report
};

SpeedParse parseMaxspeedValue(const string& raw) {
    SpeedParse out;
    const string value = toLowerCopy(raw);
    if (value.empty() || value == "none" || value == "signals" || value == "walk" ||
        value == "variable" || value == "unposted") {
        out.detail = "unsupported maxspeed='" + raw + "'";
        return out;
    }

    // Explicit unit suffixes (do not blindly strip the first number).
    const auto tryNumber = [](const string& text, size_t begin, size_t end, double& number) -> bool {
        if (begin >= end) {
            return false;
        }
        try {
            size_t consumed = 0;
            number = stod(text.substr(begin, end - begin), &consumed);
            return consumed > 0 && begin + consumed == end;
        } catch (...) {
            return false;
        }
    };

    auto endsWith = [&](const string& suffix) {
        return value.size() >= suffix.size() &&
               value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
    };

    double number = 0.0;
    if (endsWith(" mph")) {
        if (!tryNumber(value, 0, value.size() - 4, number) || number <= 0.0) {
            out.detail = "unsupported maxspeed='" + raw + "'";
            return out;
        }
        out.ok = true;
        out.metersPerSecond = number * kMphToMs;
        out.detail = "maxspeed " + raw + " → mph";
        return out;
    }
    if (endsWith("mph")) {
        if (!tryNumber(value, 0, value.size() - 3, number) || number <= 0.0) {
            out.detail = "unsupported maxspeed='" + raw + "'";
            return out;
        }
        out.ok = true;
        out.metersPerSecond = number * kMphToMs;
        out.detail = "maxspeed " + raw + " → mph";
        return out;
    }
    if (endsWith(" km/h") || endsWith(" kmh") || endsWith(" kph")) {
        size_t cut = 5;
        if (endsWith(" kmh") || endsWith(" kph")) {
            cut = 4;
        }
        if (!tryNumber(value, 0, value.size() - cut, number) || number <= 0.0) {
            out.detail = "unsupported maxspeed='" + raw + "'";
            return out;
        }
        out.ok = true;
        out.metersPerSecond = number * kKmhToMs;
        out.detail = "maxspeed " + raw + " → km/h";
        return out;
    }
    if (endsWith("km/h") || endsWith("kmh") || endsWith("kph")) {
        size_t cut = 4;
        if (endsWith("kmh") || endsWith("kph")) {
            cut = 3;
        }
        if (!tryNumber(value, 0, value.size() - cut, number) || number <= 0.0) {
            out.detail = "unsupported maxspeed='" + raw + "'";
            return out;
        }
        out.ok = true;
        out.metersPerSecond = number * kKmhToMs;
        out.detail = "maxspeed " + raw + " → km/h";
        return out;
    }
    if (endsWith(" knots") || endsWith("knots")) {
        out.detail = "unsupported maxspeed='" + raw + "'";
        return out;
    }

    // Bare number: documented default unit = km/h (EU / Overpass samples).
    try {
        size_t consumed = 0;
        number = stod(value, &consumed);
        if (consumed != value.size() || number <= 0.0) {
            out.detail = "unsupported maxspeed='" + raw + "'";
            return out;
        }
    } catch (...) {
        out.detail = "unsupported maxspeed='" + raw + "'";
        return out;
    }
    out.ok = true;
    out.metersPerSecond = number * kKmhToMs;
    out.detail = "maxspeed " + raw + " (bare → km/h)";
    return out;
}

bool parsePositiveInt(const string& raw, int& outValue) {
    if (raw.empty()) {
        return false;
    }
    try {
        size_t consumed = 0;
        const int value = stoi(raw, &consumed);
        if (consumed != raw.size() || value < 1) {
            return false;
        }
        outValue = value;
        return true;
    } catch (...) {
        return false;
    }
}

enum class OnewayPolicy {
    Bidirectional,
    ForwardOnly,
    ReverseOnly
};

OnewayPolicy interpretOneway(const map<string, string>& tags, ImportReport& report) {
    const string junction = toLowerCopy(OsmDocument::tagOrEmpty(tags, "junction"));
    if (junction == "roundabout" || junction == "circular") {
        pushUnique(report.inferred, "junction=roundabout/circular → oneway forward");
        return OnewayPolicy::ForwardOnly;
    }

    const string oneway = toLowerCopy(OsmDocument::tagOrEmpty(tags, "oneway"));
    if (oneway.empty()) {
        return OnewayPolicy::Bidirectional;
    }
    if (oneway == "yes" || oneway == "true" || oneway == "1") {
        pushUnique(report.readFromTags, "oneway=yes → forward only");
        return OnewayPolicy::ForwardOnly;
    }
    if (oneway == "-1" || oneway == "reverse") {
        pushUnique(report.readFromTags, "oneway=-1/reverse → reverse only");
        return OnewayPolicy::ReverseOnly;
    }
    if (oneway == "no" || oneway == "false" || oneway == "0") {
        pushUnique(report.readFromTags, "oneway=no → bidirectional");
        return OnewayPolicy::Bidirectional;
    }
    pushUnique(report.unsupported, "unsupported oneway='" + oneway + "' (treated bidirectional)");
    return OnewayPolicy::Bidirectional;
}

struct LaneCounts {
    int forward = 1;
    int backward = 1;
};

LaneCounts interpretLanes(const map<string, string>& tags, OnewayPolicy oneway, ImportReport& report) {
    LaneCounts counts;
    int lanesForward = -1;
    int lanesBackward = -1;
    int lanesTotal = -1;

    const string lf = OsmDocument::tagOrEmpty(tags, "lanes:forward");
    const string lb = OsmDocument::tagOrEmpty(tags, "lanes:backward");
    const string lanes = OsmDocument::tagOrEmpty(tags, "lanes");

    if (!lf.empty()) {
        if (parsePositiveInt(lf, lanesForward)) {
            pushUnique(report.readFromTags, "lanes:forward=" + lf);
        } else {
            pushUnique(report.unsupported, "unsupported lanes:forward='" + lf + "'");
        }
    }
    if (!lb.empty()) {
        if (parsePositiveInt(lb, lanesBackward)) {
            pushUnique(report.readFromTags, "lanes:backward=" + lb);
        } else {
            pushUnique(report.unsupported, "unsupported lanes:backward='" + lb + "'");
        }
    }
    if (!lanes.empty()) {
        if (parsePositiveInt(lanes, lanesTotal)) {
            pushUnique(report.readFromTags, "lanes=" + lanes);
        } else {
            pushUnique(report.unsupported, "unsupported lanes='" + lanes + "'");
        }
    }

    if (oneway == OnewayPolicy::ForwardOnly) {
        if (lanesForward > 0) {
            counts.forward = lanesForward;
        } else if (lanesTotal > 0) {
            counts.forward = lanesTotal;
            pushUnique(report.inferred, "oneway forward uses lanes as directional count");
        } else {
            counts.forward = 1;
            pushUnique(report.defaultsApplied, "laneCount default 1 (forward oneway)");
        }
        counts.backward = 0;
        return counts;
    }
    if (oneway == OnewayPolicy::ReverseOnly) {
        if (lanesBackward > 0) {
            counts.backward = lanesBackward;
        } else if (lanesTotal > 0) {
            counts.backward = lanesTotal;
            pushUnique(report.inferred, "oneway reverse uses lanes as directional count");
        } else {
            counts.backward = 1;
            pushUnique(report.defaultsApplied, "laneCount default 1 (reverse oneway)");
        }
        counts.forward = 0;
        return counts;
    }

    // Bidirectional: directional tags win; never copy total lanes to both directions.
    if (lanesForward > 0 || lanesBackward > 0) {
        counts.forward = lanesForward > 0 ? lanesForward : 1;
        counts.backward = lanesBackward > 0 ? lanesBackward : 1;
        if (lanesForward <= 0 || lanesBackward <= 0) {
            pushUnique(report.defaultsApplied,
                       "bidirectional missing one of lanes:forward/backward → default 1 on missing side");
        }
        return counts;
    }

    if (lanesTotal > 0) {
        // Documented default: split total across directions; remainder to forward.
        // lanes=1 on a two-way street → 1 each (shared narrow road assumption).
        if (lanesTotal == 1) {
            counts.forward = 1;
            counts.backward = 1;
            pushUnique(report.defaultsApplied,
                       "bidirectional lanes=1 → 1 lane each direction (narrow-road assumption)");
        } else {
            counts.backward = max(1, lanesTotal / 2);
            counts.forward = max(1, lanesTotal - counts.backward);
            pushUnique(report.inferred,
                       "bidirectional lanes=" + to_string(lanesTotal) +
                           " → forward " + to_string(counts.forward) +
                           " / backward " + to_string(counts.backward));
        }
        return counts;
    }

    counts.forward = 1;
    counts.backward = 1;
    pushUnique(report.defaultsApplied, "laneCount default 1 each direction (no lanes tags)");
    return counts;
}

struct SpeedPair {
    double forwardMs = kDefaultMaxSpeedKmh * kKmhToMs;
    double backwardMs = kDefaultMaxSpeedKmh * kKmhToMs;
};

SpeedPair interpretSpeeds(
    const map<string, string>& tags,
    OnewayPolicy oneway,
    ImportReport& report
) {
    SpeedPair speeds;
    const string ms = OsmDocument::tagOrEmpty(tags, "maxspeed");
    const string msf = OsmDocument::tagOrEmpty(tags, "maxspeed:forward");
    const string msb = OsmDocument::tagOrEmpty(tags, "maxspeed:backward");

    bool haveForward = false;
    bool haveBackward = false;

    if (!msf.empty()) {
        const SpeedParse parsed = parseMaxspeedValue(msf);
        if (parsed.ok) {
            speeds.forwardMs = parsed.metersPerSecond;
            haveForward = true;
            pushUnique(report.readFromTags, parsed.detail + " (forward)");
        } else {
            pushUnique(report.unsupported, parsed.detail + " (forward)");
        }
    }
    if (!msb.empty()) {
        const SpeedParse parsed = parseMaxspeedValue(msb);
        if (parsed.ok) {
            speeds.backwardMs = parsed.metersPerSecond;
            haveBackward = true;
            pushUnique(report.readFromTags, parsed.detail + " (backward)");
        } else {
            pushUnique(report.unsupported, parsed.detail + " (backward)");
        }
    }
    if (!ms.empty()) {
        const SpeedParse parsed = parseMaxspeedValue(ms);
        if (parsed.ok) {
            if (!haveForward) {
                speeds.forwardMs = parsed.metersPerSecond;
                haveForward = true;
            }
            if (!haveBackward) {
                speeds.backwardMs = parsed.metersPerSecond;
                haveBackward = true;
            }
            pushUnique(report.readFromTags, parsed.detail);
        } else {
            pushUnique(report.unsupported, parsed.detail);
        }
    }

    const bool needForward =
        oneway == OnewayPolicy::Bidirectional || oneway == OnewayPolicy::ForwardOnly;
    const bool needBackward =
        oneway == OnewayPolicy::Bidirectional || oneway == OnewayPolicy::ReverseOnly;
    if (needForward && !haveForward) {
        pushUnique(report.defaultsApplied,
                   "speedLimit default " + to_string(static_cast<int>(kDefaultMaxSpeedKmh)) +
                       " km/h (forward)");
    }
    if (needBackward && !haveBackward) {
        pushUnique(report.defaultsApplied,
                   "speedLimit default " + to_string(static_cast<int>(kDefaultMaxSpeedKmh)) +
                       " km/h (backward)");
    }
    return speeds;
}

string layerKey(const map<string, string>& tags) {
    const string layer = OsmDocument::tagOrEmpty(tags, "layer");
    if (!layer.empty()) {
        return "layer:" + layer;
    }
    const string bridge = toLowerCopy(OsmDocument::tagOrEmpty(tags, "bridge"));
    if (!bridge.empty() && bridge != "no" && bridge != "false" && bridge != "0") {
        return "bridge";
    }
    const string tunnel = toLowerCopy(OsmDocument::tagOrEmpty(tags, "tunnel"));
    if (!tunnel.empty() && tunnel != "no" && tunnel != "false" && tunnel != "0") {
        return "tunnel";
    }
    return "ground";
}

bool nodeIsCrossing(const OsmNode& node) {
    const string highway = toLowerCopy(OsmDocument::tagOrEmpty(node.tags, "highway"));
    if (highway == "crossing") {
        return true;
    }
    if (!OsmDocument::tagOrEmpty(node.tags, "crossing").empty()) {
        return true;
    }
    const string footway = toLowerCopy(OsmDocument::tagOrEmpty(node.tags, "footway"));
    return footway == "crossing";
}

bool nodeIsTrafficSignals(const OsmNode& node) {
    const string highway = toLowerCopy(OsmDocument::tagOrEmpty(node.tags, "highway"));
    if (highway == "traffic_signals") {
        return true;
    }
    return toLowerCopy(OsmDocument::tagOrEmpty(node.tags, "traffic_signals")) == "yes";
}

bool nodeIsStopOrGiveWay(const OsmNode& node) {
    const string highway = toLowerCopy(OsmDocument::tagOrEmpty(node.tags, "highway"));
    return highway == "stop" || highway == "give_way";
}

} // namespace

Vector2D OsmLocalProjection::project(double latDeg, double lonDeg) const {
    const double lat0Rad = originLatDeg * kPi / 180.0;
    const double x = (lonDeg - originLonDeg) * kMetersPerDegLonEquator * cos(lat0Rad);
    const double y = (latDeg - originLatDeg) * kMetersPerDegLat;
    return Vector2D(x, y);
}

OsmLocalProjection OsmLocalProjection::fromLatLon(double latDeg, double lonDeg) {
    OsmLocalProjection projection;
    projection.originLatDeg = latDeg;
    projection.originLonDeg = lonDeg;
    return projection;
}

OsmLocalProjection OsmLocalProjection::fromDocumentBBoxCenter(const OsmDocument& document) {
    if (document.nodes.empty()) {
        return fromLatLon(0.0, 0.0);
    }
    double minLat = 90.0;
    double maxLat = -90.0;
    double minLon = 180.0;
    double maxLon = -180.0;
    for (const auto& entry : document.nodes) {
        minLat = min(minLat, entry.second.lat);
        maxLat = max(maxLat, entry.second.lat);
        minLon = min(minLon, entry.second.lon);
        maxLon = max(maxLon, entry.second.lon);
    }
    return fromLatLon(0.5 * (minLat + maxLat), 0.5 * (minLon + maxLon));
}

double OsmToMapDescription::polylineLengthMeters(const vector<Vector2D>& points) {
    if (points.size() < 2) {
        return 0.0;
    }
    double total = 0.0;
    for (size_t i = 1; i < points.size(); ++i) {
        total += points[i - 1].distanceTo(points[i]);
    }
    return total;
}

OsmToMapResult OsmToMapDescription::convertFile(const string& path) {
    const OsmParseResult parsed = OsmJsonParser::parseFile(path);
    return convertParseResult(parsed, path);
}

OsmToMapResult OsmToMapDescription::convertParseResult(
    const OsmParseResult& parsed,
    const string& sourceFile
) {
    if (!parsed.ok) {
        OsmToMapResult result;
        result.error = parsed.error.empty() ? string("OSM parse failed") : parsed.error;
        result.report.sourceFile = sourceFile;
        result.report.unsupportedRelationIds = parsed.unsupportedRelationIds;
        result.report.missingNodeRefs = parsed.missingNodeRefs;
        return result;
    }
    return convert(
        parsed.document,
        sourceFile,
        parsed.unsupportedRelationIds,
        parsed.missingNodeRefs
    );
}

OsmToMapResult OsmToMapDescription::convert(
    const OsmDocument& document,
    const string& sourceFile,
    const vector<int64_t>& unsupportedRelationIds,
    const vector<int64_t>& missingNodeRefs
) {
    OsmToMapResult result;
    ImportReport& report = result.report;
    report.sourceFile = sourceFile;
    report.inputNodeCount = document.nodes.size();
    report.inputWayCount = document.ways.size();
    report.inputRelationCount = document.relations.size() + unsupportedRelationIds.size();
    report.unsupportedRelationIds = unsupportedRelationIds;
    report.missingNodeRefs = missingNodeRefs;

    for (const int64_t relationId : unsupportedRelationIds) {
        pushUnique(report.unsupported,
                   "relation " + to_string(relationId) +
                       " not imported (parser leftover unsupported id)");
        report.warnings.push_back(
            "unsupported relation " + to_string(relationId) +
            " — routes will NOT obey turn restrictions from this relation");
    }

    for (const int64_t missingId : missingNodeRefs) {
        pushUnique(report.skipped, "missing node ref " + to_string(missingId));
    }

    // Collect car-allowed ways, apply access hierarchy, scan remaining unsupported tags.
    struct CarWayInfo {
        const OsmWay* way = nullptr;
        CarAccessClass access = CarAccessClass::Open;
        string accessValue;
    };
    vector<CarWayInfo> carWayInfos;
    carWayInfos.reserve(document.ways.size());
    unordered_map<int64_t, CarAccessClass> wayAccessById;

    for (const OsmWay& way : document.ways) {
        const string highway = OsmDocument::tagOrEmpty(way.tags, "highway");
        if (highway.empty()) {
            pushUnique(report.skipped, "way " + to_string(way.id) + " skipped (no highway tag)");
            continue;
        }
        if (!isCarHighway(highway)) {
            pushUnique(report.skipped,
                       "way " + to_string(way.id) + " skipped (highway=" + highway +
                           " not car-allowed)");
            continue;
        }

        string accessValue;
        const CarAccessClass access = classifyCarAccess(way.tags, accessValue);

        // Report remaining unsupported tags even when the way is later skipped (e.g. private).
        for (const auto& tag : way.tags) {
            if (looksLikeUnsupportedImportantTag(tag.first)) {
                pushUnique(report.unsupported,
                           "way " + to_string(way.id) + " tag " + tag.first + "=" + tag.second +
                               " not enforced");
                report.warnings.push_back(
                    "unsupported tag " + tag.first + " on way " + to_string(way.id) +
                    " — turn:lanes/conditional not enforced");
            }
        }

        if (access == CarAccessClass::Forbidden) {
            pushUnique(report.skipped,
                       "way " + to_string(way.id) + " skipped (car access=" + accessValue + ")");
            pushUnique(report.readFromTags,
                       "way/" + to_string(way.id) + " car access hierarchy → " + accessValue +
                           " (forbidden; excluded from map)");
            continue;
        }

        CarWayInfo info;
        info.way = &way;
        info.access = access;
        info.accessValue = accessValue;
        carWayInfos.push_back(info);
        wayAccessById[way.id] = access;

        if (access == CarAccessClass::Open && !accessValue.empty()) {
            pushUnique(report.readFromTags,
                       "way/" + to_string(way.id) + " car access hierarchy → " + accessValue +
                           " (open)");
        } else if (access == CarAccessClass::DestinationOnly) {
            pushUnique(report.readFromTags,
                       "way/" + to_string(way.id) + " car access hierarchy → " + accessValue +
                           " (built; excluded from auto demand)");
        } else if (access == CarAccessClass::UnsupportedRestricted) {
            pushUnique(report.unsupported,
                       "way " + to_string(way.id) + " car access='" + accessValue +
                           "' not fully supported — excluded from auto demand");
            report.warnings.push_back(
                "unsupported/conditional access on way " + to_string(way.id) +
                " value='" + accessValue + "' — auto demand will not use this road");
        }

    }

    vector<const OsmWay*> carWays;
    carWays.reserve(carWayInfos.size());
    for (const CarWayInfo& info : carWayInfos) {
        carWays.push_back(info.way);
    }

    if (carWays.empty()) {
        result.ok = true;
        result.map.name = sourceFile.empty() ? "osm_import" : sourceFile;
        report.graphComponentsNote = "no car-allowed highways";
        report.routingLimitsNote =
            "Basic from→via(node)→to turn restrictions (no_*/only_*) are enforced when present. "
            "via-way / conditional / complex restrictions and turn:lanes remain unsupported. "
            "Car access hierarchy access→vehicle→motor_vehicle→motorcar is applied.";
        report.incompletenessNote = "empty car network";
        pushUnique(report.defaultsApplied, "PriorityMode left None (simulation → YieldToRight)");
        return result;
    }

    const OsmLocalProjection projection = OsmLocalProjection::fromDocumentBBoxCenter(document);

    // Degree: count undirected highway incidences at each node among car ways.
    // Also count distinct car ways referencing each node (for multi-way junctions).
    unordered_map<int64_t, int> nodeDegree;
    unordered_map<int64_t, set<int64_t>> nodeWayIds;
    unordered_map<int64_t, set<string>> nodeLayers;

    auto addEdgeIncidence = [&](int64_t nodeId, int64_t wayId, const string& layer) {
        nodeDegree[nodeId] += 1;
        nodeWayIds[nodeId].insert(wayId);
        nodeLayers[nodeId].insert(layer);
    };

    for (const OsmWay* wayPtr : carWays) {
        const OsmWay& way = *wayPtr;
        const string layer = layerKey(way.tags);
        for (size_t i = 0; i + 1 < way.nodeRefs.size(); ++i) {
            const int64_t a = way.nodeRefs[i];
            const int64_t b = way.nodeRefs[i + 1];
            if (document.nodes.count(a) == 0 || document.nodes.count(b) == 0) {
                continue;
            }
            addEdgeIncidence(a, way.id, layer);
            addEdgeIncidence(b, way.id, layer);
        }
        if (!way.nodeRefs.empty()) {
            nodeWayIds[way.nodeRefs.front()].insert(way.id);
            nodeWayIds[way.nodeRefs.back()].insert(way.id);
        }
    }

    auto isJunctionNode = [&](int64_t nodeId, bool isWayEndpoint) -> bool {
        if (document.nodes.count(nodeId) == 0) {
            return false;
        }
        if (isWayEndpoint) {
            return true;
        }
        if (nodeDegree[nodeId] >= 3) {
            return true;
        }
        if (nodeWayIds[nodeId].size() >= 2) {
            // Shared by multiple highways. If layers disagree, still a shared OSM node
            // (intentional join). Geometry-only crossings never share a node id.
            return true;
        }
        const OsmNode& node = document.nodes.at(nodeId);
        // traffic_signals / stop / give_way become intersections (priority tags).
        // highway=crossing alone stays on the segment (not a topology split).
        if (nodeIsTrafficSignals(node) || nodeIsStopOrGiveWay(node)) {
            return true;
        }
        return false;
    };

    // Split each car way into contiguous present-node runs, then into junction-bounded segments.
    struct RawSegment {
        int64_t wayId = 0;
        const OsmWay* way = nullptr;
        vector<int64_t> nodes; // >= 2
        int fromIndexInWay = 0;
        int toIndexInWay = 0;
        string layer;
        OnewayPolicy oneway = OnewayPolicy::Bidirectional;
        LaneCounts lanes;
        SpeedPair speeds;
    };

    vector<RawSegment> segments;
    for (const OsmWay* wayPtr : carWays) {
        const OsmWay& way = *wayPtr;
        const string layer = layerKey(way.tags);
        const OnewayPolicy oneway = interpretOneway(way.tags, report);
        const LaneCounts lanes = interpretLanes(way.tags, oneway, report);
        const SpeedPair speeds = interpretSpeeds(way.tags, oneway, report);

        // Build contiguous runs of present nodes with original indices.
        vector<pair<vector<int64_t>, vector<int>>> runs; // nodes + original indices
        vector<int64_t> currentNodes;
        vector<int> currentIdx;
        for (size_t i = 0; i < way.nodeRefs.size(); ++i) {
            const int64_t nodeId = way.nodeRefs[i];
            if (document.nodes.count(nodeId) == 0) {
                if (currentNodes.size() >= 2) {
                    runs.push_back({currentNodes, currentIdx});
                } else if (!currentNodes.empty()) {
                    pushUnique(report.skipped,
                               "way " + to_string(way.id) +
                                   " incomplete chain dropped near missing nodes");
                }
                currentNodes.clear();
                currentIdx.clear();
                continue;
            }
            currentNodes.push_back(nodeId);
            currentIdx.push_back(static_cast<int>(i));
        }
        if (currentNodes.size() >= 2) {
            runs.push_back({currentNodes, currentIdx});
        } else if (way.nodeRefs.size() >= 2) {
            pushUnique(report.skipped,
                       "way " + to_string(way.id) + " skipped (fewer than 2 resolvable nodes)");
        }

        for (const auto& run : runs) {
            const vector<int64_t>& nodes = run.first;
            const vector<int>& idxs = run.second;
            vector<size_t> splitAt;
            splitAt.push_back(0);
            for (size_t i = 1; i + 1 < nodes.size(); ++i) {
                const bool endpoint = false;
                if (isJunctionNode(nodes[i], endpoint)) {
                    splitAt.push_back(i);
                }
            }
            splitAt.push_back(nodes.size() - 1);

            for (size_t s = 0; s + 1 < splitAt.size(); ++s) {
                const size_t a = splitAt[s];
                const size_t b = splitAt[s + 1];
                if (b <= a) {
                    continue;
                }
                RawSegment seg;
                seg.wayId = way.id;
                seg.way = &way;
                seg.fromIndexInWay = idxs[a];
                seg.toIndexInWay = idxs[b];
                seg.layer = layer;
                seg.oneway = oneway;
                seg.lanes = lanes;
                seg.speeds = speeds;
                for (size_t i = a; i <= b; ++i) {
                    seg.nodes.push_back(nodes[i]);
                }
                if (seg.nodes.size() >= 2) {
                    segments.push_back(std::move(seg));
                }
            }
        }
    }

    // Intersection set = endpoints of all segments (+ tagged junctions already included).
    set<int64_t> intersectionOsmIds;
    for (const RawSegment& seg : segments) {
        intersectionOsmIds.insert(seg.nodes.front());
        intersectionOsmIds.insert(seg.nodes.back());
    }

    unordered_map<int64_t, int> osmNodeToIntersectionId;
    MapDescription map;
    map.name = sourceFile.empty() ? "osm_import" : sourceFile;

    // Defer stop/give_way approach binding until directed roads exist.
    struct PendingStopNode {
        int64_t osmId = 0;
        int intersectionId = -1;
        string direction; // empty | forward | backward | other
    };
    vector<PendingStopNode> pendingStops;

    int nextIntersectionId = 0;
    for (const int64_t osmId : intersectionOsmIds) {
        const OsmNode& node = document.nodes.at(osmId);
        MapIntersectionDesc desc;
        desc.id = nextIntersectionId++;
        desc.position = projection.project(node.lat, node.lon);
        desc.sourceRef = "node/" + to_string(osmId);
        desc.priorityMode = PriorityMode::None;

        if (nodeIsTrafficSignals(node)) {
            desc.priorityMode = PriorityMode::Signalized;
            pushUnique(report.readFromTags,
                       "node/" + to_string(osmId) + " highway=traffic_signals → Signalized");
            ++report.signalizedIntersectionCount;
        } else if (nodeIsStopOrGiveWay(node)) {
            const string direction =
                toLowerCopy(OsmDocument::tagOrEmpty(node.tags, "direction"));
            PendingStopNode pending;
            pending.osmId = osmId;
            pending.intersectionId = desc.id;
            pending.direction = direction;
            pendingStops.push_back(pending);
            // Placeholder; approach-specific binding may upgrade to PriorityRoad.
            desc.priorityMode = PriorityMode::YieldToRight;
        } else {
            pushUnique(report.defaultsApplied,
                       "PriorityMode None at node/" + to_string(osmId) +
                           " (simulation assumption → YieldToRight)");
        }

        osmNodeToIntersectionId[osmId] = desc.id;
        map.intersections.push_back(desc);
    }

    auto makeSourceRef = [](const RawSegment& seg, const char* dir) {
        ostringstream ss;
        ss << "way/" << seg.wayId << "#nodes=" << seg.fromIndexInWay << "-" << seg.toIndexInWay
           << ";dir=" << dir;
        return ss.str();
    };

    int nextRoadId = 0;
    int nextCrossingId = 0;

    struct EmittedRoadMeta {
        int roadId = -1;
        int64_t wayId = 0;
        int64_t fromOsmNode = 0;
        int64_t toOsmNode = 0;
        bool alongWayForward = true; // true = dir=fwd along way node order
    };
    vector<EmittedRoadMeta> emittedMeta;

    for (const RawSegment& seg : segments) {
        vector<Vector2D> points;
        points.reserve(seg.nodes.size());
        for (const int64_t nodeId : seg.nodes) {
            const OsmNode& node = document.nodes.at(nodeId);
            points.push_back(projection.project(node.lat, node.lon));
        }
        const double length = OsmToMapDescription::polylineLengthMeters(points);
        if (length <= 0.0) {
            pushUnique(report.skipped,
                       "way " + to_string(seg.wayId) + " segment length<=0 skipped");
            continue;
        }

        vector<Vector2D> intermediates;
        if (points.size() > 2) {
            intermediates.assign(points.begin() + 1, points.end() - 1);
        }

        const int fromId = osmNodeToIntersectionId.at(seg.nodes.front());
        const int toId = osmNodeToIntersectionId.at(seg.nodes.back());
        const int64_t fromOsm = seg.nodes.front();
        const int64_t toOsm = seg.nodes.back();

        const CarAccessClass access = [&]() {
            const auto it = wayAccessById.find(seg.wayId);
            return it == wayAccessById.end() ? CarAccessClass::Open : it->second;
        }();
        const bool allowDemand =
            access == CarAccessClass::Open;

        vector<int> emittedRoadIds;
        auto emitRoad = [&](int fromIntersection, int toIntersection, int laneCount,
                            double speedMs, const char* dir, bool reverseGeometry,
                            int64_t osmFrom, int64_t osmTo, bool alongForward) {
            MapRoadDesc road;
            road.id = nextRoadId++;
            road.fromIntersectionId = fromIntersection;
            road.toIntersectionId = toIntersection;
            road.length = length;
            road.speedLimit = speedMs;
            road.laneCount = max(1, laneCount);
            road.sourceRef = makeSourceRef(seg, dir);
            road.allowAutoDemand = allowDemand;
            if (!intermediates.empty()) {
                if (reverseGeometry) {
                    road.intermediatePoints.assign(intermediates.rbegin(), intermediates.rend());
                } else {
                    road.intermediatePoints = intermediates;
                }
            }
            emittedRoadIds.push_back(road.id);
            EmittedRoadMeta meta;
            meta.roadId = road.id;
            meta.wayId = seg.wayId;
            meta.fromOsmNode = osmFrom;
            meta.toOsmNode = osmTo;
            meta.alongWayForward = alongForward;
            emittedMeta.push_back(meta);
            map.roads.push_back(road);
            report.builtLaneCount += static_cast<size_t>(road.laneCount);
            return road.id;
        };

        if (seg.oneway == OnewayPolicy::ForwardOnly) {
            emitRoad(fromId, toId, seg.lanes.forward, seg.speeds.forwardMs, "fwd", false,
                     fromOsm, toOsm, true);
        } else if (seg.oneway == OnewayPolicy::ReverseOnly) {
            emitRoad(toId, fromId, seg.lanes.backward, seg.speeds.backwardMs, "rev", true,
                     toOsm, fromOsm, false);
        } else {
            emitRoad(fromId, toId, seg.lanes.forward, seg.speeds.forwardMs, "fwd", false,
                     fromOsm, toOsm, true);
            emitRoad(toId, fromId, seg.lanes.backward, seg.speeds.backwardMs, "rev", true,
                     toOsm, fromOsm, false);
        }

        // Place crossings for nodes on this segment onto each emitted directed road.
        for (size_t i = 0; i < seg.nodes.size(); ++i) {
            const int64_t nodeId = seg.nodes[i];
            const OsmNode& node = document.nodes.at(nodeId);
            if (!nodeIsCrossing(node)) {
                continue;
            }
            double alongFwd = 0.0;
            for (size_t k = 1; k <= i; ++k) {
                alongFwd += points[k - 1].distanceTo(points[k]);
            }
            for (const int roadId : emittedRoadIds) {
                MapRoadDesc* road = nullptr;
                for (MapRoadDesc& candidate : map.roads) {
                    if (candidate.id == roadId) {
                        road = &candidate;
                        break;
                    }
                }
                if (road == nullptr) {
                    continue;
                }
                const bool reverseDir = road->sourceRef.find(";dir=rev") != string::npos;
                const double along =
                    reverseDir ? max(0.0, road->length - alongFwd) : alongFwd;
                MapCrossingDesc crossing;
                crossing.id = nextCrossingId++;
                crossing.roadSegmentId = road->id;
                crossing.positionOnRoad = min(road->length, max(0.0, along));
                map.crossings.push_back(crossing);
                pushUnique(report.readFromTags,
                           "crossing node/" + to_string(nodeId) + " on " + road->sourceRef);
            }
        }
    }

    // Bind stop/give_way to approach when direction=forward|backward is present.
    auto findIntersectionDesc = [&](int id) -> MapIntersectionDesc* {
        for (MapIntersectionDesc& desc : map.intersections) {
            if (desc.id == id) {
                return &desc;
            }
        }
        return nullptr;
    };

    for (const PendingStopNode& pending : pendingStops) {
        MapIntersectionDesc* desc = findIntersectionDesc(pending.intersectionId);
        if (desc == nullptr) {
            continue;
        }

        vector<int> incomingRoadIds;
        for (const MapRoadDesc& road : map.roads) {
            if (road.toIntersectionId == pending.intersectionId) {
                incomingRoadIds.push_back(road.id);
            }
        }

        if (pending.direction != "forward" && pending.direction != "backward") {
            if (!pending.direction.empty()) {
                pushUnique(report.unsupported,
                           "node/" + to_string(pending.osmId) +
                               " stop/give_way direction='" + pending.direction +
                               "' not supported (need forward|backward)");
                report.warnings.push_back(
                    "stop/give_way node/" + to_string(pending.osmId) +
                    " has unsupported direction — using YieldToRight for whole junction");
            }
            desc->priorityMode = PriorityMode::YieldToRight;
            pushUnique(report.readFromTags,
                       "node/" + to_string(pending.osmId) +
                           " stop/give_way → YieldToRight (no approach-specific direction)");
            continue;
        }

        // direction=forward|backward is relative to "the" way through the node.
        // Prefer a single through-way (node not an endpoint); else a single incident way.
        // Multi-way ambiguity → YieldToRight (do not blanket invent priority from road class).
        vector<int64_t> incidentWayIds;
        vector<int64_t> throughWayIds;
        for (const OsmWay* wayPtr : carWays) {
            const OsmWay& way = *wayPtr;
            int index = -1;
            for (size_t i = 0; i < way.nodeRefs.size(); ++i) {
                if (way.nodeRefs[i] == pending.osmId) {
                    index = static_cast<int>(i);
                    break;
                }
            }
            if (index < 0) {
                continue;
            }
            incidentWayIds.push_back(way.id);
            if (index > 0 && index + 1 < static_cast<int>(way.nodeRefs.size())) {
                throughWayIds.push_back(way.id);
            }
        }

        vector<int64_t> bindWayIds;
        if (throughWayIds.size() == 1) {
            bindWayIds = throughWayIds;
        } else if (incidentWayIds.size() == 1) {
            bindWayIds = incidentWayIds;
        } else {
            desc->priorityMode = PriorityMode::YieldToRight;
            pushUnique(report.unsupported,
                       "node/" + to_string(pending.osmId) +
                           " stop/give_way direction ambiguous across multiple ways");
            report.warnings.push_back(
                "stop/give_way node/" + to_string(pending.osmId) +
                " direction cannot bind to one approach (multi-way) — YieldToRight");
            pushUnique(report.readFromTags,
                       "node/" + to_string(pending.osmId) +
                           " stop/give_way direction=" + pending.direction +
                           " → YieldToRight (ambiguous multi-way)");
            continue;
        }

        const bool wantForward = pending.direction == "forward";
        unordered_set<int> stopApproachIds;
        unordered_set<int64_t> bindSet(bindWayIds.begin(), bindWayIds.end());
        for (const EmittedRoadMeta& meta : emittedMeta) {
            if (meta.toOsmNode != pending.osmId) {
                continue;
            }
            if (bindSet.count(meta.wayId) == 0) {
                continue;
            }
            if (meta.alongWayForward == wantForward) {
                stopApproachIds.insert(meta.roadId);
            }
        }

        vector<int> priorityIds;
        for (int roadId : incomingRoadIds) {
            if (stopApproachIds.count(roadId) == 0) {
                priorityIds.push_back(roadId);
            }
        }

        if (!stopApproachIds.empty() && !priorityIds.empty()) {
            desc->priorityMode = PriorityMode::PriorityRoad;
            desc->priorityRoadIds = priorityIds;
            pushUnique(report.readFromTags,
                       "node/" + to_string(pending.osmId) + " stop/give_way direction=" +
                           pending.direction + " → PriorityRoad (stop approaches bound; "
                                               "other approaches are priority)");
        } else {
            desc->priorityMode = PriorityMode::YieldToRight;
            pushUnique(report.readFromTags,
                       "node/" + to_string(pending.osmId) +
                           " stop/give_way direction=" + pending.direction +
                           " → YieldToRight (could not bind a unique stop approach)");
        }
    }

    // Enforce supported turn restrictions: from-way → via-node → to-way (no_* / only_*).
    auto addDisallow = [&](int fromRoadId, int toRoadId) {
        if (fromRoadId < 0 || toRoadId < 0 || fromRoadId == toRoadId) {
            return;
        }
        MapRoadDesc* fromRoad = nullptr;
        MapRoadDesc* toRoad = nullptr;
        for (MapRoadDesc& road : map.roads) {
            if (road.id == fromRoadId) {
                fromRoad = &road;
            }
            if (road.id == toRoadId) {
                toRoad = &road;
            }
        }
        if (fromRoad == nullptr || toRoad == nullptr) {
            return;
        }
        if (fromRoad->toIntersectionId != toRoad->fromIntersectionId) {
            return;
        }
        for (const MapConnectionDesc& existing : map.connections) {
            if (existing.fromRoadId == fromRoadId && existing.toRoadId == toRoadId &&
                !existing.allowed) {
                return;
            }
        }
        MapConnectionDesc conn;
        conn.fromRoadId = fromRoadId;
        conn.toRoadId = toRoadId;
        conn.allowed = false;
        map.connections.push_back(conn);
    };

    auto roadsEndingAtVia = [&](int64_t wayId, int64_t viaNode) {
        vector<int> ids;
        for (const EmittedRoadMeta& meta : emittedMeta) {
            if (meta.wayId == wayId && meta.toOsmNode == viaNode) {
                ids.push_back(meta.roadId);
            }
        }
        return ids;
    };
    auto roadsStartingAtVia = [&](int64_t wayId, int64_t viaNode) {
        vector<int> ids;
        for (const EmittedRoadMeta& meta : emittedMeta) {
            if (meta.wayId == wayId && meta.fromOsmNode == viaNode) {
                ids.push_back(meta.roadId);
            }
        }
        return ids;
    };
    auto geometricOutgoing = [&](int fromRoadId) {
        vector<int> ids;
        const MapRoadDesc* fromRoad = nullptr;
        for (const MapRoadDesc& road : map.roads) {
            if (road.id == fromRoadId) {
                fromRoad = &road;
                break;
            }
        }
        if (fromRoad == nullptr) {
            return ids;
        }
        for (const MapRoadDesc& road : map.roads) {
            if (road.fromIntersectionId == fromRoad->toIntersectionId &&
                road.id != fromRoadId) {
                ids.push_back(road.id);
            }
        }
        return ids;
    };

    for (const OsmRelation& relation : document.relations) {
        const string type = toLowerCopy(OsmDocument::tagOrEmpty(relation.tags, "type"));
        if (type != "restriction") {
            report.unsupportedRelationIds.push_back(relation.id);
            pushUnique(report.unsupported,
                       "relation " + to_string(relation.id) + " type=" + type +
                           " not enforced");
            report.warnings.push_back(
                "unsupported relation " + to_string(relation.id) +
                " — not a basic turn restriction");
            continue;
        }

        bool hasConditional = false;
        for (const auto& tag : relation.tags) {
            if (tag.first.find("conditional") != string::npos) {
                hasConditional = true;
                break;
            }
        }
        const string restriction =
            toLowerCopy(OsmDocument::tagOrEmpty(relation.tags, "restriction"));
        if (hasConditional || restriction.empty()) {
            report.unsupportedRelationIds.push_back(relation.id);
            pushUnique(report.unsupported,
                       "relation " + to_string(relation.id) +
                           " conditional/complex restriction not enforced");
            report.warnings.push_back(
                "unsupported restriction relation " + to_string(relation.id) +
                " — conditional/complex; routes will NOT obey it");
            continue;
        }

        const string except = OsmDocument::tagOrEmpty(relation.tags, "except");
        if (!except.empty() && exceptExemptsCars(except)) {
            pushUnique(report.readFromTags,
                       "relation/" + to_string(relation.id) + " except=" + except +
                           " exempts cars — restriction not applied");
            continue;
        }

        vector<int64_t> fromWays;
        vector<int64_t> toWays;
        vector<int64_t> viaNodes;
        vector<int64_t> viaWays;
        for (const OsmRelationMember& member : relation.members) {
            const string role = toLowerCopy(member.role);
            const string mtype = toLowerCopy(member.type);
            if (role == "from" && mtype == "way") {
                fromWays.push_back(member.ref);
            } else if (role == "to" && mtype == "way") {
                toWays.push_back(member.ref);
            } else if (role == "via" && mtype == "node") {
                viaNodes.push_back(member.ref);
            } else if (role == "via" && mtype == "way") {
                viaWays.push_back(member.ref);
            }
        }

        if (!viaWays.empty() || viaNodes.size() != 1 || fromWays.size() != 1 ||
            toWays.size() != 1) {
            report.unsupportedRelationIds.push_back(relation.id);
            pushUnique(report.unsupported,
                       "relation " + to_string(relation.id) +
                           " via-way/multi-member restriction not enforced");
            report.warnings.push_back(
                "unsupported restriction relation " + to_string(relation.id) +
                " — need single from-way, via-node, to-way (no via-way)");
            continue;
        }

        if (!isSupportedNoRestriction(restriction) &&
            !isSupportedOnlyRestriction(restriction)) {
            report.unsupportedRelationIds.push_back(relation.id);
            pushUnique(report.unsupported,
                       "relation " + to_string(relation.id) + " restriction=" + restriction +
                           " not enforced");
            report.warnings.push_back(
                "unsupported restriction='" + restriction + "' on relation " +
                to_string(relation.id));
            continue;
        }

        const int64_t fromWay = fromWays.front();
        const int64_t toWay = toWays.front();
        const int64_t viaNode = viaNodes.front();
        if (osmNodeToIntersectionId.count(viaNode) == 0) {
            report.unsupportedRelationIds.push_back(relation.id);
            pushUnique(report.unsupported,
                       "relation " + to_string(relation.id) +
                           " via node not an imported intersection");
            report.warnings.push_back(
                "restriction relation " + to_string(relation.id) +
                " via node missing from car network — not enforced");
            continue;
        }

        const vector<int> fromRoadIds = roadsEndingAtVia(fromWay, viaNode);
        const vector<int> toRoadIds = roadsStartingAtVia(toWay, viaNode);
        if (fromRoadIds.empty() || toRoadIds.empty()) {
            report.unsupportedRelationIds.push_back(relation.id);
            pushUnique(report.unsupported,
                       "relation " + to_string(relation.id) +
                           " could not map from/to ways onto directed segments at via");
            report.warnings.push_back(
                "restriction relation " + to_string(relation.id) +
                " — no matching directed segments at via after way split");
            continue;
        }

        unordered_set<int> allowedTo(toRoadIds.begin(), toRoadIds.end());
        if (isSupportedOnlyRestriction(restriction)) {
            for (int fromId : fromRoadIds) {
                for (int outId : geometricOutgoing(fromId)) {
                    if (allowedTo.count(outId) == 0) {
                        addDisallow(fromId, outId);
                    }
                }
            }
            pushUnique(report.readFromTags,
                       "relation/" + to_string(relation.id) + " " + restriction +
                           " enforced (only_* → disallowed other exits at via)");
        } else {
            for (int fromId : fromRoadIds) {
                for (int toId : toRoadIds) {
                    addDisallow(fromId, toId);
                }
            }
            pushUnique(report.readFromTags,
                       "relation/" + to_string(relation.id) + " " + restriction +
                           " enforced (no_* → disallowed from→to at via)");
        }
    }

    // Graph components (undirected over intersection adjacency via roads).
    unordered_map<int, vector<int>> adjacency;
    for (const MapRoadDesc& road : map.roads) {
        adjacency[road.fromIntersectionId].push_back(road.toIntersectionId);
        adjacency[road.toIntersectionId].push_back(road.fromIntersectionId);
    }
    unordered_set<int> visited;
    size_t components = 0;
    size_t danglingEnds = 0;
    for (const MapIntersectionDesc& intersection : map.intersections) {
        set<int> neighbors;
        for (const int next : adjacency[intersection.id]) {
            neighbors.insert(next);
        }
        if (neighbors.size() <= 1) {
            ++danglingEnds;
        }
    }
    for (const MapRoadDesc& road : map.roads) {
        const int seed = road.fromIntersectionId;
        if (visited.count(seed)) {
            continue;
        }
        ++components;
        vector<int> stack = {seed};
        visited.insert(seed);
        while (!stack.empty()) {
            const int current = stack.back();
            stack.pop_back();
            for (const int next : adjacency[current]) {
                if (visited.insert(next).second) {
                    stack.push_back(next);
                }
            }
        }
    }

    report.builtIntersectionCount = map.intersections.size();
    report.builtRoadCount = map.roads.size();
    report.builtCrossingCount = map.crossings.size();
    report.graphComponentCount = components;
    {
        ostringstream note;
        note << components << " connected component(s); " << danglingEnds
             << " dangling end(s) kept as entries/exits (no fake links outside data)";
        report.graphComponentsNote = note.str();
    }
    report.routingLimitsNote =
        "Enforced: basic restriction relations (from-way → via-node → to-way, no_*/only_*) "
        "as MapConnectionDesc disallows; car access hierarchy "
        "access→vehicle→motor_vehicle→motorcar (no/private excluded; destination/conditional "
        "excluded from auto demand). Still unsupported: via-way / conditional / complex "
        "restrictions, turn:lanes*. Crossings without a shared OSM node are never joined. "
        "layer/bridge/tunnel do not create proximity joins.";
    if (!missingNodeRefs.empty()) {
        report.incompletenessNote =
            "Missing node refs present; incomplete ways were split/skipped. "
            "Bbox clips may leave dangling ends.";
    } else if (components > 1) {
        report.incompletenessNote =
            "Multiple disconnected network components — likely incomplete extract or disjoint streets.";
    } else {
        report.incompletenessNote = "Network imported without inventing external links.";
    }

    // Note multi-layer shared nodes for awareness (still connected via shared node id).
    for (const auto& entry : nodeLayers) {
        if (entry.second.size() > 1) {
            pushUnique(report.warnings,
                       "node/" + to_string(entry.first) +
                           " shared across layers/bridge/tunnel tags — kept as OSM shared node join");
        }
    }

    result.ok = true;
    result.map = std::move(map);
    return result;
}
