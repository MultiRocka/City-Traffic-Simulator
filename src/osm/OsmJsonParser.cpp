#include "osm/OsmJsonParser.h"

#include <exception>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <set>
#include <sstream>

using json = nlohmann::json;
namespace fs = std::filesystem;

namespace {

string readEntireFile(const string& path, string& error) {
#ifdef _WIN32
    // Paths arrive as UTF-8 (GUI wide→UTF-8). Opening via u8path uses UTF-16 on Windows
    // so Polish characters and spaces in directory/filename work (ANSI ifstream does not).
    ifstream in(fs::u8path(path), ios::binary);
#else
    ifstream in(path, ios::binary);
#endif
    if (!in) {
        error = "cannot open file: " + path;
        return {};
    }
    ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

map<string, string> parseTags(const json& element) {
    map<string, string> tags;
    if (!element.contains("tags") || !element["tags"].is_object()) {
        return tags;
    }
    for (auto it = element["tags"].begin(); it != element["tags"].end(); ++it) {
        if (it.value().is_string()) {
            tags[it.key()] = it.value().get<string>();
        } else if (it.value().is_number_integer()) {
            tags[it.key()] = to_string(it.value().get<int64_t>());
        } else if (it.value().is_number_float()) {
            tags[it.key()] = to_string(it.value().get<double>());
        } else if (it.value().is_boolean()) {
            tags[it.key()] = it.value().get<bool>() ? "true" : "false";
        }
    }
    return tags;
}

} // namespace

OsmParseResult OsmJsonParser::parseFile(const string& path) {
    string ioError;
    const string text = readEntireFile(path, ioError);
    if (!ioError.empty()) {
        OsmParseResult result;
        result.error = ioError;
        return result;
    }
    return parseString(text);
}

OsmParseResult OsmJsonParser::parseString(const string& jsonText) {
    OsmParseResult result;

    json root;
    try {
        root = json::parse(jsonText);
    } catch (const json::parse_error& e) {
        result.error = string("JSON parse error: ") + e.what();
        return result;
    } catch (const exception& e) {
        result.error = string("JSON parse error: ") + e.what();
        return result;
    }

    if (!root.is_object()) {
        result.error = "root JSON value is not an object";
        return result;
    }
    if (!root.contains("elements") || !root["elements"].is_array()) {
        result.error = "missing or invalid 'elements' array";
        return result;
    }

    const json& elements = root["elements"];
    if (elements.empty()) {
        result.error = "empty 'elements' array";
        return result;
    }

    if (root.contains("generator") && root["generator"].is_string()) {
        result.document.generator = root["generator"].get<string>();
    }
    if (root.contains("osm3s") && root["osm3s"].is_object()) {
        const json& osm3s = root["osm3s"];
        if (osm3s.contains("timestamp_osm_base") && osm3s["timestamp_osm_base"].is_string()) {
            result.document.osmBaseTimestamp = osm3s["timestamp_osm_base"].get<string>();
        }
    }

    set<int64_t> seenMissingRefs;

    try {
        for (const json& el : elements) {
            if (!el.is_object() || !el.contains("type") || !el["type"].is_string()) {
                result.error = "element missing string 'type'";
                return result;
            }
            const string type = el["type"].get<string>();

            if (type == "relation") {
                OsmRelation relation;
                if (el.contains("id") && el["id"].is_number_integer()) {
                    relation.id = el["id"].get<int64_t>();
                }
                relation.tags = parseTags(el);
                if (el.contains("members") && el["members"].is_array()) {
                    for (const json& member : el["members"]) {
                        if (!member.is_object()) {
                            continue;
                        }
                        OsmRelationMember m;
                        if (member.contains("type") && member["type"].is_string()) {
                            m.type = member["type"].get<string>();
                        }
                        if (member.contains("ref") && member["ref"].is_number_integer()) {
                            m.ref = member["ref"].get<int64_t>();
                        }
                        if (member.contains("role") && member["role"].is_string()) {
                            m.role = member["role"].get<string>();
                        }
                        relation.members.push_back(move(m));
                    }
                }
                result.document.relations.push_back(move(relation));
                continue;
            }

            if (type == "node") {
                if (!el.contains("id") || !el["id"].is_number_integer()) {
                    result.error = "node missing integer 'id'";
                    return result;
                }
                if (!el.contains("lat") || !el["lat"].is_number() ||
                    !el.contains("lon") || !el["lon"].is_number()) {
                    result.error = "node missing numeric lat/lon";
                    return result;
                }
                OsmNode node;
                node.id = el["id"].get<int64_t>();
                node.lat = el["lat"].get<double>();
                node.lon = el["lon"].get<double>();
                node.tags = parseTags(el);
                result.document.nodes[node.id] = move(node);
                continue;
            }

            if (type == "way") {
                if (!el.contains("id") || !el["id"].is_number_integer()) {
                    result.error = "way missing integer 'id'";
                    return result;
                }
                if (!el.contains("nodes") || !el["nodes"].is_array()) {
                    result.error = "way missing 'nodes' array";
                    return result;
                }
                OsmWay way;
                way.id = el["id"].get<int64_t>();
                way.tags = parseTags(el);
                for (const json& ref : el["nodes"]) {
                    if (!ref.is_number_integer()) {
                        result.error = "way has non-integer node ref";
                        return result;
                    }
                    way.nodeRefs.push_back(ref.get<int64_t>());
                }
                result.document.ways.push_back(move(way));
                continue;
            }

            // Unknown element types are ignored (not fatal).
        }
    } catch (const exception& e) {
        result.error = string("element parse error: ") + e.what();
        result.document = OsmDocument{};
        result.unsupportedRelationIds.clear();
        return result;
    }

    for (const OsmWay& way : result.document.ways) {
        for (int64_t ref : way.nodeRefs) {
            if (result.document.nodes.find(ref) == result.document.nodes.end()) {
                if (seenMissingRefs.insert(ref).second) {
                    result.missingNodeRefs.push_back(ref);
                }
            }
        }
    }

    result.ok = true;
    return result;
}
