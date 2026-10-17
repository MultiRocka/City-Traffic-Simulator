#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

using namespace std;

// Raw Overpass JSON input model (Stage 2). Not MapDescription / City.
// IDs are OSM int64 values (real sample ids and synthetic fixture ids).

struct OsmNode {
    int64_t id = 0;
    double lat = 0.0;
    double lon = 0.0;
    map<string, string> tags;
};

struct OsmWay {
    int64_t id = 0;
    vector<int64_t> nodeRefs;
    map<string, string> tags;
};

struct OsmRelationMember {
    string type; // "node" | "way" | "relation"
    int64_t ref = 0;
    string role;
};

struct OsmRelation {
    int64_t id = 0;
    vector<OsmRelationMember> members;
    map<string, string> tags;
};

struct OsmDocument {
    string generator;
    string osmBaseTimestamp;
    map<int64_t, OsmNode> nodes;
    vector<OsmWay> ways;
    vector<OsmRelation> relations;

    // Convenience for Stage 3 tag mapping.
    static string tagOrEmpty(const map<string, string>& tags, const string& key) {
        const auto it = tags.find(key);
        return it == tags.end() ? string() : it->second;
    }
};
