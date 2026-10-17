#include "osm/OsmDocument.h"
#include "osm/OsmJsonParser.h"

#include <filesystem>
#include <iostream>
#include <string>
#include <system_error>

using namespace std;
namespace fs = std::filesystem;

namespace {

int g_failures = 0;
int g_passed = 0;

void expect(bool condition, const string& name) {
    if (condition) {
        ++g_passed;
        cout << "PASS  " << name << "\n";
    } else {
        ++g_failures;
        cout << "FAIL  " << name << "\n";
    }
}

string fixturePath(const string& name) {
    return string("data/osm/fixtures/") + name;
}

void testWarsawSampleSmokeIfPresent() {
    const string path = "data/osm/warsaw_sample.overpass.json";
    if (!fs::exists(path)) {
        cout << "SKIP  osm_warsaw_smoke (sample file missing)\n";
        return;
    }

    const OsmParseResult result = OsmJsonParser::parseFile(path);
    expect(result.ok, "osm_warsaw_ok");
    expect(result.error.empty(), "osm_warsaw_no_error");
    expect(!result.document.nodes.empty(), "osm_warsaw_nodes_gt0");
    expect(!result.document.ways.empty(), "osm_warsaw_ways_gt0");
    expect(result.document.nodes.size() > 0 && result.document.ways.size() > 0,
           "osm_warsaw_counts_positive");
}

void testEmptyElementsFails() {
    const OsmParseResult result = OsmJsonParser::parseFile(fixturePath("empty.overpass.json"));
    expect(!result.ok, "osm_empty_fails");
    expect(!result.error.empty(), "osm_empty_has_error");
}

void testInvalidJsonFails() {
    const OsmParseResult result = OsmJsonParser::parseFile(fixturePath("invalid_not_json.txt"));
    expect(!result.ok, "osm_invalid_json_fails");
    expect(!result.error.empty(), "osm_invalid_json_has_error");
}

void testMissingElementsFails() {
    const OsmParseResult result =
        OsmJsonParser::parseFile(fixturePath("invalid_no_elements.overpass.json"));
    expect(!result.ok, "osm_missing_elements_fails");
    expect(!result.error.empty(), "osm_missing_elements_has_error");
}

void testMissingFileFails() {
    const OsmParseResult result =
        OsmJsonParser::parseFile("data/osm/fixtures/does_not_exist.overpass.json");
    expect(!result.ok, "osm_missing_file_fails");
    expect(!result.error.empty(), "osm_missing_file_has_error");
}

void testOnewayTags() {
    const OsmParseResult result = OsmJsonParser::parseFile(fixturePath("oneway.overpass.json"));
    expect(result.ok, "osm_oneway_ok");
    expect(result.document.ways.size() == 1, "osm_oneway_one_way");
    expect(result.document.nodes.size() == 2, "osm_oneway_two_nodes");
    if (!result.document.ways.empty()) {
        const OsmWay& way = result.document.ways.front();
        expect(way.id == 800001, "osm_oneway_way_id");
        expect(OsmDocument::tagOrEmpty(way.tags, "oneway") == "yes", "osm_oneway_tag_yes");
        expect(OsmDocument::tagOrEmpty(way.tags, "highway") == "residential",
               "osm_oneway_highway");
        expect(OsmDocument::tagOrEmpty(way.tags, "maxspeed") == "50", "osm_oneway_maxspeed");
    } else {
        expect(false, "osm_oneway_tag_yes");
        expect(false, "osm_oneway_highway");
        expect(false, "osm_oneway_maxspeed");
    }
}

void testOnewayReverseTags() {
    const OsmParseResult result =
        OsmJsonParser::parseFile(fixturePath("oneway_reverse.overpass.json"));
    expect(result.ok, "osm_oneway_rev_ok");
    expect(result.document.ways.size() == 1, "osm_oneway_rev_one_way");
    if (!result.document.ways.empty()) {
        const OsmWay& way = result.document.ways.front();
        expect(OsmDocument::tagOrEmpty(way.tags, "oneway") == "-1", "osm_oneway_rev_tag");
        expect(OsmDocument::tagOrEmpty(way.tags, "maxspeed") == "30", "osm_oneway_rev_maxspeed");
    } else {
        expect(false, "osm_oneway_rev_tag");
        expect(false, "osm_oneway_rev_maxspeed");
    }
}

void testTwowayStub() {
    const OsmParseResult result = OsmJsonParser::parseFile(fixturePath("twoway_stub.overpass.json"));
    expect(result.ok, "osm_twoway_ok");
    expect(result.document.ways.size() == 1, "osm_twoway_one_way");
    if (!result.document.ways.empty()) {
        const OsmWay& way = result.document.ways.front();
        expect(OsmDocument::tagOrEmpty(way.tags, "oneway").empty(), "osm_twoway_no_oneway");
        expect(OsmDocument::tagOrEmpty(way.tags, "lanes") == "2", "osm_twoway_lanes");
    } else {
        expect(false, "osm_twoway_no_oneway");
        expect(false, "osm_twoway_lanes");
    }
}

void testMissingNodeRefsReported() {
    const OsmParseResult result =
        OsmJsonParser::parseFile(fixturePath("missing_node_ref.overpass.json"));
    expect(result.ok, "osm_missing_ref_parses");
    expect(result.document.ways.size() == 1, "osm_missing_ref_one_way");
    expect(result.document.nodes.size() == 1, "osm_missing_ref_one_node");
    expect(result.missingNodeRefs.size() == 1, "osm_missing_ref_one_missing");
    if (!result.missingNodeRefs.empty()) {
        expect(result.missingNodeRefs.front() == 999999, "osm_missing_ref_id");
    } else {
        expect(false, "osm_missing_ref_id");
    }
}

void testRelationStoredOnDocument() {
    const OsmParseResult result =
        OsmJsonParser::parseFile(fixturePath("with_relation.overpass.json"));
    expect(result.ok, "osm_relation_ok");
    expect(result.document.ways.size() == 1, "osm_relation_way_kept");
    expect(result.document.nodes.size() == 2, "osm_relation_nodes_kept");
    expect(result.document.relations.size() == 1, "osm_relation_stored");
    expect(result.unsupportedRelationIds.empty(), "osm_relation_parser_ids_empty");
    if (!result.document.relations.empty()) {
        expect(result.document.relations.front().id == 700001, "osm_relation_stored_id");
        expect(result.document.relations.front().members.size() == 1, "osm_relation_members");
    } else {
        expect(false, "osm_relation_stored_id");
        expect(false, "osm_relation_members");
    }
}

void testNonAsciiPathLoad() {
    const fs::path src = fixturePath("twoway_stub.overpass.json");
    expect(fs::exists(src), "osm_nonascii_src_exists");

    const fs::path dir = fs::temp_directory_path() / u8"EasyRider OSM Śródmieście";
    const fs::path dest = dir / u8"próbka warszawa.overpass.json";
    error_code ec;
    fs::create_directories(dir, ec);
    fs::copy_file(src, dest, fs::copy_options::overwrite_existing, ec);
    expect(!ec, "osm_nonascii_copy_ok");

    // UTF-8 path string (as GUI wide→UTF-8 would produce).
    const string pathUtf8 = dest.u8string();
    const OsmParseResult result = OsmJsonParser::parseFile(pathUtf8);
    expect(result.ok, "osm_nonascii_parse_ok");
    expect(result.error.empty(), "osm_nonascii_no_error");
    expect(!result.document.ways.empty(), "osm_nonascii_has_ways");

    fs::remove_all(dir, ec);
}

void testParseStringDirect() {
    const string json =
        R"({"elements":[{"type":"node","id":1,"lat":1.0,"lon":2.0},)"
        R"({"type":"way","id":2,"nodes":[1],"tags":{"highway":"service"}}]})";
    const OsmParseResult result = OsmJsonParser::parseString(json);
    expect(result.ok, "osm_parse_string_ok");
    expect(result.document.nodes.count(1) == 1, "osm_parse_string_node");
    expect(result.document.ways.size() == 1, "osm_parse_string_way");
}

} // namespace

int runOsmJsonParserTests() {
    cout << "\n--- OSM JSON parser tests ---\n";
    testWarsawSampleSmokeIfPresent();
    testEmptyElementsFails();
    testInvalidJsonFails();
    testMissingElementsFails();
    testMissingFileFails();
    testOnewayTags();
    testOnewayReverseTags();
    testTwowayStub();
    testMissingNodeRefsReported();
    testRelationStoredOnDocument();
    testNonAsciiPathLoad();
    testParseStringDirect();
    return g_failures;
}

int osmJsonParserPassedCount() {
    return g_passed;
}
