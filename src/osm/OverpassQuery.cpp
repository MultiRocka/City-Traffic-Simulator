#include "osm/OverpassQuery.h"

#include <sstream>

namespace osm_net {

std::string buildRoadNetworkQuery(const GeoBbox& bbox, const OverpassQueryOptions& options) {
    std::ostringstream q;
    q << "[out:json][timeout:" << options.timeoutSeconds << "];\n";
    q << "(\n";
    q << "  way[\"highway\"~\"" << kHighwayRegex << "\"]("
      << bbox.toOverpassBBox() << ");\n";
    q << ")->.roads;\n";
    q << ".roads out body;\n";
    q << ".roads >;\n";
    q << "out skel qt;\n";
    q << "rel(bw.roads)[\"type\"=\"restriction\"];\n";
    q << "out body;\n";
    return q.str();
}

} // namespace osm_net
