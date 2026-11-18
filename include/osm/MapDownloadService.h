#pragma once

#include "osm/DownloadCache.h"
#include "osm/GeoBbox.h"
#include "osm/HttpTransport.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace osm_net {

struct DownloadEndpoints {
    // Configurable; defaults are public Nominatim + Overpass.
    std::string nominatimSearchUrl = "https://nominatim.openstreetmap.org/search";
    std::string overpassInterpreterUrl = "https://overpass-api.de/api/interpreter";

    // Optional contact URL/email if a service requires identification beyond User-Agent.
    // Leave empty by default — documented placeholder in RUN.md / OSM_IMPORT.md.
    // Coordinator: fill before heavy public use if Nominatim policy demands it.
    std::string contactPlaceholder;
};

struct PlaceHit {
    std::string displayName;
    double lat = 0.0;
    double lon = 0.0;
    std::string osmType; // node|way|relation
    std::int64_t osmId = 0;
};

struct GeocodeResult {
    bool ok = false;
    bool fromCache = false;
    bool cancelled = false;
    std::string error;
    std::vector<PlaceHit> hits;
    std::string cachePath;
};

struct MapFetchResult {
    bool ok = false;
    bool fromCache = false;
    bool cancelled = false;
    bool sizeExceeded = false;
    std::string error;
    std::string cachePath;       // Overpass JSON on disk
    GeoBbox bbox;
    double radiusMeters = 0.0;
    std::size_t responseBytes = 0;
};

struct MapDownloadLimits {
    std::size_t maxGeocodeBytes = kDefaultMaxGeocodeBytes;
    std::size_t maxOverpassBytes = kDefaultMaxOverpassBytes;
    int httpTimeoutMs = kDefaultHttpTimeoutMs;
    int maxRetries = kDefaultMaxRetries;
    int maxGeocodeResults = 8;
};

using ProgressFn = std::function<void(const std::string& phaseMessage)>;

class MapDownloadService {
public:
    MapDownloadService(
        IHttpTransport& transport,
        DownloadCache cache = DownloadCache{},
        DownloadEndpoints endpoints = {},
        MapDownloadLimits limits = {}
    );

    const DownloadEndpoints& endpoints() const { return endpoints_; }
    const MapDownloadLimits& limits() const { return limits_; }
    DownloadCache& cache() { return cache_; }

    GeocodeResult searchPlaces(
        const std::string& placeQuery,
        bool forceRefresh = false,
        std::atomic<bool>* cancelFlag = nullptr,
        ProgressFn progress = nullptr
    );

    MapFetchResult downloadRoads(
        double latDeg,
        double lonDeg,
        double radiusMeters,
        bool forceRefresh = false,
        std::atomic<bool>* cancelFlag = nullptr,
        ProgressFn progress = nullptr
    );

    // Download by explicit rectangular bbox (pin+size or drawn rectangle).
    // Shown selection, Overpass query, and cache key all use the same GeoBbox.
    MapFetchResult downloadRoadsForBbox(
        const GeoBbox& bbox,
        bool forceRefresh = false,
        std::atomic<bool>* cancelFlag = nullptr,
        ProgressFn progress = nullptr
    );

    // Parse Nominatim JSON array (also used by tests / cache replay).
    static GeocodeResult parseNominatimJson(const std::string& jsonText, int maxResults);

    // Lightweight Overpass payload check (elements array present; not truncated).
    static bool looksLikeOverpassJson(const std::string& text, std::string& errorOut);

private:
    HttpResponse sendWithLimitedRetries(
        const HttpRequest& request,
        std::atomic<bool>* cancelFlag,
        ProgressFn progress
    );

    IHttpTransport& transport_;
    DownloadCache cache_;
    DownloadEndpoints endpoints_;
    MapDownloadLimits limits_;
};

} // namespace osm_net
