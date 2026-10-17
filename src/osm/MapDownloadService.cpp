#include "osm/MapDownloadService.h"

#include "osm/OverpassQuery.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <nlohmann/json.hpp>
#include <sstream>
#include <thread>

using json = nlohmann::json;

namespace osm_net {
namespace {

std::string urlEncode(const std::string& value) {
    std::ostringstream escaped;
    escaped.fill('0');
    escaped << std::hex;
    for (unsigned char c : value) {
        if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9')
            || c == '-' || c == '_' || c == '.' || c == '~') {
            escaped << c;
        } else if (c == ' ') {
            escaped << '+';
        } else {
            escaped << '%' << std::uppercase << std::setw(2) << static_cast<int>(c) << std::nouppercase;
        }
    }
    return escaped.str();
}

bool isRetryableStatus(int status) {
    return status == 429 || status == 502 || status == 503 || status == 504;
}

} // namespace

MapDownloadService::MapDownloadService(
    IHttpTransport& transport,
    DownloadCache cache,
    DownloadEndpoints endpoints,
    MapDownloadLimits limits
)
    : transport_(transport),
      cache_(std::move(cache)),
      endpoints_(std::move(endpoints)),
      limits_(limits) {
}

HttpResponse MapDownloadService::sendWithLimitedRetries(
    const HttpRequest& request,
    std::atomic<bool>* cancelFlag,
    ProgressFn progress
) {
    HttpResponse last;
    const int attempts = 1 + std::max(0, limits_.maxRetries);
    for (int attempt = 0; attempt < attempts; ++attempt) {
        if (cancelFlag != nullptr && cancelFlag->load()) {
            last.cancelled = true;
            last.error = "Cancelled";
            return last;
        }
        if (progress && attempt > 0) {
            progress("Ponawianie żądania HTTP…");
        }
        last = transport_.send(request, cancelFlag);
        if (last.cancelled || last.timedOut || last.sizeExceeded || last.ok) {
            return last;
        }
        if (!isRetryableStatus(last.statusCode) || attempt + 1 >= attempts) {
            return last;
        }
        // Short backoff — no parallel spam.
        std::this_thread::sleep_for(std::chrono::milliseconds(400 * (attempt + 1)));
    }
    return last;
}

GeocodeResult MapDownloadService::parseNominatimJson(const std::string& jsonText, int maxResults) {
    GeocodeResult result;
    json root;
    try {
        root = json::parse(jsonText);
    } catch (const std::exception& e) {
        result.error = std::string("Nominatim JSON parse error: ") + e.what();
        return result;
    }

    if (!root.is_array()) {
        result.error = "Nominatim response is not a JSON array";
        return result;
    }

    const int limit = std::max(1, maxResults);
    for (const auto& item : root) {
        if (static_cast<int>(result.hits.size()) >= limit) {
            break;
        }
        if (!item.is_object()) {
            continue;
        }
        PlaceHit hit;
        if (item.contains("display_name") && item["display_name"].is_string()) {
            hit.displayName = item["display_name"].get<std::string>();
        }
        try {
            if (item.contains("lat")) {
                if (item["lat"].is_string()) {
                    hit.lat = std::stod(item["lat"].get<std::string>());
                } else if (item["lat"].is_number()) {
                    hit.lat = item["lat"].get<double>();
                }
            }
            if (item.contains("lon")) {
                if (item["lon"].is_string()) {
                    hit.lon = std::stod(item["lon"].get<std::string>());
                } else if (item["lon"].is_number()) {
                    hit.lon = item["lon"].get<double>();
                }
            }
        } catch (...) {
            continue;
        }
        if (item.contains("osm_type") && item["osm_type"].is_string()) {
            hit.osmType = item["osm_type"].get<std::string>();
        }
        if (item.contains("osm_id") && item["osm_id"].is_number_integer()) {
            hit.osmId = item["osm_id"].get<std::int64_t>();
        }
        if (hit.displayName.empty() || !std::isfinite(hit.lat) || !std::isfinite(hit.lon)) {
            continue;
        }
        result.hits.push_back(std::move(hit));
    }

    result.ok = true;
    return result;
}

bool MapDownloadService::looksLikeOverpassJson(const std::string& text, std::string& errorOut) {
    if (text.empty()) {
        errorOut = "Empty Overpass response";
        return false;
    }
    const auto firstNonSpace = text.find_first_not_of(" \t\r\n");
    if (firstNonSpace != std::string::npos && text[firstNonSpace] == '<') {
        errorOut = "Overpass returned HTML instead of JSON";
        return false;
    }
    json root;
    try {
        root = json::parse(text);
    } catch (const std::exception& e) {
        errorOut = std::string("Overpass JSON parse error: ") + e.what();
        return false;
    }
    if (!root.is_object() || !root.contains("elements") || !root["elements"].is_array()) {
        errorOut = "Overpass JSON missing 'elements' array";
        return false;
    }
    if (root["elements"].empty()) {
        errorOut = "Overpass JSON has empty 'elements' (no roads in this area?)";
        return false;
    }
    return true;
}

GeocodeResult MapDownloadService::searchPlaces(
    const std::string& placeQuery,
    bool forceRefresh,
    std::atomic<bool>* cancelFlag,
    ProgressFn progress
) {
    GeocodeResult result;
    const std::string normalized = DownloadCache::normalizeQuery(placeQuery);
    if (normalized.empty()) {
        result.error = "Wpisz nazwę miejsca.";
        return result;
    }

    result.cachePath = cache_.geocodePath(normalized);

    if (!forceRefresh && cache_.exists(result.cachePath)) {
        if (progress) {
            progress("Odczyt cache geokodowania…");
        }
        std::string body;
        std::string err;
        if (cache_.tryRead(result.cachePath, body, err)) {
            GeocodeResult parsed = parseNominatimJson(body, limits_.maxGeocodeResults);
            if (parsed.ok) {
                parsed.fromCache = true;
                parsed.cachePath = result.cachePath;
                return parsed;
            }
        }
        // Corrupt cache → fall through to network.
    }

    if (progress) {
        progress("Szukanie miejsca (Nominatim)…");
    }

    std::ostringstream url;
    url << endpoints_.nominatimSearchUrl
        << "?q=" << urlEncode(normalized)
        << "&format=json&limit=" << limits_.maxGeocodeResults
        << "&addressdetails=0";

    HttpRequest req;
    req.method = "GET";
    req.url = url.str();
    req.userAgent = kEasyRiderUserAgent;
    req.timeoutMs = limits_.httpTimeoutMs;
    req.maxResponseBytes = limits_.maxGeocodeBytes;

    const HttpResponse http = sendWithLimitedRetries(req, cancelFlag, progress);
    if (http.cancelled) {
        result.cancelled = true;
        result.error = "Anulowano.";
        return result;
    }
    if (http.timedOut) {
        result.error = "Limit czasu geokodowania.";
        return result;
    }
    if (http.sizeExceeded) {
        result.error = "Odpowiedź geokodowania przekroczyła limit rozmiaru.";
        return result;
    }
    if (!http.ok) {
        result.error = http.error.empty() ? "Błąd HTTP geokodowania." : http.error;
        return result;
    }

    GeocodeResult parsed = parseNominatimJson(http.body, limits_.maxGeocodeResults);
    if (!parsed.ok) {
        return parsed;
    }

    std::string writeErr;
    if (!cache_.writeAtomic(result.cachePath, http.body, writeErr)) {
        // Cache write failure is non-fatal for the search itself.
        parsed.error.clear();
    }
    parsed.cachePath = result.cachePath;
    parsed.fromCache = false;
    return parsed;
}

MapFetchResult MapDownloadService::downloadRoads(
    double latDeg,
    double lonDeg,
    double radiusMeters,
    bool forceRefresh,
    std::atomic<bool>* cancelFlag,
    ProgressFn progress
) {
    MapFetchResult result;
    result.radiusMeters = radiusMeters;

    std::string radiusError;
    if (!radiusAllowed(radiusMeters, radiusError)) {
        result.error = radiusError;
        return result;
    }

    result.bbox = bboxFromRadiusMeters(latDeg, lonDeg, radiusMeters);
    if (!result.bbox.isValid()) {
        result.error = "Invalid bounding box for the selected place/radius.";
        return result;
    }
    if (result.bbox.areaKm2() > kMaxDownloadAreaKm2) {
        result.error = "Download area too large — choose a smaller radius.";
        return result;
    }

    const std::string bboxKey = result.bbox.toKey(5);
    result.cachePath = cache_.overpassPath(bboxKey);

    if (!forceRefresh && cache_.exists(result.cachePath)) {
        if (progress) {
            progress("Odczyt cache mapy…");
        }
        std::string body;
        std::string err;
        if (cache_.tryRead(result.cachePath, body, err)) {
            std::string lookErr;
            if (looksLikeOverpassJson(body, lookErr)) {
                result.ok = true;
                result.fromCache = true;
                result.responseBytes = body.size();
                return result;
            }
        }
    }

    if (progress) {
        progress("Pobieranie dróg (Overpass) — obszar prostokątny…");
    }

    const std::string query = buildRoadNetworkQuery(result.bbox);
    HttpRequest req;
    req.method = "POST";
    req.url = endpoints_.overpassInterpreterUrl;
    req.contentType = "application/x-www-form-urlencoded; charset=UTF-8";
    req.body = "data=" + urlEncode(query);
    req.userAgent = kEasyRiderUserAgent;
    req.timeoutMs = limits_.httpTimeoutMs;
    req.maxResponseBytes = limits_.maxOverpassBytes;

    const HttpResponse http = sendWithLimitedRetries(req, cancelFlag, progress);
    if (http.cancelled) {
        result.cancelled = true;
        result.error = "Anulowano.";
        return result;
    }
    if (http.timedOut) {
        result.error = "Limit czasu Overpass.";
        return result;
    }
    if (http.sizeExceeded) {
        result.sizeExceeded = true;
        result.error = "Odpowiedź mapy przekroczyła limit rozmiaru — wybierz mniejszy obszar. "
                       "Graf nie został przycięty.";
        return result;
    }
    if (!http.ok) {
        result.error = http.error.empty() ? "Błąd HTTP Overpass." : http.error;
        return result;
    }

    std::string lookErr;
    if (!looksLikeOverpassJson(http.body, lookErr)) {
        result.error = lookErr;
        return result;
    }

    std::string writeErr;
    if (!cache_.writeAtomic(result.cachePath, http.body, writeErr)) {
        result.error = writeErr.empty() ? "Nie można zapisać cache mapy." : writeErr;
        return result;
    }

    result.ok = true;
    result.fromCache = false;
    result.responseBytes = http.body.size();
    return result;
}

} // namespace osm_net
