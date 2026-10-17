#pragma once

#include <atomic>
#include <cstddef>
#include <string>

namespace osm_net {

// Identifying User-Agent for Nominatim / Overpass usage policies.
// Do not invent a personal email here; optional contact is a documented config placeholder.
inline constexpr const char* kEasyRiderUserAgent =
    "EasyRider/0.1 (City Traffic Simulator; educational; local desktop)";

inline constexpr size_t kDefaultMaxGeocodeBytes = 512 * 1024;
inline constexpr size_t kDefaultMaxOverpassBytes = 6 * 1024 * 1024;
inline constexpr int kDefaultHttpTimeoutMs = 45000;
inline constexpr int kDefaultMaxRetries = 1;

struct HttpRequest {
    std::string method = "GET"; // GET | POST
    std::string url;
    std::string body;
    std::string contentType;
    std::string userAgent = kEasyRiderUserAgent;
    int timeoutMs = kDefaultHttpTimeoutMs;
    size_t maxResponseBytes = kDefaultMaxOverpassBytes;
};

struct HttpResponse {
    bool ok = false;
    int statusCode = 0;
    std::string body;
    std::string error;
    bool timedOut = false;
    bool cancelled = false;
    bool sizeExceeded = false;
};

// Injectable HTTP transport. Production uses WinHTTP; tests inject fakes.
// Implementations MUST NOT hit the network when a fake is used.
class IHttpTransport {
public:
    virtual ~IHttpTransport() = default;

    // cancelFlag may be null. When set true, abort promptly and return cancelled=true.
    // If response would exceed maxResponseBytes, set sizeExceeded and do NOT truncate body for parse.
    virtual HttpResponse send(const HttpRequest& request, std::atomic<bool>* cancelFlag = nullptr) = 0;
};

} // namespace osm_net
