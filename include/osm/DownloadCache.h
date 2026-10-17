#pragma once

#include <cstddef>
#include <string>

namespace osm_net {

// File cache under data/cache/ (geocode + overpass JSON).
// Reuses existing files unless forceRefresh is true.
class DownloadCache {
public:
    explicit DownloadCache(std::string rootDir = "data/cache");

    const std::string& rootDir() const { return rootDir_; }

    std::string geocodePath(const std::string& normalizedQuery) const;
    std::string overpassPath(const std::string& bboxKey) const;

    bool tryRead(const std::string& path, std::string& outBody, std::string& error) const;
    bool writeAtomic(const std::string& path, const std::string& body, std::string& error) const;
    bool exists(const std::string& path) const;

    static std::string normalizeQuery(const std::string& query);
    static std::string hashKey(const std::string& text);

private:
    std::string rootDir_;
    bool ensureDir(const std::string& dir, std::string& error) const;
};

} // namespace osm_net
