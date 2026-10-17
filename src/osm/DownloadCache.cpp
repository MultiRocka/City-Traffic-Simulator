#include "osm/DownloadCache.h"

#include <cctype>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace osm_net {
namespace fs = std::filesystem;

namespace {

#ifdef _WIN32
fs::path u8path(const std::string& utf8) {
    return fs::u8path(utf8);
}
#else
fs::path u8path(const std::string& utf8) {
    return fs::path(utf8);
}
#endif

} // namespace

DownloadCache::DownloadCache(std::string rootDir)
    : rootDir_(std::move(rootDir)) {
}

std::string DownloadCache::normalizeQuery(const std::string& query) {
    std::string out;
    out.reserve(query.size());
    bool prevSpace = false;
    for (unsigned char ch : query) {
        if (std::isspace(ch)) {
            if (!out.empty() && !prevSpace) {
                out.push_back(' ');
                prevSpace = true;
            }
            continue;
        }
        prevSpace = false;
        out.push_back(static_cast<char>(std::tolower(ch)));
    }
    while (!out.empty() && out.back() == ' ') {
        out.pop_back();
    }
    return out;
}

std::string DownloadCache::hashKey(const std::string& text) {
    // FNV-1a 64-bit — stable, no crypto dependency.
    std::uint64_t hash = 14695981039346656037ull;
    for (unsigned char ch : text) {
        hash ^= ch;
        hash *= 1099511628211ull;
    }
    std::ostringstream ss;
    ss << std::hex << hash;
    return ss.str();
}

std::string DownloadCache::geocodePath(const std::string& normalizedQuery) const {
    return rootDir_ + "/geocode/" + hashKey(normalizedQuery) + ".json";
}

std::string DownloadCache::overpassPath(const std::string& bboxKey) const {
    return rootDir_ + "/overpass/" + hashKey("v1|" + bboxKey) + ".overpass.json";
}

bool DownloadCache::ensureDir(const std::string& dir, std::string& error) const {
    std::error_code ec;
    fs::create_directories(u8path(dir), ec);
    if (ec) {
        error = "Cannot create cache directory: " + dir + " (" + ec.message() + ")";
        return false;
    }
    return true;
}

bool DownloadCache::exists(const std::string& path) const {
    std::error_code ec;
    return fs::is_regular_file(u8path(path), ec);
}

bool DownloadCache::tryRead(const std::string& path, std::string& outBody, std::string& error) const {
    std::ifstream in(u8path(path), std::ios::binary);
    if (!in) {
        error = "Cannot read cache file: " + path;
        return false;
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    outBody = ss.str();
    return true;
}

bool DownloadCache::writeAtomic(const std::string& path, const std::string& body, std::string& error) const {
    const fs::path finalPath = u8path(path);
    const fs::path parent = finalPath.parent_path();
    if (!ensureDir(parent.u8string(), error)) {
        return false;
    }

    const fs::path tempPath = parent / (std::string(".") + finalPath.filename().u8string() + ".tmp");
    {
        std::ofstream out(tempPath, std::ios::binary | std::ios::trunc);
        if (!out) {
            error = "Cannot write temp cache file: " + tempPath.u8string();
            return false;
        }
        out.write(body.data(), static_cast<std::streamsize>(body.size()));
        if (!out) {
            error = "Failed while writing temp cache file.";
            return false;
        }
    }

    std::error_code ec;
    fs::remove(finalPath, ec);
    fs::rename(tempPath, finalPath, ec);
    if (ec) {
        // Fallback copy+remove for cross-device rename quirks.
        ec.clear();
        fs::copy_file(tempPath, finalPath, fs::copy_options::overwrite_existing, ec);
        fs::remove(tempPath, ec);
        if (ec) {
            error = "Cannot finalize cache file: " + path;
            return false;
        }
    }
    return true;
}

} // namespace osm_net
