#pragma once

#include <cmath>
#include <string>

namespace osm_net {

// Axis-aligned geographic bounding box (degrees). Download area is always rectangular.
struct GeoBbox {
    double south = 0.0;
    double west = 0.0;
    double north = 0.0;
    double east = 0.0;

    bool isValid() const {
        return south < north && west < east
            && south >= -90.0 && north <= 90.0
            && west >= -180.0 && east <= 180.0;
    }

    // Approximate area of the rectangle in km² (equirectangular).
    double areaKm2() const;

    // Stable cache / Overpass key: "S,W,N,E" with fixed decimals.
    std::string toKey(int decimals = 5) const;

    // Same formatting as toKey(5) — Overpass QL bbox fragment must match cache key.
    std::string toOverpassBBox() const;
};

struct BboxDimensionsMeters {
    double widthMeters = 0.0;   // east–west
    double heightMeters = 0.0;  // north–south
};

// Equirectangular width/height of a geographic rectangle.
BboxDimensionsMeters bboxDimensionsMeters(const GeoBbox& box);

struct AreaSizeChoice {
    const char* id;           // stable id e.g. "r500"
    const wchar_t* labelUi;   // Polish UI label
    double radiusMeters;
    bool warnBeforeDownload;  // true for larger presets
};

// Limited size choices — do NOT offer whole-city downloads.
inline constexpr AreaSizeChoice kAreaSizeChoices[] = {
    {"r250",  L"~0,25 km (mały)",                 250.0,  false},
    {"r500",  L"~0,5 km (domyślny)",              500.0,  false},
    {"r1000", L"~1 km (ostrzeżenie)",            1000.0,  true},
    {"r2000", L"~2 km (duży — ostrzeżenie)",     2000.0,  true},
};
inline constexpr int kAreaSizeChoiceCount = 4;
inline constexpr int kDefaultAreaSizeIndex = 1; // ~0.5 km

inline constexpr double kMaxDownloadRadiusMeters = 2000.0;
inline constexpr double kMaxDownloadAreaKm2 = 20.0; // hard reject above this

// Rectangular bbox centered on (lat,lon) with half-side ≈ radiusMeters.
// Note: this is a rectangle in lat/lon, not a geodesic circle.
GeoBbox bboxFromRadiusMeters(double latDeg, double lonDeg, double radiusMeters);

bool radiusAllowed(double radiusMeters, std::string& errorOut);

// Arbitrary rectangle limits (pin+size or drawn). Max side ≈ 2 × max radius.
bool bboxAllowed(const GeoBbox& box, std::string& errorOut);

} // namespace osm_net
