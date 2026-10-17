#include "osm/GeoBbox.h"

#include <iomanip>
#include <sstream>

namespace osm_net {
namespace {

constexpr double kMetersPerDegLat = 111320.0;

double clampLat(double lat) {
    if (lat < -90.0) {
        return -90.0;
    }
    if (lat > 90.0) {
        return 90.0;
    }
    return lat;
}

} // namespace

double GeoBbox::areaKm2() const {
    if (!isValid()) {
        return 0.0;
    }
    const double midLat = 0.5 * (south + north);
    const double heightM = (north - south) * kMetersPerDegLat;
    const double cosLat = std::cos(midLat * 3.14159265358979323846 / 180.0);
    const double widthM = (east - west) * kMetersPerDegLat * std::max(0.01, std::abs(cosLat));
    return (heightM * widthM) / 1.0e6;
}

std::string GeoBbox::toKey(int decimals) const {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(decimals)
       << south << "," << west << "," << north << "," << east;
    return ss.str();
}

std::string GeoBbox::toOverpassBBox() const {
    return toKey(5);
}

GeoBbox bboxFromRadiusMeters(double latDeg, double lonDeg, double radiusMeters) {
    GeoBbox box;
    if (!(radiusMeters > 0.0) || !std::isfinite(latDeg) || !std::isfinite(lonDeg)) {
        return box;
    }

    const double latRad = latDeg * 3.14159265358979323846 / 180.0;
    const double cosLat = std::cos(latRad);
    const double metersPerDegLon = kMetersPerDegLat * std::max(0.01, std::abs(cosLat));

    const double dLat = radiusMeters / kMetersPerDegLat;
    const double dLon = radiusMeters / metersPerDegLon;

    box.south = clampLat(latDeg - dLat);
    box.north = clampLat(latDeg + dLat);
    box.west = lonDeg - dLon;
    box.east = lonDeg + dLon;

    if (box.west < -180.0) {
        box.west = -180.0;
    }
    if (box.east > 180.0) {
        box.east = 180.0;
    }
    return box;
}

bool radiusAllowed(double radiusMeters, std::string& errorOut) {
    if (!(radiusMeters > 0.0) || !std::isfinite(radiusMeters)) {
        errorOut = "Radius must be a positive finite number.";
        return false;
    }
    if (radiusMeters > kMaxDownloadRadiusMeters + 1e-6) {
        errorOut = "Radius exceeds the maximum allowed download size (~2 km). Choose a smaller area.";
        return false;
    }
    // Reject absurd areas even if radius somehow passes (safety net).
    const GeoBbox probe = bboxFromRadiusMeters(0.0, 0.0, radiusMeters);
    if (probe.areaKm2() > kMaxDownloadAreaKm2) {
        errorOut = "Computed download area is too large. Choose a smaller radius.";
        return false;
    }
    return true;
}

} // namespace osm_net
