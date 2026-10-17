#pragma once

// Named right-of-way policy for the whole simulation.
// Stage 3 assumes right-hand traffic; flip RIGHT_HAND_TRAFFIC for left-hand regions
// instead of scattering magic side checks through call sites.

namespace TrafficPolicy {

constexpr bool RIGHT_HAND_TRAFFIC = true;

// How far past the exit entry a vehicle must travel before releasing the junction reservation.
constexpr double INTERSECTION_CLEARANCE_M = 6.0;

// Vehicles this close to the stop line (or already there) count as relevant for yield/priority.
constexpr double APPROACH_RELEVANCE_DISTANCE_M = 12.0;

// |turn angle| below ~45° counts as straight.
constexpr double STRAIGHT_COS_THRESHOLD = 0.70710678118; // cos(45°)

// Inbounds nearly opposite + both straight → compatible (no conflict).
constexpr double OPPOSITE_INBOUND_COS_THRESHOLD = -0.85;

// Dot product threshold when matching "approach from the right" inbound direction.
constexpr double FROM_RIGHT_COS_THRESHOLD = 0.5;

} // namespace TrafficPolicy
