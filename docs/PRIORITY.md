# Intersection priority (Stage 3)

Right-of-way at junctions. Crossing-local pedestrians are in [PEDESTRIANS.md](PEDESTRIANS.md) (Stage 4). Lane blocks / zipper are in [LANE_BLOCKS.md](LANE_BLOCKS.md) (Stage 5).

## Right-hand traffic policy

All side-sensitive rules go through `TrafficPolicy::RIGHT_HAND_TRAFFIC` (see `include/core/TrafficPolicy.h`). Do not scatter left/right magic `if`s in call sites — flip that one constant for left-hand regions.

Under right-hand traffic:

- **Left** turn = counter-clockwise yaw from inbound to outbound (positive 2D cross product).
- **Yield to the right** = give way to a conflicting approach whose inbound ≈ rotate90CCW(ego inbound).

## Priority modes (`PriorityMode`)

| Mode | Behavior |
|------|----------|
| `Signalized` | Existing lights/phases. Green does **not** skip entrance space (`canEnterRoad`) or conflicting occupation. |
| `PriorityRoad` | Approaches listed in `priorityRoadIds` go before subordinates when movements conflict. Same-class ties use yield-to-right / wait order. |
| `YieldToRight` | Conflicting traffic from the right has priority. |
| `None` | Treated as **YieldToRight** for unsignalized junctions at runtime. |

Set modes on `MapIntersectionDesc` → `CityBuilder` → `Intersection`.  
Demo City: Signalized (SigJ), PriorityRoad (PrioJ + west approach ids), YieldToRight (NorthMid).  
Legacy intersection-test grid: signalized (ids 6, 10), PriorityRoad (id 2), YieldToRight (id 5).

## Turn classification

`IntersectionRules::classifyTurn` uses geometry only: inbound direction of `fromRoad`, outbound of `toRoad`, shared intersection position. No dependency on vehicle id.

## Conflict + reservation

- Two movements **conflict** if their junction chords cross, they merge to the same exit, or a left turn meets a non-compatible opposing move. Opposite straights are compatible.
- Occupation: `intersectionId →` list of `{vehicleId, fromRoad, toRoad, turn}`.
- Claim on grant; release when the vehicle travels `TrafficPolicy::INTERSECTION_CLEARANCE_M` on the exit road, finishes, or is removed. **No permanent reservation** after leave/finish.
- Vehicles are **not** stopped at every junction “just in case” — only when lights, blocked exit, yield duty, or a real conflicting reservation applies.

## Waiting order

Among peers: earliest wait-start at the stop line, then approach bearing. **Not** vehicleId as the sole priority rule. Circular yield-to-right is broken with the same ordered grant.

## Stop reasons

| Reason | Meaning |
|--------|---------|
| `red_light` | Signal red (unchanged) |
| `blocked_next_road` | No entrance space on next road |
| `yield_priority` | Yielding to priority / right-hand traffic |
| `intersection_conflict` | Conflicting movement occupies the junction |

## Code entry points

- `IntersectionRules` — classify, conflict, yield, reservations, grant order
- `Simulation::update` — clear stale reservations → collect stop-line candidates → `resolveGrants` → reserve + `moveToNextRoad`
- `CollisionDetector::determineStopReason` — optional `IntersectionRules*` for yield/conflict labels
