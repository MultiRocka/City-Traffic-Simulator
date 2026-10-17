# Map model (Stages 1–7)

How map data enters `City`, what is stored vs enforced, and what a future OSM importer must supply.

OSM / importer readiness (honest boundary): see [OSM_READINESS.md](OSM_READINESS.md).  
Junction right-of-way details: see [PRIORITY.md](PRIORITY.md).  
Visible pedestrians at crossings: see [PEDESTRIANS.md](PEDESTRIANS.md).

## Units (core)

| Quantity | Unit |
|----------|------|
| Length / position along road / gap | **meters** |
| Simulation time / phase / crossing durations | **seconds** |
| `RoadSegment::speedLimit` and vehicle speeds | **m/s** |

Positions in the plane (`Vector2D`) are meters in a local map frame.

## Data path

```
MapDescription  →  CityBuilder::build()  →  City
     │                    │
     │                    ├─ validates ids, lengths, lanes, crossings, connections
     │                    └─ applies turn disallows
     │
     └─ MapCrossingDesc validated here; PedestrianCrossing still owned by Simulation
```

Synthetic scenarios build a `MapDescription`, call `CityBuilder`, then attach generators / lights / crossings to `Simulation`. Default: `Scenario::setupDemoCity` / `buildDemoMapDescription`. Legacy regression grid: `setupIntersectionTest`.

Empty `sourceRef` on roads or intersections means **synthetic/local** data (not from an external source). Non-empty `sourceRef` is an opaque provenance string (e.g. future `way/123`).

## Directions vs lanes

- One `RoadSegment` is **one directed edge** (travel from `from` → `to`).
- The opposite direction is a **separate** `RoadSegment`.
- `laneCount` is the number of lanes for **that direction only** (≥ 1). Never treat opposite travel as “lanes” of the same segment.

## Vehicle motion (Stage 2)

| Field | Unit / notes |
|-------|----------------|
| `laneIndex` | `0 .. laneCount-1` on the current directed segment; clamped on road change |
| `length` | meters (front reference = `positionOnRoad`) |
| `desiredSpeed` / `maxSpeed` | m/s; cruise ≤ `min(desired, max, road speedLimit)` |
| `maxAcceleration` / `maxDeceleration` | m/s²; assigned once at spawn, never re-rolled |

Car-following predecessor = nearest vehicle ahead on the **same road and same lane** (gap uses leader length + `minVehicleGap`). Independent lanes do not block each other.

Lane change (`Simulation::tryLaneChange`): same directed road only; requires safe longitudinal gap (no overlap + min gap) on the target lane; rejects when `laneCount == 1` or cooldown active (`SimConstants::LANE_CHANGE_COOLDOWN`). Optional `preferLane` may request an LC when safe. Closed-lane escape / zipper merge (Stage 5) reuses this API — see [LANE_BLOCKS.md](LANE_BLOCKS.md).

### VehicleGenerator seed policy

- Default seed: `VehicleGenerator::defaultSeed` (`20261013`).
- Call `setSeed` before `update` for reproducible trait sequences.
- Traits (desired speed 80–100% of generator base, length, accel/decel) are assigned **once** at spawn via an LCG.

## Geometry vs length

- `length` (meters) is **authoritative** for motion and crossing placement.
- `intermediatePoints` are optional polyline vertices between endpoints (for rendering / future geometry).
- If intermediates are present, `CityBuilder` checks that polyline length (from → points → to) is within `CityBuilder::polylineLengthTolerance` (default **1.0 m**) of `length`.

## Connections / turns

- Default: any pair that shares an intersection geometrically (`from.to == to.from`) is allowed — same as historical `City::areRoadsConnected`.
- `MapConnectionDesc` with `allowed = false` records a disallowed turn on `City`.
- `areRoadsConnected` / path finding honor disallows.

## Intersection priority (Stage 3)

`PriorityMode` (`None` / `Signalized` / `PriorityRoad` / `YieldToRight`) and optional `priorityRoadIds` are stored on `Intersection` and enforced by `IntersectionRules` (see [PRIORITY.md](PRIORITY.md)). Unsignalized `None` → YieldToRight at runtime. Right-hand traffic is `TrafficPolicy::RIGHT_HAND_TRAFFIC`.

## Defaults vs source

| Field | Default if unset |
|-------|------------------|
| `laneCount` | `1` (`MapRoadDesc::laneCount == -1`) |
| `sourceRef` | empty → synthetic |
| `priorityMode` | `None` |
| Connections | all geometric turns allowed |
| Polyline | empty (straight chord for layout) |

## What a future OSM importer must supply

- Intersections with stable ids + positions (meters or converted)
- Directed road edges with length (m), speedLimit (**m/s** after conversion from km/h), `laneCount` for that direction, optional polyline, `sourceRef`
- Optional crossing positions in `[0, length]`
- Optional disallowed turns as connection overrides
- Optional priority mode / priority road ids (enforced in Stage 3)

## Pedestrians (Stage 4)

`MapCrossingDesc` is still validated in `CityBuilder` and attached as `PedestrianCrossing` on `Simulation`. Stage 4 adds visible `Pedestrian` entities managed by `PedestrianManager` (waiting / crossing / clear), signalized phase hold, unsignalized yield, and seeded encounter refusal. See [PEDESTRIANS.md](PEDESTRIANS.md).

## Lane blocks (Stage 5)

`LaneBlockage` + `LaneBlockageManager`: temporary per-lane closure on a directed segment, snapshot visibility, optional random generation, zipper near merge only. See [LANE_BLOCKS.md](LANE_BLOCKS.md).

## Not implemented yet

- OSM Overpass JSON → `MapDescription` → scenario/GUI load exists offline (see [OSM_IMPORT.md](OSM_IMPORT.md)); basic via-node restrictions + car access hierarchy enforced; still not plug-and-play (no PBF/XML, via-way/conditional/`turn:lanes` unsupported, generated signal timing)
- No turn-specific target lane / connection lane mapping (LC intentional / preferLane / zipper)
- No sidewalk network between crossings (Stage 4 is crossing-local only)
