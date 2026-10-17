# OSM import (Stages 2–5)

Offline Overpass JSON → `OsmDocument` → `MapDescription` + `ImportReport` →  
`OsmScenarioBuilder` → `City` + Simulation attachments → `SimulationSession` / Win32 GUI.

Demo City remains a separate synthetic scenario. GUI can switch between Demo and an imported OSM file.

Related: [OSM_READINESS.md](OSM_READINESS.md), [CODE_TOUR.md](CODE_TOUR.md), [MAP_MODEL.md](MAP_MODEL.md), [`data/osm/README.md`](../data/osm/README.md), [RUN.md](../RUN.md).

---

## JSON library (Stage 2)

| Item | Value |
|------|--------|
| Library | [nlohmann/json](https://github.com/nlohmann/json) |
| Version | **v3.11.3** (pinned) |
| License | **MIT** |
| Integration | CMake `FetchContent` in root `CMakeLists.txt` |
| Artifact | `https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz` |
| SHA-256 | `d6c65aca6b1ed68e7a182f4757257b107ae403032760ed6ef121c9d55e81757d` |
| Link target | `nlohmann_json::nlohmann_json` → `easy_rider_core` (PUBLIC) |

This is the **only** JSON library in the repo. No third-party HTTP/GIS libraries — limited download uses native **WinHTTP** (`WinHttpTransport`) with an injectable `IHttpTransport` for offline tests.

---

## Parse API (Stage 2)

Headers: `include/osm/OsmDocument.h`, `include/osm/OsmJsonParser.h`  
Source: `src/osm/OsmJsonParser.cpp`

| Type | Fields |
|------|--------|
| `OsmNode` | `int64_t id`, `lat`, `lon`, `tags` |
| `OsmWay` | `int64_t id`, `nodeRefs`, `tags` |
| `OsmRelation` | `int64_t id`, `members` (`type`/`ref`/`role`), `tags` |
| `OsmDocument` | `generator`, `osmBaseTimestamp`, `nodes`, `ways`, `relations` |

```cpp
OsmParseResult OsmJsonParser::parseFile(const std::string& path);
OsmParseResult OsmJsonParser::parseString(const std::string& jsonText);
```

**Fails cleanly:** missing file, non-JSON, missing/invalid/empty `elements`.  
**Reports without failing:** missing node refs. Relations are stored on the document for Stage 3 classification.

### File paths (Windows)

`parseFile` expects a **UTF-8** path string (GUI `GetOpenFileNameW` → UTF-8). On Windows it opens via UTF-16 (`MultiByteToWideChar` + wide `ifstream`), so directories/filenames with Polish characters and spaces work without relocating the project to an ASCII path.

---

## Map build API (Stage 3)

Headers: `include/osm/OsmToMapDescription.h`, `include/osm/ImportReport.h`  
Source: `src/osm/OsmToMapDescription.cpp`

```cpp
OsmToMapResult OsmToMapDescription::convertFile(const std::string& path);
OsmToMapResult OsmToMapDescription::convertParseResult(const OsmParseResult&, const std::string& sourceFile);
OsmToMapResult OsmToMapDescription::convert(const OsmDocument&, ...);

OsmLocalProjection OsmLocalProjection::fromDocumentBBoxCenter(const OsmDocument&);
Vector2D OsmLocalProjection::project(double latDeg, double lonDeg) const;
double OsmToMapDescription::polylineLengthMeters(const std::vector<Vector2D>&);
```

`OsmToMapResult`: `ok`, `error`, `MapDescription map`, `ImportReport report`.

Parse failures stay at the parse layer (`ok=false`, empty map). Successful convert always produces a `MapDescription` (possibly empty car network) and a filled report.

Target model is the existing `MapDescription` → `CityBuilder` path. **No parallel city model.**

### Projection

Equirectangular meters around the document bbox center:

- `x = (lon − lon₀) · 111319.9 · cos(lat₀)`
- `y = (lat − lat₀) · 111132.92`

(`lat₀`, `lon₀` in degrees; `lat₀` converted to radians for `cos`.) Suitable for small city extracts (~hundreds of meters), not global CRS.

### Topology / geometry

| Rule | Behavior |
|------|----------|
| Junctions | Shared highway nodes with **degree ≥ 3**, **highway endpoints**, nodes shared by **≥2 car highways**, or tagged `traffic_signals` / `stop` / `give_way` |
| Shape nodes | Degree-2 mid-way vertices (including `highway=crossing` alone) stay as `intermediatePoints` / on-segment features — **not** topology splits |
| Length | Sum of consecutive projected segment lengths; authoritative `MapRoadDesc::length` |
| `sourceRef` | Roads: `way/<id>#nodes=<fromIdx>-<toIdx>;dir=fwd\|rev`; intersections: `node/<id>` |
| No fake joins | Crossing geometries **without** a shared OSM node are **not** connected |
| layer / bridge / tunnel | Never create proximity joins; shared node ids still join (OSM intent). Multi-layer shared nodes are warned |
| Boundary | Dangling ends kept as entries/exits; incompleteness noted; **no** invented links outside data |

Car-allowed `highway` values: motorway(+link), trunk(+link), primary(+link), secondary(+link), tertiary(+link), unclassified, residential, living_street, service, road. Others skipped.

Directed edges: bidirectional → two `MapRoadDesc`; `oneway=yes` → forward only; `oneway=-1`/`reverse` → reverse only; `junction=roundabout`/`circular` → inferred forward oneway.

---

## Tag policies

Report categories: **read** / **inferred** / **default** / **unsupported**.

| Topic | Policy |
|-------|--------|
| `oneway` | yes/true/1 → forward; -1/reverse → reverse; no/false/0 → bidirectional; unknown → bidirectional + unsupported |
| Roundabout | `junction=roundabout\|circular` → inferred oneway forward |
| `lanes` / `lanes:forward` / `lanes:backward` | Directional tags win. Bidirectional with only `lanes=N`: split (`N=1` → 1 each as narrow-road **default**; else remainder to forward). **Never** copy total `lanes` onto both directions |
| `maxspeed` (+ `:forward`/`:backward`) | Parse units explicitly: bare number → **km/h**; `km/h`/`kmh`/`kph`; `mph`. Do **not** strip the first number from free text. `walk`/`none`/`signals`/… → unsupported + default |
| Default speed | **50 km/h** (→ m/s) when missing/unsupported for an emitted direction |
| Default lanes | **1** per emitted direction when no usable lane tags |
| Crossings | Node `highway=crossing`, `crossing=*`, or `footway=crossing` → `MapCrossingDesc` on each directed segment containing the node |
| Signals | Node `highway=traffic_signals` (or `traffic_signals=yes`) → `PriorityMode::Signalized` |
| stop / give_way | See [Priority / stop & give_way](#priority--stop--give_way) |
| Turn restrictions | See [Turn restrictions (enforced)](#turn-restrictions-enforced) |
| Car access | See [Access hierarchy for cars](#access-hierarchy-for-cars) |
| Unsupported (WARN) | via-way / conditional / complex restrictions; `turn:lanes*`; `access:*` subkeys; `*:conditional` — reported; **not** claimed as enforced |

### Defaults table

| Field | Default / assumption | Report category |
|-------|----------------------|-----------------|
| `speedLimit` if no usable maxspeed | 50 km/h → m/s | default |
| `laneCount` if no usable lanes | 1 per direction | default |
| Bidirectional `lanes=1` | 1 each direction | default |
| Bidirectional `lanes=N` (N≥2), no forward/backward | split; remainder → forward | inferred |
| Roundabout without oneway | oneway forward | inferred |
| `PriorityMode` without stop/give_way/signals | `None` (sim → YieldToRight) | default / assumption |

---

## Turn restrictions (enforced)

Supported when `type=restriction` with **exactly**:

- one `from` **way**, one `via` **node**, one `to` **way**
- `restriction` in `no_left_turn` / `no_right_turn` / `no_straight_on` / `no_u_turn` / `no_entry` / `no_exit`  
  or `only_left_turn` / `only_right_turn` / `only_straight_on` / `only_u_turn`

**Mapping:** after way splitting, directed segments that **arrive** at the via node on the from-way and **leave** on the to-way are identified. Results are written as `MapConnectionDesc{allowed=false}` between those segment ids.

| Kind | Effect |
|------|--------|
| `no_*` | Disallow from-segment → to-segment at via |
| `only_*` | Disallow all other geometric exits from the from-segment(s) at via |

**Honored by:** `CityBuilder` → `City::disallowConnection` → `areRoadsConnected` / `getOutgoingRoadIds` → **PathFinder** and vehicle road-to-road transitions. PathFinder state remains `(roadId)`; previous-edge context is not required because bans are encoded as from→to connection disallows at the via.

**Still WARN (not enforced):** via-way, multiple from/to/via, conditional (`*:conditional` / `@`), unrecognized restriction values, incomplete members. `except` that names motorcar/motor_vehicle/vehicle skips applying the ban to cars.

---

## Access hierarchy for cars

Resolved most-specific-wins (OSM wiki tree, car mode):

`access` → `vehicle` → `motor_vehicle` → `motorcar`

| Resolved value | Behavior |
|----------------|----------|
| empty (default for car highways) | **Open** — routable; auto demand OK |
| `yes` / `permissive` / `designated` / `official` / `public` | **Open** |
| `no` / `private` / `military` | **Forbidden** — way **not** emitted into the map |
| `destination` / `customers` / `delivery` / `agricultural` / `forestry` / `permit` | **Destination-only** — road built; `allowAutoDemand=false` (generators / auto routes skip it) |
| `unknown`, values with `@`, any `*conditional*` tag, other unrecognized | **UnsupportedRestricted** — WARN; treated like destination for auto demand (conservative; not free transit) |

`turn:lanes*` and `access:*` subkeys remain unsupported (WARN).

---

## Priority / stop & give_way

| Case | Behavior |
|------|----------|
| `highway=traffic_signals` | `PriorityMode::Signalized` |
| `stop` / `give_way` **without** approach `direction` | `YieldToRight` for the junction (no `priorityRoadIds` invented from road class) |
| `stop` / `give_way` with `direction=forward\|backward` | Bind relative to a single through-way (node mid-way) or single incident way → `PriorityMode::PriorityRoad` with `priorityRoadIds` = **other** incoming approaches. Multi-way ambiguity → WARN + `YieldToRight` |
| Other `direction` values | WARN + `YieldToRight` |

Generated signal plans (Stage 4) stay simulation-generated, one approach green at a time, conflict-free — timing is **not** from OSM.

---

## ImportReport

| Field | Meaning |
|-------|---------|
| `sourceFile` | Path or label |
| `inputNodeCount` / `inputWayCount` / `inputRelationCount` | Parse-scale counts |
| `builtIntersectionCount` / `builtRoadCount` / `builtLaneCount` | Built sizes (`builtLaneCount` = sum of per-direction lane counts) |
| `builtCrossingCount` / `signalizedIntersectionCount` | Crossings / signalized nodes |
| `readFromTags` / `inferred` / `defaultsApplied` / `unsupported` | Interpretation accounting |
| `skipped` | Skipped ways/segments with reasons |
| `missingNodeRefs` / `unsupportedRelationIds` | Missing refs; relations not enforced |
| `graphComponentCount` + `graphComponentsNote` | Disconnected components + dangling ends |
| `routingLimitsNote` / `incompletenessNote` / `warnings` | Honest limits |

---

## Simulation attachment (Stage 4)

Headers: `include/osm/OsmScenarioBuilder.h`  
Source: `src/osm/OsmScenarioBuilder.cpp`

```cpp
OsmScenarioResult OsmScenarioBuilder::loadOsmFile(path, city, sim, config);
OsmScenarioResult OsmScenarioBuilder::attach(map, report, city, sim, config);
```

`loadOsmFile` is **transactional**: convert + `CityBuilder` + attachments run on staging objects;  
on failure the caller’s existing `City` / `Simulation` are left unchanged (safe for a pre-loaded Demo).

| Step | Behavior |
|------|----------|
| City | `CityBuilder::build(MapDescription)` — no Demo id hardcoding; applies connection disallows |
| Crossings | `MapDescription.crossings` → `PedestrianCrossing` (`Unsignalized`); yield-refusal default **5%** |
| Signals | For each `PriorityMode::Signalized` with incoming roads: **generated** `PhaseController` — one approach green at a time (`phaseGreenSeconds`, default **8 s**). Timing is **not** from OSM |
| Demand | Finite `VehicleGenerator`s (default ≤8 × 5 vehicles), explicit `seed`, PathFinder routes only on roads with `allowAutoDemand` |
| Routing | Largest weakly-connected intersection component; directed edges; `MapConnectionDesc` turn disallows honored |

### Defaults (signals / demand)

| Knob | Default |
|------|---------|
| `seed` | `VehicleGenerator::defaultSeed` (`20261013`) |
| `maxGenerators` | 8 |
| `vehiclesPerGenerator` | 5 |
| `spawnIntervalSeconds` | 2.5 |
| `phaseGreenSeconds` | 8.0 |
| `minVehicleGap` | 4.0 |
| `yieldRefusalProbability` | 0.05 |

### Simulation policy caps (not map truncation)

These `OsmScenarioConfig` knobs limit **what the simulation attaches / routes**, not how much of the Overpass JSON is kept in `MapDescription`. The full car network from convert stays in `City`; HTTP Overpass bodies that exceed size limits are **rejected** (never silently truncated). Caps of `0` mean unlimited for that attachment.

| Knob | Default | Policy meaning |
|------|---------|----------------|
| `maxAttachedCrossings` | **32** | At most this many imported `MapCrossingDesc` become live `PedestrianCrossing`s (prefers crossings on generator-route roads; thins densely tagged extracts) |
| `maxGeneratedPhaseControllers` | **24** | At most this many `PriorityMode::Signalized` junctions get a generated `PhaseController` (prefers intersections on routes). Remaining signalized nodes stay signalized in topology but without a generated cycle |
| `maxRouteHops` | **12** | Prefer / truncate auto-demand PathFinder routes to at most this many directed edges (`≤0` → legacy unlimited / longest). Keeps large extracts completable in finite sim time |
| `minRouteHops` | **2** | Soft preference when scoring candidate routes |
| `maxRouteSearchAttempts` | **240** | Cap start/goal pair attempts when building demand routes |
| `crossingInactiveDuration` | **18 s** | Base inactive interval for attached Unsignalized OSM crossings |

**Do not confuse with download limits:** max Overpass body **6 MiB** and max radius **2 km** reject the fetch; they do not drop edges from an accepted map.

### Routing limits (honest)

- Enforced: basic `no_*` / `only_*` via-node restrictions; car access hierarchy (forbidden excluded; destination/conditional skipped for auto demand).
- Still unsupported (WARN only): via-way / conditional / complex restrictions, `turn:lanes*`.
- Disconnected extracts → demand only on the largest useful component; dangling boundary ends are not invented away.
- Generated signal plans are safe (no simultaneous conflicting greens) but not realistic coordinated timing.
- Scenario attachment caps above are **simulation policy**, not claims that OSM data was truncated.

---

## Session + GUI (Stage 5)

Headers: `include/ui/SimulationSession.h`, `include/gui/MainWindow.h`  
Source: `src/ui/SimulationSession.cpp`, `src/gui/MainWindow.cpp`

```cpp
session.loadDemoScenario();
OsmScenarioResult r = session.loadOsmFromFile(path);  // transactional; stops first
session.reset();  // clears traffic; reloads Demo or the same OSM path
```

| Behavior | Detail |
|----------|--------|
| Swap gate | Session **stops** before import so city/sim never swap mid-tick |
| Failure | Previous scenario (Demo or OSM) kept; mode / path unchanged |
| Success | Mode = `SessionScenario::Osm`; path stored; import summary cached |
| Reset | Reloads current mode (re-imports OSM file → fresh generators, same map topology) |
| Demo City | GUI button returns to `Scenario::setupDemoCity` |
| Fit / camera | **Dopasuj mapę** fits `WorldBounds`; mouse wheel zoom + LMB drag pan (`MapCamera` / `MapView`) — view-only |
| Summary UX | One-shot MessageBox + status line on import; not spammed every tick |

GUI controls: **Wczytaj OSM…** / Load OSM (`GetOpenFileName`), **Szukaj miejsca…** (Nominatim + limited Overpass), **Miasto demo**, **Dopasuj mapę**, plus existing Start / Pause / Resume / Stop / Reset.

---

## Limited place → download → import

Headers / sources under `include/osm/` + `src/osm/`:

| Module | Role |
|--------|------|
| `HttpTransport.h` / `WinHttpTransport` | Native WinHTTP GET/POST, HTTPS, timeouts, cancel, size cap |
| `FakeHttpTransport` | Offline scripted responses for tests |
| `DownloadCache` | `data/cache/geocode/`, `data/cache/overpass/` (atomic write; reuse unless Force) |
| `GeoBbox` / `bboxFromRadiusMeters` | Radius → **rectangular** bbox; max radius 2 km |
| `OverpassQuery` | Shared QL (highways + `type=restriction`) — keep in sync with `scripts/fetch_osm_sample.ps1` |
| `MapDownloadService` | Nominatim search + Overpass fetch orchestration |
| `gui/MapDownloadDialog` | Win32 UI; worker thread; `PostMessage` marshal; cancel + join on close |

**User-Agent:** `EasyRider/0.1 (City Traffic Simulator; educational; local desktop)`  
**Contact placeholder:** `DownloadEndpoints::contactPlaceholder` (empty by default). Fill if Nominatim/Overpass policy requires a contact URL/email beyond UA before heavy public use.

| Limit | Value |
|-------|--------|
| Area choices | ~0.25 / **0.5 (default)** / 1 / 2 km radius → rectangular bbox |
| Max radius / area | 2 km / ~20 km² hard reject |
| Max geocode body | 512 KiB |
| Max Overpass body | 6 MiB — **reject**, never silently truncate the graph |
| Retries | at most 1 retry on 429/502/503/504; no parallel spam |
| HTTP timeout | 45 s default |

Failed download or failed import leaves the previous scenario unchanged (same transactional swap as file load).

Attribution: © OpenStreetMap contributors ([ODbL](https://opendatacommons.org/licenses/odbl/)); Nominatim geocoding; Overpass API.

---

## Data & tests

- Real sample: `data/osm/warsaw_sample.overpass.json`
- Fixtures: `data/osm/fixtures/` (including restriction / access / stop-direction / non-ASCII path tests)
- Cache dir: `data/cache/` (payloads gitignored; see `data/cache/README.md`)
- Tests: `tests/osm_json_parser_tests.cpp`, `tests/osm_to_map_tests.cpp`, `tests/osm_scenario_tests.cpp`, `tests/osm_session_tests.cpp`, `tests/map_download_tests.cpp` (fake transport only)

Run from repo root via CTest or `easy_rider_motion_tests.exe` (cwd = project source).

---

## What is not done (do not over-claim)

- No `.osm` XML / PBF; **no full-OSM / whole-city** auto-download (radius ≤ 2 km rectangle)
- No via-way / conditional / complex restriction enforcement; no `turn:lanes*` (**WARN** in `ImportReport`, not enforced)
- Signal **timing** is generated, not read from OSM
- Scenario caps may attach fewer crossings / phase controllers / shorter auto routes than the map contains — that is sim policy, not incomplete parse
- No sidewalk network, elevation, or left-hand traffic switch
- Visual GUI QA remains manual (see [RUN.md](../RUN.md))

**Do claim (enforced):** basic via-node `no_*`/`only_*` restrictions and the car access hierarchy above are enforced for PathFinder / transitions / auto demand.  
**Do claim (WARN only):** via-way / conditional / complex restrictions, `turn:lanes*`, unrecognized access — reported without fake compliance.  
**Do claim (limited download):** place search + bounded Overpass fetch with cache, size limits, and offline-tested fake transport — **not** unbounded OSM import.
