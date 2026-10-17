# OSM sample data

Offline Overpass JSON for the OSM → `MapDescription` → simulation importer path (Stages 2–5).

See [docs/OSM_IMPORT.md](../../docs/OSM_IMPORT.md) and [RUN.md](../../RUN.md).

## Attribution

Map data © [OpenStreetMap](https://www.openstreetmap.org/copyright) contributors, available under the [Open Database License (ODbL)](https://opendatacommons.org/licenses/odbl/).

When redistributing derived maps, follow ODbL share-alike / attribution requirements.

## Real samples

### Small — `warsaw_sample` (regression)

| Field | Value |
|-------|--------|
| Path | `data/osm/warsaw_sample.overpass.json` |
| Status | **Present** (fetched successfully) |
| Source | Overpass API → OpenStreetMap |
| Endpoint | `https://overpass-api.de/api/interpreter` |
| BBox (S,W,N,E) | `52.22480,21.01580,52.22680,21.01900` |
| Area | Small patch near ul. Wilcza / ul. Krucza (Warsaw Śródmieście), ~220×220 m — **not** the whole city |
| Retrieval date | 2026-10-13 |
| OSM base timestamp (in file) | `2026-10-13T17:47:39Z` |
| Size / contents | ~16 KB; 20 ways + 103 nodes (highway ways + referenced nodes) |
| Script preset | `-Sample Small` (default) |

#### Overpass QL (Small — matches in-app `buildRoadNetworkQuery`)

```
[out:json][timeout:25];
(
  way["highway"~"^(motorway|trunk|primary|secondary|tertiary|unclassified|residential|living_street|service)$"](52.22480,21.01580,52.22680,21.01900);
)->.roads;
.roads out body;
.roads >;
out skel qt;
rel(bw.roads)["type"="restriction"];
out body;
```

`out body` returns ways with tags and node id lists; `>;` + `out skel qt` pulls member nodes (ids + lat/lon) needed for topology. `rel(bw.roads)["type"="restriction"]` pulls turn-restriction relations. Re-fetch with `-Force` to refresh.

### Larger — `warsaw_1km` (Stage 4 credible checks)

| Field | Value |
|-------|--------|
| Path | `data/osm/warsaw_1km.overpass.json` |
| Status | **Present** (fetched successfully) |
| Source | Overpass API → OpenStreetMap |
| Endpoint | `https://overpass-api.de/api/interpreter` |
| BBox (S,W,N,E) | `52.22130,21.01040,52.23030,21.02440` |
| Area | Same center as Small, ~1.0×1.0 km — **not** the whole city |
| Retrieval date | 2026-10-13 |
| OSM base timestamp (in file) | `2026-10-13T19:34:33Z` |
| Size / contents | ~460 KB; 571 ways + ~2349 nodes + 18 relations (includes tagged crossing / signal / stop / give_way nodes) |
| Script preset | `-Sample Larger` |

#### Overpass QL (Larger — extended for tagged nodes)

`out skel` omits node tags, so Larger also requests tagged highway nodes in the bbox (crossings, traffic signals, stop, give_way) with `out body`:

```
[out:json][timeout:80];
(
  way["highway"~"^(motorway|trunk|primary|secondary|tertiary|unclassified|residential|living_street|service)$"](52.22130,21.01040,52.23030,21.02440);
)->.roads;
.roads out body;
.roads >;
out skel qt;
(
  node["highway"~"^(crossing|traffic_signals|stop|give_way)$"](52.22130,21.01040,52.23030,21.02440);
);
out body;
rel(bw.roads)["type"="restriction"];
out body;
```

If this file is **MISSING** after a failed fetch, keep the script params above, note the date of the attempt, and use Small + `data/osm/fixtures/` for offline CI. Do not invent a fake city-sized OSM dump.

## How to re-fetch

From the repo root (Windows PowerShell):

```powershell
# Small regression sample (default)
.\scripts\fetch_osm_sample.ps1

# Larger ~1 km extract (Stage 4)
.\scripts\fetch_osm_sample.ps1 -Sample Larger

# Force refresh
.\scripts\fetch_osm_sample.ps1 -Sample Larger -Force
```

Optional arguments:

```powershell
.\scripts\fetch_osm_sample.ps1 -Sample Small -OutFile data\osm\warsaw_sample.overpass.json -TimeoutSec 60
.\scripts\fetch_osm_sample.ps1 -Sample Larger -TimeoutSec 180
.\scripts\fetch_osm_sample.ps1 -Sample Larger -BBox "52.22130,21.01040,52.23030,21.02440"
```

Behavior:

- Single HTTP POST via `Invoke-WebRequest` (no parallel / unlimited retries)
- Writes to a temp file first; replaces the final path only after validating JSON with a non-empty `elements` array
- Never overwrites a good sample with error HTML/JSON
- If fetch fails and no valid file exists, treat that sample as **MISSING** — do not invent fake OSM data

## Synthetic fixtures (`data/osm/fixtures/`)

**All fixtures are SYNTHETIC — not real OpenStreetMap data.** Tiny Overpass-shaped stubs for offline tests.

| File | Purpose |
|------|---------|
| `empty.overpass.json` | Valid Overpass shell, empty `elements` |
| `invalid_not_json.txt` | Non-JSON / HTML-like error stub |
| `invalid_no_elements.overpass.json` | JSON without `elements` |
| `oneway.overpass.json` | `oneway=yes` highway stub |
| `oneway_reverse.overpass.json` | `oneway=-1` reverse stub |
| `missing_node_ref.overpass.json` | Way refs a node id not present in `elements` |
| `twoway_stub.overpass.json` | Bidirectional (no `oneway`) stub |
| `with_relation.overpass.json` | Incomplete restriction stored; Stage 3 WARN (not enforced) |
| `directional_lanes.overpass.json` | `lanes:forward` / `lanes:backward` |
| `shared_junction.overpass.json` | Shared-node junction (connected) |
| `crossing_no_shared_node.overpass.json` | Crossing geometries without shared node |
| `curved_polyline.overpass.json` | Shape nodes → intermediates / length sum |
| `maxspeed_variants.overpass.json` | Bare km/h, mph, unsupported `walk` |
| `crossing_and_signals.overpass.json` | Crossing + `traffic_signals` |
| `disconnected_components.overpass.json` | Two disjoint components |
| `roundabout.overpass.json` | `junction=roundabout` → inferred oneway |
| `unsupported_access.overpass.json` | `access=private` excluded; `turn:lanes` WARN |
| `stop_give_way.overpass.json` | stop / give_way → YieldToRight |
| `restriction_no_left.overpass.json` | `no_left_turn` enforced |
| `restriction_only_straight.overpass.json` | `only_straight_on` enforced |
| `restriction_split_way.overpass.json` | Restriction survives way split |
| `restriction_via_way_unsupported.overpass.json` | via-way → WARN only |
| `access_destination.overpass.json` | destination spur excluded from auto demand |
| `stop_direction_approach.overpass.json` | stop `direction=forward` → PriorityRoad |

## App / tests

- GUI: **Load OSM…** / **Demo City** (Stage 5) — see [RUN.md](../../RUN.md)
- Offline tests: `tests/osm_json_parser_tests.cpp`, `tests/osm_to_map_tests.cpp`, `tests/osm_scenario_tests.cpp`, `tests/osm_larger_map_tests.cpp`, `tests/osm_session_tests.cpp`
- Larger-map suite measures parse/build/tick times, model size, progress, freezes (reason must match obstacle), lane-block/zipper resume, reset/restart, time multipliers — all offline once the JSON is saved
