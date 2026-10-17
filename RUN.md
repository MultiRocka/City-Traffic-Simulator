# Easy Rider / City Traffic Simulator — Build & Run

## Prerequisites

- Windows with Visual Studio 2022 (MSVC)
- CMake 3.16+
- Network on **first** configure (downloads pinned **nlohmann/json v3.11.3** via FetchContent; MIT; see [docs/OSM_IMPORT.md](docs/OSM_IMPORT.md))

## Configure

From the repo root:

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
```

First configure fetches `nlohmann/json` (only JSON library; header-only MIT). Later configures reuse the CMake cache / `_deps` copy.

## Build Release

```powershell
cmake --build build --config Release
```

Outputs:

- `build\Release\easy_rider_app.exe` — Win32 GUI
- `build\Release\easy_rider_console.exe` — console runner
- `build\Release\easy_rider_motion_tests.exe` — regression tests

## Build Debug

```powershell
cmake --build build --config Debug
```

Outputs:

- `build\Debug\easy_rider_app.exe`
- `build\Debug\easy_rider_console.exe`
- `build\Debug\easy_rider_motion_tests.exe`

Use Debug when stepping through the debugger; runtime and test behavior match Release for the Stage 0 lifecycle (start / pause / resume / stop / reset).

## Units (core + GUI)

| Quantity | Unit |
|----------|------|
| Length / position | **m** |
| Time / duration / interval | **s** |
| Speed (core + stats) | **m/s** (GUI also shows **km/h** = m/s × 3.6) |
| Spawn / tempo / time multipliers | dimensionless (`1.0` = default; range **0.1–10**) |

Physics step is always fixed: `simulationDt = 0.1 s` (10 Hz). The time multiplier only scales how much sim time accumulates per wall frame.

## Run GUI

```powershell
.\build\Release\easy_rider_app.exe
# or
.\build\Debug\easy_rider_app.exe
```

### Transport controls (GUI labels often Polish)

| Control | Behavior |
|---------|----------|
| **Start** | Starts the simulation, or resumes if already running but paused |
| **Pauza** | Freezes simulation time and vehicle motion (render may continue) |
| **Wznów** | Continues after Pause (clears the frame accumulator) |
| **Stop** | Stops the simulation (keeps current vehicles/scenario in memory) |
| **Reset** | Clears runtime state and reloads the **currently loaded** scenario (Demo City **or** the last successful OSM file). Time ≈ 0, no vehicles; generators/lights/crossings restored. Ready to Start again. Keeps current tempo / time-multiplier settings and map camera fit |
| **Wczytaj OSM…** | File picker for Overpass JSON. Validates + builds fully before swapping; on failure the previous scenario is kept. Stops the sim first (no mid-tick swap). Shows a one-shot import summary (counts, warnings, generated-signals note); errors also in status bar / MessageBox |
| **Szukaj miejsca…** | Place name → Nominatim search (only on **Szukaj** click) → pick result → limited rectangular area → Overpass download → import. See below |
| **Miasto demo** | Returns to the synthetic Demo City scenario |
| **Dopasuj mapę** | Fits the loaded map into the map viewport |

Layout: action bar (top) → map (main) + right column (statystyki / parametry / raport importu + dziennik) → status bar (selected map + app state). Controls enable/disable with app state.

`SimulationSession::reset()` / `loadOsmFromFile` / `ControlPanel::reset()` are also available for non-GUI callers.

### Tempo / rate parameters (Apply tempo)

| Field | Meaning | Unit / range |
|-------|---------|--------------|
| **Veh rate x** | Scales vehicle generator spawn frequency (higher → shorter spawn intervals) | ×, 0.1–10 |
| **Ped tempo x** | Scales pedestrian request / spawn cadence at crossings | ×, 0.1–10 |
| **Time x** | Simulation time multiplier: sim seconds advanced per wall second | ×, 0.1–10 |

**Time multiplier details**

- Does **not** change physics `dt` (always 0.1 s steps).
- Catch-up per wall frame is clamped to `maxCatchUpSimSeconds = 0.5 s` so a lagged render cannot jump many seconds of sim time at once.

### Manual lane blockage

| Field | Meaning | Unit |
|-------|---------|------|
| **RoadId** | Directed `RoadSegment` id | id |
| **Lane** | Lane index on that directed road (`0`…`laneCount-1`) | index |
| **Dur s** | Block duration | **s** (0.5–600) |
| **Add block** | Adds a block at ~40% along the road, length 5 m (defaults) | |
| **Clear blocks** | Removes all lane blockages | |

Rejected if geometry is invalid or the span would overlap a vehicle body on that lane.

### Random lane blockages (Apply random)

| Field | Meaning | Unit |
|-------|---------|------|
| **Random blocks** | Enable / disable | checkbox |
| **Int s** | Interval between random blocks (min=max when set from GUI) | **s** |
| **Dur s** | Duration of each random block | **s** |
| **Seed** | RNG seed for reproducible random blocks | integer |

### Map observability & navigation

The map draws: directed roads with per-lane offsets, vehicles on correct lanes, pedestrians, signals/crossings, active lane blockages (red), and zipper merge approach (dotted amber) on multi-lane blocked roads. On-map legend (vehicles, pedestrians, signals, blocks). Double-buffered GDI; resize works; min window ~1024×640; DPI-aware fonts.

| Input | Behavior |
|-------|----------|
| **Mouse wheel** over map | Zoom in/out around cursor (clamped scale) |
| **Drag** (LMB) on map | Pan view |
| **Dopasuj mapę** | Fit full world bounds into the map viewport |

Camera is view-only (`MapCamera` / `MapView`) — it does **not** change core world coordinates. **Reset** keeps the current map and re-fits; **Miasto demo** / **Wczytaj OSM…** clear/swap scenario then fit.

### Event log stop reasons

State-change stop reasons from core are shown with labels when known:

| Core reason | Label |
|-------------|-------|
| `red_light` | red |
| `vehicle_ahead` | vehicle ahead |
| `pedestrian_crossing` / `pedestrian_on_road` | pedestrian |
| `yield_priority` | priority/yield |
| `intersection_conflict` | intersection conflict |
| `blocked_next_road` | blocked next road |
| `lane_blockage` | lane blockage |
| `zipper_wait` | waiting lane change |
| `destination_reached` | destination |
| `route_invalid` | invalid route |

Unknown reasons are shown unchanged (never invented).

## Run console

```powershell
.\build\Release\easy_rider_console.exe
# or
.\build\Debug\easy_rider_console.exe
```

## Run tests (CTest)

Release:

```powershell
ctest --test-dir build -C Release --output-on-failure
```

Debug (if configured):

```powershell
ctest --test-dir build -C Debug --output-on-failure
```

Or run the executable directly:

```powershell
.\build\Release\easy_rider_motion_tests.exe
.\build\Debug\easy_rider_motion_tests.exe
```

Test artifacts are written under `tests/output/` (not the committed `event_log.json`).

## Map model

Core units are **meters**, **seconds**, and **m/s** (`speedLimit`). Map data enters via `MapDescription` → `CityBuilder` → `City`. See [docs/MAP_MODEL.md](docs/MAP_MODEL.md) and [docs/OSM_READINESS.md](docs/OSM_READINESS.md) (importer boundary; not OSM plug-and-play).

### OSM Overpass JSON (Stages 2–5 + limited download)

Offline pipeline: Overpass JSON → `OsmDocument` → `MapDescription` + `ImportReport` → `OsmScenarioBuilder` → session/GUI. Details: [docs/OSM_IMPORT.md](docs/OSM_IMPORT.md). Honesty / limits: [docs/OSM_READINESS.md](docs/OSM_READINESS.md). Learning tour: [docs/CODE_TOUR.md](docs/CODE_TOUR.md). Sample/fixtures under `data/osm/`.

**Not full-OSM:** file import and place download are bounded extracts. Via-node turn bans + car access hierarchy are **enforced**; via-way / conditional / `turn:lanes*` are **WARN only**. Signal timing is **generated**.

**Simulation policy caps** (`OsmScenarioConfig`, defaults): `maxAttachedCrossings=32`, `maxGeneratedPhaseControllers=24`, `maxRouteHops=12`. These limit live ped crossings / generated phases / auto-route length — they do **not** truncate the imported map graph. Download size/radius limits **reject** oversized fetches (never silent graph truncation). See [OSM_IMPORT.md](docs/OSM_IMPORT.md#simulation-policy-caps-not-map-truncation).

#### Szukaj miejsca… (limited map download)

1. Click **Szukaj miejsca…** (GUI stays on Demo/previous map until a successful import).
2. Type a place name and click **Szukaj** (no search-on-keystroke).
3. Pick a result from the list.
4. Choose area size (default **~0.5 km** radius). The download area is a **rectangle (bbox)** around the center, not a circle.
5. Larger presets (**~1 km**, **~2 km**) show a warning before download. Whole-city download is not offered.
6. **Pobierz i zaimportuj** — worker thread + progress; **Anuluj czekanie** cancels. Cache under `data/cache/` is reused unless **Wymuś ponowne pobranie**.
7. On success: import summary → **Start**. Failed download/import keeps the previous scenario.

| Limit | Value |
|-------|--------|
| User-Agent | `EasyRider/0.1 (City Traffic Simulator; educational; local desktop)` |
| Nominatim | `https://nominatim.openstreetmap.org/search` (configurable in code) |
| Overpass | `https://overpass-api.de/api/interpreter` (configurable) |
| Max radius | **2 km** (hard reject above) |
| Max HTTP body | geocode **512 KiB**, Overpass **6 MiB** (reject; never silently truncate) |
| Cache | `data/cache/geocode/`, `data/cache/overpass/` |

**Contact placeholder (Nominatim / Overpass):** if a provider requires a contact URL/email beyond User-Agent, set `DownloadEndpoints::contactPlaceholder` (see [OSM_IMPORT.md](docs/OSM_IMPORT.md)). Default is **empty** — fill before heavy public use when policy demands a contact. Live Nominatim+Overpass smoke is optional/manual (not part of CTest); offline tests use `FakeHttpTransport` only. A one-shot live probe may be recorded under `tests/output/stage5_live_smoke.txt` when network works.

**Cache:** successful geocode/Overpass payloads under `data/cache/geocode/` and `data/cache/overpass/` (gitignored payloads). Reused unless **Wymuś ponowne pobranie** / Force.

Attribution: map data © [OpenStreetMap](https://www.openstreetmap.org/copyright) contributors ([ODbL](https://opendatacommons.org/licenses/odbl/)); geocoding via Nominatim; roads via Overpass.

#### Fetch a sample (optional network / script)

From the repo root (PowerShell):

```powershell
# Small regression sample (~220 m) — default
.\scripts\fetch_osm_sample.ps1

# Larger Stage-4 extract (~1 km bbox, same Warsaw center — not whole city)
.\scripts\fetch_osm_sample.ps1 -Sample Larger

# Force refresh either preset:
.\scripts\fetch_osm_sample.ps1 -Sample Small -Force
.\scripts\fetch_osm_sample.ps1 -Sample Larger -Force -TimeoutSec 180
```

| Preset | Output | Notes |
|--------|--------|--------|
| `Small` (default) | `data/osm/warsaw_sample.overpass.json` | Query matches `osm_net::buildRoadNetworkQuery` |
| `Larger` | `data/osm/warsaw_1km.overpass.json` | Same highway filter + tagged crossing/signal/stop nodes (skel omits tags) |

If fetch fails, keep script params documented in [`data/osm/README.md`](data/osm/README.md), mark that file **MISSING**, and use the other sample / `data/osm/fixtures/` for offline tests. CI stays offline once the JSON is committed/saved.

#### Load file in the GUI

1. Build and run `easy_rider_app.exe` (starts on Demo City).
2. Click **Wczytaj OSM…** and select e.g. `data\osm\warsaw_sample.overpass.json`, `data\osm\warsaw_1km.overpass.json`, or a fixture under `data\osm\fixtures\`.
3. On success: MessageBox summary (roads, generators, generated-signals note, warnings). Map fits to view. Click **Start**.
4. **Reset** clears traffic but keeps the imported OSM map (reloads the same file).
5. **Miasto demo** returns to the synthetic scenario.
6. A failed import leaves the previous scenario unchanged.

#### Offline tests

```powershell
ctest --test-dir build -C Release --output-on-failure
# or
.\build\Release\easy_rider_motion_tests.exe
```

Includes parser / map / scenario / **larger-map credible checks** / session / **map download** tests (`tests/osm_*.cpp`, `tests/map_download_tests.cpp`). Download tests use a **fake HTTP transport** and never hit the public network. Working directory must be the repo root so `data/osm/...` resolves.

Larger-map suite (`tests/osm_larger_map_tests.cpp`) runs only when `warsaw_1km.overpass.json` is present: moderate + higher demand, ped crossings when imported, lane-block/zipper resume, reset/restart, time-multiplier physics, and metrics (parse/build/tick, roads/intersections, progress, freezes with reason↔obstacle match). Reports under `tests/output/osm4_*`.

### Default scenario: Demo City

GUI and console start with `SimulationSession::loadDemoScenario()` → `Scenario::setupDemoCity` (same `MapDescription` path the OSM importer targets).

Includes: one-way multi-lane approach (lane-block / zipper site), two-way as pairs of directed edges, 1-lane and ≥2-lane segments, varied `speedLimit`, a non-straight polyline road, signalized + PriorityRoad + unsignalized (YieldToRight) junctions, and pedestrian crossings. Generators use fixed seeds and PathFinder routes that respect directionality.

Regression grid remains available: `loadIntersectionTestScenario()` / `Scenario::setupIntersectionTest`.

### Stage 2 notes

- Multi-lane corridors use independent same-lane car-following; render offsets lanes by `LANE_WIDTH`.
- Vehicles have per-instance length, desired/max speed, accel/decel; cruise ≤ `min(desired, road limit)`.
- Intentional lane change: `Simulation::tryLaneChange(vehicleId, targetLane, city)`.
- Polyline intermediates affect layout/render; motion length remains authoritative.

## Manual GUI checklist (smoke)

1. App window opens with the **Demo City** map visible (polyline curve, multi-lane west approach).
2. Start/resume simulation — vehicles spawn and move.
3. Pause — vehicles and sim time freeze despite elapsed wall-clock time.
4. Resume — motion continues without a visible teleport.
5. Reset — map returns to a fresh copy of the **current** scenario (Demo or last OSM); Start works again.
5b. Wczytaj OSM… — pick a JSON sample; summary appears; Start runs on the imported map. Failed load keeps Demo.
5c. Szukaj miejsca… — type place, Szukaj, pick, ~0.5 km, download; import summary; Start. Cancel/error keeps previous map.
5d. Miasto demo — returns from OSM to Demo; Start works again.
6. Change **Time x** to 2 and Apply — sim time advances faster; motion still uses fixed physics dt.
7. Change **Veh rate x** / **Ped tempo x** — spawn/request cadence changes.
8. Add a manual lane block (valid roadId/lane/duration) — red span appears; Clear removes it.
9. Optionally enable random blocks with interval/duration/seed.
10. Vehicles stop at red lights and move again on green.
11. Queues form with visible gaps; no obvious permanent freeze on an empty green approach.
12. Pedestrian crossings briefly block, then traffic resumes when clear.
13. Statistics / event log panels update (stop reasons labeled).

This checklist is manual only; automated tests cover core motion logic, session lifecycle, time-multiplier/catch-up, and manual block API — not pixel/GUI verification.
