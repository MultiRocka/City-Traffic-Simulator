# Project context — City Traffic Simulator (Easy Rider)

Last discovery: Etap 0 (structure audit). Prefer this file + `RUN.md` + `docs/OSM_READINESS.md` over verbal history.

## What this is

Educational city traffic simulator in **C++17**. Product name in CMake: **EasyRider**. Map data enters as plain `MapDescription`, builds a directed-road `City`, then runs a fixed-step simulation (`dt = 0.1 s`) with vehicles, signals, pedestrians, and lane blockages. Desktop UI is **native Win32 GDI** (Polish labels), not Qt.

## Repository map

| Area | Location |
|------|----------|
| Core simulation | `include/core`, `src/core` |
| OSM import / download | `include/osm`, `src/osm` |
| Session + render snapshots | `include/ui`, `src/ui` |
| Win32 GUI | `include/gui`, `src/gui`, `easy_rider_app` |
| Console runner | `src/app/main_console.cpp` |
| Tests (one CTest binary) | `tests/`, target `easy_rider_motion_tests` |
| Samples / fixtures | `data/osm/` |
| Runtime HTTP cache | `data/cache/{geocode,overpass}/` (payloads gitignored) |
| How to build/run | [`RUN.md`](../RUN.md) |

CMake targets: `easy_rider_core`, `easy_rider_ui`, `easy_rider_gui`, `easy_rider_app`, `easy_rider_console`, `easy_rider_motion_tests`. Dependency: **nlohmann/json v3.11.3** via FetchContent. Live HTTP uses **WinHTTP** on Windows; tests use `FakeHttpTransport`.

## Pipeline (truth)

```
Overpass JSON → OsmJsonParser → OsmToMapDescription → MapDescription + ImportReport
  → CityBuilder → City → OsmScenarioBuilder → Simulation
  → SimulationSession → RenderSnapshot → MapView / GUI
```

Demo City uses the same `MapDescription` path. Limited download: place text search → Nominatim → rectangular bbox → Overpass → same import (transactional).

## Units

Meters, seconds, m/s (GUI also shows km/h). See `RUN.md`.

## Honest limits (do not over-claim)

- Not full-OSM / whole-city download (area and HTTP body caps).
- Via-node turn bans + car access hierarchy enforced; via-way / conditional / `turn:lanes*` WARN only.
- Signal **timing** is generated.
- Simulation policy caps (`maxAttachedCrossings`, `maxGeneratedPhaseControllers`, `maxRouteHops`) limit attachments/routes, not silent map truncation.
- Details: [`OSM_READINESS.md`](OSM_READINESS.md), [`OSM_IMPORT.md`](OSM_IMPORT.md), [`CODE_TOUR.md`](CODE_TOUR.md).

## Platform reality

- **Documented primary path:** Windows + Visual Studio 2022 (MSVC) + CMake.
- **This Project’s cloud VM:** Linux; can configure with `g++`, but **cannot** build/run the Win32 GUI as shipped; `WinHttpTransport.cpp` is compiled into `easy_rider_core` without a `WIN32` CMake gate.
- Historical mentions of **Qt 6 / `easy_rider_qt` / `Uruchom.bat`** do **not** match this repository.

## Product priorities (upcoming stages)

1. Road geometry continuity (authoritative geometry, not paint).
2. More vehicles / continuous urban traffic (measured).
3. Pin + rectangle map selection (keep text search).
4. Careful cleanup + tech notes (no rewrite).

Plan: [`WORK_PLAN.md`](WORK_PLAN.md).
