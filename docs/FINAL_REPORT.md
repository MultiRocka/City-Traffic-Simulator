# Easy Rider — Final integration report (Etap 5 draft)

**Status:** draft for Independent Review. **No blanket PASS.**  
**Stack:** PR [#1](https://github.com/MultiRocka/City-Traffic-Simulator/pull/1) → [#2](https://github.com/MultiRocka/City-Traffic-Simulator/pull/2) → [#3](https://github.com/MultiRocka/City-Traffic-Simulator/pull/3) → [#5](https://github.com/MultiRocka/City-Traffic-Simulator/pull/5) → [#6](https://github.com/MultiRocka/City-Traffic-Simulator/pull/6) → Etap 5 cleanup/docs.  
**UI stack:** native **Win32 GDI** only — **no Qt** in this tree.

This file is the in-repo mirror of the Project store report. Prefer the store copy under Project Context `docs/final-report.md` for the full kickoff §14 sections and absolute evidence paths.

## Status per area (honest)

| Area | Status | Notes |
|------|--------|-------|
| Geometry continuity | **Improved (measured)** | Stub merge + turn entry lanes; residual hard–hard shorts; dual carriage stays split |
| Continuous traffic | **Improved (measured)** | Presets fill soft caps 80/160/280; continuous probe peaks ~964 / 1000 (cap unbound) |
| Map selection | **Implemented (code+tests)** | Pin+size + rectangle + text search; **no Windows visual PASS** from Linux cloud |
| Cleanup | **Conservative** | Removed tracked `diag*.obj` + tracked `event_log.json`; ignored `*.obj`/`*.o`/`*.pdb` |
| Integration | **Draft** | Linux offline suite green on Etap 4 head; GUI checklist still Windows-only |

## As found (Etap 0)

Mature C++17 Easy Rider: `core` → `ui` → Win32 `gui`, OSM Overpass pipeline, finite demand, text+radius download. **Missing vs verbal history:** Qt / pin-on-map / Linux-complete core build (later gated in Etap 1). Hygiene: tracked MSVC `diag*.obj` + tracked `event_log.json`.

## Changes by stage

| Stage | PR | Delivered |
|-------|----|-----------|
| 0 Discovery | #1 | Context + work plan; evidence of Win32-not-Qt |
| 1 Baselines | #2 | Geometry/traffic controls; Linux `WIN32` gates; Warsaw 1 km peak active ≈306 @ demand 1000 |
| 2 Geometry | #3 | Degree-2 `<5 m` stub merge; turn-aware entry lane; shorts **57→18**; components **9** |
| 3 Traffic | #5 | `continuousDemand` + soft `maxActive`; presets; peak **~964** @ 1000; route prep ~3 s→~0.2 s |
| 4 Selection | #6 | Win32 locator pin+size / rectangle; bbox≡query≡cache; 1032 PASS / 0 FAIL offline |
| 5 Cleanup | (this) | Proven cruft removal + docs clarifications + this report |

## Roads / vehicles / selection (measured)

- **Roads (`warsaw_1km`):** roads 1036→999; intersections 629→604; `<5 m` 57→18; `poly_mismatch=0`; dual carriage control stays 2 components.
- **Vehicles:** presets small/medium/large fill **80 / 160 / 280**; continuous maxActive 1000 → peak **964**, deferred=0 (honest unbound).
- **Selection:** text search retained; locator modes Pan / Pinezka+rozmiar / Prostokąt; no tile API; visual QA = Windows checklist only.

## Cleanup (this PR)

**Removed (SAFE TO REMOVE, verified unused as build inputs):** `diag2.obj`, `diag_freeze.obj`, `diag_spd.obj`, tracked root `event_log.json` (runtime path still writes locally; already gitignored).

**Kept:** OSM fixtures/samples, Win32 GUI, FakeHttp tests, useful diagnostics code, cache directory layout.

**Needs decision (user):** Linux CI Actions beyond local g++; whether Qt is ever desired (absent today); residual geometry (hard–hard shorts, dual connectors, `turn:lanes*`); Nominatim contact placeholder.

## Verification

- Etap 4 independent review: **1032 PASS / 0 FAIL** on Linux (`easy_rider_motion_tests`).
- **No Win32 visual PASS** claimed from this cloud environment.
- Etap 5: same Linux suite expected green after doc/cruft-only changes.

## How to run

See root [`RUN.md`](../RUN.md). Short form:

```powershell
# Windows GUI
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
.\build\Release\easy_rider_app.exe
.\build\Release\easy_rider_motion_tests.exe
```

```bash
# Linux core/tests
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=g++
cmake --build build --target easy_rider_motion_tests -j"$(nproc)"
./build/easy_rider_motion_tests   # cwd = repo root
```
