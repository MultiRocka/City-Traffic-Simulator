# Work plan — Stages 0–5 (tied to current code)

Based on Etap 0 discovery of `main` @ `fb9565b`. Implementation starts only after Stage 0 evidence is accepted.

## Stage 0 — Discovery / baseline (this pass)

**Goal:** Map real modules, toolchain, historical claims, hygiene candidates. No feature work.

**Done when:** `docs/PROJECT_CONTEXT.md`, this plan, and store evidence under Project Context `internal/etap0/` exist; claims marked EXISTS/CHANGED/MISSING/UNVERIFIED.

**Reality check:** Clean git tree; Win32 GUI + WinHTTP; no Qt; Linux cloud cannot run GUI; core compile on Linux blocked by unconditional `WinHttpTransport.cpp`.

## Stage 1 — Road geometry continuity

**Goal:** Make directed-road geometry trustworthy at junctions and along polylines (motion length vs intermediates vs shared nodes). Prefer **metrics and fixtures** over paint tweaks.

**Likely touch:** `OsmToMapDescription`, `CityBuilder` validation, `CityLayout` only if layout diverges from core truth; new/extended tests under `tests/osm_*` / `map_model_tests`.

**Out of scope:** Full CRS rewrite; inventing joins across bridges/layers; GUI-only cosmetic offsets without core agreement.

**Depends on:** Stage 0 map of length-vs-polyline rules (`MAP_MODEL.md`, `OSM_IMPORT.md`).

## Stage 2 — More vehicles / continuous urban traffic

**Goal:** Measurable increase in credible continuous traffic on Demo + sample OSM maps (spawn/demand/routing), without silent freezes or gap violations.

**Likely touch:** `OsmScenarioConfig` / `OsmScenarioBuilder`, `VehicleGenerator`, maybe Demo scenario demand; extend larger-map / scenario metrics (`CredibleFreeze`, progress, completion).

**Guardrails:** Keep finite, understandable demand; respect `allowAutoDemand` and turn bans; do not delete vehicles to “fix” freezes.

**Depends on:** Stage 1 geometry stable enough that density does not amplify bogus topology.

## Stage 3 — Pin + rectangle map selection

**Goal:** Add pin (and keep/improve rectangle) selection for download/import area; **retain** text place search.

**Likely touch:** `MapView` / `MainWindow` / `MapDownloadDialog`, `GeoBbox` helpers, `MapDownloadService`; tests for bbox construction; fake-HTTP download tests.

**Guardrails:** Keep hard radius/area/HTTP caps; transactional import; no whole-city download.

**Depends on:** Windows GUI environment for UX verification; Stage 0 confirmed text+rectangle already exist.

## Stage 4 — Careful cleanup + tech notes

**Goal:** Remove confirmed cruft, small CMake portability gates if approved, document Win32-vs-myth-Qt and module boundaries. **No rewrite.**

**Candidates (from Etap 0):** tracked `diag*.obj`, tracked `event_log.json`; optional `if(WIN32)` around `WinHttpTransport` / encoding tests; docs clarifying GUI stack.

**Needs decision:** Linux CI scope; whether Qt is ever desired (currently absent).

## Stage 5 — Integration review / freeze criteria

**Goal:** Independent review of Stages 1–4: geometry metrics, traffic metrics, selection UX, cleanup safety; update readiness docs; manual GUI checklist on Windows.

**Exit:** Priorities 1–3 demonstrably improved with tests/docs; cleanup only where evidence justified; no silent behavior changes outside stated goals.

---

## Cross-cutting rules

- Modular, readable C++; prefer correctness over patterns.
- Do not treat untracked/generated paths as trash without checking `.gitignore` and `data/cache` rules.
- Prefer offline fixtures + FakeHttp for CI; live Nominatim/Overpass remains optional/manual.
- Max concurrent Project workers: leave Independent Reviewer for Stage 5+.
