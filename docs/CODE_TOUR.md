# Code tour / Przewodnik po kodzie

Short bilingual map of Easy Rider for learning. Not a full architecture doc.  
Krótki dwujęzyczny przewodnik po klasach i jednym ticku.

Related: [OSM_READINESS.md](OSM_READINESS.md), [OSM_IMPORT.md](OSM_IMPORT.md), [MAP_MODEL.md](MAP_MODEL.md), [RUN.md](../RUN.md).

---

## Key classes / Kluczowe klasy

| Class | Role (EN) | Rola (PL) |
|-------|-----------|-----------|
| `MapDescription` | Plain-data boundary (roads, junctions, crossings, turn bans) | Granica danych mapy (bez logiki ruchu) |
| `CityBuilder` | Validates + builds `City` from `MapDescription` | Walidacja i budowa `City` |
| `City` | Directed road graph, connections, queries | Graf dróg skierowanych + zapytania |
| `PathFinder` | Shortest directed routes; honors turn disallows | Najkrótsze trasy; respektuje zakazy skrętów |
| `Simulation` | One fixed physics tick (`dt = 0.1 s`) | Jeden tick fizyki |
| `Vehicle` / `VehicleGenerator` | Motion + finite spawns on routes | Ruch + skończone generowanie |
| `IntersectionRules` / `PhaseController` | Grants + generated signal cycles | Pierwszeństwo + wygenerowane fazy |
| `PedestrianCrossing` / `PedestrianManager` | Crossing demand / hold | Piesi na przejściach |
| `LaneBlockageManager` | Temp blocks, zipper, escape LC | Blokady pasów, zipper, ucieczka |
| `OsmJsonParser` → `OsmToMapDescription` | Overpass JSON → map + `ImportReport` | Import OSM → mapa + raport |
| `OsmScenarioBuilder` | Attach gens / phases / crossings (policy caps) | Podpięcie scenariusza (limity polityki) |
| `SimulationSession` | Demo/OSM swap, accumulator, render snapshot | Sesja GUI: start/pauza/reset, interpolacja |
| `MainWindow` / `MapView` / `MapCamera` | **Win32 GDI** UI (not Qt), zoom/pan/fit | UI **Win32 GDI** (bez Qt), zoom/pan/fit |
| `MapDownloadDialog` / `LocatorMap` | Place search + pin/size / rectangle locator | Szukaj miejsca + pinezka / prostokąt |
| `MapDownloadService` | Nominatim + Overpass (cache, size caps) | Szukaj miejsca + pobranie (cache, limity) |

---

## One tick flow / Jeden tick

Physics step is always **0.1 s** (10 Hz). Wall clock and **Time x** only change how many fixed steps run per frame.

```
GUI frame (~16 ms)
  → SimulationSession::update(frameDt)
       simAdvance = frameDt × timeMultiplier  (catch-up clamped)
       while accumulator ≥ 0.1:
         Simulation::update(0.1, city)
           1. PhaseController (+ hold) → sync TrafficLight
           2. PedestrianCrossing + PedestrianManager
           3. LaneBlockageManager (expire / zipper / escape LC)
           4. VehicleGenerator spawn
           5. Per-road vehicle motion (gaps, lights, peds, blocks)
           6. IntersectionRules grants at stop lines
           7. Prune finished; statistics + EventLog
         capture RenderSnapshot
       interpolate previous → current for MapView
```

**OSM path (offline file):**  
`Overpass JSON` → `OsmJsonParser` → `OsmToMapDescription` → `CityBuilder` → `OsmScenarioBuilder` → `SimulationSession` / GUI.

**Limited download:** **Szukaj miejsca…** → Nominatim → bbox → Overpass → same import path (transactional; failure keeps previous scenario).

---

## Honesty reminders / Uczciwe limity

- Via-node `no_*` / `only_*` + car access hierarchy → **enforced**. Via-way / conditional / `turn:lanes*` → **WARN only**.
- Signal **timing** is generated, not from OSM.
- `maxAttachedCrossings` / `maxGeneratedPhaseControllers` / `maxRouteHops` = **simulation policy**, not map truncation.
- Not full-OSM / whole-city import (area and HTTP body caps).
