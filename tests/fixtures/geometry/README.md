# Etap 1 geometry control fixtures

Synthetic Overpass-shaped JSON used by `geometry_control_tests` (importer path).
Additional precise layouts are built in-process as `MapDescription` (no JSON).

| File | Case |
|------|------|
| `straight_oneway.overpass.json` | Straight short oneway |
| `gentle_curve.overpass.json` | Gentle polyline curve |
| `sharp_turn.overpass.json` | Near-90° two-segment turn |
| `very_short_segment.overpass.json` | Sub-5 m directed edges (Etap 2 merges degree-2 chain) |
| `t_junction.overpass.json` | T junction (shared node) |
| `four_way.overpass.json` | 4-way crossroads |
| `fork_merge.overpass.json` | Fork then merge |
| `lanes_2_1.overpass.json` | `lanes:forward=2` / `backward=1` |
| `lanes_2_2.overpass.json` | Equal 2+2 bidirectional |
| `dual_carriageway.overpass.json` | Parallel oneway pair |
| `crossing_island.overpass.json` | Dual carriage + island crossing nodes |

Also exercised from `data/osm/`: `warsaw_sample`, `warsaw_1km`, and existing `fixtures/*`.
