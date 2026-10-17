# Pedestrians (Stage 4)

Visible pedestrian entities at marked crossings. **Not** a full sidewalk / footpath network.

## Model

| Piece | Role |
|-------|------|
| `PedestrianCrossing` | Location on a directed road + mode / walk speed / spawn cadence |
| `Pedestrian` | Entity: `Waiting` → `Crossing` → `Finished` |
| `PedestrianManager` | Spawn, entry safety, phase hold, seeded yield-refusal |

### Crossing modes

| Mode | Cars | Pedestrians |
|------|------|-------------|
| `Timed` (default) | Stop while crossing timer `isActive()` (legacy) | Visible ped synced to active window |
| `Signalized` | Respect vehicle signals | Enter only while vehicle light is **red**; phase advance **held** until roadway clear |
| `Unsignalized` | Yield to waiting ped (subject to refusal) | Enter only when **all** crossed lanes are safe |

Crosswalk width = `laneCount × LANE_WIDTH` (3.5 m).

## Encounter refusal (unsignalized)

Configurable probability (default **0.05**), seeded LCG (`PedestrianManager::defaultSeed`).

**Means only:** for a waiting (off-roadway) ped, the driver may proceed once per `(vehicleId, crossingId)` encounter.

Still enforced:

- Car **always** stops for a ped already on the roadway (`pedestrian_on_road`)
- Car **always** respects red lights
- Pedestrian **does not** step in front of a refusing approaching car
- **No** hit / collision events

Roll is **once per encounter**, not every tick. Same seed → same decisions.

```text
sim.setPedestrianSeed(20261013);
sim.setPedestrianYieldRefusalProbability(0.05); // 0.0 / 1.0 for tests
```

## Stop reasons

- `pedestrian_crossing` — timed active window or yielding to a waiting ped
- `pedestrian_on_road` — pedestrian entity currently crossing

Cars resume when the binding constraint clears.

## Snapshot / GUI

`RenderSnapshot.pedestrians[]` carries id, crossing, world position, state (0 waiting / 1 crossing), progress.
`PedestrianRenderer` + `MapView` draw entities; console preview lists them.

## Out of scope

- Sidewalk graph / routing between crossings
- OSM import (Stage 1 boundary only)
- Lane blocks / zipper merge — see [LANE_BLOCKS.md](LANE_BLOCKS.md) (Stage 5)
