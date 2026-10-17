# Lane blocks & zipper merge (Stage 5)

Temporary closure of **one lane on one directed** `RoadSegment`. Opposite travel is a different segment and is **not** touched unless a separate block is added on it.

## Model

| Piece | Role |
|-------|------|
| `LaneBlockage` | `roadId`, `laneIndex`, `positionAlongRoad`, `length`, `startTime`, `duration` |
| `LaneBlockageManager` | Add/clear, random generation, stop caps, zipper grants, escape LC |
| `RandomBlockConfig` | Optional seeded random blocks (intervals, duration, length, position fraction) |

Stop line for a closed lane = `positionAlongRoad - LANE_BLOCK_STOP_DISTANCE` (1.5 m). Cars **queue**; they do **not** dodge via the opposite directed road. No auto detours this stage.

### Multi-lane (≥2 same direction)

- Cars on the closed lane may lane-change onto an open lane using existing `canChangeLane` / cooldown (Stage 2).
- **Zipper** applies only in the merge zone (`ZIPPER_MERGE_ZONE` ≈ 22 m before the block start): fair alternate between closed-lane head and open-lane head.
- If one queue is empty, the other does **not** wait for a ghost car.
- If there is no safe gap, the closed-lane car **waits** (never teleports). Open traffic is not stalled for an impossible merge.
- Zipper is **not** forced along the whole road or for normal `preferLane` LC.

### All lanes blocked

Every lane gets a stop cap → queue until a block ends. No route rewrite.

## API (Stage 6 GUI hooks)

```text
int  sim.addLaneBlockage(roadId, laneIndex, positionAlongRoad, length, startTime, duration, city);
bool sim.clearLaneBlockage(blockageId);
void sim.clearAllLaneBlockages();
const vector<LaneBlockage>& sim.getLaneBlockages();
void sim.setRandomLaneBlockConfig(RandomBlockConfig);
void sim.setLaneBlockSeed(uint32_t);
LaneBlockageManager& sim.getLaneBlockageManager();
```

`addLaneBlockage` returns `-1` if geometry is invalid or the span would **overlap a vehicle body** on that lane (no spawn-inside-vehicle, no delete/teleport).

### Random config fields

`enabled`, `minInterval`/`maxInterval`, `minDuration`/`maxDuration`, `minLength`/`maxLength`, `minPositionFraction`/`maxPositionFraction`, `seed` (default `20261013`).

## Events

State-change only (not every tick):

- `lane_blockage_started` / reason `lane_blockage`
- `lane_blockage_ended` / reason `lane_blockage_cleared`
- Escape LC: `vehicle_lane_change` / reason `lane_blockage_escape`

Stop reasons: `lane_blockage`, `zipper_wait`.

## Snapshot / GUI

`RenderSnapshot.laneBlockages[]` carries id, road, lane, world start/end, span, timing, `active`. MapView draws active spans; console lists them. Parameter panel is Stage 6.

## Out of scope

- OSM import
- Full GUI controls (Stage 6)
- Auto detours / opposite-direction dodge
- Zipper on the entire road or for every normal LC
