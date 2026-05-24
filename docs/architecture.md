# Architecture

The wand is a pure broadcaster: it reads its own position and pointing direction and transmits them over P2P radio. All grasping logic, and flight control run independently on each receiver. This means the system scales to any number of drones with no added load on the wand.

## App types

### Wand (`wand.c`)

On every loop iteration (100 ms) it:

1. Reads its own position `(x, y, z)` from the Lighthouse state estimator.
2. Reads pitch and yaw angles and computes the body X-axis in world frame as the direction vector `(dx, dy, dz)`.
3. While the button on deck IO1 is held, packs the 6 floats plus its radio ID into a P2P packet and broadcasts it on port `0x01`.

The button is managed by `button.c` as a registered deck driver with 40 ms debouncing on `DECK_GPIO_IO1`. An optional melody plays via `wand_sounds.c` on press/release.

### Receiver (`receiver.c`, `receiver_hlc.c`)

Each receiver listens for wand P2P packets and implements the grasping algorithm: while the wand is pointed at it, it builds a confidence score; once the score exceeds the threshold the drone arms, takes off, and tracks the point on the wand line it was assigned. If the wand signal is lost, the drone hovers in place, or lands, depending on the release height.

Two variants are available (`receiver.c`, `receiver_hlc.c`), differing only in how they send flight commands. `receiver.c` uses the "low-level" commander, while `receiver_hlc.c` uses the high-level commander.

## P2P packet format

```
Byte  0     : wand radio ID (uint8)
Bytes 1–4   : x  (float32)
Bytes 5–8   : y  (float32)
Bytes 9–12  : z  (float32)
Bytes 13–16 : dx (float32)
Bytes 17–20 : dy (float32)
Bytes 21–24 : dz (float32)
```

## Grasping algorithm

The wand packet defines a 3-D line: origin **P** = `(x, y, z)` and unit direction **D** = `(dx, dy, dz)`.

1. **Distance**: On each packet, the receiver computes its perpendicular distance to the wand line:
   ```
   v    = receiver_pos − P        (vector from wand origin to the drone)
   dist = |v × D|                 (cross product gives perpendicular distance)
   ```
   While `dist < graspDist`, the confidence score increases by `buildRate`.

2. **Grasp**: When `score > graspThres`, the drone is grasped. The scalar projection of the drone onto the wand line is stored once:
   ```
   range = dot(v, D)              (dot product: how far along the wand line the drone sits,
                                   i.e. dot(v, D) = |v| * cos(angle between v and D))
   ```

3. **Follow**: Every subsequent packet computes the target:
   ```
   target = P + range · D
   ```
   The receiver commands this absolute position, keeping the drone fixed to the same point on the wand line as it moves.

4. **Release**: If no packet arrives for `lossTimeout` ms, the grasp is cleared and the drone hovers in place at its last commanded position.

5. **Auto-land**: If the drone is released below `landHeight`, it descends gradually and lands.

## Parameters

All receiver parameters are in the `receiver` group and are persistent across reboots.

| Parameter | Default | Description |
|---|---|---|
| `receiver.graspDist` | 0.15 m | Max perpendicular distance to wand line for a valid grasp attempt |
| `receiver.graspThres` | 30 | Score threshold (0–100) to trigger grasp |
| `receiver.buildRate` | 2 | Score added per packet while within `graspDist` |
| `receiver.lossTimeout` | 2000 ms | Time without wand packets before grasp is released |
| `receiver.landHeight` | 0.3 m | Altitude below which the drone auto-lands when not grasped |
| `receiver.appEnabled` | 1 | Set to 0 to hand control back to cfclient |
| `receiver.gotoDuration` | 0.2 s | *(HLC only)* Duration per `goTo` command |

Wand parameters:

| Parameter | Type | Description |
|---|---|---|
| `wand.buttonPressed` | uint8 | Live button state |
| `wand.melodyEnabled` | uint8 | Play melody on button press |

## LED feedback (receiver)

The bottom-mounted Color LED deck (`colorLedBot.wrgb8888`) indicates state:

| Color | State |
|---|---|
| Off | Idle / not grasped |
| Yellow | Grasp accumulating (within `graspDist`, score building) |
| Green | Grasped/following wand |
| Red | Auto-landing |
