# Button and Speed-Preset Settings

[한국어](../ko/buttons-presets.md)

> [!NOTE]
> This is the canonical English user guide maintained with the `carrot-wip` code. When user-visible behavior changes, update this document together with the related code and tests.

This page explains all **15 button and preset settings** from the current implementation, including short/long presses, custom cruise modes, and the conditions under which stock-SCC button messages are sent.

Change these values in **Carrot Web → Settings → Driving control → Buttons and presets**.

> [!CAUTION]
> Button events and stock-SCC messages differ by vehicle. Record the current values and confirm button recognition while safely parked. During a driving test, change only one item and restore normal mode immediately if the result is unexpected.

## Four sections

1. [Button modes](#button-modes)
2. [Speed units](#speed-units)
3. [Button tests and message bursts](#button-spam)
4. [Speed presets](#speed-presets)

## Key behavior to know first

- These functions apply only when the vehicle interface reports the corresponding button event.
- Short and long presses do not share the same increment. A **long press always uses a 10 km/h grid** in the current code.
- In custom modes 1–3, `CruiseSpeedUnit` matters more than `CruiseSpeedUnitBasic`.
- Resume logic can take priority over speed adjustment while cruise is off, ready, stopped, or returning from braking.
- On an mph display, the short-press unit is interpreted as mph and converted internally. The fixed long-press unit remains 10 km/h, about 6.2 mph.

### Why initial values may differ

Carrot Web restores an individual item using the `default` in `carrot_settings.json`, while newly created Params use `params_keys.h` initial values. This branch currently has these differences:

| Parameter | Catalog default | Initial Params value |
|---|---:|---:|
| `PaddleMode` | 1 | 0 |
| `CruiseSpeedUnitBasic` | 10 | 1 |
| `CruiseButtonTest1` / `2` / `3` | 0 / 0 / 0 | 8 / 30 / 1 |
| `CruiseSpeed1` through `5` | 10 / 10 / 10 / 10 / 10 | 30 / 50 / 80 / 110 / 130 |

The button-test catalog default of `0` is even below the displayed minimum. Do not force values to match a table; back up and use the current value on your own device as the baseline.

<a id="button-modes"></a>
## 1. Button modes

### `CruiseButtonMode`

This table describes a short press during normal driving with cruise already active.

| Value | Short RES/+ | Short SET/- |
|---:|---|---|
| `0` normal | Next `CruiseSpeedUnitBasic` grid point | Previous `CruiseSpeedUnitBasic` grid point |
| `1` custom 1 | Set 30 below 30 km/h, then next `CruiseSpeedUnit` point | Previous `CruiseSpeedUnit` point |
| `2` custom 2 | Same as custom 1 | Match current speed or, under some conditions, enter Carrot cruise |
| `3` custom 3 | First larger value from `CruiseSpeed1` through `5` | Same as custom 2 |

For custom modes 1–3, RES/+ first raises a set speed below 30 km/h to 30. It then searches the grid beginning at 40 km/h with `CruiseSpeedUnit` spacing. Custom mode 3 uses the five presets first and continues on the grid only after the final preset.

SET/- in custom modes 2 and 3 is not a simple subtraction:

- If actual speed is sufficiently above set speed, the set speed can move toward actual speed.
- If actual speed is below set speed, the set speed can be lowered to actual speed.
- If no lower-speed condition applies, the state can enter Carrot cruise.

Use `CruiseButtonMode=0` first if you want predictable grid-based `+/-` behavior.

### `CancelButtonMode`

| Value | Short cancel press |
|---:|---|
| `0` | Cancel cruise while retaining lateral control |
| `1` | Cancel cruise and disengage lateral control |

A **long cancel press always disengages lateral control**, regardless of this mode.

### `LfaButtonMode`

| Value | Short LFA press |
|---:|---|
| `0` normal | Toggle lateral control |
| `1` decelerate-to-stop and ready | Enable the automatic-deceleration/ready flow |
| `2` Carrot cruise | Activate Carrot cruise |

Mode `1` does not immediately command a fixed deceleration. It enables an internal paddle-deceleration waiting state; subsequent stop, lead, and cruise states determine Ready and reactivation behavior. Treat it as experimental because the current code still marks this flow for further work.

A **long LFA press** temporarily toggles the speed condition used by lane mode, independently of `LfaButtonMode`.

### `PaddleMode`

| Value | Current paddle behavior |
|---:|---|
| `0` | No separate paddle action |
| `1` | Cruise off, then Ready |
| `2` | Cruise off, then Ready plus automatic-deceleration state |
| `3` | Activate Carrot cruise |

> [!IMPORTANT]
> The catalog description says `0: cruise ON`, but the running code enters the paddle branch only when `PaddleMode > 0`. On this branch, interpret `0` as **paddle function disabled**.

Vehicles that do not report paddle events will not respond. Left and right paddles currently use the same mode behavior.

### Common long-press behavior

| Button | Long press |
|---|---|
| RES/+ or SET/- | Move repeatedly to the next or previous 10 km/h grid point |
| Following-gap | Cycle `MyDrivingMode`: eco → safe → normal → high speed |
| LFA | Temporarily toggle lane-mode speed application |
| Cancel | Cancel cruise and disengage lateral control |

<a id="speed-units"></a>
## 2. Speed units

| Setting | Range | Actual role |
|---|---:|---|
| `CruiseSpeedUnitBasic` | 1–100 | Normal-mode grid for short RES/SET presses |
| `CruiseSpeedUnit` | 1–100 | Custom-mode grid, SET/- unit, and the unit after the last preset |
| `CruiseButtonLongDelay` | 30–100 | Control frames before RES/SET is considered a long press |

The code rounds to the next grid point rather than simply adding the unit. With normal mode, a unit of 5, and a set speed of 83 km/h, short RES/+ selects 85 and short SET/- selects 80.

### `CruiseButtonLongDelay` is measured in frames

Button processing runs at about 100 Hz:

| Stored value | RES/SET long press | Gap/LFA/cancel long press |
|---:|---:|---:|
| 30 | About 0.30 s | About 0.60 s |
| 40 | About 0.40 s | About 0.70 s |
| 60 | About 0.60 s | About 0.90 s |

Gap, LFA, and cancel add 30 frames internally. Perceived timing can vary with the vehicle's button-message rate.

> [!NOTE]
> Neither speed-unit setting changes the fixed 10 km/h long-press grid.

Recommended order: verify normal mode and `CruiseSpeedUnitBasic=1`, tune only the long-press delay, and then test a custom mode with `CruiseSpeedUnit`.

<a id="button-spam"></a>
## 3. Button tests and message bursts

`CruiseButtonTest1` through `3` do not adjust physical button sensitivity. They control virtual RES/SET messages used to synchronize the set speed of **Hyundai/Kia stock SCC** with the desired speed.

| Setting | Initial Params value | Role |
|---|---:|---|
| `CruiseButtonTest1` | 8 | Consecutive transmissions allowed in one burst |
| `CruiseButtonTest2` | 30 | Rest frames after a burst or driver button input |
| `CruiseButtonTest3` | 1 | Number of message copies for one allowed CAN FD transmission |

At roughly 0.01 seconds per control frame, `Test2=30` is about 0.30 seconds. The code may instead use a separate seven-frame wait after detecting that the stock set speed changed.

Scope and exclusions:

- Used for stock-SCC speed synchronization on Hyundai/Kia configurations without openpilot longitudinal control.
- `Test1` and `Test2` can affect both classic CAN and CAN FD button paths.
- `Test3` duplication currently applies to the CAN FD path.
- With `SpeedFromPCM=1`, the RES/SET branch that corrects the difference between desired and stock set speeds does not run.
- These values do nothing on vehicles that do not use this transmission path.

> [!WARNING]
> Values that are too large can flood button messages or make stock SCC miss inputs. Values that are too small can slow or prevent synchronization. Keep the initial Params values `8 / 30 / 1` if there is no problem.

<a id="ka4-stock-scc-standstill"></a>
### Experimental KA4 stock-SCC standstill extension

This behavior is applied automatically, without a user setting, only when all of the following are identified. `Ka4StockSccStandstillRearm` remains internal state for log classification and compatibility with older installations; it is not shown as a Carrot Web switch.

- Fourth-generation Kia Carnival KA4 (the current vehicle-validation target is model year 2023)
- CAN FD, PCM cruise, and stock radar SCC
- No openpilot longitudinal control and no camera SCC

It attempts short RES groups only after the vehicle is physically near zero speed, SCC is active, and a nearby stopped lead is stable. Raw vehicle speed must be at most `0.03 m/s`, lead distance must be greater than zero and at most `20 m`, absolute relative speed must be at most `0.5 m/s`, and there must be no SCC failure or takeover request. The normal first request is about 2.5 seconds into the qualified stop and subsequent requests are at least about 2.5 seconds apart. If the stock `ALERTS_5=5` resume prompt is already present, a recovery request is advanced to follow the 0.30-second qualification dwell. The last group finishes no later than 27 seconds so the controller does not queue more requests beyond the 30-second boundary.

On HDA1 vehicles where this branch replaces the cluster message, only the `ALERTS_5=5` resume-prompt field is masked from the start of an SCC-enabled stop (or from the first already-active prompt) until just before 30.00 seconds. This display timer is independent from the lead, brake, and Auto Hold gates for RES requests, so a transient gate change cannot leak the prompt immediately after stopping. Every other OEM alert, sound, DAW, and mute field is preserved, and the prompt is restored at 30.00 seconds. HDA2 does not use this cluster-message replacement path.

Brake, accelerator, Auto Hold, parking brake, SET/RES/LFA buttons, or an unsafe lead state immediately abort synthetic RES requests, while the display timer remains tied to the same stopped epoch until the vehicle moves. SCC faults, takeover requests, CANCEL/main buttons, software cancel requests, or invalid CAN abort requests and restore the prompt immediately; the display timer cannot restart until the vehicle actually moves. The alert-field masking is deterministic in code, but in-vehicle validation has not yet established that the stock SCC ECU accepts synthetic RES or that the actual auto-resume window is extended. The driver must verify the cluster and road conditions and test only in a controlled environment.

With separate `CarrotValidationAutoUpload` consent, the device can automatically select nearby full rlogs after the drive. Those logs record the experimental RES frame-request count and qualification state. That count means the controller appended a frame to its outgoing CAN list; it does not prove Panda transmission or acceptance by the stock SCC ECU. See [Sending Dashcam Logs for Analysis](dashcam-log-sharing.md#automatic-validation-upload) for upload scope and privacy details.

<a id="speed-presets"></a>
## 4. Speed presets

`CruiseSpeed1` through `CruiseSpeed5` form an ordered table used by short RES/+ presses in `CruiseButtonMode=3`.

| Setting | Initial Params value | Example role |
|---|---:|---|
| `CruiseSpeed1` | 30 | Low-speed/local road |
| `CruiseSpeed2` | 50 | Urban road |
| `CruiseSpeed3` | 80 | Limited-access road |
| `CruiseSpeed4` | 110 | Highway |
| `CruiseSpeed5` | 130 | Final custom step |

These role names are examples, not legal limits or recommended speeds. Use values appropriate for local law and conditions.

The code reads values 1 through 5 and selects the **first one greater than the current set speed**. After all presets, it moves to the next `CruiseSpeedUnit` grid point. Presets are not sorted automatically, so save them in ascending order; duplicates or reversed values may skip steps.

### Special behavior when `CruiseSpeed1=0`

Only preset 1 allows zero. It then becomes a dynamic value based on the current road limit:

- `AutoRoadSpeedLimitOffset >= 0`: road limit + offset
- `AutoRoadSpeedLimitOffset < 0`: road limit × `AutoNaviSpeedSafetyFactor`

For a 60 km/h limit, an offset of 5 produces 65 km/h. An offset of -1 with a 105% safety factor produces 63 km/h. This preset cannot work as expected without valid road-limit input.

Remember that SET/- in custom mode 3 does not walk backward through the preset table.

## Quick troubleshooting

| Symptom | Check first |
|---|---|
| A short press changes speed by more than expected | `CruiseButtonMode`, both speed units |
| Long-press unit differs from the setting | Current code uses a fixed 10 km/h grid |
| Custom mode 3 skips steps | Ascending order and duplicates in presets 1–5 |
| SET/- enters Carrot cruise | This can be normal in custom modes 2 and 3 |
| LFA or paddle does not respond | Whether the vehicle reports that button event |
| Reset produces a surprising value | Difference between catalog default and initial Params value |

## Code references

- Button/preset state machine: `openpilot/selfdrive/car/cruise.py`
- Hyundai/Kia stock-SCC messages: `opendbc_repo/opendbc/car/hyundai/carcontroller.py`
- Catalog ranges/defaults: `openpilot/selfdrive/carrot_settings.json`
- Initial Params values: `openpilot/common/params_keys.h`

[Back to Understanding Settings](settings.md)
