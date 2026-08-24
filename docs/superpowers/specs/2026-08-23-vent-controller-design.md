# Vent Controller — Design

**Date:** 2026-08-23
**Status:** Draft, awaiting review

## Problem

A central air conditioner over-cools one room. Its thermostat lives elsewhere in
the house, so it keeps calling for cooling long after this room is cold. A
normally-closed damper on the room's supply vent, driven by its own temperature
sensor, closes off the cold air once the room reaches its target.

## Hardware

| Part | Role | Connection |
| --- | --- | --- |
| Arduino Uno (ATmega328P) | Controller | 32KB flash, 2KB SRAM |
| Waveshare BME280 | Room temperature | Hardware SPI |
| H_Relay opto-isolated relay module | Switches 24VAC | One digital output |
| Spring-return damper actuator | Power-to-open, normally closed | 24VAC through the relay |
| 16x2 character LCD, PCF8574 backpack | Status and setpoint | I2C |
| 2 momentary push buttons | Setpoint and mode | Two digital inputs, `INPUT_PULLUP` |

The BME280 is on SPI rather than I2C so the sensor and the display do not share
a bus. A wedged PCF8574 backpack cannot then take the temperature reading down
with it.

### Pin map

| Pin | Function |
| --- | --- |
| D0, D1 | Serial (115200) |
| D2 | Up button, `INPUT_PULLUP` |
| D3 | Down button, `INPUT_PULLUP` |
| D7 | Relay output |
| D10 | BME280 chip select |
| D11, D12, D13 | SPI MOSI, MISO, SCK |
| A4, A5 | I2C SDA, SCL (LCD) |

## Scope

**In:** one room, one damper, cooling and heating modes, a local display and two
buttons, fail-closed behaviour on a bad reading.

**Out:** networking, scheduling, a real-time clock, multiple zones, humidity or
pressure use, persistent settings.

## Architecture

```
src/
├── main.cnx                      # Arduino entry, delegates to VentController
├── AppConfig.cnx                 # pins, limits, defaults
├── Domain/
│   ├── VentController.cnx        # orchestrator, owns mode and setpoints
│   └── VentLogic.cnx             # cool/heat cycle handlers + pure predicates
├── Data/
│   ├── Elapsed.cnx               # pure wrap-safe millisecond subtraction
│   ├── Stopwatch.cnx             # ElapsedMilliseconds — resettable elapsed counters
│   ├── RoomSensor.cnx            # Adafruit_BME280 over SPI, validity gate
│   └── types/EHvacMode.cnx       # HVAC_COOL, HVAC_HEAT
└── Display/
    ├── ButtonLogic.cnx           # pure debounce and gesture state machine
    ├── Buttons.cnx               # reads pins, delegates to ButtonLogic
    ├── VentDisplay.cnx           # 16x2 rendering
    ├── SerialLog.cnx             # 1 Hz CSV status line
    └── VentRelay.cnx             # relay pin, single source of truth for vent state
```

Generated `.cpp` and `.hpp` files are committed alongside their `.cnx` sources,
matching the convention in `ossm` and `ovgt`.

### The loop

`main.cnx` holds `setup()` and `loop()` and does nothing but delegate to
`VentController`, matching `ossm`. The loop below is `VentController.loop()`,
where `mode` is a scope member and resolves bare.

```c-next
void loop() {
    Buttons.update();
    RoomSensor.update();
    if (mode = HVAC_COOL) { VentLogic.handleCoolCycle(); }
    else                  { VentLogic.handleHeatCycle(); }
    VentDisplay.update();
    SerialLog.update();
}
```

Every scope reads its own hardware and delegates the decision to a pure inner
function, so the loop names what happens without describing how.

Each `update()` also owns its own cadence. It holds an `ElapsedMilliseconds`,
checks it first, and returns immediately when it is not yet due:

```c-next
void update() {
    u32 sinceRead <- Stopwatch.elapsed(readTimer);
    if (sinceRead < READ_INTERVAL_MILLISECONDS) { return; }
    Stopwatch.reset(readTimer);
    // ... do the work
}
```

The alternative — holding every timer in the loop and gating each call there —
puts each subsystem's cadence in the orchestrator and invites an `else if` chain
between them, where two tasks due in the same pass compete and the slower one
pushes the others behind it. Self-gating decouples them: the timers live beside
the code they pace, and nothing can starve anything else.

`Buttons.update()` is the exception that shows the rule. It runs at full loop
rate because debouncing needs to *sample* continuously; its timer measures how
long a level has been stable rather than gating how often the function runs.

Resetting a counter to zero rather than advancing its base by the interval means
each period is the interval plus however long the handler took — about 2 ms per
second here, which nothing in this project notices. Teensy's `elapsedMillis`
offers `-=` for cadences that do care; if one ever appears, it is a six-line
`Elapsed.advanced()` beside `Elapsed.between()`.

## Time base

The Teensy's `elapsedMillis` is the model, and this mirrors it as closely as
C-Next allows:

```cpp
elapsedMillis sinceRead;
if (sinceRead > 1000) { sinceRead = 0; readSensor(); }
```

It stores one `unsigned long` — the `millis()` value at the last reset — and
reads back `millis() - base`. Nothing accumulates, so a missed update cannot
corrupt it and there is no global clock to keep.

The one thing that does not carry over is the arithmetic. `millis() - base` is
correct across the 49.7-day rollover only because C's unsigned subtraction
wraps; C-Next defaults to clamp, which saturates that subtraction to zero and
stalls every timer for 49.7 days. `wrap u32` would restore it while hiding the
rollover where nothing can observe or test it. So the wrap becomes one explicit
branch, and everything else is `elapsedMillis` unchanged.

```c-next
// Elapsed.cnx — pure, no Arduino.h, host-testable
scope Elapsed {
    u32 between(u32 base, u32 now) {
        if (now >= base) { return now - base; }
        // millis() wrapped since base was taken: the step from base up
        // through the u32 ceiling, then from zero up to now.
        return (4294967295 - base) + now + 1;
    }
}
```

```c-next
// Stopwatch.cnx
struct ElapsedMilliseconds {
    u32 base;
}

scope Stopwatch {
    u32 elapsed(const ElapsedMilliseconds counter) {
        u32 now <- millis();
        return Elapsed.between(counter.base, now);
    }

    void reset(ElapsedMilliseconds counter) {
        counter.base <- millis();
    }
}
```

The wrap expression cannot overflow. It runs only when `now < base`, so
`(4294967295 - base) + now + 1` is at most `4294967295` exactly, and clamp never
engages anywhere in the project.

Structs are data containers in C-Next, so the operations live in a scope beside
the type rather than on it. A written parameter transpiles to a mutable pointer
and a read-only struct parameter auto-consts (ADR-006), so `reset()` updates the
caller's counter with no pointer syntax.

`Elapsed.between()` is split into its own file because it holds the only
non-obvious arithmetic in the project and must compile without `Arduino.h` to be
tested on the host. `Stopwatch` is the two-line hardware wrapper over it.

Usage is the `elapsedMillis` pattern, with the comparison extracted to satisfy
MISRA 13.5:

```c-next
ElapsedMilliseconds readTimer;

void update() {
    u32 sinceRead <- Stopwatch.elapsed(readTimer);
    if (sinceRead < READ_INTERVAL_MILLISECONDS) { return; }
    Stopwatch.reset(readTimer);
    // ... take a reading
}
```

Five behaviours use one: the 1 s sensor read, the 25 ms debounce, the 600 ms
hold delay and 150 ms repeat, the 250 ms redraw, and the fault duration shown on
the display.

This inherits `elapsedMillis`'s own limitation — an interval longer than 49.7
days is indistinguishable from a short one. Every interval here is at most one
second.

## Sensor

`RoomSensor` wraps `Adafruit_BME280` in SPI mode and reads once per second.
Temperature is `f32` in degrees Celsius, as the sensor reports it — there is no
conversion layer.

A reading is valid when the sensor initialised and the value falls inside the
BME280's own specified range:

```c-next
bool inRange <- (temperature > -40.0) && (temperature < 85.0);
```

C-Next has no NaN concept, and this check does not need one. A failed read, a
miswired bus returning `0xFF`, and an out-of-spec value all fail the same
comparison, and a value that fails it can never reach the relay.

## Control logic

The setpoint is the middle of the band. Hysteresis is the distance from the
setpoint to each edge, so the total band is twice the hysteresis.

In cooling, with a setpoint of 18.0 °C and 1.0 °C of hysteresis: the vent opens
when the room rises to 19.0 °C and closes when it falls to 17.0 °C. The room
swings across 17.0–19.0 °C. Heating mirrors it: the vent opens when the room
falls to 17.0 °C and closes when it rises to 19.0 °C.

```c-next
bool shouldOpenForCooling(f32 temperature, f32 setpoint, f32 hysteresis,
                          bool currentlyOpen, bool readingValid) {
    if (!readingValid) { return false; }
    if (currentlyOpen) { return temperature > (setpoint - hysteresis); }
    return temperature > (setpoint + hysteresis);
}

bool shouldOpenForHeating(f32 temperature, f32 setpoint, f32 hysteresis,
                          bool currentlyOpen, bool readingValid) {
    if (!readingValid) { return false; }
    if (currentlyOpen) { return temperature < (setpoint + hysteresis); }
    return temperature < (setpoint - hysteresis);
}
```

The two `handleXCycle()` functions read the sensor and the current relay state,
call their predicate, and set the relay. They are four lines each. A shared
helper would need a direction flag that reintroduces the same branch one level
down, so they stay written out.

Both predicates return `false` for an invalid reading, and `false` closes the
vent. Fail-closed is the default path rather than a special case: a dead sensor,
a wedged bus, and a satisfied room all reach the same code.

A 2.0 °C band against a sensor resolving 0.1 °C also removes any possibility of
boundary chatter, so the decision can run every loop without cycling the
actuator.

## Relay

`VentRelay` owns the pin and is the single source of truth for vent state;
`VentLogic` and `VentDisplay` both read `isOpen()` from it. The pin is driven to
the closed state as the first action in `setup()`.

Between reset and `setup()`, an Uno's pins are high-impedance inputs. The
H_Relay module's active level must be measured before 24VAC is connected, and an
external pull resistor added to hold the safe level through that window.

Every failure — dead sensor, dead board, brownout, cut power — lands on a closed
damper, because the actuator is spring-return and the relay de-energises.

## Buttons

`ButtonLogic` is a pure state machine; `Buttons` reads the two pins and passes
their raw levels and the current time into it.

- Each pin is accepted only after 25 ms of stable sampling.
- Both buttons down toggles the mode, once per press.
- One button down steps the active setpoint by ±0.1 °C.
- Holding repeats after 600 ms at 150 ms intervals, so a 4 °C move is a hold
  rather than 40 presses.

Debouncing is what makes the both-down gesture reliable. Two contacts a human
closes together settle inside a single 25 ms window, so "both down" is a state
of the debounced pair rather than a race needing its own timeout.

## Display

```
+----------------+     +----------------+
|18.4C   SET 18.0|     |SENSOR FAULT    |
|COOL VENT CLOSED|     |VENT CLOSED  12m|
+----------------+     +----------------+
```

Line 1 is the current temperature and the setpoint for the active mode. Line 2
is the mode, left-justified, and the vent state, right-justified to column 15.
The fault screen replaces both lines whenever the reading is invalid, and shows
how long the fault has persisted. Redraw runs at 250 ms.

The fault screen exists because fail-closed makes a dead sensor and a satisfied
room look identical from outside the box.

## Configuration

| Setting | Value |
| --- | --- |
| Cooling setpoint | 18.0 °C |
| Heating setpoint | 20.0 °C |
| Hysteresis | 1.0 °C either side of setpoint, both modes (2.0 °C band) |
| Setpoint range | 10.0 °C to 30.0 °C |
| Setpoint step | 0.1 °C |
| Mode at boot | `HVAC_COOL` |
| Sensor read interval | 1000 ms |
| Button debounce | 25 ms |
| Hold repeat | 600 ms delay, 150 ms interval |
| Display redraw | 250 ms |
| Serial log interval | 1000 ms |
| Watchdog | 2 s |

There is no EEPROM use. Setpoints and mode live in RAM and return to these
defaults on power loss.

### Two shapes, because they are two different things

`scope AppConfig` holds only values that are fixed at build time — defaults,
limits, step sizes, intervals. They are `public const`, they belong to no
instance, and nothing can write to them. Measured on the Uno they cost nothing:
every one folds into its use site and `--gc-sections` drops the definitions, so
`avr-nm` finds no `AppConfig` symbol in the ELF at all.

Runtime state — the active cooling and heating setpoints and the current mode —
goes in an `AppData` struct instead. It is the state that buttons mutate and
that `VentLogic` and `VentDisplay` read, so passing it explicitly makes the data
flow visible rather than reaching it as ambient scope variables.

The split is the point: a constant that can never change and a value that
changes at runtime are different kinds of thing, and giving them the same shape
loses that signal.

`AppData` is introduced when there is state to put in it, not before. At Task 1
there is none.

Hysteresis starts at 1.0 °C and is expected to be tuned against how the room
actually behaves.

## Build

`platformio.ini` carries two environments:

```ini
[env:uno]
platform = atmelavr
board = uno
framework = arduino
extra_scripts = pre:cnext_build.py
monitor_speed = 115200
build_flags = -I include
lib_deps =
    adafruit/Adafruit BME280 Library
    adafruit/Adafruit Unified Sensor
    adafruit/Adafruit BusIO
    marcoschwartz/LiquidCrystal_I2C
    SPI
    Wire

[env:native]
platform = native
test_framework = unity
test_build_src = yes
build_flags = -I src -I include
; build_src_filter is filled in with the generated Elapsed/VentLogic/ButtonLogic
; files once they exist; listing them explicitly keeps Arduino code out.
```

`cnext_build.py` is `ossm`'s, unchanged. `cnext.config.json`:

```json
{
    "cppRequired": true,
    "target": "avr",
    "include": ["include/", ".pio/libdeps/"],
    "headerOut": "include",
    "basePath": "src"
}
```

`[env:native]` compiles only the generated files for `Elapsed`, `VentLogic`, and
`ButtonLogic` — none of which include `Arduino.h` — through `build_src_filter`.

Flash budget is roughly 4KB core and Wire, 10KB for the Adafruit BME280 stack,
2KB for the LCD, and 4KB of project code: about 20KB of the Uno's 32KB.

## Testing

Unity tests running on the host:

**Elapsed** — `between()` returns the plain difference when `now >= base`;
returns zero when they are equal; returns the true interval across a wrap (base
4,294,967,000 with now 204 gives 500); and the worst case, `base` at
`4,294,967,295` with `now` one below it, returns `4,294,967,295` without
saturating.

**VentLogic, cooling** — a closed vent stays closed up to `setpoint +
hysteresis` and opens above it; an open vent stays open down to `setpoint -
hysteresis` and closes at it; the vent does not change state anywhere inside the
band; an invalid reading returns `false` in every combination of the other
inputs.

**VentLogic, heating** — the mirror of the above, which pins the direction so a
later change cannot silently invert it.

**ButtonLogic** — a single-sample glitch is rejected; both down yields one mode
toggle per press; one down yields one step; a hold produces repeats at the
specified delay and interval.

## Bring-up

1. Confirm the Waveshare board's SPI pinout against its silkscreen.
2. Scan I2C for the LCD backpack — 0x27 or 0x3F — and confirm the
   `LiquidCrystal_I2C` fork, since several incompatible ones share the name.
3. Measure the H_Relay module's active level, then add the pull resistor for the
   pre-`setup()` window. Do this before 24VAC is connected.
4. Verify `"target": "avr"` is accepted by cnext 0.2.18.
5. Confirm the Uno's 5V supply and whether it is powered independently of the
   air handler.
6. Site the BME280 away from the register and out of the controller enclosure —
   self-heating and discharge air both defeat the control loop regardless of the
   firmware.

## Open

The 24VAC source — the air handler's own transformer versus a separate supply —
is not yet decided. It does not affect the firmware.
