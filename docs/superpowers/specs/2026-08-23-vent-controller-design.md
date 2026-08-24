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
│   ├── Uptime.cnx                # monotonic 64-bit millisecond time base
│   ├── Stopwatch.cnx             # ElapsedMilliseconds — resettable elapsed counters
│   ├── RoomSensor.cnx            # Adafruit_BME280 over SPI, validity gate
│   └── types/EHvacMode.cnx       # HVAC_COOL, HVAC_HEAT
└── Display/
    ├── ButtonLogic.cnx           # pure debounce and gesture state machine
    ├── Buttons.cnx               # reads pins, delegates to ButtonLogic
    ├── VentDisplay.cnx           # 16x2 rendering
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
    u32 now <- millis();
    Uptime.update(now);
    Buttons.update();
    RoomSensor.update();
    if (mode = HVAC_COOL) { VentLogic.handleCoolCycle(); }
    else                  { VentLogic.handleHeatCycle(); }
    VentDisplay.update();
}
```

`Uptime.update()` takes the reading as a parameter rather than calling `millis()`
itself, and the reading is saved to a variable rather than passed inline. That
one deviation from the no-argument pattern is what makes the epoch arithmetic —
the trickiest code in the project — testable on the host by feeding it a
synthetic sequence. Every other scope reads its own hardware and delegates the
decision to a pure inner function.

## Time base

The Teensy's `elapsedMillis` is the model — a counter read as "how long since
this last happened" and reset by assigning zero:

```cpp
elapsedMillis sinceRead;
if (sinceRead > 1000) { sinceRead = 0; readSensor(); }
```

Its implementation is `millis() - base`, which stays correct across the 49.7-day
`millis()` rollover only because C's unsigned subtraction wraps. C-Next defaults
to clamp arithmetic, so at the rollover that subtraction saturates to zero and
every timer in the firmware stalls for 49.7 days. Declaring `wrap u32` restores
the arithmetic but puts the rollover somewhere nothing can observe or test.

So the ergonomics are kept and the foundation is replaced. `Uptime` accumulates
elapsed steps into a monotonic 64-bit millisecond count that is exact across the
hardware wrap. `ElapsedMilliseconds` then provides `elapsedMillis` semantics on
top of a clock that never goes backward, where the subtraction cannot underflow.

### Uptime

`withinEpoch` resets every 10 days and `epochCount` counts complete 10-day
periods of uptime.

```c-next
scope Uptime {
    const u32 EPOCH_MILLISECONDS <- 864000000;   // 10 days

    u32 lastReading <- 0;    // previous millis() reading
    u32 withinEpoch <- 0;    // milliseconds elapsed inside the current epoch
    u32 epochCount <- 0;     // completed 10-day epochs

    void update(u32 now) {
        u32 delta <- 0;
        if (now >= lastReading) {
            delta <- now - lastReading;
        } else {
            // millis() wrapped: the step from lastReading up through the u32
            // ceiling, then from zero up to now.
            delta <- (4294967295 - lastReading) + now + 1;
        }
        lastReading <- now;

        withinEpoch <- withinEpoch + delta;
        while (withinEpoch >= EPOCH_MILLISECONDS) {
            withinEpoch <- withinEpoch - EPOCH_MILLISECONDS;
            epochCount +<- 1;
        }
    }

    u64 milliseconds() {
        u64 total <- epochCount;
        total <- total * EPOCH_MILLISECONDS;
        total <- total + withinEpoch;
        return total;
    }
}
```

The wrap expression cannot overflow: it runs only when `now < lastReading`, so
`(4294967295 - lastReading) + now + 1` is at most `4294967295` exactly.

The `while` handles a `delta` spanning more than one epoch, which a stalled loop
could produce. `update()` must be called at least once per `millis()` period for
the wrap to be seen at all; the main loop satisfies that by four orders of
magnitude.

### ElapsedMilliseconds

```c-next
struct ElapsedMilliseconds {
    u64 base;
}

scope Stopwatch {
    u64 elapsed(const ElapsedMilliseconds counter) {
        u64 now <- Uptime.milliseconds();
        return now - counter.base;
    }

    void reset(ElapsedMilliseconds counter) {
        counter.base <- Uptime.milliseconds();
    }
}
```

Structs are data containers in C-Next, so the operations live in a scope beside
the type rather than on it. A parameter that is written transpiles to a mutable
pointer and a read-only struct parameter auto-consts (ADR-006), so `reset()`
updates the caller's counter with no pointer syntax.

Usage is the `elapsedMillis` pattern, with the comparison extracted to satisfy
MISRA 13.5:

```c-next
ElapsedMilliseconds readTimer;

void update() {
    u64 sinceRead <- Stopwatch.elapsed(readTimer);
    if (sinceRead < READ_INTERVAL_MILLISECONDS) { return; }
    Stopwatch.reset(readTimer);
    // ... take a reading
}
```

Five behaviours use one: the 1 s sensor read, the 25 ms debounce, the 600 ms
hold delay and 150 ms repeat, the 250 ms redraw, and the fault duration shown on
the display. Each would otherwise hand-roll the same comparison.

Because the clock only ever increases, C-Next's default clamp arithmetic is
correct throughout and no `wrap` declaration appears anywhere in the project.

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
| Watchdog | 2 s |
| Uptime epoch | 10 days |

There is no EEPROM use. Setpoints and mode live in RAM and return to these
defaults on power loss.

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
; build_src_filter is filled in with the generated Uptime/VentLogic/ButtonLogic
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

`[env:native]` compiles only the generated files for `Uptime`, `VentLogic`, and
`ButtonLogic` — none of which include `Arduino.h` — through `build_src_filter`.

Flash budget is roughly 4KB core and Wire, 10KB for the Adafruit BME280 stack,
2KB for the LCD, and 4KB of project code: about 20KB of the Uno's 32KB.

## Testing

Unity tests running on the host:

**Uptime** — the epoch rolls at exactly 864,000,000; a `millis()` wrap yields
the exact elapsed step rather than an invented one (4,294,967,295 followed by 0
advances `milliseconds()` by 1); a single `delta` spanning more than one epoch
rolls the count more than once; the value never decreases across a long
synthetic sequence including several epoch rolls and several hardware wraps.

**Stopwatch** — `elapsed()` reads zero immediately after `reset()`; it tracks
the clock as `Uptime` advances; two counters reset independently; a counter
spanning an epoch roll and a hardware wrap still reports the true interval.

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
4. Verify the transpiler accepts `u64 total <- epochCount` followed by
   `total * EPOCH_MILLISECONDS` — widening a `u32` into `u64` and multiplying
   by a `u32` constant, which MISRA 10.4 essential-type rules could reject.
5. Verify `"target": "avr"` is accepted by cnext 0.2.18.
6. Confirm the Uno's 5V supply and whether it is powered independently of the
   air handler.
7. Site the BME280 away from the register and out of the controller enclosure —
   self-heating and discharge air both defeat the control loop regardless of the
   firmware.

## Open

The 24VAC source — the air handler's own transformer versus a separate supply —
is not yet decided. It does not affect the firmware.
