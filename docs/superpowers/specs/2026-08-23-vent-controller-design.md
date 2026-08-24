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
    Uptime.update(millis());
    Buttons.update();
    RoomSensor.update();
    if (mode = HVAC_COOL) { VentLogic.handleCoolCycle(); }
    else                  { VentLogic.handleHeatCycle(); }
    VentDisplay.update();
}
```

`Uptime.update()` takes the reading as a parameter rather than calling `millis()`
itself. That one deviation from the no-argument pattern is what makes the epoch
arithmetic — the trickiest code in the project — testable on the host by feeding
it a synthetic sequence. Every other scope reads its own hardware and delegates
the decision to a pure inner function.

## Time base

`millis()` returns `u32` and wraps every 49.7 days. Rather than depend on
observing that wrap, `Uptime` keeps its own 10-day epoch, which always rolls
first. `epochCount` therefore counts complete 10-day periods of uptime, and no
reading is ever subtracted across a discontinuity.

```c-next
scope Uptime {
    const u32 EPOCH_MILLISECONDS <- 864000000;   // 10 days

    u32 epochBase <- 0;      // millis() reading when the current epoch began
    u32 epochCount <- 0;     // completed 10-day epochs
    u32 withinEpoch <- 0;    // milliseconds elapsed inside the current epoch

    void update(u32 now) {
        if (now < epochBase) {
            // millis() moved backward: the hardware wrapped, or a reading was
            // lost. Start a fresh epoch instead of subtracting across it.
            epochBase <- now;
            withinEpoch <- 0;
            epochCount +<- 1;
            return;
        }

        withinEpoch <- now - epochBase;

        if (withinEpoch >= EPOCH_MILLISECONDS) {
            withinEpoch <- withinEpoch - EPOCH_MILLISECONDS;
            epochBase <- now - withinEpoch;
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

Carrying the remainder on an epoch roll — rather than zeroing `withinEpoch` —
keeps `milliseconds()` exactly continuous across the boundary. `now - withinEpoch`
cannot underflow, because the new `withinEpoch` is always less than `now`.

Every interval in the firmware compares `u64` values from `milliseconds()`. The
value never decreases, so C-Next's default clamp arithmetic is correct
throughout and no `wrap` declaration is needed anywhere.

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

The setpoint is the limit the room is not pushed past. Hysteresis is how far the
room drifts back before the vent reopens.

In cooling, with a setpoint of 18.0 °C and 1.0 °C of hysteresis: the vent closes
when the room reaches 18.0 °C and reopens only once it has drifted up to
19.0 °C. The room lives in the band 18.0–19.0 °C and never goes below the
setpoint. Heating mirrors this: the vent closes at the setpoint and reopens a
band below it, so the room never goes above.

This orientation is deliberate. The complaint is over-cooling, so the setpoint
is a floor in cooling mode, not a midpoint. Centring the band on the setpoint
instead would let the room reach 17.5 °C.

```c-next
bool shouldOpenForCooling(f32 temperature, f32 setpoint, f32 hysteresis,
                          bool currentlyOpen, bool readingValid) {
    if (!readingValid) { return false; }
    if (currentlyOpen) { return temperature > setpoint; }
    return temperature > (setpoint + hysteresis);
}

bool shouldOpenForHeating(f32 temperature, f32 setpoint, f32 hysteresis,
                          bool currentlyOpen, bool readingValid) {
    if (!readingValid) { return false; }
    if (currentlyOpen) { return temperature < setpoint; }
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

Hysteresis of 1.0 °C against a sensor resolving 0.1 °C also removes any
possibility of boundary chatter, so the decision can run every loop without
cycling the actuator.

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
| Hysteresis | 1.0 °C, both modes |
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

**Uptime** — the epoch rolls at exactly 864,000,000; `milliseconds()` is
continuous across a roll; a backward `millis()` reading starts a fresh epoch and
increments the count; the value never decreases across a long synthetic
sequence including several rolls.

**VentLogic, cooling** — a closed vent stays closed at the setpoint and up to
`setpoint + hysteresis`; it opens above that; an open vent stays open down to the
setpoint and closes at it; an invalid reading returns `false` in every
combination of the other inputs.

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
4. Verify the transpiler accepts `total[32, 32] <- epochCount` — a 32-bit-wide
   bit write into a `u64` is documented but is not used anywhere in the existing
   projects.
5. Verify `"target": "avr"` is accepted by cnext 0.2.18.
6. Confirm the Uno's 5V supply and whether it is powered independently of the
   air handler.
7. Site the BME280 away from the register and out of the controller enclosure —
   self-heating and discharge air both defeat the control loop regardless of the
   firmware.

## Open

The 24VAC source — the air handler's own transformer versus a separate supply —
is not yet decided. It does not affect the firmware.
