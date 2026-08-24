# Vent Controller Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** An Arduino Uno that closes a normally-closed supply damper once a room reaches its target temperature, so central A/C stops over-cooling it.

**Architecture:** A thin `VentController` loop calls one `update()` per subsystem; each subsystem owns its own hardware and its own cadence via an `ElapsedMilliseconds` counter modelled on Teensy's `elapsedMillis`. All non-obvious arithmetic lives in pure scopes with no `Arduino.h`, which compile into a native Unity environment for host tests. Every failure path leaves the relay de-energised and the damper spring-closed.

**Tech Stack:** PlatformIO (`atmelavr`/`uno`), C-Next 0.2.18 transpiling to C++, Adafruit_BME280 over hardware SPI, LiquidCrystal_I2C, Unity for host tests.

**Spec:** `docs/superpowers/specs/2026-08-23-vent-controller-design.md`

## Global Constraints

- **Celsius everywhere.** No Fahrenheit in code, output, comments, or docs.
- **Temperatures are `f32`**, as the BME280 reports them. No fixed-point conversion layer.
- **No EEPROM writes.** Setpoints and mode live in RAM and reset to defaults on power loss.
- **No NaN.** C-Next has no NaN concept; reading validity is a range check, `(temperature > -40.0) && (temperature < 85.0)`.
- **No `wrap` declarations.** Every subtraction is guarded so C-Next's default clamp arithmetic is correct.
- **Never work around a C-Next defect.** If the transpiler rejects valid syntax or miscompiles, build a minimal reproducer under `~/code/c-next2/bugs/issue-<name>/`, show the user, and block the dependent step. No C++ shims, no edits to generated files.
- **Spell names out.** No invented abbreviations in any identifier. Only universally standard ones (`RPM`, `URL`, `ID`, `CAN`, `ADC`, `PWM`) are allowed.
- **MISRA 13.5:** no function calls inside `if` conditions. Extract to a variable first.
- **Commit generated files.** `cnext` emits `.cpp` beside each `.cnx` and a flat `.hpp` into `include/`. Both are committed, matching `ossm` and `ovgt`.
- **C-Next syntax reminders:** `<-` assigns, `=` compares, `[a, b]` indexes bits, enum members are qualified (`EHvacMode.HVAC_COOL`).

## Verified transpiler behaviour

These were confirmed against cnext 0.2.18 before this plan was written. Do not re-derive them.

| Behaviour | Result |
| --- | --- |
| `scope Elapsed { u32 between(...) }` | → `uint32_t Elapsed_between(uint32_t base, uint32_t now)` |
| Header output path | **Flattened** — `src/Data/Elapsed.cnx` → `include/Elapsed.hpp`, not `include/Data/`. Include as `<Elapsed.hpp>`. |
| `.cpp` output path | Beside the source: `src/Data/Elapsed.cpp` |
| Struct parameter, written | → `ElapsedMilliseconds&` (a C++ reference, not a pointer) |
| Struct parameter, read-only | → `const ElapsedMilliseconds&` |
| Constructor arg, bare literal — `Sensor s(10)` | **Parse error.** Bind a const first. |
| Constructor arg, const in the *same scope* | **Works.** This is the form this plan uses. |
| Constructor arg, global const + object at global scope | Works. |
| Constructor arg, global const + object *inside* a scope | **Rejected** — "must be const" though it is. c-next#1187. Reproducer: `~/code/c-next2/bugs/issue-scoped-const-constructor-arg/`. |
| Constructor arg, qualified const — `Sensor s(AppConfig.PIN)` | **Parse error.** c-next#1188. Reproducer: `~/code/c-next2/bugs/issue-qualified-const-constructor-arg/`. |
| Ternary on a bare bool — `flag ? a : b` | **Parse error.** Write `(flag = true) ? a : b`; it generates `flag == true`. |

**Consequence for this plan:** pin constants for a driver live in that driver's own scope, not in `AppConfig`. That is the documented working form and is defensible design — the driver owns its pin. Once c-next#1187 and c-next#1188 are fixed, pins can centralise into `AppConfig` as a mechanical change.

## File Structure

```
platformio.ini                       [env:uno] and [env:native]
cnext.config.json                    transpiler config
cnext_build.py                       pre: script, copied from ossm unchanged
src/
  main.cnx                           setup()/loop(), delegates only
  AppConfig.cnx                      setpoint defaults, limits, intervals
  Domain/
    VentController.cnx               the loop; owns mode and both setpoints
    VentLogic.cnx                    cool/heat handlers + pure predicates
  Data/
    Elapsed.cnx                      PURE: wrap-safe millisecond subtraction
    Stopwatch.cnx                    ElapsedMilliseconds struct + reset/elapsed
    RoomSensor.cnx                   Adafruit_BME280 over SPI, validity gate
    types/EHvacMode.cnx              HVAC_COOL, HVAC_HEAT
  Display/
    VentRelay.cnx                    relay pin; source of truth for vent state
    ButtonLogic.cnx                  PURE: debounce + gesture state machine
    Buttons.cnx                      reads pins, feeds ButtonLogic
    VentDisplay.cnx                  16x2 rendering
    SerialLog.cnx                    1 Hz CSV status line
include/                             generated .hpp files, flat
test/
  test_elapsed/test_elapsed.cpp
  test_vent_logic/test_vent_logic.cpp
  test_button_logic/test_button_logic.cpp
```

`Elapsed`, `VentLogic`, and `ButtonLogic` are the only scopes that must never include `Arduino.h`. They are the three compiled into `[env:native]`.

---

### Task 1: Scaffolding and a serial banner

Proves the whole toolchain — `cnext` → PlatformIO → AVR → flash — before any logic exists.

**Files:**
- Create: `platformio.ini`, `cnext.config.json`, `cnext_build.py`, `src/main.cnx`, `src/AppConfig.cnx`, `src/Domain/VentController.cnx`
- Test: none (hardware observation)

**Interfaces:**
- Consumes: nothing
- Produces: `VentController.setup()`, `VentController.loop()`; `AppConfig` constants `COOL_SETPOINT_DEFAULT_C`, `HEAT_SETPOINT_DEFAULT_C`, `HYSTERESIS_C`, `SETPOINT_MINIMUM_C`, `SETPOINT_MAXIMUM_C`, `SETPOINT_STEP_C`, `SERIAL_BAUD`

- [ ] **Step 1: Copy the build script from ossm**

```bash
cp ~/code/ossm/cnext_build.py /home/linux/code/vent-controller/cnext_build.py
```

- [ ] **Step 2: Write `cnext.config.json`**

```json
{
    "cppRequired": true,
    "target": "avr",
    "include": ["include/", ".pio/libdeps/"],
    "noCache": true,
    "headerOut": "include",
    "basePath": "src"
}
```

- [ ] **Step 3: Write `platformio.ini`**

```ini
[env:uno]
platform = atmelavr
board = uno
framework = arduino
extra_scripts = pre:cnext_build.py
monitor_speed = 115200
build_flags = -I include
build_src_flags =
    -Wall
    -Wextra
    -Wno-unused-parameter
lib_deps =
    adafruit/Adafruit BME280 Library
    adafruit/Adafruit Unified Sensor
    adafruit/Adafruit BusIO
    SPI
    Wire
```

- [ ] **Step 4: Write `src/AppConfig.cnx`**

```c-next
// Tunable behaviour. Pin numbers live in the scope that owns the pin --
// see the constructor-argument note in the plan header.
scope AppConfig {
    const f32 COOL_SETPOINT_DEFAULT_C <- 18.0;
    const f32 HEAT_SETPOINT_DEFAULT_C <- 20.0;
    const f32 HYSTERESIS_C <- 1.0;
    const f32 SETPOINT_MINIMUM_C <- 10.0;
    const f32 SETPOINT_MAXIMUM_C <- 30.0;
    const f32 SETPOINT_STEP_C <- 0.1;
    const u32 SERIAL_BAUD <- 115200;
}
```

- [ ] **Step 5: Write `src/Domain/VentController.cnx`**

```c-next
#include <Arduino.h>
#include <AppConfig.cnx>

scope VentController {
    void setup() {
        Serial.begin(AppConfig.SERIAL_BAUD);
        Serial.println("vent-controller starting");
    }

    void loop() {
    }
}
```

- [ ] **Step 6: Write `src/main.cnx`**

```c-next
#include <Arduino.h>
#include <Domain/VentController.cnx>

void setup() {
    VentController.setup();
}

void loop() {
    VentController.loop();
}
```

- [ ] **Step 7: Transpile and confirm the generated layout**

Run: `cnext src/main.cnx`
Expected: reports "Generated N output files". Confirm `src/main.cpp`, `src/Domain/VentController.cpp` and `include/VentController.hpp` exist. If `"target": "avr"` is rejected, that is a transpiler defect — build a reproducer and stop.

- [ ] **Step 8: Build**

Run: `pio run -e uno`
Expected: SUCCESS. Note the reported flash and RAM percentages — this is the baseline for the budget.

- [ ] **Step 9: Flash and observe**

Run: `pio run -e uno -t upload && pio device monitor`
Expected: `vent-controller starting` appears once on reset.

- [ ] **Step 10: Commit**

```bash
git add -A
git commit -m "feat: project scaffolding, cnext and PlatformIO wiring"
```

---

### Task 2: Elapsed — wrap-safe millisecond subtraction

The only non-obvious arithmetic in the project, and the one thing that must be exhaustively tested.

**Files:**
- Create: `src/Data/Elapsed.cnx`, `test/test_elapsed/test_elapsed.cpp`
- Modify: `platformio.ini` (add `[env:native]`)

**Interfaces:**
- Consumes: nothing
- Produces: `uint32_t Elapsed_between(uint32_t base, uint32_t now)` — milliseconds from `base` to `now`, correct across a `millis()` wrap

- [ ] **Step 1: Add the native environment to `platformio.ini`**

```ini
[env:native]
platform = native
test_framework = unity
test_build_src = yes
build_flags = -I src -I include
build_src_filter = -<*> +<Data/Elapsed.cpp>
```

- [ ] **Step 2: Write the failing test**

Create `test/test_elapsed/test_elapsed.cpp`:

```cpp
#include <unity.h>
#include <Elapsed.hpp>

void setUp(void) {}
void tearDown(void) {}

void test_plain_difference_when_now_is_ahead(void) {
    TEST_ASSERT_EQUAL_UINT32(500u, Elapsed_between(1000u, 1500u));
}

void test_zero_when_now_equals_base(void) {
    TEST_ASSERT_EQUAL_UINT32(0u, Elapsed_between(1000u, 1000u));
}

void test_spans_a_millis_wrap(void) {
    // base taken 295 ms before the ceiling, read 204 ms after zero
    TEST_ASSERT_EQUAL_UINT32(500u, Elapsed_between(4294967000u, 204u));
}

void test_one_millisecond_across_the_ceiling(void) {
    TEST_ASSERT_EQUAL_UINT32(1u, Elapsed_between(4294967295u, 0u));
}

void test_largest_measurable_interval_does_not_saturate(void) {
    TEST_ASSERT_EQUAL_UINT32(4294967295u, Elapsed_between(4294967295u, 4294967294u));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_plain_difference_when_now_is_ahead);
    RUN_TEST(test_zero_when_now_equals_base);
    RUN_TEST(test_spans_a_millis_wrap);
    RUN_TEST(test_one_millisecond_across_the_ceiling);
    RUN_TEST(test_largest_measurable_interval_does_not_saturate);
    return UNITY_END();
}
```

- [ ] **Step 3: Run the test to verify it fails**

Run: `pio test -e native`
Expected: FAIL — `Elapsed.hpp: No such file or directory`.

- [ ] **Step 4: Write `src/Data/Elapsed.cnx`**

```c-next
// Milliseconds between two millis() readings, correct across the 49.7-day
// wrap. Pure -- no Arduino.h -- so it compiles into the native test build.
scope Elapsed {
    u32 between(u32 base, u32 now) {
        if (now >= base) {
            return now - base;
        }
        // millis() wrapped: the step from base up through the u32 ceiling,
        // then from zero up to now. Cannot overflow: this branch runs only
        // when now < base, so the result is at most 4294967295 exactly.
        return (4294967295 - base) + now + 1;
    }
}
```

- [ ] **Step 5: Transpile**

Run: `cnext src/Data/Elapsed.cnx`
Expected: generates `src/Data/Elapsed.cpp` and `include/Elapsed.hpp`. Confirm the header declares `uint32_t Elapsed_between(uint32_t base, uint32_t now);`.

- [ ] **Step 6: Run the tests to verify they pass**

Run: `pio test -e native`
Expected: 5 tests, 0 failures.

- [ ] **Step 7: Commit**

```bash
git add -A
git commit -m "feat: wrap-safe millisecond subtraction with host tests"
```

---

### Task 3: Stopwatch, proven by a heartbeat LED

Gives every later subsystem its cadence primitive, and proves it on real hardware where you can see it.

**Files:**
- Create: `src/Data/Stopwatch.cnx`
- Modify: `src/Domain/VentController.cnx`

**Interfaces:**
- Consumes: `Elapsed_between`
- Produces: `struct ElapsedMilliseconds { uint32_t base; }`, `uint32_t Stopwatch_elapsed(const ElapsedMilliseconds&)`, `void Stopwatch_reset(ElapsedMilliseconds&)`

- [ ] **Step 1: Write `src/Data/Stopwatch.cnx`**

```c-next
#include <Arduino.h>
#include <Data/Elapsed.cnx>

// Modelled on Teensy's elapsedMillis: one u32 holding the millis() value at
// the last reset. Nothing accumulates, so a missed update cannot corrupt it.
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

- [ ] **Step 2: Add a heartbeat to `src/Domain/VentController.cnx`**

Replace the whole file:

```c-next
#include <Arduino.h>
#include <AppConfig.cnx>
#include <Data/Stopwatch.cnx>

scope VentController {
    const u32 HEARTBEAT_MILLISECONDS <- 500;

    ElapsedMilliseconds heartbeatTimer;
    bool heartbeatOn <- false;

    void setup() {
        Serial.begin(AppConfig.SERIAL_BAUD);
        Serial.println("vent-controller starting");
        pinMode(LED_BUILTIN, OUTPUT);
        Stopwatch.reset(heartbeatTimer);
    }

    void loop() {
        u32 sinceHeartbeat <- Stopwatch.elapsed(heartbeatTimer);
        if (sinceHeartbeat < HEARTBEAT_MILLISECONDS) {
            return;
        }
        Stopwatch.reset(heartbeatTimer);
        heartbeatOn <- !heartbeatOn;
        digitalWrite(LED_BUILTIN, heartbeatOn);
    }
}
```

- [ ] **Step 3: Build and flash**

Run: `cnext src/main.cnx && pio run -e uno -t upload`
Expected: SUCCESS.

- [ ] **Step 4: Observe**

Expected: the Uno's built-in LED blinks at 1 Hz — on for 500 ms, off for 500 ms. If it blinks at the wrong rate or not at all, `Stopwatch` is wrong; do not proceed.

- [ ] **Step 5: Commit**

```bash
git add -A
git commit -m "feat: resettable elapsed counters, proven by heartbeat LED"
```

---

### Task 4: VentRelay

**Files:**
- Create: `src/Display/VentRelay.cnx`
- Modify: `src/Domain/VentController.cnx`

**Interfaces:**
- Consumes: nothing
- Produces: `void VentRelay_initialize(void)`, `void VentRelay_set(bool open)`, `bool VentRelay_isOpen(void)`

**Before this task:** measure the H_Relay module's active level with a meter and confirm `RELAY_ACTIVE_HIGH` below matches. Do not connect 24VAC until the click test in Step 4 behaves correctly on the bench.

- [ ] **Step 1: Write `src/Display/VentRelay.cnx`**

```c-next
#include <Arduino.h>

// Owns the relay pin and is the single source of truth for vent state.
// The damper is spring-return and normally closed, so a de-energised relay
// is a closed vent. Every failure path ends here.
scope VentRelay {
    const u8 RELAY_PIN <- 7;
    const bool RELAY_ACTIVE_HIGH <- true;

    bool ventOpen <- false;

    void initialize() {
        pinMode(RELAY_PIN, OUTPUT);
        set(false);
    }

    void set(bool open) {
        ventOpen <- open;
        bool level <- open;
        if (!RELAY_ACTIVE_HIGH) {
            level <- !open;
        }
        digitalWrite(RELAY_PIN, level);
    }

    bool isOpen() {
        return ventOpen;
    }
}
```

- [ ] **Step 2: Add a temporary click test to `VentController`**

Add the include, a timer, and toggle the relay in `loop()`. Add to the includes:

```c-next
#include <Display/VentRelay.cnx>
```

Add to the scope body:

```c-next
    const u32 CLICK_TEST_MILLISECONDS <- 3000;
    ElapsedMilliseconds clickTestTimer;
```

Add to `setup()`, after `Stopwatch.reset(heartbeatTimer);`:

```c-next
        VentRelay.initialize();
        Stopwatch.reset(clickTestTimer);
```

Add at the top of `loop()`, before the heartbeat block:

```c-next
        u32 sinceClick <- Stopwatch.elapsed(clickTestTimer);
        if (sinceClick >= CLICK_TEST_MILLISECONDS) {
            Stopwatch.reset(clickTestTimer);
            bool open <- VentRelay.isOpen();
            VentRelay.set(!open);
            Serial.println((open = true) ? "relay -> closed" : "relay -> open");
        }
```

- [ ] **Step 3: Build and flash**

Run: `cnext src/main.cnx && pio run -e uno -t upload && pio device monitor`
Expected: SUCCESS.

- [ ] **Step 4: Observe**

Expected: an audible relay click every 3 seconds, alternating with the serial lines. Confirm the relay's LED is **off** for the first 3 seconds after reset — that is the fail-closed boot state. If it energises at boot, flip `RELAY_ACTIVE_HIGH` and add the external pull resistor before going further.

- [ ] **Step 5: Remove the click test**

Delete `CLICK_TEST_MILLISECONDS`, `clickTestTimer`, its `Stopwatch.reset`, and the click block from `loop()`. Keep `VentRelay.initialize()` in `setup()`.

- [ ] **Step 6: Rebuild and commit**

```bash
cnext src/main.cnx && pio run -e uno
git add -A
git commit -m "feat: relay output with fail-closed boot state"
```

---

### Task 5: RoomSensor over SPI

**Files:**
- Create: `src/Data/RoomSensor.cnx`
- Modify: `src/Domain/VentController.cnx`

**Interfaces:**
- Consumes: `Stopwatch`
- Produces: `void RoomSensor_initialize(void)`, `void RoomSensor_update(void)`, `float RoomSensor_temperature(void)`, `bool RoomSensor_isValid(void)`

- [ ] **Step 1: Write `src/Data/RoomSensor.cnx`**

The chip-select constant is declared inside this scope deliberately — see the constructor-argument note in the plan header.

```c-next
#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_BME280.h>
#include <Data/Stopwatch.cnx>

scope RoomSensor {
    const u8 CHIP_SELECT_PIN <- 10;
    const u32 READ_INTERVAL_MILLISECONDS <- 1000;

    // The BME280's own specified operating range. A failed read, a miswired
    // bus, and an out-of-spec value all fail this one comparison, so no NaN
    // concept is needed.
    const f32 VALID_MINIMUM_C <- -40.0;
    const f32 VALID_MAXIMUM_C <- 85.0;

    Adafruit_BME280 sensor(CHIP_SELECT_PIN);

    ElapsedMilliseconds readTimer;
    bool started <- false;
    f32 lastTemperatureC <- 0.0;
    bool valid <- false;

    void initialize() {
        started <- sensor.begin();
        if (!started) {
            Serial.println("BME280 did not start");
            return;
        }
        Stopwatch.reset(readTimer);
    }

    void update() {
        if (!started) {
            valid <- false;
            return;
        }

        u32 sinceRead <- Stopwatch.elapsed(readTimer);
        if (sinceRead < READ_INTERVAL_MILLISECONDS) {
            return;
        }
        Stopwatch.reset(readTimer);

        f32 reading <- sensor.readTemperature();
        bool inRange <- (reading > VALID_MINIMUM_C) && (reading < VALID_MAXIMUM_C);
        valid <- inRange;
        if (inRange) {
            lastTemperatureC <- reading;
        }
    }

    f32 temperature() {
        return lastTemperatureC;
    }

    bool isValid() {
        return valid;
    }
}
```

- [ ] **Step 2: Wire it into `VentController`**

Add the include:

```c-next
#include <Data/RoomSensor.cnx>
```

Add to `setup()`:

```c-next
        RoomSensor.initialize();
```

Replace the body of `loop()` with:

```c-next
        RoomSensor.update();

        u32 sinceHeartbeat <- Stopwatch.elapsed(heartbeatTimer);
        if (sinceHeartbeat < HEARTBEAT_MILLISECONDS) {
            return;
        }
        Stopwatch.reset(heartbeatTimer);
        heartbeatOn <- !heartbeatOn;
        digitalWrite(LED_BUILTIN, heartbeatOn);

        f32 temperature <- RoomSensor.temperature();
        bool valid <- RoomSensor.isValid();
        Serial.print(temperature);
        Serial.println((valid = true) ? " C" : " C (invalid)");
```

- [ ] **Step 3: Build and flash**

Run: `cnext src/main.cnx && pio run -e uno -t upload && pio device monitor`
Expected: SUCCESS. Note the new flash usage.

- [ ] **Step 4: Observe**

Expected: a plausible room temperature every 500 ms, marked valid. Breathe on the sensor and watch it rise. If it reads `0.00 C (invalid)`, the SPI wiring or chip select is wrong; check the Waveshare silkscreen before changing code.

- [ ] **Step 5: Commit**

```bash
git add -A
git commit -m "feat: BME280 over SPI with range-gated validity"
```

---

### Task 6: VentLogic — the working v1

At the end of this task the device does its job. Everything after is refinement.

**Files:**
- Create: `src/Data/types/EHvacMode.cnx`, `src/Domain/VentLogic.cnx`, `test/test_vent_logic/test_vent_logic.cpp`
- Modify: `platformio.ini`, `src/Domain/VentController.cnx`

**Interfaces:**
- Consumes: `RoomSensor`, `VentRelay`, `AppConfig`
- Produces: `bool VentLogic_shouldOpenForCooling(float temperature, float setpoint, float hysteresis, bool currentlyOpen, bool readingValid)`, the same for `...ForHeating`, `void VentLogic_handleCoolCycle(float setpoint)`, `void VentLogic_handleHeatCycle(float setpoint)`, `enum EHvacMode { HVAC_COOL, HVAC_HEAT }`

- [ ] **Step 1: Extend `build_src_filter` in `[env:native]`**

```ini
build_src_filter = -<*> +<Data/Elapsed.cpp> +<Domain/VentLogic.cpp>
```

- [ ] **Step 2: Write the failing test**

Create `test/test_vent_logic/test_vent_logic.cpp`. Setpoint 18.0 °C with 1.0 °C hysteresis gives a band of 17.0–19.0 °C.

```cpp
#include <unity.h>
#include <VentLogic.hpp>

static const float SETPOINT = 18.0f;
static const float BAND = 1.0f;

void setUp(void) {}
void tearDown(void) {}

// --- cooling ---
void test_cooling_closed_stays_closed_inside_the_band(void) {
    TEST_ASSERT_FALSE(VentLogic_shouldOpenForCooling(18.0f, SETPOINT, BAND, false, true));
}

void test_cooling_closed_stays_closed_at_the_upper_edge(void) {
    TEST_ASSERT_FALSE(VentLogic_shouldOpenForCooling(19.0f, SETPOINT, BAND, false, true));
}

void test_cooling_closed_opens_above_the_upper_edge(void) {
    TEST_ASSERT_TRUE(VentLogic_shouldOpenForCooling(19.1f, SETPOINT, BAND, false, true));
}

void test_cooling_open_stays_open_inside_the_band(void) {
    TEST_ASSERT_TRUE(VentLogic_shouldOpenForCooling(18.0f, SETPOINT, BAND, true, true));
}

void test_cooling_open_closes_at_the_lower_edge(void) {
    TEST_ASSERT_FALSE(VentLogic_shouldOpenForCooling(17.0f, SETPOINT, BAND, true, true));
}

void test_cooling_invalid_reading_closes_even_when_hot(void) {
    TEST_ASSERT_FALSE(VentLogic_shouldOpenForCooling(30.0f, SETPOINT, BAND, true, false));
}

// --- heating: the mirror image, which pins the direction ---
void test_heating_closed_stays_closed_inside_the_band(void) {
    TEST_ASSERT_FALSE(VentLogic_shouldOpenForHeating(18.0f, SETPOINT, BAND, false, true));
}

void test_heating_closed_opens_below_the_lower_edge(void) {
    TEST_ASSERT_TRUE(VentLogic_shouldOpenForHeating(16.9f, SETPOINT, BAND, false, true));
}

void test_heating_open_stays_open_inside_the_band(void) {
    TEST_ASSERT_TRUE(VentLogic_shouldOpenForHeating(18.0f, SETPOINT, BAND, true, true));
}

void test_heating_open_closes_at_the_upper_edge(void) {
    TEST_ASSERT_FALSE(VentLogic_shouldOpenForHeating(19.0f, SETPOINT, BAND, true, true));
}

void test_heating_invalid_reading_closes_even_when_cold(void) {
    TEST_ASSERT_FALSE(VentLogic_shouldOpenForHeating(5.0f, SETPOINT, BAND, true, false));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_cooling_closed_stays_closed_inside_the_band);
    RUN_TEST(test_cooling_closed_stays_closed_at_the_upper_edge);
    RUN_TEST(test_cooling_closed_opens_above_the_upper_edge);
    RUN_TEST(test_cooling_open_stays_open_inside_the_band);
    RUN_TEST(test_cooling_open_closes_at_the_lower_edge);
    RUN_TEST(test_cooling_invalid_reading_closes_even_when_hot);
    RUN_TEST(test_heating_closed_stays_closed_inside_the_band);
    RUN_TEST(test_heating_closed_opens_below_the_lower_edge);
    RUN_TEST(test_heating_open_stays_open_inside_the_band);
    RUN_TEST(test_heating_open_closes_at_the_upper_edge);
    RUN_TEST(test_heating_invalid_reading_closes_even_when_cold);
    return UNITY_END();
}
```

- [ ] **Step 3: Run the test to verify it fails**

Run: `pio test -e native -f test_vent_logic`
Expected: FAIL — `VentLogic.hpp: No such file or directory`.

- [ ] **Step 4: Write `src/Data/types/EHvacMode.cnx`**

```c-next
enum EHvacMode {
    HVAC_COOL,
    HVAC_HEAT
}
```

- [ ] **Step 5: Write `src/Domain/VentLogic.cnx`**

The predicates take everything as parameters and touch no hardware, so the native build compiles this file. The handlers are what the loop calls.

```c-next
#include <Data/RoomSensor.cnx>
#include <Display/VentRelay.cnx>

scope VentLogic {
    // The setpoint is the middle of the band and hysteresis is the distance
    // to each edge, so 18.0 C with 1.0 C swings the room across 17.0-19.0 C.
    bool shouldOpenForCooling(f32 temperature, f32 setpoint, f32 hysteresis,
                              bool currentlyOpen, bool readingValid) {
        if (!readingValid) {
            return false;
        }
        if (currentlyOpen) {
            return temperature > (setpoint - hysteresis);
        }
        return temperature > (setpoint + hysteresis);
    }

    bool shouldOpenForHeating(f32 temperature, f32 setpoint, f32 hysteresis,
                              bool currentlyOpen, bool readingValid) {
        if (!readingValid) {
            return false;
        }
        if (currentlyOpen) {
            return temperature < (setpoint + hysteresis);
        }
        return temperature < (setpoint - hysteresis);
    }

    void handleCoolCycle(f32 setpoint) {
        f32 temperature <- RoomSensor.temperature();
        bool valid <- RoomSensor.isValid();
        bool currentlyOpen <- VentRelay.isOpen();
        bool open <- shouldOpenForCooling(temperature, setpoint,
                                          AppConfig.HYSTERESIS_C,
                                          currentlyOpen, valid);
        VentRelay.set(open);
    }

    void handleHeatCycle(f32 setpoint) {
        f32 temperature <- RoomSensor.temperature();
        bool valid <- RoomSensor.isValid();
        bool currentlyOpen <- VentRelay.isOpen();
        bool open <- shouldOpenForHeating(temperature, setpoint,
                                          AppConfig.HYSTERESIS_C,
                                          currentlyOpen, valid);
        VentRelay.set(open);
    }
}
```

- [ ] **Step 6: Run the tests to verify they pass**

Run: `cnext src/main.cnx && pio test -e native`
Expected: 16 tests total across both suites, 0 failures.

If the native build fails because `VentLogic.cpp` pulls in `RoomSensor.hpp` and therefore `Arduino.h`, that is a real structural problem, not a build-flag problem: split the two predicates into their own `src/Domain/VentDecision.cnx` with no includes, have `VentLogic` call them, and point `build_src_filter` at `VentDecision.cpp` instead. Update the test include to `<VentDecision.hpp>` and the symbol prefix to `VentDecision_`.

- [ ] **Step 7: Wire it into `VentController`**

Add the includes:

```c-next
#include <Data/types/EHvacMode.cnx>
#include <Domain/VentLogic.cnx>
```

Add to the scope body:

```c-next
    EHvacMode mode <- EHvacMode.HVAC_COOL;
    f32 coolSetpointC <- AppConfig.COOL_SETPOINT_DEFAULT_C;
    f32 heatSetpointC <- AppConfig.HEAT_SETPOINT_DEFAULT_C;
```

Replace `loop()` with:

```c-next
    void loop() {
        RoomSensor.update();

        if (mode = EHvacMode.HVAC_COOL) {
            VentLogic.handleCoolCycle(coolSetpointC);
        } else {
            VentLogic.handleHeatCycle(heatSetpointC);
        }

        // No early return -- Tasks 7 to 9 append to the end of this loop.
        u32 sinceHeartbeat <- Stopwatch.elapsed(heartbeatTimer);
        if (sinceHeartbeat >= HEARTBEAT_MILLISECONDS) {
            Stopwatch.reset(heartbeatTimer);
            f32 temperature <- RoomSensor.temperature();
            bool valid <- RoomSensor.isValid();
            bool open <- VentRelay.isOpen();
            Serial.print(temperature);
            Serial.print((valid = true) ? " C " : " C invalid ");
            Serial.println((open = true) ? "OPEN" : "CLOSED");
        }
    }
```

Delete `heartbeatOn` and the `digitalWrite(LED_BUILTIN, ...)` line; the relay is now the thing to watch.

- [ ] **Step 8: Flash and observe**

Run: `cnext src/main.cnx && pio run -e uno -t upload && pio device monitor`
Expected: temperature, validity and vent state twice a second. Warm the sensor past 19.0 °C and the relay must click open; cool it to 17.0 °C and it must click closed. Between those it must not click at all — that is the hysteresis working.

- [ ] **Step 9: Commit**

```bash
git add -A
git commit -m "feat: hysteresis vent control -- working v1"
```

---

### Task 7: 16x2 status display

**Files:**
- Create: `src/Display/VentDisplay.cnx`
- Modify: `platformio.ini` (add the LCD library), `src/Domain/VentController.cnx`

**Interfaces:**
- Consumes: `RoomSensor`, `VentRelay`, `Stopwatch`
- Produces: `void VentDisplay_initialize(void)`, `void VentDisplay_update(float setpoint, bool coolingMode)`

- [ ] **Step 1: Find the LCD's I2C address**

Add this to the end of `VentController.setup()` temporarily, with `#include <Wire.h>`:

```c-next
        Wire.begin();
        for (u8 address <- 1; address < 127; address +<- 1) {
            Wire.beginTransmission(address);
            u8 error <- Wire.endTransmission();
            if (error = 0) {
                Serial.print("I2C device at 0x");
                Serial.println(address, HEX);
            }
        }
```

Run: `cnext src/main.cnx && pio run -e uno -t upload && pio device monitor`
Expected: one address, `0x27` or `0x3F`. Record it, then delete the loop.

- [ ] **Step 2: Add the library to `[env:uno]` `lib_deps`**

```ini
    marcoschwartz/LiquidCrystal_I2C
```

If that fork does not compile, try `johnrickman/LiquidCrystal_I2C`. Several incompatible libraries share the name.

- [ ] **Step 3: Write `src/Display/VentDisplay.cnx`**

Both lines are exactly 16 characters. Set `LCD_ADDRESS` to what Step 1 found.

```c-next
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Data/RoomSensor.cnx>
#include <Data/Stopwatch.cnx>
#include <Display/VentRelay.cnx>

scope VentDisplay {
    const u8 LCD_ADDRESS <- 0x27;
    const u8 LCD_COLUMNS <- 16;
    const u8 LCD_ROWS <- 2;
    const u32 REDRAW_MILLISECONDS <- 250;

    LiquidCrystal_I2C screen(LCD_ADDRESS, LCD_COLUMNS, LCD_ROWS);

    ElapsedMilliseconds redrawTimer;
    ElapsedMilliseconds faultTimer;
    bool faultActive <- false;

    void initialize() {
        screen.init();
        screen.backlight();
        Stopwatch.reset(redrawTimer);
        Stopwatch.reset(faultTimer);
    }

    void update(f32 setpoint, bool coolingMode) {
        u32 sinceRedraw <- Stopwatch.elapsed(redrawTimer);
        if (sinceRedraw < REDRAW_MILLISECONDS) {
            return;
        }
        Stopwatch.reset(redrawTimer);

        bool valid <- RoomSensor.isValid();
        if (!valid) {
            if (!faultActive) {
                faultActive <- true;
                Stopwatch.reset(faultTimer);
            }
            u32 faultMilliseconds <- Stopwatch.elapsed(faultTimer);
            u32 faultMinutes <- faultMilliseconds / 60000;
            screen.setCursor(0, 0);
            screen.print("SENSOR FAULT    ");
            screen.setCursor(0, 1);
            screen.print("VENT CLOSED ");
            screen.print(faultMinutes);
            screen.print("m  ");
            return;
        }

        faultActive <- false;

        screen.setCursor(0, 0);
        screen.print(RoomSensor.temperature(), 1);
        screen.print("C   SET ");
        screen.print(setpoint, 1);
        screen.print(" ");

        bool open <- VentRelay.isOpen();
        screen.setCursor(0, 1);
        screen.print((coolingMode = true) ? "COOL " : "HEAT ");
        screen.print((open = true) ? "  VENT OPEN" : "VENT CLOSED");
    }
}
```

- [ ] **Step 4: Wire it into `VentController`**

Add `#include <Display/VentDisplay.cnx>`, add `VentDisplay.initialize();` to `setup()`, and add this as the last line of `loop()`:

```c-next
        bool coolingMode <- (mode = EHvacMode.HVAC_COOL);
        f32 activeSetpoint <- heatSetpointC;
        if (coolingMode) {
            activeSetpoint <- coolSetpointC;
        }
        VentDisplay.update(activeSetpoint, coolingMode);
```

- [ ] **Step 5: Flash and observe**

Run: `cnext src/main.cnx && pio run -e uno -t upload`
Expected: both lines readable, neither wrapping or leaving stale characters. Check the flash percentage — if it is above 90%, drop `Adafruit_Unified_Sensor` usage or switch to a lighter LCD library rather than continuing.

- [ ] **Step 6: Verify the fault screen**

Unplug the BME280's chip-select wire. Expected: within a second the display shows `SENSOR FAULT` and the relay closes. Reconnect and it returns to normal.

- [ ] **Step 7: Commit**

```bash
git add -A
git commit -m "feat: 16x2 status and fault display"
```

---

### Task 8: Buttons

**Files:**
- Create: `src/Display/ButtonLogic.cnx`, `src/Display/Buttons.cnx`, `test/test_button_logic/test_button_logic.cpp`
- Modify: `platformio.ini`, `src/Domain/VentController.cnx`

**Interfaces:**
- Consumes: `Elapsed_between`
- Produces: `enum EButtonEvent { NONE, STEP_UP, STEP_DOWN, TOGGLE_MODE }`, `struct DebouncedButton`, `struct ButtonState`, `EButtonEvent ButtonLogic_step(ButtonState&, bool upPressed, bool downPressed, uint32_t now)`, `void Buttons_initialize(void)`, `EButtonEvent Buttons_update(void)`

**Known behaviour:** pressing both buttons with more than ~25 ms of skew emits one setpoint step before the mode toggle, because debouncing stabilises each pin independently and cannot synchronise two of them. The stray step is 0.1 °C on the setpoint you just switched away from. If it becomes annoying in use, the fix is to emit steps on release rather than press.

- [ ] **Step 1: Extend `build_src_filter` in `[env:native]`**

```ini
build_src_filter = -<*> +<Data/Elapsed.cpp> +<Domain/VentLogic.cpp> +<Display/ButtonLogic.cpp>
```

- [ ] **Step 2: Write the failing test**

Create `test/test_button_logic/test_button_logic.cpp`:

```cpp
#include <unity.h>
#include <ButtonLogic.hpp>

static ButtonState state;

void setUp(void) { state = ButtonState(); }
void tearDown(void) {}

// Hold a level long enough for the 25 ms debounce to accept it.
static EButtonEvent settle(bool up, bool down, uint32_t at) {
    ButtonLogic_step(state, up, down, at);
    return ButtonLogic_step(state, up, down, at + 30u);
}

void test_no_event_while_nothing_is_pressed(void) {
    TEST_ASSERT_EQUAL(NONE, settle(false, false, 1000u));
}

void test_glitch_shorter_than_debounce_is_rejected(void) {
    ButtonLogic_step(state, false, false, 1000u);
    ButtonLogic_step(state, true, false, 1010u);
    TEST_ASSERT_EQUAL(NONE, ButtonLogic_step(state, false, false, 1020u));
}

void test_up_press_steps_up_once(void) {
    TEST_ASSERT_EQUAL(STEP_UP, settle(true, false, 1000u));
    TEST_ASSERT_EQUAL(NONE, ButtonLogic_step(state, true, false, 1040u));
}

void test_down_press_steps_down(void) {
    TEST_ASSERT_EQUAL(STEP_DOWN, settle(false, true, 1000u));
}

void test_both_pressed_toggles_mode_once(void) {
    TEST_ASSERT_EQUAL(TOGGLE_MODE, settle(true, true, 1000u));
    TEST_ASSERT_EQUAL(NONE, ButtonLogic_step(state, true, true, 1100u));
}

void test_holding_repeats_after_the_delay(void) {
    settle(true, false, 1000u);
    // still inside the 600 ms hold delay
    TEST_ASSERT_EQUAL(NONE, ButtonLogic_step(state, true, false, 1500u));
    // past the delay, first repeat
    TEST_ASSERT_EQUAL(STEP_UP, ButtonLogic_step(state, true, false, 1700u));
    // too soon for the next repeat
    TEST_ASSERT_EQUAL(NONE, ButtonLogic_step(state, true, false, 1750u));
    // 150 ms later
    TEST_ASSERT_EQUAL(STEP_UP, ButtonLogic_step(state, true, false, 1860u));
}

void test_release_rearms_a_fresh_press(void) {
    settle(true, false, 1000u);
    settle(false, false, 2000u);
    TEST_ASSERT_EQUAL(STEP_UP, settle(true, false, 3000u));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_no_event_while_nothing_is_pressed);
    RUN_TEST(test_glitch_shorter_than_debounce_is_rejected);
    RUN_TEST(test_up_press_steps_up_once);
    RUN_TEST(test_down_press_steps_down);
    RUN_TEST(test_both_pressed_toggles_mode_once);
    RUN_TEST(test_holding_repeats_after_the_delay);
    RUN_TEST(test_release_rearms_a_fresh_press);
    return UNITY_END();
}
```

- [ ] **Step 3: Run the test to verify it fails**

Run: `pio test -e native -f test_button_logic`
Expected: FAIL — `ButtonLogic.hpp: No such file or directory`.

- [ ] **Step 4: Write `src/Display/ButtonLogic.cnx`**

Pure — no `Arduino.h`. Works in "true means pressed" terms; `Buttons` inverts the pull-up levels before calling in.

```c-next
#include <Data/Elapsed.cnx>

enum EButtonEvent {
    NONE,
    STEP_UP,
    STEP_DOWN,
    TOGGLE_MODE
}

struct DebouncedButton {
    bool rawLast;
    u32 changedAt;
    bool stable;
}

struct ButtonState {
    DebouncedButton up;
    DebouncedButton down;
    bool modeFired;
    bool singleWasDown;
    u32 pressedAt;
    u32 lastRepeatAt;
}

scope ButtonLogic {
    const u32 DEBOUNCE_MILLISECONDS <- 25;
    const u32 HOLD_DELAY_MILLISECONDS <- 600;
    const u32 REPEAT_MILLISECONDS <- 150;

    // Accept a level only once it has held steady for the debounce window.
    void updateDebounce(DebouncedButton button, bool raw, u32 now) {
        if (raw != button.rawLast) {
            button.rawLast <- raw;
            button.changedAt <- now;
            return;
        }
        u32 held <- Elapsed.between(button.changedAt, now);
        if (held >= DEBOUNCE_MILLISECONDS) {
            button.stable <- raw;
        }
    }

    EButtonEvent step(ButtonState state, bool upPressed, bool downPressed, u32 now) {
        updateDebounce(state.up, upPressed, now);
        updateDebounce(state.down, downPressed, now);

        bool bothDown <- state.up.stable && state.down.stable;
        bool eitherDown <- state.up.stable || state.down.stable;

        if (bothDown) {
            if (state.modeFired) {
                return EButtonEvent.NONE;
            }
            state.modeFired <- true;
            state.singleWasDown <- false;
            return EButtonEvent.TOGGLE_MODE;
        }

        if (!eitherDown) {
            state.modeFired <- false;
            state.singleWasDown <- false;
            return EButtonEvent.NONE;
        }

        // Exactly one button is down. If a chord already fired, the user is
        // lifting one finger before the other -- do not emit a stray step.
        if (state.modeFired) {
            return EButtonEvent.NONE;
        }

        if (!state.singleWasDown) {
            state.singleWasDown <- true;
            state.pressedAt <- now;
            state.lastRepeatAt <- now;
            if (state.up.stable) {
                return EButtonEvent.STEP_UP;
            }
            return EButtonEvent.STEP_DOWN;
        }

        u32 heldFor <- Elapsed.between(state.pressedAt, now);
        if (heldFor < HOLD_DELAY_MILLISECONDS) {
            return EButtonEvent.NONE;
        }

        u32 sinceRepeat <- Elapsed.between(state.lastRepeatAt, now);
        if (sinceRepeat < REPEAT_MILLISECONDS) {
            return EButtonEvent.NONE;
        }

        state.lastRepeatAt <- now;
        if (state.up.stable) {
            return EButtonEvent.STEP_UP;
        }
        return EButtonEvent.STEP_DOWN;
    }
}
```

- [ ] **Step 5: Run the tests to verify they pass**

Run: `cnext src/main.cnx && pio test -e native`
Expected: all three suites pass, 0 failures.

- [ ] **Step 6: Write `src/Display/Buttons.cnx`**

```c-next
#include <Arduino.h>
#include <Display/ButtonLogic.cnx>

scope Buttons {
    const u8 UP_PIN <- 2;
    const u8 DOWN_PIN <- 3;

    ButtonState state;

    void initialize() {
        pinMode(UP_PIN, INPUT_PULLUP);
        pinMode(DOWN_PIN, INPUT_PULLUP);
    }

    // Runs at full loop rate: debouncing samples continuously rather than
    // gating how often this is called.
    EButtonEvent update() {
        u8 upLevel <- digitalRead(UP_PIN);
        u8 downLevel <- digitalRead(DOWN_PIN);
        bool upPressed <- (upLevel = LOW);
        bool downPressed <- (downLevel = LOW);
        u32 now <- millis();
        return ButtonLogic.step(state, upPressed, downPressed, now);
    }
}
```

- [ ] **Step 7: Wire it into `VentController`**

Add `#include <Display/Buttons.cnx>`, add `Buttons.initialize();` to `setup()`, and add this as the first statement of `loop()`:

```c-next
        EButtonEvent event <- Buttons.update();
        bool coolingNow <- (mode = EHvacMode.HVAC_COOL);

        if (event = EButtonEvent.TOGGLE_MODE) {
            if (coolingNow) {
                mode <- EHvacMode.HVAC_HEAT;
            } else {
                mode <- EHvacMode.HVAC_COOL;
            }
        } else if (event != EButtonEvent.NONE) {
            f32 delta <- AppConfig.SETPOINT_STEP_C;
            if (event = EButtonEvent.STEP_DOWN) {
                delta <- -AppConfig.SETPOINT_STEP_C;
            }
            f32 updated <- coolSetpointC + delta;
            if (!coolingNow) {
                updated <- heatSetpointC + delta;
            }
            if (updated < AppConfig.SETPOINT_MINIMUM_C) {
                updated <- AppConfig.SETPOINT_MINIMUM_C;
            }
            if (updated > AppConfig.SETPOINT_MAXIMUM_C) {
                updated <- AppConfig.SETPOINT_MAXIMUM_C;
            }
            if (coolingNow) {
                coolSetpointC <- updated;
            } else {
                heatSetpointC <- updated;
            }
        }
```

- [ ] **Step 8: Flash and verify each gesture**

Run: `cnext src/main.cnx && pio run -e uno -t upload`

Check all four:
- Tap Up: setpoint rises 0.1 °C.
- Tap Down: setpoint falls 0.1 °C.
- Hold Up: pauses, then climbs steadily.
- Press both: the display switches between `COOL` and `HEAT` and shows that mode's own setpoint.
- Drive the setpoint to 10.0 and 30.0 and confirm it stops there.

- [ ] **Step 9: Commit**

```bash
git add -A
git commit -m "feat: two-button setpoint and mode control with host-tested debounce"
```

---

### Task 9: CSV logging and the watchdog

**Files:**
- Create: `src/Display/SerialLog.cnx`
- Modify: `src/Domain/VentController.cnx`

**Interfaces:**
- Consumes: `RoomSensor`, `VentRelay`, `Stopwatch`
- Produces: `void SerialLog_initialize(void)`, `void SerialLog_update(float setpoint, bool coolingMode)`

- [ ] **Step 1: Write `src/Display/SerialLog.cnx`**

```c-next
#include <Arduino.h>
#include <Data/RoomSensor.cnx>
#include <Data/Stopwatch.cnx>
#include <Display/VentRelay.cnx>

// One CSV line per second: milliseconds,temperatureC,setpointC,mode,vent,valid
scope SerialLog {
    const u32 LOG_INTERVAL_MILLISECONDS <- 1000;

    ElapsedMilliseconds logTimer;

    void initialize() {
        Serial.println("milliseconds,temperatureC,setpointC,mode,vent,valid");
        Stopwatch.reset(logTimer);
    }

    void update(f32 setpoint, bool coolingMode) {
        u32 sinceLog <- Stopwatch.elapsed(logTimer);
        if (sinceLog < LOG_INTERVAL_MILLISECONDS) {
            return;
        }
        Stopwatch.reset(logTimer);

        bool valid <- RoomSensor.isValid();
        bool open <- VentRelay.isOpen();
        u32 now <- millis();

        Serial.print(now);
        Serial.print(",");
        Serial.print(RoomSensor.temperature(), 2);
        Serial.print(",");
        Serial.print(setpoint, 1);
        Serial.print(",");
        Serial.print((coolingMode = true) ? "COOL," : "HEAT,");
        Serial.print((open = true) ? "OPEN," : "CLOSED,");
        Serial.println((valid = true) ? "1" : "0");
    }
}
```

- [ ] **Step 2: Replace the ad-hoc logging in `VentController`**

Add `#include <Display/SerialLog.cnx>`, add `SerialLog.initialize();` to `setup()`, and delete the `heartbeatTimer`, `HEARTBEAT_MILLISECONDS`, and every `Serial.print` left in `loop()`. Add as the last line of `loop()`:

```c-next
        SerialLog.update(activeSetpoint, coolingMode);
```

- [ ] **Step 3: Add the watchdog**

Add `#include <avr/wdt.h>` to `VentController`. Add as the last line of `setup()`:

```c-next
        global.wdt_enable(WDTO_2S);
```

Add as the last line of `loop()`:

```c-next
        global.wdt_reset();
```

- [ ] **Step 4: Flash and confirm the watchdog does not reset a healthy board**

Run: `cnext src/main.cnx && pio run -e uno -t upload && pio device monitor`
Expected: the CSV header appears once, then one line per second, and the header does **not** reappear. A repeating header means the watchdog is resetting the board — the loop is taking longer than 2 s, or `wdt_reset()` is not being reached on every path.

- [ ] **Step 5: Confirm it does reset a hung board**

Temporarily add `delay(5000);` at the end of `loop()`. Expected: the header repeats every ~2 s. Remove the `delay` afterwards.

- [ ] **Step 6: Run the full test suite and build**

Run: `pio test -e native && pio run -e uno`
Expected: all tests pass, build succeeds. Record final flash and RAM percentages.

- [ ] **Step 7: Commit**

```bash
git add -A
git commit -m "feat: CSV status logging and watchdog"
```

---

## After the plan

Left deliberately for after the device is running and observed:

- Tune `HYSTERESIS_C` against how the room actually behaves. It starts at 1.0 °C by agreement, not by measurement.
- Site the BME280 away from the register and outside the controller enclosure. Self-heating and discharge air defeat the loop regardless of firmware.
- Decide the 24VAC source — the air handler's transformer or a separate supply. No firmware impact.
- Confirm the Uno's 5V supply, and whether it is powered independently of the air handler. Determines whether the board brownout-resets when the blower starts.
- Centralise pin constants into `AppConfig` once c-next#1187 and c-next#1188 are fixed.
