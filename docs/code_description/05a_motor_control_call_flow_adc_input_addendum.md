# Motor Control Call Flow - ADC Input Addendum

**Purpose:** Addendum to `05_motor_control_call_flow.md` showing the call flow when using hall effect accelerator pedal (ADC input) instead of RC receiver (PPM input)

## Overview

This document traces the complete execution path from a **hall effect accelerator pedal** connected to the ADC inputs through to motor PWM output. This is an alternative input method to the PPM RC receiver described in the main motor control call flow document.

**Key Differences from PPM Input:**
- **Input Type:** Analog voltage (0-3.3V) instead of PPM pulse width
- **Thread:** `app_adc_thread` instead of `app_ppm_thread`
- **Hardware:** ADC peripheral instead of timer input capture
- **Configuration:** Voltage mapping instead of pulse width mapping

---

## Hardware Connection

### Accelerator Pedal Pin Connections

Hall effect accelerator pedals connect to the **COMM Port** (also called sensor port) on VESC hardware:

#### Primary ADC Input (ADC_EXT)
**File:** `hwconf/trampa/60_75/hw_60_75_core.h:152-153`

```c
#define HW_ADC_EXT_GPIO         GPIOA
#define HW_ADC_EXT_PIN          5
#define ADC_IND_EXT             6       // ADC channel index
```

**Physical Connection:**
- **Pin:** PA5 (GPIO Port A, Pin 5)
- **Location:** COMM Port connector, typically labeled "ADC" or "Analog 1"
- **Voltage Range:** 0V to 3.3V (VESC ADC reference voltage)

#### Secondary ADC Input (ADC_EXT2) - Optional for Dual Pedal or Brake
**File:** `hwconf/trampa/60_75/hw_60_75_core.h:154-155`

```c
#define HW_ADC_EXT2_GPIO        GPIOA
#define HW_ADC_EXT2_PIN         6
#define ADC_IND_EXT2            7       // ADC channel index
```

**Physical Connection:**
- **Pin:** PA6 (GPIO Port A, Pin 6)
- **Location:** COMM Port connector, typically labeled "ADC2" or "Analog 2"
- **Use Cases:**
  - Brake pedal (separate from accelerator)
  - Second accelerator channel for redundancy
  - Combined throttle/brake pedal

#### Typical COMM Port Pinout

```
COMM Port Connector (JST-PH or similar):
┌────────────────────────────────┐
│ Pin 1: 5V or 3.3V              │ ← Power for hall sensor
│ Pin 2: ADC_EXT  (PA5)          │ ← Accelerator signal
│ Pin 3: ADC_EXT2 (PA6)          │ ← Brake signal (optional)
│ Pin 4: TX (UART transmit)      │
│ Pin 5: RX (UART receive)       │
│ Pin 6: GND                     │ ← Ground reference
└────────────────────────────────┘
```

**Note:** Pin order and connector type vary by hardware variant. Consult your specific VESC hardware documentation. Common variants:
- VESC 4.x: 6-pin JST-PH connector
- VESC 6.x: 6-pin JST-PH connector
- VESC 75/300: 6-pin or 8-pin connector with additional features

### Hall Effect Accelerator Pedal Wiring

**Typical Hall Effect Pedal (3-wire):**
```
Hall Pedal          VESC COMM Port
┌─────────┐         ┌──────────┐
│  RED    │────────→│ 5V       │ (Power)
│  BLACK  │────────→│ GND      │ (Ground)
│  YELLOW │────────→│ ADC_EXT  │ (Signal, typically 0.5V to 4.5V range)
└─────────┘         └──────────┘
```

**Important:** Most hall effect pedals output 0.5V (idle) to 4.5V (full throttle), but the VESC ADC can only handle **0V to 3.3V**. Use a voltage divider if your pedal exceeds 3.3V:

```
Voltage Divider (if pedal output > 3.3V):
Pedal Signal ──┬── 10kΩ ──┬── ADC_EXT (PA5)
               │          │
               │          └── 22kΩ ──┬── GND
               │                     │
               └─────────────────────┘
Divider ratio: Vout = Vin × (22kΩ / (10kΩ + 22kΩ)) = Vin × 0.69
Example: 4.5V input → 3.1V output (safe for ADC)
```

**VESC Tool Configuration:**
The voltage divider is configured in VESC Tool → App Settings → General → ADC:
- Voltage Start: Minimum voltage at pedal idle (e.g., 0.35V after divider)
- Voltage End: Maximum voltage at pedal full press (e.g., 3.1V after divider)

---

## The Complete Control Loop - ADC Input

```
USER INPUT (Physical)
    │
    ├─► Hall Effect Pedal → Analog Voltage (0-3.3V)
    │
    ▼
┌────────────────────────────────────┐
│   HARDWARE ADC PERIPHERAL          │
│   (Continuous sampling)            │
├────────────────────────────────────┤
│   ADC1/ADC2/ADC3 (STM32)           │
│   - Regular conversion sequence    │
│   - DMA transfer to RAM buffer     │
└────────────────────────────────────┘
         │
         │ ADC_Value[ADC_IND_EXT]
         │ Updated continuously by DMA
         │
         ▼
┌────────────────────────────────────┐
│   CHIBIOS APPLICATION THREAD       │
│   (100-1000 Hz, NORMALPRIO)        │
├────────────────────────────────────┤
│   app_adc_thread()                 │
│   applications/app_adc.c:163       │
└────────────────────────────────────┘
         │
         │ Read ADC value
         │ Apply voltage mapping
         │ Apply filtering
         │ Apply curves/limits
         │
         ▼
┌────────────────────────────────────┐
│   MOTOR CONTROL INTERFACE          │
│   (Thread Context)                 │
├────────────────────────────────────┤
│   mc_interface_set_current_rel()   │
│   mc_interface_set_duty()          │
│   mc_interface_set_pid_speed()     │
└────────────────────────────────────┘
         │
         │ [Same flow as PPM from here]
         │ See 05_motor_control_call_flow.md
         │
         ▼
    FOC Control → ISR → Motor PWM
```

---

## Detailed Trace: Hall Effect Pedal Example

Let's trace a complete example: **Hall effect pedal pressed to 75% throttle**

### Step 1: Hardware ADC Conversion (No CPU involved)

**Hardware:** STM32 ADC peripheral (ADC1/ADC2/ADC3)

```
Hall Effect Sensor Output: 2.5V
    │
    ▼
STM32 ADC Peripheral
    │
    ├─ Regular conversion group
    ├─ 12-bit resolution (0-4095)
    ├─ DMA transfer to ADC_Value[] array
    └─ Continuous sampling mode
    │
    ▼
ADC_Value[ADC_IND_EXT] = 3103
    (2.5V / 3.3V × 4095 = 3103)

[No CPU cycles used - ADC + DMA handles conversion]
```

**ADC Initialization:**
**File:** `hwconf/hw.c` (hardware-specific)

```c
// ADC GPIO configuration
palSetPadMode(HW_ADC_EXT_GPIO, HW_ADC_EXT_PIN, PAL_MODE_INPUT_ANALOG);
palSetPadMode(HW_ADC_EXT2_GPIO, HW_ADC_EXT2_PIN, PAL_MODE_INPUT_ANALOG);

// ADC configured for:
// - 12-bit resolution (0-4095)
// - Regular conversion group
// - DMA circular buffer mode
// - Continuous scanning at ~10 kHz
```

**ADC Characteristics:**
- **Resolution:** 12-bit (4096 levels)
- **Reference Voltage:** 3.3V (V_REG)
- **Conversion Time:** ~3 µs per channel
- **Update Rate:** ~10 kHz (all channels scanned continuously)

---

### Step 2: ADC Application Thread (Thread Context)

**File:** `applications/app_adc.c:163`

**Thread:** `adc_thread` (NORMALPRIO, configurable 100-1000 Hz)

```c
static THD_FUNCTION(adc_thread, arg) {
    (void)arg;

    chRegSetThreadName("APP_ADC");  // Register with ChibiOS
    is_running = true;

    for(;;) {
        // ═══════════════════════════════════════
        // Step 2a: Sleep for configured rate
        // ═══════════════════════════════════════
        systime_t sleep_time = CH_CFG_ST_FREQUENCY / config.update_rate_hz;
        // For 500 Hz: sleep_time = 10000 / 500 = 20 ticks (2ms)

        if (sleep_time == 0) {
            sleep_time = 1;  // At least 1 tick
        }
        chThdSleep(sleep_time);
        // ChibiOS: Yield CPU for 2ms

        if (stop_now) {
            is_running = false;
            return;
        }

        // ═══════════════════════════════════════
        // Step 2b: Read External ADC Pin Voltage
        // ═══════════════════════════════════════
        float pwr = ADC_VOLTS(ADC_IND_EXT);
        // Macro expands to:
        // pwr = (float)ADC_Value[6] / 4096.0 * 3.3
        // pwr = 3103 / 4096.0 * 3.3 = 2.50V

        // Override for LISP control (if detached)
        if (adc_detached == 1 || adc_detached == 2) {
            pwr = adc1_override;
        }

        // ═══════════════════════════════════════
        // Step 2c: Apply Low-Pass Filter
        // ═══════════════════════════════════════
        static float read_filter = 0.0;
        UTILS_LP_MOVING_AVG_APPROX(read_filter, pwr, FILTER_SAMPLES);
        // FILTER_SAMPLES = 5
        // Approximate exponential moving average
        // read_filter = (read_filter * 4 + pwr) / 5

        if (config.use_filter) {
            read_voltage = read_filter;
        } else {
            read_voltage = pwr;
        }
        // Result: read_voltage = 2.50V (filtered)

        // ═══════════════════════════════════════
        // Step 2d: Range Check
        // ═══════════════════════════════════════
        range_ok = read_voltage >= config.voltage_min &&
                   read_voltage <= config.voltage_max;
        // Config example:
        // voltage_min = 0.0V
        // voltage_max = 3.3V
        // Result: range_ok = true

        // ═══════════════════════════════════════
        // Step 2e: Map Voltage to Normalized Range
        // ═══════════════════════════════════════
        switch (config.ctrl_type) {
        case ADC_CTRL_TYPE_CURRENT:
        case ADC_CTRL_TYPE_CURRENT_REV_BUTTON:
            // Linear mapping: voltage_start → voltage_end = 0.0 → 1.0
            pwr = utils_map(pwr, config.voltage_start, config.voltage_end, 0.0, 1.0);
            // Config example:
            // voltage_start = 0.5V (pedal idle)
            // voltage_end = 3.0V (pedal full)
            // Calculation:
            // pwr = (2.5 - 0.5) / (3.0 - 0.5) = 2.0 / 2.5 = 0.8
            break;

        case ADC_CTRL_TYPE_CURRENT_REV_CENTER:
        case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_CENTER:
            // Mapping with center voltage (for throttle/brake combined)
            if (pwr < config.voltage_center) {
                // Brake region
                pwr = utils_map(pwr, config.voltage_start,
                              config.voltage_center, 0.0, 0.5);
            } else {
                // Throttle region
                pwr = utils_map(pwr, config.voltage_center,
                              config.voltage_end, 0.5, 1.0);
            }
            break;
        }
        // Result: pwr = 0.8 (80% of range)

        // ═══════════════════════════════════════
        // Step 2f: Apply Filter to Mapped Value
        // ═══════════════════════════════════════
        static float pwr_filter = 0.0;
        UTILS_LP_MOVING_AVG_APPROX(pwr_filter, pwr, FILTER_SAMPLES);

        if (config.use_filter) {
            pwr = pwr_filter;
        }

        // ═══════════════════════════════════════
        // Step 2g: Truncate and Invert (if configured)
        // ═══════════════════════════════════════
        utils_truncate_number(&pwr, 0.0, 1.0);

        if (config.voltage_inverted) {
            pwr = 1.0 - pwr;
        }

        decoded_level = pwr;  // Store for debugging/telemetry
        // Result: pwr = 0.8

        // ═══════════════════════════════════════
        // Step 2h: Read Secondary ADC (Brake)
        // ═══════════════════════════════════════
#ifdef ADC_IND_EXT2
        float brake = ADC_VOLTS(ADC_IND_EXT2);
        // Same processing as above for brake pedal
#else
        float brake = 0.0;
#endif

        // Override for LISP control (if detached)
        if (adc_detached == 1 || adc_detached == 3) {
            brake = adc2_override;
        }

        read_voltage2 = brake;
        // [Brake processing similar to throttle...]
        decoded_level2 = brake;

        // ═══════════════════════════════════════
        // Step 2i: Read Button Inputs (Optional)
        // ═══════════════════════════════════════
        bool cc_button = false;  // Cruise control button
        bool rev_button = false; // Reverse button

        if (use_rx_tx_as_buttons) {
            cc_button = !palReadPad(HW_UART_TX_PORT, HW_UART_TX_PIN);
            rev_button = !palReadPad(HW_UART_RX_PORT, HW_UART_RX_PIN);
        } else {
            // Use PPM/ICU pin as button
            if (CTRL_USES_BUTTON(config.ctrl_type)) {
                rev_button = !palReadPad(HW_ICU_GPIO, HW_ICU_PIN);
            } else {
                cc_button = !palReadPad(HW_ICU_GPIO, HW_ICU_PIN);
            }
        }

        // Override for LISP control
        if (buttons_detached) {
            cc_button = cc_override;
            rev_button = rev_override;
        }

        // ═══════════════════════════════════════
        // Step 2j: Check if Output Enabled
        // ═══════════════════════════════════════
        if (app_is_output_disabled()) {
            continue;  // Don't send motor commands
        }

        if (adc_detached && timeout_has_timeout()) {
            continue;  // LISP control timed out
        }

        // ═══════════════════════════════════════
        // Step 2k: Handle Center/Button Modes
        // ═══════════════════════════════════════
        switch (config.ctrl_type) {
        case ADC_CTRL_TYPE_CURRENT_REV_CENTER:
        case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_CENTER:
            // Scale to [-1.0, +1.0] range
            pwr *= 2.0;
            pwr -= 1.0;
            // Result: pwr = 0.8 * 2 - 1 = 0.6
            break;

        case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_ADC:
        case ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_ADC:
            // Subtract brake from throttle
            pwr -= brake;
            break;

        case ADC_CTRL_TYPE_CURRENT_REV_BUTTON:
            // Invert if reverse button pressed
            if (rev_button) {
                pwr = -pwr;
            }
            break;
        }

        // ═══════════════════════════════════════
        // Step 2l: Apply Deadband
        // ═══════════════════════════════════════
        utils_deadband(&pwr, config.hyst, 1.0);
        // Hysteresis around zero (typically 0.05)
        // If |pwr| < 0.05, set pwr = 0
        // Result: pwr = 0.6 (> 0.05, no change)

        // ═══════════════════════════════════════
        // Step 2m: Apply Throttle Curve
        // ═══════════════════════════════════════
        pwr = utils_throttle_curve(pwr, config.throttle_exp,
                                   config.throttle_exp_brake,
                                   config.throttle_exp_mode);
        // Example: Exponential curve with exp = 0.5
        // For forward: pwr = sign(pwr) * |pwr|^0.5
        // Result: pwr = 0.6^0.5 = 0.775

        // ═══════════════════════════════════════
        // Step 2n: Apply Ramping
        // ═══════════════════════════════════════
        static systime_t last_time = 0;
        static float pwr_ramp = 0.0;

        float ramp_time = fabsf(pwr) > fabsf(pwr_ramp) ?
                         config.ramp_time_pos : config.ramp_time_neg;
        // ramp_time_pos = 0.5 seconds (acceleration ramp)
        // ramp_time_neg = 0.1 seconds (deceleration ramp)

        if (ramp_time > 0.01) {
            const float ramp_step = (float)ST2MS(chVTTimeElapsedSinceX(last_time)) /
                                   (ramp_time * 1000.0);
            // For 2ms interval and 0.5s ramp:
            // ramp_step = 2ms / 500ms = 0.004 (0.4% per iteration)

            utils_step_towards(&pwr_ramp, pwr, ramp_step);
            last_time = chVTGetSystemTimeX();
            pwr = pwr_ramp;
        }
        // Result: pwr gradually ramps toward 0.775

        // ═══════════════════════════════════════
        // Step 2o: Convert to Motor Command
        // ═══════════════════════════════════════
        float current_rel = 0.0;
        bool current_mode = false;
        bool current_mode_brake = false;

        switch (config.ctrl_type) {
        case ADC_CTRL_TYPE_CURRENT:
        case ADC_CTRL_TYPE_CURRENT_REV_CENTER:
        case ADC_CTRL_TYPE_CURRENT_REV_BUTTON:
            current_mode = true;
            current_rel = pwr;  // 0.775
            break;

        case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_CENTER:
        case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_ADC:
            current_mode = true;
            if (pwr >= 0.0) {
                // Throttle mode
                current_rel = pwr;
            } else {
                // Brake mode
                current_rel = fabsf(pwr);
                current_mode_brake = true;
            }
            break;

        case ADC_CTRL_TYPE_DUTY:
            // Duty cycle mode
            const volatile mc_configuration *mcconf = mc_interface_get_configuration();
            mc_interface_set_duty(utils_map(pwr, -1.0, 1.0,
                                           -mcconf->l_max_duty,
                                           mcconf->l_max_duty));
            break;

        case ADC_CTRL_TYPE_PID:
            // Speed control mode
            const volatile mc_configuration *mcconf = mc_interface_get_configuration();
            float speed = pwr >= 0.0 ?
                         pwr * mcconf->l_max_erpm :
                         pwr * fabsf(mcconf->l_min_erpm);
            mc_interface_set_pid_speed(speed);
            break;
        }

        // ═══════════════════════════════════════
        // Step 2p: Safe Start Check
        // ═══════════════════════════════════════
        if ((ms_without_power < MIN_MS_WITHOUT_POWER && config.safe_start) ||
            !range_ok) {
            // Don't allow motor start if:
            // - Pedal hasn't been at zero for 500ms
            // - ADC voltage out of range
            mc_interface_set_brake_current(timeout_get_brake_current());
            continue;
        }

        // ═══════════════════════════════════════
        // Step 2q: Send Motor Command
        // ═══════════════════════════════════════
        if (current_mode) {
            if (current_mode_brake) {
                mc_interface_set_brake_current_rel(current_rel);
            } else {
                mc_interface_set_current_rel(current_rel);
                // Calls: mc_interface_set_current_rel(0.775)
                // This translates to:
                // Absolute current = 0.775 * mcconf->lo_current_max
                // Example: 0.775 * 60A = 46.5A
            }

            // Multi-ESC support (traction control)
            if (config.multi_esc) {
                // Send commands to other VESCs on CAN bus
                for (int i = 0; i < CAN_STATUS_MSGS_TO_STORE; i++) {
                    can_status_msg *msg = comm_can_get_status_msg_index(i);

                    if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
                        if (current_mode_brake) {
                            comm_can_set_current_brake_rel(msg->id, current_rel);
                        } else {
                            // Traction control logic
                            comm_can_set_current_rel(msg->id, current_out);
                        }
                    }
                }
            }
        }

        // Reset timeout
        if (!adc_detached) {
            timeout_reset();
        }
    }
}
```

**Duration:** ~100-200 µs (depends on complexity)
**Blocks:** No (only at sleep)
**ChibiOS Calls:**
- `chRegSetThreadName()` - Register thread name
- `chThdSleep()` - Yield CPU for configured period
- `chVTTimeElapsedSinceX()` - Get elapsed time for ramping
- `chVTGetSystemTimeX()` - Get current system time

**Timeline:**
```
0 µs:     Thread wakes up (2ms expired for 500Hz)
10 µs:    Read ADC value from buffer
20 µs:    Apply voltage mapping
30 µs:    Apply filter
50 µs:    Check range
70 µs:    Apply throttle curve
100 µs:   Calculate ramp
150 µs:   Call mc_interface_set_current_rel(0.775)
200 µs:   Thread sleeps for 2ms
```

---

### Step 3: Motor Control Interface (Same as PPM)

**File:** `motor/mc_interface.c:661`

From this point, the call flow is **identical to the PPM input flow** described in `05_motor_control_call_flow.md`:

1. `mc_interface_set_current_rel()` → Converts relative current to absolute
2. `mc_interface_set_current()` → Checks limits and timeout
3. `mcpwm_foc_set_current()` → Stores setpoint in `motor->m_iq_set`
4. FOC ISR reads setpoint and executes control algorithm
5. PWM registers updated, motor responds

**See `05_motor_control_call_flow.md` Steps 4-9 for complete details.**

---

## Timing Analysis - ADC Input vs PPM Input

### Latency Breakdown

**From Hall Pedal Movement to Motor Response:**

| Stage | Duration | Accumulated | Notes |
|-------|----------|-------------|-------|
| ADC conversion | 3 µs | 3 µs | Hardware, continuous |
| DMA transfer | 1 µs | 4 µs | Hardware |
| Wait for ADC thread | 0-2 ms | 2 ms | For 500 Hz thread |
| ADC thread processing | 150 µs | 2.15 ms | Thread |
| mc_interface call | 5 µs | 2.155 ms | Thread |
| mcpwm_foc_set_current | 2 µs | 2.157 ms | Thread |
| Wait for next ISR | 0-25 µs | 2.18 ms | ISR period |
| FOC ISR execution | 12 µs | 2.192 ms | Interrupt |
| **TOTAL LATENCY** | **~2-4 ms** | | **250-500 Hz effective** |

### Comparison: ADC vs PPM Latency

| Input Type | Thread Rate | Typical Latency | Effective Response |
|------------|-------------|-----------------|-------------------|
| **ADC (Hall Pedal)** | 500 Hz | 2-4 ms | 250-500 Hz |
| **PPM (RC Receiver)** | 100 Hz | 10-20 ms | 50-100 Hz |

**Key Advantage:** ADC input is **5x faster** than PPM because:
- ADC thread can run at 500-1000 Hz (vs 100 Hz for PPM)
- No pulse width measurement required
- Continuous ADC conversion (no waiting for pulses)

---

## Configuration Parameters

### ADC App Configuration (VESC Tool)

**Location:** VESC Tool → App Settings → General → ADC

**Essential Parameters:**

| Parameter | Description | Typical Value | File Location |
|-----------|-------------|---------------|---------------|
| `ctrl_type` | Control mode | `ADC_CTRL_TYPE_CURRENT` | `datatypes.h:360` |
| `voltage_start` | Voltage at pedal idle | 0.5V | `applications/app_adc.c:218` |
| `voltage_end` | Voltage at pedal full press | 3.0V | `applications/app_adc.c:222` |
| `voltage_min` | Minimum valid voltage | 0.0V | `applications/app_adc.c:207` |
| `voltage_max` | Maximum valid voltage | 3.3V | `applications/app_adc.c:207` |
| `voltage_center` | Center voltage (for brake modes) | 1.65V | `applications/app_adc.c:217` |
| `voltage_inverted` | Invert voltage mapping | `false` | `applications/app_adc.c:244` |
| `use_filter` | Enable low-pass filter | `true` | `applications/app_adc.c:201` |
| `update_rate_hz` | Thread update rate | 500 Hz | `applications/app_adc.c:171` |
| `hyst` | Deadband (hysteresis) | 0.05 | `applications/app_adc.c:375` |
| `throttle_exp` | Throttle exponential curve | 0.0 to 3.0 | `applications/app_adc.c:378` |
| `throttle_exp_brake` | Brake exponential curve | 0.0 to 3.0 | `applications/app_adc.c:378` |
| `ramp_time_pos` | Acceleration ramp time | 0.5 s | `applications/app_adc.c:383` |
| `ramp_time_neg` | Deceleration ramp time | 0.1 s | `applications/app_adc.c:383` |
| `safe_start` | Require pedal at zero to start | `SAFE_START_REGULAR` | `applications/app_adc.c:215` |

### Control Type Options

**File:** `datatypes.h:360-372`

```c
typedef enum {
    ADC_CTRL_TYPE_NONE = 0,
    ADC_CTRL_TYPE_CURRENT,                          // Current control, no reverse
    ADC_CTRL_TYPE_CURRENT_REV_CENTER,               // Current, reverse via center voltage
    ADC_CTRL_TYPE_CURRENT_REV_BUTTON,               // Current, reverse via button
    ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_ADC,     // Current, reverse button, brake on ADC2
    ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_CENTER,  // Current, reverse button, brake via center
    ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_CENTER,       // Current forward only, brake via center
    ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_BUTTON,       // Current forward only, brake via button
    ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_ADC,          // Current forward only, brake on ADC2
    ADC_CTRL_TYPE_DUTY,                             // Duty cycle control
    ADC_CTRL_TYPE_DUTY_REV_CENTER,                  // Duty cycle, reverse via center
    ADC_CTRL_TYPE_DUTY_REV_BUTTON,                  // Duty cycle, reverse via button
    ADC_CTRL_TYPE_PID,                              // PID speed control
    ADC_CTRL_TYPE_PID_REV_CENTER,                   // PID speed, reverse via center
    ADC_CTRL_TYPE_PID_REV_BUTTON                    // PID speed, reverse via button
} adc_control_type;
```

**Most Common for E-bikes/E-scooters:**
- `ADC_CTRL_TYPE_CURRENT`: Simple throttle-only control
- `ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_CENTER`: Combined throttle/brake pedal
- `ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_ADC`: Separate throttle and brake pedals

---

## ADC Hardware Details

### ADC Conversion and DMA

**File:** `hwconf/hw.c` (hardware initialization)

**ADC Configuration:**
- **Resolution:** 12-bit (0-4095)
- **Reference Voltage:** 3.3V (internal regulator)
- **Conversion Mode:** Regular group, continuous scan
- **Trigger:** Software trigger (continuous)
- **DMA Mode:** Circular buffer (auto-restart)
- **Sampling Time:** 15 cycles @ 84 MHz = ~180 ns
- **Total Conversion Time:** ~3 µs per channel

**ADC Channels Scanned:**
```
ADC Regular Scan Sequence (18 channels):
0:  SENS1        (Phase A voltage)
1:  SENS2        (Phase B voltage)
2:  SENS3        (Phase C voltage)
3:  CURR1        (Phase A current)
4:  CURR2        (Phase B current)
5:  CURR3        (Phase C current)
6:  ADC_EXT      ← Hall pedal input
7:  ADC_EXT2     ← Brake pedal input (optional)
8:  TEMP_MOS     (MOSFET temperature)
9:  TEMP_MOTOR   (Motor temperature)
10: Shutdown     (Shutdown button)
11: VIN_SENS     (Battery voltage)
12: Vrefint      (Reference voltage)
[Additional channels for redundancy/filtering]
```

**DMA Operation:**
```c
// Simplified DMA operation (actual code in hw.c)
uint16_t ADC_Value[HW_ADC_CHANNELS];  // Global buffer

void adc_init(void) {
    // Configure ADC for continuous conversion
    // Configure DMA to auto-transfer ADC results to ADC_Value[]
    // Start ADC conversion

    // DMA continuously updates ADC_Value[] array
    // No CPU intervention required
}

// Application thread reads from buffer:
float voltage = ADC_VOLTS(ADC_IND_EXT);
// Expands to:
// voltage = (float)ADC_Value[6] / 4096.0 * 3.3;
```

### ADC Voltage Macro

**File:** `hwconf/trampa/60_75/hw_60_75_core.h:149`

```c
// Convert ADC reading to voltage
#define ADC_VOLTS(ch)  ((float)ADC_Value[ch] / 4096.0 * V_REG)

// Example:
// ADC_Value[ADC_IND_EXT] = 3103
// ADC_VOLTS(ADC_IND_EXT) = 3103 / 4096.0 * 3.3 = 2.50V
```

---

## Key Differences Summary: ADC vs PPM

| Aspect | ADC Input (Hall Pedal) | PPM Input (RC Receiver) |
|--------|----------------------|------------------------|
| **Hardware** | STM32 ADC peripheral | STM32 TIM4 input capture |
| **Signal Type** | Analog voltage (0-3.3V) | Pulse width (1-2ms) |
| **Pins** | PA5 (ADC_EXT), PA6 (ADC_EXT2) | PB6 (TIM4_CH1) |
| **Thread** | `app_adc_thread` | `app_ppm_thread` |
| **File** | `applications/app_adc.c` | `applications/app_ppm.c` |
| **Update Rate** | 100-1000 Hz (configurable) | 100 Hz (fixed) |
| **Typical Latency** | 2-4 ms | 10-20 ms |
| **CPU Usage** | Very low (DMA-based) | Low (hardware timer) |
| **Input Mapping** | Voltage range mapping | Pulse width mapping |
| **Filtering** | 5-sample moving average | 5-sample moving average |
| **Curves** | Exponential, polynomial | Exponential, polynomial |
| **Ramping** | Configurable accel/decel | Configurable accel/decel |
| **Reverse** | Via button or center voltage | Via button or pulse range |
| **Brake** | Separate ADC or center voltage | Separate channel or pulse range |
| **Multi-ESC** | CAN commands with traction control | CAN commands |

---

## Common Use Cases

### 1. E-bike with Hall Effect Throttle

**Hardware:**
- Hall effect thumb throttle: 0.8V (idle) to 3.2V (full)

**Configuration:**
```c
ctrl_type = ADC_CTRL_TYPE_CURRENT
voltage_start = 0.8V
voltage_end = 3.2V
voltage_inverted = false
use_filter = true
update_rate_hz = 500
safe_start = SAFE_START_REGULAR
ramp_time_pos = 0.3s    // Smooth acceleration
ramp_time_neg = 0.1s    // Quick deceleration
throttle_exp = 0.5      // Exponential curve for smooth feel
```

**Wiring:**
```
Thumb Throttle          VESC COMM Port
┌─────────┐            ┌──────────┐
│  RED    │───────────→│ 5V       │
│  BLACK  │───────────→│ GND      │
│  GREEN  │───────────→│ ADC_EXT  │
└─────────┘            └──────────┘
```

---

### 2. E-scooter with Combined Throttle/Brake Pedal

**Hardware:**
- Hall effect pedal: 1.0V (full brake) → 1.65V (center/idle) → 3.0V (full throttle)

**Configuration:**
```c
ctrl_type = ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_CENTER
voltage_start = 1.0V     // Full brake voltage
voltage_center = 1.65V   // Idle voltage
voltage_end = 3.0V       // Full throttle voltage
voltage_inverted = false
use_filter = true
update_rate_hz = 500
safe_start = SAFE_START_REGULAR
ramp_time_pos = 0.4s
ramp_time_neg = 0.2s
```

**Operation:**
- Pedal idle (1.65V): No throttle, no brake
- Pedal forward (1.65V → 3.0V): Throttle increases
- Pedal backward (1.65V → 1.0V): Brake increases

---

### 3. Electric Car with Separate Throttle and Brake Pedals

**Hardware:**
- Throttle pedal: 0.5V (idle) to 2.5V (full)
- Brake pedal: 0.5V (idle) to 2.5V (full)

**Configuration:**
```c
ctrl_type = ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_ADC
voltage_start = 0.5V     // Throttle idle
voltage_end = 2.5V       // Throttle full
voltage2_start = 0.5V    // Brake idle
voltage2_end = 2.5V      // Brake full
voltage_inverted = false
voltage2_inverted = false
use_filter = true
update_rate_hz = 1000    // High rate for car responsiveness
safe_start = SAFE_START_REGULAR
```

**Wiring:**
```
Throttle Pedal          VESC COMM Port
┌─────────┐            ┌──────────┐
│  RED    │───────────→│ 5V       │
│  BLACK  │───────────→│ GND      │
│  YELLOW │───────────→│ ADC_EXT  │
└─────────┘            │          │

Brake Pedal            │          │
┌─────────┐            │          │
│  RED    │───────────→│ 5V       │
│  BLACK  │───────────→│ GND      │
│  YELLOW │───────────→│ ADC_EXT2 │
└─────────┘            └──────────┘
```

---

## Debugging Tips

### 1. Monitor ADC Values in Real-Time

**VESC Tool → Terminal:**
```
# Read raw ADC value
get_adc 6

# Read voltage
get_adc_decoded 0

# Read secondary ADC
get_adc 7
get_adc_decoded 1
```

**Expected Output:**
```
ADC Channel 6: 3103 (raw)
ADC Voltage 0: 2.50V (mapped to 0.80 throttle)
ADC Channel 7: 620 (raw)
ADC Voltage 1: 0.50V (mapped to 0.00 brake)
```

### 2. Test ADC Mapping without Motor

**VESC Tool → App Settings → General:**
- Enable "Send CAN Status Messages"
- Set update rate to 50 Hz

**Monitor in real-time plotting:**
- ADC voltage vs throttle output
- Verify voltage_start/end mapping
- Check filter response
- Verify throttle curve

### 3. GPIO Toggle for Timing Measurement

**File:** Custom code in `app_adc.c`

```c
static THD_FUNCTION(adc_thread, arg) {
    // Add at start of loop
    palSetPad(GPIOA, 8);  // Set PA8 high

    // [ADC processing code...]

    // Add at end of loop (before sleep)
    palClearPad(GPIOA, 8);  // Set PA8 low

    chThdSleep(sleep_time);
}

// Oscilloscope on PA8 shows:
// - Pulse width = thread execution time
// - Pulse period = thread update rate
```

### 4. Check for ADC Noise

**Symptoms:**
- Jittery motor response at idle
- Throttle jumps unexpectedly
- "range_ok" false alarms

**Solutions:**
```c
// Increase filter samples (requires recompile)
#define FILTER_SAMPLES  10  // Was 5

// Or in VESC Tool:
// - Enable "Use Filter"
// - Increase ramp times (smooths noise)
// - Adjust voltage_min/max to avoid noise regions
```

### 5. Verify Voltage Divider (if used)

**Test:**
1. Disconnect pedal
2. Apply known voltage to ADC_EXT (e.g., 2.0V from bench supply)
3. Read voltage in VESC Tool terminal: `get_adc_decoded 0`
4. Verify: Displayed voltage = Applied voltage

**If mismatch:**
- Check resistor values (actual vs calculated)
- Verify ADC reference voltage (should be 3.3V)
- Check for loading effects (add buffer if needed)

---

## Safety Considerations

### 1. Over-Voltage Protection

**IMPORTANT:** VESC ADC pins are **NOT 5V tolerant**. Maximum input is **3.3V**.

**Protection Methods:**
- **Voltage Divider:** As shown in Hardware Connection section
- **Zener Diode:** 3.3V zener to GND (with series resistor)
- **Buffer IC:** Use 3.3V output buffer (e.g., MCP6001)

**Circuit Example (Zener Protection):**
```
Pedal Signal ─── 1kΩ ───┬─── ADC_EXT (PA5)
                        │
                       ┴─ 3.3V Zener
                       │
                      GND
```

### 2. Safe Start Configuration

**Prevent unintended motor start:**

```c
safe_start = SAFE_START_REGULAR  // Require pedal at zero for 500ms
```

**How it works (`applications/app_adc.c:483-502`):**
```c
if ((ms_without_power < MIN_MS_WITHOUT_POWER && config.safe_start) || !range_ok) {
    // Pedal hasn't been at zero for 500ms, or ADC out of range
    // Apply brake instead of motor command
    mc_interface_set_brake_current(timeout_get_brake_current());
    continue;  // Don't send throttle command
}
```

### 3. Range Checking

**Detect disconnected or faulty pedal:**

```c
range_ok = read_voltage >= config.voltage_min &&
           read_voltage <= config.voltage_max;
```

**Recommended Settings:**
- `voltage_min = 0.0V` (detect short to GND)
- `voltage_max = 3.3V` (detect open circuit or overvoltage)

**Fault Behavior:**
- If ADC voltage outside range, motor command blocked
- Brake current applied
- Fault logged for diagnostics

---

## Performance Optimization

### 1. Optimal Thread Update Rate

**Trade-offs:**

| Update Rate | Latency | CPU Usage | Use Case |
|-------------|---------|-----------|----------|
| 100 Hz | 10 ms | 1% | Low-performance applications |
| 500 Hz | 2 ms | 2% | **Recommended for most uses** |
| 1000 Hz | 1 ms | 4% | High-performance racing |

**Configuration:**
```c
config.update_rate_hz = 500;  // VESC Tool → App Settings → ADC
```

**CPU Usage Calculation:**
```
Thread execution time: ~150 µs
At 500 Hz: 150 µs × 500 = 75,000 µs/s = 7.5% theoretical
Actual: ~2% (due to sleep time)
```

### 2. Filter Tuning

**Low-Pass Filter Trade-offs:**

```c
// More samples = smoother, but more latency
#define FILTER_SAMPLES  5   // Default (good balance)
#define FILTER_SAMPLES  10  // Smoother (more latency)
#define FILTER_SAMPLES  2   // Faster (more noise)
```

**Filter Latency:**
```
Settling time ≈ FILTER_SAMPLES / update_rate_hz
For FILTER_SAMPLES=5 at 500 Hz: 5/500 = 10ms
```

**Alternative:** Use hardware RC filter on pedal signal:
```
Pedal Signal ─── 1kΩ ───┬─── ADC_EXT
                        │
                       === 10µF
                        │
                       GND

Cutoff frequency: fc = 1 / (2π × 1kΩ × 10µF) ≈ 16 Hz
Good for noise rejection, no CPU overhead
```

### 3. Minimize Thread Duration

**Current Implementation:** ~150 µs
**Optimization Opportunities:**
- Remove unnecessary floating-point operations
- Pre-calculate constants
- Use lookup tables for exponential curves
- Optimize multi-ESC traction control

---

## Summary

**Key Takeaways:**

1. **Hardware Connection:**
   - Hall effect pedal connects to **PA5** (ADC_EXT) on COMM port
   - Optional brake pedal on **PA6** (ADC_EXT2)
   - Voltage range: **0-3.3V** (over-voltage protection required)

2. **Control Flow:**
   - ADC continuously samples pedal voltage via DMA
   - `app_adc_thread` reads buffer at 100-1000 Hz
   - Voltage mapped, filtered, and converted to motor command
   - `mc_interface_set_current_rel()` → same flow as PPM

3. **Performance:**
   - **5x lower latency** than PPM (2-4ms vs 10-20ms)
   - Typical configuration: 500 Hz update rate
   - ~2% CPU usage

4. **Configuration:**
   - VESC Tool → App Settings → General → ADC
   - Critical: `voltage_start`, `voltage_end`, `ctrl_type`
   - Safety: `safe_start`, `voltage_min/max`

5. **Common Applications:**
   - E-bikes: Single throttle, 500 Hz
   - E-scooters: Combined throttle/brake, center voltage mode
   - Electric cars: Dual pedals (ADC_EXT + ADC_EXT2), 1000 Hz

**Next Steps:**
- Refer to `05_motor_control_call_flow.md` for FOC ISR details
- See `03_app.md` for application layer architecture
- Check `07_adding_custom_inputs_guide.md` for custom ADC processing
