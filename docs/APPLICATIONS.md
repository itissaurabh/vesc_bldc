# Application Layer Documentation

## Table of Contents

1. [Introduction](#introduction)
2. [Application Framework](#application-framework)
3. [ADC Application](#adc-application)
4. [PPM Application](#ppm-application)
5. [Nunchuk Application](#nunchuk-application)
6. [PAS Application](#pas-application)
7. [UART Communication Application](#uart-communication-application)
8. [Custom Applications](#custom-applications)
9. [Configuration](#configuration)
10. [Practical Examples](#practical-examples)

---

## Introduction

The `/applications/` directory implements the **application layer** - the bridge between user input and motor control. Applications read various input sources (throttles, controllers, sensors) and translate them into motor commands.

**Purpose:**
- Convert human input into motor commands
- Support multiple input types (analog, digital, wireless)
- Provide safety features (timeout, ramping, limits)
- Enable different vehicle types (bikes, scooters, skateboards, etc.)

**Available Applications:**

| Application | Input Type | Use Case |
|-------------|------------|----------|
| **ADC** | Analog voltage (0-3.3V) | Thumb throttle, twist grip, pedal sensor |
| **PPM** | PWM pulse width | RC receiver, servo tester |
| **Nunchuk** | I2C (Wii controller) | Handheld controller, wireless input |
| **PAS** | Digital pulses | E-bike pedal assist sensor |
| **UART** | Serial communication | Custom external controllers |
| **DPV** | Custom | Diver propulsion vehicles |
| **Custom** | User-defined | Any custom input method |

---

## Application Framework

### How Applications Work

The application layer runs in a dedicated thread that:

1. **Reads input** from hardware (ADC, GPIO, I2C, etc.)
2. **Processes input** with filtering, scaling, dead zones
3. **Applies safety logic** (timeouts, ramp limits)
4. **Sends motor commands** via `mc_interface`

```
┌────────────────────────────────────────┐
│  Input Source                          │
│  (Throttle, Controller, Sensor)        │
└───────────────┬────────────────────────┘
                │
                ▼
┌────────────────────────────────────────┐
│  Application Thread                    │
│  - Read input                          │
│  - Filter & scale                      │
│  - Safety checks                       │
└───────────────┬────────────────────────┘
                │
                ▼
┌────────────────────────────────────────┐
│  Motor Control Interface               │
│  (mc_interface_set_current, etc.)      │
└───────────────┬────────────────────────┘
                │
                ▼
            Motor Control
```

### Application State Machine

Each application runs a simple state machine:

**States:**
- **IDLE** - No input, motor released
- **RUNNING** - Valid input, motor active
- **TIMEOUT** - Lost input signal, safety stop

**Transitions:**
```
        Valid Input
IDLE ───────────────────→ RUNNING
  ↑                          │
  │                          │ Timeout/Stop
  └──────────────────────────┘
```

### Safety Features

All applications include:

1. **Timeout Protection:**
   - If no valid input for N milliseconds, stop motor
   - Prevents runaway if controller disconnects

2. **Soft Start:**
   - Gradual power ramp on startup
   - Prevents sudden acceleration

3. **Current Ramping:**
   - Limits rate of change
   - Smooth acceleration/deceleration

4. **Direction Control:**
   - Forward/reverse logic
   - Speed-based direction switching

5. **Cruise Control:**
   - Maintain speed without input (some apps)

---

## ADC Application

**File:** `app_adc.c/.h`

### Overview

The ADC application reads one or two analog voltage inputs (0-3.3V) and converts them to motor commands. This is the most common input method for electric vehicles.

**Typical Hardware:**
- Thumb throttle (Hall effect sensor)
- Twist grip throttle
- Foot pedal
- Joystick

### ADC Theory

**Analog-to-Digital Conversion:**
```
Voltage (0-3.3V) → 12-bit ADC → Digital value (0-4095)
```

**Mapping to Motor Command:**
```
ADC Reading → Voltage → Normalized (0-1) → Motor Command
```

### Control Types

The ADC app supports multiple control modes:

| Control Type | Description | Use Case |
|--------------|-------------|----------|
| `ADC_CTRL_TYPE_NONE` | Disabled | - |
| `ADC_CTRL_TYPE_CURRENT` | Current control (forward/reverse) | Standard throttle with brake |
| `ADC_CTRL_TYPE_CURRENT_REV_CENTER` | Center = stop, fwd/rev from center | Joystick, reversible vehicles |
| `ADC_CTRL_TYPE_CURRENT_REV_BUTTON` | Button for reverse | E-scooter, e-bike |
| `ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_CENTER` | Center = stop, no reverse | One-way with brake |
| `ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_BUTTON` | Button for brake | Simple forward-only |
| `ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_ADC` | Second ADC for brake | Separate brake throttle |
| `ADC_CTRL_TYPE_DUTY` | Duty cycle control | Direct voltage control |
| `ADC_CTRL_TYPE_PID` | Speed (RPM) control | Cruise control |

### Configuration

**Key Parameters:**

```c
typedef struct {
    adc_control_type ctrl_type;     // Control type (see above)

    // ADC1 (primary throttle)
    float hyst;                      // Hysteresis (deadband)
    float voltage_start;             // Voltage at 0% throttle
    float voltage_end;               // Voltage at 100% throttle
    float voltage_center;            // Center voltage (for center modes)
    bool voltage_inverted;           // Invert ADC reading

    // ADC2 (secondary input - brake, second throttle)
    float voltage2_start;
    float voltage2_end;
    bool voltage2_inverted;

    // Safety & Response
    bool safe_start;                 // Require release before start
    float rpm_lim_start;             // RPM to start limiting
    float rpm_lim_end;               // RPM for full limit
    float ramp_time_pos;             // Ramp up time (seconds)
    float ramp_time_neg;             // Ramp down time (seconds)

    // Traction Control
    bool use_filter;                 // Enable input filtering
    float throttle_exp;              // Throttle curve exponent
    float throttle_exp_mode;         // Exponential mode

    // Buttons (optional)
    uint8_t buttons;                 // Button configuration bitmask

    // Multi-ESC
    bool multi_esc;                  // Send to multiple ESCs over CAN
    uint32_t tc_offset;              // Traction control offset
    uint32_t tc_max_diff;            // Max speed diff for TC
} adc_config;
```

### Throttle Curves

**Linear (default):**
```
Output = Input
```

**Exponential:**
```
Output = Input ^ exponent

exponent < 1.0: Softer response (more control at low speeds)
exponent > 1.0: Sharper response (more aggressive)
```

Example with exponent = 0.5:
```
Input:   0%    25%    50%    75%   100%
Output:  0%    50%    71%    87%   100%
```

### Voltage Calibration

**Process:**
1. Throttle at rest → Measure voltage → Set `voltage_start`
2. Throttle at full → Measure voltage → Set `voltage_end`
3. For center types → Throttle centered → Set `voltage_center`

**Example Throttle:**
- At rest: 0.8V → `voltage_start = 0.8`
- At full: 3.0V → `voltage_end = 3.0`
- Reading: 1.5V → Position = (1.5 - 0.8) / (3.0 - 0.8) = 0.318 = 31.8%

### Safe Start

When enabled, prevents motor from starting if throttle is active:

```c
if (safe_start && throttle_at_startup) {
    // Wait for throttle to return to zero
    while (throttle > threshold) {
        // Motor stays off
    }
}
// Now allow motor to start
```

**Use Case:** Prevents sudden acceleration when turning on VESC

### Multi-ESC and Traction Control

For multi-motor setups:

```c
adc_config.multi_esc = true;
adc_config.tc_max_diff = 3000;  // Max 3000 ERPM difference

// Main ESC reads throttle and sends commands to others via CAN
// If one wheel spins faster, reduce its power
if (rpm_diff > tc_max_diff) {
    reduce_power_to_faster_motor();
}
```

### API Functions

```c
// Start ADC application
void app_adc_start(bool use_rx_tx);

// Stop application
void app_adc_stop(void);

// Configure
void app_adc_configure(adc_config *conf);

// Read decoded throttle position (0-1)
float app_adc_get_decoded_level(void);

// Read raw voltage
float app_adc_get_voltage(void);

// Override for testing
void app_adc_adc1_override(float val);
```

---

## PPM Application

**File:** `app_ppm.c/.h`

### Overview

PPM (Pulse Position Modulation) or PWM (Pulse Width Modulation) input from RC receivers or servo testers. Reads pulse width (typically 1-2ms) and converts to motor command.

**Common Sources:**
- RC car/boat/plane receivers
- RC servo tester
- PWM-output controllers

### PPM/PWM Theory

**Signal Format:**
```
   ┌─────────┐           ┌─────────┐
   │         │           │         │
───┘         └───────────┘         └─────

   ├─Tₕ─────┤
   ├─────────Tₚ──────────┤

Tₕ = High time (pulse width): 1-2ms typical
Tₚ = Period: 20ms typical (50 Hz)

Position = Tₕ / (max_pulse - min_pulse)
```

**Standard RC Servo:**
- 1.0ms = 0% (full reverse/left)
- 1.5ms = 50% (center/neutral)
- 2.0ms = 100% (full forward/right)

### Control Types

| Control Type | Description |
|--------------|-------------|
| `PPM_CTRL_TYPE_NONE` | Disabled |
| `PPM_CTRL_TYPE_CURRENT` | Current control (bidirectional) |
| `PPM_CTRL_TYPE_CURRENT_NOREV` | Current control (forward only) |
| `PPM_CTRL_TYPE_DUTY` | Duty cycle control |
| `PPM_CTRL_TYPE_PID` | Speed control |
| `PPM_CTRL_TYPE_PID_NOREV` | Speed control (forward only) |
| `PPM_CTRL_TYPE_PID_NOACCELERATION` | Speed control without acceleration |
| `PPM_CTRL_TYPE_PID_POSITION_360` | Position control (servo mode) |

### Configuration

```c
typedef struct {
    ppm_control_type ctrl_type;    // Control mode
    float pid_max_erpm;            // Max ERPM for PID mode
    float hyst;                    // Hysteresis/deadband
    float pulse_start;             // Min pulse width (ms)
    float pulse_end;               // Max pulse width (ms)
    float pulse_center;            // Center pulse (ms)
    bool median_filter;            // Enable median filtering
    bool safe_start;               // Safe start feature
    float throttle_exp;            // Throttle curve exponent
    float throttle_exp_brake;      // Brake curve exponent
    float throttle_exp_mode;       // Exponential mode
    float ramp_time_pos;           // Positive ramp time
    float ramp_time_neg;           // Negative ramp time
    float max_erpm_for_dir;        // Max ERPM for direction change

    // Traction control
    bool multi_esc;
    uint32_t tc_max_diff;
} ppm_config;
```

### Pulse Calibration

**Procedure:**
1. Move RC stick to minimum → Measure pulse → Set `pulse_start`
2. Move RC stick to maximum → Measure pulse → Set `pulse_end`
3. Center RC stick → Measure pulse → Set `pulse_center`

**Example RC Receiver:**
- Stick down: 1.1ms → `pulse_start = 1.1`
- Stick up: 1.9ms → `pulse_end = 1.9`
- Stick center: 1.5ms → `pulse_center = 1.5`

### Median Filtering

Filters out spurious pulses:

```c
median_filter = true;

// Stores last N samples, outputs median
samples[] = {1.51, 1.52, 1.51, 1.98, 1.52}  // One outlier
median = 1.52  // Outlier rejected
```

**Use when:** Noisy RC signal, long cable runs

### Direction Control

```c
max_erpm_for_dir = 1000;  // ERPM

// Allow direction change only when motor nearly stopped
if (abs(current_erpm) < max_erpm_for_dir) {
    allow_direction_change();
} else {
    maintain_current_direction();
}
```

**Safety:** Prevents reverse while moving forward fast

### API Functions

```c
// Start PPM application
void app_ppm_start(void);

// Stop application
void app_ppm_stop(void);

// Configure
void app_ppm_configure(ppm_config *conf);

// Read decoded level (-1 to 1)
float app_ppm_get_decoded_level(void);

// Override for testing
void app_ppm_override(float val);
```

---

## Nunchuk Application

**File:** `app_nunchuk.c/.h`

### Overview

Uses Nintendo Wii Nunchuk controller via I2C for wireless motor control. The Nunchuk provides:
- 2-axis joystick (X, Y)
- 2 buttons (C, Z)
- 3-axis accelerometer

**Advantages:**
- Wireless (with Nunchuk adapter)
- Ergonomic handheld controller
- Multiple control inputs
- Low cost

### Nunchuk Hardware

**I2C Communication:**
- Address: 0x52
- Clock: 100 kHz (standard I2C)
- 6-byte data packet

**Data Format:**
```
Byte 0: Joystick X (0-255)
Byte 1: Joystick Y (0-255)
Byte 2: Accel X (high 8 bits)
Byte 3: Accel Y (high 8 bits)
Byte 4: Accel Z (high 8 bits)
Byte 5: Buttons + Accel low bits
```

### Control Types

| Control Type | Description | Joystick Usage |
|--------------|-------------|----------------|
| `CHUK_CTRL_TYPE_NONE` | Disabled | - |
| `CHUK_CTRL_TYPE_CURRENT` | Current control | Y-axis = throttle/brake |
| `CHUK_CTRL_TYPE_CURRENT_NOREV` | Current (no reverse) | Y-axis = throttle only |
| `CHUK_CTRL_TYPE_CURRENT_BIDIRECTIONAL` | Fwd on Y, rev on button | Y + Z button |

### Configuration

```c
typedef struct {
    chuk_control_type ctrl_type;
    float hyst;                    // Hysteresis
    float rpm_lim_start;
    float rpm_lim_end;
    float ramp_time_pos;
    float ramp_time_neg;
    float stick_erpm_per_s_in_cc; // Cruise control rate
    float throttle_exp;
    float throttle_exp_brake;
    float throttle_exp_mode;

    // Button configuration
    bool use_smart_rev;            // Smart reverse logic

    // Multi-ESC
    bool multi_esc;
    uint32_t tc_max_diff;
} chuk_config;
```

### Joystick Mapping

**Raw to Normalized:**
```c
// Joystick centered at 128
float x = (chuck_data.js_x - 128.0) / 128.0;  // -1.0 to 1.0
float y = (chuck_data.js_y - 128.0) / 128.0;  // -1.0 to 1.0

// Y-axis typically controls throttle/brake
// X-axis can control steering (for differential drive)
```

### Cruise Control

Hold C button to activate cruise control:

```c
if (button_c_pressed) {
    // Lock current speed
    target_speed = current_speed;

    // Joystick now adjusts from this speed
    target_speed += joystick_y * adjustment_rate;
}
```

**Use:** Long rides without holding throttle

### Smart Reverse

```c
use_smart_rev = true;

// Press Z button for reverse
// But only allow when speed is low
if (button_z && speed < threshold) {
    reverse_enabled = true;
}
```

### API Functions

```c
// Start Nunchuk application
void app_nunchuk_start(void);

// Stop application
void app_nunchuk_stop(void);

// Configure
void app_nunchuk_configure(chuk_config *conf);

// Read joystick position (-1 to 1)
float app_nunchuk_get_decoded_x(void);
float app_nunchuk_get_decoded_y(void);

// Read buttons
bool app_nunchuk_get_bt_c(void);
bool app_nunchuk_get_bt_z(void);

// Check update age (for timeout detection)
float app_nunchuk_get_update_age(void);
```

---

## PAS Application

**File:** `app_pas.c/.h`

### Overview

PAS (Pedal Assist System) for e-bikes. Detects pedaling and provides proportional motor assistance based on:
- Pedal cadence (RPM)
- Pedal torque (optional sensor)

**How it Works:**
1. Magnetic sensor on crank detects rotation
2. Counts pulses to determine cadence
3. Calculates assistance level
4. Applies motor current

### PAS Theory

**Cadence Measurement:**
```
Magnets on crank: N (typically 8-12)
Time between pulses: T seconds

Pedal RPM = (60 / T) / N
```

**Assistance Levels:**
```
No pedaling → 0% assist
Slow pedaling → Low assist (proportional)
Fast pedaling → High assist (up to max)
Stop pedaling → Ramp down to 0%
```

### PAS Sensor Types

**1. Cadence-Only PAS:**
- Detects rotation (yes/no)
- Provides fixed assistance when pedaling
- Simple, low cost

**2. Cadence + Torque PAS:**
- Measures pedal force
- Proportional to rider effort
- Better feel, more efficient

### Configuration

```c
typedef struct {
    pas_control_type ctrl_type;
    pas_sensor_type sensor_type;

    // Cadence parameters
    float current_scaling;         // Max assist current
    float pedal_rpm_start;         // RPM to start assist
    float pedal_rpm_end;           // RPM for full assist
    uint32_t magnets;              // Number of magnets on crank
    bool invert_pedal_direction;   // For reverse-mounted sensor

    // Torque parameters (if applicable)
    float torque_offset;
    float torque_max_raw;
    float torque_current_scaling;

    // Ramp
    float ramp_time_pos;
    float ramp_time_neg;
} pas_config;
```

### Assistance Curve

**Proportional Assist:**
```
                    Max Current
                   ╱
                  ╱
                 ╱
                ╱
               ╱
    ─────────╱──────────────────▶ Pedal RPM
       start RPM   end RPM

Current = map(pedal_rpm, start_rpm, end_rpm, 0, max_current)
```

**Example:**
- `pedal_rpm_start = 30` RPM
- `pedal_rpm_end = 70` RPM
- `current_scaling = 30` A

```c
if (pedal_rpm < 30) {
    assist = 0A;
} else if (pedal_rpm > 70) {
    assist = 30A;
} else {
    assist = map(pedal_rpm, 30, 70, 0, 30);  // 0-30A
}
```

### Primary vs Secondary Output

```c
// Primary: PAS controls motor directly
app_pas_start(true);

// Secondary: PAS adds to throttle input (throttle + PAS)
app_pas_start(false);
```

**Use Cases:**
- Primary: PAS-only e-bike (no throttle)
- Secondary: Throttle + PAS assist (hybrid control)

### Pedal Direction Detection

```c
invert_pedal_direction = false;  // Normal
invert_pedal_direction = true;   // Sensor mounted backwards

// Automatically detects forward/reverse pedaling
// Prevents assist when backpedaling (for coaster brake)
```

### API Functions

```c
// Start PAS application
void app_pas_start(bool is_primary_output);

// Stop application
void app_pas_stop(void);

// Configure
void app_pas_configure(pas_config *conf);

// Read current target (0-1)
float app_pas_get_current_target_rel(void);

// Read pedal RPM
float app_pas_get_pedal_rpm(void);

// Scale assist (for multi-mode support)
void app_pas_set_current_sub_scaling(float scaling);
```

---

## UART Communication Application

**File:** `app_uartcomm.c/.h`

### Overview

Allows external microcontroller or device to control VESC via UART using the standard VESC protocol. Useful for:
- Custom controllers
- Telemetry systems
- Integration with other systems
- Dual-VESC setups

### UART Ports

The VESC supports multiple UART ports:

```c
typedef enum {
    UART_PORT_COMM_HEADER = 0,    // Main comm header (USB alternative)
    UART_PORT_BUILTIN,            // Built-in UART on MCU
    UART_PORT_EXTRA_HEADER        // Extra header (hardware dependent)
} UART_PORT;
```

### Configuration

```c
// Initialize
app_uartcomm_initialize();

// Start on specific port
app_uartcomm_start(UART_PORT_COMM_HEADER);

// Configure baud rate
app_uartcomm_configure(115200, true, UART_PORT_COMM_HEADER);

// Send packet
unsigned char data[32];
app_uartcomm_send_packet(data, len, UART_PORT_COMM_HEADER);
```

### Protocol

Uses same packet protocol as USB communication:
- See [Communication Documentation](COMMUNICATION.md) for details
- All `COMM_PACKET_ID` commands supported
- Same CRC and framing

**Example - Set Current via UART:**
```c
uint8_t buffer[8];
int32_t ind = 0;

buffer[ind++] = COMM_SET_CURRENT;
buffer_append_float32(buffer, 15.0, 1000.0, &ind);  // 15A

app_uartcomm_send_packet(buffer, ind, UART_PORT_COMM_HEADER);
```

---

## Custom Applications

**Files:** `app_custom.c`, `app_custom_template.c`

### Overview

Template for creating your own application with custom input processing logic.

### Creating Custom Application

**1. Copy Template:**
```bash
cp app_custom_template.c app_custom.c
```

**2. Implement Functions:**

```c
// Called once at startup
void app_custom_start(void) {
    // Initialize your hardware
    // Start your thread
    stop_now = false;
    chThdCreateStatic(custom_thread_wa, sizeof(custom_thread_wa),
                     NORMALPRIO, custom_thread, NULL);
}

// Called to stop application
void app_custom_stop(void) {
    stop_now = true;
    // Wait for thread to exit
    while (is_running) {
        chThdSleepMilliseconds(1);
    }
}

// Main thread
static THD_FUNCTION(custom_thread, arg) {
    (void)arg;
    is_running = true;

    for(;;) {
        if (stop_now) {
            is_running = false;
            return;
        }

        // Read your input
        float input = read_my_sensor();

        // Process
        float output = process_input(input);

        // Send to motor
        mc_interface_set_current(output);

        // Sleep
        chThdSleepMilliseconds(10);
    }
}
```

**3. Configure in VESC Tool:**
- App Settings → General → App to Use: Custom

### Example Custom Applications

**Temperature-Controlled Fan:**
```c
static THD_FUNCTION(fan_thread, arg) {
    for(;;) {
        if (stop_now) return;

        float temp = mc_interface_temp_fet_filtered();

        if (temp < 50.0) {
            mc_interface_release_motor();  // Fan off
        } else if (temp < 70.0) {
            float duty = (temp - 50.0) / 20.0;  // 0-1
            mc_interface_set_duty(duty);  // Variable speed
        } else {
            mc_interface_set_duty(1.0);  // Full speed
        }

        chThdSleepMilliseconds(1000);
    }
}
```

**CAN-Controlled Motor:**
```c
static THD_FUNCTION(can_control_thread, arg) {
    for(;;) {
        if (stop_now) return;

        // Get status from another VESC on CAN bus
        can_status_msg *status = comm_can_get_status_msg_id(5);

        if (status != NULL) {
            // Mirror RPM of VESC ID 5
            float target_rpm = status->rpm;
            mc_interface_set_pid_speed(target_rpm);
        }

        chThdSleepMilliseconds(10);
    }
}
```

---

## Configuration

### Selecting Application

**In VESC Tool:**
1. App Settings → General → App to Use
2. Select from dropdown:
   - No App
   - PPM and UART
   - ADC and UART
   - ADC
   - UART
   - PPM
   - Nunchuk
   - PAS
   - ADC and PAS
   - Custom

### Configuration Storage

Configuration stored in flash memory:

```c
// Get current config
const app_configuration *conf = app_get_configuration();

// Modify config
app_configuration new_conf = *conf;
new_conf.app_to_use = APP_PPM;
// ... set PPM parameters ...

// Apply config
app_set_configuration(&new_conf);
```

### Timeout Configuration

**App Settings → General → Timeout:**

```c
app_configuration.timeout_msec = 1000;      // 1 second timeout
app_configuration.timeout_brake_current = 5.0;  // 5A brake on timeout
```

**Behavior:**
- If no valid input for `timeout_msec`, trigger timeout
- Apply `timeout_brake_current` (if > 0) or release motor (if 0)

---

## Practical Examples

### Example 1: Configure ADC Throttle

```c
#include "app.h"

void setup_adc_throttle(void) {
    adc_config conf;

    // Control type
    conf.ctrl_type = ADC_CTRL_TYPE_CURRENT_REV_BUTTON;

    // Calibration (measured from your throttle)
    conf.voltage_start = 0.8;      // Throttle at rest
    conf.voltage_end = 3.0;        // Throttle at full
    conf.voltage_inverted = false;

    // Safety
    conf.safe_start = true;        // Require release before start
    conf.hyst = 0.1;               // 10% deadband

    // Ramp times (smooth acceleration)
    conf.ramp_time_pos = 0.3;      // 300ms ramp up
    conf.ramp_time_neg = 0.1;      // 100ms ramp down

    // Throttle curve (0.5 = softer, easier control)
    conf.throttle_exp = 0.5;
    conf.throttle_exp_mode = THR_EXP_NATURAL;

    // RPM limits
    conf.rpm_lim_start = 100000;   // Start limiting at 100k ERPM
    conf.rpm_lim_end = 120000;     // Full limit at 120k ERPM

    // Apply configuration
    app_adc_configure(&conf);
    app_adc_start(false);
}
```

### Example 2: PPM RC Control

```c
void setup_rc_control(void) {
    ppm_config conf;

    // Control type
    conf.ctrl_type = PPM_CTRL_TYPE_CURRENT;

    // Calibration (measured from RC receiver)
    conf.pulse_start = 1.1;        // Stick down (1.1ms)
    conf.pulse_end = 1.9;          // Stick up (1.9ms)
    conf.pulse_center = 1.5;       // Stick centered (1.5ms)

    // Filtering
    conf.median_filter = true;     // Filter noise
    conf.hyst = 0.1;

    // Safety
    conf.safe_start = true;
    conf.max_erpm_for_dir = 2000;  // Allow direction change < 2000 ERPM

    // Throttle response
    conf.throttle_exp = 1.0;       // Linear
    conf.throttle_exp_brake = 1.0;

    // Ramps
    conf.ramp_time_pos = 0.2;
    conf.ramp_time_neg = 0.1;

    // Apply
    app_ppm_configure(&conf);
    app_ppm_start();
}
```

### Example 3: E-Bike with PAS + Throttle

```c
void setup_ebike_hybrid(void) {
    // Configure PAS for pedal assist
    pas_config pas_conf;
    pas_conf.ctrl_type = PAS_CTRL_TYPE_CADENCE;
    pas_conf.current_scaling = 20.0;     // 20A max PAS assist
    pas_conf.pedal_rpm_start = 40;       // Start at 40 RPM
    pas_conf.pedal_rpm_end = 80;         // Full assist at 80 RPM
    pas_conf.magnets = 12;               // 12-magnet sensor
    pas_conf.ramp_time_pos = 2.0;        // Slow ramp up
    pas_conf.ramp_time_neg = 1.0;        // Faster ramp down

    app_pas_configure(&pas_conf);
    app_pas_start(false);  // Secondary output (adds to throttle)

    // Configure ADC for throttle override
    adc_config adc_conf;
    adc_conf.ctrl_type = ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_CENTER;
    adc_conf.voltage_start = 0.8;
    adc_conf.voltage_end = 3.0;
    adc_conf.safe_start = false;         // Allow immediate throttle
    adc_conf.ramp_time_pos = 0.3;

    app_adc_configure(&adc_conf);
    app_adc_start(false);

    // Now: Throttle + PAS work together
    // Total current = throttle_current + pas_current (limited by max)
}
```

### Example 4: Multi-Motor Traction Control

```c
void setup_dual_motor_tc(void) {
    // This ESC is master, controls slave via CAN
    adc_config conf;
    conf.ctrl_type = ADC_CTRL_TYPE_CURRENT;
    conf.voltage_start = 0.8;
    conf.voltage_end = 3.0;

    // Enable multi-ESC and traction control
    conf.multi_esc = true;
    conf.tc_max_diff = 3000;     // Max 3000 ERPM difference
    conf.tc_offset = 0;          // No offset between motors

    app_adc_configure(&conf);
    app_adc_start(false);

    // Slave ESC (ID = 2) will receive commands from this master
    // If one motor spins faster (loss of traction), power is reduced
}
```

---

## Summary

The VESC application layer provides:

✅ **Multiple Input Methods:**
- ADC analog (throttles, potentiometers)
- PPM/PWM (RC receivers)
- I2C (Nunchuk controllers)
- Digital (PAS sensors)
- UART (external controllers)
- Custom (user-defined)

✅ **Flexible Control:**
- Current, duty, speed, position modes
- Forward-only or bidirectional
- Center-return or button-reverse

✅ **Safety Features:**
- Safe start (require release)
- Timeout protection
- Soft start/stop
- Current ramping

✅ **Advanced Features:**
- Throttle curves
- Cruise control
- Multi-ESC synchronization
- Traction control

✅ **Easy Customization:**
- Template for custom apps
- Full access to motor control API
- ChibiOS threading

**Key Files Reference:**
- `applications/app.h` - Application API (app.h:25)
- `applications/app_adc.c/.h` - ADC throttle app (app_adc.c:40)
- `applications/app_ppm.c/.h` - PPM RC app (app_ppm.c:40)
- `applications/app_nunchuk.c/.h` - Nunchuk app (app_nunchuk.c:44)
- `applications/app_pas.c/.h` - Pedal assist app (app_pas.c:44)
- `applications/app_custom_template.c` - Custom app template

