# Application Layer Manager Documentation

**File:** `applications/app.c` (221 lines)
**Header:** `applications/app.h`
**Purpose:** Application layer manager that selects, starts, stops, and configures control input applications (PPM, ADC, UART, Nunchuk, PAS, NRF, etc.)

## Overview

The application layer sits between the motor control interface (`mc_interface`) and external control inputs. It manages which application is active and handles configuration switching. Each application represents a different control input method for the motor controller.

## Supported Applications

From `datatypes.h` (Lines 588-600):

```c
typedef enum {
    APP_NONE = 0,      // No application (motor controlled via commands only)
    APP_PPM,           // RC PPM receiver input
    APP_ADC,           // Analog potentiometer/throttle
    APP_UART,          // UART communication
    APP_PPM_UART,      // PPM + UART simultaneously
    APP_ADC_UART,      // ADC + UART simultaneously
    APP_NUNCHUK,       // Wii Nunchuk I2C controller
    APP_NRF,           // Nordic NRF24L01+ wireless
    APP_CUSTOM,        // Custom user application
    APP_PAS,           // Pedal Assist Sensor (e-bike)
    APP_ADC_PAS        // ADC + PAS combined
} app_use;
```

## Application Descriptions

| Application | Input Method | Use Case | Hardware Required |
|-------------|--------------|----------|-------------------|
| **APP_NONE** | Commands only | PC/mobile control | USB/CAN/Bluetooth |
| **APP_PPM** | RC PWM signal | RC cars, drones | PPM receiver |
| **APP_ADC** | Analog voltage | Throttle pedal | Potentiometer |
| **APP_UART** | Serial data | Custom protocols | UART device |
| **APP_PPM_UART** | PPM + Serial | RC with telemetry | PPM + UART |
| **APP_ADC_UART** | ADC + Serial | Throttle + data | ADC + UART |
| **APP_NUNCHUK** | I2C joystick | Handheld control | Wii Nunchuk |
| **APP_NRF** | 2.4GHz wireless | Wireless remote | NRF24L01+ |
| **APP_CUSTOM** | User-defined | Custom hardware | Custom |
| **APP_PAS** | E-bike sensor | Electric bicycle | PAS sensor |
| **APP_ADC_PAS** | Throttle + PAS | E-bike hybrid | ADC + PAS |

## Key Data Structures

### app_configuration (from datatypes.h)

Main configuration structure containing:

```c
typedef struct {
    // Application selection
    app_use app_to_use;                  // Which application to run

    // Sub-configurations for each app
    ppm_config app_ppm_conf;             // PPM settings
    adc_config app_adc_conf;             // ADC settings
    uint32_t app_uart_baudrate;          // UART baud rate
    chuk_config app_chuk_conf;           // Nunchuk settings
    nrf_config app_nrf_conf;             // NRF24 settings
    pas_config app_pas_conf;             // PAS settings
    imu_config imu_conf;                 // IMU configuration

    // CAN settings
    CAN_BAUD can_baud_rate;              // CAN bus speed

    // Other options
    bool servo_out_enable;               // Enable servo output
    bool permanent_uart_enabled;         // Keep UART always on

    unsigned short crc;                  // Configuration CRC
} app_configuration;
```

## Global State

**File-scoped Variables (Lines 34-38):**

```c
static app_configuration appconf = {0};        // Current active configuration
static virtual_timer_t output_vt = {0};        // Virtual timer for output disable
static bool output_vt_init_done = false;       // Timer initialization flag
static volatile bool output_disabled_now = false;  // Output disable state
```

---

## Function Documentation

### 1. Configuration Functions

#### app_get_configuration() - Lines 43-45

**Purpose:** Get read-only pointer to current application configuration

**Parameters:** None

**Returns:** `const app_configuration*` - Pointer to active configuration

**Usage:**
```c
const app_configuration *conf = app_get_configuration();
if (conf->app_to_use == APP_PPM) {
    commands_printf("PPM application active");
}
```

---

#### app_set_configuration() - Lines 53-164

**Purpose:** Apply new application configuration. Stops old application, starts new one, and configures all subsystems.

**Parameters:**
- `conf`: Pointer to new configuration to apply

**Operation Flow:**

```
1. Detect Configuration Changes
   ├─ Check if app_to_use changed
   └─ Check if servo_out_enable changed

2. If Application Changed:
   ├─ Stop all applications:
   │   ├─ app_ppm_stop()
   │   ├─ app_adc_stop()
   │   ├─ app_uartcomm_stop()
   │   ├─ app_nunchuk_stop()
   │   ├─ app_pas_stop()
   │   └─ app_custom_stop() [if enabled]
   │
   └─ Stop NRF driver (if not permanent)

3. Update CAN Baud Rate
   └─ comm_can_set_baud()

4. Re-initialize IMU
   └─ imu_init()

5. Configure Servo Output:
   ├─ If servo output enabled AND not PPM app:
   │   ├─ servodec_stop()
   │   └─ pwm_servo_init_servo()
   └─ Else:
       └─ pwm_servo_stop()

6. Start Selected Application:
   Switch (app_to_use):
   ├─ APP_PPM:        app_ppm_start()
   ├─ APP_ADC:        app_adc_start(true)
   ├─ APP_UART:       hw_stop_i2c() + app_uartcomm_start()
   ├─ APP_PPM_UART:   app_ppm_start() + app_uartcomm_start()
   ├─ APP_ADC_UART:   app_adc_start(false) + app_uartcomm_start()
   ├─ APP_NUNCHUK:    app_nunchuk_start()
   ├─ APP_PAS:        app_pas_start(true)
   ├─ APP_ADC_PAS:    app_adc_start(false) + app_pas_start(false)
   ├─ APP_NRF:        nrf_driver_init() + rfhelp_restart()
   └─ APP_CUSTOM:     hw_stop_i2c() + app_custom_start()

7. Configure All Applications:
   ├─ app_ppm_configure()
   ├─ app_adc_configure()
   ├─ app_pas_configure()
   ├─ app_uartcomm_configure() [comm header]
   ├─ app_uartcomm_configure() [builtin]
   ├─ app_nunchuk_configure()
   ├─ app_custom_configure() [if enabled]
   └─ rfhelp_update_conf()
```

**Key Behaviors:**

1. **I2C Conflict Resolution (Lines 104, 109, 115, 142):**
   - `hw_stop_i2c()` called before UART apps
   - Prevents conflict between I2C and UART on shared pins

2. **NRF Permanence (Lines 74, 134):**
   - If permanent NRF found, driver not stopped/restarted
   - Preserves wireless connection across config changes

3. **Primary/Secondary PAS (Lines 125, 130):**
   - `app_pas_start(true)`: PAS is primary control
   - `app_pas_start(false)`: PAS assists another app (ADC)

4. **RX/TX Usage (Lines 100, 116):**
   - ADC apps can use RX/TX pins as analog inputs
   - `app_adc_start(true)`: Use RX/TX as ADC
   - `app_adc_start(false)`: Don't use RX/TX (UART needs them)

**Usage Example:**
```c
app_configuration new_conf;
memset(&new_conf, 0, sizeof(app_configuration));

// Configure for PPM RC control
new_conf.app_to_use = APP_PPM;
new_conf.app_ppm_conf.ctrl_type = PPM_CTRL_TYPE_CURRENT;
new_conf.app_ppm_conf.pulse_start = 1.0;  // 1ms
new_conf.app_ppm_conf.pulse_end = 2.0;    // 2ms
new_conf.app_ppm_conf.max_erpm = 50000.0;

app_set_configuration(&new_conf);
```

---

#### app_calc_crc() - Lines 210-220

**Purpose:** Calculate CRC16 checksum of application configuration

**Parameters:**
- `conf`: Pointer to configuration (NULL for current)

**Returns:** `unsigned short` - CRC16 value

**Algorithm:**
1. Save current CRC field
2. Set CRC field to 0
3. Calculate CRC16 over entire structure
4. Restore original CRC field
5. Return calculated CRC

**Use Case:** Configuration integrity validation (detect corruption)

**Usage:**
```c
unsigned short crc = app_calc_crc(NULL);
if (crc != appconf.crc) {
    commands_printf("Configuration CRC mismatch!");
}
```

---

### 2. Output Control Functions

#### app_disable_output() - Lines 175-190

**Purpose:** Temporarily or permanently disable motor output from applications

**Parameters:**
- `time_ms`: Disable duration
  - `0`: Enable output immediately
  - `-1`: Disable permanently (until re-enabled)
  - `>0`: Disable for specified milliseconds

**Operation:**
- Uses ChibiOS virtual timer for timed disable
- Callback automatically re-enables after timeout

**Use Cases:**
1. **Safety Interlocks:** Disable output during gear shifts
2. **Calibration:** Prevent motor movement during setup
3. **Emergency Stop:** Immediate shutdown from external signal
4. **Startup Delay:** Prevent movement until system ready

**Usage Examples:**
```c
// Disable output during calibration
app_disable_output(-1);
run_calibration_sequence();
app_disable_output(0);  // Re-enable

// Temporary disable for 500ms
app_disable_output(500);

// Emergency stop (permanent)
app_disable_output(-1);
```

---

#### app_is_output_disabled() - Lines 192-194

**Purpose:** Check if application output is currently disabled

**Parameters:** None

**Returns:** `bool` - true if output disabled

**Usage:**
```c
if (!app_is_output_disabled()) {
    mc_interface_set_current(desired_current);
} else {
    // Output disabled, don't control motor
}
```

**Note:** Each application checks this flag before sending motor commands

---

#### output_vt_cb() - Lines 196-199 (Static)

**Purpose:** Virtual timer callback to re-enable output after timeout

**Parameters:**
- `arg`: Unused callback argument

**Operation:** Sets `output_disabled_now = false`

---

## Application-Specific Functions (Declared in app.h)

These functions are implemented in separate files:

### PPM Application (`app_ppm.c`)

```c
void app_ppm_start(void);
void app_ppm_stop(void);
float app_ppm_get_decoded_level(void);       // Get normalized throttle [-1.0, 1.0]
void app_ppm_detach(bool detach);            // Temporarily disconnect
void app_ppm_override(float val);            // Force specific value
void app_ppm_configure(ppm_config *conf);
```

**Use Case:** RC car/drone control via standard PPM receiver

---

### ADC Application (`app_adc.c`)

```c
void app_adc_start(bool use_rx_tx);
void app_adc_stop(void);
void app_adc_configure(adc_config *conf);
float app_adc_get_decoded_level(void);       // ADC1 normalized value
float app_adc_get_voltage(void);             // ADC1 raw voltage
float app_adc_get_decoded_level2(void);      // ADC2 normalized value
float app_adc_get_voltage2(void);            // ADC2 raw voltage
void app_adc_detach_adc(int detach);         // Disconnect ADC control
void app_adc_adc1_override(float val);       // Override ADC1
void app_adc_adc2_override(float val);       // Override ADC2
void app_adc_detach_buttons(bool state);     // Disable button inputs
void app_adc_rev_override(bool state);       // Force reverse
void app_adc_cc_override(bool state);        // Force cruise control
bool app_adc_range_ok(void);                 // Check if ADC in valid range
```

**Use Case:** Throttle pedal, twist-grip throttle, potentiometer control

---

### UART Application (`app_uartcomm.c`)

```c
typedef enum {
    UART_PORT_COMM_HEADER = 0,    // Comm header pins
    UART_PORT_BUILTIN,            // Built-in UART
    UART_PORT_EXTRA_HEADER        // Extra header (if available)
} UART_PORT;

void app_uartcomm_initialize(void);
void app_uartcomm_start(UART_PORT port_number);
void app_uartcomm_stop(UART_PORT port_number);
void app_uartcomm_configure(uint32_t baudrate, bool permanent, UART_PORT port);
void app_uartcomm_send_packet(unsigned char *data, unsigned int len, UART_PORT port);
```

**Use Case:** Custom serial protocols, external microcontroller communication

---

### Nunchuk Application (`app_nunchuk.c`)

```c
void app_nunchuk_start(void);
void app_nunchuk_stop(void);
void app_nunchuk_configure(chuk_config *conf);
float app_nunchuk_get_decoded_x(void);       // Joystick X position
float app_nunchuk_get_decoded_y(void);       // Joystick Y position
bool app_nunchuk_get_bt_c(void);             // C button state
bool app_nunchuk_get_bt_z(void);             // Z button state
bool app_nunchuk_get_is_rev(void);           // Reverse mode active
float app_nunchuk_get_update_age(void);      // Time since last update
void app_nunchuk_update_output(chuck_data *data);  // Push new data
```

**Use Case:** Handheld joystick control (skateboard, wheelchair)

---

### PAS Application (`app_pas.c`)

```c
void app_pas_start(bool is_primary_output);
void app_pas_stop(void);
bool app_pas_is_running(void);
void app_pas_configure(pas_config *conf);
float app_pas_get_current_target_rel(void);  // Get assist level [0.0-1.0]
float app_pas_get_pedal_rpm(void);           // Get pedaling cadence
void app_pas_set_current_sub_scaling(float scale);  // Scale assist down
```

**Use Case:** E-bike pedal assist (detects pedaling, applies proportional power)

---

### Custom Application (`app_custom.c`)

```c
void app_custom_start(void);
void app_custom_stop(void);
void app_custom_configure(app_configuration *conf);
```

**Use Case:** User-defined application for custom hardware/protocols

**Compilation:** Only included if `APP_CUSTOM_TO_USE` is defined in `conf_general.h`

---

## Typical Application Lifecycle

```
System Boot
    ↓
main() calls app_set_configuration()
    ↓
┌─────────────────────────────────┐
│  Stop All Applications          │  (if app changed)
├─────────────────────────────────┤
│  • app_ppm_stop()              │
│  • app_adc_stop()              │
│  • app_uartcomm_stop()         │
│  • app_nunchuk_stop()          │
│  • app_pas_stop()              │
└─────────────────────────────────┘
    ↓
┌─────────────────────────────────┐
│  Start Selected Application     │
├─────────────────────────────────┤
│  switch(app_to_use) {           │
│    case APP_PPM:               │
│      app_ppm_start();          │
│      break;                    │
│    ...                         │
│  }                             │
└─────────────────────────────────┘
    ↓
┌─────────────────────────────────┐
│  Configure All Applications     │
├─────────────────────────────────┤
│  • app_ppm_configure()         │
│  • app_adc_configure()         │
│  • app_pas_configure()         │
│  • app_uartcomm_configure()    │
└─────────────────────────────────┘
    ↓
Application Running
    ↓
[Application reads input] → [Calls mc_interface_set_*] → [Motor Control]
    ↓
User Changes Configuration
    ↓
app_set_configuration() called again
    ↓
[Cycle repeats]
```

## Application Execution Model

Each application typically runs in one of two modes:

### 1. Periodic Polling Mode (ADC, PAS)
```c
static THD_FUNCTION(app_thread, arg) {
    chRegSetThreadName("APP_ADC");

    for (;;) {
        // Read input
        float throttle = read_adc_input();

        // Apply curve/scaling
        throttle = apply_throttle_curve(throttle);

        // Check if output enabled
        if (!app_is_output_disabled()) {
            // Send motor command
            mc_interface_set_current(throttle * max_current);
        }

        chThdSleepMilliseconds(10);  // 100 Hz update
    }
}
```

### 2. Interrupt/Event-Driven Mode (PPM, UART)
```c
// PPM: Triggered by pulse capture interrupt
void ppm_decode_callback(float pulse_width) {
    float normalized = map_pulse_to_range(pulse_width);

    if (!app_is_output_disabled()) {
        mc_interface_set_duty(normalized);
    }
}

// UART: Triggered by packet reception
void uart_process_packet(uint8_t *data, int len) {
    if (data[0] == CMD_SET_CURRENT) {
        float current = extract_float(data + 1);
        if (!app_is_output_disabled()) {
            mc_interface_set_current(current);
        }
    }
}
```

## Multi-Application Support

Some combinations run multiple apps simultaneously:

### APP_PPM_UART (Lines 108-112)
```c
case APP_PPM_UART:
    hw_stop_i2c();               // Free up I2C pins
    app_ppm_start();             // Start PPM control
    app_uartcomm_start(...);     // Start UART telemetry
    break;
```
**Use Case:** RC control with telemetry feedback

### APP_ADC_UART (Lines 114-118)
```c
case APP_ADC_UART:
    hw_stop_i2c();
    app_adc_start(false);        // ADC control (no RX/TX pins)
    app_uartcomm_start(...);     // UART data/telemetry
    break;
```
**Use Case:** Throttle control with custom protocol

### APP_ADC_PAS (Lines 128-131)
```c
case APP_ADC_PAS:
    app_adc_start(false);        // Throttle input
    app_pas_start(false);        // PAS assist (secondary)
    break;
```
**Use Case:** E-bike with both throttle and pedal assist

## Configuration Workflow

### At Compile Time:
1. Select `APP_CUSTOM_TO_USE` in `conf_general.h` (if using custom app)
2. Define hardware capabilities in `hw_*.h`

### At Runtime:
1. Load default configuration from flash (via `conf_general`)
2. User modifies configuration via VESC Tool
3. VESC Tool sends new config via commands
4. Firmware validates config CRC
5. Calls `app_set_configuration()`
6. Configuration saved to flash

### Via VESC Tool:
```
VESC Tool
    ↓
[App Settings Tab]
    ├─ Select App Type (dropdown)
    ├─ Configure App-Specific Settings
    │   ├─ PPM: pulse range, curve, ramping
    │   ├─ ADC: voltage range, buttons, curve
    │   ├─ UART: baudrate, protocol
    │   └─ PAS: magnets, RPM range, assist level
    ↓
[Write Configuration]
    ↓
COMM_SET_APPCONF command sent
    ↓
conf_general_store_app_configuration()
    ↓
app_set_configuration()
    ↓
Application Active
```

## Resource Conflicts

The application manager handles several hardware conflicts:

| Resource | Conflict | Resolution |
|----------|----------|------------|
| **UART Pins** | I2C vs UART | `hw_stop_i2c()` before UART apps |
| **RX/TX Pins** | UART vs ADC | `app_adc_start(use_rx_tx)` flag |
| **Servo Output** | PWM input vs output | Stop `servodec` if servo output enabled |
| **NRF24** | Power cycling | Don't restart if permanent module found |

## Integration with Motor Control

```c
// Application Layer (app.c, app_ppm.c, app_adc.c, etc.)
    ↓
[Read User Input]
    ↓
[Apply Curves/Scaling/Ramping]
    ↓
[Check app_is_output_disabled()]
    ↓
// Motor Control Interface (mc_interface.c)
mc_interface_set_duty()
mc_interface_set_current()
mc_interface_set_pid_speed()
mc_interface_set_brake_current()
    ↓
// Low-Level Motor Control (mcpwm_foc.c, mcpwm.c)
[FOC/BLDC Implementation]
    ↓
[Hardware PWM]
```

## Example: Switching from PPM to ADC

```c
// Initial state: PPM application running
const app_configuration *conf = app_get_configuration();
// conf->app_to_use == APP_PPM

// User wants to switch to ADC throttle
app_configuration new_conf = *conf;  // Copy existing config
new_conf.app_to_use = APP_ADC;

// Configure ADC settings
new_conf.app_adc_conf.ctrl_type = ADC_CTRL_TYPE_CURRENT_REV_BUTTON;
new_conf.app_adc_conf.voltage_start = 0.8;   // 0.8V = 0% throttle
new_conf.app_adc_conf.voltage_end = 3.0;     // 3.0V = 100% throttle
new_conf.app_adc_conf.update_rate_hz = 500;

// Apply configuration
app_set_configuration(&new_conf);

// Result:
// 1. app_ppm_stop() called
// 2. app_adc_start(true) called
// 3. app_adc_configure() called with new settings
// 4. ADC application now active and controlling motor
```

## Safety Considerations

1. **Output Disable:** Always check `app_is_output_disabled()` before motor control
2. **Timeout Protection:** Applications implement watchdog timers (timeout → safe state)
3. **Safe Start:** Many apps support safe start mode (requires zero throttle before enabling)
4. **Fault Handling:** Motor faults automatically disable application output
5. **Configuration Validation:** CRC check ensures valid configuration

## Debugging Applications

### Check Active Application:
```c
const app_configuration *conf = app_get_configuration();
commands_printf("Active app: %d", conf->app_to_use);
```

### Get Application Input:
```c
// For PPM
float ppm_level = app_ppm_get_decoded_level();

// For ADC
float adc_voltage = app_adc_get_voltage();
float adc_level = app_adc_get_decoded_level();

// For PAS
float pedal_rpm = app_pas_get_pedal_rpm();
```

### Check Output State:
```c
bool disabled = app_is_output_disabled();
commands_printf("Output %s", disabled ? "DISABLED" : "ENABLED");
```

## Call Hierarchy

```
main()
  └─ app_set_configuration()
      ├─ [Stop all applications]
      │   ├─ app_ppm_stop()
      │   ├─ app_adc_stop()
      │   ├─ app_uartcomm_stop()
      │   ├─ app_nunchuk_stop()
      │   ├─ app_pas_stop()
      │   └─ app_custom_stop()
      │
      ├─ nrf_driver_stop() / nrf_driver_init()
      ├─ comm_can_set_baud()
      ├─ imu_init()
      ├─ servodec_stop() / pwm_servo_init_servo()
      │
      ├─ [Start selected application]
      │   └─ app_xxx_start()
      │
      └─ [Configure all applications]
          ├─ app_ppm_configure()
          ├─ app_adc_configure()
          ├─ app_pas_configure()
          ├─ app_uartcomm_configure()
          ├─ app_nunchuk_configure()
          ├─ app_custom_configure()
          └─ rfhelp_update_conf()

[Application Runtime]
  └─ app_xxx_thread() / app_xxx_callback()
      ├─ [Read input]
      ├─ [Process input]
      ├─ app_is_output_disabled() check
      └─ mc_interface_set_*()
```

## Related Files

**Application Implementations:**
- `applications/app_ppm.c` - PPM RC receiver
- `applications/app_adc.c` - Analog input
- `applications/app_uartcomm.c` - UART communication
- `applications/app_nunchuk.c` - Wii Nunchuk
- `applications/app_pas.c` - Pedal assist sensor
- `applications/app_custom.c` - User custom app

**Supporting Modules:**
- `motor/mc_interface.c` - Motor control interface
- `conf_general.c` - Configuration storage
- `nrf_driver.c` - NRF24L01+ wireless driver
- `imu/imu.c` - IMU sensor integration
- `servo/pwm_servo.c` - Servo output control
- `servo/servo_dec.c` - Servo input decoding

---

**Next Steps in Call Hierarchy:**

From `app_set_configuration()`, the key subsystems initialized are:

1. **Individual Application Files** (app_ppm.c, app_adc.c, etc.) - Input processing
2. **nrf_driver.c** - Wireless communication
3. **imu.c** - Inertial measurement unit
4. **comm_can.c** - CAN bus communication

For understanding motor control flow, document the individual application files next. The most commonly used applications are:
- **app_ppm.c** - RC control
- **app_adc.c** - Analog throttle
- **app_pas.c** - E-bike assist
