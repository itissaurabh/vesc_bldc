# Adding Custom User Inputs to VESC Firmware

## Table of Contents
1. [Overview](#overview)
2. [Architecture for Custom Inputs](#architecture-for-custom-inputs)
3. [Input Types and Methods](#input-types-and-methods)
4. [Integration Strategies](#integration-strategies)
5. [Specific Input Examples](#specific-input-examples)
   - [Brake Input](#brake-input)
   - [Drive/Reverse Selector](#drivereversepark-selector)
   - [Ignition/Key Switch](#ignitionkey-switch)
   - [Eco/Sport Mode Selector](#ecosport-mode-selector)
   - [Dual Accelerator Inputs](#dual-accelerator-inputs)
6. [Implementation Approaches](#implementation-approaches)
7. [Testing and Validation](#testing-and-validation)

---

## Overview

VESC firmware is designed with modularity in mind, making it relatively straightforward to add custom inputs. There are three primary approaches:

### **Approach 1: Extend Existing Application (Recommended for Most Cases)**
- Modify existing app (ADC, PPM, UART) to add input reading
- Best for: Simple additions like mode switches, brake inputs
- **Pros:** Minimal code changes, uses existing infrastructure
- **Cons:** Limited to input types supported by that app

### **Approach 2: Create New Custom Application**
- Create new app in `applications/` directory
- Best for: Complex custom control schemes
- **Pros:** Clean separation, no impact on existing code
- **Cons:** More code to write and maintain

### **Approach 3: LispBM Scripting (Best for Prototyping)**
- Write Lisp script for custom logic
- Best for: Rapid prototyping, user-specific customizations
- **Pros:** No firmware recompilation needed, easy to modify
- **Cons:** Performance overhead, limited to exposed APIs

---

## Architecture for Custom Inputs

### System Architecture Diagram

```
┌────────────────────────────────────────────────────────────────┐
│                    PHYSICAL INPUTS                              │
│  GPIO Pins │ ADC Channels │ I2C Devices │ CAN Messages │ UART  │
└─────┬──────────────┬───────────────┬────────────┬──────────┬───┘
      │              │               │            │          │
      ▼              ▼               ▼            ▼          ▼
┌────────────────────────────────────────────────────────────────┐
│                    INPUT READING LAYER                          │
│  palReadPad() │ ADC_VOLTS() │ i2c_read() │ CAN RX │ UART RX   │
└─────┬──────────────┬───────────────┬────────────┬──────────┬───┘
      │              │               │            │          │
      └──────────────┴───────────────┴────────────┴──────────┘
                              │
                              ▼
┌────────────────────────────────────────────────────────────────┐
│                    INPUT PROCESSING                             │
│  • Debouncing (GPIO)                                           │
│  • Filtering (ADC)                                             │
│  • Validation                                                  │
│  • Mapping/Scaling                                             │
└─────┬──────────────────────────────────────────────────────────┘
      │
      ▼
┌────────────────────────────────────────────────────────────────┐
│                    CONTROL LOGIC                                │
│  • Mode Selection                                              │
│  • Safety Checks                                               │
│  • Arbitration (multiple inputs)                               │
│  • Command Generation                                          │
└─────┬──────────────────────────────────────────────────────────┘
      │
      ▼
┌────────────────────────────────────────────────────────────────┐
│                    MOTOR CONTROL INTERFACE                      │
│  mc_interface_set_current()                                    │
│  mc_interface_set_brake_current()                              │
│  mc_interface_set_duty()                                       │
│  mc_interface_set_pid_speed()                                  │
└────────────────────────────────────────────────────────────────┘
```

### Key Files and Their Roles

| File/Directory | Purpose | When to Modify |
|----------------|---------|----------------|
| `applications/app_adc.c` | ADC-based analog inputs | Adding analog sensors |
| `applications/app_ppm.c` | Digital pulse inputs | Adding pulse-based inputs |
| `hwconf/hw_*.c` | Hardware-specific pin definitions | Defining new GPIO pins |
| `conf_general.h` | Configuration structures | Adding new config parameters |
| `confgenerator.c` | Configuration UI generator | Adding UI for new parameters |
| `mc_interface.c` | Motor control interface | Rarely - only for new control modes |
| `lispBM/lispif.c` | LispBM integration | Adding new Lisp functions |

---

## Input Types and Methods

### 1. Digital GPIO Inputs (Switches, Buttons)

**Hardware Interface:**
- Direct connection to STM32 GPIO pins
- Internal pull-up/pull-down resistors available
- 3.3V logic level (5V tolerant on some pins)

**Reading Method:**
```c
// Read GPIO pin state
bool button_pressed = palReadPad(GPIOB, 5);  // Read PB5

// With debouncing
static uint8_t debounce_counter = 0;
bool stable_state = false;

if (palReadPad(GPIOB, 5)) {
    if (debounce_counter < 10) debounce_counter++;
} else {
    if (debounce_counter > 0) debounce_counter--;
}

if (debounce_counter >= 8) stable_state = true;
```

**Best For:**
- On/off switches (ignition, kill switch)
- Mode selectors (eco/sport)
- Direction selectors (forward/reverse/neutral)
- Discrete buttons

**Existing Examples:**
- `timeout.c:201-219` - Kill switch reading
- `app_adc.c:287-315` - Button reading (cruise, reverse)
- `app_ppm.c` - Servo decoder (uses timer input capture)

---

### 2. Analog ADC Inputs

**Hardware Interface:**
- 12-bit resolution (0-4095 for 0-3.3V)
- Multiple channels available (ADC1, ADC2, ADC3)
- Can use external pins or internal sensors

**Reading Method:**
```c
// Read ADC voltage
float voltage = ADC_VOLTS(ADC_IND_EXT);  // External ADC channel

// With filtering
static float filtered_value = 0.0;
UTILS_LP_MOVING_AVG_APPROX(filtered_value, voltage, 5);  // 5-sample average

// Map to useful range
float normalized = utils_map(filtered_value, 0.5, 2.5, 0.0, 1.0);
utils_truncate_number(&normalized, 0.0, 1.0);
```

**Best For:**
- Throttle pedals/potentiometers
- Analog brake sensors
- Pressure sensors
- Continuous position sensors

**Existing Examples:**
- `app_adc.c:189-247` - Primary throttle ADC reading
- `app_adc.c:249-284` - Secondary brake ADC reading
- `mc_interface.c` - Voltage/current sensing

---

### 3. I2C/SPI Devices

**Hardware Interface:**
- I2C: 2-wire serial (SDA, SCL)
- SPI: 4-wire serial (MOSI, MISO, SCK, CS)
- Supports multiple devices on same bus

**Reading Method:**
```c
// I2C example (from IMU code)
uint8_t txbuf[10];
uint8_t rxbuf[20];
txbuf[0] = register_address;

i2cAcquireBus(&HW_I2C_DEV);
i2c_status = i2cMasterTransmitTimeout(&HW_I2C_DEV, device_addr,
                                       txbuf, 1, rxbuf, data_len,
                                       MS2ST(10));
i2cReleaseBus(&HW_I2C_DEV);

// SPI example (from encoder code)
uint16_t result = spi_exchange_16(SPI_DEV, command_word);
```

**Best For:**
- Complex sensors (accelerometers, gyros)
- External ADC chips
- Digital encoders
- Display modules

**Existing Examples:**
- `imu/imu.c` - I2C accelerometer/gyro
- `encoder/encoder.c` - SPI absolute encoders
- `app_nunchuk.c` - I2C Wii controller

---

### 4. CAN Bus Messages

**Hardware Interface:**
- CAN transceiver required
- 29-bit extended ID support
- Up to 8 bytes per message

**Reading Method:**
```c
// Receive custom CAN message (from comm_can.c pattern)
void custom_can_rx_handler(CANRxFrame *frame) {
    uint32_t id = frame->EID;  // Extended ID
    uint8_t *data = frame->data8;

    // Parse custom message
    uint8_t input_id = data[0];
    uint16_t value = (data[1] << 8) | data[2];

    // Process input
    // ...
}
```

**Best For:**
- Remote inputs (dashboard, remote control)
- Multi-ESC coordination
- Integration with vehicle CAN bus
- Wireless input devices (via CAN bridge)

**Existing Examples:**
- `comm/comm_can.c:500-800` - CAN message handling
- `comm/comm_can.c:200-400` - Custom CAN app messages

---

### 5. UART/Serial Messages

**Hardware Interface:**
- Standard UART (TX, RX)
- Configurable baud rate
- Can use USB virtual serial

**Reading Method:**
```c
// Receive byte stream (similar to comm_uart pattern)
void uart_rx_callback(uint8_t *data, unsigned int len) {
    // Parse protocol
    if (data[0] == START_BYTE) {
        uint8_t command = data[1];
        // Process command
    }
}
```

**Best For:**
- External controllers
- Bluetooth modules
- GPS receivers
- Custom serial protocols

**Existing Examples:**
- `comm/comm_uart.c` - UART communication
- `comm/comm_usb.c` - USB serial (same protocol)

---

## Integration Strategies

### Strategy 1: Extend ADC App (Recommended for Analog Inputs)

**Use When:**
- Adding analog sensors (throttle, brake, position)
- Need filtering and mapping
- Want to combine with existing ADC throttle

**Files to Modify:**
1. `applications/app_adc.c` - Main application logic
2. `confgenerator.c` - Configuration UI
3. `conf_general.h` - Configuration structure
4. `hwconf/hw_*.h` - Pin definitions

**Integration Points:**

```
app_adc.c Flow:
  ├─ adc_thread() (line 162)
  │  └─ Main loop running at configurable rate
  │
  ├─ Read existing ADC inputs (line 189-284)
  │  ├─ ADC1 (throttle)
  │  └─ ADC2 (brake)
  │
  ├─ **INSERT: Read new ADC inputs here**
  │  ├─ Use ADC_VOLTS(ADC_IND_CUSTOM)
  │  ├─ Apply filtering
  │  └─ Map to useful range
  │
  ├─ Read button inputs (line 287-327)
  │  ├─ Cruise control button
  │  └─ Reverse button
  │
  ├─ **INSERT: Read new button inputs here**
  │  └─ Use palReadPad(GPIO_PORT, GPIO_PIN)
  │
  ├─ Mode-specific processing (line 343-480)
  │  └─ switch (config.ctrl_type)
  │
  ├─ **INSERT: Apply mode-dependent logic**
  │  ├─ Eco/sport mode adjustments
  │  └─ Drive/reverse selection
  │
  ├─ Apply curves and ramping (line 374-389)
  │
  ├─ Safe start logic (line 482-502)
  │
  ├─ **INSERT: Additional safety checks**
  │
  └─ Send motor command (line 574-639)
     └─ mc_interface_set_current_rel()
```

**Example: Adding Brake Pedal**

The ADC app already has brake support via ADC2. To add independent brake:

1. **Read brake ADC** (add after line 284):
   - `brake_pedal = ADC_VOLTS(ADC_IND_EXT3)`
   - Apply filtering
   - Map 0-3.3V to 0.0-1.0

2. **Process brake signal** (add in control logic):
   - If brake_pedal > threshold:
     - Set brake_active flag
     - Calculate brake current
   - Else: brake_active = false

3. **Apply brake command** (modify line 575):
   - If brake_active AND throttle < threshold:
     - `mc_interface_set_brake_current(brake_current)`
   - Else: normal throttle control

4. **Add configuration**:
   - Brake ADC pin selection
   - Brake threshold voltage
   - Brake current scaling
   - Combined brake mode (throttle + brake vs brake priority)

---

### Strategy 2: Extend PPM App (For Digital Pulse Inputs)

**Use When:**
- Adding RC-style inputs
- Using frequency/PWM signals
- Integrating with automotive CAN (via pulse converter)

**Files to Modify:**
1. `applications/app_ppm.c`
2. `servo_dec.c` - Servo decoder (if adding channels)

**Integration Points:**

```
app_ppm.c Flow:
  ├─ ppm_thread() (line 107)
  │  └─ Event-driven by PPM pulses
  │
  ├─ Initialize servo decoder (line 111-117)
  │  └─ servodec_init()
  │
  ├─ **INSERT: Initialize additional input decoders**
  │  └─ Could use separate timer channels
  │
  ├─ Wait for PPM pulse (line 123-127)
  │
  ├─ Read PPM servo value (line 132-163)
  │
  ├─ **INSERT: Read additional pulse inputs**
  │  └─ servodec_get_servo(channel)
  │
  ├─ **INSERT: Process mode switches**
  │  ├─ Decode PWM duty cycle for mode
  │  └─ e.g., 25% = eco, 50% = normal, 75% = sport
  │
  ├─ Apply curves and ramping (line 189-212)
  │
  └─ Send motor command (line 311-561)
```

**Example: Adding Drive/Reverse Selector**

PPM app already has reverse support in multiple modes. To add explicit selector:

1. **Read selector state**:
   - Option A: Use second PPM channel
   - Option B: Use GPIO switch
   - Option C: Decode from single PPM (different pulse widths)

2. **Process selector** (add after line 163):
   - Forward: 1000-1400µs or GPIO high
   - Neutral: 1400-1600µs or not connected
   - Reverse: 1600-2000µs or GPIO low

3. **Apply direction logic** (modify control type processing):
   - If selector == REVERSE:
     - Invert throttle command
     - Apply reverse speed limit
   - If selector == NEUTRAL:
     - Force brake or zero current
   - If selector == FORWARD:
     - Normal operation

4. **Add safety interlocks** (add before line 385):
   - Must be in neutral to change direction
   - Verify vehicle stopped (RPM < threshold)
   - Apply neutral brake during transition

---

### Strategy 3: Create Custom Application

**Use When:**
- Need completely custom control scheme
- Multiple complex inputs
- Want clean separation from existing code

**Steps to Create:**

1. **Create new application file:**
   - Copy `applications/app_template.c` (or create from scratch)
   - Name it `app_custom.c`

2. **Define application structure:**
```c
// app_custom.c structure

// Configuration structure
typedef struct {
    // Input configurations
    float brake_adc_min;
    float brake_adc_max;
    uint8_t mode_switch_pin;
    bool ignition_required;

    // Control parameters
    float eco_current_limit;
    float sport_current_limit;

    // Safety settings
    bool require_neutral_start;
    uint16_t brake_timeout_ms;
} custom_config;

// Thread working area
static THD_WORKING_AREA(custom_thread_wa, 1024);
static THD_FUNCTION(custom_thread, arg);

// Public API functions
void app_custom_configure(custom_config *conf);
void app_custom_start(void);
void app_custom_stop(void);

// Private functions
static void read_inputs(void);
static void process_mode_selection(void);
static void apply_safety_logic(void);
static void send_motor_command(void);
```

3. **Implement thread function:**
```c
static THD_FUNCTION(custom_thread, arg) {
    (void)arg;
    chRegSetThreadName("APP_CUSTOM");

    // Initialization
    // Configure GPIO, ADC, etc.

    for(;;) {
        // Input reading
        read_inputs();

        // Mode processing
        process_mode_selection();

        // Safety checks
        if (!apply_safety_logic()) {
            // Apply brake or zero command
            mc_interface_set_brake_current(0.0);
            chThdSleepMilliseconds(10);
            continue;
        }

        // Generate motor command
        send_motor_command();

        // Loop timing
        chThdSleepMilliseconds(10);  // 100 Hz
    }
}
```

4. **Register application:**
   - Add to `app.c:app_set_configuration()`
   - Add configuration to `conf_general.h`
   - Add UI to `confgenerator.c`

**Example File Structure:**
```
applications/
├── app_custom.c          # Main implementation
├── app_custom.h          # Public API
├── app_custom_conf.h     # Configuration structure
└── Makefile              # Add to build system
```

---

### Strategy 4: LispBM Scripting (Rapid Prototyping)

**Use When:**
- Prototyping ideas quickly
- User-specific customizations
- Don't want to rebuild firmware

**Available LispBM Functions:**

```lisp
; GPIO access
(gpio-configure pin mode)    ; Configure GPIO pin
(gpio-write pin state)       ; Write GPIO state
(gpio-read pin)              ; Read GPIO state

; ADC access
(raw-adc-voltage ch scale)   ; Read ADC channel
(adc-ext)                    ; Read external ADC (same as app_adc uses)

; CAN access
(canset-current id current)  ; Send current command to CAN ID
(canget-current id)          ; Get current from CAN node

; Motor control
(set-current amps)           ; Set motor current
(set-brake brake-amps)       ; Set brake current
(set-duty duty)              ; Set duty cycle
(get-rpm)                    ; Get current RPM
(get-speed)                  ; Get speed in m/s

; Configuration
(conf-get 'param)            ; Get configuration parameter
(conf-set 'param value)      ; Set configuration parameter

; Events
(event-register-handler event-id handler-function)
(event-enable event-id)

; Timing
(systime)                    ; Get system time
(sleep seconds)              ; Sleep for seconds
```

**Example: Eco/Sport Mode with GPIO Switch**

```lisp
; Configuration
(def eco-current-limit 20.0)
(def sport-current-limit 60.0)
(def mode-switch-pin 12)  ; GPIO pin number

; Configure GPIO as input with pull-up
(gpio-configure mode-switch-pin 'pin-mode-in-pu)

; State variables
(def current-mode 'eco)
(def throttle-input 0.0)

; Main control loop
(defun control-loop ()
    (loopwhile t
        ; Read mode switch
        (if (= (gpio-read mode-switch-pin) 1)
            (setq current-mode 'sport)
            (setq current-mode 'eco)
        )

        ; Read throttle (from ADC or override existing app)
        (setq throttle-input (adc-ext))

        ; Apply mode-specific current limit
        (def max-current
            (if (eq current-mode 'sport)
                sport-current-limit
                eco-current-limit
            )
        )

        ; Calculate and send motor command
        (def motor-current (* throttle-input max-current))
        (set-current motor-current)

        ; Loop at 100 Hz
        (sleep 0.01)
    )
)

; Start control loop
(control-loop)
```

**Advantages:**
- No firmware recompilation
- Easy to modify and test
- Can run alongside existing apps
- User can customize without programming knowledge

**Limitations:**
- ~10x slower than C code
- Limited to exposed API functions
- No access to all hardware features
- Harder to debug

---

## Specific Input Examples

### Brake Input

**Requirements:**
- Independent brake pedal/lever
- Priority over throttle
- Configurable brake force
- Safety: brake always works even in fault condition

**Implementation Options:**

#### Option 1: Add to ADC App (Recommended)

**Hardware:**
- Connect brake sensor to ADC_IND_EXT2 or ADC_IND_EXT3
- 0-3.3V output from sensor
- Ratiometric or absolute output

**Modification Points:**

1. **Read brake ADC** (`app_adc.c:~250`):
```c
// Read brake pedal position
float brake_pedal = ADC_VOLTS(ADC_IND_EXT3);

// Filter (reduce noise from long wires)
static float brake_filtered = 0.0;
UTILS_LP_MOVING_AVG_APPROX(brake_filtered, brake_pedal, 5);

// Map voltage to brake force (0.0 = no brake, 1.0 = full brake)
float brake_normalized = utils_map(brake_filtered,
                                    config.brake_voltage_min,
                                    config.brake_voltage_max,
                                    0.0, 1.0);
utils_truncate_number(&brake_normalized, 0.0, 1.0);

// Apply deadband to prevent brake drag
utils_deadband(&brake_normalized, 0.05, 1.0);
```

2. **Process brake with priority** (`app_adc.c:~480`):
```c
// Check brake activation
bool brake_active = (brake_normalized > config.brake_threshold);

if (brake_active) {
    // Brake has priority - override throttle
    float brake_current = brake_normalized * config.brake_max_current;

    // Apply brake
    mc_interface_set_brake_current(brake_current);

    // Multi-ESC: brake all motors
    if (config.multi_esc) {
        for (int i = 0; i < CAN_STATUS_MSGS_TO_STORE; i++) {
            can_status_msg *msg = comm_can_get_status_msg_index(i);
            if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
                comm_can_set_current_brake(msg->id, brake_current);
            }
        }
    }

    // Skip throttle processing
    continue;
}

// No brake - process throttle normally
// ... existing throttle code ...
```

3. **Add configuration parameters** (`conf_general.h`):
```c
typedef struct {
    // Existing ADC config...

    // Brake configuration
    float brake_voltage_min;      // Voltage at no brake
    float brake_voltage_max;      // Voltage at full brake
    float brake_threshold;        // Activation threshold (0.0-1.0)
    float brake_max_current;      // Maximum brake current (A)
    bool brake_has_priority;      // Brake overrides throttle
    float brake_regen_limit;      // Limit regenerative braking
} adc_config;
```

**Safety Considerations:**
- Brake should work even if throttle sensor fails
- Use separate ADC channel from throttle
- Consider redundant brake sensor
- Implement brake light output (GPIO)
- Add brake timeout (apply brake if signal lost)

**Flowchart:**

```
┌─────────────────────┐
│  Read Brake ADC     │
│  + Throttle ADC     │
└──────────┬──────────┘
           │
           ▼
    ┌──────────────┐
    │ Filter Both  │
    │ Map to 0-1   │
    └──────┬───────┘
           │
           ▼
    ┌─────────────────────┐
    │ Brake > Threshold?  │
    └──────┬─────────┬────┘
           │ YES     │ NO
           ▼         ▼
  ┌────────────┐  ┌──────────────┐
  │ Apply      │  │ Process      │
  │ Brake      │  │ Throttle     │
  │ Command    │  │ Command      │
  └────────────┘  └──────────────┘
           │         │
           └────┬────┘
                ▼
       ┌────────────────┐
       │ Send to        │
       │ Motor Control  │
       └────────────────┘
```

---

### Drive/Reverse/Park Selector

**Requirements:**
- Three-position selector switch
- Safety: neutral required to change direction
- Must verify vehicle stopped before direction change
- Optional park brake when in park

**Implementation Options:**

#### Option 1: GPIO-Based Selector

**Hardware:**
- Rotary switch or toggle switches
- Two GPIO pins for three states:
  - Both LOW: Park
  - Pin1 HIGH, Pin2 LOW: Drive (forward)
  - Pin1 LOW, Pin2 HIGH: Reverse
  - Both HIGH: Invalid (treat as park)

**Modification Points:**

1. **Add GPIO reading** (add to any app thread):
```c
// Read selector switch
bool pin1 = palReadPad(GPIOB, 6);  // Drive pin
bool pin2 = palReadPad(GPIOB, 7);  // Reverse pin

// Debounce
static uint8_t debounce_drive = 0;
static uint8_t debounce_reverse = 0;

if (pin1) {
    if (debounce_drive < 10) debounce_drive++;
} else {
    if (debounce_drive > 0) debounce_drive--;
}

if (pin2) {
    if (debounce_reverse < 10) debounce_reverse++;
} else {
    if (debounce_reverse > 0) debounce_reverse--;
}

// Determine mode
enum gear_mode {
    GEAR_PARK,
    GEAR_DRIVE,
    GEAR_REVERSE,
    GEAR_INVALID
};

enum gear_mode selected_gear;
if (debounce_drive >= 8 && debounce_reverse < 2) {
    selected_gear = GEAR_DRIVE;
} else if (debounce_drive < 2 && debounce_reverse >= 8) {
    selected_gear = GEAR_REVERSE;
} else if (debounce_drive < 2 && debounce_reverse < 2) {
    selected_gear = GEAR_PARK;
} else {
    selected_gear = GEAR_INVALID;  // Both high - invalid
}
```

2. **Implement gear change safety logic**:
```c
// State machine for gear changes
static enum gear_mode current_gear = GEAR_PARK;
static enum gear_mode last_selected = GEAR_PARK;
static uint32_t gear_change_request_time = 0;

// Check if gear change is requested
if (selected_gear != current_gear) {
    // New gear requested
    if (last_selected != selected_gear) {
        // First time seeing this request
        last_selected = selected_gear;
        gear_change_request_time = chVTGetSystemTimeX();
    }

    // Verify conditions for gear change
    bool change_allowed = false;

    // Must hold switch for 100ms (debounce + intentional)
    if (UTILS_AGE_S(gear_change_request_time) >= 0.1) {
        // Check safety conditions
        float rpm = mc_interface_get_rpm();
        float speed_mps = fabsf(rpm * wheel_circumference / 60.0);

        // Must be stopped or very slow
        if (speed_mps < 0.5) {  // Less than 0.5 m/s
            change_allowed = true;
        }
    }

    // Apply gear change
    if (change_allowed) {
        current_gear = selected_gear;

        // Apply park brake if entering park
        if (current_gear == GEAR_PARK) {
            mc_interface_set_brake_current(config.park_brake_current);
        }

        // Log event
        // events_add_gear_change(current_gear);
    }
} else {
    // No change requested - reset timer
    last_selected = current_gear;
    gear_change_request_time = chVTGetSystemTimeX();
}
```

3. **Apply gear logic to motor command**:
```c
// Calculate base motor command from throttle
float throttle_command = /* ... from throttle processing ... */;
float motor_command = 0.0;

switch (current_gear) {
    case GEAR_PARK:
        // Force park brake, ignore throttle
        mc_interface_set_brake_current(config.park_brake_current);
        break;

    case GEAR_DRIVE:
        // Forward only
        if (throttle_command > 0.0) {
            motor_command = throttle_command * config.drive_current_limit;
            mc_interface_set_current(motor_command);
        } else {
            // Allow regen braking when releasing throttle
            motor_command = throttle_command * config.regen_brake_current;
            mc_interface_set_brake_current(fabsf(motor_command));
        }
        break;

    case GEAR_REVERSE:
        // Reverse only, limited speed
        if (throttle_command > 0.0) {
            motor_command = -throttle_command * config.reverse_current_limit;

            // Speed limiter for reverse
            float rpm = mc_interface_get_rpm();
            if (rpm < -config.reverse_max_erpm) {
                // Switch to brake if over speed
                motor_command = 0.0;
                mc_interface_set_brake_current(5.0);
            } else {
                mc_interface_set_current(motor_command);
            }
        } else {
            // Brake when releasing throttle in reverse
            motor_command = throttle_command * config.regen_brake_current;
            mc_interface_set_brake_current(fabsf(motor_command));
        }
        break;

    case GEAR_INVALID:
        // Safety: treat as park
        mc_interface_set_brake_current(config.park_brake_current);
        break;
}
```

**Configuration Parameters:**
```c
// Add to adc_config or create new custom_config
float park_brake_current;        // Current to hold vehicle (A)
float drive_current_limit;       // Max current in drive (A)
float reverse_current_limit;     // Max current in reverse (A)
float reverse_max_erpm;          // Max reverse speed (ERPM)
float regen_brake_current;       // Regen brake on throttle release (A)
bool require_stop_for_change;    // Must be stopped to change gear
float gear_change_speed_thresh;  // Speed threshold for gear change (m/s)
```

**Flowchart:**

```
┌──────────────────┐
│ Read GPIO Pins   │
│ (Drive, Reverse) │
└────────┬─────────┘
         │
         ▼
┌────────────────────┐
│ Debounce (10 ms)   │
│ Determine Gear     │
└────────┬───────────┘
         │
         ▼
    ┌────────────────────┐
    │ Gear Change        │
    │ Requested?         │
    └────┬───────────┬───┘
         │ YES       │ NO
         ▼           ▼
 ┌───────────────┐  ┌───────────────┐
 │ Check Safety: │  │ Use Current   │
 │ - Speed < 0.5 │  │ Gear          │
 │ - Held 100ms  │  └───────┬───────┘
 └───────┬───────┘          │
         │                  │
         ▼                  │
    ┌──────────┐            │
    │ Change   │            │
    │ Gear     │            │
    └─────┬────┘            │
          │                 │
          └────────┬────────┘
                   ▼
           ┌──────────────┐
           │ Apply Gear:  │
           │ - Park: Brake│
           │ - Drive: Fwd │
           │ - Rev: -Fwd  │
           └──────────────┘
```

---

### Ignition/Key Switch

**Requirements:**
- Must be ON to enable motor
- Turn OFF to safely shut down
- Optional: different positions (OFF, ACC, IGN, START)
- Remember power-on state across power cycles

**Implementation Options:**

#### Option 1: Simple ON/OFF via GPIO

**Hardware:**
- Connect key switch to GPIO input
- ON = 3.3V (or 0V with pull-up), OFF = 0V (or 3.3V)
- Consider using kill switch functionality

**Modification Points:**

1. **Add ignition reading** (to timeout.c or app):
```c
// Read ignition switch
bool ignition_on = palReadPad(GPIOB, 8);

// Debounce
static uint8_t ignition_debounce = 0;
static bool ignition_stable = false;

if (ignition_on) {
    if (ignition_debounce < 50) ignition_debounce++;  // 500ms debounce
} else {
    if (ignition_debounce > 0) ignition_debounce--;
}

ignition_stable = (ignition_debounce >= 45);
```

2. **Integrate with timeout/safety** (`timeout.c`):
```c
// In timeout_thread, after kill switch check (line ~220)

// Check ignition
bool ignition_active = check_ignition();  // Your function

if (!ignition_active) {
    // Ignition OFF - similar to kill switch
    mc_interface_unlock();
    mc_interface_select_motor_thread(1);
    mc_interface_set_brake_current(config.ignition_off_brake_current);

    #ifdef HW_HAS_DUAL_MOTORS
    mc_interface_select_motor_thread(2);
    mc_interface_set_brake_current(config.ignition_off_brake_current);
    #endif

    // Prevent all applications from commanding motor
    mc_interface_ignore_input_both(100);  // 1 second

    // Set status flag
    ignition_off_flag = true;
}
```

3. **Application-level ignition check**:
```c
// In app_adc.c or app_ppm.c main loop

// Check ignition before processing inputs
if (!ignition_is_on()) {
    // Don't process throttle/commands
    continue;
}

// Normal operation...
```

**Configuration:**
```c
bool ignition_enable;               // Enable ignition checking
uint8_t ignition_gpio_port;         // GPIO port
uint8_t ignition_gpio_pin;          // GPIO pin
bool ignition_active_high;          // true = ON when high
float ignition_off_brake_current;   // Brake when OFF (A)
```

#### Option 2: Multi-Position Ignition (OFF/ACC/ON/START)

**Hardware:**
- Use 2 GPIO pins to read 4 positions:
  - 00: OFF
  - 01: ACC (accessories)
  - 10: IGN (ignition/run)
  - 11: START (momentary)

**State Machine:**
```
┌──────────┐
│   OFF    │──┐
└──────────┘  │ Turn key to ACC
              ▼
         ┌──────────┐
         │   ACC    │──┐
         └──────────┘  │ Turn key to IGN
                       ▼
                  ┌──────────┐
                  │   IGN    │◄─┐
                  └──────────┘  │ Release key
                       │        │
                       │ Turn to START
                       ▼        │
                  ┌──────────┐  │
                  │  START   │──┘
                  └──────────┘
```

**Implementation:**
```c
enum ignition_position {
    IGN_OFF,
    IGN_ACC,
    IGN_ON,
    IGN_START
};

static enum ignition_position get_ignition_position(void) {
    bool pin1 = palReadPad(GPIOB, 8);
    bool pin2 = palReadPad(GPIOB, 9);

    // Decode position
    if (!pin1 && !pin2) return IGN_OFF;
    if (!pin1 && pin2)  return IGN_ACC;
    if (pin1 && !pin2)  return IGN_ON;
    if (pin1 && pin2)   return IGN_START;

    return IGN_OFF;  // Shouldn't reach
}

// State machine
static enum ignition_position current_position = IGN_OFF;
static uint32_t start_position_time = 0;

void process_ignition(void) {
    enum ignition_position new_position = get_ignition_position();

    // Handle state transitions
    if (new_position != current_position) {
        switch (new_position) {
            case IGN_OFF:
                // Shutdown sequence
                mc_interface_set_brake_current(10.0);
                // Save statistics
                // Turn off accessories
                break;

            case IGN_ACC:
                // Enable accessories only
                // USB power, display, lights
                // Motor still disabled
                break;

            case IGN_ON:
                // Enable motor control
                // Full system operational
                timeout_reset();
                break;

            case IGN_START:
                // Start position - usually momentary
                start_position_time = chVTGetSystemTimeX();
                // Could trigger precharge, etc.
                break;
        }

        current_position = new_position;
    }

    // Handle START position timeout (auto-return to ON)
    if (current_position == IGN_START) {
        if (UTILS_AGE_S(start_position_time) > 2.0) {
            // After 2 seconds in START, should have returned to ON
            // If still in START, something wrong - go to OFF for safety
            current_position = IGN_OFF;
        }
    }
}
```

---

### Eco/Sport Mode Selector

**Requirements:**
- Switch between different performance profiles
- Adjusts current limits, acceleration, throttle curve
- Could be:
  - GPIO switch
  - CAN message
  - Button press
  - PWM signal level

**Implementation Options:**

#### Option 1: GPIO Toggle Switch

**Hardware:**
- Simple SPDT switch on GPIO
- Could be multi-position rotary for 3+ modes

**Modification Points:**

1. **Add mode detection** (add to app):
```c
// Read mode switch
bool sport_mode = palReadPad(GPIOB, 10);

// Debounce
static uint8_t mode_debounce = 0;
if (sport_mode) {
    if (mode_debounce < 10) mode_debounce++;
} else {
    if (mode_debounce > 0) mode_debounce--;
}

bool mode_is_sport = (mode_debounce >= 8);
```

2. **Apply mode-specific limits** (modify command generation):
```c
// Define mode profiles
struct mode_profile {
    float current_limit;
    float acceleration_rate;
    float throttle_curve_exp;
    float top_speed_limit;
};

static const struct mode_profile eco_profile = {
    .current_limit = 20.0,          // 20A max
    .acceleration_rate = 0.3,        // 30% per second
    .throttle_curve_exp = -0.3,      // Gentler throttle
    .top_speed_limit = 25.0          // 25 km/h
};

static const struct mode_profile sport_profile = {
    .current_limit = 80.0,           // 80A max
    .acceleration_rate = 1.0,        // 100% per second
    .throttle_curve_exp = 0.3,       // More aggressive
    .top_speed_limit = 50.0          // 50 km/h
};

// Select profile
const struct mode_profile *profile = mode_is_sport ? &sport_profile : &eco_profile;

// Apply throttle curve
throttle = utils_throttle_curve(throttle_raw, profile->throttle_curve_exp,
                                 0.0, THROTTLE_CURVE_MODE_EXP_NATURAL);

// Apply ramping
float ramp_rate = profile->acceleration_rate;
utils_step_towards(&throttle_ramped, throttle, ramp_rate * dt);

// Calculate motor command
float motor_current = throttle_ramped * profile->current_limit;

// Apply speed limit
float current_speed_kmh = fabsf(mc_interface_get_rpm()) * wheel_circumference * 60.0 / 1000.0;
if (current_speed_kmh > profile->top_speed_limit) {
    // Switch to brake
    motor_current = 0.0;
    mc_interface_set_brake_current(5.0);
} else {
    mc_interface_set_current(motor_current);
}
```

**Configuration:**
```c
// Mode profiles stored in config
typedef struct {
    bool mode_switch_enable;
    uint8_t mode_switch_gpio_port;
    uint8_t mode_switch_gpio_pin;

    // Eco mode settings
    float eco_current_limit;
    float eco_accel_rate;
    float eco_throttle_exp;
    float eco_top_speed;

    // Sport mode settings
    float sport_current_limit;
    float sport_accel_rate;
    float sport_throttle_exp;
    float sport_top_speed;

    // Could add more modes: comfort, off-road, etc.
} mode_config;
```

#### Option 2: Cycle Through Modes with Button

**Hardware:**
- Momentary push button on GPIO
- Press to cycle: Eco → Normal → Sport → Eco...

**Implementation:**
```c
enum drive_mode {
    MODE_ECO,
    MODE_NORMAL,
    MODE_SPORT,
    MODE_COUNT  // Number of modes
};

static enum drive_mode current_mode = MODE_NORMAL;

// Button reading with edge detection
bool button = palReadPad(GPIOB, 10);
static bool button_last = false;
static uint32_t button_press_time = 0;

if (button && !button_last) {
    // Button pressed (rising edge)
    button_press_time = chVTGetSystemTimeX();

    // Cycle to next mode
    current_mode = (current_mode + 1) % MODE_COUNT;

    // Could flash LEDs or send message to indicate mode change
    indicate_mode_change(current_mode);
}

button_last = button;

// Apply mode
switch (current_mode) {
    case MODE_ECO:
        apply_eco_profile();
        break;
    case MODE_NORMAL:
        apply_normal_profile();
        break;
    case MODE_SPORT:
        apply_sport_profile();
        break;
}
```

---

### Dual Accelerator Inputs

**Requirements:**
- Two independent throttle inputs
- Use higher of the two OR average OR specific arbitration logic
- Useful for:
  - Redundancy (safety)
  - Dual control (driver + passenger)
  - Combined control (pedal + hand throttle)

**Implementation Options:**

#### Option 1: Take Maximum (OR logic)

**Use Case:** Hand throttle OR foot pedal, whichever is higher

**Modification Points:**

1. **Read both ADC inputs** (`app_adc.c:~189`):
```c
// Read primary throttle (pedal)
float throttle1 = ADC_VOLTS(ADC_IND_EXT);
float throttle1_filtered = 0.0;
UTILS_LP_MOVING_AVG_APPROX(throttle1_filtered, throttle1, 5);
float throttle1_normalized = utils_map(throttle1_filtered,
                                        config.voltage_start,
                                        config.voltage_end,
                                        0.0, 1.0);
utils_truncate_number(&throttle1_normalized, 0.0, 1.0);

// Read secondary throttle (hand control)
float throttle2 = ADC_VOLTS(ADC_IND_EXT2);
float throttle2_filtered = 0.0;
UTILS_LP_MOVING_AVG_APPROX(throttle2_filtered, throttle2, 5);
float throttle2_normalized = utils_map(throttle2_filtered,
                                        config.voltage2_start,
                                        config.voltage2_end,
                                        0.0, 1.0);
utils_truncate_number(&throttle2_normalized, 0.0, 1.0);
```

2. **Apply arbitration logic**:
```c
// Take maximum of both inputs
float throttle_combined = utils_max_abs(throttle1_normalized, throttle2_normalized);

// Alternative: Average
// float throttle_combined = (throttle1_normalized + throttle2_normalized) / 2.0;

// Alternative: Weighted average (70% pedal, 30% hand)
// float throttle_combined = throttle1_normalized * 0.7 + throttle2_normalized * 0.3;

// Apply deadband to combined
utils_deadband(&throttle_combined, config.hyst, 1.0);

// Continue with normal processing using throttle_combined
```

3. **Add range checking for safety**:
```c
// Verify both sensors are within expected range
bool throttle1_range_ok = (throttle1 >= config.voltage_min &&
                           throttle1 <= config.voltage_max);
bool throttle2_range_ok = (throttle2 >= config.voltage2_min &&
                           throttle2 <= config.voltage2_max);

if (!throttle1_range_ok && !throttle2_range_ok) {
    // Both sensors failed - apply brake
    mc_interface_set_brake_current(timeout_get_brake_current());
    continue;
} else if (!throttle1_range_ok) {
    // Sensor 1 failed - use only sensor 2
    throttle_combined = throttle2_normalized;
} else if (!throttle2_range_ok) {
    // Sensor 2 failed - use only sensor 1
    throttle_combined = throttle1_normalized;
}
```

**Configuration:**
```c
typedef struct {
    // Throttle 1
    float voltage_start;
    float voltage_end;
    float voltage_min;  // Range check min
    float voltage_max;  // Range check max

    // Throttle 2
    float voltage2_start;
    float voltage2_end;
    float voltage2_min;
    float voltage2_max;

    // Arbitration
    enum {
        DUAL_THROTTLE_MAX,      // Use maximum
        DUAL_THROTTLE_AVG,      // Use average
        DUAL_THROTTLE_WEIGHTED, // Weighted average
        DUAL_THROTTLE_PRIORITY  // Use throttle1, fallback to throttle2
    } dual_throttle_mode;

    float dual_throttle_weight1;  // Weight for throttle 1 (0.0-1.0)

    // Safety
    bool require_both_valid;  // Brake if either sensor fails
} dual_throttle_config;
```

**Flowchart:**

```
┌─────────────────────────────┐
│ Read Throttle 1 (ADC1)      │
│ Read Throttle 2 (ADC2)      │
└──────────┬──────────────────┘
           │
           ▼
    ┌─────────────────┐
    │ Filter Both     │
    │ Map to 0-1      │
    └─────────┬───────┘
              │
              ▼
    ┌────────────────────────┐
    │ Check Range Validity   │
    └────┬──────────────┬────┘
         │              │
         ▼              ▼
    ┌─────────┐   ┌──────────┐
    │ Both OK │   │ One/Both │
    │         │   │ Invalid  │
    └────┬────┘   └─────┬────┘
         │              │
         ▼              ▼
    ┌─────────────┐  ┌──────────────┐
    │ Apply       │  │ Failsafe:    │
    │ Arbitration │  │ - Use valid  │
    │ Logic       │  │ - Or brake   │
    └─────┬───────┘  └──────┬───────┘
          │                 │
          └────────┬────────┘
                   ▼
          ┌────────────────┐
          │ Combined       │
          │ Throttle Value │
          └────────┬───────┘
                   │
                   ▼
          ┌────────────────┐
          │ Apply Curves   │
          │ and Limits     │
          └────────┬───────┘
                   │
                   ▼
          ┌────────────────┐
          │ Send Motor     │
          │ Command        │
          └────────────────┘
```

---

## Implementation Approaches

### Approach Comparison Matrix

| Feature | Extend ADC App | Extend PPM App | New Custom App | LispBM Script |
|---------|---------------|----------------|----------------|---------------|
| **Difficulty** | Low | Low | Medium | Low |
| **Flexibility** | Medium | Medium | High | High |
| **Performance** | High | High | High | Medium |
| **Recompile Required** | Yes | Yes | Yes | No |
| **Analog Inputs** | ✓✓✓ Native | ✗ Not supported | ✓✓ Need to add | ✓ Via raw-adc |
| **Digital Inputs** | ✓ Via GPIO | ✓✓✓ Native | ✓✓ Full control | ✓ Via gpio-read |
| **Complex Logic** | ✓ Limited | ✓ Limited | ✓✓✓ Unlimited | ✓✓ Good |
| **Multi-ESC** | ✓✓✓ Built-in | ✓✓✓ Built-in | ✓✓ Need to add | ✓✓ Via CAN APIs |
| **Safety Features** | ✓✓✓ Built-in | ✓✓✓ Built-in | ✓✓ Need to add | ✓ Manual |
| **Configuration UI** | ✓ via confgenerator | ✓ via confgenerator | ✓ via confgenerator | ✗ Manual params |
| **Best For** | Analog sensors, brake, dual throttle | Pulse inputs, mode switches | Complex custom control | Prototyping, user mods |

---

### Decision Tree

```
Start
  │
  ├─ Rapid prototyping / User customization?
  │  └─ YES → Use LispBM scripting
  │  └─ NO → Continue
  │
  ├─ Adding analog sensor (throttle, brake, position)?
  │  └─ YES → Extend ADC app
  │  └─ NO → Continue
  │
  ├─ Adding pulse/PWM input?
  │  └─ YES → Extend PPM app
  │  └─ NO → Continue
  │
  ├─ Complex custom control with many inputs?
  │  └─ YES → Create new custom app
  │  └─ NO → Continue
  │
  ├─ Simple GPIO switch (mode, direction)?
  │  ├─ Affects all control types → Extend timeout.c or mc_interface.c
  │  └─ Only affects one control type → Extend that app (ADC/PPM)
  │
  └─ I2C/SPI/CAN device integration?
     └─ Create new app or use LispBM
```

---

## Testing and Validation

### Testing Strategy

#### 1. Hardware Testing

**Input Signal Verification:**
```
Test: Verify ADC reading
  1. Connect known voltage to ADC pin (e.g., 1.65V)
  2. Read with debugger or log over USB
  3. Expected: ADC_VOLTS() returns ~1.65V
  4. Verify filtering doesn't introduce excessive lag

Test: Verify GPIO reading
  1. Connect GPIO to 3.3V and GND alternately
  2. Read with palReadPad()
  3. Expected: Returns 1 for high, 0 for low
  4. Check debouncing works (no bouncing on transitions)

Test: Verify I2C/SPI communication
  1. Read device ID register
  2. Expected: Returns known device ID
  3. Check for timeouts or errors
  4. Verify bus speed is appropriate
```

#### 2. Functional Testing

**Test Matrix for Brake Input:**
| Brake Position | Throttle Position | Expected Behavior |
|---------------|-------------------|-------------------|
| 0% | 0% | No motor command |
| 0% | 50% | 50% throttle command |
| 0% | 100% | 100% throttle command |
| 50% | 0% | 50% brake command |
| 50% | 50% | Brake overrides throttle |
| 100% | 100% | Full brake (throttle ignored) |

**Test Matrix for Drive/Reverse:**
| Gear | Throttle | Speed | Expected Behavior |
|------|----------|-------|-------------------|
| Park | Any | Any | Park brake applied |
| Drive | 0% | Stopped | No command |
| Drive | 50% | 10 km/h | Forward acceleration |
| Reverse | 50% | 0 km/h | Reverse acceleration |
| Reverse | 50% | 5 km/h (fwd) | Cannot change to reverse |
| Drive | 0% | -5 km/h (reverse) | Regen brake to slow down |

#### 3. Safety Testing

**Critical Tests:**
```
Test: Sensor failure
  1. Disconnect throttle sensor
  2. Expected: Timeout applies brake within 100ms
  3. Verify: Motor stops, fault logged

Test: Sensor out of range
  1. Apply voltage outside configured range
  2. Expected: Output disabled, brake applied
  3. Verify: range_ok flag is false

Test: Direction change while moving
  1. Move forward at 10 km/h
  2. Switch to reverse
  3. Expected: Gear change denied, remains in forward
  4. Verify: Must stop before changing

Test: Ignition off while moving
  1. Move forward at 20 km/h
  2. Turn ignition off
  3. Expected: Brake applied, motor stops
  4. Verify: All apps disabled, timeout active

Test: Mode change under load
  1. Accelerate in sport mode
  2. Switch to eco mode
  3. Expected: Current limit reduces smoothly
  4. Verify: No sudden deceleration or fault
```

#### 4. Integration Testing

**Multi-ESC Testing:**
```
Test: Commands sent to all ESCs
  1. Configure multi-ESC mode
  2. Apply throttle
  3. Verify: CAN messages sent to all ESCs
  4. Monitor: All ESCs respond with status

Test: Traction control with custom inputs
  1. Apply throttle with one wheel on ice
  2. Verify: TC reduces current to slipping wheel
  3. With brake: All wheels brake equally
```

### Debug Tools

**1. Real-time Monitoring (via VESC Tool):**
- Real-time data view shows all ADC channels
- Can log to file for analysis
- Oscilloscope mode for fast signals

**2. Terminal Commands:**
```
# Print ADC values
> adc

# Print GPIO states
> pins

# Test motor control
> set_current 10.0

# Enable sampling for debug
> sample_start
```

**3. LispBM Debug Script:**
```lisp
; Monitor inputs in real-time
(defun monitor-inputs ()
    (loopwhile t
        (print (list
            "Throttle1:" (adc-ext)
            "Throttle2:" (raw-adc-voltage 3 1.0)
            "Brake:" (raw-adc-voltage 4 1.0)
            "Mode:" (gpio-read 10)
            "Gear:" (gpio-read 6)
        ))
        (sleep 0.1)
    )
)

(monitor-inputs)
```

**4. LED Indicators:**
```c
// Add LED indicators for debugging
void indicate_mode(enum drive_mode mode) {
    switch (mode) {
        case MODE_ECO:
            ledpwm_set_intensity(LED_GREEN, 0.2);  // Dim green
            break;
        case MODE_NORMAL:
            ledpwm_set_intensity(LED_GREEN, 0.5);  // Medium green
            break;
        case MODE_SPORT:
            ledpwm_set_intensity(LED_RED, 1.0);    // Bright red
            break;
    }
}
```

---

## Summary

### Quick Reference for Adding Inputs

**For Analog Sensors (Throttle, Brake):**
1. Define ADC channel in `hwconf/hw_*.h`
2. Read with `ADC_VOLTS(ADC_IND_XXX)` in `app_adc.c`
3. Add filtering with `UTILS_LP_MOVING_AVG_APPROX()`
4. Map to 0-1 range with `utils_map()`
5. Add configuration to `conf_general.h` and `confgenerator.c`

**For Digital Switches (Mode, Direction):**
1. Define GPIO pin in `hwconf/hw_*.h`
2. Configure as input: `palSetPadMode(port, pin, PAL_MODE_INPUT_PULLUP)`
3. Read with `palReadPad(port, pin)` in app thread
4. Add debouncing (10-50ms)
5. Implement state machine for complex logic

**For Complex Control:**
1. Copy `applications/app_template.c` (or existing app)
2. Rename to `app_custom.c`
3. Implement custom thread function
4. Add to build system (`Makefile`)
5. Register in `app.c`

**For Quick Prototyping:**
1. Write LispBM script
2. Test in VESC Tool REPL
3. Save to firmware once working
4. No recompilation needed!

---

**Document Version:** 1.0
**Date:** 2025-11-11
**VESC Firmware Version:** Based on current master branch
**Author:** AI-generated technical documentation

