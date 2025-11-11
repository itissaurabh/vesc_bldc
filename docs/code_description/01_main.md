# main.c - System Entry Point and Main Loop

**File:** `main.c`
**Purpose:** System initialization, main loop, and background threads
**Lines:** 407

---

## Overview

The main.c file is the entry point for the VESC firmware. It handles:
- System initialization (hardware, peripherals, communication)
- Creating background threads for monitoring and control
- Main idle loop
- Emergency motor stop and system reset

**Hardware Resources Used:**
- TIM1: mcpwm (BLDC motor control)
- TIM2: mcpwm_foc (FOC motor control)
- TIM5: timer (system timing)
- TIM8: mcpwm (dual motor support)
- TIM3: servo_dec/Encoder/pwm_servo
- TIM4: WS2811/WS2812 LEDs/Encoder

---

## Function Reference

### Main Entry Point

#### `int main(void)`
**Lines:** 249-364
**Purpose:** System entry point - initializes all subsystems and starts threads

**Initialization Sequence:**

1. **HAL and OS Initialization** (Lines 250-251)
   ```c
   halInit();      // ChibiOS Hardware Abstraction Layer
   chSysInit();    // ChibiOS kernel initialization
   ```

2. **Early Hardware Init** (Lines 256-274)
   - Initialize gate driver enable pins (prevents floating pins causing current draw)
   - `HW_EARLY_INIT()` - Hardware-specific early initialization
   - Set boot OK GPIO (if defined)
   - 100ms delay for power stabilization
   - `mempools_init()` - Initialize memory pools
   - `events_init()` - Initialize event system
   - `timer_init()` - Initialize system timer
   - `hw_init_gpio()` - Initialize GPIO pins
   - Turn off status LEDs

3. **Configuration System** (Line 276)
   ```c
   conf_general_init();
   ```
   - Loads configuration from EEPROM
   - Initializes default settings if needed
   - Validates stored configuration

4. **Flash Memory Verification** (Lines 278-286)
   ```c
   if (flash_helper_verify_flash_memory() == FAULT_CODE_FLASH_CORRUPTION)
   ```
   - Verifies flash memory integrity using CRC
   - If corrupted: enters infinite loop blinking red LED (unsafe to run)
   - Prevents running with corrupted firmware

5. **Motor Control Initialization** (Lines 288-289)
   ```c
   ledpwm_init();      // LED PWM for status indicators
   mc_interface_init(); // Motor control interface
   ```

6. **Communication Initialization** (Lines 291-306)
   ```c
   commands_init();           // Command processor
   comm_usb_init();          // USB communication
   app_uartcomm_initialize(); // UART communication
   comm_can_init();          // CAN bus communication
   ```

7. **Application Configuration** (Lines 298-302)
   - Allocate app configuration from memory pool
   - Read app configuration from EEPROM
   - Start UART communication on built-in and header ports
   - Apply application configuration

8. **NRF24L01+ Wireless (Optional)** (Lines 309-324)
   - If hardware has permanent NRF module:
     - Initialize NRF driver
     - If found: restart RF helper
     - If not found: reconfigure SPI pins for external NRF

9. **Background Threads** (Lines 327-329)
   ```c
   chThdCreateStatic(led_thread_wa, sizeof(led_thread_wa),
                     NORMALPRIO, led_thread, NULL);
   chThdCreateStatic(periodic_thread_wa, sizeof(periodic_thread_wa),
                     NORMALPRIO, periodic_thread, NULL);
   chThdCreateStatic(flash_integrity_check_thread_wa,
                     sizeof(flash_integrity_check_thread_wa),
                     LOWPRIO, flash_integrity_check_thread, NULL);
   ```

10. **Final Initialization** (Lines 331-343)
    - `timeout_init()` - Safety timeout system
    - `timeout_configure()` - Configure timeout parameters
    - `bm_init()` - Black Magic Probe debug interface (if enabled)
    - `shutdown_init()` - Shutdown handler
    - `imu_reset_orientation()` - Reset IMU orientation
    - 500ms stabilization delay
    - Set `m_init_done = true` flag
    - Set boot OK GPIO high

11. **CAN Boot Notification** (Lines 349-357)
    - If CAN in VESC mode: transmit boot notification frame
    - Notifies other VESCs on CAN bus that this device has booted
    - Includes hardware name in CAN frame

12. **Main Loop** (Lines 361-363)
    ```c
    for(;;) {
        chThdSleepMilliseconds(10);
    }
    ```
    - Infinite idle loop
    - All work done in background threads
    - 10ms sleep prevents busy-waiting

**Returns:** Never returns (infinite loop)

**Call Graph:**
```
main()
├── halInit()                    [ChibiOS HAL]
├── chSysInit()                  [ChibiOS Kernel]
├── HW_EARLY_INIT()              [Hardware config]
├── mempools_init()              [mempools.c]
├── events_init()                [events.c]
├── timer_init()                 [driver/timer.c]
├── hw_init_gpio()               [hwconf/hw_*.c]
├── conf_general_init()          [conf_general.c]
├── flash_helper_verify_flash_memory() [flash_helper.c]
├── ledpwm_init()                [driver/ledpwm.c]
├── mc_interface_init()          [motor/mc_interface.c] *** CRITICAL ***
├── commands_init()              [comm/commands.c]
├── comm_usb_init()              [comm/comm_usb.c]
├── app_uartcomm_initialize()    [applications/app_uartcomm.c]
├── comm_can_init()              [comm/comm_can.c]
├── nrf_driver_init()            [driver/nrf/nrf_driver.c]
├── chThdCreateStatic()          [Creates 3 threads]
├── timeout_init()               [timeout.c]
├── shutdown_init()              [shutdown.c]
└── imu_reset_orientation()      [imu/imu.c]
```

---

### Background Threads

#### `static THD_FUNCTION(led_thread, arg)`
**Lines:** 111-155
**Thread Name:** "Main LED"
**Priority:** NORMALPRIO
**Stack:** 256 bytes
**Purpose:** Controls status LEDs based on motor state and faults

**Behavior:**

1. **Green LED - Motor State Indicator** (Lines 117-125)
   ```c
   mc_state state1 = mc_interface_get_state();
   mc_interface_select_motor_thread(2);
   mc_state state2 = mc_interface_get_state();
   mc_interface_select_motor_thread(1);

   if ((state1 == MC_STATE_RUNNING) || (state2 == MC_STATE_RUNNING)) {
       ledpwm_set_intensity(LED_GREEN, 1.0);    // Full brightness when running
   } else {
       ledpwm_set_intensity(LED_GREEN, 0.2);    // Dim when not running
   }
   ```
   - Checks both motor 1 and motor 2 (dual motor support)
   - Full brightness: Motor running
   - Dim (20%): Motor idle/stopped

2. **Red LED - Fault Indicator** (Lines 127-151)
   ```c
   mc_fault_code fault = mc_interface_get_fault();
   mc_interface_select_motor_thread(2);
   mc_fault_code fault2 = mc_interface_get_fault();
   mc_interface_select_motor_thread(1);

   if (fault != FAULT_CODE_NONE || fault2 != FAULT_CODE_NONE) {
       // Blink red LED N times (N = fault code number)
       for (int i = 0; i < (int)fault; i++) {
           ledpwm_set_intensity(LED_RED, 1.0);
           chThdSleepMilliseconds(250);
           ledpwm_set_intensity(LED_RED, 0.0);
           chThdSleepMilliseconds(250);
       }
       chThdSleepMilliseconds(500);

       // Then blink for motor 2 faults
       for (int i = 0; i < (int)fault2; i++) {
           // ... same pattern
       }
   } else {
       ledpwm_set_intensity(LED_RED, 0.0);  // Off when no fault
   }
   ```

   **Fault Code Blinking Pattern:**
   - Number of blinks = fault code value
   - Motor 1 faults first, then motor 2 faults
   - 500ms pause between motor fault displays
   - Pattern repeats continuously until fault cleared

3. **Update Rate:** 10ms sleep between updates (Line 153)

**Fault Codes (Example):**
- 1 blink: FAULT_CODE_OVER_VOLTAGE
- 2 blinks: FAULT_CODE_UNDER_VOLTAGE
- 3 blinks: FAULT_CODE_DRV
- etc. (see datatypes.h for complete list)

---

#### `static THD_FUNCTION(periodic_thread, arg)`
**Lines:** 157-209
**Thread Name:** "Main periodic"
**Priority:** NORMALPRIO
**Stack:** 256 bytes
**Purpose:** Periodic tasks including motor detection position reporting and temperature compensation

**Tasks Performed:**

1. **Motor Detection Position Reporting** (Lines 163-165)
   ```c
   if (mc_interface_get_state() == MC_STATE_DETECTING) {
       commands_send_rotor_pos(mcpwm_get_detect_pos());
   }
   ```
   - During motor parameter detection
   - Sends rotor position to VESC Tool for visualization
   - Updates at 100Hz (10ms period)

2. **Position Display Mode Handling** (Lines 167-203)

   Gets current display mode and sends appropriate position data:

   **a. Encoder Modes:**
   - `DISP_POS_MODE_ENCODER`: Raw encoder angle
   - `DISP_POS_MODE_PID_POS`: Current PID position setpoint
   - `DISP_POS_MODE_PID_POS_ERROR`: Position tracking error

   **b. FOC-Specific Modes** (only when motor type is FOC):
   - `DISP_POS_MODE_OBSERVER`: Sensorless observer angle
   - `DISP_POS_MODE_ENCODER_OBSERVER_ERROR`: Observer vs encoder error
   - `DISP_POS_MODE_HALL_OBSERVER_ERROR`: Observer vs hall sensor error

   Used for:
   - Real-time tuning in VESC Tool
   - Encoder calibration verification
   - Observer accuracy validation

3. **HSI Temperature Compensation** (Line 205)
   ```c
   HW_TRIM_HSI();  // Compensate HSI for temperature
   ```
   - Adjusts internal high-speed oscillator (HSI)
   - Compensates for temperature drift
   - Maintains accurate timing
   - Hardware-specific macro (defined in hw_*.h files)

4. **Update Rate:** 10ms (100Hz)

---

#### `static THD_FUNCTION(flash_integrity_check_thread, arg)`
**Lines:** 96-109
**Thread Name:** "Flash check"
**Priority:** LOWPRIO (lowest priority)
**Stack:** 256 bytes
**Purpose:** Continuously verifies flash memory integrity in background

**Operation:**

```c
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_CRC, ENABLE);  // Enable CRC peripheral

for(;;) {
    if (flash_helper_verify_flash_memory_chunk() == FAULT_CODE_FLASH_CORRUPTION) {
        NVIC_SystemReset();  // Reset system if corruption detected
    }
    chThdSleepMilliseconds(6);
}
```

**Key Features:**
- Verifies flash memory in small chunks (non-blocking)
- Uses hardware CRC peripheral for speed
- 6ms sleep between checks (allows other threads to run)
- **Automatic system reset** if corruption detected
- Low priority - runs when CPU idle

**Why This Is Important:**
- Detects single-event upsets (SEU) from cosmic rays, EMI
- Prevents running corrupted code
- Fails safe by resetting immediately
- Continuous monitoring during operation

**Performance:**
- Checks ~170 times per second (6ms period)
- Each check verifies a small chunk
- Full firmware verified over multiple iterations
- Minimal CPU overhead due to low priority

---

### Utility Functions

#### `void assert_failed(uint8_t* file, uint32_t line)`
**Lines:** 212-218
**Purpose:** Assertion failure handler (called when assert() fails)

**Behavior:**
```c
commands_printf("Wrong parameters value: file %s on line %d\r\n", file, line);
mc_interface_release_motor();  // Stop motor immediately
while(1) {
    chThdSleepMilliseconds(1);  // Infinite loop
}
```

**When Called:**
- STM32 library assertion failures
- Invalid parameters passed to library functions
- Enabled with `USE_FULL_ASSERT` compile flag

**Safety:**
- Stops motor immediately before halting
- Prints error location for debugging
- Prevents system from continuing in unsafe state

---

#### `bool main_init_done(void)`
**Lines:** 220-222
**Purpose:** Check if main initialization is complete

**Returns:**
- `true`: All initialization complete, system ready
- `false`: Still initializing

**Usage:**
- Called by other modules to check if they can start operations
- Prevents accessing uninitialized subsystems
- Example: CAN communication waits for this before starting

---

#### `uint32_t main_calc_hw_crc(void)`
**Lines:** 224-247
**Purpose:** Calculate CRC of hardware-specific data (QML UI, custom config XML)

**Calculates CRC of:**
1. QML hardware UI data (if defined)
2. Custom configuration XML files
3. QML code stored in flash

**Returns:** 32-bit CRC value

**Usage:**
- VESC Tool uses this to verify UI compatibility
- Ensures configuration XML matches firmware
- Detects if custom UI needs updating

**Implementation:**
```c
uint32_t crc = 0;

// Add QML hardware UI
#ifdef QMLUI_SOURCE_HW
    crc = crc32_with_init(data_qml_hw, DATA_QML_HW_SIZE, crc);
#endif

// Add all custom config XMLs
for (int i = 0; i < conf_custom_cfg_num(); i++) {
    uint8_t *data = 0;
    int len = conf_custom_get_cfg_xml(i, &data);
    if (len > 0) {
        crc = crc32_with_init(data, len, crc);
    }
}

// Add QML code from flash
if (flash_helper_code_size(CODE_IND_QML) > 0) {
    crc = crc32_with_init(
        flash_helper_code_data(CODE_IND_QML),
        flash_helper_code_size(CODE_IND_QML),
        crc);
}

return crc;
```

---

#### `void main_stop_motor_and_reset(void)`
**Lines:** 366-406
**Purpose:** Emergency motor stop and system reset

**Operation:**

1. **Force All PWM Outputs Inactive** (Lines 367-379)
   ```c
   // Motor 1 (TIM1)
   TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_InActive);
   TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);      // High-side ON
   TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);   // Low-side OFF
   // ... repeat for channels 2 and 3
   TIM_GenerateEvent(TIM1, TIM_EventSource_COM);
   ```

   **PWM State After Forcing:**
   - All high-side outputs: Active (but forced to inactive level)
   - All low-side outputs: Disabled
   - Result: All phases pulled low through high-side FETs body diodes
   - Motor coasts to stop

2. **Disable Gate Driver** (Lines 381-383)
   ```c
   #ifdef HW_HAS_DRV8313
       DISABLE_BR();  // Disable DRV8313 gate driver
   #endif
   ```

3. **Dual Motor Support** (Lines 385-403)
   - If hardware has dual motors (TIM8)
   - Repeat same PWM forcing for motor 2
   - Disable second gate driver if present

4. **System Reset** (Line 405)
   ```c
   NVIC_SystemReset();  // Reset microcontroller
   ```

**When Called:**
- Bootloader entry request
- Emergency stop command
- Firmware update preparation
- Critical error recovery

**Safety Features:**
- Forces safe PWM state before reset
- Disables gate drivers
- Ensures motors cannot draw current during reset
- Prevents startup glitches

---

## Initialization Flowchart

```
System Power-On
      |
      v
+------------------+
| halInit()        | Hardware Abstraction Layer
| chSysInit()      | ChibiOS Kernel
+------------------+
      |
      v
+------------------+
| HW_EARLY_INIT()  | Gate drivers, GPIO
| mempools_init()  | Memory pools
| events_init()    | Event system
| timer_init()     | System timer
+------------------+
      |
      v
+------------------+
| conf_general_init() | Load EEPROM config
+------------------+
      |
      v
+------------------+
| Flash Verify     | CRC check firmware
+------------------+
      |
      +--- Corrupted? ---> Infinite Blink Loop (SAFE HALT)
      |
      v OK
+------------------+
| mc_interface_init() | Motor control init
| ledpwm_init()    |
+------------------+
      |
      v
+------------------+
| Communication    | USB, UART, CAN
| commands_init()  |
| comm_usb_init()  |
| comm_can_init()  |
+------------------+
      |
      v
+------------------+
| App Config       | Load & apply app config
+------------------+
      |
      v
+------------------+
| Wireless (opt)   | NRF24L01+
+------------------+
      |
      v
+------------------+
| Start Threads    | LED, Periodic, Flash Check
+------------------+
      |
      v
+------------------+
| Timeout System   | Safety timeout
| Shutdown Handler |
| IMU Init         |
+------------------+
      |
      v
+------------------+
| m_init_done = 1  | Signal init complete
+------------------+
      |
      v
+------------------+
| CAN Boot Notify  | Tell other VESCs we're up
+------------------+
      |
      v
+------------------+
| Main Idle Loop   | Sleep forever
| (10ms sleep)     | Work done in threads
+------------------+
```

---

## Thread Summary

| Thread | Priority | Stack | Update Rate | Purpose |
|--------|----------|-------|-------------|---------|
| led_thread | NORMAL | 256B | 10ms | Status LED control |
| periodic_thread | NORMAL | 256B | 10ms | Position reporting, temp compensation |
| flash_integrity_check_thread | LOW | 256B | 6ms | Flash CRC verification |

**Plus threads created by subsystems:**
- Motor control ISR threads (mcpwm, mcpwm_foc)
- Communication threads (USB, CAN, UART)
- Application threads (app_*)
- LispBM thread (if enabled)

---

## Critical Initialization Dependencies

**Must Init First:**
1. `halInit()`, `chSysInit()` - OS must be running
2. `timer_init()` - Required by many subsystems
3. `hw_init_gpio()` - GPIO must be configured
4. `conf_general_init()` - Config needed by most subsystems

**Must Init Before Motor Control:**
1. `ledpwm_init()` - Status indicators
2. `hw_init_gpio()` - Gate driver pins

**Must Init Before Communication:**
1. `mc_interface_init()` - Commands need motor interface
2. `commands_init()` - Command processor

**Last to Init:**
1. Threads (need all subsystems ready)
2. Timeout system (needs config)
3. IMU (can reset orientation anytime)

---

## Key Macros Used

### Hardware-Specific Macros (from hwconf/hw_*.h)

```c
HW_EARLY_INIT()              // Early hardware initialization
HW_INIT_GPIO()               // GPIO initialization (deprecated, use hw_init_gpio())
HW_TRIM_HSI()                // HSI temperature compensation
INIT_BR()                    // Initialize bootstrap/brake resistor
DISABLE_BR()                 // Disable bootstrap/brake resistor
HW_PERMANENT_NRF_FAILED_HOOK() // Called if permanent NRF init fails
```

### Configuration Macros

```c
COMM_USE_USB                 // Enable USB communication
CAN_ENABLE                   // Enable CAN bus
HAS_BLACKMAGIC               // Enable Black Magic Probe debug
USE_LISPBM                   // Enable LispBM scripting
HW_HAS_PERMANENT_NRF         // Hardware has permanent NRF module
HW_HAS_DRV8313               // Gate driver type
HW_HAS_DUAL_MOTORS           // Dual motor support
```

---

## Next Steps in Call Hierarchy

From main.c, the most critical paths to document next:

1. **`mc_interface_init()`** → `motor/mc_interface.c`
   - Motor control initialization
   - Creates motor control threads
   - Sets up PWM, ADC

2. **`conf_general_init()`** → `conf_general.c`
   - Configuration system
   - EEPROM reading/writing

3. **`commands_init()`** → `comm/commands.c`
   - Command processing
   - Packet handling

4. **`comm_can_init()`** → `comm/comm_can.c`
   - CAN bus initialization
   - Multi-VESC communication

5. **`app_set_configuration()`** → `applications/app.c`
   - Application layer
   - Throttle/brake control
