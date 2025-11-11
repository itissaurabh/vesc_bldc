# VESC Thread Call Flow Documentation

## Table of Contents
1. [System Threads](#system-threads)
   - [LED Thread](#led-thread)
   - [Periodic Thread](#periodic-thread)
   - [Flash Integrity Check Thread](#flash-integrity-check-thread)
2. [Motor Control Threads](#motor-control-threads)
   - [Timer Thread](#timer-thread)
   - [Fault Stop Thread](#fault-stop-thread)
   - [Statistics Thread](#statistics-thread)
   - [Sample Send Thread](#sample-send-thread)
3. [Safety Thread](#safety-thread)
   - [Timeout Thread](#timeout-thread)
4. [Application Threads](#application-threads)
   - [PPM Thread](#ppm-thread)
   - [ADC Thread](#adc-thread)
   - [PAS Thread](#pas-thread)
   - [Nunchuk Thread](#nunchuk-thread)
5. [Communication Threads](#communication-threads)
   - [USB Serial Thread](#usb-serial-thread)
   - [CAN RX Thread](#can-rx-thread)
   - [CAN TX Thread](#can-tx-thread)
6. [Subsystem Threads](#subsystem-threads)
   - [IMU Thread](#imu-thread)
   - [NRF Thread](#nrf-thread)
   - [Encoder Thread](#encoder-thread)
   - [LispBM Thread](#lispbm-thread)

---

## System Threads

### LED Thread

**File:** `main.c:111-155`
**Created at:** `main.c:327`
**Priority:** `NORMALPRIO` (64)
**Execution Rate:** 100 Hz (10ms period)
**Stack Size:** 256 bytes

#### Purpose
Provides visual feedback about motor state and fault conditions through red and green LEDs.

#### Thread Configuration
```c
static THD_WORKING_AREA(led_thread_wa, 256);
chThdCreateStatic(led_thread_wa, sizeof(led_thread_wa),
                  NORMALPRIO, led_thread, NULL);
```

#### Call Flow

**Entry Point:** `led_thread()`

```
led_thread()
  ├─ chRegSetThreadName("Main LED")
  │
  └─ for(;;)  // Infinite loop
      │
      ├─ Get motor 1 state
      │  ├─ mc_interface_get_state() → MC_STATE_RUNNING or other
      │  └─ Store in state1
      │
      ├─ Switch to motor 2 context
      │  └─ mc_interface_select_motor_thread(2)
      │
      ├─ Get motor 2 state
      │  ├─ mc_interface_get_state() → MC_STATE_RUNNING or other
      │  └─ Store in state2
      │
      ├─ Switch back to motor 1 context
      │  └─ mc_interface_select_motor_thread(1)
      │
      ├─ Update GREEN LED based on motor state
      │  ├─ if (state1 == MC_STATE_RUNNING || state2 == MC_STATE_RUNNING)
      │  │   └─ ledpwm_set_intensity(LED_GREEN, 1.0)  // Full brightness
      │  └─ else
      │      └─ ledpwm_set_intensity(LED_GREEN, 0.2)  // Dim (standby)
      │
      ├─ Get fault codes from both motors
      │  ├─ fault = mc_interface_get_fault()  // Motor 1
      │  ├─ mc_interface_select_motor_thread(2)
      │  ├─ fault2 = mc_interface_get_fault()  // Motor 2
      │  └─ mc_interface_select_motor_thread(1)
      │
      ├─ Update RED LED based on fault conditions
      │  ├─ if (fault != FAULT_CODE_NONE || fault2 != FAULT_CODE_NONE)
      │  │   │
      │  │   ├─ Blink fault code for motor 1
      │  │   │  └─ for (i = 0; i < fault; i++)
      │  │   │      ├─ ledpwm_set_intensity(LED_RED, 1.0)
      │  │   │      ├─ chThdSleepMilliseconds(250)
      │  │   │      ├─ ledpwm_set_intensity(LED_RED, 0.0)
      │  │   │      └─ chThdSleepMilliseconds(250)
      │  │   │
      │  │   ├─ chThdSleepMilliseconds(500)  // Pause between motors
      │  │   │
      │  │   └─ Blink fault code for motor 2
      │  │      └─ for (i = 0; i < fault2; i++)
      │  │          ├─ ledpwm_set_intensity(LED_RED, 1.0)
      │  │          ├─ chThdSleepMilliseconds(250)
      │  │          ├─ ledpwm_set_intensity(LED_RED, 0.0)
      │  │          └─ chThdSleepMilliseconds(250)
      │  │
      │  └─ else
      │      └─ ledpwm_set_intensity(LED_RED, 0.0)  // Turn off
      │
      └─ chThdSleepMilliseconds(10)  // 100 Hz rate
```

#### Key ChibiOS APIs Used
- `chRegSetThreadName()` - Register thread name for debugging
- `chThdSleepMilliseconds()` - Periodic sleep for 100 Hz rate

#### Function Calls Summary
1. `mc_interface_get_state()` - Read motor state (2x for dual motors)
2. `mc_interface_select_motor_thread()` - Switch motor context (4x)
3. `mc_interface_get_fault()` - Read fault code (2x for dual motors)
4. `ledpwm_set_intensity()` - Control LED brightness (multiple times)
5. `chThdSleepMilliseconds()` - Timing control

#### Timing Characteristics
- **Base Period:** 10ms (100 Hz)
- **Fault Display Time:** 250ms on + 250ms off per blink
- **CPU Usage:** Minimal (~0.1%)
- **Blocking:** Yes, during fault code blinking

#### Notes
- Thread blocks during fault code display, extending cycle time
- Number of blinks indicates fault code number
- 500ms pause separates motor 1 and motor 2 fault codes
- Green LED brightness: 100% running, 20% idle

---

### Periodic Thread

**File:** `main.c:157-209`
**Created at:** `main.c:328`
**Priority:** `NORMALPRIO` (64)
**Execution Rate:** 100 Hz (10ms period)
**Stack Size:** 256 bytes

#### Purpose
Periodically sends position/angle data to host for display and performs HSI temperature compensation.

#### Thread Configuration
```c
static THD_WORKING_AREA(periodic_thread_wa, 256);
chThdCreateStatic(periodic_thread_wa, sizeof(periodic_thread_wa),
                  NORMALPRIO, periodic_thread, NULL);
```

#### Call Flow

**Entry Point:** `periodic_thread()`

```
periodic_thread()
  ├─ chRegSetThreadName("Main periodic")
  │
  └─ for(;;)  // Infinite loop
      │
      ├─ Check if motor detection is running
      │  ├─ if (mc_interface_get_state() == MC_STATE_DETECTING)
      │  │   └─ commands_send_rotor_pos(mcpwm_get_detect_pos())
      │  └─ [Detection sends rotor position during parameter detection]
      │
      ├─ Get display position mode
      │  └─ display_mode = commands_get_disp_pos_mode()
      │
      ├─ Send position data based on display mode (BLDC/DC compatible)
      │  └─ switch (display_mode)
      │      │
      │      ├─ case DISP_POS_MODE_ENCODER:
      │      │   └─ commands_send_rotor_pos(encoder_read_deg())
      │      │
      │      ├─ case DISP_POS_MODE_PID_POS:
      │      │   └─ commands_send_rotor_pos(mc_interface_get_pid_pos_now())
      │      │
      │      ├─ case DISP_POS_MODE_PID_POS_ERROR:
      │      │   ├─ set_pos = mc_interface_get_pid_pos_set()
      │      │   ├─ now_pos = mc_interface_get_pid_pos_now()
      │      │   └─ commands_send_rotor_pos(utils_angle_difference(set_pos, now_pos))
      │      │
      │      └─ default: break
      │
      ├─ Send FOC-specific position data (if FOC mode)
      │  ├─ if (mc_interface_get_configuration()->motor_type == MOTOR_TYPE_FOC)
      │  │   │
      │  │   └─ switch (display_mode)
      │  │       │
      │  │       ├─ case DISP_POS_MODE_OBSERVER:
      │  │       │   └─ commands_send_rotor_pos(mcpwm_foc_get_phase_observer())
      │  │       │
      │  │       ├─ case DISP_POS_MODE_ENCODER_OBSERVER_ERROR:
      │  │       │   ├─ obs = mcpwm_foc_get_phase_observer()
      │  │       │   ├─ enc = mcpwm_foc_get_phase_encoder()
      │  │       │   └─ commands_send_rotor_pos(utils_angle_difference(obs, enc))
      │  │       │
      │  │       ├─ case DISP_POS_MODE_HALL_OBSERVER_ERROR:
      │  │       │   ├─ obs = mcpwm_foc_get_phase_observer()
      │  │       │   ├─ hall = mcpwm_foc_get_phase_hall()
      │  │       │   └─ commands_send_rotor_pos(utils_angle_difference(obs, hall))
      │  │       │
      │  │       └─ default: break
      │  │
      │  └─ [FOC observer provides sensorless position estimation]
      │
      ├─ Perform HSI oscillator temperature compensation
      │  └─ HW_TRIM_HSI()
      │     └─ [Adjusts internal oscillator frequency based on temperature]
      │
      └─ chThdSleepMilliseconds(10)  // 100 Hz rate
```

#### Key ChibiOS APIs Used
- `chRegSetThreadName()` - Register thread name
- `chThdSleepMilliseconds()` - Periodic sleep

#### Function Calls Summary
1. `mc_interface_get_state()` - Check if detection is running
2. `mcpwm_get_detect_pos()` - Get rotor position during detection
3. `commands_get_disp_pos_mode()` - Read display mode setting
4. `commands_send_rotor_pos()` - Send position to host (USB/UART/CAN)
5. `encoder_read_deg()` - Read encoder angle in degrees
6. `mc_interface_get_pid_pos_now()` - Get current PID position
7. `mc_interface_get_pid_pos_set()` - Get PID position setpoint
8. `utils_angle_difference()` - Calculate angle error
9. `mc_interface_get_configuration()` - Read motor configuration
10. `mcpwm_foc_get_phase_observer()` - Get observer angle (FOC)
11. `mcpwm_foc_get_phase_encoder()` - Get encoder angle (FOC)
12. `mcpwm_foc_get_phase_hall()` - Get hall sensor angle (FOC)
13. `HW_TRIM_HSI()` - Temperature compensate HSI oscillator

#### Display Modes Supported
- `DISP_POS_MODE_ENCODER` - Raw encoder position
- `DISP_POS_MODE_PID_POS` - PID controller current position
- `DISP_POS_MODE_PID_POS_ERROR` - PID position error
- `DISP_POS_MODE_OBSERVER` - FOC observer angle (sensorless)
- `DISP_POS_MODE_ENCODER_OBSERVER_ERROR` - Observer vs encoder error
- `DISP_POS_MODE_HALL_OBSERVER_ERROR` - Observer vs hall error

#### Timing Characteristics
- **Period:** 10ms (100 Hz)
- **CPU Usage:** Minimal (~0.1%)
- **Non-blocking:** Yes

#### Notes
- Provides real-time position feedback to VESC Tool
- HSI trimming prevents clock drift due to temperature changes
- Observer error modes useful for tuning sensorless FOC
- Detection mode only active during motor parameter detection

---

### Flash Integrity Check Thread

**File:** `main.c:96-109`
**Created at:** `main.c:329`
**Priority:** `LOWPRIO` (2)
**Execution Rate:** ~167 Hz (6ms period)
**Stack Size:** 256 bytes

#### Purpose
Continuously verifies flash memory integrity to detect corruption and trigger system reset if needed.

#### Thread Configuration
```c
static THD_WORKING_AREA(flash_integrity_check_thread_wa, 256);
chThdCreateStatic(flash_integrity_check_thread_wa, sizeof(flash_integrity_check_thread_wa),
                  LOWPRIO, flash_integrity_check_thread, NULL);
```

#### Call Flow

**Entry Point:** `flash_integrity_check_thread()`

```
flash_integrity_check_thread()
  ├─ chRegSetThreadName("Flash check")
  │
  ├─ Enable CRC hardware peripheral
  │  └─ RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_CRC, ENABLE)
  │     └─ [Enables STM32 hardware CRC engine for fast verification]
  │
  └─ for(;;)  // Infinite loop
      │
      ├─ Verify next chunk of flash memory
      │  └─ if (flash_helper_verify_flash_memory_chunk() == FAULT_CODE_FLASH_CORRUPTION)
      │      │
      │      └─ NVIC_SystemReset()
      │         └─ [System reset - critical corruption detected]
      │
      └─ chThdSleepMilliseconds(6)  // ~167 Hz rate
```

#### Key ChibiOS APIs Used
- `chRegSetThreadName()` - Register thread name
- `chThdSleepMilliseconds()` - Periodic sleep

#### Function Calls Summary
1. `RCC_AHB1PeriphClockCmd()` - Enable CRC hardware
2. `flash_helper_verify_flash_memory_chunk()` - Verify flash chunk
3. `NVIC_SystemReset()` - Reset MCU on corruption

#### Flash Verification Algorithm
The thread uses chunked verification:
1. Divides flash into small chunks
2. Verifies one chunk per iteration (6ms)
3. Uses hardware CRC for fast calculation
4. Compares against stored CRC values
5. Resets system immediately on mismatch

#### Timing Characteristics
- **Period:** 6ms (~167 Hz)
- **CPU Usage:** Very low due to hardware CRC (~0.05%)
- **Priority:** LOWPRIO (2) - lowest priority, runs when CPU available
- **Non-blocking:** Yes

#### Notes
- **Low priority** ensures it doesn't interfere with motor control
- **Chunked verification** spreads CPU load over time
- **Immediate reset** on corruption prevents undefined behavior
- Verification includes firmware, configuration, and calibration data
- System reset will trigger watchdog fault code on next boot
- CRC verification much faster than read-back comparison

#### Why It's Important
Flash memory corruption can occur due to:
- Electrical noise during write operations
- Power loss during configuration save
- Hardware failures
- Cosmic ray bit flips (rare but possible)

Continuous verification catches corruption quickly before it causes:
- Motor control errors
- Safety system failures
- Unpredictable behavior

---

## Motor Control Threads

### Timer Thread

**File:** `motor/mc_interface.c:156-157, 197`
**Implementation:** See timer thread implementation in motor control files
**Priority:** `NORMALPRIO` (64)
**Execution Rate:** 100 Hz (10ms period)
**Stack Size:** 512 bytes

#### Purpose
Performs periodic motor control housekeeping tasks including statistics updates, override limit calculations, and sampling coordination.

#### Thread Configuration
```c
static THD_WORKING_AREA(timer_thread_wa, 512);
chThdCreateStatic(timer_thread_wa, sizeof(timer_thread_wa),
                  NORMALPRIO, timer_thread, NULL);
```

#### Call Flow

**Entry Point:** `timer_thread()` (in mc_interface.c)

```
timer_thread()
  ├─ chRegSetThreadName("mcif timer")
  │
  └─ for(;;)  // Infinite loop
      │
      ├─ Process motor 1
      │  ├─ run_timer_tasks(&m_motor_1)
      │  │   │
      │  │   ├─ Update override limits
      │  │   │  └─ update_override_limits(motor, &motor->m_conf)
      │  │   │      ├─ Check BMS limits
      │  │   │      ├─ Check temperature limits
      │  │   │      └─ Adjust current/speed limits dynamically
      │  │   │
      │  │   ├─ Update statistics
      │  │   │  └─ update_stats(motor)
      │  │   │      ├─ Calculate average motor current
      │  │   │      ├─ Calculate average input current
      │  │   │      ├─ Update amp-hours consumed/regenerated
      │  │   │      ├─ Update watt-hours consumed/regenerated
      │  │   │      └─ Update odometer based on RPM
      │  │   │
      │  │   ├─ Sample collection coordination
      │  │   │  └─ if (sampling active)
      │  │   │      ├─ Coordinate with ISR sampling
      │  │   │      └─ Signal sample_send_thread when buffer full
      │  │   │
      │  │   └─ Input voltage filtering
      │  │       ├─ Read ADC_VOLTS(ADC_IND_VIN_SENS)
      │  │       ├─ Apply low-pass filter (fast)
      │  │       └─ Apply low-pass filter (slow)
      │  │
      │  └─ [Motor 1 housekeeping complete]
      │
      ├─ Process motor 2 (if dual motor hardware)
      │  └─ run_timer_tasks(&m_motor_2)
      │      └─ [Same processing as motor 1]
      │
      └─ chThdSleepMilliseconds(10)  // 100 Hz rate
```

#### Key ChibiOS APIs Used
- `chRegSetThreadName()` - Register thread name
- `chThdSleepMilliseconds()` - Periodic sleep
- `chEvtSignal()` - Signal sample send thread when needed

#### Function Calls Summary
1. `update_override_limits()` - Dynamic limit adjustment
2. `update_stats()` - Statistics calculation
3. ADC voltage reading and filtering
4. Sample buffer management

#### Timing Characteristics
- **Period:** 10ms (100 Hz)
- **CPU Usage:** Low (~0.5%)
- **Non-blocking:** Yes

#### Notes
- Coordinates with ISR for sample collection
- Updates limits based on temperature and BMS
- Critical for accurate statistics tracking

---

### Fault Stop Thread

**File:** `motor/mc_interface.c:161-163, 199`
**Priority:** `HIGHPRIO - 3` (124)
**Execution Rate:** Event-driven (typically <1ms response)
**Stack Size:** 512 bytes

#### Purpose
Immediately stops motor on fault conditions. High priority ensures fast response to dangerous conditions.

#### Thread Configuration
```c
static THD_WORKING_AREA(fault_stop_thread_wa, 512);
static thread_t *fault_stop_tp;
chThdCreateStatic(fault_stop_thread_wa, sizeof(fault_stop_thread_wa),
                  HIGHPRIO - 3, fault_stop_thread, NULL);
```

#### Call Flow

**Entry Point:** `fault_stop_thread()`

```
fault_stop_thread()
  ├─ chRegSetThreadName("fault stop")
  │
  ├─ fault_stop_tp = chThdGetSelfX()
  │  └─ [Store thread pointer for signaling]
  │
  └─ for(;;)  // Infinite loop
      │
      ├─ Wait for fault event
      │  └─ chEvtWaitAny((eventmask_t)1)
      │     └─ [Blocks until signaled by fault detection code]
      │
      ├─ Retrieve fault data
      │  ├─ is_second_motor = m_fault_data.is_second_motor
      │  ├─ fault = m_fault_data.fault_code
      │  └─ info = m_fault_data.info_str / info_args
      │
      ├─ Select faulted motor
      │  └─ mc_interface_select_motor_thread(is_second_motor ? 2 : 1)
      │
      ├─ Stop motor immediately
      │  ├─ Switch on motor type
      │  │   │
      │  │   ├─ case MOTOR_TYPE_BLDC / MOTOR_TYPE_DC:
      │  │   │   └─ mcpwm_stop_pwm()
      │  │   │       ├─ Disable all PWM outputs
      │  │   │       └─ Set state to MC_STATE_OFF
      │  │   │
      │  │   └─ case MOTOR_TYPE_FOC:
      │  │       └─ mcpwm_foc_stop_pwm()
      │  │           ├─ Disable TIM1/TIM8 PWM outputs
      │  │           ├─ Set all duty cycles to 0
      │  │           └─ Set state to MC_STATE_OFF
      │  │
      │  └─ [Motor stopped, no torque]
      │
      ├─ Store fault code
      │  └─ motor->m_fault_now = fault
      │
      ├─ Notify system of fault
      │  └─ events_add_fault(fault, is_second_motor, info_str, info_args)
      │
      └─ Send fault notification to host
          └─ commands_send_fault(fault, is_second_motor, info_str, info_args)
             ├─ Serialize fault data
             └─ Send via USB/UART/CAN
```

#### Fault Detection and Signaling

**How faults trigger this thread:**
```c
// From ISR or other thread context:
mc_interface_fault_stop(fault_code, is_second_motor, info)
  ├─ Store fault data in m_fault_data
  ├─ chSysLockFromISR()  // If in ISR
  ├─ chEvtSignalI(fault_stop_tp, (eventmask_t)1)
  │  └─ [Wakes up fault_stop_thread immediately]
  └─ chSysUnlockFromISR()
```

#### Key ChibiOS APIs Used
- `chRegSetThreadName()` - Register thread name
- `chThdGetSelfX()` - Get own thread pointer
- `chEvtWaitAny()` - Wait for fault event (blocking)
- `chEvtSignalI()` - Signaled from ISR/thread when fault occurs
- `mc_interface_select_motor_thread()` - Select faulted motor

#### Function Calls Summary
1. `mcpwm_stop_pwm()` / `mcpwm_foc_stop_pwm()` - Stop motor immediately
2. `events_add_fault()` - Log fault event
3. `commands_send_fault()` - Notify host

#### Fault Types Handled
- `FAULT_CODE_OVER_VOLTAGE` - Battery overvoltage
- `FAULT_CODE_UNDER_VOLTAGE` - Battery undervoltage
- `FAULT_CODE_DRV` - Gate driver fault
- `FAULT_CODE_ABS_OVER_CURRENT` - Absolute overcurrent
- `FAULT_CODE_OVER_TEMP_FET` - FET overtemperature
- `FAULT_CODE_OVER_TEMP_MOTOR` - Motor overtemperature
- `FAULT_CODE_GATE_DRIVER_OVER_VOLTAGE`
- `FAULT_CODE_GATE_DRIVER_UNDER_VOLTAGE`
- And 20+ other fault conditions

#### Timing Characteristics
- **Response Time:** <1ms from fault detection to motor stop
- **Priority:** HIGHPRIO - 3 (124) - very high priority
- **CPU Usage:** Near zero when idle, <0.1ms when triggered
- **Event-driven:** Only runs when fault occurs

#### Notes
- **High priority** ensures fast response to dangerous conditions
- **Event-driven** design means zero CPU usage when no faults
- Motor stopped **before** host notification (safety first)
- Can be triggered from ISR or thread context
- Thread-safe fault data storage
- Supports dual motor systems

---

### Statistics Thread

**File:** `motor/mc_interface.c:164-165, 200`
**Priority:** `NORMALPRIO` (64)
**Execution Rate:** 1 Hz (1000ms period)
**Stack Size:** 512 bytes

#### Purpose
Periodically saves statistics (amp-hours, watt-hours, odometer) to flash memory for persistence across power cycles.

#### Thread Configuration
```c
static THD_WORKING_AREA(stat_thread_wa, 512);
chThdCreateStatic(stat_thread_wa, sizeof(stat_thread_wa),
                  NORMALPRIO, stat_thread, NULL);
```

#### Call Flow

**Entry Point:** `stat_thread()`

```
stat_thread()
  ├─ chRegSetThreadName("stat")
  │
  └─ for(;;)  // Infinite loop
      │
      ├─ Check if statistics need saving
      │  ├─ motor1_needs_save = (m_motor_1.m_amp_seconds != m_motor_1.m_amp_seconds_last)
      │  │                    || (m_motor_1.m_watt_seconds != m_motor_1.m_watt_seconds_last)
      │  │                    || (odometer changed significantly)
      │  │
      │  └─ motor2_needs_save = [same check for motor 2]
      │
      ├─ Save motor 1 statistics (if changed)
      │  └─ if (motor1_needs_save)
      │      │
      │      ├─ Prepare statistics structure
      │      │  ├─ stats.amp_hours = m_motor_1.m_amp_seconds / 3600.0
      │      │  ├─ stats.amp_hours_charged = m_motor_1.m_amp_seconds_charged / 3600.0
      │      │  ├─ stats.watt_hours = m_motor_1.m_watt_seconds / 3600.0
      │      │  ├─ stats.watt_hours_charged = m_motor_1.m_watt_seconds_charged / 3600.0
      │      │  ├─ stats.odometer_meters = calculated from RPM integration
      │      │  └─ stats.runtime_seconds = calculated from active time
      │      │
      │      ├─ Write to flash memory
      │      │  └─ conf_general_store_backup_data(&stats, motor_1)
      │      │      ├─ Erase flash sector if needed
      │      │      ├─ Write statistics structure
      │      │      ├─ Calculate and store CRC
      │      │      └─ [Flash write takes 5-50ms depending on erase]
      │      │
      │      └─ Update last saved values
      │          ├─ m_motor_1.m_amp_seconds_last = m_motor_1.m_amp_seconds
      │          └─ [etc for other values]
      │
      ├─ Save motor 2 statistics (if dual motor and changed)
      │  └─ [Same process as motor 1]
      │
      └─ chThdSleepMilliseconds(1000)  // 1 Hz rate
```

#### Key ChibiOS APIs Used
- `chRegSetThreadName()` - Register thread name
- `chThdSleepMilliseconds()` - Periodic sleep (1 second)

#### Function Calls Summary
1. `conf_general_store_backup_data()` - Write statistics to flash
2. Flash erase/write operations (in flash_helper.c)
3. CRC calculation for data integrity

#### Statistics Tracked
- **Amp-hours consumed** - Total energy from battery
- **Amp-hours regenerated** - Total energy to battery (braking)
- **Watt-hours consumed** - Total power consumed
- **Watt-hours regenerated** - Total power regenerated
- **Odometer** - Total distance traveled (calculated from RPM + wheel radius)
- **Runtime** - Total active time

#### Flash Write Strategy
- **Write-on-change** - Only writes when values change
- **Sector management** - Uses dedicated flash sectors
- **CRC protection** - Detects corrupted data on boot
- **Wear leveling** - Rotates through multiple flash sectors
- **Erase minimization** - Groups multiple changes

#### Timing Characteristics
- **Check Period:** 1 second (1 Hz)
- **Write Time:** 5-50ms when flash erase needed
- **CPU Usage:** Very low (~0.1% average)
- **Non-blocking:** Yes (other threads continue during flash write)

#### Notes
- **1 Hz rate** balances write frequency vs flash wear
- Statistics persist across power cycles
- Flash has limited write cycles (~10,000-100,000)
- Writing every second allows ~3 months continuous operation per sector
- Wear leveling extends flash life significantly
- Critical for tracking battery state of charge
- Used by VESC Tool to display total mileage/energy

---

### Sample Send Thread

**File:** `motor/mc_interface.c:158-160, 198`
**Priority:** `NORMALPRIO - 1` (63)
**Execution Rate:** Event-driven (when sample buffer full)
**Stack Size:** 512 bytes

#### Purpose
Sends ADC sample buffers to host for real-time oscilloscope display in VESC Tool. Event-driven to minimize CPU usage.

#### Thread Configuration
```c
static THD_WORKING_AREA(sample_send_thread_wa, 512);
static thread_t *sample_send_tp;
chThdCreateStatic(sample_send_thread_wa, sizeof(sample_send_thread_wa),
                  NORMALPRIO - 1, sample_send_thread, NULL);
```

#### Call Flow

**Entry Point:** `sample_send_thread()`

```
sample_send_thread()
  ├─ chRegSetThreadName("sample send")
  │
  ├─ sample_send_tp = chThdGetSelfX()
  │  └─ [Store thread pointer for signaling]
  │
  └─ for(;;)  // Infinite loop
      │
      ├─ Wait for sample buffer ready event
      │  └─ chEvtWaitAny((eventmask_t)1)
      │     └─ [Blocks until signaled by timer_thread or ISR]
      │
      ├─ Check if sampling is active
      │  └─ if (m_sample_mode == DEBUG_SAMPLING_OFF)
      │      └─ continue  // Skip if disabled
      │
      ├─ Determine sample type and buffer
      │  ├─ switch (m_sample_mode)
      │  │   ├─ DEBUG_SAMPLING_NOW:      // Single trigger
      │  │   ├─ DEBUG_SAMPLING_START:    // Continuous
      │  │   ├─ DEBUG_SAMPLING_TRIGGER:  // Trigger on condition
      │  │   └─ [Multiple sample types supported]
      │  │
      │  └─ Select appropriate buffer:
      │      ├─ m_curr0_samples[]  // Phase current A
      │      ├─ m_curr1_samples[]  // Phase current B
      │      ├─ m_curr2_samples[]  // Phase current C
      │      ├─ m_ph1_samples[]    // Phase voltage A
      │      ├─ m_ph2_samples[]    // Phase voltage B
      │      ├─ m_ph3_samples[]    // Phase voltage C
      │      ├─ m_vzero_samples[]  // Zero vector voltage
      │      ├─ m_status_samples[] // Motor state
      │      ├─ m_curr_fir_samples[] // Filtered current
      │      └─ m_f_sw_samples[]   // Switching frequency
      │
      ├─ Send sample blocks to host
      │  ├─ Calculate number of blocks needed
      │  │  └─ blocks = (m_sample_len + SAMPLES_PER_BLOCK - 1) / SAMPLES_PER_BLOCK
      │  │
      │  └─ for (block = 0; block < blocks; block++)
      │      │
      │      ├─ Prepare packet
      │      │  ├─ send_sample_block(block, offset)
      │      │  │   │
      │      │  │   ├─ Create packet header
      │      │  │   │  ├─ packet[0] = COMM_SAMPLE_PRINT
      │      │  │   │  ├─ packet[1] = block index
      │      │  │   │  ├─ packet[2] = total blocks
      │      │  │   │  └─ packet[3] = sample mode
      │      │  │   │
      │      │  │   ├─ Serialize sample data
      │      │  │   │  └─ for each sample in block
      │      │  │   │      ├─ Convert int16 to bytes
      │      │  │   │      └─ Add to packet
      │      │  │   │
      │      │  │   └─ Send packet
      │      │  │       └─ send_func_sample(packet, length)
      │      │  │           ├─ Write to USB/UART/CAN
      │      │  │           └─ [Actual transport handled by comm layer]
      │      │  │
      │      │  └─ [Block sent]
      │      │
      │      └─ Small delay between blocks
      │          └─ chThdSleepMilliseconds(1)
      │             └─ [Prevents overwhelming host USB/UART]
      │
      ├─ Stop sampling if single-shot mode
      │  └─ if (m_sample_mode == DEBUG_SAMPLING_NOW)
      │      └─ m_sample_mode = DEBUG_SAMPLING_OFF
      │
      └─ [Ready for next sample buffer]
```

#### Sample Collection (ISR Side)

**How samples are collected (from FOC ISR):**
```c
// In mcpwm_foc_adc_int_handler() - runs at 10-40 kHz
if (m_sample_mode != DEBUG_SAMPLING_OFF) {
    if (m_sample_now >= m_sample_offset_last) {
        m_curr0_samples[m_sample_trigger] = motor->m_currents.ia_raw;
        m_curr1_samples[m_sample_trigger] = motor->m_currents.ib_raw;
        m_curr2_samples[m_sample_trigger] = motor->m_currents.ic_raw;
        m_ph1_samples[m_sample_trigger] = motor->m_phase_a_voltage;
        // ... etc for other signals

        m_sample_trigger++;

        if (m_sample_trigger >= m_sample_len) {
            m_sample_trigger = 0;
            // Signal sample_send_thread
            chSysLockFromISR();
            chEvtSignalI(sample_send_tp, (eventmask_t)1);
            chSysUnlockFromISR();
        }
    }
    m_sample_now++;
}
```

#### Key ChibiOS APIs Used
- `chRegSetThreadName()` - Register thread name
- `chThdGetSelfX()` - Get thread pointer
- `chEvtWaitAny()` - Wait for buffer ready (blocking)
- `chEvtSignalI()` - Signaled from ISR when buffer full
- `chThdSleepMilliseconds()` - Delay between blocks

#### Function Calls Summary
1. `send_sample_block()` - Serialize and send sample block
2. `send_func_sample()` - Transport layer (USB/UART/CAN)

#### Sample Modes
- `DEBUG_SAMPLING_NOW` - Single capture, then stop
- `DEBUG_SAMPLING_START` - Continuous capture
- `DEBUG_SAMPLING_TRIGGER` - Trigger when condition met
- `DEBUG_SAMPLING_TRIGGER_START` - Trigger start of buffer
- `DEBUG_SAMPLING_TRIGGER_FAULT` - Trigger on fault

#### Buffer Sizes
- Default: 1000 samples per buffer
- Configurable up to ADC_SAMPLE_MAX_LEN
- At 20 kHz ISR: 1000 samples = 50ms window
- At 40 kHz ISR: 1000 samples = 25ms window

#### Timing Characteristics
- **Event-driven** - Only runs when buffer full
- **Sample rate:** Matches ISR frequency (10-40 kHz)
- **Send time:** ~10-50ms for 1000 samples
- **CPU Usage:** Low when active (~5%), zero when idle
- **Priority:** NORMALPRIO - 1 (63) - slightly below normal

#### Notes
- Critical for debugging motor control issues
- Provides oscilloscope view in VESC Tool
- Can capture transient events with trigger modes
- Multiple signals captured simultaneously
- Time-aligned with motor control ISR
- 1ms delay between blocks prevents USB saturation
- Event-driven design minimizes CPU usage

---

## Safety Thread

### Timeout Thread

**File:** `timeout.c:192-276`
**Created at:** `timeout.c:83`
**Priority:** `NORMALPRIO` (64)
**Execution Rate:** 100 Hz (10ms period)
**Stack Size:** 256 bytes

#### Purpose
Provides safety watchdog functionality by monitoring for communication timeouts, kill switch activation, and thread health. Automatically applies brake current when timeout occurs.

#### Thread Configuration
```c
static THD_WORKING_AREA(timeout_thread_wa, 256);
chThdCreateStatic(timeout_thread_wa, sizeof(timeout_thread_wa),
                  NORMALPRIO, timeout_thread, NULL);
```

#### Call Flow

**Entry Point:** `timeout_thread()`

```
timeout_thread()
  ├─ chRegSetThreadName("Timeout")
  │
  └─ for(;;)  // Infinite loop
      │
      ├─ Check kill switch based on mode
      │  ├─ switch (timeout_kill_sw_mode)
      │  │   │
      │  │   ├─ case KILL_SW_MODE_PPM_LOW:
      │  │   │   └─ kill_sw = !palReadPad(HW_ICU_GPIO, HW_ICU_PIN)
      │  │   │       └─ [Read PPM input pin, active low]
      │  │   │
      │  │   ├─ case KILL_SW_MODE_PPM_HIGH:
      │  │   │   └─ kill_sw = palReadPad(HW_ICU_GPIO, HW_ICU_PIN)
      │  │   │       └─ [Read PPM input pin, active high]
      │  │   │
      │  │   ├─ case KILL_SW_MODE_ADC2_LOW:
      │  │   │   └─ kill_sw = ADC_VOLTS(ADC_IND_EXT2) < 1.65
      │  │   │       └─ [Read ADC2, trigger below 1.65V]
      │  │   │
      │  │   ├─ case KILL_SW_MODE_ADC2_HIGH:
      │  │   │   └─ kill_sw = ADC_VOLTS(ADC_IND_EXT2) > 1.65
      │  │   │       └─ [Read ADC2, trigger above 1.65V]
      │  │   │
      │  │   └─ default:
      │  │       └─ kill_sw = false
      │  │
      │  └─ Check external kill switch override
      │      └─ if (kill_sw_ext_set)
      │          └─ kill_sw = true
      │             └─ [Can be set via commands or LispBM]
      │
      ├─ Check for timeout condition
      │  ├─ timeout_elapsed = chVTTimeElapsedSinceX(last_update_time) > MS2ST(timeout_msec)
      │  └─ timeout_active = (timeout_msec != 0) && timeout_elapsed
      │
      ├─ Handle timeout or kill switch activation
      │  └─ if (kill_sw || timeout_active)
      │      │
      │      ├─ Release motor override (first occurrence only)
      │      │  └─ if (!has_timeout && !kill_sw_active)
      │      │      └─ mc_interface_release_motor_override()
      │      │         └─ [Clear any manual motor control override]
      │      │
      │      ├─ Unlock motor control
      │      │  └─ mc_interface_unlock()
      │      │     └─ [Allow brake command even if locked]
      │      │
      │      ├─ Apply brake to motor 1
      │      │  ├─ mc_interface_select_motor_thread(1)
      │      │  └─ mc_interface_set_brake_current(timeout_brake_current)
      │      │
      │      ├─ Apply brake to motor 2 (if dual motor)
      │      │  ├─ mc_interface_select_motor_thread(2)
      │      │  └─ mc_interface_set_brake_current(timeout_brake_current)
      │      │
      │      ├─ Handle kill switch vs timeout differently
      │      │  ├─ if (kill_sw)
      │      │  │   └─ mc_interface_ignore_input_both(20)
      │      │  │       └─ [Ignore all inputs for 20 iterations (200ms)]
      │      │  │           [Prevents immediate restart]
      │      │  └─ else
      │      │      └─ has_timeout = true
      │      │         └─ [Flag timeout condition]
      │      │
      │      └─ [Motors now braking]
      │
      ├─ Clear timeout flag when recovered
      │  └─ else
      │      └─ has_timeout = false
      │
      ├─ Update kill switch status
      │  └─ kill_sw_active = kill_sw
      │
      ├─ Monitor critical thread health (watchdog feeding)
      │  ├─ bool threads_ok = true
      │  │
      │  ├─ Check FOC thread health
      │  │  └─ if (feed_counter[THREAD_MCPWM] < MIN_THREAD_ITERATIONS)
      │  │      └─ threads_ok = false
      │  │         └─ [FOC ISR must report at least 1 iteration per 10ms]
      │  │
      │  ├─ Check CAN thread health (if enabled)
      │  │  └─ if (feed_counter[THREAD_CANBUS] < MIN_THREAD_ITERATIONS)
      │  │      └─ threads_ok = false
      │  │         └─ [CAN RX must process at least 1 message per 10ms]
      │  │
      │  └─ Reset feed counters for next iteration
      │      └─ for (i = 0; i < MAX_THREADS_MONITOR; i++)
      │          └─ feed_counter[i] = 0
      │
      ├─ Feed or starve watchdog based on thread health
      │  ├─ if (threads_ok == true)
      │  │   └─ IWDG_ReloadCounter()
      │  │       ├─ [Reload independent watchdog timer]
      │  │       └─ [Must be called every 12ms max]
      │  │
      │  └─ else
      │      └─ [Don't feed watchdog]
      │         ├─ [Watchdog will trigger reset after 12ms]
      │         └─ [System will boot with FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET]
      │
      └─ chThdSleepMilliseconds(10)  // 100 Hz rate
```

#### Thread Health Monitoring

**How threads report health:**
```c
// From FOC ISR (runs at 10-40 kHz):
void mcpwm_foc_adc_int_handler(void) {
    // ... FOC algorithm execution ...
    timeout_feed_WDT(THREAD_MCPWM);  // Increment feed counter
}

// From CAN RX thread:
while (msg_available) {
    // ... process CAN message ...
    timeout_feed_WDT(THREAD_CANBUS);  // Increment feed counter
}
```

#### Timeout Reset

**How timeout is reset (from application threads):**
```c
// PPM thread resets timeout on valid pulse:
static void servodec_func(void) {
    timeout_reset();  // Updates last_update_time
}

// ADC thread resets on valid input
// Commands reset on valid packet received
```

#### Independent Watchdog (IWDG) Configuration

**Initialization (from timeout_init):**
```c
timeout_init()
  ├─ Configure IWDG
  │  ├─ IWDG_SetPrescaler(IWDG_Prescaler_4)
  │  │   └─ [LSI clock / 4]
  │  │
  │  ├─ IWDG_SetReload(140)
  │  │   └─ [12ms timeout @ 47kHz LSI]
  │  │       [33ms timeout @ 17kHz LSI]
  │  │       [Accounts for LSI frequency variation]
  │  │
  │  └─ IWDG_Enable()
  │      └─ [Watchdog now running, must be fed every 12ms]
  │
  └─ Create timeout thread
```

#### Key ChibiOS APIs Used
- `chRegSetThreadName()` - Register thread name
- `chVTGetSystemTimeX()` - Get current system time
- `chVTTimeElapsedSinceX()` - Calculate elapsed time
- `MS2ST()` - Convert milliseconds to system ticks
- `chThdSleepMilliseconds()` - Periodic sleep
- `palReadPad()` - Read GPIO pin for kill switch

#### Function Calls Summary
1. `palReadPad()` - Read kill switch GPIO
2. `ADC_VOLTS()` - Read kill switch ADC
3. `chVTTimeElapsedSinceX()` - Check timeout elapsed
4. `mc_interface_release_motor_override()` - Clear overrides
5. `mc_interface_unlock()` - Unlock control
6. `mc_interface_select_motor_thread()` - Select motor
7. `mc_interface_set_brake_current()` - Apply brake
8. `mc_interface_ignore_input_both()` - Ignore inputs
9. `IWDG_ReloadCounter()` - Feed watchdog

#### Kill Switch Modes
- `KILL_SW_MODE_DISABLED` - No kill switch
- `KILL_SW_MODE_PPM_LOW` - PPM pin low = kill
- `KILL_SW_MODE_PPM_HIGH` - PPM pin high = kill
- `KILL_SW_MODE_ADC2_LOW` - ADC2 < 1.65V = kill
- `KILL_SW_MODE_ADC2_HIGH` - ADC2 > 1.65V = kill

#### Timing Characteristics
- **Period:** 10ms (100 Hz)
- **Watchdog Timeout:** 12ms minimum, 33ms maximum
- **Timeout Resolution:** 10ms (thread period)
- **CPU Usage:** Minimal (~0.1%)
- **Response Time:** <10ms to timeout/kill switch

#### Critical Safety Features

**1. Dual Safety Mechanisms:**
- **Software Timeout:** Detects communication loss
- **Hardware Watchdog:** Detects software hang/crash

**2. Thread Health Monitoring:**
- Ensures critical threads (FOC ISR, CAN) are running
- Automatic reset if thread stops responding
- Prevents silent failures

**3. Kill Switch:**
- Hardware-level emergency stop
- Multiple input modes (GPIO, ADC)
- Ignores all inputs for 200ms after activation

**4. Graceful Degradation:**
- Applies configurable brake current (not full brake)
- Can be configured for zero brake (coast to stop)
- Smooth deceleration prevents mechanical stress

#### Notes
- **Watchdog must be fed every 12ms** - timeout thread runs at 10ms
- Thread health monitoring prevents silent failures
- Kill switch has higher priority than timeout
- Brake current is configurable (0-max)
- Can trigger from multiple sources:
  - Communication timeout (PPM/ADC/UART/CAN)
  - Kill switch activation
  - Thread failure (automatic reset)
- External kill switch can be set via commands/LispBM
- Used for:
  - Emergency stop
  - Range limiting (geofencing)
  - Remote shutdown
  - Failsafe operation

---

## Application Threads

Application threads implement different control input methods (RC, throttle, pedal assist, Wii controller). They all follow a similar pattern: read input → process → send motor command.

### PPM Thread

**File:** `applications/app_ppm.c:107-564`
**Created at:** `app_ppm.c:72` via `app_ppm_start()`
**Priority:** `NORMALPRIO` (64)
**Execution Rate:** Event-driven + 100 Hz timeout check
**Stack Size:** 512 bytes (in RAM4 for performance)

#### Purpose
Processes RC receiver PPM (Pulse Position Modulation) signals for motor control. Supports multiple control modes including current, duty cycle, speed, and position control.

#### Thread Configuration
```c
__attribute__((section(".ram4"))) static THD_WORKING_AREA(ppm_thread_wa, 512);
static thread_t *ppm_tp;
chThdCreateStatic(ppm_thread_wa, sizeof(ppm_thread_wa),
                  NORMALPRIO, ppm_thread, NULL);
```

#### Call Flow

**Entry Point:** `ppm_thread()`

```
ppm_thread()
  ├─ chRegSetThreadName("APP_PPM")
  │
  ├─ ppm_tp = chThdGetSelfX()
  │  └─ [Store thread pointer for ISR signaling]
  │
  ├─ Initialize servo decoder
  │  ├─ servodec_set_pulse_options(pulse_start, pulse_end, median_filter)
  │  │  └─ [Configure: 1ms-2ms typical, median filter for noise]
  │  │
  │  └─ servodec_init(servodec_func)
  │      ├─ [Setup timer input capture on PPM pin]
  │      └─ [Register ISR callback: servodec_func]
  │
  ├─ is_running = true
  │
  └─ for(;;)  // Main loop
      │
      ├─ Wait for PPM pulse or timeout
      │  └─ chEvtWaitAnyTimeout((eventmask_t)1, MS2ST(2))
      │      ├─ [Blocks until signaled by servodec ISR]
      │      └─ [Or 2ms timeout (500 Hz max check rate)]
      │
      ├─ Check for stop request
      │  └─ if (stop_now)
      │      └─ return  // Exit thread
      │
      ├─ Handle PPM pulse received
      │  └─ if (ppm_rx)
      │      ├─ ppm_rx = false
      │      └─ timeout_reset()
      │         └─ [Reset watchdog - valid signal received]
      │
      ├─ Read motor configuration
      │  ├─ mcconf = mc_interface_get_configuration()
      │  └─ rpm_now = mc_interface_get_rpm()
      │
      ├─ Read PPM servo value
      │  ├─ servo_val = servodec_get_servo(0)
      │  │   ├─ [Returns -1.0 to +1.0]
      │  │   └─ [Based on pulse width: 1ms=-1, 1.5ms=0, 2ms=+1]
      │  │
      │  └─ if (ppm_detached)
      │      └─ servo_val = ppm_override
      │         └─ [LispBM can override PPM input]
      │
      ├─ Convert to milliseconds for mode-specific mapping
      │  └─ servo_ms = utils_map(servo_val, -1.0, 1.0, pulse_start, pulse_end)
      │
      ├─ Map pulse width based on control type
      │  └─ switch (ctrl_type)
      │      │
      │      ├─ case PPM_CTRL_TYPE_CURRENT_NOREV:
      │      ├─ case PPM_CTRL_TYPE_DUTY_NOREV:
      │      ├─ case PPM_CTRL_TYPE_PID_NOREV:
      │      ├─ case PPM_CTRL_TYPE_PID_POSITION_360:
      │      │   ├─ input_val = servo_val
      │      │   ├─ servo_val += 1.0  // Convert [-1,1] to [0,2]
      │      │   └─ servo_val /= 2.0  // Then to [0,1]
      │      │       └─ [No reverse - full range is forward only]
      │      │
      │      └─ default:  // Bidirectional modes
      │          ├─ if (servo_ms < pulse_center)
      │          │   └─ servo_val = map(servo_ms, pulse_start, pulse_center, -1.0, 0.0)
      │          │       └─ [Below center = reverse/brake]
      │          └─ else
      │              └─ servo_val = map(servo_ms, pulse_center, pulse_end, 0.0, 1.0)
      │                  └─ [Above center = forward]
      │
      ├─ Check if output is disabled
      │  └─ if (app_is_output_disabled())
      │      └─ continue  // Skip motor control but keep reading input
      │
      ├─ Handle timeout condition
      │  └─ if (timeout_has_timeout() || servodec_get_time_since_update() > timeout)
      │      ├─ pulses_without_power = 0
      │      ├─ servoError = true
      │      ├─ timeoutCurrent = timeout_get_brake_current()
      │      │
      │      ├─ Apply brake to local motor
      │      │  └─ mc_interface_set_brake_current(timeoutCurrent)
      │      │
      │      ├─ Apply brake to CAN motors (if multi-ESC)
      │      │  └─ if (config.multi_esc)
      │      │      └─ for each CAN motor
      │      │          └─ comm_can_set_current_brake(id, timeoutCurrent)
      │      │
      │      └─ continue  // Skip normal processing
      │
      ├─ Handle fault condition (if safe start enabled)
      │  └─ if (mc_interface_get_fault() != FAULT_CODE_NONE && config.safe_start)
      │      └─ pulses_without_power = 0
      │         └─ [Reset safe start counter]
      │
      ├─ Apply deadband
      │  └─ utils_deadband(&servo_val, config.hyst, 1.0)
      │      └─ [Creates "dead zone" around neutral]
      │          [Prevents drift from slightly off-center stick]
      │
      ├─ Apply throttle curve
      │  └─ servo_val = utils_throttle_curve(servo_val, exp, exp_brake, mode)
      │      ├─ [Exponential curve for finer control at low throttle]
      │      ├─ exp=0.0: Linear
      │      ├─ exp>0.0: More power at low end
      │      └─ exp<0.0: Less power at low end (smoother)
      │
      ├─ Apply ramping (acceleration/deceleration limiting)
      │  ├─ dt = ST2MS(chVTTimeElapsedSinceX(last_time)) / 1000.0
      │  ├─ last_time = chVTGetSystemTimeX()
      │  │
      │  ├─ Select ramp time based on direction
      │  │  └─ ramp_time = (|servo_val| > |servo_val_ramp|)
      │  │                  ? ramp_time_pos  // Accelerating
      │  │                  : ramp_time_neg  // Decelerating
      │  │
      │  └─ if (ramp_time > 0.01)
      │      ├─ ramp_step = dt / ramp_time
      │      ├─ utils_step_towards(&servo_val_ramp, servo_val, ramp_step)
      │      └─ servo_val = servo_val_ramp
      │         └─ [Smooth transition limited by ramp time]
      │
      ├─ Process control type and send motor command
      │  └─ switch (config.ctrl_type)
      │      │
      │      ├─ case PPM_CTRL_TYPE_CURRENT:
      │      ├─ case PPM_CTRL_TYPE_CURRENT_NOREV:
      │      │   ├─ current_mode = true
      │      │   │
      │      │   ├─ Calculate current based on direction
      │      │   │  └─ if ((servo_val >= 0 && rpm_now >= 0) || (servo_val < 0 && rpm_now <= 0))
      │      │   │      └─ current = servo_val * mcconf->lo_current_max  // Accelerate
      │      │   │      else
      │      │   │          └─ current = servo_val * |mcconf->lo_current_min|  // Brake
      │      │   │
      │      │   └─ if (|servo_val| < 0.001)
      │      │       └─ pulses_without_power++
      │      │          └─ [Count neutral position for safe start]
      │      │
      │      ├─ case PPM_CTRL_TYPE_CURRENT_BRAKE_REV_HYST:
      │      │   ├─ [Complex mode with direction hysteresis]
      │      │   ├─ Implements soft reverse with brake zone
      │      │   ├─ force_brake flag prevents instant direction change
      │      │   ├─ Requires stopping below threshold RPM before reverse
      │      │   └─ idle_once counter ensures full stop before direction change
      │      │
      │      ├─ case PPM_CTRL_TYPE_DUTY:
      │      ├─ case PPM_CTRL_TYPE_DUTY_NOREV:
      │      │   ├─ if (|servo_val| < 0.001)
      │      │   │   └─ pulses_without_power++
      │      │   │
      │      │   └─ if (!(pulses_without_power < MIN && safe_start))
      │      │       ├─ duty = map(servo_val, -1.0, 1.0, -max_duty, max_duty)
      │      │       ├─ mc_interface_set_duty(duty)
      │      │       └─ send_duty = true
      │      │
      │      ├─ case PPM_CTRL_TYPE_PID:
      │      ├─ case PPM_CTRL_TYPE_PID_NOREV:
      │      │   ├─ if (|servo_val| < 0.001)
      │      │   │   └─ pulses_without_power++
      │      │   │
      │      │   └─ if (!(pulses_without_power < MIN && safe_start))
      │      │       ├─ rpm = servo_val * config.pid_max_erpm
      │      │       ├─ mc_interface_set_pid_speed(rpm)
      │      │       └─ send_current = true
      │      │
      │      ├─ case PPM_CTRL_TYPE_PID_POSITION_180:
      │      ├─ case PPM_CTRL_TYPE_PID_POSITION_360:
      │      │   ├─ if (|servo_val| < 0.02)
      │      │   │   └─ pulses_without_power++
      │      │   │
      │      │   ├─ Calculate target angle
      │      │   │  ├─ if (POSITION_180): angle = servo_val * 180  // -180 to +180
      │      │   │  └─ if (POSITION_360): angle = servo_val * 360  // 0 to +360
      │      │   │
      │      │   └─ Smart safe start for position mode
      │      │       ├─ if (not in position mode yet)
      │      │       │   └─ if (|angle - current_pos| < 10°)
      │      │       │       └─ mc_interface_set_pid_pos(angle)
      │      │       │          └─ [Enter position mode only when close]
      │      │       └─ else
      │      │           └─ mc_interface_set_pid_pos(angle)
      │      │
      │      └─ case PPM_CTRL_TYPE_CURRENT_SMART_REV:
      │           ├─ [Advanced reverse with automatic duty mode]
      │           ├─ Switches to duty control for smooth reverse
      │           └─ Uses traction control for multiple motors
      │
      ├─ Apply safe start logic
      │  └─ if (pulses_without_power < MIN_PULSES && safe_start)
      │      ├─ if (pulses_without_power == pulses_without_power_before)
      │      │   └─ pulses_without_power = 0  // Reset if stuck
      │      │
      │      ├─ if (servoError)
      │      │   └─ continue  // Don't drive motor
      │      │
      │      └─ if (current_mode)
      │          └─ current = 0.0  // Zero current during safe start
      │
      ├─ Handle multi-ESC traction control and synchronization
      │  └─ if (config.multi_esc)
      │      ├─ Collect RPM from all ESCs via CAN
      │      ├─ Find highest and lowest RPM
      │      ├─ Apply traction control algorithm
      │      │  └─ Reduce current to faster motors
      │      └─ Send commands to slave ESCs
      │
      └─ Send final motor command
          └─ if (current_mode)
              ├─ if (current_mode_brake)
              │   ├─ mc_interface_set_brake_current(|current|)
              │   └─ [Multi-ESC: comm_can_set_current_brake_rel(id, |servo_val|)]
              │
              └─ else
                  ├─ mc_interface_set_current(current_out)
                  └─ [Multi-ESC: comm_can_set_current_rel(id, servo_val)]
                      └─ [With traction control if enabled]
```

#### ISR Callback (Servo Decoder)

**Triggered on each PPM pulse edge:**
```c
static void servodec_func(void) {
    ppm_rx = true;  // Set flag
    chSysLockFromISR();
    chEvtSignalI(ppm_tp, (eventmask_t)1);  // Wake thread
    chSysUnlockFromISR();
}
```

#### Key ChibiOS APIs Used
- `chRegSetThreadName()` - Register thread name
- `chThdGetSelfX()` - Get thread pointer for ISR signaling
- `chEvtWaitAnyTimeout()` - Event-driven with timeout
- `chEvtSignalI()` - Signaled from servo decoder ISR
- `chVTGetSystemTimeX()` - Timestamp for ramping
- `chVTTimeElapsedSinceX()` - Delta time calculation

#### Function Calls Summary
1. `servodec_set_pulse_options()` - Configure pulse width range
2. `servodec_init()` - Initialize timer input capture
3. `servodec_get_servo()` - Read decoded servo value
4. `servodec_get_time_since_update()` - Check for signal loss
5. `timeout_reset()` - Reset watchdog on valid pulse
6. `mc_interface_get_configuration()` - Read motor config
7. `mc_interface_get_rpm()` - Read current RPM
8. `app_is_output_disabled()` - Check if output disabled
9. `timeout_has_timeout()` - Check timeout status
10. `mc_interface_get_fault()` - Check fault status
11. `utils_deadband()` - Apply dead zone
12. `utils_throttle_curve()` - Apply exponential curve
13. `utils_step_towards()` - Ramping function
14. `mc_interface_set_current()` - Send current command
15. `mc_interface_set_brake_current()` - Send brake command
16. `mc_interface_set_duty()` - Send duty cycle command
17. `mc_interface_set_pid_speed()` - Send speed command
18. `mc_interface_set_pid_pos()` - Send position command
19. `comm_can_set_current()` / `_brake()` / `_duty()` - Multi-ESC commands

#### Control Modes Supported
- `PPM_CTRL_TYPE_CURRENT` - Current control, bidirectional
- `PPM_CTRL_TYPE_CURRENT_NOREV` - Current control, forward only
- `PPM_CTRL_TYPE_CURRENT_NOREV_BRAKE` - Forward + brake
- `PPM_CTRL_TYPE_CURRENT_SMART_REV` - Smart reverse with duty mode
- `PPM_CTRL_TYPE_CURRENT_BRAKE_REV_HYST` - Reverse with hysteresis
- `PPM_CTRL_TYPE_DUTY` - Duty cycle control
- `PPM_CTRL_TYPE_DUTY_NOREV` - Duty cycle, forward only
- `PPM_CTRL_TYPE_PID` - Speed (RPM) control
- `PPM_CTRL_TYPE_PID_NOREV` - Speed control, forward only
- `PPM_CTRL_TYPE_PID_POSITION_180` - Position control ±180°
- `PPM_CTRL_TYPE_PID_POSITION_360` - Position control 0-360°

#### Timing Characteristics
- **Event-driven:** Triggered by PPM pulses (typically 50 Hz from RC receiver)
- **Timeout check:** 2ms (500 Hz maximum)
- **Typical execution:** 50-100 Hz (matches RC receiver update rate)
- **CPU Usage:** Low (~0.5% at 50 Hz)
- **Response latency:** <2ms from pulse to motor command

#### PPM Signal Characteristics
- **Standard RC:** 1000-2000 µs pulse width, 20ms period (50 Hz)
- **Center:** 1500 µs = neutral
- **Configurable range:** Can adjust min/max/center
- **Median filter:** Reduces noise from poor quality receivers

#### Safety Features
- **Safe start:** Requires neutral stick for 50 pulses (~1 second) before enabling
- **Timeout:** Automatic brake on signal loss
- **Deadband:** Prevents drift from stick not perfectly centered
- **Ramping:** Prevents sudden acceleration/deceleration
- **Direction hysteresis:** Prevents accidental reverse (some modes)

#### Multi-ESC Features
- **Synchronized control:** All ESCs receive same command
- **Traction control:** Reduces wheel slip
- **CAN communication:** Commands sent to slave ESCs
- **Health monitoring:** Only controls ESCs with recent CAN updates (<100ms)

#### Notes
- Most common input method for electric skateboards and RC vehicles
- RAM4 placement for faster access (time-critical)
- Event-driven design minimizes CPU usage
- Extensive configurability for different use cases
- Supports both hobby RC and custom PPM sources

