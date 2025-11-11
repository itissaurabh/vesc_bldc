# Motor Control Interface Documentation

**File:** `motor/mc_interface.c` (3030 lines)
**Purpose:** High-level abstraction layer for motor control, providing a unified API for both BLDC and FOC motor types. This is the primary interface used by applications and communication protocols to control the motor.

## Overview

The motor control interface (`mc_interface`) acts as a facade between the application layer and the low-level motor control implementations (BLDC via `mcpwm.c` or FOC via `mcpwm_foc.c`). It handles:

- Motor control commands (duty, current, speed, position)
- Dual motor support
- Real-time fault monitoring and protection
- Energy consumption tracking
- Statistical data collection
- Dynamic current/voltage limiting based on temperature and battery state
- Debug sampling and telemetry

## Key Data Structures

### motor_if_state_t (Lines 92-146)

Main state structure for each motor instance:

```c
typedef struct {
    mc_configuration m_conf;              // Motor configuration
    mc_fault_code m_fault_now;           // Current fault code
    volatile bool m_lock_enabled;         // Control lock state
    volatile bool m_lock_override_once;   // One-time lock override
    int m_ignore_iterations;              // Fault recovery delay counter

    // Temperature monitoring
    float m_temp_fet;                     // Filtered MOSFET temperature
    float m_temp_motor;                   // Filtered motor temperature
    float m_temp_override;                // Manual temp override

    // Energy tracking
    float m_amp_seconds;                  // Amp-hours consumed
    float m_amp_seconds_charged;          // Amp-hours regenerated
    float m_watt_seconds;                 // Watt-hours consumed
    float m_watt_seconds_charged;         // Watt-hours regenerated

    // Current averaging (for DQ axes)
    float m_motor_id_sum;                 // D-axis current sum
    float m_motor_iq_sum;                 // Q-axis current sum
    float m_motor_vd_sum;                 // D-axis voltage sum
    float m_motor_vq_sum;                 // Q-axis voltage sum

    // Statistics
    setup_stats m_stats;                  // Performance statistics

    // Voltage filtering
    float m_input_voltage_filtered;       // Fast filtered input voltage
    float m_input_voltage_filtered_slower;// Slow filtered input voltage
    float m_i_in_filter;                  // Filtered input current

    // Runtime tracking
    uint32_t m_cycles_running;            // Consecutive PWM cycles running
    float m_f_samp_now;                   // Current sampling frequency
    float m_position_set;                 // Position setpoint
} motor_if_state_t;
```

## Function Categories

---

## 1. Initialization and Configuration

### mc_interface_init() - Lines 168-255

**Purpose:** Initialize the motor control interface subsystem. This is the first function called from `main()` to set up motor control.

**Initialization sequence:**
1. Initialize motor state structures
2. Read motor configuration from EEPROM
3. Configure encoder based on sensor port mode
4. Initialize gate driver (DRV8301/8320S/8323S)
5. Initialize appropriate motor control mode (BLDC or FOC)
6. Create 4 background threads:
   - `timer_thread`: 1ms periodic tasks (fault checking, limit updates)
   - `sample_send_thread`: Debug sample transmission
   - `fault_stop_thread`: Fault handling and motor shutdown
   - `stat_thread`: 10ms statistics collection
7. Initialize BMS integration

**Parameters:** None

**Returns:** void

**Code Example:**
```c
// Called from main.c
mc_interface_init();

// Wait for DC calibration to complete
while (!mc_interface_dccal_done()) {
    chThdSleepMilliseconds(1);
}

// Now safe to control motor
mc_interface_set_current(15.0);
```

---

### mc_interface_get_configuration() - Lines 306-308

**Purpose:** Get read-only pointer to current motor configuration

**Parameters:** None

**Returns:** `const volatile mc_configuration*` - Configuration pointer

**Usage:**
```c
const volatile mc_configuration *conf = mc_interface_get_configuration();
float max_current = conf->l_current_max;
```

---

### mc_interface_set_configuration() - Lines 310-417

**Purpose:** Apply new motor configuration. Handles motor type changes, encoder reconfiguration, and gate driver settings.

**Key Operations:**
1. Check if encoder mode changed → re-initialize encoder
2. Update gate driver overcurrent settings
3. If motor type changed (BLDC ↔ FOC):
   - De-initialize old motor control
   - Apply new configuration
   - Initialize new motor control
4. Update override limits based on temperature/voltage
5. Re-initialize BMS integration

**Parameters:**
- `configuration`: Pointer to new configuration

**Returns:** void

**Notes:**
- Forces FOC mode on dual motor hardware
- Thread-safe via motor_now()

---

### mc_interface_dccal_done() - Lines 419-436

**Purpose:** Check if DC bus current calibration is complete. Must be done before controlling motor.

**Parameters:** None

**Returns:** `bool` - true if calibration complete

**Usage:**
```c
// Wait for calibration
while (!mc_interface_dccal_done()) {
    chThdSleepMilliseconds(1);
}
```

---

## 2. Motor Control - Setpoint Functions

### mc_interface_set_duty() - Lines 544-568

**Purpose:** Set motor duty cycle (voltage control mode)

**Parameters:**
- `dutyCycle`: Range [-1.0, 1.0] where 1.0 = 100% duty

**Control Flow:**
1. Reset shutdown timer if duty > 0.001
2. Check if input is allowed via `mc_interface_try_input()`
3. Apply direction multiplier
4. Dispatch to BLDC (`mcpwm_set_duty`) or FOC (`mcpwm_foc_set_duty`)
5. Log event

**Usage:**
```c
mc_interface_set_duty(0.5);  // 50% forward duty
mc_interface_set_duty(-0.3); // 30% reverse duty
```

---

### mc_interface_set_duty_noramp() - Lines 570-594

**Purpose:** Set duty cycle without ramping (immediate transition)

**Parameters:** Same as `mc_interface_set_duty()`

**Use Case:** For applications requiring instant response (e.g., RC car steering correction)

---

### mc_interface_set_current() - Lines 661-685

**Purpose:** Set motor current (torque control mode) - **Most commonly used control method**

**Parameters:**
- `current`: Motor current in amperes (positive = forward, negative = reverse)

**Advantages:**
- Best for traction control
- Precise torque control
- Regenerative braking support
- Battery current limiting applied automatically

**Usage:**
```c
mc_interface_set_current(15.0);   // 15A forward
mc_interface_set_current(-10.0);  // 10A braking
```

---

### mc_interface_set_brake_current() - Lines 687-711

**Purpose:** Set braking current (always brakes, regardless of direction)

**Parameters:**
- `current`: Braking current in amperes (positive value)

**Usage:**
```c
mc_interface_set_brake_current(20.0);  // 20A braking
```

---

### mc_interface_set_current_rel() - Lines 719-736

**Purpose:** Set current relative to configured limits

**Parameters:**
- `val`: Range [-1.0, 1.0] mapped to [l_current_min, l_current_max]

**Smart Behavior:**
- If duty cycle < 2%, uses configured limits directly
- If moving, considers direction for proper acceleration/braking
- Automatically resets current-off-delay timer

**Usage:**
```c
// 50% of max current forward
mc_interface_set_current_rel(0.5);
```

---

### mc_interface_set_brake_current_rel() - Lines 744-755

**Purpose:** Set brake current as percentage of max brake current

**Parameters:**
- `val`: Range [0.0, 1.0]

---

### mc_interface_set_handbrake() - Lines 763-788

**Purpose:** Apply open-loop handbrake (DC current injection for FOC, brake current for BLDC)

**Parameters:**
- `current`: Handbrake current in amperes

**Use Case:** Holding motor in position without encoder feedback

**Usage:**
```c
mc_interface_set_handbrake(10.0);  // 10A holding current
```

---

### mc_interface_set_handbrake_rel() - Lines 796-802

**Purpose:** Set handbrake as percentage of max brake current

---

### mc_interface_set_pid_speed() - Lines 596-620

**Purpose:** Set motor speed using PID controller (RPM control mode)

**Parameters:**
- `rpm`: Target motor electrical RPM (ERPM)

**Note:** ERPM = mechanical RPM × (pole pairs)

**Usage:**
```c
mc_interface_set_pid_speed(10000.0);  // 10,000 ERPM
```

---

### mc_interface_set_pid_pos() - Lines 622-659

**Purpose:** Set motor position (angle control mode)

**Parameters:**
- `pos`: Target position in degrees [0-360]

**Features:**
- Applies position offset from config
- Handles encoder inversion
- Normalizes angle to [0-360] range

**Usage:**
```c
mc_interface_set_pid_pos(180.0);  // Move to 180 degrees
```

---

### mc_interface_update_pid_pos_offset() - Lines 1472-1486

**Purpose:** Update position offset so current angle becomes specified angle (calibration)

**Parameters:**
- `angle_now`: Desired current angle
- `store`: If true, save offset to EEPROM

**Use Case:** Position sensor calibration

---

### mc_interface_set_openloop_current() - Lines 804-827

**Purpose:** Open-loop current control with specified RPM (FOC only)

**Parameters:**
- `current`: Current setpoint
- `rpm`: Open-loop rotation speed

**Use Case:** Motor startup, sensor calibration

---

### mc_interface_set_openloop_phase() - Lines 828-851

**Purpose:** Open-loop current at fixed rotor phase angle

**Parameters:**
- `current`: Current magnitude
- `phase`: Rotor electrical angle (degrees)

---

### mc_interface_set_openloop_duty() - Lines 852-875

**Purpose:** Open-loop duty cycle control with RPM

---

### mc_interface_set_openloop_duty_phase() - Lines 876-899

**Purpose:** Open-loop duty cycle at fixed phase

---

### mc_interface_brake_now() - Lines 901-905

**Purpose:** Immediate brake by setting duty to zero

---

### mc_interface_release_motor() - Lines 910-930

**Purpose:** Disconnect motor phases (free-wheel)

**Usage:**
```c
mc_interface_release_motor();  // Motor coasts freely
```

---

### mc_interface_release_motor_override() - Lines 932-948

**Purpose:** Force motor release, bypassing input checks

---

### mc_interface_release_motor_override_both() - Lines 1733-1740

**Purpose:** Release both motors (dual motor systems)

---

### mc_interface_wait_for_motor_release() - Lines 950-983

**Purpose:** Block until motor state is OFF or timeout

**Parameters:**
- `timeout`: Maximum wait time in seconds

**Returns:** `bool` - true if released within timeout

---

### mc_interface_wait_for_motor_release_both() - Lines 1742-1760

**Purpose:** Wait for both motors to release

---

## 3. Motor Control - State and Status Reading

### mc_interface_get_state() - Lines 512-529

**Purpose:** Get current motor control state

**Returns:** `mc_state` enum:
- `MC_STATE_OFF`: Motor disabled
- `MC_STATE_DETECTING`: Running motor detection
- `MC_STATE_RUNNING`: Motor actively controlled
- `MC_STATE_FULL_BRAKE`: Full braking applied

---

### mc_interface_get_control_mode() - Lines 531-542

**Purpose:** Get current control mode (FOC only)

**Returns:** `mc_control_mode` enum:
- `CONTROL_MODE_DUTY`
- `CONTROL_MODE_SPEED`
- `CONTROL_MODE_CURRENT`
- `CONTROL_MODE_CURRENT_BRAKE`
- `CONTROL_MODE_POS`
- `CONTROL_MODE_HANDBRAKE`
- `CONTROL_MODE_OPENLOOP`
- `CONTROL_MODE_NONE`

---

### mc_interface_get_duty_cycle_set() - Lines 988-1006

**Purpose:** Get duty cycle setpoint (commanded value)

**Returns:** `float` - Duty cycle [-1.0, 1.0]

---

### mc_interface_get_duty_cycle_now() - Lines 1008-1026

**Purpose:** Get actual current duty cycle (measured value)

**Returns:** `float` - Duty cycle [-1.0, 1.0]

---

### mc_interface_get_sampling_frequency_now() - Lines 1028-1046

**Purpose:** Get current PWM sampling frequency

**Returns:** `float` - Frequency in Hz

---

### mc_interface_get_rpm() - Lines 1048-1066

**Purpose:** Get motor electrical RPM (ERPM)

**Returns:** `float` - RPM value (positive = forward)

**Note:** To get mechanical RPM: `erpm / (pole_pairs)`

---

### mc_interface_get_tot_current() - Lines 1144-1162

**Purpose:** Get instantaneous total battery current

**Returns:** `float` - Current in amperes

---

### mc_interface_get_tot_current_filtered() - Lines 1164-1182

**Purpose:** Get filtered total battery current

---

### mc_interface_get_tot_current_directional() - Lines 1184-1202

**Purpose:** Get battery current with sign indicating direction (positive = forward)

---

### mc_interface_get_tot_current_directional_filtered() - Lines 1204-1222

**Purpose:** Filtered directional battery current

---

### mc_interface_get_tot_current_in() - Lines 1224-1242

**Purpose:** Get instantaneous input (battery) current

---

### mc_interface_get_tot_current_in_filtered() - Lines 1244-1262

**Purpose:** Get filtered input current

---

### mc_interface_get_input_voltage_filtered() - Lines 1264-1266

**Purpose:** Get filtered battery voltage

**Returns:** `float` - Voltage in volts

---

### mc_interface_get_abs_motor_current_unbalance() - Lines 1268-1286

**Purpose:** Get motor phase current imbalance (FOC with 3 shunts only)

**Returns:** `float` - Current imbalance magnitude

**Use Case:** Detect faulty current sensors

---

### mc_interface_get_pid_pos_set() - Lines 1433-1435

**Purpose:** Get position control setpoint

**Returns:** `float` - Position in degrees

---

### mc_interface_get_pid_pos_now() - Lines 1437-1467

**Purpose:** Get current motor position

**Returns:** `float` - Position in degrees [0-360]

**Processing:**
1. Read from encoder or FOC observer
2. Apply encoder inversion if configured
3. Apply direction multiplier
4. Subtract position offset
5. Normalize to [0-360]

---

### mc_interface_temp_fet_filtered() - Lines 1525-1527

**Purpose:** Get filtered MOSFET temperature

**Returns:** `float` - Temperature in °C

---

### mc_interface_temp_motor_filtered() - Lines 1536-1538

**Purpose:** Get filtered motor temperature

**Returns:** `float` - Temperature in °C

---

### mc_interface_get_last_inj_adc_isr_duration() - Lines 1347-1365

**Purpose:** Get ADC interrupt execution time

**Returns:** `float` - Duration in microseconds

**Use Case:** Performance monitoring, detecting CPU overload

---

### mc_interface_get_last_sample_adc_isr_duration() - Lines 1488-1490

**Purpose:** Get last sampled ADC ISR duration

---

## 4. Energy Consumption and Battery Tracking

### mc_interface_get_amp_hours() - Lines 1077-1085

**Purpose:** Get total amp-hours consumed from battery

**Parameters:**
- `reset`: If true, reset counter after reading

**Returns:** `float` - Amp-hours

**Usage:**
```c
float ah = mc_interface_get_amp_hours(false);  // Read without reset
commands_printf("Battery used: %.2f Ah", ah);
```

---

### mc_interface_get_amp_hours_charged() - Lines 1096-1104

**Purpose:** Get amp-hours fed back to battery (regenerative braking)

---

### mc_interface_get_watt_hours() - Lines 1115-1123

**Purpose:** Get watt-hours consumed from battery

**Returns:** `float` - Watt-hours

---

### mc_interface_get_watt_hours_charged() - Lines 1134-1142

**Purpose:** Get watt-hours regenerated to battery

---

### mc_interface_get_battery_level() - Lines 1550-1596

**Purpose:** Calculate remaining battery percentage based on voltage and configuration

**Parameters:**
- `wh_left`: Output pointer for remaining watt-hours (can be NULL)

**Returns:** `float` - Battery level [0.0, 1.0] where 1.0 = full

**Supported Battery Types:**
- `BATTERY_TYPE_LIION_3_0__4_2`: Li-ion (3.2V-4.2V per cell)
- `BATTERY_TYPE_LIIRON_2_6__3_6`: LiFePO4 (2.6V-3.6V per cell)
- `BATTERY_TYPE_LEAD_ACID`: Lead acid (2.1V-2.36V per cell)

**Algorithm:**
1. Calculate voltage-to-capacity curve for battery type
2. Estimate remaining Ah based on voltage per cell
3. Calculate remaining Wh = Ah × average_voltage_left
4. Return fraction of total Wh

**Usage:**
```c
float wh_left;
float battery_pct = mc_interface_get_battery_level(&wh_left);
commands_printf("Battery: %.0f%%, %.1f Wh left", battery_pct * 100, wh_left);
```

---

## 5. Tachometer Functions

### mc_interface_set_tachometer_value() - Lines 1288-1305

**Purpose:** Set tachometer step count (for calibration)

**Parameters:**
- `steps`: New tachometer value

**Returns:** `int` - Updated tachometer value

---

### mc_interface_get_tachometer_value() - Lines 1307-1325

**Purpose:** Get signed tachometer count (forward positive, reverse negative)

**Parameters:**
- `reset`: If true, reset counter

**Returns:** `int` - Tachometer steps

**Relationship:** `3 steps = 1 electrical revolution = 360 degrees`

---

### mc_interface_get_tachometer_abs_value() - Lines 1327-1345

**Purpose:** Get absolute tachometer (always increasing, no direction)

**Returns:** `int` - Absolute steps

---

## 6. Speed and Distance Functions

### mc_interface_get_speed() - Lines 1604-1616

**Purpose:** Calculate vehicle speed based on wheel diameter and motor RPM

**Returns:** `float` - Speed in meters/second

**Calculation:**
```c
mechanical_rpm = erpm / (poles / 2)
speed_m_s = (mechanical_rpm / 60) × wheel_diameter × PI / gear_ratio
```

**Configuration Requirements:**
- `si_motor_poles`: Motor pole count
- `si_wheel_diameter`: Wheel diameter in meters
- `si_gear_ratio`: Gear ratio

**Usage:**
```c
float speed_ms = mc_interface_get_speed();
float speed_kmh = speed_ms * 3.6;
commands_printf("Speed: %.1f km/h", speed_kmh);
```

---

### mc_interface_override_wheel_speed() - Lines 1646-1649

**Purpose:** Override speed calculation (for testing or external speed sensors)

**Parameters:**
- `ovr`: Enable/disable override
- `speed`: Override speed value

---

### mc_interface_get_distance() - Lines 1624-1628

**Purpose:** Get signed distance traveled (forward positive, reverse negative)

**Returns:** `float` - Distance in meters since boot

**Calculation:**
```c
tacho_scale = (wheel_diameter × PI) / (3 × motor_poles × gear_ratio)
distance = tachometer_steps × tacho_scale
```

---

### mc_interface_get_distance_abs() - Lines 1636-1644

**Purpose:** Get total distance (unsigned, always increasing)

**Returns:** `float` - Distance in meters

---

### mc_interface_set_odometer() - Lines 1700-1702

**Purpose:** Set odometer value (persisted across reboots)

**Parameters:**
- `new_odometer_meters`: New odometer value in meters

---

### mc_interface_get_odometer() - Lines 1710-1712

**Purpose:** Get total odometer reading

**Returns:** `uint64_t` - Total meters traveled

---

## 7. Statistics Functions

### mc_interface_get_setup_values() - Lines 1651-1688

**Purpose:** Get aggregated statistics from all VESCs on CAN bus

**Returns:** `setup_values` struct:
```c
typedef struct {
    float current_tot;          // Total current all motors
    float current_in_tot;       // Total input current
    int num_vescs;              // Number of VESCs detected
    float ah_tot;               // Total Ah consumed
    float ah_charge_tot;        // Total Ah regenerated
    float wh_tot;               // Total Wh consumed
    float wh_charge_tot;        // Total Wh regenerated
} setup_values;
```

**Features:**
- Includes local motor
- Aggregates CAN bus motors (within 100ms timeout)
- Used for multi-motor systems

---

### mc_interface_read_reset_avg_motor_current() - Lines 1367-1372

**Purpose:** Read and reset averaged motor current

**Returns:** `float` - Average current since last reset

---

### mc_interface_read_reset_avg_input_current() - Lines 1374-1379

**Purpose:** Read and reset averaged input current

---

### mc_interface_read_reset_avg_id() - Lines 1387-1392

**Purpose:** Read and reset average D-axis current (FOC only)

**Returns:** `float` - Average Id current

---

### mc_interface_read_reset_avg_iq() - Lines 1400-1405

**Purpose:** Read and reset average Q-axis current (FOC only)

**Returns:** `float` - Average Iq current

---

### mc_interface_read_reset_avg_vd() - Lines 1413-1418

**Purpose:** Read and reset average D-axis voltage (FOC only)

---

### mc_interface_read_reset_avg_vq() - Lines 1426-1431

**Purpose:** Read and reset average Q-axis voltage (FOC only)

---

### mc_interface_stat_speed_avg() - Lines 2742-2746

**Purpose:** Get average speed since statistics reset

**Returns:** `float` - Average speed in m/s

---

### mc_interface_stat_speed_max() - Lines 2748-2750

**Purpose:** Get maximum speed recorded

---

### mc_interface_stat_power_avg() - Lines 2752-2756

**Purpose:** Get average power consumption

**Returns:** `float` - Average power in watts

---

### mc_interface_stat_power_max() - Lines 2758-2760

**Purpose:** Get peak power recorded

---

### mc_interface_stat_current_avg() - Lines 2762-2766

**Purpose:** Get average motor current magnitude

---

### mc_interface_stat_current_max() - Lines 2768-2770

**Purpose:** Get peak motor current magnitude

---

### mc_interface_stat_temp_mosfet_avg() - Lines 2772-2776

**Purpose:** Get average MOSFET temperature

---

### mc_interface_stat_temp_mosfet_max() - Lines 2778-2780

**Purpose:** Get maximum MOSFET temperature recorded

---

### mc_interface_stat_temp_motor_avg() - Lines 2782-2786

**Purpose:** Get average motor temperature

---

### mc_interface_stat_temp_motor_max() - Lines 2788-2790

**Purpose:** Get maximum motor temperature recorded

---

### mc_interface_stat_count_time() - Lines 2792-2794

**Purpose:** Get elapsed time since statistics reset

**Returns:** `float` - Time in seconds

---

### mc_interface_stat_reset() - Lines 2796-2802

**Purpose:** Reset all statistics counters

**Usage:**
```c
mc_interface_stat_reset();
// Run motor for test
chThdSleepMilliseconds(10000);
// Check stats
float avg_power = mc_interface_stat_power_avg();
float max_temp = mc_interface_stat_temp_mosfet_max();
```

---

## 8. Fault Management

### mc_interface_get_fault() - Lines 471-473

**Purpose:** Get current fault code

**Returns:** `mc_fault_code` enum (30+ fault types)

**Common Faults:**
- `FAULT_CODE_NONE`: No fault
- `FAULT_CODE_OVER_VOLTAGE`: Input voltage too high
- `FAULT_CODE_UNDER_VOLTAGE`: Input voltage too low
- `FAULT_CODE_DRV`: Gate driver fault
- `FAULT_CODE_ABS_OVER_CURRENT`: Absolute current limit exceeded
- `FAULT_CODE_OVER_TEMP_FET`: MOSFET overtemperature
- `FAULT_CODE_OVER_TEMP_MOTOR`: Motor overtemperature
- `FAULT_CODE_ENCODER_SPI`: Encoder communication error
- `FAULT_CODE_FLASH_CORRUPTION`: Configuration corrupted

---

### mc_interface_fault_to_string() - Lines 475-510

**Purpose:** Convert fault code to human-readable string

**Parameters:**
- `fault`: Fault code enum

**Returns:** `const char*` - Fault name string

**Usage:**
```c
mc_fault_code fault = mc_interface_get_fault();
if (fault != FAULT_CODE_NONE) {
    commands_printf("Fault: %s", mc_interface_fault_to_string(fault));
}
```

---

### mc_interface_set_fault_info() - Lines 1839-1844

**Purpose:** Set additional fault information (for debugging)

**Parameters:**
- `str`: Info string
- `argn`: Number of arguments (0-2)
- `arg0`, `arg1`: Numeric arguments

---

### mc_interface_fault_stop() - Lines 1846-1857

**Purpose:** Trigger fault condition and stop motor

**Parameters:**
- `fault`: Fault code
- `is_second_motor`: Which motor faulted
- `is_isr`: Called from interrupt context

**Note:** Internal function, called by protection logic

---

## 9. Lock/Unlock Functions

### mc_interface_lock() - Lines 453-455

**Purpose:** Lock all motor control commands

**Use Case:** External safety systems, emergency stop

**Usage:**
```c
mc_interface_lock();
// Motor will not respond to any control commands
```

---

### mc_interface_unlock() - Lines 460-462

**Purpose:** Unlock motor control

---

### mc_interface_lock_override_once() - Lines 467-469

**Purpose:** Allow one motor command while locked, then re-lock

**Use Case:** Calibration sequences requiring single controlled movement

---

## 10. Timing and Delay Functions

### mc_interface_ignore_input() - Lines 1717-1720

**Purpose:** Ignore motor control commands for specified time

**Parameters:**
- `time_ms`: Delay in milliseconds

**Use Case:** Motor detection, sensor calibration sequences

---

### mc_interface_ignore_input_both() - Lines 1725-1731

**Purpose:** Ignore control on both motors

---

### mc_interface_set_current_off_delay() - Lines 1762-1785

**Purpose:** Keep motor control active for specified time (prevent sleep during pulsed control)

**Parameters:**
- `delay_sec`: Delay in seconds (max 5.0s)

**Use Case:** Applications with intermittent control (e.g., balance board)

---

## 11. Dual Motor Support

### mc_interface_motor_now() - Lines 257-272

**Purpose:** Get currently selected motor number

**Returns:** `int` - Motor number (1 or 2)

---

### mc_interface_select_motor_thread() - Lines 284-292

**Purpose:** Select which motor the current thread controls

**Parameters:**
- `motor`: Motor number (1 or 2)

**Usage:**
```c
mc_interface_select_motor_thread(1);
mc_interface_set_current(10.0);  // Controls motor 1

mc_interface_select_motor_thread(2);
mc_interface_set_current(15.0);  // Controls motor 2
```

---

### mc_interface_get_motor_thread() - Lines 302-304

**Purpose:** Get motor number for current thread

**Returns:** `int` - Motor number

---

## 12. Sampling and Debug

### mc_interface_sample_print_data() - Lines 1492-1516

**Purpose:** Configure and trigger ADC sample data collection

**Parameters:**
- `mode`: Sampling mode (now, on start, on trigger, on fault)
- `len`: Number of samples to collect
- `decimation`: Sample every Nth PWM cycle
- `raw`: Collect raw ADC values or calibrated
- `reply_func`: Callback for sending data

**Sampling Modes:**
- `DEBUG_SAMPLING_NOW`: Start immediately
- `DEBUG_SAMPLING_START`: Start when motor starts
- `DEBUG_SAMPLING_TRIGGER_START`: Trigger on motor start, capture before+after
- `DEBUG_SAMPLING_TRIGGER_FAULT`: Trigger on fault

**Use Case:** Oscilloscope-like waveform capture for debugging

---

### mc_interface_set_pwm_callback() - Lines 446-448

**Purpose:** Register callback function called after each PWM cycle

**Parameters:**
- `p_func`: Function pointer (called from ISR context)

**Warning:** Callback runs in interrupt - must be fast!

---

## 13. Miscellaneous Functions

### mc_interface_override_temp_motor() - Lines 1787-1789

**Purpose:** Override motor temperature reading (for testing without sensor)

**Parameters:**
- `temp`: Temperature in °C

---

### mc_interface_gnss() - Lines 1690-1692

**Purpose:** Get pointer to GNSS data structure

**Returns:** `volatile gnss_data*`

---

### mc_interface_calc_crc() - Lines 3010-3029

**Purpose:** Calculate CRC16 checksum of motor configuration

**Parameters:**
- `conf_in`: Configuration pointer (NULL for current)
- `is_motor_2`: Which motor configuration

**Returns:** `unsigned` - CRC16 value

**Use Case:** Verify configuration integrity

---

## 14. Internal/Helper Functions (Static)

### motor_now() - Lines 2502-2508

**Purpose:** Get pointer to current motor state structure

**Returns:** `volatile motor_if_state_t*`

---

### mc_interface_try_input() - Lines 1801-1837

**Purpose:** Check if motor control input is allowed

**Returns:** `int` - Milliseconds until input allowed (0 = allowed now)

**Blocks input if:**
- Motor detection running
- Ignore iterations counter > 0
- Motor locked (unless override set)
- Motor control not initialized

---

### update_override_limits() - Lines 2213-2500

**Purpose:** Calculate dynamic current/voltage limits based on:

**Limiting Factors:**
1. **Temperature Derating:**
   - MOSFET temperature (l_temp_fet_start → l_temp_fet_end)
   - Motor temperature (l_temp_motor_start → l_temp_motor_end)
   - Acceleration-specific derating (preserves braking torque)

2. **RPM Limiting:**
   - Max RPM limit (l_max_erpm)
   - Min RPM limit (l_min_erpm)
   - Start current decrease below threshold RPM

3. **Duty Cycle Limiting:**
   - Reduces current at high duty cycles

4. **Battery Protection:**
   - Low voltage cutoff (l_battery_cut_start → l_battery_cut_end)
   - Regen overvoltage cutoff (l_battery_regen_cut_start → end)
   - Wattage limits (l_watt_max, l_watt_min)

5. **BMS Integration:**
   - External BMS current limits via CAN

6. **Input Current Mapping:**
   - Recursive Iq limiting based on input current

**Output:** Updates `conf->lo_current_max` and `conf->lo_current_min`

**Example Derating:**
```
FET temp: 60°C, Start: 80°C, End: 100°C
→ No derating (below start temp)

FET temp: 90°C
→ 50% derating (halfway between start and end)

FET temp: 100°C
→ 0% current allowed (fault triggered)
```

---

## 15. Background Threads

### timer_thread() - Lines 2689-2702

**Purpose:** 1ms periodic tasks for each motor

**Tasks (via run_timer_tasks):**
- Filter battery voltage
- Update odometer and runtime
- Decrement fault iteration counter
- Clear faults when conditions resolve
- Update dynamic current limits (temperature, voltage, RPM)
- Control auxiliary output based on mode
- Check encoder faults
- Validate current sensor offsets
- Monitor 3-phase current balance
- Update wheel speed sensor

**Period:** 1ms (1000 Hz)

---

### stat_thread() - Lines 2804-2817

**Purpose:** Collect performance statistics

**Collected Every 10ms:**
- Current power (voltage × current)
- Speed
- MOSFET temperature
- Motor temperature
- Motor current

**Tracks:**
- Running averages (sum / samples)
- Peak values (max seen)

**Period:** 10ms (100 Hz)

---

### sample_send_thread() - Lines 2864-2900

**Purpose:** Send collected ADC samples over communication interface

**Operation:**
- Waits for event signal from sampling logic
- Determines sample range based on trigger mode
- Sends blocks of samples via callback function
- Each block contains: currents, phase voltages, switching frequency, status

---

### fault_stop_thread() - Lines 2902-2996

**Purpose:** Handle fault conditions and stop motor safely

**Operation:**
1. Wait for fault event signal
2. Check if fault is new (not already active)
3. Log fault data if DC calibration done:
   - Fault code
   - Motor currents (filtered and instantaneous)
   - Voltage, duty, RPM, tachometer
   - Temperature
   - Gate driver fault registers (if applicable)
   - Custom fault info string
4. Stop PWM output (BLDC or FOC)
5. Set ignore iterations timer
6. Set motor fault state

**Fault Data Logged:**
```c
fault_data {
    int motor;
    mc_fault_code fault;
    float current, current_filtered;
    float voltage, gate_driver_voltage;
    float duty, rpm;
    int tacho, cycles_running;
    int tim_val_samp, tim_current_samp, tim_top;
    int comm_step;
    float temperature;
    unsigned int drv8301_faults;
    const char *info_str;
    float info_args[2];
}
```

---

## 16. Interrupt Service Routine

### mc_interface_mc_timer_isr() - Lines 1861-2190

**Purpose:** Called every PWM cycle (~10-40 kHz) for real-time monitoring and protection

**Critical Operations:**

1. **Update LED PWM** (ledpwm_update_pwm)

2. **Filter Input Voltage:**
   ```c
   UTILS_LP_FAST(motor->m_input_voltage_filtered, input_voltage, 0.02);
   ```

3. **Voltage Fault Detection:**
   - Integrates voltage violations (above l_max_vin or below l_min_vin)
   - Triggers `FAULT_CODE_OVER_VOLTAGE` or `FAULT_CODE_UNDER_VOLTAGE`
   - Windup protection prevents false triggers

4. **Current Monitoring:**
   - Read motor current (FOC or BLDC specific)
   - Accumulate for averaging (Id, Iq, Vd, Vq)
   - Check absolute current limit → `FAULT_CODE_ABS_OVER_CURRENT`

5. **Gate Driver Fault:**
   - Check DRV fault pin
   - Trigger `FAULT_CODE_DRV` if asserted

6. **Gate Driver Supply Monitoring** (if enabled):
   - Check supply voltage within range
   - Trigger over/under voltage faults

7. **Energy Accounting:**
   ```c
   if (current_in_filtered > 0.0) {
       m_amp_seconds += current_in_filtered × t_samp;
       m_watt_seconds += current_in_filtered × t_samp × voltage;
   } else {
       m_amp_seconds_charged -= current_in_filtered × t_samp;
       m_watt_seconds_charged -= current_in_filtered × t_samp × voltage;
   }
   ```

8. **Debug Sampling:**
   - Trigger-based ADC sample capture
   - Stores currents, voltages, phase angle per PWM cycle
   - Multiple trigger modes (start, fault, manual)

**Execution Time:** Typically 5-15 µs (must be <50% of PWM period)

**Frequency:** Called at PWM frequency (10,000 - 40,000 Hz depending on config)

---

### mc_interface_adc_inj_int_handler() - Lines 2192-2205

**Purpose:** ADC injection interrupt handler (BLDC mode only)

**Operation:**
- Dispatches to `mcpwm_adc_inj_int_handler()` if in BLDC/DC mode
- FOC mode handles ADC internally

---

## Function Call Hierarchy

```
main()
  └─ mc_interface_init()
      ├─ conf_general_read_mc_configuration()
      ├─ encoder_init()
      ├─ drv8301_init() / drv8320s_init() / drv8323s_init()
      ├─ mcpwm_init() or mcpwm_foc_init()
      ├─ bms_init()
      └─ [Creates 4 threads]
          ├─ timer_thread
          │   └─ run_timer_tasks()
          │       └─ update_override_limits()
          ├─ sample_send_thread
          ├─ fault_stop_thread
          └─ stat_thread

[PWM Interrupt]
  └─ mc_interface_mc_timer_isr()
      ├─ ledpwm_update_pwm()
      ├─ mc_interface_fault_stop() [if fault detected]
      └─ [ADC sampling logic]
```

## Typical Usage Pattern

```c
// 1. Initialize (from main)
mc_interface_init();

// 2. Wait for calibration
while (!mc_interface_dccal_done()) {
    chThdSleepMilliseconds(1);
}

// 3. Control motor
mc_interface_set_current(20.0);  // 20A forward

// 4. Monitor status
float rpm = mc_interface_get_rpm();
float voltage = mc_interface_get_input_voltage_filtered();
float temp = mc_interface_temp_fet_filtered();

// 5. Check for faults
mc_fault_code fault = mc_interface_get_fault();
if (fault != FAULT_CODE_NONE) {
    commands_printf("Fault: %s", mc_interface_fault_to_string(fault));
}

// 6. Energy monitoring
float ah_used = mc_interface_get_amp_hours(false);
float wh_used = mc_interface_get_watt_hours(false);
float battery_pct = mc_interface_get_battery_level(NULL);

// 7. Stop motor
mc_interface_set_brake_current(10.0);  // Brake
// or
mc_interface_release_motor();  // Coast
```

## Key Design Patterns

1. **Hardware Abstraction:** All motor type differences (BLDC vs FOC) hidden behind unified API

2. **Dual Motor Support:** Thread-local motor selection via `motor_now()` and `mc_interface_select_motor_thread()`

3. **Direction Multiplier:** `DIR_MULT` macro inverts all directional values (duty, current, RPM) based on config

4. **Dynamic Limiting:** Real-time current/voltage limit adjustment in `update_override_limits()`

5. **Fault Protection:** Multi-layer protection:
   - ISR level: Absolute current, voltage, gate driver
   - Thread level: Temperature, sensor offsets, current balance
   - Integration: Prevents false triggers from transients

6. **Statistics:** Dual tracking systems:
   - Cumulative (amp-hours, watt-hours, odometer)
   - Windowed (average speed, power, temp over reset interval)

7. **Thread Safety:** Motor state accessed via `motor_now()` which reads thread-local variable

## Performance Considerations

- **ISR Execution:** `mc_interface_mc_timer_isr()` runs at 10-40 kHz → must complete in <25µs for 40kHz
- **Filter Constants:** Input voltage filtering prevents false voltage faults from PWM noise
- **Fault Integrators:** Voltage fault uses integrator with windup protection
- **Callback Warning:** PWM callback runs in ISR context → no blocking calls allowed

## Configuration Dependencies

Key configuration parameters affecting interface behavior:

```c
// Current limits
l_current_min, l_current_max          // Base current limits
l_current_min_scale, l_current_max_scale  // Scale factors
l_abs_current_max                     // Absolute hardware limit

// Voltage limits
l_min_vin, l_max_vin                  // Input voltage range
l_battery_cut_start, l_battery_cut_end  // Low voltage cutoff
l_battery_regen_cut_start, _end       // Regen overvoltage

// Temperature limits
l_temp_fet_start, l_temp_fet_end      // MOSFET temp derating
l_temp_motor_start, l_temp_motor_end  // Motor temp derating
l_temp_accel_dec                      // Acceleration derating factor

// RPM limits
l_max_erpm, l_min_erpm                // RPM limits
l_erpm_start                          // Soft limit start point

// Speed calculation
si_motor_poles                        // Motor pole count
si_wheel_diameter                     // Wheel diameter (m)
si_gear_ratio                         // Gear ratio

// Battery
si_battery_type                       // Battery chemistry
si_battery_cells                      // Cell count in series
si_battery_ah                         // Battery capacity
```

## Related Files

- `motor/mc_interface.h` - Public API header
- `motor/mcpwm_foc.c` - FOC motor control implementation
- `motor/mcpwm.c` - BLDC motor control implementation
- `hwconf/hw.h` - Hardware-specific definitions
- `conf_general.c` - Configuration storage/retrieval
- `encoder/encoder.c` - Position sensor interface

---

**Next Steps in Call Hierarchy:**

From `mc_interface_init()`, the next critical subsystems are:

1. **mcpwm_foc.c** - FOC motor control (most important)
2. **conf_general.c** - Configuration management
3. **encoder/encoder.c** - Position sensing
4. **drv83xx.c** - Gate driver control

Most applications will want to understand `mcpwm_foc.c` next, as it contains the core field-oriented control implementation.
