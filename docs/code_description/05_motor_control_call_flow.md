# Motor Control Call Flow and Thread Integration

**Purpose:** Trace complete call hierarchy from application input to motor PWM output, showing ChibiOS thread and ISR integration

## Overview

This document traces the **complete execution path** from user input (e.g., RC receiver) through ChibiOS threads to the hardware PWM that drives the motor. Understanding this flow is critical for:
- Debugging motor control issues
- Understanding timing and latency
- Adding custom control logic
- Performance optimization

---

## The Complete Control Loop

```
USER INPUT (Physical)
    │
    ├─► RC Receiver → PPM Signal
    ├─► Potentiometer → ADC Voltage
    ├─► Pedal Sensor → Hall Pulses
    └─► USB/CAN → Packet Data
         │
         ▼
┌────────────────────────────────────┐
│   CHIBIOS APPLICATION THREAD       │
│   (100 Hz, NORMALPRIO)             │
├────────────────────────────────────┤
│   app_ppm_thread()                 │
│   app_adc_thread()                 │
│   app_pas_thread()                 │
│   commands_process_packet()        │
└────────────────────────────────────┘
         │
         │ Read/Decode Input
         │ Apply Curves/Limits
         │
         ▼
┌────────────────────────────────────┐
│   MOTOR CONTROL INTERFACE          │
│   (Thread Context)                 │
├────────────────────────────────────┤
│   mc_interface_set_duty()          │
│   mc_interface_set_current()       │
│   mc_interface_set_pid_speed()     │
│   mc_interface_set_pid_pos()       │
└────────────────────────────────────┘
         │
         │ Thread-safe API
         │ Lock checking
         │
         ▼
┌────────────────────────────────────┐
│   FOC CONTROL IMPLEMENTATION       │
│   (Thread Context)                 │
├────────────────────────────────────┤
│   mcpwm_foc_set_duty()             │
│   mcpwm_foc_set_current()          │
│   mcpwm_foc_set_pid_speed()        │
│   mcpwm_foc_set_pid_pos()          │
└────────────────────────────────────┘
         │
         │ Store Setpoint
         │ (Non-blocking)
         │
    [Setpoint stored in RAM]
         │
         │ ═══════════════════════════════
         │     Context Switch to ISR
         │ ═══════════════════════════════
         │
         ▼
┌────────────────────────────────────┐
│   HARDWARE TIMER INTERRUPT         │
│   (40 kHz, Highest Priority)       │
├────────────────────────────────────┤
│   TIM1 Update ISR                  │
│   → Triggers ADC sampling          │
└────────────────────────────────────┘
         │
         │ Hardware trigger (synchronous)
         │
         ▼
┌────────────────────────────────────┐
│   ADC CONVERSION (Hardware)        │
│   Phase currents sampled           │
└────────────────────────────────────┘
         │
         │ Conversion complete
         │
         ▼
┌────────────────────────────────────┐
│   ADC DMA INTERRUPT                │
│   (40 kHz, Highest Priority)       │
├────────────────────────────────────┤
│   DMA Transfer Complete            │
│   → mcpwm_foc_adc_int_handler()    │
└────────────────────────────────────┘
         │
         │ ═══════════════════════════════
         │  FOC Algorithm (5-15 µs)
         │ ═══════════════════════════════
         │
         ├─► Read ADC values
         ├─► Clarke Transform (abc → αβ)
         ├─► Park Transform (αβ → dq)
         ├─► PI Current Control
         ├─► Inverse Park (dq → αβ)
         ├─► Space Vector Modulation
         └─► Update TIM1 CCR registers
         │
         ▼
┌────────────────────────────────────┐
│   HARDWARE PWM OUTPUT              │
│   (Phase A, B, C duty cycles)      │
└────────────────────────────────────┘
         │
         ▼
    MOTOR TURNS
```

---

## Detailed Trace: PPM RC Control Example

Let's trace a complete example: **RC transmitter stick moved to 75% throttle**

### Step 1: Hardware Capture (No CPU involved)

**File:** Hardware peripheral (STM32 TIM4)

```
RC Receiver → PPM Signal (1.75ms pulse)
    │
    ▼
STM32 TIM4 Input Capture
    │
    ├─ Measures pulse width
    └─ Stores in TIM4->CCR1 register

[No CPU cycles used - hardware timer captures pulse width]
```

---

### Step 2: Servo Decoder (Interrupt Context)

**File:** `servo_dec.c`

**Frequency:** Every PPM pulse (~50 Hz for RC)

```c
// Hardware timer capture interrupt
void TIM4_IRQHandler(void) {
    CH_IRQ_PROLOGUE();  // ChibiOS ISR entry

    // Quick: Calculate pulse width
    uint16_t capture = TIM4->CCR1;
    pulse_width_us = capture * timer_scale;

    // Store for thread to read
    servodec_ppm_pulse_width = pulse_width_us;

    CH_IRQ_EPILOGUE();  // ChibiOS ISR exit
}
```

**Duration:** <1 µs
**ChibiOS Role:** `CH_IRQ_PROLOGUE/EPILOGUE` handle context saving

---

### Step 3: PPM Application Thread (Thread Context)

**File:** `applications/app_ppm.c:108`

**Thread:** `ppm_thread` (NORMALPRIO, 100 Hz)

```c
static THD_FUNCTION(ppm_thread, arg) {
    chRegSetThreadName("APP_PPM");  // Register with ChibiOS

    for (;;) {
        // ═══════════════════════════════════════
        // Step 3a: Read PPM Pulse Width
        // ═══════════════════════════════════════
        float ppm_value = app_ppm_get_decoded_level();
        // Reads: servodec_ppm_pulse_width → 1.75ms
        // Maps to: [1.0ms, 2.0ms] → [-1.0, +1.0]
        // Result: ppm_value = 0.75

        // ═══════════════════════════════════════
        // Step 3b: Apply Throttle Curve
        // ═══════════════════════════════════════
        if (config->multi_esc) {
            ppm_value += 1.0;  // [0, 2.0] for multi-ESC
            ppm_value /= 2.0;  // [0, 1.0]
        }

        // Exponential curve
        if (config->curve_type == THR_EXP_EXPO) {
            ppm_value = utils_throttle_curve(
                ppm_value,
                config->curve_acc,
                config->curve_brake,
                config->throttle_exp);
        }
        // Result: ppm_value = 0.65 (after expo curve)

        // ═══════════════════════════════════════
        // Step 3c: Check Safe Start
        // ═══════════════════════════════════════
        if (config->safe_start) {
            if (fabsf(ppm_value) > 0.001 &&
                !is_running &&
                !was_at_zero) {
                // Don't allow start unless throttle was zero
                ppm_value = 0.0;
            }
        }

        // ═══════════════════════════════════════
        // Step 3d: Apply Ramping
        // ═══════════════════════════════════════
        float ramp_time = ppm_value > current_value ?
                         config->ramp_time_pos :
                         config->ramp_time_neg;

        float ramp_step = (1.0 / ramp_time) * 0.01;  // 100 Hz

        if (ppm_value > current_value) {
            current_value += ramp_step;
            if (current_value > ppm_value) {
                current_value = ppm_value;
            }
        }
        // Result: current_value = 0.65 (ramped)

        // ═══════════════════════════════════════
        // Step 3e: Check Output Enabled
        // ═══════════════════════════════════════
        if (app_is_output_disabled()) {
            continue;  // Don't send command
        }

        // ═══════════════════════════════════════
        // Step 3f: Convert to Motor Command
        // ═══════════════════════════════════════
        switch (config->ctrl_type) {
        case PPM_CTRL_TYPE_CURRENT:
            // Current control mode
            float current = current_value * config->current_max;
            mc_interface_set_current(current);
            // Calls: mc_interface_set_current(19.5A)
            break;

        case PPM_CTRL_TYPE_DUTY:
            // Duty cycle mode
            float duty = current_value;
            mc_interface_set_duty(duty);
            break;

        case PPM_CTRL_TYPE_PID_SPEED:
            // Speed control mode
            float rpm = current_value * config->max_erpm;
            mc_interface_set_pid_speed(rpm);
            break;
        }

        // ═══════════════════════════════════════
        // Step 3g: Sleep (Yield CPU)
        // ═══════════════════════════════════════
        chThdSleepMilliseconds(10);  // 100 Hz update rate
        // ChibiOS: Switch to next ready thread
    }
}
```

**Duration:** ~100-200 µs (depends on complexity)
**Blocks:** No (only at sleep)
**ChibiOS Calls:**
- `chRegSetThreadName()` - Register thread name
- `chThdSleepMilliseconds()` - Yield CPU for 10ms

**Timeline:**
```
0 µs:     Thread wakes up (10ms expired)
10 µs:    Read PPM value
20 µs:    Apply curve
30 µs:    Check limits
50 µs:    Calculate ramp
100 µs:   Call mc_interface_set_current(19.5)
150 µs:   Thread sleeps
```

---

### Step 4: Motor Control Interface (Thread Context)

**File:** `motor/mc_interface.c:661`

**Context:** Still in PPM thread (function call, not ISR)

```c
void mc_interface_set_current(float current) {
    // ═══════════════════════════════════════
    // Step 4a: Reset Timeout
    // ═══════════════════════════════════════
    if (fabsf(current) > 0.001) {
        SHUTDOWN_RESET();
        // Sets: last_command_time = chVTGetSystemTime()
    }

    // ═══════════════════════════════════════
    // Step 4b: Check if Input Allowed
    // ═══════════════════════════════════════
    if (mc_interface_try_input()) {
        return;  // Input blocked (detection, fault, lock)
    }
    // Checks:
    // - Motor not in detection mode
    // - No fault active
    // - Control not locked
    // - Initialization complete

    // ═══════════════════════════════════════
    // Step 4c: Get Motor Configuration
    // ═══════════════════════════════════════
    volatile motor_if_state_t *motor = motor_now();
    // ChibiOS: Reads thread-local motor selection
    // Returns: &m_motor_1 or &m_motor_2

    // ═══════════════════════════════════════
    // Step 4d: Apply Direction Multiplier
    // ═══════════════════════════════════════
    float current_dir = DIR_MULT * current;
    // DIR_MULT = 1.0 or -1.0 based on config
    // Result: current_dir = 19.5A

    // ═══════════════════════════════════════
    // Step 4e: Dispatch to Motor Type
    // ═══════════════════════════════════════
    switch (motor->m_conf.motor_type) {
    case MOTOR_TYPE_BLDC:
        mcpwm_set_current(current_dir);
        break;

    case MOTOR_TYPE_FOC:
        mcpwm_foc_set_current(current_dir);
        // Calls: mcpwm_foc_set_current(19.5)
        break;
    }

    // ═══════════════════════════════════════
    // Step 4f: Log Event
    // ═══════════════════════════════════════
    events_add("set_current", current);
    // For debugging/telemetry
}
```

**Duration:** ~5-10 µs
**Blocks:** No
**Thread-Safe:** Yes (uses `motor_now()` for thread-local state)

**Key ChibiOS Integration:**
```c
static volatile motor_if_state_t *motor_now(void) {
    // Read thread-local variable set by:
    // mc_interface_select_motor_thread(1 or 2)

    return chThdGetSelfX()->motor_selected == 1 ?
           &m_motor_1 : &m_motor_2;
}
```

---

### Step 5: FOC Control Implementation (Thread Context)

**File:** `motor/mcpwm_foc.c:3015`

**Context:** Still in PPM thread

```c
void mcpwm_foc_set_current(float current) {
    // ═══════════════════════════════════════
    // Step 5a: Get Motor State
    // ═══════════════════════════════════════
    volatile motor_all_state_t *motor = get_motor_now();
    // Dual motor support: &m_motor_1 or &m_motor_2

    // ═══════════════════════════════════════
    // Step 5b: Check Current Limits
    // ═══════════════════════════════════════
    utils_truncate_number(&current,
                         motor->m_conf->lo_current_min,
                         motor->m_conf->lo_current_max);
    // lo_current_max = 30.0A (example)
    // Input: 19.5A → No truncation

    // ═══════════════════════════════════════
    // Step 5c: Store Setpoint (Critical!)
    // ═══════════════════════════════════════
    motor->m_iq_set = current;
    // ★ THIS IS IT! ★
    // ISR will read this variable at 40 kHz

    // ═══════════════════════════════════════
    // Step 5d: Set Control Mode
    // ═══════════════════════════════════════
    motor->m_control_mode = CONTROL_MODE_CURRENT;
    motor->m_state = MC_STATE_RUNNING;

    // NO PWM UPDATE HERE!
    // The ISR will handle actual control
}
```

**Duration:** ~2-3 µs
**Blocks:** No
**Critical Section:** No lock needed (ISR reads, thread writes - single write)

**Memory Barrier:**
```c
// Compiler ensures m_iq_set written before m_control_mode
// ARM Cortex-M4 has strong memory ordering
```

**Function Returns →** Back to PPM thread → Sleep → ChibiOS schedules next thread

---

## Context Switch: Thread to ISR

**Key Point:** The ISR does NOT wait for thread! It runs independently at 40 kHz.

**Timeline:**

```
Time: 0.000 ms - PPM thread sets m_iq_set = 19.5A
Time: 0.002 ms - PPM thread sleeps
Time: 0.005 ms - ChibiOS schedules USB thread
Time: 0.010 ms - USB thread running
Time: 0.015 ms - CAN thread running
Time: 0.025 ms - ★ TIM1 ISR fires! ★ (preempts CAN thread)
```

**ChibiOS Preemption:**
```
Lower Priority Thread (CAN, NORMALPRIO)
    │ Running normally...
    │
    ├─► TIM1 Interrupt fires
    │
    ▼
ChibiOS Scheduler
    │
    ├─► Save CAN thread context (registers, PC, SP)
    ├─► Load ISR context
    └─► Jump to ISR handler
    │
    ▼
TIM1 ISR (Highest Priority)
    │ Executes FOC algorithm...
    │
    ▼
ISR Returns
    │
    ▼
ChibiOS Scheduler
    │
    ├─► Check if higher priority thread ready
    ├─► If yes: context switch to that thread
    └─► If no: restore CAN thread context
```

---

### Step 6: Timer ISR (Interrupt Context)

**Hardware:** STM32 TIM1 Update event

**Frequency:** 40 kHz (every 25 µs)

**File:** `hw.h` or `mcpwm_foc.c` (configured via HAL)

```c
// Simplified - actual implementation varies by hardware
void TIM1_UP_TIM10_IRQHandler(void) {
    // Hardware clears UIF flag
    TIM1->SR = ~TIM_SR_UIF;

    // ═══════════════════════════════════════
    // Step 6a: Trigger ADC Sampling
    // ═══════════════════════════════════════
    // TIM1 TRGO2 triggers ADC injection
    // (Configured in hardware)

    // ═══════════════════════════════════════
    // Step 6b: Call ChibiOS Timer Callback
    // ═══════════════════════════════════════
    // Some implementations call:
    mc_interface_mc_timer_isr(false);  // Motor 1
    // This handles fault checks, voltage filtering

    // ISR returns, ADC conversion happening in parallel
}
```

**Duration:** ~1-2 µs
**Preempts:** Everything (highest priority)

---

### Step 7: ADC Conversion (Hardware Parallel)

**Hardware:** STM32 ADC1/ADC2/ADC3 (injected sequence)

**Trigger:** TIM1 TRGO2 event (synchronous with PWM)

```
TIM1 triggers ADC
    │
    ▼
ADC samples phase currents (simultaneously)
    ├─ ADC1: Phase A current
    ├─ ADC2: Phase B current
    └─ ADC3: Phase C current
    │
    │ [Conversion time: ~1 µs]
    │
    ▼
ADC conversion complete
    │
    ▼
DMA transfers ADC→RAM (optional)
    │
    ▼
DMA interrupt fires
```

---

### Step 8: FOC Algorithm ISR (Interrupt Context)

**File:** `motor/mcpwm_foc.c` (DMA complete callback)

**Frequency:** 40 kHz (every 25 µs)

**Duration:** 5-15 µs (CRITICAL TIMING!)

```c
void mcpwm_foc_adc_int_handler(void) {
    // ═══════════════════════════════════════
    // Step 8a: Read ADC Values
    // ═══════════════════════════════════════
    volatile motor_all_state_t *motor = &m_motor_1;

    // Read phase currents from ADC
    int ia_raw = ADC1->JDR1;  // Phase A
    int ib_raw = ADC2->JDR1;  // Phase B
    int ic_raw = ADC3->JDR1;  // Phase C

    // Offset correction (from DC calibration)
    float ia = (ia_raw - motor->m_curr0_offset) * FAC_CURRENT;
    float ib = (ib_raw - motor->m_curr1_offset) * FAC_CURRENT;
    float ic = (ic_raw - motor->m_curr2_offset) * FAC_CURRENT;
    // Result: ia=15.2A, ib=-7.6A, ic=-7.6A

    // ═══════════════════════════════════════
    // Step 8b: Clarke Transform (abc → αβ)
    // ═══════════════════════════════════════
    float i_alpha = ia;
    float i_beta = ONE_BY_SQRT3 * ia + TWO_BY_SQRT3 * ib;
    // Transforms 3-phase to 2-phase stationary frame

    // ═══════════════════════════════════════
    // Step 8c: Park Transform (αβ → dq)
    // ═══════════════════════════════════════
    float phase = motor->m_phase_now_observer;  // Electrical angle

    float cos_phase = utils_fast_cos(phase);
    float sin_phase = utils_fast_sin(phase);

    float id = i_alpha * cos_phase + i_beta * sin_phase;
    float iq = i_beta * cos_phase - i_alpha * sin_phase;
    // Transforms to rotating frame aligned with rotor
    // Result: id=0.5A, iq=19.3A

    // ═══════════════════════════════════════
    // Step 8d: Read Setpoint from Thread
    // ═══════════════════════════════════════
    float iq_set = motor->m_iq_set;  // ★ From thread! ★
    // Value: 19.5A (set by PPM thread earlier)

    float id_set = 0.0;  // Typically zero (no field weakening)

    // ═══════════════════════════════════════
    // Step 8e: PI Current Control
    // ═══════════════════════════════════════
    // Q-axis (torque) controller
    float iq_error = iq_set - iq;  // 19.5 - 19.3 = 0.2A
    motor->m_iq_integrator += iq_error * motor->m_conf->foc_current_ki * dt;
    float vq = motor->m_conf->foc_current_kp * iq_error +
               motor->m_iq_integrator;

    // D-axis (flux) controller
    float id_error = id_set - id;
    motor->m_id_integrator += id_error * motor->m_conf->foc_current_ki * dt;
    float vd = motor->m_conf->foc_current_kp * id_error +
               motor->m_id_integrator;

    // Result: vq=12.5V, vd=0.2V

    // ═══════════════════════════════════════
    // Step 8f: Voltage Decoupling (Optional)
    // ═══════════════════════════════════════
    if (motor->m_conf->foc_cc_decoupling) {
        float rpm_now = motor->m_speed_est_fast;
        float L = motor->m_conf->foc_motor_l;
        float lambda = motor->m_conf->foc_motor_flux_linkage;

        // Cross-coupling compensation
        vq += rpm_now * L * id;
        vd -= rpm_now * (L * iq + lambda);
    }

    // ═══════════════════════════════════════
    // Step 8g: Inverse Park Transform (dq → αβ)
    // ═══════════════════════════════════════
    float v_alpha = vd * cos_phase - vq * sin_phase;
    float v_beta = vq * cos_phase + vd * sin_phase;

    // ═══════════════════════════════════════
    // Step 8h: Space Vector Modulation (αβ → abc)
    // ═══════════════════════════════════════
    float v_bus = motor->m_v_bus_filtered;  // 50.0V

    // Calculate phase voltages
    float va = v_alpha;
    float vb = -0.5 * v_alpha + SQRT3_BY_2 * v_beta;
    float vc = -0.5 * v_alpha - SQRT3_BY_2 * v_beta;

    // Normalize to duty cycles
    float duty_a = va / v_bus;  // 0.25 = 25%
    float duty_b = vb / v_bus;  // -0.12 = -12%
    float duty_c = vc / v_bus;  // -0.13 = -13%

    // Add offset for midpoint (0.5 = 50%)
    duty_a += 0.5;  // 0.75
    duty_b += 0.5;  // 0.38
    duty_c += 0.5;  // 0.37

    // ═══════════════════════════════════════
    // Step 8i: Update PWM Registers (Hardware)
    // ═══════════════════════════════════════
    uint32_t top = TIM1->ARR;  // Timer top value

    TIM1->CCR1 = (uint32_t)(duty_a * top);  // Phase A
    TIM1->CCR2 = (uint32_t)(duty_b * top);  // Phase B
    TIM1->CCR3 = (uint32_t)(duty_c * top);  // Phase C

    // ★ PWM updated! Motor voltage changes immediately ★

    // ═══════════════════════════════════════
    // Step 8j: Update Observer (Sensorless)
    // ═══════════════════════════════════════
    // Estimate rotor position and speed
    observer_update(motor, v_alpha, v_beta, i_alpha, i_beta);

    motor->m_phase_now_observer += motor->m_speed_est_fast * dt;
    utils_norm_angle_rad(&motor->m_phase_now_observer);

    // ISR complete - return to interrupted thread
}
```

**Duration:** 5-15 µs depending on:
- CPU speed (168 MHz STM32F4)
- Compiler optimization
- FPU usage
- Observer complexity

**Critical Timing:**
```
PWM Period: 25 µs (40 kHz)
ISR Duration: 12 µs (example)
Available for threads: 13 µs (52% CPU for ISR!)
```

**ChibiOS Integration:**
- ISR preempts any thread
- No ChibiOS calls in ISR (too slow!)
- Can signal threads via `chEvtSignalI()` if needed

---

### Step 9: PWM Output (Hardware)

**Hardware:** STM32 TIM1 PWM

```
TIM1 Counter running at PWM frequency
    │
    ├─► When counter < CCR1: Phase A high
    ├─► When counter < CCR2: Phase B high
    ├─► When counter < CCR3: Phase C high
    │
    └─► Counter overflow: Restart, trigger ADC
```

**Gate Driver:** DRV8301/8320S/8323S receives PWM signals

**Motor:** Phase voltages applied → Current flows → Torque generated

---

## Timing Analysis

### Latency Breakdown

**From RC Stick Move to Motor Response:**

| Stage | Duration | Accumulated | Notes |
|-------|----------|-------------|-------|
| PPM pulse capture | 0 µs | 0 µs | Hardware |
| Servo decoder ISR | 1 µs | 1 µs | Interrupt |
| Wait for PPM thread | 0-10 ms | 10 ms | ChibiOS scheduling |
| PPM thread processing | 150 µs | 10.15 ms | Thread |
| mc_interface call | 5 µs | 10.155 ms | Thread |
| mcpwm_foc_set_current | 2 µs | 10.157 ms | Thread |
| Wait for next ISR | 0-25 µs | 10.18 ms | ISR period |
| FOC ISR execution | 12 µs | 10.192 ms | Interrupt |
| **TOTAL LATENCY** | **~10-20 ms** | | **50-100 Hz effective** |

**Key Bottleneck:** PPM thread update rate (100 Hz = 10 ms period)

---

### CPU Usage Estimation

**At 40 kHz PWM:**

```
ISR time: 12 µs
ISR period: 25 µs
ISR CPU: 48%

Thread overhead: ~10%

Available for applications: 42%
```

**At 20 kHz PWM (more common):**

```
ISR time: 12 µs
ISR period: 50 µs
ISR CPU: 24%

Thread overhead: ~10%

Available for applications: 66%
```

---

## ChibiOS Synchronization Points

### 1. Thread to Thread (Rare in VESC)

**Via Events:**
```c
// Thread A
chEvtSignal(thread_b_tp, (eventmask_t)1);

// Thread B
chEvtWaitAny((eventmask_t)1);
```

**Example:** Sample complete → Sample send thread

---

### 2. Thread to ISR (Not Allowed)

**Cannot signal ISR** - ISR runs on hardware triggers

---

### 3. ISR to Thread (Common Pattern)

**Via Events:**
```c
// In ISR (fast path)
chSysLockFromISR();
chEvtSignalI(fault_thread_tp, (eventmask_t)1);
chSysUnlockFromISR();

// Thread (slow path)
chEvtWaitAny((eventmask_t)1);
// Handle fault...
```

**Example:** Fault detection → Fault stop thread

---

### 4. Shared Variables (Thread ↔ ISR)

**Pattern: Thread writes, ISR reads**

```c
// Thread context
motor->m_iq_set = 19.5;  // Write setpoint

// ISR context (later)
float iq_set = motor->m_iq_set;  // Read setpoint
```

**Thread-Safe:** Single write, single read (ARM guarantees atomicity for 32-bit)

**Not Thread-Safe:** Complex structures, need mutex or disable interrupts

---

## Performance Optimization Tips

### 1. Minimize Thread Latency

```c
// ❌ Bad: Slow processing in thread
static THD_FUNCTION(app_thread, arg) {
    for (;;) {
        expensive_calculation();  // 5ms!
        mc_interface_set_current(result);
        chThdSleepMilliseconds(10);
    }
}

// ✅ Good: Fast processing
static THD_FUNCTION(app_thread, arg) {
    for (;;) {
        quick_read();  // 50µs
        mc_interface_set_current(value);
        chThdSleepMilliseconds(10);
    }
}
```

### 2. Minimize ISR Duration

```c
// ❌ Bad: Complex math in ISR
void foc_isr(void) {
    float result = powf(x, 2.5);  // Slow!
}

// ✅ Good: Lookup table or fast math
void foc_isr(void) {
    float result = fast_pow25(x);  // Fast!
}
```

### 3. Use Correct Priority

```c
// ❌ Bad: Safety thread at NORMALPRIO
chThdCreateStatic(..., NORMALPRIO, timeout_thread, ...);

// ✅ Good: Safety thread at HIGHPRIO
chThdCreateStatic(..., HIGHPRIO, timeout_thread, ...);
```

---

## Debugging Tips

### 1. Trace Execution with GPIO

```c
// In thread
palSetPad(GPIOA, 5);  // Set PA5 high
mc_interface_set_current(10.0);
palClearPad(GPIOA, 5);  // Set PA5 low

// Oscilloscope on PA5 shows thread timing
```

### 2. Measure ISR Duration

```c
void foc_isr(void) {
    uint32_t start = DWT->CYCCNT;  // Cycle counter

    // FOC algorithm...

    uint32_t duration = DWT->CYCCNT - start;
    isr_duration_us = duration / (SystemCoreClock / 1000000);
}
```

### 3. Check Stack Usage

```c
// In ChibiOS config
#define CH_DBG_FILL_THREADS    TRUE

// At runtime
size_t unused = chThdGetFillLevel(thread_tp);
commands_printf("Stack unused: %d bytes", unused);
```

---

## Summary

**Complete Flow:**
1. **Hardware** captures input (PPM, ADC)
2. **Application Thread** (100 Hz) processes input
3. **mc_interface** provides thread-safe API
4. **mcpwm_foc** stores setpoint (non-blocking)
5. **Timer ISR** (40 kHz) triggers ADC
6. **FOC ISR** reads setpoint, runs algorithm
7. **PWM** updates, motor responds

**ChibiOS Role:**
- **Threads** handle slow operations (10-100 Hz)
- **ISR** handles fast operations (40 kHz)
- **Scheduler** provides preemptive multitasking
- **Events/Mutexes** coordinate threads
- **Context Switching** saves/restores thread state

**Key Insight:**
Motor control is **time-decoupled**:
- Applications set **target** (what to do)
- ISR achieves **target** (how to do it)
- Threads and ISR communicate via **shared variables**

This architecture allows VESC to:
- Accept input at 100 Hz (comfortable for applications)
- Control motor at 40 kHz (necessary for FOC)
- Run communications/logging in parallel
- Maintain real-time response

**Next Steps:**
Study `motor/mcpwm_foc.c` for complete FOC algorithm implementation!
