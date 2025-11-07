# Motor Control System Documentation

## Table of Contents

1. [Introduction](#introduction)
2. [BLDC Motor Theory](#bldc-motor-theory)
3. [Control Methods](#control-methods)
4. [Field-Oriented Control (FOC) Theory](#field-oriented-control-foc-theory)
5. [VESC Motor Control Architecture](#vesc-motor-control-architecture)
6. [Implementation Files](#implementation-files)
7. [Control Modes](#control-modes)
8. [Using the Motor Control API](#using-the-motor-control-api)

---

## Introduction

The `/motor/` directory contains the heart of the VESC firmware - the motor control algorithms. This system is responsible for converting high-level commands (like "spin at 1000 RPM" or "apply 10A of current") into precisely-timed PWM signals that drive the motor phases.

**Key Features:**
- Support for BLDC, DC, and FOC motor types
- Multiple control modes (duty cycle, speed, current, position)
- Sensorless and sensored operation
- Advanced Field-Oriented Control (FOC) with Clarke/Park transforms
- Real-time current and speed control loops
- Safety features and fault detection

---

## BLDC Motor Theory

### What is a BLDC Motor?

A **Brushless DC (BLDC)** motor is an electronically controlled motor that uses permanent magnets on the rotor and electromagnets (coils) on the stator. Unlike brushed DC motors, BLDC motors require electronic commutation - the controller must energize the correct phase windings at the correct time.

### Three-Phase BLDC Motor Structure

```
        Phase A
           |
           |
    Phase B---O---Phase C
           |
         Rotor
      (Permanent
       Magnets)
```

A typical BLDC motor has:
- **3 phases** (A, B, C) - three sets of stator windings
- **Rotor** with permanent magnets (typically 2-14 pole pairs)
- **Hall sensors** (optional) for position feedback

### How BLDC Motors Work

1. **Electromagnetic Principles:**
   - Current through a coil creates a magnetic field
   - Magnetic fields attract/repel permanent magnets
   - By energizing coils in sequence, we create a rotating magnetic field
   - The rotor follows this rotating field

2. **Commutation:**
   The process of switching which phases are energized to keep the rotor spinning. There are 6 electrical states in a full rotation:

   ```
   Step 1: A+ B-  (C floating)
   Step 2: A+ C-  (B floating)
   Step 3: B+ C-  (A floating)
   Step 4: B+ A-  (C floating)
   Step 5: C+ A-  (B floating)
   Step 6: C+ B-  (A floating)
   ```

3. **Electrical vs Mechanical Angle:**
   - **Mechanical angle**: Physical rotation of the rotor (0-360°)
   - **Electrical angle**: Position within electrical cycle (0-360°)
   - Relationship: `Electrical Angle = Mechanical Angle × Pole Pairs`
   - Example: 7 pole pair motor makes 7 electrical rotations per mechanical rotation

---

## Control Methods

### 1. Trapezoidal Commutation (Basic BLDC)

**File:** `mcpwm.c/.h`

This is the simpler control method, energizing two phases at a time with rectangular (trapezoidal) current waveforms.

**Advantages:**
- Simple to implement
- Less computational overhead
- Works well with Hall sensors

**Disadvantages:**
- Higher torque ripple
- Less efficient
- More audible noise
- Requires position feedback (Hall sensors or back-EMF detection)

**Implementation in VESC:**
- Uses Hall sensor feedback or sensorless back-EMF detection
- 6-step commutation sequence
- Implemented in `mcpwm.c` (81KB)

### 2. Field-Oriented Control (FOC)

**Files:** `mcpwm_foc.c/.h`, `foc_math.c/.h`

FOC (also called Vector Control) is an advanced control method that treats the 3-phase motor as if it were a DC motor by controlling current in a rotating reference frame.

**Advantages:**
- Smooth torque production (minimal ripple)
- Higher efficiency
- Quieter operation
- Better low-speed performance
- Can run sensorless using observers

**Disadvantages:**
- More computationally intensive
- Requires precise motor parameters
- More complex tuning

**Implementation in VESC:**
- Full FOC implementation in `mcpwm_foc.c` (172KB - largest file!)
- Clarke and Park transforms
- Current control in d-q reference frame
- Multiple observer implementations for sensorless operation
- Support for multiple HFI (High-Frequency Injection) modes

---

## Field-Oriented Control (FOC) Theory

### The FOC Principle

FOC simplifies 3-phase motor control by transforming the problem into a 2D rotating coordinate system where control is much easier.

### Coordinate Transformations

#### 1. Clarke Transform (ABC → αβ)

Converts 3-phase stationary coordinates (A, B, C) to 2-phase stationary coordinates (α, β):

```
α = Ia
β = (Ia + 2·Ib) / √3
```

Where:
- Ia, Ib, Ic are the three phase currents
- Ic can be calculated as: Ic = -(Ia + Ib)

**Purpose:** Reduces 3 variables to 2 (simpler mathematics)

#### 2. Park Transform (αβ → dq)

Converts 2-phase stationary coordinates (α, β) to 2-phase rotating coordinates (d, q):

```
Id = α·cos(θ) + β·sin(θ)
Iq = -α·sin(θ) + β·cos(θ)
```

Where θ is the electrical angle of the rotor.

**Purpose:** Creates a coordinate system that rotates with the rotor. In this frame:
- **Id (direct axis)**: Controls magnetic flux (like field current in DC motor)
- **Iq (quadrature axis)**: Controls torque (like armature current in DC motor)

### Why is FOC Powerful?

In the d-q reference frame:
- Id and Iq are DC values (not oscillating) - easy to control with PI controllers
- Torque is directly proportional to Iq: `Torque = Kt × Iq`
- For maximum efficiency, we typically set Id = 0 (no flux weakening)
- We control Iq to control torque/current

### FOC Control Loop Structure

```
┌─────────────┐
│ Target      │
│ Current/RPM │
└──────┬──────┘
       │
       ▼
┌─────────────────────────┐
│ Speed/Position          │
│ PI Controller           │
└───────────┬─────────────┘
            │ Target Iq
            ▼
     ┌──────────────┐
     │ Current      │
     │ Controller   │
     └──┬───────────┘
        │ Vd, Vq
        ▼
   ┌────────────┐
   │ Inverse    │
   │ Park       │ (dq → αβ)
   └─────┬──────┘
         │ Vα, Vβ
         ▼
   ┌────────────┐
   │ Inverse    │
   │ Clarke     │ (αβ → ABC)
   └─────┬──────┘
         │ Va, Vb, Vc
         ▼
   ┌────────────┐
   │ Space      │
   │ Vector PWM │
   └─────┬──────┘
         │ PWM Duty Cycles
         ▼
   ┌────────────┐
   │ 3-Phase    │
   │ Inverter   │
   └─────┬──────┘
         │
         ▼
    ┌─────────┐
    │  Motor  │
    └────┬────┘
         │ Position/Current
         │ Feedback
         └─────────────────┐
                           │
         ┌─────────────────┘
         │
         ▼
   ┌────────────┐
   │ Clarke     │ (ABC → αβ)
   └─────┬──────┘
         │
         ▼
   ┌────────────┐
   │ Park       │ (αβ → dq)
   └─────┬──────┘
         │ Id, Iq (measured)
         └─────────────────▶ (feedback to controller)
```

### Current Control in FOC

The VESC uses cascaded PI (Proportional-Integral) controllers:

1. **Outer Loop** (slower):
   - Speed Controller: Target RPM → Target Iq
   - Position Controller: Target Position → Target RPM

2. **Inner Loop** (faster):
   - Current Controller: Target Id/Iq → Voltage commands Vd/Vq
   - Runs every PWM cycle (typically 20-40 kHz)

### Sensorless FOC

VESC supports multiple sensorless FOC methods:

1. **Observer-based** (used during normal operation):
   - Estimates rotor position from back-EMF
   - Multiple observer types available (Ortega, MxLemming, etc.)
   - Works well at medium to high speeds

2. **HFI (High-Frequency Injection)** (used at low speed/standstill):
   - Injects high-frequency signal into motor
   - Detects rotor position from saliency (magnetic asymmetry)
   - Multiple HFI variants (V2, V3, V4, V5) for different motor types
   - Essential for low-speed sensorless operation

---

## VESC Motor Control Architecture

### Abstraction Layers

```
┌────────────────────────────────────────┐
│  mc_interface.h                        │  ← Unified API
│  (Motor Control Interface)             │
└───────────┬───────────┬────────────────┘
            │           │
    ┌───────┘           └────────┐
    │                            │
    ▼                            ▼
┌────────────────┐    ┌──────────────────┐
│  mcpwm.c/.h    │    │  mcpwm_foc.c/.h  │
│  (Trapezoidal) │    │  (FOC Control)   │
└────────────────┘    └──────────────────┘
                             │
                             ├─────────────┐
                             ▼             ▼
                      ┌──────────┐  ┌──────────┐
                      │foc_math.c│  │virtual   │
                      │(Clarke/  │  │_motor.c  │
                      │ Park)    │  │(Sim)     │
                      └──────────┘  └──────────┘
```

### Design Philosophy

1. **mc_interface.c/.h**: Provides a hardware-independent API
   - Applications don't need to know if FOC or BLDC mode is used
   - Automatically routes calls to the appropriate implementation
   - Handles motor selection for dual-motor hardware

2. **mcpwm.c/.h**: Trapezoidal BLDC implementation
   - Hall sensor support
   - Sensorless back-EMF detection
   - 6-step commutation

3. **mcpwm_foc.c/.h**: FOC implementation
   - Full vector control
   - Clarke/Park transforms
   - Current control loops
   - Multiple observer implementations
   - HFI for sensorless low-speed operation

4. **foc_math.c/.h**: Mathematical functions for FOC
   - Optimized trigonometric functions
   - Coordinate transformations
   - Fixed-point and floating-point math

5. **virtual_motor.c/.h**: Virtual motor simulation
   - Test control algorithms without hardware
   - Motor model simulation
   - Useful for development

---

## Implementation Files

### mc_interface.c/.h (81KB)

**Purpose:** Master motor control interface - the "front door" to motor control

**Key Functions:**

```c
// Initialization
void mc_interface_init(void);

// Control Commands
void mc_interface_set_duty(float dutyCycle);        // Set PWM duty cycle (0.0 to 1.0)
void mc_interface_set_current(float current);       // Set motor current (Amps)
void mc_interface_set_brake_current(float current); // Set braking current
void mc_interface_set_pid_speed(float rpm);         // Set target RPM
void mc_interface_set_pid_pos(float pos);           // Set target position (degrees)

// Open-loop control (for testing/startup)
void mc_interface_set_openloop_current(float current, float rpm);
void mc_interface_set_openloop_phase(float current, float phase);

// Read motor state
float mc_interface_get_rpm(void);                   // Get current RPM
float mc_interface_get_duty_cycle_now(void);        // Get current duty cycle
float mc_interface_get_tot_current(void);           // Get motor current
mc_state mc_interface_get_state(void);              // Get motor state
mc_fault_code mc_interface_get_fault(void);         // Get fault code if any

// Statistics
float mc_interface_get_amp_hours(bool reset);       // Energy used (Ah)
float mc_interface_get_watt_hours(bool reset);      // Energy used (Wh)
int mc_interface_get_tachometer_value(bool reset);  // Position counter

// Motor selection (for dual-motor hardware)
void mc_interface_select_motor_thread(int motor);   // Switch between motor 1 & 2
```

**How it Works:**
- Acts as a dispatcher to the actual motor control implementation
- Checks which motor type is configured (BLDC vs FOC)
- Routes function calls to `mcpwm_*()` or `mcpwm_foc_*()` functions
- Handles thread synchronization for dual-motor setups

**Example from mc_interface.c:**

```c
void mc_interface_set_current(float current) {
    if (motor_now() == 2) {
        // Handle motor 2
    }

    // Select implementation based on motor type
    switch (conf_now->motor_type) {
        case MOTOR_TYPE_BLDC:
            mcpwm_set_current(current);
            break;
        case MOTOR_TYPE_FOC:
            mcpwm_foc_set_current(current);
            break;
        default:
            break;
    }
}
```

---

### mcpwm.c/.h (81KB)

**Purpose:** Trapezoidal BLDC motor control implementation

**Key Concepts:**
- 6-step commutation
- Hall sensor or sensorless (back-EMF) operation
- PWM generation for 3-phase inverter
- Current control using ADC feedback

**Key Functions:**

```c
void mcpwm_init(volatile mc_configuration *configuration);
void mcpwm_set_duty(float dutyCycle);
void mcpwm_set_current(float current);
void mcpwm_set_pid_speed(float rpm);
float mcpwm_get_rpm(void);
mc_state mcpwm_get_state(void);

// Hall sensor functions
void mcpwm_init_hall_table(int8_t *table);
int mcpwm_read_hall_phase(void);
```

**Commutation Logic:**
- Reads Hall sensors or detects zero-crossings in back-EMF
- Determines current commutation step (1-6)
- Energizes appropriate phases
- Controls current through PWM duty cycle modulation

---

### mcpwm_foc.c/.h (172KB)

**Purpose:** Field-Oriented Control (FOC) implementation - the most complex and capable control method

**Key Features:**
- Clarke and Park coordinate transformations
- PI current controllers for Id and Iq
- Space Vector Modulation (SVM) for PWM generation
- Multiple observer implementations for sensorless operation
- High-Frequency Injection (HFI) for low-speed sensorless
- Support for encoders and Hall sensors
- Motor parameter measurement (R, L, flux linkage)

**Key Functions:**

```c
// Initialization
void mcpwm_foc_init(mc_configuration *conf_m1, mc_configuration *conf_m2);
bool mcpwm_foc_init_done(void);

// Control commands
void mcpwm_foc_set_current(float current);
void mcpwm_foc_set_duty(float dutyCycle);
void mcpwm_foc_set_pid_speed(float rpm);
void mcpwm_foc_set_pid_pos(float pos);
void mcpwm_foc_set_openloop_current(float current, float rpm);

// Read FOC-specific parameters
float mcpwm_foc_get_id(void);              // Get d-axis current
float mcpwm_foc_get_iq(void);              // Get q-axis current
float mcpwm_foc_get_vd(void);              // Get d-axis voltage
float mcpwm_foc_get_vq(void);              // Get q-axis voltage
float mcpwm_foc_get_phase(void);           // Get electrical angle
float mcpwm_foc_get_phase_observer(void);  // Get observed angle

// Motor parameter detection
int mcpwm_foc_measure_resistance(float current, int samples, bool stop_after, float *resistance);
int mcpwm_foc_measure_inductance(float duty, int samples, float *curr, float *ld_lq_diff, float *inductance);
int mcpwm_foc_encoder_detect(float current, bool print, float *offset, float *ratio, bool *inverted);
```

**FOC Control Loop (executed every PWM cycle - typically 20-40 kHz):**

1. **ADC Sampling**: Read phase currents (Ia, Ib, Ic)
2. **Clarke Transform**: Convert to α-β frame
3. **Park Transform**: Convert to d-q frame using rotor angle
4. **Current Controllers**:
   - PI controller for Id (typically targets 0)
   - PI controller for Iq (controls torque)
5. **Inverse Park Transform**: Convert Vd, Vq back to Vα, Vβ
6. **Space Vector Modulation**: Calculate PWM duty cycles for each phase
7. **Update PWM timers**: Set new duty cycles
8. **Observer Update**: Estimate rotor position for next cycle

---

### foc_math.c/.h (25KB)

**Purpose:** Mathematical functions optimized for FOC calculations

**Key Functions:**

```c
// Angle normalization
float utils_angle_difference(float angle1, float angle2);
void utils_norm_angle(float *angle);

// Fast trigonometric functions (using lookup tables)
float utils_fast_sin(float angle);
float utils_fast_cos(float angle);

// Coordinate transformations
void foc_clarke(float ia, float ib, float *alpha, float *beta);
void foc_park(float alpha, float beta, float angle, float *d, float *q);
void foc_inv_park(float d, float q, float angle, float *alpha, float *beta);

// Current reconstruction
void foc_reconstruct_currents(float va, float vb, float vdc, float *ia, float *ib, float *ic);
```

**Optimizations:**
- Lookup tables for sine/cosine (faster than calculating)
- Fixed-point arithmetic where appropriate
- Minimized floating-point divisions

---

### virtual_motor.c/.h (14KB)

**Purpose:** Virtual motor simulation for testing control algorithms without hardware

**Key Features:**
- Simulates motor electrical and mechanical behavior
- Models back-EMF, inductance, resistance
- Provides realistic feedback for testing
- Useful during development and algorithm validation

**When to Use:**
- Testing new control algorithms
- Validating parameter changes
- Development without hardware
- Education and demonstration

---

## Control Modes

The VESC supports multiple control modes, each suitable for different applications. The control mode is specified using the `mc_control_mode` enum.

### 1. Duty Cycle Control (`CONTROL_MODE_DUTY`)

**Function:** `mc_interface_set_duty(float dutyCycle)`

**Description:** Directly controls the PWM duty cycle (0.0 to 1.0)

**Use Cases:**
- Direct voltage control
- Testing and calibration
- Simple open-loop control

**Example:**
```c
// Set 50% duty cycle
mc_interface_set_duty(0.5);
```

**Characteristics:**
- No feedback control
- Motor speed depends on load
- Highest responsiveness
- Requires external regulation

---

### 2. Current Control (`CONTROL_MODE_CURRENT`, `CONTROL_MODE_BRAKE_CURRENT`)

**Functions:**
- `mc_interface_set_current(float current)` - Motor current (Amps)
- `mc_interface_set_brake_current(float current)` - Braking current (Amps)

**Description:** Controls motor torque by regulating current

**Use Cases:**
- Torque control
- Gentle acceleration/deceleration
- Current limiting
- Most e-bike/e-scooter applications

**Example:**
```c
// Apply 10A motor current
mc_interface_set_current(10.0);

// Apply 5A braking current
mc_interface_set_brake_current(5.0);
```

**Characteristics:**
- **FOC Mode:** Controls Iq (torque-producing current)
- Torque proportional to current: `Torque ≈ Kt × Current`
- Speed varies with load
- Smooth, predictable torque
- Excellent for EVs and robotics

---

### 3. Speed Control (`CONTROL_MODE_SPEED`)

**Function:** `mc_interface_set_pid_speed(float rpm)`

**Description:** Maintains target RPM using a PID controller

**Use Cases:**
- Constant speed applications
- Cruise control
- Fans, pumps, conveyor belts
- Any application requiring speed regulation

**Example:**
```c
// Set target speed to 3000 ERPM (electrical RPM)
mc_interface_set_pid_speed(3000.0);
```

**Characteristics:**
- PID controller adjusts current to maintain speed
- Compensates for load changes
- Tunable gains (Kp, Ki, Kd)
- Inner current control loop for torque
- Requires proper PID tuning

**Control Structure:**
```
Target RPM → [Speed PID] → Target Current → [Current Controller] → Motor
                  ↑                                                    |
                  └────────────────────────────────────────────────────┘
                                    Measured RPM
```

---

### 4. Position Control (`CONTROL_MODE_POS`)

**Function:** `mc_interface_set_pid_pos(float pos)`

**Description:** Servo-like position control using cascaded PID controllers

**Use Cases:**
- Robotic joints
- Steering actuators
- Antenna positioning
- CNC machines
- Any servo application

**Example:**
```c
// Move to position 90 degrees
mc_interface_set_pid_pos(90.0);
```

**Characteristics:**
- Requires position feedback (encoder)
- Cascaded control: Position → Speed → Current
- Highly accurate positioning
- Tunable position and speed PID gains

**Control Structure:**
```
Target Pos → [Pos PID] → Target RPM → [Speed PID] → Target Current → Motor
               ↑                           ↑                            |
               └─────────────────────────────────────────────────────────┘
                        Measured Position & Speed
```

---

### 5. Handbrake Mode (`CONTROL_MODE_HANDBRAKE`)

**Function:** `mc_interface_set_handbrake(float current)`

**Description:** Holds motor position using current

**Use Cases:**
- Parking brake
- Holding position on slopes
- Emergency stop
- Preventing rollback

**Example:**
```c
// Engage handbrake with 20A holding current
mc_interface_set_handbrake(20.0);
```

**Characteristics:**
- Actively holds position
- Consumes power (battery drain)
- Can generate significant heat
- Not suitable for long-term holding

---

### 6. Open-Loop Modes

These modes are used for testing, startup, and special applications:

#### Open-Loop Current/RPM
```c
mc_interface_set_openloop_current(float current, float rpm);
```
- Spins motor at fixed RPM with specified current
- No feedback required
- Used for initial motor startup in sensorless mode

#### Open-Loop Phase
```c
mc_interface_set_openloop_phase(float current, float phase);
```
- Applies current at specific electrical angle
- Used for position detection and calibration

#### Open-Loop Duty Cycle
```c
mc_interface_set_openloop_duty(float dutyCycle, float rpm);
mc_interface_set_openloop_duty_phase(float dutyCycle, float phase);
```
- Direct voltage control at specified angle/speed
- Used for testing and calibration

---

## Motor Configuration Parameters

Configuration is stored in the `mc_configuration` structure defined in `datatypes.h`. Key parameters are defined in `motor/mcconf_default.h`.

### Current Limits

| Parameter | Default | Description |
|-----------|---------|-------------|
| `l_current_max` | 60.0 A | Maximum motor current (acceleration) |
| `l_current_min` | -60.0 A | Minimum motor current (braking) |
| `l_in_current_max` | 99.0 A | Maximum input (battery) current |
| `l_in_current_min` | -60.0 A | Minimum input current (regen) |
| `l_max_abs_current` | 130.0 A | Absolute current limit (fault trigger) |

### Voltage Limits

| Parameter | Default | Description |
|-----------|---------|-------------|
| `l_min_voltage` | 8.0 V | Minimum battery voltage |
| `l_max_voltage` | 57.0 V | Maximum battery voltage (fault trigger) |
| `l_battery_cut_start` | 10.0 V | Begin current limiting |
| `l_battery_cut_end` | 8.0 V | Full current cutoff |

### Speed Limits

| Parameter | Default | Description |
|-----------|---------|-------------|
| `l_rpm_max` | 100,000 ERPM | Maximum motor speed |
| `l_rpm_min` | -100,000 ERPM | Minimum motor speed (reverse) |
| `l_min_duty` | 0.005 | Minimum PWM duty cycle |
| `l_max_duty` | 0.95 | Maximum PWM duty cycle |

### Temperature Limits

| Parameter | Default | Description |
|-----------|---------|-------------|
| `l_lim_temp_fet_start` | 85°C | Begin MOSFET current limiting |
| `l_lim_temp_fet_end` | 100°C | Full MOSFET shutdown |
| `l_lim_temp_motor_start` | 85°C | Begin motor current limiting |
| `l_lim_temp_motor_end` | 100°C | Full motor shutdown |

### Speed PID Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `s_pid_kp` | 0.004 | Proportional gain |
| `s_pid_ki` | 0.004 | Integral gain |
| `s_pid_kd` | 0.0001 | Derivative gain |
| `s_pid_kd_filter` | 0.2 | Derivative filter coefficient |

**PID Tuning Tips:**
- Start with Kp only, increase until oscillation, then reduce by 50%
- Add Ki to eliminate steady-state error
- Add Kd for faster response (use with caution - amplifies noise)
- Adjust `s_pid_kd_filter` to smooth derivative term

### FOC-Specific Parameters

| Parameter | Description |
|-----------|-------------|
| `foc_sensor_mode` | Sensorless, Encoder, Hall, or HFI variants |
| `foc_current_kp` | Current loop proportional gain |
| `foc_current_ki` | Current loop integral gain |
| `foc_motor_r` | Motor resistance (mΩ) |
| `foc_motor_l` | Motor inductance (μH) |
| `foc_motor_flux_linkage` | Motor flux linkage |
| `foc_observer_gain` | Sensorless observer gain |

---

## Using the Motor Control API

### Example 1: Simple Current Control

```c
#include "mc_interface.h"

void my_application(void) {
    // Initialize motor control
    mc_interface_init();

    // Wait for calibration to complete
    while (!mc_interface_dccal_done()) {
        chThdSleepMilliseconds(1);
    }

    // Apply 15A of motor current
    mc_interface_set_current(15.0);

    // Run for 5 seconds
    chThdSleepMilliseconds(5000);

    // Brake with 10A
    mc_interface_set_brake_current(10.0);

    // Release motor
    mc_interface_release_motor();
}
```

### Example 2: Speed Control with Monitoring

```c
#include "mc_interface.h"

void speed_control_example(void) {
    float target_rpm = 5000.0;
    float current_rpm;
    float current;

    // Set target speed
    mc_interface_set_pid_speed(target_rpm);

    // Monitor motor state
    for (int i = 0; i < 100; i++) {
        current_rpm = mc_interface_get_rpm();
        current = mc_interface_get_tot_current();

        // Check for faults
        mc_fault_code fault = mc_interface_get_fault();
        if (fault != FAULT_CODE_NONE) {
            // Handle fault
            const char* fault_str = mc_interface_fault_to_string(fault);
            // Log error...
            break;
        }

        chThdSleepMilliseconds(100);
    }

    mc_interface_release_motor();
}
```

### Example 3: Position Control (Servo Mode)

```c
#include "mc_interface.h"

void position_control_example(void) {
    float target_positions[] = {0.0, 90.0, 180.0, 270.0, 360.0};

    for (int i = 0; i < 5; i++) {
        // Move to target position
        mc_interface_set_pid_pos(target_positions[i]);

        // Wait for position to be reached
        float error;
        do {
            float current_pos = mc_interface_get_pid_pos_now();
            error = fabsf(target_positions[i] - current_pos);
            chThdSleepMilliseconds(10);
        } while (error > 1.0); // Within 1 degree

        // Hold position for 2 seconds
        chThdSleepMilliseconds(2000);
    }

    mc_interface_release_motor();
}
```

### Example 4: Reading Motor Statistics

```c
#include "mc_interface.h"

void read_motor_stats(void) {
    // Read current state
    mc_state state = mc_interface_get_state();
    float rpm = mc_interface_get_rpm();
    float duty = mc_interface_get_duty_cycle_now();
    float current = mc_interface_get_tot_current();
    float voltage = mc_interface_get_input_voltage_filtered();

    // Read temperatures
    float temp_fet = mc_interface_temp_fet_filtered();
    float temp_motor = mc_interface_temp_motor_filtered();

    // Read energy consumption
    float amp_hours = mc_interface_get_amp_hours(false);
    float watt_hours = mc_interface_get_watt_hours(false);

    // Read position
    int tachometer = mc_interface_get_tachometer_value(false);
    float distance = mc_interface_get_distance();

    // For FOC mode, read d-q currents
    if (state == MC_STATE_RUNNING) {
        float id = mc_interface_read_reset_avg_id();
        float iq = mc_interface_read_reset_avg_iq();
        // Process FOC-specific data...
    }
}
```

---

## Safety Features and Fault Protection

The motor control system includes comprehensive safety features to protect hardware and users.

### Fault Detection

All faults trigger immediate motor shutdown and are reported via `mc_interface_get_fault()`:

| Fault Code | Description | Cause |
|------------|-------------|-------|
| `FAULT_CODE_OVER_VOLTAGE` | Battery voltage too high | Regenerative braking, disconnected load |
| `FAULT_CODE_UNDER_VOLTAGE` | Battery voltage too low | Discharged battery, poor connection |
| `FAULT_CODE_ABS_OVER_CURRENT` | Excessive current | Short circuit, motor stall |
| `FAULT_CODE_OVER_TEMP_FET` | MOSFETs overheating | Excessive current, poor cooling |
| `FAULT_CODE_OVER_TEMP_MOTOR` | Motor overheating | Continuous high current |
| `FAULT_CODE_DRV` | Gate driver fault | Hardware failure, shoot-through |
| `FAULT_CODE_ENCODER_SPI` | Encoder communication error | Wiring issue, EMI |
| `FAULT_CODE_UNBALANCED_CURRENTS` | Phase current mismatch | Hardware failure, poor calibration |

### Soft Limits

The firmware implements soft limiting before hard faults:

1. **Current Limiting**:
   - Gradually reduces current as temperature approaches limit
   - Smooth transition prevents sudden loss of power

2. **Voltage Limiting**:
   - `battery_cut_start` to `battery_cut_end` provides gradual current reduction
   - Prevents battery over-discharge
   - Allows safe shutdown rather than sudden cutoff

3. **Temperature Derating**:
   - Linear reduction from `temp_start` to `temp_end`
   - Prevents thermal shutdown during normal operation
   - `temp_accel_dec` provides extra margin during acceleration

### Current Measurement and Calibration

**DC Current Calibration (`mcpwm_foc_dc_cal()`):**
- Performed at startup
- Measures ADC offsets for all three phase current sensors
- Critical for accurate current measurement
- Must complete before motor operation

**Checking Calibration Status:**
```c
if (mc_interface_dccal_done()) {
    // Safe to start motor
}
```

### Deadtime Compensation

**Problem:** Gate driver deadtime (time when both high and low side MOSFETs are off) causes voltage errors

**Solution:** Firmware compensates by adjusting PWM duty cycles based on current direction

**Configuration:** `HW_DEAD_TIME_NSEC` (typically 360ns)

---

## Advanced Topics

### Observer-Based Sensorless Control

**Purpose:** Estimate rotor position without sensors

**Methods Available:**
1. **Ortega Original** - Classic nonlinear observer
2. **MxLemming** - Modified observer with better low-speed performance
3. **Ortega Lambda Comp** - Ortega with flux linkage compensation
4. **MxLemming Lambda Comp** - MxLemming with compensation
5. **MXV** - Voltage-based observer
6. **MXV variants** - With compensation and linearization

**Key Parameters:**
- `foc_observer_gain` - Higher gain = faster response, more noise sensitivity
- `foc_observer_type` - Select observer algorithm

**Limitations:**
- Requires sufficient back-EMF (typically > 5% duty cycle)
- Poor performance at very low speeds
- Requires accurate motor parameters

### High-Frequency Injection (HFI)

**Purpose:** Enable sensorless operation at low speeds and standstill

**Principle:**
- Injects high-frequency signal into motor
- Detects rotor position from magnetic saliency
- Works even at zero speed

**HFI Variants:**
- **HFI_V2**: Original implementation
- **HFI_V3**: Improved for interior permanent magnet motors
- **HFI_V4**: Better noise rejection
- **HFI_V5**: Optimized for surface-mount motors

**Requirements:**
- Motor must have magnetic saliency (not all motors work)
- Proper HFI voltage and frequency tuning
- May produce audible noise

### Motor Parameter Detection

VESC can automatically measure motor parameters:

```c
// Measure resistance
float resistance;
mcpwm_foc_measure_resistance(4.0, 1000, true, &resistance);

// Measure inductance
float inductance, ld_lq_diff, current;
mcpwm_foc_measure_inductance(0.2, 1000, &current, &ld_lq_diff, &inductance);

// Combined R-L measurement
float res, ind, ld_lq;
mcpwm_foc_measure_res_ind(&res, &ind, &ld_lq);
```

**When to Use:**
- Setting up new motor
- After hardware changes
- Optimizing FOC performance

---

## Performance Considerations

### PWM Frequency

**Typical Values:** 20-40 kHz

**Trade-offs:**
- **Higher frequency:**
  - Less audible noise
  - Smoother current waveforms
  - More switching losses
  - Higher EMI

- **Lower frequency:**
  - Lower switching losses
  - More efficient
  - Audible noise
  - Higher current ripple

**Configuration:** Set in hardware configuration files

### Control Loop Timing

| Loop | Frequency | Period | Purpose |
|------|-----------|--------|---------|
| Current (FOC) | 20-40 kHz | 25-50 μs | Current control, FOC transforms |
| Speed PID | 1 kHz | 1 ms | Speed regulation |
| Position PID | 1 kHz | 1 ms | Position control |

### CPU Load

**FOC is computationally intensive:**
- Clarke/Park transforms
- Trigonometric functions
- PI controllers
- Observer calculations
- Space vector modulation

**Optimizations in VESC:**
- Lookup tables for sine/cosine
- Efficient fixed-point arithmetic
- Optimized assembly for critical paths
- DMA for ADC sampling

---

## Troubleshooting Common Issues

### Motor Won't Spin

1. **Check Fault Status:**
   ```c
   mc_fault_code fault = mc_interface_get_fault();
   ```
   - Address any fault codes

2. **Verify Calibration:**
   ```c
   if (!mc_interface_dccal_done()) {
       // Wait for calibration
   }
   ```

3. **Check Configuration:**
   - Motor type (BLDC vs FOC)
   - Correct pole count
   - Proper sensor mode

4. **Test Motor Detection:**
   - Use VESC Tool motor detection wizard
   - Verify phase wire connections

### Cogging/Rough Operation

1. **FOC Mode:**
   - Run motor detection to measure R, L, flux linkage
   - Check encoder offset/direction
   - Verify current sensor calibration

2. **BLDC Mode:**
   - Check Hall sensor connections
   - Run Hall sensor detection
   - Verify Hall sensor table

### Excessive Heat

1. **Check Current Limits:**
   - Reduce `l_current_max`
   - Verify motor current rating

2. **Improve Cooling:**
   - Add heatsinks
   - Increase airflow
   - Reduce duty cycle

3. **Check Efficiency:**
   - FOC typically more efficient than BLDC mode
   - Optimize motor parameters

### Position/Speed Oscillation

1. **Reduce PID Gains:**
   - Lower Kp first
   - Reduce or remove Kd
   - May need to lower Ki

2. **Add Filtering:**
   - Increase `s_pid_kd_filter`
   - Enable current filtering

3. **Check Mechanical:**
   - Verify encoder mounting
   - Check for mechanical play/backlash

---

## Summary

The VESC motor control system provides:

✅ **Multiple Control Methods:**
- Trapezoidal BLDC (simple, reliable)
- Advanced FOC (smooth, efficient)

✅ **Flexible Control Modes:**
- Duty cycle, current, speed, position
- Open-loop and closed-loop

✅ **Safety Features:**
- Comprehensive fault detection
- Soft limiting with thermal derating
- Overcurrent/overvoltage protection

✅ **Advanced Features:**
- Sensorless operation with observers
- HFI for low-speed/standstill
- Automatic parameter detection
- Real-time statistics and telemetry

✅ **Production-Ready:**
- Proven in thousands of applications
- Extensive testing and validation
- Active development and community support

**Key Files Reference:**
- `motor/mc_interface.c/.h` - Main API (mc_interface.c:28)
- `motor/mcpwm_foc.c/.h` - FOC implementation (mcpwm_foc.c:29)
- `motor/mcpwm.c/.h` - BLDC implementation (mcpwm.c:26)
- `motor/foc_math.c/.h` - FOC mathematics (foc_math.c)
- `motor/mcconf_default.h` - Configuration defaults (mcconf_default.h:23)

