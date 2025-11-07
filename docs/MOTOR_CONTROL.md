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

