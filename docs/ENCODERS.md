# Encoder System Documentation

## Table of Contents

1. [Introduction](#introduction)
2. [Encoder Theory](#encoder-theory)
3. [Encoder Types Overview](#encoder-types-overview)
4. [Absolute Encoders](#absolute-encoders)
5. [Incremental Encoders](#incremental-encoders)
6. [Analog Encoders](#analog-encoders)
7. [Resolver Encoders](#resolver-encoders)
8. [Serial Protocol Encoders](#serial-protocol-encoders)
9. [Encoder Configuration](#encoder-configuration)
10. [Implementation Details](#implementation-details)
11. [Troubleshooting](#troubleshooting)

---

## Introduction

The `/encoder/` directory implements support for 10+ different encoder types, providing precise position feedback for Field-Oriented Control (FOC). Encoders are critical for:

- **Accurate position control** - Servo-like positioning
- **Improved FOC performance** - Better low-speed operation than sensorless
- **Eliminating cogging** - Smooth torque at all speeds
- **Multiturn position tracking** - Know absolute position across rotations

**Why Encoders Matter:**

In FOC mode, the controller needs to know the rotor's electrical angle to properly align the magnetic field. While sensorless observers can estimate this from back-EMF, encoders provide direct, accurate measurements:

- **Resolution:** Typically 12-14 bits (4096-16384 positions per revolution)
- **Update Rate:** Up to 100 kHz for fast-sampling encoders
- **Accuracy:** 0.1° or better
- **Reliability:** Works at zero speed and standstill (unlike sensorless)

---

## Encoder Theory

### What is an Encoder?

An encoder is a sensor that converts rotational position into an electrical signal. There are two main categories:

#### 1. Incremental Encoders

Generate pulses as the shaft rotates. Position is determined by counting pulses from a known reference point.

**Characteristics:**
- Relative position only
- Require homing to find absolute position
- Simple, low-cost
- High resolution possible

**Example:** ABI quadrature encoder

#### 2. Absolute Encoders

Provide unique position code for every shaft angle. Know absolute position immediately on power-up.

**Characteristics:**
- Absolute position without homing
- More complex electronics
- Higher cost
- Various interfaces (SPI, SSI, BiSS, etc.)

**Examples:** AS5047, MT6816, TLE5012

### Position Resolution

Encoder resolution is typically specified in bits:

| Bits | Positions/Rev | Angular Resolution |
|------|---------------|-------------------|
| 10 | 1,024 | 0.352° |
| 12 | 4,096 | 0.088° |
| 14 | 16,384 | 0.022° |
| 16 | 65,536 | 0.0055° |

**For FOC:** Higher resolution = smoother low-speed operation and better position control accuracy.

### Magnetic vs Optical Encoders

**Magnetic Encoders:**
- Sense magnetic field from rotating magnet
- Robust, immune to dust/dirt
- Work in harsh environments
- Most VESC applications use magnetic

**Optical Encoders:**
- Use light and optical disk
- Very high resolution possible
- Sensitive to contamination
- Require clean environment

### Single-turn vs Multi-turn

**Single-turn:**
- Track position within one revolution (0-360°)
- Most common for motor control

**Multi-turn:**
- Track position across multiple revolutions
- Useful for linear actuators, robotics
- May use gears or electronic counting

---

## Encoder Types Overview

The VESC firmware supports these encoder types:

| Encoder Type | Interface | Resolution | Speed | Multi-turn | Cost |
|--------------|-----------|------------|-------|------------|------|
| **AS5047/AS5048** | SPI | 14-bit | Fast | Software | $$$ |
| **AS5147U/AS5247U** | SPI/ABI | 14-bit | Fast | Hardware (5247) | $$$ |
| **MT6816** | SPI | 14-bit | Fast | No | $$ |
| **TLE5012** | SPI | 15-bit | Fast | No | $$ |
| **AD2S1205** | SPI (Resolver) | 12-16 bit | Medium | No | $$$ |
| **BiSSC** | BiSS-C | Variable | Fast | Yes | $$$ |
| **TS5700N8501** | UART | 13-bit | Medium | Yes | $$ |
| **ABI Quadrature** | Digital I/O | Variable | Fast | Software | $ |
| **Sin/Cos** | Analog | Variable | Fast | No | $$ |
| **PWM** | PWM Input | Variable | Slow | No | $ |

---

## Absolute Encoders

Absolute encoders provide instant position readout without requiring a reference point.

### AS5047 / AS5048 (Austria Microsystems)

**File:** `enc_as504x.c/.h`

#### Specifications

- **Resolution:** 14-bit (16,384 positions/rev)
- **Interface:** SPI (up to 10 MHz)
- **Accuracy:** ±0.07° (12-bit mode), ±0.042° (14-bit mode)
- **Update Rate:** ~28 kHz maximum
- **Magnetic Field:** 30-70 mT typical
- **Power:** 3.3V or 5V

#### Features

- On-chip DSP for automatic gain control
- Magnetic field strength monitoring
- Error detection (weak/strong field)
- Optional PWM output
- AS5047: SPI only
- AS5048: SPI + PWM output

#### SPI Communication

**Command Structure:**
```
┌────────────────────────────────────┐
│  CMD  │  Parity │    Address       │
│ (R/W) │   (1)   │    (14 bits)     │
└────────────────────────────────────┘
```

**Read Angle:**
```c
// Read 14-bit angle from AS5047
uint16_t spi_val = enc_as504x_read_angle(cfg);
float angle_deg = (spi_val & 0x3FFF) * (360.0 / 16384.0);
```

#### Diagnostics

The AS5047/48 provides comprehensive diagnostics:

```c
AS504x_diag *diag = &cfg->state.sensor_diag;

// Check connection
if (!diag->is_connected) {
    // Encoder not responding
}

// Check magnetic field
if (diag->is_Comp_high) {
    // Magnet too strong
}
if (diag->is_Comp_low) {
    // Magnet too weak
}

// Automatic Gain Control value (0-255)
uint8_t agc = diag->AGC_value;
if (agc < 50 || agc > 200) {
    // Suboptimal magnetic field
}

// Magnetic field magnitude
uint16_t magnitude = diag->magnitude;
```

#### Typical Wiring

```
AS5047         VESC
┌──────┐      ┌──────┐
│ VDD  ├──────┤ 3.3V │
│ GND  ├──────┤ GND  │
│ MOSI ├──────┤ MOSI │
│ MISO ├──────┤ MISO │
│ CLK  ├──────┤ SCK  │
│ CSn  ├──────┤ NSS  │
└──────┘      └──────┘

Diametric magnet on shaft
positioned 0.5-2mm from IC
```

---

### AS5147U / AS5247U

**File:** `enc_as5x47u.c/.h`

#### Specifications

- **Resolution:** 14-bit (16,384 positions/rev)
- **Interface:** SPI, ABI, UVW, PWM (configurable)
- **Accuracy:** ±0.045° RMS
- **Update Rate:** 28.6 kHz (SPI)
- **Multi-turn:** AS5247U has 12-bit multi-turn counter
- **Daisy-chain:** Multiple encoders on one SPI bus

#### Advantages over AS5047

- Hardware multi-turn counting (AS5247U)
- On-chip angle velocity calculation
- Configurable outputs (ABI, UVW, PWM)
- Better diagnostic features
- Lower power consumption

#### SPI Protocol

Uses a different protocol than AS5047:

```c
// Read angle with parity check
uint16_t angle = enc_as5x47u_read_angle(cfg);

// Check error flags
AS5x47U_diag *diag = &cfg->state.sensor_diag;
if (diag->is_error) {
    // Various error conditions
    if (diag->is_broken_hall) {
        // Hall sensor failure
    }
    if (diag->is_crc_error) {
        // CRC error in communication
    }
}
```

#### Multi-turn Configuration (AS5247U)

```c
// Read single-turn angle (0-360°)
float angle = encoder_read_deg();

// Read multi-turn position (-N to +N revolutions)
float multiturn_angle = encoder_read_deg_multiturn();

// Reset multi-turn counter
encoder_reset_multiturn();
```

---

### MT6816 (MagnTek)

**File:** `enc_mt6816.c/.h`

#### Specifications

- **Resolution:** 14-bit (16,384 positions/rev)
- **Interface:** SPI (up to 12 MHz)
- **Accuracy:** ±0.1°
- **Update Rate:** 10 kHz typical
- **Power:** 3.0-3.6V
- **Cost:** Lower than AS504x series

#### Features

- Simple SPI interface
- No configuration required
- Magnetic field status
- Good value for cost

#### SPI Read Operation

**Frame Format:**
```
┌────────────────────────────────┐
│ Angle(13:6) │ Angle(5:0) │ ST │
│   8 bits    │   6 bits   │ 2  │
└────────────────────────────────┘
```

**Status Bits:**
- `ST[1]`: Magnetic field too strong
- `ST[0]`: Magnetic field too weak

```c
// Read angle
float angle = encoder_read_deg();

// Check magnetic field status
MT6816_state *state = &cfg->state;
if (state->encoder_no_magnet_error_cnt > 0) {
    // Magnet positioning issue
    float error_rate = state->encoder_no_magnet_error_rate;
}
```

---

### TLE5012 (Infineon)

**File:** `enc_tle5012.c/.h`

#### Specifications

- **Resolution:** 15-bit (32,768 positions/rev)
- **Interface:** SPI (SSC protocol)
- **Accuracy:** ±0.5° typical
- **Update Rate:** Up to 100 kHz
- **Temperature Range:** -40°C to +150°C
- **Automotive Grade:** AEC-Q100 qualified

#### Features

- Highest resolution (15-bit)
- Very fast update rate
- Integrated temperature sensor
- On-chip angle calculation and filtering
- Predictive angle calculation
- GMR (Giant Magneto-Resistive) technology

#### SSC Protocol

TLE5012 uses Infineon's SSC (Short Serial Communication) protocol:

```c
// Initialize
TLE5012_config_t cfg;
enc_tle5012_init(&cfg);

// Read angle
float angle = encoder_read_deg();

// Check for errors
TLE5012_state *state = &cfg.state;
switch (state->last_status_error) {
    case NO_ERROR:
        // Normal operation
        break;
    case SYSTEM_ERROR:
        // Power supply or ROM issue
        break;
    case INVALID_ANGLE_ERROR:
        // No magnet detected
        break;
    case CRC_ERROR:
        // Communication error
        break;
}
```

#### Advanced Features

```c
// Temperature sensor reading (if supported by hardware)
// Angle velocity (calculated by encoder IC)
// Predictive angle for compensating delays
```

---

### BiSSC (BiSS-C Protocol)

**File:** `enc_bissc.c/.h`

#### Specifications

- **Resolution:** Variable (typically 12-22 bits)
- **Interface:** BiSS-C serial protocol
- **Speed:** Up to 10 MHz clock
- **Multi-turn:** Yes (typically 12-bit)
- **Standardized:** Open industrial standard

#### BiSS-C Protocol

**Bidirectional Synchronous Serial (BiSS):**

```
Master → Slave: Clock + Start
Slave → Master: Status + Position + CRC

┌──────┬───────────────────────┬─────┐
│ ACK  │ Error │ Warn │ Angle  │ CRC │
└──────┴───────────────────────┴─────┘
```

**Features:**
- CRC error detection
- Warning and error flags
- Register read/write for configuration
- Multi-turn position

```c
// Read BiSS encoder
float angle = encoder_read_deg();
float multiturn = encoder_read_deg_multiturn();

// Check communication quality
BISSC_state *state = &cfg->state;
float comm_error_rate = state->spi_comm_error_rate;
float data_error_rate = state->spi_data_error_rate;

if (comm_error_rate > 0.01) {
    // More than 1% communication errors
    // Check wiring, cable length, clock speed
}
```

---

## Incremental Encoders

### ABI Quadrature Encoder

**File:** `enc_abi.c/.h`

#### Theory

Incremental encoders use two channels (A and B) that are 90° out of phase:

```
     ___     ___     ___
A __|   |___|   |___|   |___

  ___     ___     ___
B|   |___|   |___|   |___

Position:  1   2   3   4   (counting edges)
Direction: →  →  →  →     (A leads B = clockwise)
```

**Quadrature Decoding:**
- Counting both edges of both channels: 4× resolution
- Direction determined by which signal leads

**Index Pulse (I):**
- One pulse per revolution
- Used for homing/reference point
- Resets position counter

#### Specifications

- **Resolution:** Depends on encoder (100-10,000 PPR common)
- **Interface:** 3 digital inputs (A, B, I)
- **Speed:** Limited by MCU timer frequency
- **Multi-turn:** Software counting

#### STM32 Timer Encoder Mode

The VESC uses hardware timer in encoder mode for quadrature counting:

```c
// Initialize ABI encoder
ABI_config_t cfg;
cfg.counts = 2048;  // 2048 pulses per revolution
cfg.timer = TIM3;   // Use Timer 3
enc_abi_init(&cfg);

// Read angle (hardware counts automatically)
float angle = enc_abi_read_deg(&cfg);

// Index pulse handling
void enc_abi_pin_isr(ABI_config_t *cfg) {
    // Called when index pulse detected
    cfg->state.index_found = true;
    // Reset counter to known position
}
```

#### Advantages

- Very low cost
- High resolution possible
- Simple interface
- No SPI/communication overhead

#### Disadvantages

- Requires 3 GPIO pins (or timer input)
- No absolute position on power-up
- Need index pulse for homing
- Can lose count if pulses missed

---

## Analog Encoders

### Sin/Cos Encoder

**File:** `enc_sincos.c/.h`

#### Theory

Sin/Cos encoders output two analog voltage signals:
- **Sine output:** Voltage proportional to sin(θ)
- **Cosine output:** Voltage proportional to cos(θ)

Where θ is the mechanical angle.

```
Voltage
  ↑
  │    Sine
  │   ╱╲      ╱╲
  │  ╱  ╲    ╱  ╲
  │ ╱    ╲  ╱    ╲
  ├──────────────────→ Angle
  │╲      ╲╱      ╲╱
  │ ╲    ╱  ╲    ╱ Cosine
  │  ╲  ╱    ╲  ╱
  │   ╲╱      ╲╱
```

**Position Calculation:**
```
θ = atan2(sin, cos)
```

#### Specifications

- **Resolution:** Limited by ADC (12-bit typ = 0.088°)
- **Interface:** 2 analog inputs
- **Accuracy:** Depends on signal quality and ADC
- **Interpolation:** Can interpolate between cycles for higher resolution

#### Configuration

```c
ENCSINCOS_config_t cfg;

// Calibration parameters
cfg.s_gain = 1.0 / sin_amplitude;    // Gain compensation
cfg.c_gain = 1.0 / cos_amplitude;
cfg.s_offset = sin_offset;            // DC offset compensation
cfg.c_offset = cos_offset;
cfg.filter_constant = 0.5;            // Low-pass filtering
cfg.phase_correction = 0.0;           // Phase angle correction (deg)

// Sampling rate
cfg.refresh_rate_hz = 10000;
```

#### Signal Quality

```c
ENCSINCOS_state *state = &cfg.state;

// Check for signal faults
if (state->signal_low_error_rate > 0.01) {
    // Sine amplitude too low
}
if (state->signal_above_max_error_rate > 0.01) {
    // Cosine amplitude too high
}

// Filtered signals for visualization
float sin_filtered = state->sin_filter;
float cos_filtered = state->cos_filter;
```

#### Advantages

- Continuous analog signal
- Can achieve very high resolution with interpolation
- Immune to digital noise
- Standard in high-performance servos

#### Disadvantages

- Requires calibration (offset, gain, phase)
- Sensitive to analog noise
- ADC quality critical
- More complex signal conditioning

---

## Resolver Encoders

### AD2S1205 (Resolver to Digital Converter)

**File:** `enc_ad2s1205.c/.h`

#### What is a Resolver?

A resolver is an electromagnetic transducer similar to a rotary transformer:

**Structure:**
- Stator: Primary coil
- Rotor: Two secondary coils at 90°

**Operation:**
1. Primary coil excited with AC carrier (10-20 kHz)
2. Rotor coils output sine and cosine modulated signals
3. Amplitude varies with rotor position

**Resolver Signals:**
```
Excitation:  Ve = V₀ sin(ωt)

Sine out:    Vs = Ve × sin(θ)
Cosine out:  Vc = Ve × cos(θ)
```

#### AD2S1205 R/D Converter

Converts resolver signals to digital angle:

**Features:**
- 12, 14, or 16-bit resolution
- Tracking rate: 3125 RPS max
- Velocity output
- Built-in excitation generator
- Fault detection (LOT, LOS, DOS)

#### Fault Detection

```c
AD2S1205_state *state = &cfg->state;

// Loss of Tracking (LOT): Resolver moving too fast
if (state->resolver_loss_of_tracking_error_rate > 0.01) {
    // Reduce speed or check connections
}

// Degradation of Signal (DOS): Signal quality degraded
if (state->resolver_degradation_of_signal_error_rate > 0.01) {
    // Check wiring, excitation level
}

// Loss of Signal (LOS): No signal from resolver
if (state->resolver_loss_of_signal_error_rate > 0.01) {
    // Resolver disconnected or failed
}

// SPI communication errors
float spi_error_rate = state->spi_error_rate;
```

#### Advantages

- Extremely robust (automotive, aerospace)
- No magnetic components (works in strong magnetic fields)
- High temperature operation
- Long life (no wearing parts)

#### Disadvantages

- More expensive
- Requires AC excitation
- More complex installation
- Heavier and larger than magnetic encoders

---

## Serial Protocol Encoders

### TS5700N8501 (Multi-turn Absolute)

**File:** `enc_ts5700n8501.c/.h`

#### Specifications

- **Resolution:** 13-bit single-turn (8192 pos/rev)
- **Multi-turn:** 12-bit (4096 revolutions)
- **Interface:** UART (2.5 Mbps)
- **Accuracy:** ±0.088°
- **Total positions:** 8192 × 4096 = 33,554,432

#### UART Communication

**Protocol:**
```
Request:  [Start] [Command] [CRC]
Response: [Start] [Data...] [Status] [CRC]
```

**Commands:**
- Read single-turn position
- Read multi-turn position
- Reset multi-turn counter
- Read temperature
- Read error status

```c
// Initialize UART encoder
TS5700N8501_config_t cfg;
cfg.sd = &SD3;              // UART3
cfg.uart_param.speed = 2500000;  // 2.5 Mbps
enc_ts5700n8501_init(&cfg);

// Read position (asynchronous)
float angle = encoder_read_deg();
float multiturn = encoder_read_deg_multiturn();

// Check status
TS5700N8501_state *state = &cfg.state;
uint8_t *status = state->raw_status;
// Parse status bytes for errors
```

#### Advantages

- True multi-turn absolute position
- High resolution
- UART interface (easy to implement)
- Battery-less (uses internal capacitor)

#### Disadvantages

- Slower update rate than SPI
- More expensive
- UART bandwidth limits polling rate

---

### PWM Encoder

**File:** `enc_pwm.c/.h`

#### Theory

Position encoded as PWM duty cycle:

```
Duty Cycle = (Position / 360°) × 100%

0°:      |▔|_______|
180°:    |▔▔▔▔|____|
360°:    |▔▔▔▔▔▔▔▔|
```

#### Specifications

- **Resolution:** Limited by timer resolution (12-bit typical)
- **Interface:** Single PWM input
- **Update Rate:** Depends on PWM frequency
- **Common Frequencies:** 1-10 kHz

#### Implementation

```c
// PWM input capture using timer
// Measures pulse width and period
// Calculates duty cycle
float duty = pulse_width / period;
float angle = duty * 360.0;
```

#### Advantages

- Very simple wiring (1 signal wire)
- Can use same encoder as AS5048 PWM output
- No SPI/UART required

#### Disadvantages

- Lower resolution
- Slower update rate
- More susceptible to noise
- Limited accuracy

---

## Encoder Configuration

### VESC Tool Configuration

**Motor Settings → General → Sensor Mode:**
1. **Select encoder type** from dropdown
2. **Configure encoder ratio** (if using gearing)
3. **Set encoder direction** (normal/inverted)
4. **Run encoder detection** to find offset

### Encoder Offset Detection

The electrical angle doesn't align with encoder zero, so offset must be measured:

```c
// Encoder detection procedure:
// 1. Apply DC current to align rotor at known electrical angle
// 2. Read encoder position
// 3. Calculate offset = electrical_angle - encoder_angle
// 4. Store offset in configuration

int mcpwm_foc_encoder_detect(float current, bool print,
                              float *offset, float *ratio, bool *inverted) {
    // Apply current to phases
    // Wait for rotor to settle
    // Read encoder
    // Calculate offset
    // Return results
}
```

### Encoder Ratio

For geared encoders or special mounting:

```
Motor Angle = Encoder Angle × Ratio
```

**Examples:**
- Direct mount: ratio = 1.0
- 2:1 gearbox: ratio = 2.0
- Encoder on output shaft of 5:1 gearbox: ratio = 0.2

### Inverted Direction

If encoder counts opposite to motor rotation:
- Check "Inverted" in VESC Tool
- Or set encoder ratio to negative value

---

## Implementation Details

### Encoder Interface (encoder.c/.h)

**Main API:**

```c
// Initialize encoder subsystem
bool encoder_init(volatile mc_configuration *conf);

// Read encoder angle (0-360°)
float encoder_read_deg(void);

// Read multi-turn angle
float encoder_read_deg_multiturn(void);

// Set encoder position (for homing)
void encoder_set_deg(float deg);

// Check if encoder is working
encoder_type_t encoder_is_configured(void);

// For ABI encoders: check if index found
bool encoder_index_found(void);

// Reset multi-turn counter
void encoder_reset_multiturn(void);

// Get error statistics
float encoder_get_error_rate(void);
void encoder_reset_errors(void);
```

### Custom Encoder Support

For encoders not natively supported:

```c
// Register custom encoder callbacks
void encoder_set_custom_callbacks(
    float (*read_deg)(void),
    bool (*has_fault)(void),
    char* (*print_info)(void)
);

// Example custom encoder
float my_encoder_read(void) {
    // Read your encoder hardware
    // Convert to degrees (0-360)
    return angle_deg;
}

bool my_encoder_fault(void) {
    // Check for faults
    return has_error;
}

void setup_custom_encoder(void) {
    encoder_set_custom_callbacks(
        my_encoder_read,
        my_encoder_fault,
        NULL
    );
}
```

### Error Handling

All encoders track error rates:

```c
// Get overall encoder error rate
float error_rate = encoder_get_error_rate();

if (error_rate > 0.05) {
    // More than 5% errors
    // Check wiring, power, magnetic field
}

// Reset error counters
encoder_reset_errors();
```

### Multi-turn Tracking

For encoders without hardware multi-turn:

```c
// Software multi-turn counting
static int32_t multiturn_count = 0;
static float last_angle = 0.0;

float current_angle = encoder_read_deg();
float diff = current_angle - last_angle;

if (diff > 180.0) {
    multiturn_count--;  // Wrapped backwards
} else if (diff < -180.0) {
    multiturn_count++;  // Wrapped forwards
}

last_angle = current_angle;

float total_angle = current_angle + (multiturn_count * 360.0);
```

---

## Practical Examples

### Example 1: Configure AS5047 Encoder

```c
#include "encoder/encoder.h"

void setup_as5047_encoder(void) {
    volatile mc_configuration *conf = mc_interface_get_configuration();

    // Configure encoder type
    conf->m_sensor_port_mode = SENSOR_PORT_MODE_AS5047_SPI;

    // Set encoder parameters
    conf->foc_encoder_offset = 0.0;      // Will be calibrated
    conf->foc_encoder_ratio = 1.0;       // Direct mount
    conf->foc_encoder_inverted = false;  // Not inverted

    // Initialize encoder
    if (encoder_init(conf)) {
        commands_printf("AS5047 initialized successfully");
    } else {
        commands_printf("AS5047 initialization failed!");
    }

    // Run encoder detection
    float offset, ratio;
    bool inverted;
    int res = mcpwm_foc_encoder_detect(4.0, true,
                                       &offset, &ratio, &inverted);

    if (res == 0) {
        conf->foc_encoder_offset = offset;
        conf->foc_encoder_ratio = ratio;
        conf->foc_encoder_inverted = inverted;
        commands_printf("Encoder offset: %.2f°", offset);
    }
}
```

### Example 2: Monitor Encoder Quality

```c
void check_encoder_quality(void) {
    // Get error rate
    float error_rate = encoder_get_error_rate();

    commands_printf("Encoder error rate: %.2f%%", error_rate * 100.0);

    // For AS5047
    AS504x_config_t *cfg = /* get config */;
    AS504x_diag *diag = &cfg->state.sensor_diag;

    if (diag->is_Comp_high) {
        commands_printf("Warning: Magnet too strong!");
    }
    if (diag->is_Comp_low) {
        commands_printf("Warning: Magnet too weak!");
    }

    commands_printf("AGC value: %d (should be 50-200)", diag->AGC_value);
    commands_printf("Magnitude: %d", diag->magnitude);
}
```

### Example 3: ABI Encoder with Index Homing

```c
void home_abi_encoder(void) {
    // Move motor slowly until index found
    mc_interface_set_duty(0.1);  // 10% duty

    while (!encoder_index_found()) {
        chThdSleepMilliseconds(10);
    }

    // Stop motor
    mc_interface_release_motor();

    // Set encoder to zero at index
    encoder_set_deg(0.0);

    commands_printf("Encoder homed at index pulse");
}
```

### Example 4: Multi-turn Position Tracking

```c
void track_linear_actuator_position(void) {
    // Lead screw: 5mm per revolution
    float mm_per_rev = 5.0;

    // Read total angle including multi-turn
    float total_angle = encoder_read_deg_multiturn();

    // Convert to linear position
    float revolutions = total_angle / 360.0;
    float position_mm = revolutions * mm_per_rev;

    commands_printf("Position: %.2f mm (%.1f rev)",
                   position_mm, revolutions);

    // Reset to zero
    encoder_reset_multiturn();
}
```

---

## Troubleshooting

### Encoder Not Detected

**Symptoms:** `encoder_init()` fails, no position reading

**Checks:**
1. **Power:** Verify 3.3V or 5V at encoder
2. **Wiring:** Check SPI connections (MOSI, MISO, SCK, CS)
3. **Pull-ups:** Some encoders need pull-up resistors
4. **Compatibility:** Verify encoder type matches configuration

**Debug:**
```c
if (!encoder_is_configured()) {
    commands_printf("Encoder not configured!");
    // Check hardware connections
}
```

### High Error Rate

**Symptoms:** `encoder_get_error_rate() > 5%`

**Possible Causes:**
1. **Weak magnetic field** (AS504x, MT6816, TLE5012)
   - Move magnet closer (0.5-2mm gap)
   - Use stronger magnet
   - Check AGC value

2. **Electrical noise**
   - Add ferrite beads on encoder wires
   - Route encoder cable away from motor phases
   - Use twisted pair or shielded cable

3. **High speed**
   - Reduce SPI clock speed
   - Check encoder update rate specification

4. **Poor connection**
   - Check solder joints
   - Verify cable integrity

### Incorrect Position Reading

**Symptoms:** Encoder reads but position is wrong

**Checks:**
1. **Offset not calibrated**
   - Run encoder detection
   - Verify offset value stored

2. **Inverted direction**
   - Check encoder direction setting
   - Try inverting in configuration

3. **Wrong ratio**
   - Verify encoder ratio for gearing
   - ratio = motor_rev / encoder_rev

4. **Magnet misalignment**
   - Magnet must be centered on encoder IC
   - Check for axial and radial runout

### Position Jumps or Glitches

**Symptoms:** Occasional large position errors

**Causes:**
1. **Magnetic interference**
   - Motor magnets too close to encoder magnet
   - Shield encoder from motor field

2. **Vibration**
   - Secure encoder mounting
   - Isolate from vibration

3. **SPI communication errors**
   - Check SPI clock speed
   - Verify cable length < 30cm for high-speed SPI
   - Add delay between SPI transactions

### AS5047 Specific Issues

**Magnet Strength:**
```c
// Ideal AGC range: 50-200
if (diag->AGC_value < 50) {
    // Magnet too far or too weak
    // Move magnet closer
}
if (diag->AGC_value > 200) {
    // Magnet too close or too strong
    // Move magnet farther away
}
```

**Optimal Distance:** 0.5-1.5mm for diametric magnets

### ABI Encoder Issues

**Missing Counts:**
- GPIO interrupt priority too low
- Motor speed too high for encoder PPR
- Use hardware timer encoder mode

**No Index Pulse:**
- Check index wire connection
- Verify index interrupt configured
- Some cheap encoders don't have index

### Sin/Cos Encoder Calibration

**Poor Accuracy:**

1. **Measure signal amplitudes:**
   ```c
   // Read ADC values
   float sin_min, sin_max, cos_min, cos_max;
   // Rotate motor full revolution
   // Record min/max values

   // Calculate offset and gain
   float sin_offset = (sin_max + sin_min) / 2.0;
   float sin_amplitude = (sin_max - sin_min) / 2.0;
   float sin_gain = 1.0 / sin_amplitude;
   ```

2. **Check phase alignment:**
   - Sine and cosine should be 90° apart
   - Measure actual phase difference
   - Apply phase correction

---

## Summary

The VESC encoder system provides:

✅ **10+ Encoder Types:**
- Absolute magnetic (AS5047, MT6816, TLE5012, AS5x47U)
- Serial protocols (BiSSC, UART)
- Incremental (ABI quadrature)
- Analog (Sin/Cos)
- Resolver (AD2S1205)

✅ **High Performance:**
- Up to 15-bit resolution (TLE5012)
- Update rates to 100 kHz
- Sub-degree accuracy
- Multi-turn tracking

✅ **Robust Error Handling:**
- SPI communication error detection
- Magnetic field monitoring
- Position validation
- Error rate tracking

✅ **Easy Integration:**
- Unified API across all encoder types
- Automatic offset detection
- VESC Tool configuration
- Custom encoder support

**Key Files Reference:**
- `encoder/encoder.c/.h` - Main encoder API (encoder.c:39)
- `encoder/encoder_datatype.h` - Type definitions (encoder_datatype.h:30)
- `encoder/enc_as504x.c/.h` - AS5047/AS5048 implementation
- `encoder/enc_abi.c/.h` - ABI quadrature encoder
- `encoder/enc_sincos.c/.h` - Sin/Cos analog encoder

