# IMU and Sensor Fusion Documentation

## Table of Contents
1. [Overview](#overview)
2. [IMU Theory and Concepts](#imu-theory-and-concepts)
3. [Supported IMU Sensors](#supported-imu-sensors)
4. [AHRS Algorithms](#ahrs-algorithms)
5. [Main IMU Interface](#main-imu-interface)
6. [Fusion Library](#fusion-library)
7. [Sensor Drivers](#sensor-drivers)
8. [Calibration and Tuning](#calibration-and-tuning)
9. [Practical Applications](#practical-applications)

---

## Overview

The `/imu/` directory implements Inertial Measurement Unit (IMU) support with sensor fusion for accurate orientation tracking. The system combines data from gyroscopes, accelerometers, and magnetometers to compute 3D orientation (roll, pitch, yaw) for applications like self-balancing vehicles, antenna tracking, and platform stabilization.

### Key Features

- **Multiple IMU Sensors:** MPU9150/9250, ICM20948, LSM6DS3, BMI160
- **Sensor Fusion Algorithms:** Mahony, Madgwick, and Fusion library
- **6-DOF and 9-DOF Support:** Works with or without magnetometer
- **Quaternion Output:** Numerically stable orientation representation
- **Euler Angles:** Roll, pitch, yaw in degrees
- **Linear Acceleration:** Gravity-compensated acceleration
- **Real-Time Operation:** Optimized for embedded systems

### File Structure

```
imu/
├── imu.c/h                      # Main IMU interface
├── ahrs.c/h                     # AHRS algorithms (Mahony/Madgwick)
├── mpu9150.c/h                  # MPU9150/MPU9250 driver
├── icm20948.c/h                 # ICM20948 driver
├── lsm6ds3.c/h                  # LSM6DS3 driver
├── bmi160_wrapper.c/h           # BMI160 wrapper
├── BMI160_driver/               # BMI160 vendor driver
│   ├── bmi160.c/h
│   └── bmi160_defs.h
└── Fusion/                      # Third-party Fusion library
    ├── Fusion.h                 # Main header
    ├── FusionTypes.h            # Data types
    ├── FusionAhrs.c/h           # AHRS algorithm
    ├── FusionBias.c/h           # Gyro bias correction
    └── FusionCompass.c/h        # Tilt-compensated compass
```

---

## IMU Theory and Concepts

### Inertial Measurement Unit (IMU)

An IMU combines multiple sensors to measure motion and orientation:

**6-DOF (Degrees of Freedom) IMU:**
- **3-axis Gyroscope:** Measures angular velocity (rotation rate) in degrees/second
- **3-axis Accelerometer:** Measures linear acceleration including gravity

**9-DOF IMU:**
- 6-DOF sensors +
- **3-axis Magnetometer:** Measures magnetic field strength (compass)

### Sensor Characteristics

#### Gyroscope

**Principle:** Measures angular velocity using Coriolis effect (MEMS vibrating mass)

**Output:** Rotation rate in °/s (degrees per second)
- Roll rate (rotation around X-axis)
- Pitch rate (rotation around Y-axis)
- Yaw rate (rotation around Z-axis)

**Advantages:**
- Fast response time (~1ms)
- High precision for short-term measurements
- Immune to external accelerations

**Limitations:**
- **Drift:** Integration of angular velocity accumulates errors over time
- **Bias:** Constant offset that varies with temperature
- **Typical drift:** 0.01-0.1°/s → 36-360°/hour uncorrected

#### Accelerometer

**Principle:** Measures acceleration using spring-mass system

**Output:** Acceleration in g (9.81 m/s²)
- X, Y, Z acceleration including gravity
- At rest: measures 1g downward (gravity)

**Advantages:**
- No drift (absolute reference)
- Can determine tilt angles from gravity vector
- Inexpensive

**Limitations:**
- Cannot distinguish gravity from linear acceleration
- Noisy during motion
- Cannot measure yaw (rotation around gravity vector)

**Tilt Angle from Accelerometer:**
```
When stationary:
Roll  = atan2(Ay, Az)
Pitch = atan2(-Ax, sqrt(Ay² + Az²))
```

#### Magnetometer

**Principle:** Measures Earth's magnetic field

**Output:** Magnetic field strength in µT (microtesla)
- Earth's field: 20-70 µT depending on location
- Points to magnetic north (not true north)

**Advantages:**
- Absolute heading reference
- No drift for yaw measurement
- Enables full 9-DOF orientation

**Limitations:**
- **Magnetic Distortions:** Nearby ferromagnetic materials, electrical currents
- **Hard Iron:** Permanent magnets create constant offset
- **Soft Iron:** Ferromagnetic materials distort field direction
- **Requires Calibration:** Must map min/max values in all directions

### Coordinate Frames

**Sensor Frame (Body Frame):**
```
      X (forward)
      ↑
      |
      |___→ Y (right)
     /
    /
   ↓ Z (down)
```

**Earth Frame (NED - North East Down):**
```
      N (north)
      ↑
      |
      |___→ E (east)
     /
    /
   ↓ D (down)
```

### Euler Angles

**ZYX Aerospace Sequence:**
1. **Yaw (ψ):** Rotation around Z-axis (heading, 0-360°)
2. **Pitch (θ):** Rotation around Y-axis (-90° to +90°)
3. **Roll (φ):** Rotation around X-axis (-180° to +180°)

**Gimbal Lock:**
- Occurs when pitch = ±90°
- Two axes align, losing one degree of freedom
- Solution: Use quaternions internally, convert to Euler for display

### Quaternions

A quaternion represents orientation as 4 values: **q = [w, x, y, z]**

**Properties:**
- No gimbal lock
- Computationally efficient
- Smooth interpolation
- Unit quaternion: w² + x² + y² + z² = 1

**Identity (no rotation):** q = [1, 0, 0, 0]

**Quaternion to Euler Angles:**
```c
roll  = atan2(2*(q.w*q.x + q.y*q.z), 1 - 2*(q.x² + q.y²))
pitch = asin(2*(q.w*q.y - q.z*q.x))
yaw   = atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y² + q.z²))
```

### Sensor Fusion

**Problem:** Each sensor has complementary strengths and weaknesses:
- Gyro: accurate short-term, drifts long-term
- Accel: absolute reference, noisy during motion
- Mag: absolute heading, susceptible to distortions

**Solution:** Sensor fusion combines sensors to overcome individual limitations

**Complementary Filter Concept:**
```
Orientation = α × (Gyro integration) + (1-α) × (Accel/Mag correction)
```

**AHRS (Attitude Heading Reference System):**
- Advanced sensor fusion algorithms
- Mahony: Explicit complementary filter with PI controller
- Madgwick: Gradient descent optimization
- Fusion: Adaptive gain with bias correction

---

## Supported IMU Sensors

### MPU9150 / MPU9250

**Type:** 9-DOF IMU (most common in VESC applications)

**Specifications:**
- **Gyroscope:** ±250, ±500, ±1000, ±2000 °/s
- **Accelerometer:** ±2, ±4, ±8, ±16 g
- **Magnetometer:** AK8975 (MPU9150) or AK8963 (MPU9250), ±4800 µT
- **Interface:** I2C (400kHz)
- **I2C Address:** 0x68 (AD0=LOW) or 0x69 (AD0=HIGH)
- **Sample Rate:** Up to 1kHz (accel/gyro), 100Hz (mag)
- **Package:** 4×4×1mm QFN

**Features:**
- On-chip DMP (Digital Motion Processor) - not used in VESC
- Integrated temperature sensor
- Internal I2C master for magnetometer access
- Interrupt output

**Typical Configuration:**
- Gyro: ±2000°/s
- Accel: ±8g
- Sample rate: 1000Hz
- DLPF (Digital Low Pass Filter): 92Hz

### ICM20948

**Type:** 9-DOF IMU (newer, improved version of MPU9250)

**Specifications:**
- **Gyroscope:** ±250, ±500, ±1000, ±2000 °/s
- **Accelerometer:** ±2, ±4, ±8, ±16 g
- **Magnetometer:** AK09916, ±4900 µT
- **Interface:** I2C or SPI
- **I2C Address:** 0x68 (AD0=LOW) or 0x69 (AD0=HIGH)
- **Sample Rate:** Up to 1.125kHz
- **Package:** 3×3×1mm QFN

**Improvements over MPU9250:**
- Lower noise gyroscope
- Better temperature stability
- Improved magnetometer
- Bank-based register structure

### LSM6DS3

**Type:** 6-DOF IMU (no magnetometer)

**Specifications:**
- **Gyroscope:** ±125, ±245, ±500, ±1000, ±2000 °/s
- **Accelerometer:** ±2, ±4, ±8, ±16 g
- **Interface:** I2C or SPI
- **I2C Address:** 0x6A (SA0=LOW) or 0x6B (SA0=HIGH)
- **Sample Rate:** Up to 6.6kHz
- **Package:** 2.5×3×0.83mm LGA

**Features:**
- Very low power consumption
- Hardware FIFO buffer
- Integrated pedometer/step counter
- Activity recognition

**Use Cases:**
- When magnetometer not needed
- Space-constrained applications
- Low power requirements

### BMI160

**Type:** 6-DOF IMU

**Specifications:**
- **Gyroscope:** ±125, ±250, ±500, ±1000, ±2000 °/s
- **Accelerometer:** ±2, ±4, ±8, ±16 g
- **Interface:** I2C or SPI
- **I2C Address:** 0x68 (SDO=LOW) or 0x69 (SDO=HIGH)
- **Sample Rate:** Up to 3.2kHz (accel), 3.2kHz (gyro)
- **Package:** 2.5×3×0.8mm LGA

**Features:**
- Ultra-low power
- Supports both I2C and SPI in VESC firmware
- On-chip interrupt engine
- FIFO buffer (1024 bytes)

---

## AHRS Algorithms

### Overview

The VESC firmware implements three AHRS (Attitude and Heading Reference System) algorithms:

1. **Mahony Filter:** Explicit complementary filter with PI feedback
2. **Madgwick Filter:** Gradient descent optimization
3. **Fusion Library:** Advanced adaptive algorithm

All algorithms output orientation as a **quaternion** which is converted to Euler angles (roll, pitch, yaw) for applications.

### Mahony Filter

**File:** `ahrs.c`, function `ahrs_update_mahony_imu()`

**Developer:** Robert Mahony (2008)

**Concept:** Explicit complementary filter with proportional-integral (PI) controller

**Algorithm:**
```
1. Estimate orientation error from accelerometer
   error = gyro_world × accel_measured
2. Apply PI correction
   correction = Kp × error + Ki × ∫error dt
3. Correct gyro measurement
   gyro_corrected = gyro_measured + correction
4. Integrate quaternion
   q̇ = 0.5 × q ⊗ gyro_corrected
   q_new = q + q̇ × dt
```

**Parameters:**
- **Kp (Proportional gain):** Typical 0.5-2.0
  - Higher = faster correction, more noise during motion
  - Lower = slower correction, more drift
- **Ki (Integral gain):** Typical 0.0-0.1
  - Corrects gyro bias over time
  - Set to 0 if gyro well-calibrated

**Advantages:**
- Computationally efficient
- Well-understood behavior
- Explicit tuning parameters

**Disadvantages:**
- Requires manual tuning
- No automatic adaptation to motion

### Madgwick Filter

**File:** `ahrs.c`, function `ahrs_update_madgwick_imu()`

**Developer:** Sebastian Madgwick (2010)

**Concept:** Gradient descent optimization to minimize orientation error

**Algorithm:**
```
1. Define objective function (orientation error)
   f = q* ⊗ a_measured ⊗ q - a_reference
2. Compute gradient
   ∇f = J^T × f
   (where J is Jacobian matrix)
3. Normalize gradient
   correction = ∇f / ||∇f||
4. Apply correction to gyro
   q̇ = 0.5 × q ⊗ gyro - β × correction
5. Integrate
   q_new = q + q̇ × dt
```

**Parameters:**
- **β (Beta):** Convergence rate, typical 0.033-0.1
  - Similar to Kp in Mahony filter
  - Represents estimated gyro drift in rad/s

**Advantages:**
- Only one tuning parameter
- Elegant mathematical formulation
- Good performance with default settings

**Disadvantages:**
- Slightly more computational cost
- Less intuitive parameter tuning

### Comparison: Mahony vs Madgwick

| Aspect | Mahony | Madgwick |
|--------|--------|----------|
| Tuning Parameters | Kp, Ki | β |
| Computation | ~450 FLOPs | ~550 FLOPs |
| Gyro Bias Correction | Explicit (Ki) | Implicit in β |
| Typical Performance | Excellent | Excellent |
| Use Case | When bias correction needed | General purpose |

**Practical Recommendation:** Both work well. Madgwick is slightly simpler to tune (one parameter).

### Algorithm Selection

The AHRS module supports both algorithms:

```c
// Mahony filter
ahrs_update_mahony_imu(gyro, accel, dt, &attitude);

// Madgwick filter
ahrs_update_madgwick_imu(gyro, accel, dt, &attitude);
```

Most VESC applications use the **Fusion library** (see below) which includes additional features like adaptive gain and advanced bias correction.

---

## Main IMU Interface

### Overview

The main IMU interface (`imu.c/h`) provides a unified API for all supported IMU sensors with integrated sensor fusion.

**File:** `imu/imu.c`, `imu/imu.h`

### Initialization

#### MPU9150/MPU9250

```c
void imu_init_mpu9x50(stm32_gpio_t *sda_gpio, int sda_pin,
                       stm32_gpio_t *scl_gpio, int scl_pin);
```

**Example:**
```c
// Initialize MPU9250 on PB6 (SCL) and PB7 (SDA)
imu_init_mpu9x50(GPIOB, 7, GPIOB, 6);
```

#### ICM20948

```c
void imu_init_icm20948(stm32_gpio_t *sda_gpio, int sda_pin,
                        stm32_gpio_t *scl_gpio, int scl_pin,
                        int ad0_val);
```

**Parameters:**
- `ad0_val` - AD0 pin state (0 or 1) determines I2C address

**Example:**
```c
// ICM20948 with AD0=0 (address 0x68)
imu_init_icm20948(GPIOB, 7, GPIOB, 6, 0);
```

#### BMI160 (I2C)

```c
void imu_init_bmi160_i2c(stm32_gpio_t *sda_gpio, int sda_pin,
                          stm32_gpio_t *scl_gpio, int scl_pin);
```

#### BMI160 (SPI)

```c
void imu_init_bmi160_spi(stm32_gpio_t *nss_gpio, int nss_pin,
                          stm32_gpio_t *sck_gpio, int sck_pin,
                          stm32_gpio_t *mosi_gpio, int mosi_pin,
                          stm32_gpio_t *miso_gpio, int miso_pin);
```

**Example:**
```c
// BMI160 on SPI pins
imu_init_bmi160_spi(
    GPIOB, 12,  // NSS
    GPIOB, 13,  // SCK
    GPIOB, 15,  // MOSI
    GPIOB, 14   // MISO
);
```

#### LSM6DS3

```c
void imu_init_lsm6ds3(stm32_gpio_t *sda_gpio, int sda_pin,
                       stm32_gpio_t *scl_gpio, int scl_pin);
```

#### Configuration-Based Initialization

```c
void imu_init(imu_config *set);
```

Uses configuration structure from `conf_general.h`. Typically called during system initialization with settings from EEPROM.

### Reading Orientation

#### Euler Angles

```c
float imu_get_roll(void);
float imu_get_pitch(void);
float imu_get_yaw(void);
void imu_get_rpy(float *rpy);  // Get all three at once
```

**Returns:** Angles in degrees
- Roll: -180° to +180°
- Pitch: -90° to +90°
- Yaw: 0° to 360°

**Example:**
```c
float roll = imu_get_roll();
float pitch = imu_get_pitch();
float yaw = imu_get_yaw();

// Or get all at once
float rpy[3];
imu_get_rpy(rpy);
// rpy[0] = roll, rpy[1] = pitch, rpy[2] = yaw
```

#### Quaternion

```c
void imu_get_quaternions(float *q);
```

**Returns:** Quaternion as [w, x, y, z]

**Example:**
```c
float quat[4];
imu_get_quaternions(quat);
// quat[0] = w, quat[1] = x, quat[2] = y, quat[3] = z
```

### Reading Raw Sensor Data

#### Accelerometer

```c
void imu_get_accel(float *accel);
void imu_get_accel_derotated(float *accel);
```

**Returns:** Acceleration in g (9.81 m/s²) as [x, y, z]

**Example:**
```c
float accel[3];
imu_get_accel(accel);
// accel[0] = X-axis acceleration
// accel[1] = Y-axis acceleration
// accel[2] = Z-axis acceleration

// Get acceleration in Earth frame (gravity removed)
float accel_earth[3];
imu_get_accel_derotated(accel_earth);
```

#### Gyroscope

```c
void imu_get_gyro(float *gyro);
void imu_get_gyro_derotated(float *gyro);
```

**Returns:** Angular velocity in °/s as [x, y, z]

**Example:**
```c
float gyro[3];
imu_get_gyro(gyro);
// gyro[0] = Roll rate (°/s)
// gyro[1] = Pitch rate (°/s)
// gyro[2] = Yaw rate (°/s)
```

#### Magnetometer

```c
void imu_get_mag(float *mag);
```

**Returns:** Magnetic field in µT as [x, y, z]

**Example:**
```c
float mag[3];
imu_get_mag(mag);
```

### Orientation Control

#### Reset Orientation

```c
void imu_reset_orientation(void);
```

Resets AHRS algorithm to re-initialize orientation from accelerometer and magnetometer. Use when IMU has been moved while stationary.

#### Set Yaw

```c
void imu_set_yaw(float yaw_deg);
```

Manually sets yaw angle. Useful for:
- Compensating for magnetic declination
- Resetting heading reference
- Aligning to known direction

**Example:**
```c
// Set yaw to north (0°)
imu_set_yaw(0.0f);

// Set yaw to 90° (east)
imu_set_yaw(90.0f);
```

### Coordinate Transformations

```c
void imu_derotate(const float *input, float *output);
```

Transforms vector from sensor frame to Earth frame.

**Use Cases:**
- Convert sensor-frame acceleration to Earth-frame
- Calculate true vertical acceleration
- Implement GPS-aided navigation

**Example:**
```c
float accel_sensor[3];
imu_get_accel(accel_sensor);

float accel_earth[3];
imu_derotate(accel_sensor, accel_earth);

// Now accel_earth[2] contains vertical acceleration
// (after removing gravity component)
```

### Calibration

```c
void imu_get_calibration(float yaw, float *imu_cal);
```

Gets calibration matrix for specified yaw angle.

**Parameters:**
- `yaw` - Yaw angle in degrees
- `imu_cal` - Output 3×3 rotation matrix (9 floats)

### Status Checking

```c
bool imu_startup_done(void);
```

Returns `true` when IMU has completed initialization and sensor fusion has converged.

**Typical startup time:** 1-3 seconds

**Example:**
```c
imu_init_mpu9x50(GPIOB, 7, GPIOB, 6);

while (!imu_startup_done()) {
    chThdSleepMilliseconds(10);
}

// IMU ready to use
float roll = imu_get_roll();
```

### Callbacks

```c
void imu_set_read_callback(void (*func)(float *acc, float *gyro, float *mag, float dt));
```

Sets callback function called on each IMU update (typically 1000Hz).

**Use Cases:**
- Custom sensor fusion algorithms
- Data logging
- Advanced filtering

**Example:**
```c
void my_imu_callback(float *acc, float *gyro, float *mag, float dt) {
    // Called at IMU sample rate (~1kHz)
    log_imu_data(acc, gyro, mag, dt);
}

imu_set_read_callback(my_imu_callback);
```

### Shutdown

```c
void imu_stop(void);
```

Stops IMU sampling and releases resources.

---

## Fusion Library

### Overview

The Fusion library (by Seb Madgwick) is a third-party AHRS library integrated into VESC firmware. It provides advanced features beyond the basic Mahony/Madgwick implementations.

**Files:** `imu/Fusion/`

### Key Features

1. **Adaptive Gain:** Automatically adjusts fusion gain based on acceleration magnitude
2. **Gyro Bias Correction:** Continuously estimates and removes gyro bias
3. **Magnetic Rejection:** Detects and rejects magnetic distortions
4. **Linear Acceleration:** Outputs gravity-compensated acceleration
5. **Optimized:** Inline functions for maximum performance

### Data Types

```c
// 3D vector
typedef struct {
    float x, y, z;
} FusionVector3;

// Quaternion [w, x, y, z]
typedef struct {
    float w, x, y, z;
} FusionQuaternion;

// Euler angles (degrees)
typedef struct {
    float roll, pitch, yaw;
} FusionEulerAngles;
```

### AHRS Algorithm

**File:** `Fusion/FusionAhrs.c`

**Structure:**
```c
typedef struct {
    float gain;                        // Fusion gain
    float acc_conf_decay;             // Accelerometer confidence decay
    float minimumMagneticFieldSquared;
    float maximumMagneticFieldSquared;
    FusionQuaternion quaternion;
    FusionVector3 linearAcceleration;
    float accMagP;                     // Accelerometer confidence
} FusionAhrs;
```

### Initialization

```c
void FusionAhrsInitialise(FusionAhrs *fusionAhrs,
                           const float gain,
                           const float acc_conf_decay);
```

**Parameters:**
- `gain` - Fusion gain, typical 0.5
- `acc_conf_decay` - Accelerometer confidence decay rate

**Example:**
```c
FusionAhrs ahrs;
FusionAhrsInitialise(&ahrs, 0.5f, 0.0f);

// Set magnetic field valid range (20-70 µT)
FusionAhrsSetMagneticField(&ahrs, 20.0f, 70.0f);
```

### Updating AHRS

#### With Magnetometer (9-DOF)

```c
void FusionAhrsUpdate(FusionAhrs *fusionAhrs,
                       const FusionVector3 gyroscope,      // °/s
                       const FusionVector3 accelerometer,  // g
                       const FusionVector3 magnetometer,   // µT
                       const float samplePeriod);          // seconds
```

**Example:**
```c
FusionVector3 gyro = {.x = gyro_x, .y = gyro_y, .z = gyro_z};
FusionVector3 accel = {.x = accel_x, .y = accel_y, .z = accel_z};
FusionVector3 mag = {.x = mag_x, .y = mag_y, .z = mag_z};

float dt = 0.001f;  // 1ms sample period (1000Hz)

FusionAhrsUpdate(&ahrs, gyro, accel, mag, dt);
```

#### Without Magnetometer (6-DOF)

```c
void FusionAhrsUpdateWithoutMagnetometer(FusionAhrs *fusionAhrs,
                                          const FusionVector3 gyroscope,
                                          const FusionVector3 accelerometer,
                                          const float samplePeriod);
```

**Note:** Yaw will drift over time without magnetometer.

### Getting Orientation

```c
FusionQuaternion FusionAhrsGetQuaternion(const FusionAhrs *fusionAhrs);
```

Returns orientation as quaternion.

**Convert to Euler Angles:**
```c
FusionQuaternion quat = FusionAhrsGetQuaternion(&ahrs);
FusionEulerAngles euler = FusionQuaternionToEulerAngles(quat);

float roll = euler.angle.roll;
float pitch = euler.angle.pitch;
float yaw = euler.angle.yaw;
```

### Linear Acceleration

```c
FusionVector3 FusionAhrsGetLinearAcceleration(const FusionAhrs *fusionAhrs);
FusionVector3 FusionAhrsGetEarthAcceleration(const FusionAhrs *fusionAhrs);
```

**Linear Acceleration:** Sensor-frame acceleration with gravity removed
**Earth Acceleration:** Earth-frame acceleration

**Example:**
```c
FusionVector3 lin_accel = FusionAhrsGetLinearAcceleration(&ahrs);
// lin_accel.x, lin_accel.y, lin_accel.z
// Useful for detecting bumps, jumps, impacts

FusionVector3 earth_accel = FusionAhrsGetEarthAcceleration(&ahrs);
// earth_accel.z = vertical acceleration (up positive)
```

### Advanced Features

#### Adaptive Gain

The Fusion library automatically reduces gain during high acceleration (motion) to prevent tilt errors:

```c
if (accel_magnitude > 1.0g ± threshold) {
    reduce gain // Rely more on gyro
} else {
    normal gain // Trust accelerometer
}
```

**Confidence Decay Parameter:**
```c
FusionAhrsSetAccConfDecay(&ahrs, 0.01f);
```
Controls how quickly confidence recovers after motion.

#### Magnetic Field Validation

```c
FusionAhrsSetMagneticField(&ahrs, min_field, max_field);
```

Magnetometer readings outside this range are rejected.

**Earth's Magnetic Field:**
- Equator: ~25-30 µT
- Mid-latitudes: ~45-55 µT
- Poles: ~60-70 µT

**Recommended range:** 20-70 µT (rejects most distortions)

#### Reinitialize

```c
void FusionAhrsReinitialise(FusionAhrs *fusionAhrs);
```

Resets AHRS state while keeping configuration.

#### Check Initialization

```c
bool FusionAhrsIsInitialising(const FusionAhrs *fusionAhrs);
```

Returns `true` during initial convergence period.

---

## Sensor Drivers

### MPU9150/MPU9250 Driver

**File:** `imu/mpu9150.c`

The MPU9x50 driver is the most mature and widely used in VESC applications.

#### Initialization

```c
void mpu9150_init(stm32_gpio_t *sda_gpio, int sda_pin,
                   stm32_gpio_t *scl_gpio, int scl_pin,
                   stkalign_t *work_area, size_t work_area_size);
```

**Work Area:** Thread stack space (typically 1-2KB)

#### Configuration

```c
void mpu9150_set_rate_hz(int hz);
```

Sets sample rate in Hz (default 1000Hz).

```c
void mpu9150_set_mag_enabled(bool enabled);
```

Enable/disable magnetometer sampling.

#### Reading Sensors

```c
void mpu9150_get_accel(float *accel);
void mpu9150_get_gyro(float *gyro);
void mpu9150_get_mag(float *mag);

// Get all at once (more efficient)
void mpu9150_get_accel_gyro_mag(float *accel, float *gyro, float *mag);
```

#### Raw Data

```c
void mpu9150_get_raw_accel_gyro_mag(int16_t *data);
```

Returns raw 16-bit ADC values:
- data[0..2] = accel X, Y, Z
- data[3..5] = gyro X, Y, Z
- data[6..8] = mag X, Y, Z

#### Calibration

```c
void mpu9150_sample_gyro_offsets(uint32_t iterations);
```

Samples gyro for specified iterations and calculates zero offset. Device must be stationary.

**Example:**
```c
// Calibrate gyro (take 1000 samples)
mpu9150_sample_gyro_offsets(1000);
```

#### Diagnostics

```c
uint32_t mpu9150_get_time_since_update(void);
float mpu9150_get_last_sample_duration(void);
int mpu9150_get_failed_reads(void);
int mpu9150_get_failed_mag_reads(void);
int mpu9150_mag_updated(void);
```

**Use for:**
- Detecting I2C communication failures
- Monitoring sample timing
- Debugging magnetometer issues

#### Device Detection

```c
bool mpu9150_is_mpu9250(void);
```

Returns `true` if MPU9250 detected, `false` if MPU9150.

**Differences:**
- MPU9250: AK8963 magnetometer (improved)
- MPU9150: AK8975 magnetometer

### ICM20948 Driver

**File:** `imu/icm20948.c`

Newer 9-DOF IMU with improved performance.

#### Initialization

```c
void icm20948_init(ICM20948_STATE *s,
                    i2c_bb_state *i2c_state,
                    int ad0_val,
                    stkalign_t *work_area,
                    size_t work_area_size);
```

**Example:**
```c
ICM20948_STATE icm_state;
i2c_bb_state *i2c = imu_get_i2c();  // Get shared I2C

icm20948_init(&icm_state, i2c, 0, work_area, sizeof(work_area));
```

#### Callback

```c
void icm20948_set_read_callback(ICM20948_STATE *s,
                                 void(*func)(float *accel, float *gyro, float *mag));
```

**Example:**
```c
void icm_callback(float *accel, float *gyro, float *mag) {
    // Process sensor data
}

icm20948_set_read_callback(&icm_state, icm_callback);
```

#### Shutdown

```c
void icm20948_stop(ICM20948_STATE *s);
```

### LSM6DS3 Driver

**File:** `imu/lsm6ds3.c`

6-DOF IMU (no magnetometer).

**Similar API to other drivers** - initialization and data reading functions follow same patterns.

### BMI160 Driver

**File:** `imu/bmi160_wrapper.c` + `BMI160_driver/`

Supports both I2C and SPI interfaces.

**Vendor driver** from Bosch included in `BMI160_driver/` subdirectory.

---

## Calibration and Tuning

### Gyroscope Calibration

**Purpose:** Remove constant bias (zero offset)

**Procedure:**
1. Place device on stable surface (no vibration)
2. Ensure device is completely stationary
3. Run calibration routine

**Implementation:**
```c
// MPU9150/9250
mpu9150_sample_gyro_offsets(1000);  // 1000 samples

// Custom calibration
float gyro_sum[3] = {0};
for (int i = 0; i < 1000; i++) {
    float gyro[3];
    imu_get_gyro(gyro);
    gyro_sum[0] += gyro[0];
    gyro_sum[1] += gyro[1];
    gyro_sum[2] += gyro[2];
    chThdSleepMilliseconds(1);
}
float gyro_offset[3] = {
    gyro_sum[0] / 1000.0f,
    gyro_sum[1] / 1000.0f,
    gyro_sum[2] / 1000.0f
};
// Apply offsets in subsequent readings
```

**When to Calibrate:**
- After every power-up (offsets change with temperature)
- After significant temperature change
- If orientation drifts when stationary

### Accelerometer Calibration

**Purpose:** Correct scale factor and offset

**Ideal Output:**
- +1g when axis pointing down
- -1g when axis pointing up
- 0g when axis horizontal

**6-Position Calibration:**
1. Place device with each axis pointing up, then down (6 positions)
2. Record measurements
3. Calculate scale and offset

**Simple 2-Position Calibration (Z-axis only):**
```c
// Position 1: Z-axis pointing up
float accel_up[3];
imu_get_accel(accel_up);
float z_up = accel_up[2];  // Should be ~-1g

// Position 2: Z-axis pointing down
float accel_down[3];
imu_get_accel(accel_down);
float z_down = accel_down[2];  // Should be ~+1g

// Calculate offset and scale
float z_offset = (z_up + z_down) / 2.0f;  // Should be ~0
float z_scale = (z_down - z_up) / 2.0f;   // Should be ~1g

// Apply correction
float accel_corrected = (accel_raw - z_offset) / z_scale;
```

**Full 6-Position Example:**
See `encoder.c` encoder calibration for similar multi-position calibration pattern.

### Magnetometer Calibration

**Purpose:** Correct hard-iron and soft-iron distortions

**Hard-Iron Calibration (Offset):**
1. Rotate device slowly in all directions (figure-8 pattern)
2. Record min/max values for each axis
3. Calculate offset

```c
float mag_min[3] = {1000, 1000, 1000};
float mag_max[3] = {-1000, -1000, -1000};

// Collect data while rotating
for (int i = 0; i < 500; i++) {
    float mag[3];
    imu_get_mag(mag);

    for (int j = 0; j < 3; j++) {
        if (mag[j] < mag_min[j]) mag_min[j] = mag[j];
        if (mag[j] > mag_max[j]) mag_max[j] = mag[j];
    }
    chThdSleepMilliseconds(20);
}

// Calculate offset
float mag_offset[3] = {
    (mag_max[0] + mag_min[0]) / 2.0f,
    (mag_max[1] + mag_min[1]) / 2.0f,
    (mag_max[2] + mag_min[2]) / 2.0f
};

// Apply offset
float mag_calibrated[3] = {
    mag_raw[0] - mag_offset[0],
    mag_raw[1] - mag_offset[1],
    mag_raw[2] - mag_offset[2]
};
```

**Soft-Iron Calibration (Scale):**

More complex - requires ellipsoid fitting. Typically done offline with collected data.

**Practical Tip:** If magnetometer highly distorted, use 6-DOF mode (no magnetometer). Yaw will drift but roll/pitch remain accurate.

### AHRS Tuning

#### Mahony Filter Parameters

```c
#define MAHONY_KP  1.0f   // Proportional gain
#define MAHONY_KI  0.01f  // Integral gain

ahrs_update_mahony_imu(gyro, accel, dt, &attitude);
```

**Tuning Guide:**
- **Too much Kp:** Orientation jumps during acceleration/vibration
- **Too little Kp:** Slow correction, drift visible
- **Kp = 0.5-2.0:** Good starting range
- **Ki = 0.0:** If gyro well-calibrated
- **Ki = 0.01-0.1:** For bias correction

#### Madgwick Filter Parameters

```c
#define MADGWICK_BETA  0.1f  // Convergence rate

ahrs_update_madgwick_imu(gyro, accel, dt, &attitude);
```

**Tuning Guide:**
- β ≈ estimated gyro drift in rad/s
- **β = 0.033:** For ±2°/s drift (good gyro)
- **β = 0.1:** For ±6°/s drift (lower quality gyro)
- **Too high β:** Noisy during motion
- **Too low β:** Slow convergence, drift

#### Fusion Library Parameters

```c
FusionAhrs ahrs;
FusionAhrsInitialise(&ahrs, 0.5f, 0.0f);
```

**Gain:** Similar to Kp/β above (0.5 is good default)
**Acc Conf Decay:** Leave at 0.0 for automatic adaptation

---

## Practical Applications

### Self-Balancing Vehicle

```c
#include "imu.h"
#include "mc_interface.h"

#define BALANCE_KP  20.0f   // Proportional gain
#define BALANCE_KD  2.0f    // Derivative gain

void balance_control_init(void) {
    imu_init_mpu9x50(GPIOB, 7, GPIOB, 6);

    while (!imu_startup_done()) {
        chThdSleepMilliseconds(10);
    }
}

void balance_control_update(void) {
    // Get pitch angle and rate
    float pitch = imu_get_pitch();
    float gyro[3];
    imu_get_gyro(gyro);
    float pitch_rate = gyro[1];  // Y-axis

    // PD controller
    float error = 0.0f - pitch;  // Target pitch = 0°
    float output = BALANCE_KP * error + BALANCE_KD * (-pitch_rate);

    // Apply to motor (positive = forward)
    mc_interface_set_current(output);
}

// Call from 100Hz thread
```

### Antenna Tracker

```c
void antenna_tracker_update(void) {
    // Get orientation
    float rpy[3];
    imu_get_rpy(rpy);

    float current_azimuth = rpy[2];   // Yaw
    float current_elevation = rpy[1]; // Pitch

    // Target coordinates from GPS
    float target_azimuth = gps_calculate_bearing();
    float target_elevation = gps_calculate_elevation();

    // Calculate servo positions
    float azimuth_error = target_azimuth - current_azimuth;
    float elevation_error = target_elevation - current_elevation;

    // Apply to servos
    pwm_servo_set_servo_out(azimuth_error / 180.0f);  // Normalize to ±1.0
}
```

### Vibration Monitoring

```c
void vibration_monitor(void) {
    float accel[3];
    imu_get_accel(accel);

    // Remove gravity component (use linear acceleration)
    float lin_accel[3];
    imu_get_accel_derotated(lin_accel);

    // Calculate vibration magnitude
    float vib_mag = sqrtf(lin_accel[0]*lin_accel[0] +
                          lin_accel[1]*lin_accel[1] +
                          lin_accel[2]*lin_accel[2]);

    if (vib_mag > 2.0f) {  // > 2g vibration
        // Excessive vibration detected
        fault_handler(FAULT_CODE_HIGH_VIBRATION);
    }
}
```

### Tilt Compensation

```c
void tilt_compensated_speed_control(void) {
    float pitch = imu_get_pitch();

    // Reduce speed on steep inclines
    float max_current_scale = 1.0f - (fabsf(pitch) / 90.0f) * 0.5f;

    if (max_current_scale < 0.5f) {
        max_current_scale = 0.5f;  // Minimum 50% power
    }

    float user_throttle = app_get_throttle();
    float motor_current = user_throttle * 50.0f * max_current_scale;

    mc_interface_set_current(motor_current);
}
```

### Jump/Impact Detection

```c
void detect_jumps(void) {
    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs, 0.5f, 0.0f);

    // ... update AHRS each cycle ...

    // Get vertical acceleration in Earth frame
    FusionVector3 earth_accel = FusionAhrsGetEarthAcceleration(&ahrs);

    // earth_accel.z is vertical (positive = up)
    float vertical_accel = earth_accel.axis.z;

    // Detect freefall (acceleration ~0g)
    if (fabsf(vertical_accel) < 0.2f) {
        // In freefall / airborne
        airborne_time++;
    }

    // Detect landing (strong downward acceleration)
    if (vertical_accel < -2.0f) {
        // Impact detected
        handle_landing();
    }
}
```

---

## Summary

The VESC IMU system provides:

**Hardware Support:**
- 4 different IMU sensors (MPU9x50, ICM20948, LSM6DS3, BMI160)
- Both I2C and SPI interfaces
- 6-DOF and 9-DOF configurations

**Sensor Fusion:**
- Mahony and Madgwick AHRS algorithms
- Advanced Fusion library with adaptive features
- Quaternion-based computation (no gimbal lock)
- Gyro bias correction

**Outputs:**
- Euler angles (roll, pitch, yaw)
- Quaternion orientation
- Linear acceleration
- Raw sensor data

**Applications:**
- Self-balancing vehicles
- Antenna/camera tracking
- Platform stabilization
- Vibration monitoring
- Impact detection

**Best Practices:**
- Calibrate gyro after every power-up
- Mount IMU rigidly to reduce vibration
- Use 6-DOF mode if magnetic distortions present
- Start with default AHRS gains (0.5 or β=0.1)
- Allow 1-3 seconds for initial convergence

The IMU system enables advanced vehicle dynamics control and situational awareness for robotics and vehicle applications.
