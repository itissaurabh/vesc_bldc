# Hardware Configuration Documentation

## Table of Contents

1. [Introduction](#introduction)
2. [Hardware Configuration Structure](#hardware-configuration-structure)
3. [Required Definitions](#required-definitions)
4. [Pin Mapping](#pin-mapping)
5. [ADC Configuration](#adc-configuration)
6. [Gate Drivers](#gate-drivers)
7. [Current Sensing](#current-sensing)
8. [Peripheral Configuration](#peripheral-configuration)
9. [Hardware Limits](#hardware-limits)
10. [Creating Custom Hardware Configuration](#creating-custom-hardware-configuration)
11. [Build System Integration](#build-system-integration)

---

## Introduction

The `/hwconf/` directory contains hardware-specific configurations for 82+ different VESC board variants from various manufacturers. Each hardware configuration defines:

- Pin assignments for all peripherals
- ADC channel mapping
- Gate driver configuration
- Current and voltage sensing parameters
- Hardware-specific limits and safety features
- Peripheral configurations (UART, SPI, I2C, CAN, etc.)

**Purpose:**
- Abstract hardware differences from main firmware
- Support multiple board variants with same codebase
- Enable custom hardware development
- Provide hardware-specific optimizations

**Note:** This documentation focuses on **how to define a new board configuration**, not documenting individual boards. Use the example configuration as a template for custom hardware.

---

## Hardware Configuration Structure

Each hardware configuration consists of three main files:

### File Structure

```
hwconf/
└── your_board/
    ├── hw_your_board.h          # Hardware selector
    ├── hw_your_board_core.h     # Hardware definitions
    └── hw_your_board_core.c     # Hardware initialization
```

### 1. hw_your_board.h (Selector)

This file selects which hardware variant to build:

```c
#ifndef HW_YOUR_BOARD_H_
#define HW_YOUR_BOARD_H_

// Define which variant (if you have multiple versions)
#define HW_YOUR_BOARD_IS_MK1

// Include the core configuration
#include "hw_your_board_core.h"

#endif /* HW_YOUR_BOARD_H_ */
```

**Purpose:** Allows multiple hardware revisions to share common code

### 2. hw_your_board_core.h (Definitions)

This file contains all hardware-specific constants and macros:

- Hardware name and version
- Pin definitions (GPIO ports and pins)
- ADC channel mapping
- Peripheral configurations
- Measurement macros
- Hardware limits

**Size:** Typically 250-500 lines

### 3. hw_your_board_core.c (Implementation)

This file contains hardware initialization code:

```c
void hw_init_gpio(void);           // Initialize GPIO pins
void hw_setup_adc_channels(void);  // Configure ADC channels
void hw_start_i2c(void);           // Start I2C peripheral
void hw_stop_i2c(void);            // Stop I2C peripheral
void hw_try_restore_i2c(void);     // I2C bus recovery
```

**Size:** Typically 200-400 lines

---

## Required Definitions

### Hardware Identification

```c
// Hardware name (displayed in VESC Tool)
#define HW_NAME                 "My Custom VESC"

// Hardware version
#define HW_MAJOR                1
#define HW_MINOR                0
```

### Hardware Properties

These defines enable features and select gate drivers:

```c
// Gate driver selection (choose one)
#define HW_HAS_DRV8301          // TI DRV8301 gate driver
//#define HW_HAS_DRV8305        // TI DRV8305 gate driver
//#define HW_HAS_DRV8316        // TI DRV8316 gate driver
//#define HW_HAS_DRV8320S       // TI DRV8320S gate driver
//#define HW_HAS_DRV8323S       // TI DRV8323S gate driver

// Current sensing configuration
#define HW_HAS_3_SHUNTS         // Three current sensors (one per phase)
//#define HW_HAS_PHASE_SHUNTS   // Shunts in motor phase lines (not low-side)
//#define HW_HAS_PHASE_FILTERS  // Hardware low-pass filters on phase voltage sensing

// Optional features
//#define HW_HAS_NO_CAN         // No CAN bus hardware
//#define HW_USE_INTERNAL_RC    // Use internal RC oscillator (no external crystal)
//#define HW_HAS_PERMANENT_NRF  // Onboard NRF24L01+ wireless
```

### Control Macros

```c
// Gate driver enable/disable
#define ENABLE_GATE()           palSetPad(GPIOB, 5)
#define DISABLE_GATE()          palClearPad(GPIOB, 5)

// DC calibration control (if separate enable)
#define DCCAL_ON()              // Enable DC offset calibration mode
#define DCCAL_OFF()             // Disable DC offset calibration mode

// Fault detection
#define IS_DRV_FAULT()          (!palReadPad(GPIOB, 7))

// LED control
#define LED_GREEN_GPIO          GPIOB
#define LED_GREEN_PIN           0
#define LED_GREEN_ON()          palSetPad(LED_GREEN_GPIO, LED_GREEN_PIN)
#define LED_GREEN_OFF()         palClearPad(LED_GREEN_GPIO, LED_GREEN_PIN)
#define LED_RED_GPIO            GPIOB
#define LED_RED_PIN             1
#define LED_RED_ON()            palSetPad(LED_RED_GPIO, LED_RED_PIN)
#define LED_RED_OFF()           palClearPad(LED_RED_GPIO, LED_RED_PIN)
```

---

## Pin Mapping

### PWM Outputs (Motor Phases)

The STM32F4 uses TIM1 for motor PWM generation:

```c
// TIM1 generates 3 complementary PWM pairs for 3-phase inverter
// PA8  = TIM1_CH1  (Phase A high-side)
// PA9  = TIM1_CH2  (Phase B high-side)
// PA10 = TIM1_CH3  (Phase C high-side)
// PB13 = TIM1_CH1N (Phase A low-side)
// PB14 = TIM1_CH2N (Phase B low-side)
// PB15 = TIM1_CH3N (Phase C low-side)

// These are typically fixed for STM32F4-based VESCs
```

**Note:** PWM pins are usually hardware-dependent and cannot be changed.

### Hall Sensor / Encoder Pins

```c
// Hall sensor inputs (also used for ABI encoder)
#define HW_HALL_ENC_GPIO1       GPIOC
#define HW_HALL_ENC_PIN1        6
#define HW_HALL_ENC_GPIO2       GPIOC
#define HW_HALL_ENC_PIN2        7
#define HW_HALL_ENC_GPIO3       GPIOC
#define HW_HALL_ENC_PIN3        8

// Encoder timer (for ABI quadrature counting)
#define HW_ENC_TIM              TIM3
#define HW_ENC_TIM_AF           GPIO_AF_TIM3
#define HW_ENC_TIM_CLK_EN()     RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE)

// Interrupt configuration for index pulse
#define HW_ENC_EXTI_PORTSRC     EXTI_PortSourceGPIOC
#define HW_ENC_EXTI_PINSRC      EXTI_PinSource8
#define HW_ENC_EXTI_CH          EXTI9_5_IRQn
#define HW_ENC_EXTI_LINE        EXTI_Line8
#define HW_ENC_EXTI_ISR_VEC     EXTI9_5_IRQHandler
#define HW_ENC_TIM_ISR_CH       TIM3_IRQn
#define HW_ENC_TIM_ISR_VEC      TIM3_IRQHandler

// Macros to read Hall sensors
#define READ_HALL1()            palReadPad(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1)
#define READ_HALL2()            palReadPad(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2)
#define READ_HALL3()            palReadPad(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3)
```

### UART Pins

```c
// Main UART (usually for COMM port - USB alternative)
#define HW_UART_DEV             SD3             // Serial driver 3
#define HW_UART_GPIO_AF         GPIO_AF_USART3
#define HW_UART_TX_PORT         GPIOB
#define HW_UART_TX_PIN          10
#define HW_UART_RX_PORT         GPIOB
#define HW_UART_RX_PIN          11

// Permanent UART (if available - for external devices)
#define HW_UART_P_BAUD          115200
#define HW_UART_P_DEV           SD4
#define HW_UART_P_GPIO_AF       GPIO_AF_UART4
#define HW_UART_P_TX_PORT       GPIOC
#define HW_UART_P_TX_PIN        10
#define HW_UART_P_RX_PORT       GPIOC
#define HW_UART_P_RX_PIN        11
```

### SPI Pins

```c
// Main SPI (for encoders like AS5047)
#define HW_SPI_DEV              SPID1
#define HW_SPI_GPIO_AF          GPIO_AF_SPI1
#define HW_SPI_PORT_NSS         GPIOB
#define HW_SPI_PIN_NSS          11
#define HW_SPI_PORT_SCK         GPIOA
#define HW_SPI_PIN_SCK          5
#define HW_SPI_PORT_MOSI        GPIOA
#define HW_SPI_PIN_MOSI         7
#define HW_SPI_PORT_MISO        GPIOA
#define HW_SPI_PIN_MISO         6

// DRV8301 SPI (gate driver)
#define DRV8301_MOSI_GPIO       GPIOB
#define DRV8301_MOSI_PIN        4
#define DRV8301_MISO_GPIO       GPIOB
#define DRV8301_MISO_PIN        3
#define DRV8301_SCK_GPIO        GPIOC
#define DRV8301_SCK_PIN         10
#define DRV8301_CS_GPIO         GPIOC
#define DRV8301_CS_PIN          9
```

### I2C Pins

```c
// I2C (for Nunchuk controller, IMU, etc.)
#define HW_I2C_DEV              I2CD2
#define HW_I2C_GPIO_AF          GPIO_AF_I2C2
#define HW_I2C_SCL_PORT         GPIOB
#define HW_I2C_SCL_PIN          10
#define HW_I2C_SDA_PORT         GPIOB
#define HW_I2C_SDA_PIN          11
```

### PPM/Servo Input

```c
// Input Capture Unit (for RC servo/PPM input)
#define HW_USE_SERVO_TIM4       // Use timer 4 for servo input
#define HW_ICU_TIMER            TIM4
#define HW_ICU_TIM_CLK_EN()     RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE)
#define HW_ICU_DEV              ICUD4
#define HW_ICU_CHANNEL          ICU_CHANNEL_1
#define HW_ICU_GPIO_AF          GPIO_AF_TIM4
#define HW_ICU_GPIO             GPIOB
#define HW_ICU_PIN              6
```

---

## ADC Configuration

ADC configuration is critical for current sensing, voltage measurement, and temperature monitoring.

### ADC Channel Assignment

```c
// ADC Vector Documentation
/*
 * ADC Index assignments (example):
 * 0:   IN10    CURR1       (Phase A current)
 * 1:   IN11    CURR2       (Phase B current)
 * 2:   IN12    CURR3       (Phase C current)
 * 3:   IN0     SENS1       (Phase A voltage)
 * 4:   IN1     SENS2       (Phase B voltage)
 * 5:   IN2     SENS3       (Phase C voltage)
 * 6:   IN5     ADC_EXT1    (External ADC 1 - throttle)
 * 7:   IN6     ADC_EXT2    (External ADC 2 - brake)
 * 8:   IN3     TEMP_PCB    (PCB temperature)
 * 9:   IN14    TEMP_MOTOR  (Motor temperature)
 * 10:  IN15    ADC_EXT3    (External ADC 3)
 * 11:  IN13    AN_IN       (Analog input - battery voltage)
 * 12:  Vrefint             (Internal reference voltage)
 */

// Number of ADC channels
#define HW_ADC_INJ_CHANNELS     3   // Injected channels (current sensing)
#define HW_ADC_NBR_CONV         5   // Regular conversions per ADC
#define HW_ADC_CHANNELS         (HW_ADC_NBR_CONV * 3)  // Total channels

// ADC channel indices (used by firmware)
#define ADC_IND_SENS1           3   // Phase A voltage
#define ADC_IND_SENS2           4   // Phase B voltage
#define ADC_IND_SENS3           5   // Phase C voltage
#define ADC_IND_CURR1           0   // Phase A current
#define ADC_IND_CURR2           1   // Phase B current
#define ADC_IND_CURR3           2   // Phase C current
#define ADC_IND_VIN_SENS        11  // Battery voltage
#define ADC_IND_EXT             6   // External analog 1
#define ADC_IND_EXT2            7   // External analog 2
#define ADC_IND_TEMP_MOS        8   // MOSFET temperature
#define ADC_IND_TEMP_MOTOR      9   // Motor temperature
#define ADC_IND_VREFINT         12  // Vref internal
```

### ADC GPIO Configuration

```c
// External ADC inputs
#define HW_ADC_EXT_GPIO         GPIOA
#define HW_ADC_EXT_PIN          5
#define HW_ADC_EXT2_GPIO        GPIOA
#define HW_ADC_EXT2_PIN         6
```

### Hardware-Specific ADC Parameters

```c
// Voltage reference
#ifndef V_REG
#define V_REG                   3.3         // ADC reference voltage
#endif

// Battery voltage divider
#ifndef VIN_R1
#define VIN_R1                  39000.0     // Upper resistor (Ω)
#endif
#ifndef VIN_R2
#define VIN_R2                  2200.0      // Lower resistor (Ω)
#endif

// Current sensing amplifier
#ifndef CURRENT_AMP_GAIN
#define CURRENT_AMP_GAIN        20.0        // Current amplifier gain
#endif
#ifndef CURRENT_SHUNT_RES
#define CURRENT_SHUNT_RES       0.0005      // Shunt resistance (Ω) - 0.5mΩ
#endif
```

### Measurement Macros

```c
// Battery voltage calculation
#define GET_INPUT_VOLTAGE() \
    ((V_REG / 4095.0) * (float)ADC_Value[ADC_IND_VIN_SENS] * \
     ((VIN_R1 + VIN_R2) / VIN_R2))

// NTC thermistor temperature (high-side - MOSFET)
#define NTC_RES(adc_val) \
    ((4095.0 * 10000.0) / adc_val - 10000.0)

#define NTC_TEMP(adc_ind) \
    (1.0 / ((logf(NTC_RES(ADC_Value[adc_ind]) / 10000.0) / 3380.0) + \
     (1.0 / 298.15)) - 273.15)

// NTC thermistor (low-side - Motor)
#define NTC_RES_MOTOR(adc_val) \
    (10000.0 / ((4095.0 / (float)adc_val) - 1.0))

#define NTC_TEMP_MOTOR(beta) \
    (1.0 / ((logf(NTC_RES_MOTOR(ADC_Value[ADC_IND_TEMP_MOTOR]) / 10000.0) / \
     beta) + (1.0 / 298.15)) - 273.15)

// Generic ADC voltage
#define ADC_VOLTS(ch)           ((float)ADC_Value[ch] / 4096.0 * V_REG)

// Phase voltage measurements
#define ADC_V_L1                ADC_Value[ADC_IND_SENS1]
#define ADC_V_L2                ADC_Value[ADC_IND_SENS2]
#define ADC_V_L3                ADC_Value[ADC_IND_SENS3]
#define ADC_V_ZERO              (ADC_Value[ADC_IND_VIN_SENS] / 2)
```

---

## Gate Drivers

The VESC supports multiple Texas Instruments gate driver ICs. Choose one based on your hardware.

### DRV8301 (Most Common)

**Specifications:**
- Up to 60V operating voltage
- 3 half-bridge drivers
- Integrated current amplifiers (3 shunt)
- Overcurrent protection
- SPI interface for configuration

**Configuration:**
```c
#define HW_HAS_DRV8301

// SPI pins
#define DRV8301_MOSI_GPIO       GPIOB
#define DRV8301_MOSI_PIN        4
#define DRV8301_MISO_GPIO       GPIOB
#define DRV8301_MISO_PIN        3
#define DRV8301_SCK_GPIO        GPIOC
#define DRV8301_SCK_PIN         10
#define DRV8301_CS_GPIO         GPIOC
#define DRV8301_CS_PIN          9

// In hw_init_gpio(), call:
drv8301_init();
```

### DRV8305

**Specifications:**
- Up to 60V operating voltage
- 3 half-bridge drivers
- Integrated buck regulator
- More configuration options than DRV8301

**Configuration:**
```c
#define HW_HAS_DRV8305

#define DRV8305_MOSI_GPIO       GPIOC
#define DRV8305_MOSI_PIN        12
#define DRV8305_MISO_GPIO       GPIOC
#define DRV8305_MISO_PIN        11
#define DRV8305_SCK_GPIO        GPIOC
#define DRV8305_SCK_PIN         10
#define DRV8305_CS_GPIO         GPIOD
#define DRV8305_CS_PIN          8

// In hw_init_gpio(), call:
drv8305_init();
```

### DRV8320S

**Specifications:**
- Up to 60V operating voltage
- Smart gate drive technology
- Lower cost than DRV8301/8305
- SPI configuration

**Configuration:**
```c
#define HW_HAS_DRV8320S

#define DRV8320S_MOSI_GPIO      GPIOC
#define DRV8320S_MOSI_PIN       12
#define DRV8320S_MISO_GPIO      GPIOC
#define DRV8320S_MISO_PIN       11
#define DRV8320S_SCK_GPIO       GPIOC
#define DRV8320S_SCK_PIN        10
#define DRV8320S_CS_GPIO        GPIOC
#define DRV8320S_CS_PIN         9

// In hw_init_gpio(), call:
drv8320s_init();
```

### DRV8323S

**Specifications:**
- Up to 65V operating voltage
- Lowest RDS(on)
- Smart gate drive with current sensing
- Best performance

**Configuration:**
```c
#define HW_HAS_DRV8323S

#define DRV8323S_MOSI_GPIO      GPIOC
#define DRV8323S_MOSI_PIN       12
#define DRV8323S_MISO_GPIO      GPIOB
#define DRV8323S_MISO_PIN       4
#define DRV8323S_SCK_GPIO       GPIOB
#define DRV8323S_SCK_PIN        3
#define DRV8323S_CS_GPIO        GPIOC
#define DRV8323S_CS_PIN         9

// In hw_init_gpio(), call:
drv8323s_init();
```

---

## Current Sensing

### 3-Shunt Configuration (Recommended)

```c
#define HW_HAS_3_SHUNTS

// One shunt in each motor phase (or low-side of each half-bridge)
// Allows current measurement in all conditions
// Required for FOC control
```

**Advantages:**
- Can sample current anytime
- Best for FOC
- No blind spots

### Phase Shunt vs Low-Side Shunt

```c
// If shunts are in phase lines (not low-side of MOSFETs)
#define HW_HAS_PHASE_SHUNTS

// Enables sampling during V0 and V7 PWM states
// Allows additional filtering
```

**Phase Shunts:**
- Shunt between half-bridge and motor
- Can sample in more PWM states
- Better noise immunity

**Low-Side Shunts:**
- Shunt between low-side MOSFET and ground
- More common in gate driver ICs
- Sample only when low-side is on

### Current Calculation

Current is measured via:
1. **Shunt voltage drop:** `V_shunt = I × R_shunt`
2. **Amplification:** `V_adc = V_shunt × Gain`
3. **ADC conversion:** Digital value
4. **Firmware conversion:**

```c
// Current calculation (simplified)
float phase_current =
    (ADC_VOLTS(ADC_IND_CURR1) - V_REG/2) /
    (CURRENT_SHUNT_RES * CURRENT_AMP_GAIN);
```

---

## Hardware Limits

Define safe operating limits for your hardware:

```c
// Current limits (Amps)
#define HW_LIM_CURRENT          -120.0, 120.0      // Motor current range
#define HW_LIM_CURRENT_IN       -120.0, 120.0      // Battery current range
#define HW_LIM_CURRENT_ABS      0.0, 160.0         // Absolute current max

// Voltage limits (Volts)
#define HW_LIM_VIN              6.0, 57.0          // Battery voltage range

// Speed limits (ERPM)
#define HW_LIM_ERPM             -200e3, 200e3      // RPM range

// Duty cycle limits
#define HW_LIM_DUTY_MIN         0.0, 0.1           // Minimum duty cycle
#define HW_LIM_DUTY_MAX         0.0, 0.99          // Maximum duty cycle

// Temperature limits (°C)
#define HW_LIM_TEMP_FET         -40.0, 110.0       // MOSFET temperature range
```

**VESC Tool** uses these limits to prevent user from entering dangerous values.

### Default Configuration Overrides

Override firmware defaults for your specific hardware:

```c
// Default motor type
#ifndef MCCONF_DEFAULT_MOTOR_TYPE
#define MCCONF_DEFAULT_MOTOR_TYPE       MOTOR_TYPE_FOC
#endif

// FOC PWM frequency
#ifndef MCCONF_FOC_F_ZV
#define MCCONF_FOC_F_ZV                 30000.0     // 30kHz PWM
#endif

// Maximum absolute current
#ifndef MCCONF_L_MAX_ABS_CURRENT
#define MCCONF_L_MAX_ABS_CURRENT        150.0       // 150A max
#endif

// FOC sampling mode
#ifndef MCCONF_FOC_SAMPLE_V0_V7
#define MCCONF_FOC_SAMPLE_V0_V7         false
#endif
```

---

## Peripheral Configuration

### CAN Bus

If your hardware has CAN:

```c
// CAN is enabled by default
// If NO CAN hardware, define:
//#define HW_HAS_NO_CAN
```

CAN pins are typically fixed:
- `PA11` = CAN_RX
- `PA12` = CAN_TX

### NRF24L01+ (Optional)

For permanently mounted wireless:

```c
#define HW_HAS_PERMANENT_NRF

#define NRF_PORT_CSN            GPIOB
#define NRF_PIN_CSN             12
#define NRF_PORT_SCK            GPIOB
#define NRF_PIN_SCK             4
#define NRF_PORT_MOSI           GPIOB
#define NRF_PIN_MOSI            3
#define NRF_PORT_MISO           GPIOD
#define NRF_PIN_MISO            2
```

### IMU (Optional)

For onboard IMU (MPU9150/MPU9250):

```c
#define MPU9X50_SDA_GPIO        GPIOB
#define MPU9X50_SDA_PIN         7
#define MPU9X50_SCL_GPIO        GPIOB
#define MPU9X50_SCL_PIN         6

// Flip axes if mounted upside-down
//#define MPU9x50_FLIP
```

---

## Creating Custom Hardware Configuration

### Step 1: Copy Example Template

```bash
cd hwconf/
cp -r example/ my_board/
cd my_board/
```

### Step 2: Rename Files

```bash
mv hw_example.h hw_my_board.h
mv hw_example_core.h hw_my_board_core.h
mv hw_example_core.c hw_my_board_core.c
```

### Step 3: Edit hw_my_board.h

```c
#ifndef HW_MY_BOARD_H_
#define HW_MY_BOARD_H_

#define HW_MY_BOARD_IS_MK1      // Your hardware revision

#include "hw_my_board_core.h"

#endif
```

### Step 4: Edit hw_my_board_core.h

1. **Update hardware name:**
```c
#define HW_NAME                 "My Custom VESC"
#define HW_MAJOR                1
#define HW_MINOR                0
```

2. **Select gate driver:**
```c
#define HW_HAS_DRV8301          // Or DRV8305, DRV8323S, etc.
```

3. **Configure pin assignments:**
   - Map all ADC channels
   - Define UART, SPI, I2C pins
   - Set LED pins
   - Configure encoder/Hall pins

4. **Set hardware parameters:**
   - Voltage divider resistors
   - Shunt resistance
   - Amplifier gain
   - Temperature sensor type

5. **Define hardware limits:**
   - Maximum current
   - Voltage range
   - Temperature limits

### Step 5: Edit hw_my_board_core.c

1. **Implement `hw_init_gpio()`:**

```c
void hw_init_gpio(void) {
    // Enable GPIO clocks
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    // Configure LED pins
    palSetPadMode(LED_GREEN_GPIO, LED_GREEN_PIN,
                  PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
    palSetPadMode(LED_RED_GPIO, LED_RED_PIN,
                  PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);

    // Configure gate enable pin
    palSetPadMode(GPIOB, 5,
                  PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
    ENABLE_GATE();

    // Configure PWM pins (TIM1 for motor control)
    palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
                  PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_FLOATING);
    palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
                  PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_FLOATING);
    palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
                  PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_FLOATING);
    palSetPadMode(GPIOB, 13, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
                  PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_FLOATING);
    palSetPadMode(GPIOB, 14, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
                  PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_FLOATING);
    palSetPadMode(GPIOB, 15, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
                  PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_FLOATING);

    // Configure Hall/encoder pins
    palSetPadMode(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1, PAL_MODE_INPUT_PULLUP);
    palSetPadMode(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2, PAL_MODE_INPUT_PULLUP);
    palSetPadMode(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3, PAL_MODE_INPUT_PULLUP);

    // Configure ADC pins (analog inputs)
    palSetPadMode(GPIOA, 0, PAL_MODE_INPUT_ANALOG);  // ADC channel 0
    palSetPadMode(GPIOA, 1, PAL_MODE_INPUT_ANALOG);  // ADC channel 1
    // ... configure all ADC pins ...

    // Initialize gate driver
    drv8301_init();  // Or drv8305_init(), etc.
}
```

2. **Implement `hw_setup_adc_channels()`:**

```c
void hw_setup_adc_channels(void) {
    uint8_t t_samp = ADC_SampleTime_15Cycles;

    // Configure regular channels for ADC1, ADC2, ADC3
    ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, t_samp);  // CURR1
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 2, t_samp);   // SENS1
    // ... configure all channels ...

    // Configure injected channels (for current sampling during PWM)
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 1, t_samp);
    ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 1, t_samp);
    ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 1, t_samp);
}
```

3. **Implement I2C functions** (if using I2C):

```c
void hw_start_i2c(void) {
    // Configure I2C pins
    // Start I2C peripheral
    // See example code for full implementation
}

void hw_stop_i2c(void) {
    // Stop I2C
    // Deconfigure pins
}

void hw_try_restore_i2c(void) {
    // I2C bus recovery if hung
}
```

### Step 6: Test Your Configuration

1. **Build firmware:**
```bash
make my_board
```

2. **Flash and test:**
   - Verify LEDs work
   - Check voltage reading
   - Test motor detection
   - Verify current sensing
   - Check all peripherals

---

## Build System Integration

### Adding to Makefile

Edit `/hwconf/hwconf.mk`:

```makefile
# Add your hardware to the list
ifeq ($(TARGET),my_board)
    MCU = STM32F405xx
    CONFDIR = hwconf/my_board
    CONFXML = my_board_no_hw.xml
    HEADER = hw_my_board.h
endif
```

### Build Commands

```bash
# Build your hardware
make my_board

# Build with upload
make my_board_upload

# Clean
make clean
```

---

## Best Practices

### 1. Start with Working Example

Copy `/hwconf/example/` or a similar existing board as your starting point.

### 2. Document Your Pin Mappings

Add comments in your header file showing physical pin mappings:

```c
/*
 * Pin Mapping:
 *
 * PA0  - ADC_IN0   - Phase A voltage sensing
 * PA1  - ADC_IN1   - Phase B voltage sensing
 * PA2  - ADC_IN2   - Phase C voltage sensing
 * PA3  - ADC_IN3   - Temperature sensor
 * ...
 */
```

### 3. Verify Hardware Parameters

Double-check:
- Voltage divider ratios
- Shunt resistor values
- Amplifier gains
- Temperature sensor type and beta value

**Incorrect values = inaccurate readings = potential damage!**

### 4. Set Conservative Limits

Start with conservative current/voltage limits, then increase based on testing:

```c
// Initial testing
#define HW_LIM_CURRENT          -30.0, 30.0      // Start low
#define HW_LIM_CURRENT_ABS      0.0, 50.0

// After validation, increase:
//#define HW_LIM_CURRENT        -120.0, 120.0
//#define HW_LIM_CURRENT_ABS    0.0, 160.0
```

### 5. Test Incrementally

1. Flash firmware, verify boot (LED blinks)
2. Test voltage measurement (use voltmeter to verify)
3. Test temperature sensors
4. Run motor detection (low current first)
5. Gradually increase current limits
6. Test all peripherals (UART, CAN, SPI, etc.)

---

## Summary

The VESC hardware configuration system provides:

✅ **Hardware Abstraction:**
- Single firmware for 82+ board variants
- Easy custom hardware development
- Modular, maintainable code

✅ **Comprehensive Configuration:**
- Pin mapping for all peripherals
- ADC channel assignment
- Gate driver selection
- Measurement calibration
- Hardware-specific limits

✅ **Safety:**
- Configurable limits prevent damage
- Hardware fault detection
- Temperature monitoring
- Overcurrent/overvoltage protection

✅ **Flexibility:**
- Support for multiple gate drivers
- Various current sensing configurations
- Optional peripherals (NRF, IMU, etc.)
- Custom features per board

**Key Files Reference:**
- `hwconf/hw.h` - Main hardware header (hw.h:20)
- `hwconf/example/hw_example_core.h` - Example configuration (hw_example_core.h:23)
- `hwconf/example/hw_example_core.c` - Example initialization (hw_example_core.c:42)
- `hwconf/hwconf.mk` - Build system integration

**Next Steps for Custom Hardware:**
1. Copy `/hwconf/example/` directory
2. Modify pin mappings and parameters
3. Implement initialization functions
4. Add to build system
5. Build, flash, and test incrementally

