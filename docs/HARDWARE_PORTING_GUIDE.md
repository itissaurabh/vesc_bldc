# VESC Hardware Porting Guide

## Overview

This guide analyzes the complexity and required modifications for common hardware changes to the VESC firmware. Use this to make informed decisions during hardware design.

**Purpose:** Help hardware designers understand firmware implications of component choices BEFORE finalizing PCB design.

**Decision Framework:**
- ✅ **Easy** - Few files, well-documented, < 1 week
- ⚠️ **Moderate** - Multiple subsystems, requires expertise, 1-4 weeks
- ❌ **Complex** - Major refactoring, high risk, 1-3 months

---

## Table of Contents

1. [Microcontroller Migration (STM32 → TI C2000)](#1-microcontroller-migration-stm32--ti-c2000)
2. [Gate Driver Changes](#2-gate-driver-changes)
3. [STM32 Pin Mapping Changes](#3-stm32-pin-mapping-changes)
4. [Other Component Changes](#4-other-component-changes)
5. [Decision Matrix](#5-decision-matrix)
6. [Recommendations](#6-recommendations)

---

## 1. Microcontroller Migration (STM32 → TI C2000)

### Complexity: ❌ **VERY COMPLEX** - Not Recommended

**Effort Estimate:** 3-6 months for experienced team

**Impact Level:** Complete firmware rewrite required

---

### 1.1 Architecture Differences

#### STM32F4 (Current)
- **Architecture:** ARM Cortex-M4
- **Clock:** 168 MHz
- **FPU:** Single-precision floating point
- **Peripherals:** Standard ARM peripherals
- **Ecosystem:** Mature ARM toolchain (GCC, Keil, IAR)
- **RTOS:** ChibiOS 3.0.5 (ARM-optimized)

#### TI C2000 (Target)
- **Architecture:** C28x fixed-point DSP
- **Clock:** 90-200 MHz (depending on variant)
- **FPU:** Single/double-precision (newer models)
- **Peripherals:** Motor-control optimized
- **Ecosystem:** TI Code Composer Studio (CCS)
- **RTOS:** Would need FreeRTOS or TI-RTOS port

---

### 1.2 Required Changes

#### **Core System Files (Complete Rewrite)**

**Complexity:** ❌ **CRITICAL**

```
Affected Files: 100+ files across entire codebase
```

**Major Areas:**

**A. Startup and Initialization**
```
Current (STM32):
└── ChibiOS/               # RTOS for ARM
    ├── chconf.h           # ChibiOS config
    ├── mcuconf.h          # MCU-specific config
    └── board files        # Board initialization

Required Changes (C2000):
└── Port entire RTOS (FreeRTOS or TI-RTOS)
    ├── Interrupt vector table (C28x format)
    ├── System clock setup (PLL, clocks)
    ├── Memory map (different from ARM)
    └── Boot sequence (C2000-specific)
```

**Files to Replace/Rewrite:**
- `ChibiOS/` - Entire RTOS (20,000+ lines)
- `main.c:40-100` - System initialization
- `mcuconf.h` - All peripheral clocks
- `chconf.h` - OS configuration
- `board.c/h` - Board initialization

---

**B. PWM Generation (High Priority)**

**Complexity:** ❌ **CRITICAL**

Current STM32 implementation uses **Advanced Timers (TIM1/TIM8)**:

```c
// Current: STM32 timer registers
TIM1->CCR1 = duty_cycle;  // ARM register access
TIM1->CR1 |= TIM_CR1_CEN; // Enable timer
```

C2000 uses **ePWM modules** (completely different):

```c
// Required: C2000 ePWM registers
EPwm1Regs.CMPA.bit.CMPA = duty_cycle;  // Different structure
EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
```

**Files Requiring Complete Rewrite:**

1. **`mcpwm_foc.c` (2000+ lines)** - FOC PWM generation
   ```
   Lines 300-500:   Timer configuration
   Lines 800-1200:  ADC synchronization
   Lines 1500-1800: Dead-time insertion
   ```

   **C2000 Equivalents Needed:**
   - ePWM module setup (different register structure)
   - HRPWM for high-resolution edges
   - Trip-zone configuration
   - Time-base synchronization

2. **`mcpwm.c` (1200+ lines)** - BLDC PWM generation
   - Similar complete rewrite needed

3. **`hw_*.c` files** - All hardware configurations
   - PWM pin mapping (C2000 GPIO muxing different)
   - ADC trigger setup (SOCA/SOCB on C2000)

**Specific Challenges:**
- **Dead-time Generation:** STM32 has built-in dead-time. C2000 uses rising/falling edge delays (different concept)
- **Center-aligned PWM:** Different implementation on ePWM
- **ADC Triggering:** C2000 SOCA/SOCB vs STM32 timer events

---

**C. ADC Sampling (Critical for Control)**

**Complexity:** ❌ **CRITICAL**

Current STM32 uses **3× ADC modules** in interleaved mode:

```c
// STM32: Triple interleaved ADC
ADC1->CR2 |= ADC_CR2_SWSTART;
ADC2->CR2 |= ADC_CR2_SWSTART;
ADC3->CR2 |= ADC_CR2_SWSTART;

// Read via DMA
uint16_t adc_buffer[18];
```

C2000 uses **ADC-A, ADC-B, ADC-C, ADC-D** with SOC (Start of Conversion):

```c
// C2000: SOC-based ADC
AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0;  // Channel 0
AdcaRegs.ADCSOC0CTL.bit.ACQPS = 14; // Acquisition window
AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5; // ePWM1 SOCA trigger
```

**Files Requiring Complete Rewrite:**

1. **`mcpwm_foc.c:400-600`** - ADC ISR and current sampling
   ```
   Critical section: 3-phase current measurement
   Timing requirement: < 2µs from PWM center to reading
   ```

2. **`hw_*.c`** - ADC channel mapping
   - STM32: Channel numbers (ADC1_IN0, ADC2_IN1, etc.)
   - C2000: ADCINA0, ADCINB1, etc. (different naming)

3. **DMA Removal** - C2000 doesn't use DMA for ADC in same way
   - Replace with result register reads
   - Different interrupt structure

**Specific Challenges:**
- **Simultaneous Sampling:** C2000 simultaneous sampling different from STM32
- **Sample Timing:** Critical for FOC (must sample at PWM center)
- **Calibration:** Different offset/gain correction

---

**D. Interrupts and ISR**

**Complexity:** ❌ **CRITICAL**

```
Current (STM32):          Required (C2000):
├── NVIC (ARM)           ├── PIE (Peripheral Interrupt Expansion)
├── 82 interrupts        ├── 96+ interrupts
├── 16 priority levels   ├── Different priority scheme
└── Standard ARM ISR     └── C28x ISR syntax
```

**Every ISR Needs Modification:**

```c
// STM32 ISR
void TIM1_UP_TIM10_IRQHandler(void) {
    // ARM syntax
    TIM1->SR = ~TIM_SR_UIF;
}

// C2000 ISR (different syntax)
__interrupt void epwm1_isr(void) {
    // C28x syntax
    EPwm1Regs.ETCLR.bit.INT = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}
```

**Files Affected:**
- `mcpwm_foc.c` - ADC ISR (most critical)
- `encoder/*.c` - Timer ISRs for encoders
- `comm/*.c` - UART/CAN ISRs
- `driver/*.c` - Peripheral ISRs

---

**E. Communication Peripherals**

**1. CAN Bus**

**Complexity:** ❌ **MAJOR REWRITE**

```
STM32 bxCAN:              C2000 eCAN:
├── 2× CAN peripherals   ├── eCAN-A, eCAN-B
├── 28 filter banks      ├── 32 mailboxes
├── 3 TX mailboxes       ├── Different acceptance filtering
└── HAL drivers          └── No HAL - register level
```

**Files:**
- `comm/comm_can.c` (800+ lines) - Complete rewrite
- Different register structure
- Different filtering mechanism

**2. UART**

**Complexity:** ⚠️ **MODERATE**

C2000 has SCI (Serial Communication Interface) instead of USART.

**Files:**
- `comm/packet.c` - UART handling
- DMA changes (C2000 DMA different)

**3. USB**

**Complexity:** ❌ **CRITICAL**

C2000 has limited/no native USB support (depends on variant).

**Options:**
- Use external USB-to-UART chip (FT232, etc.)
- Some C2000F28379D has USB peripheral
- Complete USB stack rewrite needed

**Files:**
- Entire USB stack (if keeping USB)
- May need to drop USB entirely

---

**F. Memory Map and Linker**

**Complexity:** ❌ **CRITICAL**

```
STM32 Memory Map:         C2000 Memory Map:
├── Flash: 0x08000000    ├── Flash: 0x080000 (different)
├── SRAM: 0x20000000     ├── M0 SRAM: 0x000000
├── CCM: 0x10000000      ├── M1 SRAM: 0x000400
└── Peripherals: 0x40000000 └── Peripherals: 0x007000

Linker Script:            Linker Script:
├── ARM GCC format       ├── TI linker format (completely different)
├── ld file              ├── .cmd file
└── Sections for ARM     └── Sections for C28x
```

**Files:**
- `ld/STM32F405_*.ld` - Complete rewrite
- New: `.cmd` files for C2000
- `conf_general.h` - Memory addresses

---

**G. Compiler and Toolchain**

**Complexity:** ❌ **CRITICAL**

```
Current:                  Required:
├── ARM GCC              ├── TI C2000 Compiler (CCS)
├── Make-based           ├── CCS project files
├── GCC extensions       ├── TI compiler pragmas
└── ARM assembly         └── C28x assembly
```

**Changes:**
- Entire build system (Makefile → CCS project)
- Compiler-specific code (inline assembly, attributes)
- Optimization flags different

---

### 1.3 Components That Would Work (Minimal Changes)

#### ✅ **Application Logic**

These would mostly work with API adaptation:

- `applications/app_adc.c` - Control logic (if APIs updated)
- `applications/app_ppm.c` - PPM decoding (if timer APIs updated)
- High-level state machines
- LispBM (if ported to C2000)

#### ✅ **Algorithms**

Math-heavy code would port easier:

- `motor/mcpwm_foc.c:1000-1500` - FOC math (Park/Clarke transforms)
- PID controllers
- Filters (observer, Kalman, etc.)

---

### 1.4 Risks and Challenges

**Critical Risks:**

1. **Real-Time Performance**
   - C28x is fixed-point DSP (older models)
   - ARM Cortex-M4 has FPU (faster for floating-point FOC)
   - May need to convert floating-point to fixed-point

2. **Tool Ecosystem**
   - ChibiOS not available for C2000
   - Need to port or use different RTOS
   - Different debugging tools

3. **Testing and Validation**
   - Every feature needs re-testing
   - Motor control timing critical
   - Safety implications

4. **Community Support**
   - VESC community uses STM32
   - No existing C2000 VESC designs
   - Limited help available

---

### 1.5 Recommendation

**❌ DO NOT migrate to TI C2000 unless absolutely necessary**

**Why Keep STM32:**
- Proven design (10+ years in field)
- Large community support
- Mature toolchain (free GCC)
- ChibiOS well-suited for motor control
- Lower development risk

**If Motor-Control Features Needed:**
- STM32G4 series has motor-control peripherals
- Maintains ARM ecosystem
- Pin-compatible upgrades possible

**When C2000 Makes Sense:**
- Corporate requirement (TI-only shop)
- Need C2000-specific features
- Team has C2000 expertise
- Budget for 6+ month development

**Better Alternative:**
- Use STM32G473 or STM32F7 (motor-control optimized)
- Keep existing firmware with minor changes
- Upgrade path instead of complete rewrite

---

## 2. Gate Driver Changes

### Complexity: ✅ **EASY to ⚠️ MODERATE** (Depends on Driver)

**Effort Estimate:** 1-3 days per gate driver variant

---

### 2.1 Current Supported Gate Drivers

VESC firmware already supports **5 different gate drivers**:

```c
// From hwconf/hw.h
#define HW_HAS_DRV8301          // TI DRV8301 (most common)
#define HW_HAS_DRV8305          // TI DRV8305
#define HW_HAS_DRV8320S         // TI DRV8320S
#define HW_HAS_DRV8323S         // TI DRV8323S
#define HW_HAS_DRV8316          // TI DRV8316 (newer)
```

**Good News:** Infrastructure for multiple drivers already exists!

---

### 2.2 Adding Microchip MIC4104

**Complexity:** ⚠️ **MODERATE**

**Effort:** 2-5 days

#### MIC4104 Overview

**Type:** 3-phase MOSFET driver (bootstrap)

**Key Differences from DRV8301:**
- No SPI interface (simpler - only PWM input)
- No current shunt amplifiers (external op-amps needed)
- No fault reporting via SPI
- Simpler fault outputs (digital pins)

#### Required Changes

**A. Hardware Configuration File**

**Files to Create/Modify:**

1. **Create: `hwconf/mic4104/hw_mic4104_core.h`**

```c
#ifndef HW_MIC4104_CORE_H_
#define HW_MIC4104_CORE_H_

// Define that this hardware uses MIC4104
#define HW_HAS_MIC4104

// Hardware name
#define HW_NAME                 "MIC4104"

// Pin definitions
#define HW_GATE_DRIVER_ENABLE_GPIO  GPIOB
#define HW_GATE_DRIVER_ENABLE_PIN   5

// Fault pin (MIC4104 /FAULT output)
#define HW_GATE_DRIVER_FAULT_GPIO   GPIOB
#define HW_GATE_DRIVER_FAULT_PIN    7

// No SPI (MIC4104 has no SPI)
// #define HW_GATE_DRIVER_SPI (not needed)

// Current sensing via external op-amps
#define CURRENT_SHUNT_RES       0.0005  // 0.5mΩ
#define CURRENT_AMP_GAIN        20.0    // External op-amp gain

// ADC channels for current sensing
#define ADC_IND_CURR1           ADC_IND_SENS1
#define ADC_IND_CURR2           ADC_IND_SENS2
#define ADC_IND_CURR3           ADC_IND_SENS3

// Hardware limits
#define HW_LIM_CURRENT          -120.0, 120.0
#define HW_LIM_CURRENT_IN       -120.0, 120.0
#define HW_LIM_CURRENT_ABS      0.0, 160.0
#define HW_LIM_VIN              6.0, 57.0
#define HW_LIM_ERPM             -100000.0, 100000.0
#define HW_LIM_DUTY_MIN         0.0, 0.1
#define HW_LIM_DUTY_MAX         0.0, 0.99
#define HW_LIM_TEMP_FET         -40.0, 110.0

#endif /* HW_MIC4104_CORE_H_ */
```

2. **Create: `hwconf/mic4104/hw_mic4104_core.c`**

```c
#include "hw_mic4104_core.h"

void hw_init_gpio(void) {
    // Enable pin as output
    palSetPadMode(HW_GATE_DRIVER_ENABLE_GPIO,
                  HW_GATE_DRIVER_ENABLE_PIN,
                  PAL_MODE_OUTPUT_PUSHPULL);

    // Fault pin as input with pull-up
    palSetPadMode(HW_GATE_DRIVER_FAULT_GPIO,
                  HW_GATE_DRIVER_FAULT_PIN,
                  PAL_MODE_INPUT_PULLUP);

    // Start with driver disabled
    palClearPad(HW_GATE_DRIVER_ENABLE_GPIO,
                HW_GATE_DRIVER_ENABLE_PIN);
}

void hw_setup_gate_driver(void) {
    // MIC4104 has no SPI configuration
    // Just enable the driver
    chThdSleepMilliseconds(10);  // Power-up delay

    // Enable gate driver
    palSetPad(HW_GATE_DRIVER_ENABLE_GPIO,
              HW_GATE_DRIVER_ENABLE_PIN);

    chThdSleepMilliseconds(1);  // Stabilization
}

bool hw_gate_driver_fault(void) {
    // MIC4104 /FAULT is active low
    return !palReadPad(HW_GATE_DRIVER_FAULT_GPIO,
                       HW_GATE_DRIVER_FAULT_PIN);
}

float hw_get_current_scale(void) {
    // Current = (ADC_voltage - offset) / (shunt_R * gain)
    return 1.0 / (CURRENT_SHUNT_RES * CURRENT_AMP_GAIN);
}
```

**B. Fault Handling**

**Modify: `mcpwm_foc.c`** (or create abstraction)

Current code checks DRV8301 faults via SPI. For MIC4104:

```c
// Add to mcpwm_foc.c or hw_mic4104_core.c
static void check_gate_driver_fault(void) {
#ifdef HW_HAS_MIC4104
    if (hw_gate_driver_fault()) {
        mc_interface_fault_stop(FAULT_CODE_DRV);
        // Optional: Read which phase faulted via additional pins
    }
#elif defined(HW_HAS_DRV8301)
    // Existing DRV8301 SPI fault check
#endif
}
```

**C. Current Sensing Calibration**

MIC4104 has no programmable gain like DRV8301. Current sensing uses external op-amps.

**Modify: `mcpwm_foc.c:200-300`** - DC offset calibration

```c
void mcpwm_foc_adc_int_handler(void) {
    // ... existing code ...

#ifdef HW_HAS_MIC4104
    // External op-amps may have different offset
    // Calibrate during init
    float offset1 = ADC_V_MIDDLE;  // May need tuning
    float offset2 = ADC_V_MIDDLE;
    float offset3 = ADC_V_MIDDLE;
#elif defined(HW_HAS_DRV8301)
    // DRV8301 offset
#endif

    // ... rest of code ...
}
```

#### Testing Required

1. **Fault Detection:**
   - Short circuit test
   - Over-current test
   - Under-voltage lockout test

2. **Current Sensing:**
   - Verify current measurement accuracy
   - Check all three phases
   - Validate at different currents

3. **Thermal:**
   - Ensure driver doesn't overheat
   - Check bootstrap capacitor sizing

---

### 2.3 Adding Infineon Gate Driver

**Example: Infineon IR2103** (half-bridge driver)

**Complexity:** ⚠️ **MODERATE**

**Differences from DRV8301:**
- No integrated current sensing
- No SPI interface
- Separate drivers per half-bridge (need 3× IR2103)
- SD (Shutdown) pins instead of enable

**Required Changes:**

Similar to MIC4104 but:
- Different enable logic (SD is shutdown, not enable)
- Need to handle 3 separate driver ICs
- Different fault detection (no fault pin)

**Files:**
```
hwconf/infineon/hw_infineon_core.h
hwconf/infineon/hw_infineon_core.c
```

**Code Example:**

```c
// hw_infineon_core.c
void hw_setup_gate_driver(void) {
    // Infineon SD pins (active high to shutdown)
    // Keep low to enable
    palClearPad(SD_GPIO, SD_PIN_U);
    palClearPad(SD_GPIO, SD_PIN_V);
    palClearPad(SD_GPIO, SD_PIN_W);
}

void hw_gate_driver_fault(void) {
    // Infineon drivers don't have fault output
    // Monitor via ADC (over-current) or external comparators
    return false;  // Or implement external fault detection
}
```

---

### 2.4 Gate Driver Comparison

| Feature | DRV8301 | DRV8305 | MIC4104 | IR2103 |
|---------|---------|---------|---------|--------|
| **Integration** | High | High | Medium | Low |
| **Current Sense** | Built-in | Built-in | External | External |
| **SPI Config** | ✅ Yes | ✅ Yes | ❌ No | ❌ No |
| **Fault Reporting** | SPI | SPI | Pin | None |
| **Firmware Complexity** | High | High | Low | Low |
| **Parts Count** | 1 IC | 1 IC | 1 IC + op-amps | 3 ICs |
| **Porting Effort** | Already done | Already done | 2-3 days | 2-3 days |

---

### 2.5 Recommendation for Gate Drivers

**✅ RECOMMENDED: Stick with TI DRV Series**

**Why:**
- Already supported (5 variants)
- Integrated current sensing
- SPI configuration
- Fault reporting
- Proven in field

**If You Must Change:**

**Best Alternative: MIC4104 or similar bootstrap driver**
- Moderate complexity (2-3 days)
- Well-defined interface
- Clear documentation

**Second Choice: Infineon drivers**
- More complex (3 separate ICs)
- Limited fault detection
- Higher parts count

**Custom Design Considerations:**
1. **Current Sensing:** Ensure 3-shunt or phase shunt architecture
2. **Fault Detection:** Need /FAULT or similar output
3. **Enable Logic:** Simple enable/disable interface
4. **Bootstrap:** Proper bootstrap capacitor design critical
5. **Dead-time:** Verify dead-time requirements match

---

## 3. STM32 Pin Mapping Changes

### Complexity: ✅ **EASY** - Recommended Flexibility

**Effort Estimate:** 1-2 days per board variant

**Impact:** Minimal - well-supported in firmware

---

### 3.1 Current Pin Mapping System

VESC firmware has **excellent support** for different pin mappings via hardware configuration files.

**Example: 82+ board variants already exist:**
```
hwconf/
├── 100_250/
├── 410/
├── 60/
├── hd60/
├── mini4/
└── ... (82+ variants)
```

Each variant has different STM32 pin assignments!

---

### 3.2 Pin Categories

Pins are organized by function:

#### **A. Motor Control Pins (Critical Timing)**

**1. PWM Outputs (6 pins)**
```c
// From hwconf/hw.h
#define HW_PWM_U_HIGH_GPIO      GPIOA
#define HW_PWM_U_HIGH_PIN       8
#define HW_PWM_U_LOW_GPIO       GPIOA
#define HW_PWM_U_LOW_PIN        7

#define HW_PWM_V_HIGH_GPIO      GPIOA
#define HW_PWM_V_HIGH_PIN       9
#define HW_PWM_V_LOW_GPIO       GPIOB
#define HW_PWM_V_LOW_PIN        0

#define HW_PWM_W_HIGH_GPIO      GPIOA
#define HW_PWM_W_HIGH_PIN       10
#define HW_PWM_W_LOW_GPIO       GPIOB
#define HW_PWM_W_LOW_PIN        1
```

**Constraints:**
- Must use timer channels (TIM1 or TIM8)
- Complementary pairs on same timer
- Example valid combinations:
  ```
  TIM1_CH1: PA8 (high) + PA7 or PB13 (low)
  TIM1_CH2: PA9 (high) + PB0 or PB14 (low)
  TIM1_CH3: PA10 (high) + PB1 or PB15 (low)
  ```

**2. ADC Inputs (10-15 pins)**
```c
// Current sensing (critical - must be on ADC1/2/3)
#define HW_ADC_CURR1            ADC_Channel_0   // Phase U current
#define HW_ADC_CURR2            ADC_Channel_3   // Phase V current
#define HW_ADC_CURR3            ADC_Channel_6   // Phase W current

// Voltage/temperature sensing
#define HW_ADC_VOLT             ADC_Channel_10  // Bus voltage
#define HW_ADC_TEMP_MOTOR       ADC_Channel_11  // Motor temp
#define HW_ADC_TEMP_FET         ADC_Channel_12  // MOSFET temp

// User inputs
#define HW_ADC_EXT              ADC_Channel_8   // External ADC
```

**Constraints:**
- Current sensing must use ADC1/2/3 for simultaneous sampling
- Needs timer trigger (usually TIM1)

**3. Hall Sensor Inputs (3 pins)**
```c
#define HW_HALL_GPIO_1          GPIOC
#define HW_HALL_PIN_1           6
#define HW_HALL_GPIO_2          GPIOC
#define HW_HALL_PIN_2           7
#define HW_HALL_GPIO_3          GPIOC
#define HW_HALL_PIN_3           8
```

**Constraints:**
- Can be any GPIO
- Optional: use timer inputs for hardware capture

#### **B. Communication Pins**

**1. CAN Bus**
```c
#define HW_CAN_GPIO_TX          GPIOB
#define HW_CAN_PIN_TX           9
#define HW_CAN_GPIO_RX          GPIOB
#define HW_CAN_PIN_RX           8
```

**Constraints:**
- Must use CAN peripheral pins (CAN1 or CAN2)
- STM32F405: CAN1 on PB8/PB9 or PA11/PA12
- STM32F405: CAN2 on PB5/PB6 or PB12/PB13

**2. UART/USB**
```c
#define HW_UART_GPIO_TX         GPIOC
#define HW_UART_PIN_TX          10
#define HW_UART_GPIO_RX         GPIOC
#define HW_UART_PIN_RX          11

// USB (usually fixed)
#define HW_USB_GPIO_DM          GPIOA
#define HW_USB_PIN_DM           11
#define HW_USB_GPIO_DP          GPIOA
#define HW_USB_PIN_DP           12
```

**Constraints:**
- UART: Any UART peripheral pins
- USB: Usually PA11/PA12 (USB_DM/USB_DP)

**3. SPI (for encoder/IMU)**
```c
#define HW_SPI_GPIO_SCK         GPIOB
#define HW_SPI_PIN_SCK          13
#define HW_SPI_GPIO_MOSI        GPIOB
#define HW_SPI_PIN_MOSI         15
#define HW_SPI_GPIO_MISO        GPIOB
#define HW_SPI_PIN_MISO         14
#define HW_SPI_GPIO_NSS         GPIOB
#define HW_SPI_PIN_NSS          12
```

**Constraints:**
- Must use SPI peripheral pins (SPI1, SPI2, or SPI3)

#### **C. GPIO (Flexible)**

```c
// Status LEDs
#define LED_GREEN_GPIO          GPIOB
#define LED_GREEN_PIN           0
#define LED_RED_GPIO            GPIOB
#define LED_RED_PIN             1

// Enable/shutdown
#define HW_SHUTDOWN_GPIO        GPIOC
#define HW_SHUTDOWN_PIN         13
```

**Constraints:**
- Any available GPIO
- Most flexible category

---

### 3.3 How to Create Custom Pin Mapping

**Step-by-Step Process:**

#### **Step 1: Create Hardware Directory**

```bash
mkdir hwconf/my_custom_board/
```

#### **Step 2: Create Pin Definition File**

**File: `hwconf/my_custom_board/hw_my_custom_board.h`**

```c
#ifndef HW_MY_CUSTOM_BOARD_H_
#define HW_MY_CUSTOM_BOARD_H_

#define HW_NAME                 "MY_CUSTOM_BOARD"
#define HW_MAJOR                1
#define HW_MINOR                0

// Which gate driver?
#define HW_HAS_DRV8301          // Or your chosen driver

// Hardware limits
#define HW_LIM_CURRENT          -120.0, 120.0
#define HW_LIM_VIN              6.0, 57.0

// ===== PWM PINS (Use TIM1) =====
// Must use timer-capable pins!
#define HW_PWM_U_HIGH_GPIO      GPIOA    // TIM1_CH1
#define HW_PWM_U_HIGH_PIN       8
#define HW_PWM_U_LOW_GPIO       GPIOA    // TIM1_CH1N
#define HW_PWM_U_LOW_PIN        7

#define HW_PWM_V_HIGH_GPIO      GPIOA    // TIM1_CH2
#define HW_PWM_V_HIGH_PIN       9
#define HW_PWM_V_LOW_GPIO       GPIOB    // TIM1_CH2N
#define HW_PWM_V_LOW_PIN        0

#define HW_PWM_W_HIGH_GPIO      GPIOA    // TIM1_CH3
#define HW_PWM_W_HIGH_PIN       10
#define HW_PWM_W_LOW_GPIO       GPIOB    // TIM1_CH3N
#define HW_PWM_W_LOW_PIN        1

// ===== ADC PINS =====
// Check STM32 datasheet for ADC channel mapping
#define HW_ADC_CURR1            ADC_Channel_0    // PA0 -> ADC123_IN0
#define HW_ADC_CURR2            ADC_Channel_3    // PA3 -> ADC123_IN3
#define HW_ADC_CURR3            ADC_Channel_6    // PA6 -> ADC12_IN6

#define HW_ADC_VOLT             ADC_Channel_12   // PC2 -> ADC123_IN12
#define HW_ADC_TEMP_MOTOR       ADC_Channel_10   // PC0 -> ADC123_IN10
#define HW_ADC_TEMP_FET         ADC_Channel_8    // PB0 -> ADC12_IN8

// ===== CAN PINS =====
// CAN1 on PB8/PB9
#define HW_CAN_GPIO_TX          GPIOB
#define HW_CAN_PIN_TX           9
#define HW_CAN_GPIO_RX          GPIOB
#define HW_CAN_PIN_RX           8

// ===== UART PINS =====
// USART3 on PC10/PC11
#define HW_UART_GPIO_TX         GPIOC
#define HW_UART_PIN_TX          10
#define HW_UART_GPIO_RX         GPIOC
#define HW_UART_PIN_RX          11

// ===== HALL SENSOR PINS =====
#define HW_HALL_GPIO_1          GPIOC
#define HW_HALL_PIN_1           6
#define HW_HALL_GPIO_2          GPIOC
#define HW_HALL_PIN_2           7
#define HW_HALL_GPIO_3          GPIOC
#define HW_HALL_PIN_3           8

// ===== LED PINS =====
#define LED_GREEN_GPIO          GPIOB
#define LED_GREEN_PIN           5
#define LED_RED_GPIO            GPIOB
#define LED_RED_PIN             7

// ===== GATE DRIVER PINS =====
#define HW_GATE_DRIVER_ENABLE_GPIO  GPIOB
#define HW_GATE_DRIVER_ENABLE_PIN   3

// If using DRV8301 with SPI:
#define DRV8301_MOSI_GPIO       GPIOB
#define DRV8301_MOSI_PIN        15
#define DRV8301_MISO_GPIO       GPIOB
#define DRV8301_MISO_PIN        14
#define DRV8301_SCK_GPIO        GPIOB
#define DRV8301_SCK_PIN         13
#define DRV8301_CS_GPIO         GPIOB
#define DRV8301_CS_PIN          12

#endif /* HW_MY_CUSTOM_BOARD_H_ */
```

#### **Step 3: Create Initialization File (Optional)**

**File: `hwconf/my_custom_board/hw_my_custom_board.c`**

```c
#include "hw_my_custom_board.h"
#include "mc_interface.h"
#include "mcpwm_foc.h"

void hw_init_gpio(void) {
    // Initialize any custom GPIO
    // Most initialization is automatic from .h file

    // Example: Extra features
    palSetPadMode(EXTRA_OUTPUT_GPIO, EXTRA_OUTPUT_PIN,
                  PAL_MODE_OUTPUT_PUSHPULL);
}

void hw_setup_adc_channels(void) {
    // Usually automatic, but can customize
    // Example: Different ADC scaling
}

// Optional: Custom current sensor configuration
float hw_read_phase_current_u(void) {
    // If using non-standard current sensing
    // Otherwise use default implementation
}
```

#### **Step 4: Add to Build System**

**Modify: `Makefile`**

```makefile
# Add your board to the list
ifeq ($(BOARD), MY_CUSTOM_BOARD)
    HWSRC = hwconf/my_custom_board/hw_my_custom_board.c
    HWINC = hwconf/my_custom_board
    HWDEF = -DHW_MY_CUSTOM_BOARD
endif
```

#### **Step 5: Build and Test**

```bash
make BOARD=MY_CUSTOM_BOARD
```

---

### 3.4 Pin Mapping Validation Checklist

Before finalizing PCB:

**✅ PWM Pins:**
- [ ] All 6 PWM pins on TIM1 or TIM8
- [ ] Complementary pairs correct (CH1/CH1N, etc.)
- [ ] Check STM32 datasheet alternate functions

**✅ ADC Pins:**
- [ ] Current sense on ADC1, ADC2, ADC3
- [ ] All ADC channels available on chosen pins
- [ ] Voltage sense on valid ADC channel

**✅ Communication:**
- [ ] CAN TX/RX on CAN peripheral pins
- [ ] UART TX/RX on UART peripheral pins
- [ ] SPI pins on SPI peripheral (if used)

**✅ Conflicts:**
- [ ] No pin used twice
- [ ] No conflicts with internal peripherals
- [ ] Check errata for pin limitations

**✅ Testing:**
- [ ] Test PWM generation on all 6 pins
- [ ] Verify ADC readings
- [ ] Test CAN communication
- [ ] Verify Hall sensor inputs

---

### 3.5 Common Pitfalls

**❌ Wrong Timer:**
```c
// WRONG: PA0 is not a timer pin
#define HW_PWM_U_HIGH_PIN   0  // PA0 - Error!

// CORRECT: PA8 is TIM1_CH1
#define HW_PWM_U_HIGH_PIN   8  // PA8 - OK
```

**❌ Non-simultaneous ADC:**
```c
// WRONG: ADC_IN16 not on ADC1/2/3
#define HW_ADC_CURR1   ADC_Channel_16  // Temperature sensor - Error!

// CORRECT: Use ADC1/2/3 channels
#define HW_ADC_CURR1   ADC_Channel_0   // PA0 - OK
```

**❌ Pin Conflicts:**
```c
// WRONG: Same pin for two functions
#define HW_PWM_U_HIGH_PIN       8   // PA8
#define LED_GREEN_PIN           8   // PA8 - Conflict!

// CORRECT: Different pins
#define HW_PWM_U_HIGH_PIN       8   // PA8
#define LED_GREEN_PIN           5   // PB5 - OK
```

---

### 3.6 Recommendation for Pin Mapping

**✅ HIGHLY FLEXIBLE - Change pins freely**

**Guidelines:**

1. **Follow Reference Designs:**
   - Start with similar VESC variant
   - Copy proven pin mapping
   - Make minimal changes

2. **Validate with STM32 Datasheet:**
   - Check alternate function table
   - Verify timer channels
   - Confirm ADC channels

3. **Use Hardware Abstraction:**
   - All pins defined in hw_*.h
   - No hardcoded pins in source
   - Easy to modify

4. **Test Early:**
   - Validate each subsystem independently
   - PWM first, then ADC, then communication

5. **Document Changes:**
   - Comment why each pin chosen
   - Note any special requirements
   - Keep schematic updated

---

## 4. Other Component Changes

### 4.1 Current Shunt Resistors

**Complexity:** ✅ **EASY**

**Change Required:** Update hardware configuration

```c
// hwconf/hw_my_board.h
#define CURRENT_SHUNT_RES    0.0005   // 0.5mΩ (change this)
#define CURRENT_AMP_GAIN     20.0     // Or different gain
```

**Recalculate:**
- Maximum measurable current
- ADC resolution per amp
- Ensure ADC doesn't saturate

---

### 4.2 MOSFETs

**Complexity:** ✅ **EASY**

**Usually no firmware changes** needed if:
- Same package (TO-220, D2PAK, etc.)
- Similar Rds(on) and Qg
- Same voltage rating

**May need adjustment:**
- Dead-time (if very different Qg)
- Gate driver current limit

```c
// mcpwm_foc.c - Dead time adjustment
#define HW_DEAD_TIME_NSEC    360  // Adjust for MOSFET switching
```

---

### 4.3 Microcontroller Variant (STM32F4 family)

**Complexity:** ✅ **EASY to ⚠️ MODERATE**

**Within STM32F4 family:**

| Change | Complexity | Notes |
|--------|------------|-------|
| F405 → F407 | ✅ Very Easy | Almost identical, just more peripherals |
| F405 → F446 | ✅ Easy | Compatible, more features |
| F405 → F7 | ⚠️ Moderate | Faster, some peripheral differences |
| F4 → G4 | ⚠️ Moderate | Motor-control optimized, peripheral changes |
| F4 → H7 | ⚠️ Moderate | Much faster, different architecture |

**Files to Change:**
```
Makefile          - MCU definition
ld/*.ld           - Linker script (if memory different)
mcuconf.h         - Peripheral clocks
```

**Example: F405 → F446**
```makefile
# Makefile
MCU = cortex-m4
MCU_TYPE = STM32F446xx  # Changed from STM32F40_41xxx
```

---

### 4.4 Encoders

**Complexity:** ✅ **EASY**

Already supports **10+ encoder types**:
- ABI quadrature
- AS5047 (SPI)
- MT6816 (SPI)
- AD2S1205 (resolver)
- Many more...

**To add new encoder:** See `encoder/` directory. Similar to gate driver - create wrapper.

---

### 4.5 IMU Sensors

**Complexity:** ✅ **EASY**

Already supports **4 IMU types**:
- MPU9150/MPU9250
- ICM20948
- LSM6DS3
- BMI160

**To add new IMU:** See `imu/` directory.

---

## 5. Decision Matrix

### Complexity vs Benefit Analysis

| Change | Complexity | Time | Risk | Benefit | Recommended |
|--------|------------|------|------|---------|-------------|
| **Microcontroller Family** |
| STM32 F4 → F7 | ⚠️ Moderate | 1-2 weeks | Medium | Faster CPU | ⚠️ Maybe |
| STM32 F4 → G4 | ⚠️ Moderate | 2-3 weeks | Medium | Motor peripherals | ⚠️ Maybe |
| STM32 → TI C2000 | ❌ Very High | 3-6 months | Very High | DSP features | ❌ No |
| STM32 → Different ARM | ❌ High | 2-4 months | High | Vendor preference | ❌ No |
| **Gate Driver** |
| TI DRV → DRV variant | ✅ None | 0 days | None | Cost/features | ✅ Yes |
| TI DRV → MIC4104 | ⚠️ Moderate | 2-5 days | Low | Cost | ⚠️ Maybe |
| TI DRV → Infineon | ⚠️ Moderate | 3-7 days | Low | Availability | ⚠️ Maybe |
| TI DRV → Custom | ❌ High | 1-2 weeks | Medium | Special needs | ❌ Rarely |
| **Pin Mapping** |
| Within STM32F4 | ✅ Easy | 1-2 days | Low | PCB optimization | ✅ Yes |
| Change peripherals | ⚠️ Moderate | 3-5 days | Medium | Routing | ⚠️ Maybe |
| **Other Components** |
| Shunt resistor | ✅ Very Easy | 1 hour | None | Cost/accuracy | ✅ Yes |
| MOSFET type | ✅ Easy | 2 hours | Low | Performance | ✅ Yes |
| Encoder type | ✅ Easy | 1-2 days | Low | Requirements | ✅ Yes |
| IMU type | ✅ Easy | 1-2 days | Low | Features | ✅ Yes |

---

## 6. Recommendations

### For New Hardware Design

**DO:** ✅
1. **Keep STM32F4 architecture**
   - Proven, well-supported
   - Easy firmware updates
   - Large community

2. **Use existing gate driver**
   - DRV8301, DRV8305, DRV8320S
   - Integrated current sensing
   - SPI configuration

3. **Customize pin mapping freely**
   - Well-supported in firmware
   - Low risk, easy validation
   - Optimize for PCB layout

4. **Select appropriate encoder**
   - Many options supported
   - Choose based on application
   - Easy to add new types

5. **Adjust shunts/MOSFETs as needed**
   - Simple configuration changes
   - Optimize for current/voltage

**DON'T:** ❌
1. **Change microcontroller family**
   - Massive effort (months)
   - High risk of bugs
   - Lose community support

2. **Use exotic gate drivers**
   - Unless unavoidable
   - Adds development time
   - Testing complexity

3. **Change communication protocols**
   - CAN bus well-established
   - USB widely supported
   - Breaking compatibility risky

---

### Decision Tree

```
Need Different Hardware?
│
├─ Different STM32 pins?
│  └─ ✅ DO IT - Easy, 1-2 days
│
├─ Different gate driver (still bootstrap)?
│  ├─ Still TI DRV series? ✅ DO IT - Already supported
│  ├─ Microchip/Infineon? ⚠️ MAYBE - 2-5 days work
│  └─ Custom/exotic? ❌ AVOID - Unless necessary
│
├─ Different microcontroller?
│  ├─ Same STM32F4 variant? ✅ DO IT - Easy
│  ├─ STM32F7/G4? ⚠️ MAYBE - If benefits clear
│  └─ Different architecture (C2000, etc.)? ❌ AVOID - Massive effort
│
├─ Different current sensing?
│  ├─ Different shunt value? ✅ DO IT - Config change
│  └─ Different architecture? ⚠️ REQUIRES REVIEW
│
└─ Different sensors (encoder/IMU)?
   └─ ✅ LIKELY SUPPORTED - Check existing drivers
```

---

### Cost-Benefit Analysis Example

**Scenario: Want to use cheaper gate driver**

**Option A: TI DRV8305 → DRV8320S**
- Effort: 0 hours (already supported)
- Risk: None
- Savings: $2-3 per unit
- **Decision: ✅ DO IT**

**Option B: TI DRV8301 → MIC4104**
- Effort: 16-40 hours (2-5 days)
- Risk: Low (similar architecture)
- Savings: $3-5 per unit
- Additional: Need external op-amps (+$1)
- **Decision: ⚠️ MAYBE** - If volume justifies development

**Option C: TI DRV8301 → Custom discrete**
- Effort: 80-160 hours (2-4 weeks)
- Risk: High (testing, validation)
- Savings: $5-8 per unit
- Additional: More PCB area, complexity
- **Decision: ❌ AVOID** - Unless very high volume (>10k units)

---

## Summary

### Quick Reference Table

| Component | Change Difficulty | Recommendation | Notes |
|-----------|------------------|----------------|-------|
| **Microcontroller** | ❌ Very Hard | Keep STM32F4 | Proven design |
| **Gate Driver** | ⚠️ Moderate | Keep DRV series | Or plan 1 week development |
| **Pin Mapping** | ✅ Easy | Change freely | Well-supported |
| **Shunt Resistors** | ✅ Very Easy | Optimize as needed | Config change only |
| **MOSFETs** | ✅ Easy | Select for performance | Minimal firmware impact |
| **Encoder** | ✅ Easy | Choose from supported | 10+ types available |
| **IMU** | ✅ Easy | Choose from supported | 4 types available |

---

### Key Takeaways

1. **STM32F4 is the sweet spot**
   - Don't change unless absolutely necessary
   - Any other architecture = months of work

2. **Pin mapping is flexible**
   - Change pins freely for PCB optimization
   - Validate with datasheet
   - 1-2 days effort

3. **Gate drivers: Use TI DRV if possible**
   - If must change: 2-5 days development
   - Prefer integrated solutions
   - Test thoroughly

4. **Other components (shunts, MOSFETs, sensors)**
   - Usually easy to modify
   - Well-abstracted in firmware
   - Low risk

5. **Always consider:**
   - Development time vs savings
   - Risk of bugs/failures
   - Community support impact
   - Testing requirements

---

### Final Recommendation

**For 90% of designs:**
- ✅ STM32F405 or F407
- ✅ TI DRV8301 or DRV8305
- ✅ Custom pin mapping for your PCB
- ✅ Standard components (shunts, MOSFETs)

**Only deviate if:**
- Strong business case (cost at volume)
- Team has expertise in new components
- Willing to invest development time
- Understand firmware implications

**Questions to ask before changing:**
1. What is the cost savings per unit?
2. What is our production volume?
3. Do we have firmware expertise?
4. What is our time-to-market deadline?
5. What is the risk if we have bugs?

**Use this guide during schematic review** to catch issues before PCB manufacturing!
