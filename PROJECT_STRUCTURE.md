# VESC BLDC Firmware - Project Structure Documentation

## Table of Contents

1. [Project Overview](#project-overview)
2. [Repository Statistics](#repository-statistics)
3. [Root Directory Files](#root-directory-files)
4. [Major Directories](#major-directories)
5. [Firmware Architecture](#firmware-architecture)
6. [Data Flow and Control](#data-flow-and-control)
7. [Supported Features](#supported-features)
8. [Build System](#build-system)
9. [External Dependencies](#external-dependencies)

---

## Project Overview

**VESC BLDC Firmware** is an open-source motor controller firmware for DC/BLDC (Brushless DC)/FOC (Field-Oriented Control) controllers. This is a fork of the Vedder VESC BLDC code repository.

- **Official Website:** [https://vesc-project.com/](https://vesc-project.com/)
- **Current Version:** 7.00
- **License:** GNU General Public License v3.0
- **RTOS:** ChibiOS 3.0.5
- **Target Platform:** STM32F4 microcontrollers

---

## Repository Statistics

- **Total C/H Files:** ~173 files
- **Root Level Code:** ~9,900 lines
- **Supported Hardware Configs:** 82+ subdirectories, 289+ header files
- **Largest File:** `motor/mcpwm_foc.c` (172KB - FOC algorithm implementation)

---

## Root Directory Files

### Configuration Files

| File | Lines | Description |
|------|-------|-------------|
| `conf_general.h` | 1,457 | Main firmware configuration with version info and hardware selection macros |
| `conf_general.c` | 2,283 | Configuration implementation and management functions |
| `conf_custom.h/.c` | - | Custom configuration overrides for user-specific builds |
| `datatypes.h` | 1,457 | Core data structures: motor states, control modes, sensor modes, FOC types |
| `firmware_metadata.h` | - | Firmware version and metadata definitions |

### Core Application Files

| File | Lines | Description |
|------|-------|-------------|
| `main.c` | 406 | Firmware entry point - initializes all subsystems and starts RTOS |
| `main.h` | - | Main function declarations |

### Hardware Configuration

| File | Lines | Description |
|------|-------|-------------|
| `halconf.h` | 341 | ChibiOS HAL (Hardware Abstraction Layer) configuration |
| `chconf.h` | 509 | ChibiOS kernel configuration (threads, mutexes, timing) |
| `isr_vector_table.h` | - | Interrupt service routine vector table definitions |
| `irq_handlers.c` | - | Interrupt handler implementations for peripherals |

### Communication & Terminal

| File | Lines | Description |
|------|-------|-------------|
| `terminal.c/.h` | 1,380 | Terminal command processing interface for debugging |
| `events.c/.h` | - | Event system for firmware lifecycle management |

### Memory & Flash

| File | Lines | Description |
|------|-------|-------------|
| `flash_helper.c/.h` | 534 | Flash memory read/write operations for configuration storage |
| `ld_eeprom_emu.ld` | - | Linker script for EEPROM emulation in flash |

### Build & Utilities

| File | Description |
|------|-------------|
| `confgenerator.c` (1,008 lines) | Configuration code generator utility |
| `bms.c/.h` (697 lines) | Battery Management System interface |
| `timeout.c/.h` | Timeout management for safety features |
| `package_firmware.py` | Python script for firmware packaging |
| `Makefile` | Main build system configuration |
| `README.md` | Project documentation and build instructions |
| `CHANGELOG.md` | Version history and release notes |
| `CONTRIBUTING` | Contribution guidelines for developers |
| `.gdbinit` | GDB debugger initialization script |
| `.travis.yml` | Travis CI continuous integration configuration |
| `pi_stm32.cfg` | OpenOCD programming configuration for Raspberry Pi |
| `stm32-bv_openocd.cfg` | Alternative OpenOCD configuration |

---

## Major Directories

### 1. `/comm/` - Communication Module

**Size:** 14 files, 173KB
**Purpose:** Handles all communication protocols and packet processing

| File | Size | Description |
|------|------|-------------|
| `comm_can.c/.h` | 60KB | CAN bus communication protocol implementation |
| `comm_usb.c/.h` | - | USB communication interface |
| `comm_usb_serial.c/.h` | 12KB | USB serial communication driver |
| `commands.c/.h` | - | Command packet handling and processing |
| `packet.c/.h` | - | Low-level packet encoding/decoding with CRC |
| `log.c/.h` | - | Logging functionality for debugging |
| `comm.mk` | - | Build configuration for communication module |

**Protocols Supported:** USB, CAN, NRF24L01+ wireless, LoRa wireless

---

### 2. `/motor/` - Motor Control Core

**Size:** 12 files, 421KB
**Purpose:** The heart of BLDC/FOC motor control algorithms

| File | Size | Description |
|------|------|-------------|
| `mc_interface.c/.h` | 81KB | Main motor control interface - abstracts hardware and provides unified API |
| `mcpwm.c/.h` | 81KB | PWM control for basic BLDC (trapezoidal commutation mode) |
| `mcpwm_foc.c/.h` | 172KB | Field-Oriented Control (FOC) implementation - Clarke/Park transforms |
| `foc_math.c/.h` | 25KB | FOC mathematical functions (trig, transforms) |
| `mcconf_default.h` | 24KB | Default motor configuration parameters |
| `virtual_motor.c/.h` | 14KB | Virtual motor simulation for testing without hardware |
| `motor.mk` | - | Build configuration for motor control |

**Key Features:**
- PWM generation for 3-phase inverters
- Current control loops
- Speed (RPM) control
- Position tracking
- Sensorless and sensored modes

---

### 3. `/encoder/` - Encoder Support

**Size:** 25 files, 141KB
**Purpose:** Abstract interface for multiple encoder types

#### Supported Encoder Types

| Encoder Type | Files | Description |
|--------------|-------|-------------|
| ABI | `enc_abi.c/.h` | Incremental quadrature encoder (A/B/Index) |
| AD2S1205 | `enc_ad2s1205.c/.h` | Resolver-to-digital converter |
| AS504x | `enc_as504x.c/.h` | AS5047/AS5048 absolute magnetic encoders |
| AS5x47U | `enc_as5x47u.c/.h` (14KB) | AS5147U/AS5247U series encoders |
| BiSSC | `enc_bissc.c/.h` | BiSS-C protocol encoders |
| MT6816 | `enc_mt6816.c/.h` | MT6816 magnetic encoder |
| PWM | `enc_pwm.c/.h` | PWM-based encoder input |
| Sin/Cos | `enc_sincos.c/.h` | Sine/Cosine analog encoder |
| TLE5012 | `enc_tle5012.c/.h` (24KB) | TLE5012 GMR sensor |
| TS5700N8501 | `enc_ts5700n8501.c/.h` | TS5700N8501 encoder |

#### Core Encoder Files

| File | Size | Description |
|------|------|-------------|
| `encoder.c/.h` | 30KB | Main encoder interface and abstraction layer |
| `encoder_cfg.c/.h` | - | Encoder configuration management |
| `encoder_datatype.h` | - | Encoder data structures and enumerations |

---

### 4. `/applications/` - Application Layer

**Size:** 14 files, 128KB
**Purpose:** Different control application modes for various input types

#### Core Application Framework

| File | Description |
|------|-------------|
| `app.c/.h` | Application framework and dispatcher |
| `appconf_default.h` | Default application configuration |

#### Control Applications

| Application | File | Size | Description |
|-------------|------|------|-------------|
| ADC Input | `app_adc.c` | 18KB | Analog input control (throttle, brake) |
| PPM Input | `app_ppm.c` | 17KB | PWM input control (RC receiver) |
| Nunchuk | `app_nunchuk.c` | 15KB | Nintendo Nunchuk controller via I2C |
| UART Comm | `app_uartcomm.c` | 8.8KB | UART communication interface |
| PAS | `app_pas.c` | 9.1KB | Pedal Assist Sensor for e-bikes |
| DPV | `app_dpv.c` | 3.7KB | Diver Propulsion Vehicle mode |
| Sten | `app_sten.c` | 5.2KB | Sten motor controller mode |
| Skypuff | `app_skypuff.c` | 18KB | Custom skypuff control mode |
| Custom | `app_custom.c` | - | Custom application template |

#### Specialized Applications

**`applications/er/`** - EroCkit application (motorcycle control)
- `app_erockit_v2.c` - EroCkit v2 implementation
- `ErConfig.qml` - QML UI configuration

**`applications/finn/`** - Finn application
- `app_finn_az.c` - Finn azimuth controller

---

### 5. `/hwconf/` - Hardware Configurations

**Size:** 82 subdirectories, 289+ files
**Purpose:** Hardware-specific configurations for different VESC boards

#### Common Files Per Hardware

Each hardware configuration typically includes:
- `hw_<name>.h` - Main hardware header with pin definitions
- `hw_<name>_core.h` - Core hardware configuration
- `hw_<name>_core.c` - Hardware initialization code
- `hw_<name>_no_limits.h` - Version without hardware limits

#### Supported Hardware Manufacturers

1. **VESC Official** (`vesc/`) - Duet, Pronto, Maximp, Basic
2. **Trampa** (`trampa/`) - 100_250, 75_300_MKIV, 60_alva, RB, HD series
3. **Flipsky** (`flipsky/`, `flipsky_official/`)
4. **MakerX** (`makerx/`)
5. **Makerbase** (`makerbase/`)
6. **Luna** (`luna/`)
7. **Stormcore** (`stormcore/`)
8. **ENNOID** (`ENNOID/`)
9. **Repas** (`repas/`)
10. **Shaman** (`shaman/`)
11. **Floatwheel** (`floatwheel/`)
12. **Fungineers** (`fungineers/`)
13. **Tronic** (`tronic/`)
14. **TeamTriforceUK** (`teamtriforceuk/`)
15. **IPM** (`ipm/`)
16. **ITR** (`itr/`)
17. **JetFleet** (`JetFleet/`)
18. **Ubox** (`Ubox/`)
19. **Other** (`other/`)
20. **Example** (`example/`) - Template for custom hardware

#### Common Hardware Files

| File | Description |
|------|-------------|
| `board.c/.h` | Generic board configuration |
| `hw.c/.h` | Hardware abstraction layer main interface |
| `hwconf.mk` | Hardware configuration build system |
| `drv8301.c/.h` | DRV8301 gate driver |
| `drv8305.c/.h` | DRV8305 gate driver |
| `drv8316.c/.h` | DRV8316 gate driver |
| `drv8320s.c/.h` | DRV8320S gate driver |
| `drv8323s.c/.h` | DRV8323S gate driver |

---

### 6. `/driver/` - Hardware Drivers

**Size:** 16 files
**Purpose:** Low-level hardware interface drivers

#### Main Drivers

| Driver | Files | Description |
|--------|-------|-------------|
| EEPROM | `eeprom.c/.h` (20KB) | EEPROM read/write operations |
| I2C Bit-Bang | `i2c_bb.c/.h` | Software I2C implementation |
| SPI Bit-Bang | `spi_bb.c/.h` | Software SPI implementation |
| Timer | `timer.c/.h` | Timer utilities |
| LED PWM | `ledpwm.c/.h` | LED PWM control |
| Servo PWM | `pwm_servo.c/.h` | Servo PWM output |
| Servo Decoder | `servo_dec.c/.h` | Servo signal decoder |

#### Wireless Drivers

**`driver/nrf/`** (8 files) - NRF24L01+ wireless driver
- `nrf_driver.c/.h` - Main NRF driver
- `rf.c/.h` - RF communication layer
- `rfhelp.c/.h` - RF helper functions
- `spi_sw.c/.h` - Software SPI for NRF

**`driver/lora/`** (6 files) - LoRa wireless driver
- `SX1278.c/.h` (14KB) - SX1278 LoRa module driver
- `SX1278_hw.c/.h` - Hardware-specific LoRa interface
- `lora.c/.h` - LoRa abstraction layer

---

### 7. `/imu/` - IMU/Sensor Support

**Size:** 17 files, 169KB
**Purpose:** Inertial Measurement Unit support for orientation tracking

#### Sensor Drivers

| Sensor | Files | Description |
|--------|-------|-------------|
| IMU Core | `imu.c/.h` (22KB) | Main IMU interface and management |
| AHRS | `ahrs.c/.h` (9.6KB) | Attitude and Heading Reference System |
| MPU9150 | `mpu9150.c/.h` (13KB) | MPU9150 9-axis sensor |
| ICM20948 | `icm20948.c/.h` (5.4KB) | ICM20948 9-axis sensor |
| BMI160 | `bmi160_wrapper.c/.h` (5.3KB) | BMI160 sensor wrapper |
| LSM6DS3 | `lsm6ds3.c/.h` (9KB) | LSM6DS3 6-axis sensor |

#### Sub-modules

- **`imu/BMI160_driver/`** - Complete BMI160 sensor driver implementation
- **`imu/Fusion/`** - Sensor fusion algorithm library for orientation estimation

---

### 8. `/util/` - Utility Functions

**Size:** 12 files, 67KB
**Purpose:** Core utility and math libraries

| File | Size | Description |
|------|------|-------------|
| `utils_math.c/.h` | 17KB | Mathematical functions (angle handling, filtering, normalization) |
| `utils_sys.c/.h` | 4.1KB | System utilities |
| `buffer.c/.h` | 7.3KB | Buffer management and serialization |
| `digital_filter.c/.h` | 6KB | Digital signal filtering (Biquad, etc.) |
| `crc.c/.h` | 4.2KB | CRC checksum calculation |
| `mempools.c/.h` | 3.4KB | Memory pool allocation for real-time safety |
| `worker.c/.h` | - | Worker thread utilities |

#### Sub-modules

- **`util/lzo/`** - LZO compression library for data compression

---

### 9. `/lispBM/` - Lisp-based Scripting

**Size:** 465KB
**Purpose:** Embedded Lisp interpreter for user-defined control logic

#### VESC-specific Lisp Files

| File | Size | Description |
|------|------|-------------|
| `lispif.c/.h` | 26KB | Lisp to firmware interface bridge |
| `lispif_c_lib.c` | 40KB | C library bindings for Lisp |
| `lispif_vesc_extensions.c` | 193KB | VESC-specific Lisp extensions (motor control, CAN, etc.) |
| `lbm_vesc_utils.c/.h` | - | VESC utilities for Lisp |
| `lispbm.mk` | - | Build configuration |

#### Sub-modules

- **`lispBM/lispBM/`** - Full LispBM interpreter implementation
  - Complete Lisp compiler/interpreter
  - Documentation and examples
  - REPL (Read-Eval-Print Loop)
  - Unit tests

- **`lispBM/c_libs/`** - C library bindings and examples

**Use Cases:**
- Custom control algorithms without firmware recompilation
- Dynamic behavior modification
- Advanced telemetry processing
- Custom CAN protocols

---

### 10. `/libcanard/` - UAVCAN Protocol

**Size:** 8 files, 142KB
**Purpose:** UAVCAN distributed communication protocol

| File | Size | Description |
|------|------|-------------|
| `canard.c/.h` | 53KB + 27KB | UAVCAN protocol stack implementation |
| `canard_driver.c/.h` | 46KB | CAN driver integration for UAVCAN |
| `canard_internals.h` | - | Internal protocol structures |
| `canard.mk` | - | Build configuration |

#### Sub-modules

- **`libcanard/dsdl/`** - Data Structure Definition Language files for UAVCAN

**Purpose:** Enables distributed motor control systems using UAVCAN standard protocol

---

### 11. `/blackmagic/` - Debug Probe Support

**Size:** 17 files, 49KB
**Purpose:** Black Magic Probe debugger integration

| File | Size | Description |
|------|------|-------------|
| `bm_if.c/.h` | 16KB | Black Magic interface |
| `target.h` | - | Target device definitions |
| `platform.c/.h` | - | Platform abstraction |
| `swdptap.c/.h` | - | SWD (Serial Wire Debug) protocol |
| `exception.c/.h` | - | Exception handling |
| `timing.c/.h` | - | Timing utilities |

#### Sub-modules

- **`blackmagic/target/`** - Target-specific implementations for various processors

**Purpose:** Hardware-in-the-loop debugging and flash programming during development

---

### 12. `/qmlui/` - QML User Interface

**Size:** 4 files, 10KB
**Purpose:** Custom UI definitions for VESC Tool

| Directory | Description |
|-----------|-------------|
| `qmlui/app/` | Application-specific UI definitions |
| `qmlui/hw/` | Hardware-specific UI definitions |

**Purpose:** Allows custom hardware/application UIs in VESC Tool via QML (Qt Modeling Language)

---

### 13. `/ChibiOS_3.0.5/` - RTOS Kernel

**Size:** 58KB
**Purpose:** ChibiOS real-time operating system

#### Contents

- **`os/`** - ChibiOS kernel implementation
  - Thread management
  - Synchronization primitives
  - Hardware Abstraction Layer (HAL)
  - Board support packages

- **`ext/`** - External libraries
  - CMSIS (Cortex Microcontroller Software Interface Standard)
  - STM32 peripheral libraries

**Features:**
- Preemptive multitasking
- Thread priorities
- Mutexes and semaphores
- Event-driven programming
- Hardware timers
- ADC, SPI, I2C, UART drivers

---

### 14. `/make/` - Build System

**Size:** 8 files
**Purpose:** Makefile infrastructure for multi-platform builds

| File | Description |
|------|-------------|
| `fw.mk` | Firmware build rules and compiler flags |
| `system-id.mk` | Operating system detection |
| `tools.mk` | Tool setup and installation |
| `unittest.mk` | Unit test build rules |
| `unittest-defs.mk` | Unit test definitions |
| `linux.mk` | Linux-specific build rules |
| `macos.mk` | macOS-specific build rules |
| `windows.mk` | Windows-specific build rules |

**Supported Platforms:** Linux, macOS, Windows

---

### 15. `/documentation/` - Project Documentation

| File | Size | Description |
|------|------|-------------|
| `comm_can.md` | 11KB | CAN communication protocol documentation |

---

### 16. `/Project/` - IDE Configuration

#### Subdirectories

**`Project/Qt Creator/`** - Qt Creator IDE project files
- `vesc.pro` - Main Qt project file for graphical development

**`Project/scripts/`** - Build and helper scripts

---

### 17. `/tests/` - Unit Tests

**Purpose:** Automated testing of critical firmware functions

#### Test Suites

| Directory | Description |
|-----------|-------------|
| `tests/angles/` | Angle calculation and normalization tests |
| `tests/utils_math/` | Math utilities unit tests |
| `tests/float_serialization/` | Float data serialization tests |
| `tests/packet_recovery/` | Packet recovery and error handling tests |
| `tests/overvoltage_fault/` | Overvoltage protection tests |

---

## Firmware Architecture

### Layered Architecture

```
┌─────────────────────────────────────────────┐
│      Applications (app_*.c)                 │ ← Input handling
│  ADC, PPM, Nunchuk, UART, PAS, etc.        │
├─────────────────────────────────────────────┤
│   Motor Control Interface (mc_interface)    │ ← Master API
│  Unified control API for all motor modes    │
├─────────────────────────────────────────────┤
│  FOC/PWM Control (mcpwm_foc.c, mcpwm.c)    │ ← Core algorithms
│  Clarke/Park transforms, current loops      │
├─────────────────────────────────────────────┤
│  Encoders & Sensors (encoder/*, imu/*)     │ ← Feedback
│  Position, orientation, speed sensing       │
├─────────────────────────────────────────────┤
│  Communication (CAN, USB, NRF, LoRa)       │ ← External I/F
│  Commands, telemetry, configuration         │
├─────────────────────────────────────────────┤
│  Drivers (PWM, I2C, SPI, ADC, Timer)       │ ← Hardware I/O
│  Low-level peripheral access                │
├─────────────────────────────────────────────┤
│  ChibiOS RTOS & HAL                         │ ← OS & HAL
│  Threading, synchronization, drivers        │
├─────────────────────────────────────────────┤
│  STM32F4 Hardware                           │ ← MCU
│  ARM Cortex-M4, peripherals, GPIO          │
└─────────────────────────────────────────────┘
```

---

## Data Flow and Control

### 1. Input Sources

- **Applications:** ADC throttle, PPM receiver, Nunchuk controller, UART commands
- **Terminal:** Debug commands via USB/UART
- **CAN Messages:** Remote control from other devices
- **Lisp Scripts:** User-defined custom control logic

### 2. Processing Pipeline

```
Input → Application Layer → Motor Control Interface → FOC/PWM Engine
                                      ↑
                                      |
                              Encoder Feedback (Closed-loop)
```

### 3. Output

- **PWM Signals:** Gate drivers → Power MOSFETs → Motor phases
- **Telemetry:** Status data → CAN/USB → VESC Tool
- **Logging:** Debug data → Terminal/USB

---

## Supported Features

### Motor Types
- **BLDC** (Brushless DC) - Trapezoidal commutation
- **DC** - Brushed DC motors
- **FOC** (Field-Oriented Control) - Sinusoidal commutation

### Encoder Types (10+ supported)
- Incremental (ABI)
- Absolute (AS5047, AS5048, MT6816, etc.)
- Resolver (AD2S1205)
- SPI encoders
- Sine/Cosine analog
- PWM-based

### Sensor Modes
- **Sensorless:** Back-EMF detection, HFI (High-Frequency Injection)
- **Sensored:** Hall sensors, encoders
- **Hybrid:** Sensorless startup + encoder

### FOC Modes
- Voltage mode
- Current mode
- Multiple HFI variants for sensorless operation
- Encoder feedback for precision

### Control Modes
- **Speed Control:** Target RPM with PID
- **Current Control:** Torque control via current
- **Duty Cycle Control:** Direct PWM duty
- **Position Control:** Servo-like positioning

### Wireless Communication
- **NRF24L01+:** 2.4GHz wireless
- **LoRa:** Long-range wireless
- **UAVCAN/CAN:** Distributed multi-motor systems

### IMU Support
- MPU9150, ICM20948, BMI160, LSM6DS3
- Sensor fusion with AHRS
- 9-axis orientation tracking

### Input Types
- **ADC:** 0-10V analog input
- **PWM/PPM:** RC receiver input
- **UART:** Serial commands
- **I2C:** Nunchuk controller
- **CAN:** Multi-device communication

### Advanced Features
- **Lisp Scripting:** Full Lisp interpreter with VESC extensions
- **Multi-motor:** Dual-motor control on capable hardware
- **BMS Integration:** Battery management system interface
- **Virtual Motor:** Simulation mode for testing
- **QML UI:** Custom user interfaces in VESC Tool

---

## Build System

### Build Commands

```bash
# List all available targets
make

# Build for specific hardware
make <board_name>

# Build and flash
make <board_name>_flash

# Clean build
make clean
```

### Compiler Configuration

- **Optimization:** `-O2` (balanced speed/size)
- **Float Precision:** Single-precision (`-fsingle-precision-constant`)
- **Instruction Set:** Thumb mode (ARM Cortex-M4)
- **LTO:** Link-Time Optimization (optional)
- **Dead Code Elimination:** Enabled

### Supported Boards

55+ board variants across manufacturers (see `/hwconf/` directory)

---

## External Dependencies

| Library | Version | Purpose |
|---------|---------|---------|
| **ChibiOS** | 3.0.5 | Real-time operating system and HAL |
| **LispBM** | Latest | Embedded Lisp interpreter |
| **libcanard** | - | UAVCAN protocol stack |
| **BlackMagic** | - | Debug probe support |
| **STM32 CMSIS** | - | ARM Cortex-M processor support |
| **LZO** | - | Lossless data compression |

---

## Documentation Resources

### Built-in Documentation

- `README.md` - Setup and build instructions
- `CONTRIBUTING` - Development guidelines
- `CHANGELOG.md` - Version history (30KB)
- `documentation/comm_can.md` - CAN protocol reference
- `hwconf/example/README.md` - Hardware configuration guide
- `lispBM/README.md` - LispBM documentation

### Code Documentation

- Doxygen-style function comments throughout codebase
- Configuration header comments explaining parameters
- Hardware configuration examples and templates

---

## Key Technical Insights

1. **Modularity:** Well-organized into functional modules (motor, comm, encoder, etc.)
2. **Hardware Abstraction:** 82+ hardware configs using same core firmware
3. **Extensibility:** Pluggable applications, Lisp scripts, custom hardware configs
4. **Real-time Performance:** ChibiOS RTOS provides deterministic motor control timing
5. **Multi-protocol:** Supports CAN, USB, NRF, LoRa for diverse applications
6. **Production Quality:** Comprehensive unit tests, fault handling, safety features
7. **Open Source:** GPL v3 license ensures community contributions and transparency

---

## Getting Started

### Prerequisites

- ARM GCC toolchain
- Make
- OpenOCD (for flashing)
- Python 3 (for build scripts)

### Quick Build

```bash
# Clone repository (if not already done)
git clone <repository-url>
cd vesc_bldc

# Build for a specific hardware (example)
make 100_250

# Flash to hardware
make 100_250_flash
```

### Hardware Configuration

1. Choose existing hardware from `/hwconf/` or create custom configuration
2. Copy `/hwconf/example/` template for new hardware
3. Modify pin definitions and parameters
4. Add to build system in `/hwconf/hwconf.mk`

---

**Last Updated:** 2025-11-07
**Firmware Version:** 7.00
**Documentation Version:** 1.0
