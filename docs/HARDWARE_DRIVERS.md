# Hardware Drivers Documentation

## Table of Contents
1. [Overview](#overview)
2. [EEPROM Emulation](#eeprom-emulation)
3. [I2C Bit-Bang Driver](#i2c-bit-bang-driver)
4. [SPI Bit-Bang Driver](#spi-bit-bang-driver)
5. [Timer Utilities](#timer-utilities)
6. [LED PWM Control](#led-pwm-control)
7. [Servo Decoder](#servo-decoder)
8. [PWM Servo Output](#pwm-servo-output)
9. [NRF24L01+ Wireless Driver](#nrf24l01-wireless-driver)
10. [LoRa SX1278 Driver](#lora-sx1278-driver)

---

## Overview

The `/driver/` directory contains low-level hardware drivers that provide abstraction for various peripherals and communication interfaces. These drivers handle direct hardware interaction, bit-banging protocols, non-volatile storage, and wireless communication.

### Driver Categories

**Core Infrastructure:**
- `eeprom.c/h` - Flash-based EEPROM emulation for configuration storage
- `timer.c/h` - High-resolution timing utilities
- `ledpwm.c/h` - LED brightness control via PWM

**Software Protocol Implementations:**
- `i2c_bb.c/h` - Software I2C (bit-bang) implementation
- `spi_bb.c/h` - Software SPI (bit-bang) with SSC support

**Signal Processing:**
- `servo_dec.c/h` - RC servo PWM signal decoder (input)
- `pwm_servo.c/h` - PWM servo signal generator (output)

**Wireless Communication:**
- `nrf/` - NRF24L01+ 2.4GHz transceiver driver (5 files)
- `lora/` - SX1278 LoRa long-range radio driver (3 files)

### File Structure
```
driver/
├── eeprom.c/h              # Flash-based EEPROM emulation
├── i2c_bb.c/h              # I2C bit-bang driver
├── spi_bb.c/h              # SPI bit-bang driver
├── timer.c/h               # Timing utilities
├── ledpwm.c/h              # LED PWM control
├── servo_dec.c/h           # Servo signal decoder
├── pwm_servo.c/h           # PWM servo output
├── nrf/                    # NRF24L01+ wireless
│   ├── nrf_driver.c/h      # High-level NRF interface
│   ├── rf.c/h              # Low-level NRF register access
│   ├── rfhelp.c/h          # NRF helper functions
│   └── spi_sw.c/h          # Software SPI for NRF
└── lora/                   # LoRa wireless
    ├── lora.c/h            # LoRa initialization
    ├── SX1278.c/h          # SX1278 chip driver
    └── SX1278_hw.c/h       # Hardware abstraction
```

---

## EEPROM Emulation

### Overview

The EEPROM emulation driver (`eeprom.c/h`) provides non-volatile storage for configuration data using STM32F4 internal flash memory. Since the STM32F4 doesn't have dedicated EEPROM, this driver emulates EEPROM behavior using two flash sectors with wear-leveling.

**Location:** `driver/eeprom.c`, `driver/eeprom.h`

### Theory of Operation

#### Flash-Based EEPROM Emulation

Traditional EEPROM allows byte-level writes and has high endurance (100k-1M cycles). Flash memory has limitations:
- **Sector Erase:** Must erase entire sectors (16KB) before writing
- **Limited Endurance:** ~10k erase cycles per sector
- **Write Granularity:** Can only write once between erases

The emulation uses a **two-page system** with wear-leveling:

1. **Page 0 & Page 1:** Two 16KB flash sectors (Sector 1 & 2)
2. **Active Page:** Contains current valid data
3. **Receive Page:** Target for data transfer during page full condition
4. **Virtual Addressing:** Maps logical addresses to physical flash

#### Page States
```
State           Value    Description
─────────────────────────────────────────────
ERASED          0xFFFF   Page is empty (all bits set)
RECEIVE_DATA    0xEEEE   Page marked to receive data
VALID_PAGE      0x0000   Page contains valid data
```

#### Data Storage Format

Each variable entry in flash:
```
┌────────────────┬────────────────┐
│ Virtual Addr   │   Data Value   │
│   (16 bits)    │   (16 bits)    │
└────────────────┴────────────────┘
```

Variables are appended sequentially. Multiple writes to same address create new entries; the **most recent** is valid.

### Memory Layout

```
Flash Memory Map:
┌─────────────────────────────────────┐
│ Sector 0 (16KB)                     │ ← Bootloader/Application
├─────────────────────────────────────┤
│ Sector 1 (16KB) - PAGE 0            │ ← 0x08004000
│  ┌──────────────────────────────┐   │
│  │ Page Status (0x0000=Valid)   │   │
│  ├──────────────────────────────┤   │
│  │ VirtAddr_0 | Data_0          │   │
│  │ VirtAddr_1 | Data_1          │   │
│  │ ...                          │   │
│  └──────────────────────────────┘   │
├─────────────────────────────────────┤
│ Sector 2 (16KB) - PAGE 1            │ ← 0x08008000
│  (Mirror layout of PAGE 0)          │
├─────────────────────────────────────┤
│ Remaining Flash...                  │
└─────────────────────────────────────┘
```

### Page Transfer Algorithm

When a page becomes full (no space for new write):

1. **Mark Receive Page:** Set second page to RECEIVE_DATA
2. **Transfer Valid Data:** Copy most recent value of each variable
3. **Validate New Page:** Mark receive page as VALID_PAGE
4. **Erase Old Page:** Erase previous valid page

This implements wear-leveling by alternating between pages.

### API Reference

#### Initialization

```c
uint16_t EE_Init(void);
```

Initializes EEPROM emulation system:
- Validates page states
- Performs recovery if corruption detected
- Formats pages if both are erased

**Returns:**
- `FLASH_COMPLETE` on success
- Flash error code on failure

**Example:**
```c
void system_init(void) {
    uint16_t status = EE_Init();
    if (status != FLASH_COMPLETE) {
        // Handle EEPROM initialization failure
        fault_handler();
    }
}
```

#### Read Variable

```c
uint16_t EE_ReadVariable(uint16_t VirtAddress, uint16_t* Data);
```

Reads variable from EEPROM.

**Parameters:**
- `VirtAddress` - Virtual address (0 to NB_OF_VAR-1)
- `Data` - Pointer to store read value

**Returns:**
- `0` if variable found
- `1` if variable not found (uninitialized)

**Example:**
```c
uint16_t saved_value;
if (EE_ReadVariable(0x0001, &saved_value) == 0) {
    // Variable exists, use saved_value
    restore_configuration(saved_value);
} else {
    // Variable not found, use default
    saved_value = DEFAULT_VALUE;
}
```

#### Write Variable

```c
uint16_t EE_WriteVariable(uint16_t VirtAddress, uint16_t Data);
```

Writes variable to EEPROM with automatic page management.

**Parameters:**
- `VirtAddress` - Virtual address (0 to NB_OF_VAR-1)
- `Data` - 16-bit value to write

**Returns:**
- `FLASH_COMPLETE` on success
- `PAGE_FULL` if page full (triggers transfer)
- Flash error code on failure

**Example:**
```c
uint16_t new_value = 1234;
uint16_t status = EE_WriteVariable(0x0001, new_value);
if (status != FLASH_COMPLETE && status != PAGE_FULL) {
    // Handle write error
    error_handler(status);
}
```

### Configuration Storage

The EEPROM is used to store:

```c
#define NB_OF_VAR  ((uint16_t)((2 * sizeof(mc_configuration) + \
                                sizeof(app_configuration) + 1) / 2) + \
                               EEPROM_VARS_HW * 2 + \
                               EEPROM_VARS_CUSTOM * 2 + \
                               (sizeof(backup_data) + 1) / 2)
```

**Stored Configurations:**
- **Motor Configuration:** FOC parameters, limits, PID gains (×2 for dual motor)
- **Application Configuration:** Control mode, PPM/ADC settings
- **Hardware Variables:** Board-specific calibration data
- **Custom Variables:** User-defined persistent data
- **Backup Data:** System state preservation

### Flash Programming Details

#### Voltage Range Adaptation

The driver adapts to supply voltage for optimal flash programming:

```c
#define VOLTAGE_RANGE  (uint8_t)((PWR->CSR & PWR_CSR_PVDO) ? \
                                  VoltageRange_2 : VoltageRange_3)
```

- **VoltageRange_3:** 2.7V-3.6V (normal operation, word programming)
- **VoltageRange_2:** 2.1V-2.7V (low voltage, different timing)

#### Sector Addresses

```c
#define EEPROM_START_ADDRESS  0x08004000  // Sector 1 start
#define PAGE0_BASE_ADDRESS    0x08004000  // Sector 1
#define PAGE1_BASE_ADDRESS    0x08008000  // Sector 2
#define PAGE_SIZE             0x4000      // 16KB
```

### Practical Usage

#### Saving Motor Configuration

```c
#include "eeprom.h"
#include "mc_interface.h"

void save_motor_config(void) {
    mc_configuration *conf = mc_interface_get_configuration();

    // EEPROM stores 16-bit values
    // Large structures are stored sequentially
    uint16_t *data_ptr = (uint16_t*)conf;
    int num_words = sizeof(mc_configuration) / 2;

    for (int i = 0; i < num_words; i++) {
        uint16_t addr = MOTOR_CONF_BASE + i;
        EE_WriteVariable(addr, data_ptr[i]);
    }
}

void load_motor_config(void) {
    mc_configuration conf;
    uint16_t *data_ptr = (uint16_t*)&conf;
    int num_words = sizeof(mc_configuration) / 2;

    for (int i = 0; i < num_words; i++) {
        uint16_t addr = MOTOR_CONF_BASE + i;
        uint16_t value;
        if (EE_ReadVariable(addr, &value) == 0) {
            data_ptr[i] = value;
        } else {
            // Use default configuration
            conf_general_get_default_motor_conf(&conf);
            break;
        }
    }

    mc_interface_set_configuration(&conf);
}
```

#### Storing Calibration Data

```c
// Store encoder zero angle
void save_encoder_offset(float offset_degrees) {
    // Convert float to two 16-bit values
    uint32_t offset_raw = *((uint32_t*)&offset_degrees);
    uint16_t low = offset_raw & 0xFFFF;
    uint16_t high = (offset_raw >> 16) & 0xFFFF;

    EE_WriteVariable(ADDR_ENCODER_OFFSET_LOW, low);
    EE_WriteVariable(ADDR_ENCODER_OFFSET_HIGH, high);
}

float load_encoder_offset(void) {
    uint16_t low, high;

    if (EE_ReadVariable(ADDR_ENCODER_OFFSET_LOW, &low) == 0 &&
        EE_ReadVariable(ADDR_ENCODER_OFFSET_HIGH, &high) == 0) {
        uint32_t offset_raw = ((uint32_t)high << 16) | low;
        return *((float*)&offset_raw);
    }

    return 0.0f;  // Default offset
}
```

### Performance Characteristics

**Write Performance:**
- **Normal Write:** ~1ms (single variable)
- **Page Transfer:** ~100ms (full page copy + erase)
- **Occurs When:** Page fills (after ~4000 writes to different addresses)

**Endurance:**
- **Sector Erase Cycles:** ~10,000 per sector
- **With Wear-Leveling:** ~20,000 configuration saves (alternating pages)
- **At 1 save/hour:** ~2.3 years continuous operation

**Capacity:**
```c
Page Size:        16KB (16,384 bytes)
Data per Entry:   4 bytes (addr + value)
Max Entries:      ~4000 per page
Variables:        Determined by NB_OF_VAR
```

### Error Handling

The driver reports various error conditions:

```c
// Flash operation errors
FLASH_ERROR_PG     // Programming error
FLASH_ERROR_WRP    // Write protection error
FLASH_ERROR_OPE    // Operation error

// EEPROM specific
PAGE_FULL          // Page needs transfer (normal operation)
NO_VALID_PAGE      // Both pages invalid (corruption)
```

**Recovery Strategy:**
```c
uint16_t status = EE_Init();
if (status == NO_VALID_PAGE) {
    // Format and restore defaults
    conf_general_store_backup_data();
    EE_Init();  // Retry
}
```

---

## I2C Bit-Bang Driver

### Overview

The I2C bit-bang driver (`i2c_bb.c/h`) implements the I2C (Inter-Integrated Circuit) protocol in software using GPIO bit manipulation. This allows I2C communication on any GPIO pins, independent of hardware I2C peripherals.

**Location:** `driver/i2c_bb.c`, `driver/i2c_bb.h`

### I2C Protocol Basics

#### Theory of Operation

I2C is a synchronous, multi-master, multi-slave, two-wire serial protocol:

**Physical Layer:**
- **SDA (Serial Data):** Bidirectional data line
- **SCL (Serial Clock):** Clock line driven by master
- **Pull-ups:** Both lines require external pull-up resistors (typically 4.7kΩ)
- **Open-Drain:** Devices pull low, pull-ups provide high

**Signal States:**
```
START Condition:  SDA falls while SCL is high
        ┌─────┐
SCL     │     └─────
      ──┐     ┌─────
SDA     └─────┘

STOP Condition:   SDA rises while SCL is high
        ┌─────────┐
SCL     │         │
      ──┘   ┌─────┘
SDA         │
```

**Data Transfer:**
- Data valid when SCL is HIGH
- Data can change when SCL is LOW
- MSB transmitted first
- 8 bits + 1 ACK/NACK bit per byte

**Acknowledge (ACK):**
- **ACK:** Receiver pulls SDA LOW during 9th clock
- **NACK:** Receiver leaves SDA HIGH (last byte or error)

### Data Structures

```c
typedef enum {
    I2C_BB_RATE_100K = 0,   // Standard mode (100 kbit/s)
    I2C_BB_RATE_200K,       // Fast mode (200 kbit/s)
    I2C_BB_RATE_400K,       // Fast mode (400 kbit/s)
    I2C_BB_RATE_700K        // Fast mode+ (700 kbit/s)
} I2C_BB_RATE;

typedef struct {
    stm32_gpio_t *sda_gpio;  // SDA GPIO port (e.g., GPIOB)
    int sda_pin;             // SDA pin number (0-15)
    stm32_gpio_t *scl_gpio;  // SCL GPIO port
    int scl_pin;             // SCL pin number
    I2C_BB_RATE rate;        // Bus speed
    bool has_started;        // Internal: START sent flag
    bool has_error;          // Internal: Error flag
    mutex_t mutex;           // Thread safety mutex
} i2c_bb_state;
```

### API Reference

#### Initialization

```c
void i2c_bb_init(i2c_bb_state *s);
```

Initializes I2C bit-bang interface:
- Configures GPIO pins as open-drain outputs
- Sets initial idle state (both lines HIGH)
- Initializes mutex for thread safety

**Example:**
```c
i2c_bb_state my_i2c = {
    .sda_gpio = GPIOB,
    .sda_pin = 7,
    .scl_gpio = GPIOB,
    .scl_pin = 6,
    .rate = I2C_BB_RATE_400K
};

i2c_bb_init(&my_i2c);
```

#### Bus Recovery

```c
void i2c_bb_restore_bus(i2c_bb_state *s);
```

Recovers I2C bus from stuck condition:
- Generates clock pulses to clear stuck SDA
- Sends STOP condition to reset bus state
- Used when slave holds SDA low

**When to Use:**
- After power-up with I2C devices
- When transaction times out
- If SDA stuck low

**Example:**
```c
// Attempt communication
if (!i2c_bb_tx_rx(&my_i2c, addr, NULL, 0, data, len)) {
    // Failed, try bus recovery
    i2c_bb_restore_bus(&my_i2c);

    // Retry communication
    i2c_bb_tx_rx(&my_i2c, addr, NULL, 0, data, len);
}
```

#### Transaction

```c
bool i2c_bb_tx_rx(i2c_bb_state *s,
                   uint16_t addr,
                   uint8_t *txbuf, size_t txbytes,
                   uint8_t *rxbuf, size_t rxbytes);
```

Performs complete I2C transaction with optional write and read.

**Parameters:**
- `s` - I2C state structure
- `addr` - 7-bit slave address (not shifted)
- `txbuf` - Transmit buffer (NULL if read-only)
- `txbytes` - Number of bytes to transmit
- `rxbuf` - Receive buffer (NULL if write-only)
- `rxbytes` - Number of bytes to receive

**Returns:** `true` on success, `false` on error (NACK or timeout)

**Example Use Cases:**

```c
// 1. Write Only (e.g., send command to device)
uint8_t cmd[] = {0x01, 0x23};
bool ok = i2c_bb_tx_rx(&i2c, 0x50, cmd, 2, NULL, 0);

// 2. Read Only (e.g., read sensor data)
uint8_t data[4];
bool ok = i2c_bb_tx_rx(&i2c, 0x50, NULL, 0, data, 4);

// 3. Write-Then-Read (e.g., read register)
uint8_t reg_addr = 0x0A;
uint8_t reg_value[2];
bool ok = i2c_bb_tx_rx(&i2c, 0x50, &reg_addr, 1, reg_value, 2);
```

#### Low-Level Byte Operations

```c
bool i2c_bb_write_byte(i2c_bb_state *s,
                        bool send_start,
                        bool send_stop,
                        unsigned char byte);
```

Writes single byte with optional START/STOP conditions.

**Parameters:**
- `send_start` - Send START condition before byte
- `send_stop` - Send STOP condition after byte
- `byte` - Data byte to send

**Returns:** `true` if ACK received, `false` if NACK

```c
unsigned char i2c_bb_read_byte(i2c_bb_state *s,
                                bool nack,
                                bool send_stop);
```

Reads single byte with ACK/NACK control.

**Parameters:**
- `nack` - Send NACK instead of ACK (true for last byte)
- `send_stop` - Send STOP condition after byte

**Returns:** Read data byte

**Advanced Example:**
```c
// Manual transaction: Write 2 bytes, read 3 bytes
i2c_bb_write_byte(&i2c, true, false, (addr << 1) | 0);  // START + addr+W
i2c_bb_write_byte(&i2c, false, false, 0x10);             // Data byte 1
i2c_bb_write_byte(&i2c, false, false, 0x20);             // Data byte 2

// Repeated START for read
i2c_bb_write_byte(&i2c, true, false, (addr << 1) | 1);  // START + addr+R
uint8_t b1 = i2c_bb_read_byte(&i2c, false, false);      // Read + ACK
uint8_t b2 = i2c_bb_read_byte(&i2c, false, false);      // Read + ACK
uint8_t b3 = i2c_bb_read_byte(&i2c, true, true);        // Read + NACK + STOP
```

### Practical Applications

#### Reading IMU Sensor (MPU6050)

```c
#include "i2c_bb.h"

#define MPU6050_ADDR       0x68
#define MPU6050_WHO_AM_I   0x75
#define MPU6050_ACCEL_XOUT 0x3B

i2c_bb_state imu_i2c;

void imu_init(void) {
    // Configure I2C pins
    imu_i2c.sda_gpio = GPIOB;
    imu_i2c.sda_pin = 7;
    imu_i2c.scl_gpio = GPIOB;
    imu_i2c.scl_pin = 6;
    imu_i2c.rate = I2C_BB_RATE_400K;

    i2c_bb_init(&imu_i2c);
    i2c_bb_restore_bus(&imu_i2c);  // Ensure clean startup

    // Verify WHO_AM_I register
    uint8_t who_am_i;
    uint8_t reg = MPU6050_WHO_AM_I;
    if (i2c_bb_tx_rx(&imu_i2c, MPU6050_ADDR, &reg, 1, &who_am_i, 1)) {
        if (who_am_i == 0x68) {
            // MPU6050 detected
        }
    }
}

void imu_read_accel(int16_t *ax, int16_t *ay, int16_t *az) {
    uint8_t reg = MPU6050_ACCEL_XOUT;
    uint8_t data[6];

    if (i2c_bb_tx_rx(&imu_i2c, MPU6050_ADDR, &reg, 1, data, 6)) {
        *ax = (data[0] << 8) | data[1];
        *ay = (data[2] << 8) | data[3];
        *az = (data[4] << 8) | data[5];
    }
}
```

#### Reading EEPROM (24C02)

```c
#define EEPROM_ADDR  0x50

// Write byte to EEPROM
void eeprom_write(uint8_t addr, uint8_t data) {
    uint8_t buf[2] = {addr, data};
    i2c_bb_tx_rx(&i2c, EEPROM_ADDR, buf, 2, NULL, 0);
    chThdSleepMilliseconds(5);  // Write cycle time
}

// Read byte from EEPROM
uint8_t eeprom_read(uint8_t addr) {
    uint8_t data;
    i2c_bb_tx_rx(&i2c, EEPROM_ADDR, &addr, 1, &data, 1);
    return data;
}
```

### Timing and Performance

**Clock Rates:**
```c
I2C_BB_RATE_100K:  ~100 kbit/s  (Standard mode)
I2C_BB_RATE_200K:  ~200 kbit/s  (Fast mode)
I2C_BB_RATE_400K:  ~400 kbit/s  (Fast mode)
I2C_BB_RATE_700K:  ~700 kbit/s  (Fast mode+)
```

**Timing Implementation:**
- Software delays calibrated for STM32F4 at 168MHz
- Actual speed depends on CPU load and interrupts
- Timing may vary ±10% under real-time OS

**Transfer Time Examples (at 400kHz):**
```
Single byte read:   ~45 µs  (addr + data + overhead)
16-byte burst:      ~400 µs
```

### Thread Safety

The driver uses a mutex for multi-threaded operation:

```c
// Multiple threads can share one I2C bus
// Thread 1
i2c_bb_tx_rx(&shared_i2c, 0x50, ...);  // Acquires mutex

// Thread 2
i2c_bb_tx_rx(&shared_i2c, 0x51, ...);  // Waits for mutex
```

### Advantages vs Hardware I2C

**Advantages of Bit-Bang:**
- Works on any GPIO pins (no hardware peripheral needed)
- Can have multiple independent I2C buses
- Full control over timing and error recovery
- No DMA/interrupt configuration needed

**Disadvantages:**
- Higher CPU usage (blocking operation)
- Less accurate timing (affected by interrupts)
- Lower maximum speed than hardware I2C

---

## SPI Bit-Bang Driver

### Overview

The SPI bit-bang driver (`spi_bb.c/h`) implements SPI (Serial Peripheral Interface) and SSC (Synchronous Serial Communication) protocols in software using GPIO manipulation.

**Location:** `driver/spi_bb.c`, `driver/spi_bb.h`

### SPI Protocol Basics

#### Theory of Operation

SPI is a synchronous, full-duplex, master-slave serial protocol:

**Physical Layer:**
- **SCK (Serial Clock):** Clock generated by master
- **MOSI (Master Out Slave In):** Data from master to slave
- **MISO (Master In Slave Out):** Data from slave to master
- **NSS/CS (Chip Select):** Slave selection (active low)

**SPI Modes:**
```
Mode  CPOL  CPHA  Description
────────────────────────────────────────────
0     0     0     Clock idle LOW, sample on rising edge
1     0     1     Clock idle LOW, sample on falling edge
2     1     0     Clock idle HIGH, sample on falling edge
3     1     1     Clock idle HIGH, sample on rising edge
```

This driver implements **Mode 0** (most common).

**Timing Diagram (Mode 0):**
```
NSS   ────┐                             ┌────
          └─────────────────────────────┘

SCK   ────┐ ┌─┐ ┌─┐ ┌─┐ ┌─┐ ┌─┐ ┌─┐ ┌─┐ ┌───
          └─┘ └─┘ └─┘ └─┘ └─┘ └─┘ └─┘ └─┘

MOSI  ────X─7─X─6─X─5─X─4─X─3─X─2─X─1─X─0─X───
          MSB                         LSB

MISO  ────X─7─X─6─X─5─X─4─X─3─X─2─X─1─X─0─X───
```

### Data Structures

```c
typedef struct {
    stm32_gpio_t *nss_gpio;   // Chip select GPIO port
    int nss_pin;              // Chip select pin number
    stm32_gpio_t *sck_gpio;   // Clock GPIO port
    int sck_pin;              // Clock pin number
    stm32_gpio_t *mosi_gpio;  // MOSI GPIO port
    int mosi_pin;             // MOSI pin number
    stm32_gpio_t *miso_gpio;  // MISO GPIO port
    int miso_pin;             // MISO pin number
    mutex_t mutex;            // Thread safety mutex
} spi_bb_state;
```

### API Reference

#### Initialization

```c
void spi_bb_init(spi_bb_state *s);
```

Initializes SPI bit-bang interface in standard SPI mode:
- Configures MOSI, SCK, NSS as outputs
- Configures MISO as input
- Sets idle state (NSS high, SCK low for mode 0)

```c
void ssc_bb_init(spi_bb_state *s);
```

Initializes in SSC mode (Synchronous Serial Communication):
- Used for protocols like TLE5012 encoder
- Different timing characteristics

**Example:**
```c
spi_bb_state encoder_spi = {
    .nss_gpio = GPIOB,
    .nss_pin = 12,
    .sck_gpio = GPIOB,
    .sck_pin = 13,
    .mosi_gpio = GPIOB,
    .mosi_pin = 15,
    .miso_gpio = GPIOB,
    .miso_pin = 14
};

spi_bb_init(&encoder_spi);
```

#### Deinitialization

```c
void spi_bb_deinit(spi_bb_state *s);
void ssc_bb_deinit(spi_bb_state *s);
```

Releases GPIO pins back to default state.

#### Data Transfer

```c
uint8_t spi_bb_exchange_8(spi_bb_state *s, uint8_t x);
```

Transfers single byte (full-duplex).

**Parameters:**
- `x` - Byte to transmit

**Returns:** Received byte

**Example:**
```c
// Send command, receive status
uint8_t status = spi_bb_exchange_8(&spi, 0x03);
```

```c
void spi_bb_transfer_8(spi_bb_state *s,
                        uint8_t *in_buf,
                        const uint8_t *out_buf,
                        int length);
```

Transfers buffer of 8-bit values.

**Parameters:**
- `in_buf` - Buffer for received data (can be NULL)
- `out_buf` - Buffer of data to transmit (can be NULL)
- `length` - Number of bytes

```c
void spi_bb_transfer_16(spi_bb_state *s,
                         uint16_t *in_buf,
                         const uint16_t *out_buf,
                         int length);
```

Transfers buffer of 16-bit values (MSB first).

```c
void ssc_bb_transfer_16(spi_bb_state *s,
                         uint16_t *in_buf,
                         const uint16_t *out_buf,
                         int length,
                         bool write);
```

SSC mode transfer with write flag control.

#### Chip Select Control

```c
void spi_bb_begin(spi_bb_state *s);  // Assert NSS (active low)
void spi_bb_end(spi_bb_state *s);    // Deassert NSS (idle high)
```

Manual chip select control for multi-byte transactions.

**Example:**
```c
spi_bb_begin(&spi);
spi_bb_exchange_8(&spi, 0x03);  // READ command
spi_bb_exchange_8(&spi, 0x00);  // Address MSB
spi_bb_exchange_8(&spi, 0x00);  // Address LSB
uint8_t data = spi_bb_exchange_8(&spi, 0x00);  // Read data
spi_bb_end(&spi);
```

#### Utility Functions

```c
void spi_bb_delay(void);        // Standard SPI delay
void spi_bb_delay_short(void);  // Shorter delay for fast operation
bool spi_bb_check_parity(uint16_t x);  // Even parity check
```

### Practical Applications

#### Reading Encoder (AS5047)

```c
#include "spi_bb.h"

#define AS5047_ANGLECOM  0x3FFF  // Angle command

spi_bb_state encoder_spi;

void encoder_init(void) {
    encoder_spi.nss_gpio = GPIOB;
    encoder_spi.nss_pin = 12;
    encoder_spi.sck_gpio = GPIOB;
    encoder_spi.sck_pin = 13;
    encoder_spi.mosi_gpio = GPIOB;
    encoder_spi.mosi_pin = 15;
    encoder_spi.miso_gpio = GPIOB;
    encoder_spi.miso_pin = 14;

    spi_bb_init(&encoder_spi);
}

uint16_t encoder_read_angle(void) {
    spi_bb_begin(&encoder_spi);

    // Send read command
    spi_bb_transfer_16(&encoder_spi, NULL, &AS5047_ANGLECOM, 1);

    spi_bb_end(&encoder_spi);

    // Small delay required by AS5047
    spi_bb_delay_short();

    spi_bb_begin(&encoder_spi);

    // Read angle data (NOP command, receive previous data)
    uint16_t angle_data;
    uint16_t nop = 0x0000;
    spi_bb_transfer_16(&encoder_spi, &angle_data, &nop, 1);

    spi_bb_end(&encoder_spi);

    // Check parity
    if (!spi_bb_check_parity(angle_data)) {
        // Parity error
        return 0xFFFF;
    }

    // Extract 14-bit angle
    return angle_data & 0x3FFF;
}
```

#### Communicating with SPI Flash

```c
#define CMD_READ_ID      0x9F
#define CMD_READ_DATA    0x03

void flash_read_id(uint8_t *manufacturer, uint8_t *device_id) {
    spi_bb_begin(&spi);
    spi_bb_exchange_8(&spi, CMD_READ_ID);
    *manufacturer = spi_bb_exchange_8(&spi, 0x00);
    *device_id = spi_bb_exchange_8(&spi, 0x00);
    spi_bb_exchange_8(&spi, 0x00);  // Capacity byte
    spi_bb_end(&spi);
}

void flash_read_data(uint32_t address, uint8_t *buffer, int length) {
    spi_bb_begin(&spi);
    spi_bb_exchange_8(&spi, CMD_READ_DATA);
    spi_bb_exchange_8(&spi, (address >> 16) & 0xFF);
    spi_bb_exchange_8(&spi, (address >> 8) & 0xFF);
    spi_bb_exchange_8(&spi, address & 0xFF);
    spi_bb_transfer_8(&spi, buffer, NULL, length);
    spi_bb_end(&spi);
}
```

### Performance

**Transfer Speed:**
- Approximately **500 kHz** clock rate at 168MHz CPU
- Limited by software delay loops
- Affected by interrupt latency

**Transfer Time Examples:**
```
Single byte:        ~16 µs
16-byte transfer:   ~256 µs
```

---

## Timer Utilities

### Overview

The timer module (`timer.c/h`) provides high-resolution timing functions using the STM32 system tick timer.

**Location:** `driver/timer.c`, `driver/timer.h`

### API Reference

```c
void timer_init(void);
```

Initializes timer subsystem (called during system startup).

```c
uint32_t timer_time_now(void);
```

Returns current timestamp in ticks.

**Returns:** 32-bit timestamp (wraps after ~49 days at 1kHz tick)

```c
float timer_seconds_elapsed_since(uint32_t time);
```

Calculates elapsed time in seconds.

**Parameters:**
- `time` - Previous timestamp from `timer_time_now()`

**Returns:** Elapsed time in seconds (float)

```c
void timer_sleep(float seconds);
```

Sleep for specified duration.

**Parameters:**
- `seconds` - Sleep duration (supports fractional seconds)

### Usage Examples

```c
// Measure execution time
uint32_t start = timer_time_now();
perform_calculation();
float duration = timer_seconds_elapsed_since(start);
// duration now contains execution time in seconds

// Periodic execution
uint32_t last_run = timer_time_now();
while (1) {
    if (timer_seconds_elapsed_since(last_run) >= 0.1f) {
        // Execute every 100ms
        periodic_task();
        last_run = timer_time_now();
    }
}

// Short delay
timer_sleep(0.001f);  // 1ms delay
```

---

## LED PWM Control

### Overview

The LED PWM module (`ledpwm.c/h`) controls LED brightness using pulse-width modulation. It supports 2-6 LEDs depending on hardware configuration.

**Location:** `driver/ledpwm.c`, `driver/ledpwm.h`

### LED Definitions

```c
#define LED_GREEN   0  // Status LED 1 (green)
#define LED_RED     1  // Status LED 2 (red)
#define LED_HW1     2  // Hardware LED 1 (optional)
#define LED_HW2     3  // Hardware LED 2 (optional)
#define LED_HW3     4  // Hardware LED 3 (optional)
#define LED_HW4     5  // Hardware LED 4 (optional)
```

Available LEDs determined by hardware configuration macros:
- `LED_PWM1_ON` through `LED_PWM4_ON` in hardware config

### API Reference

```c
void ledpwm_init(void);
```

Initializes LED PWM system:
- Configures timer for PWM generation
- Sets initial LED states (off)
- Must be called before using other LED functions

```c
void ledpwm_set_intensity(unsigned int led, float intensity);
```

Sets LED brightness.

**Parameters:**
- `led` - LED number (LED_GREEN, LED_RED, LED_HW1, etc.)
- `intensity` - Brightness 0.0 (off) to 1.0 (full brightness)

```c
void ledpwm_led_on(int led);   // Full brightness
void ledpwm_led_off(int led);  // Turn off
```

Convenience functions for on/off control.

```c
void ledpwm_update_pwm(void);
```

Updates PWM outputs (called automatically by timer ISR).

### Usage Examples

```c
#include "ledpwm.h"

void status_indicator_init(void) {
    ledpwm_init();

    // Set startup pattern
    ledpwm_led_on(LED_GREEN);
    ledpwm_led_off(LED_RED);
}

void indicate_error(void) {
    ledpwm_led_off(LED_GREEN);
    ledpwm_led_on(LED_RED);
}

void indicate_warning(void) {
    // Dim green, bright red = orange/yellow
    ledpwm_set_intensity(LED_GREEN, 0.3f);
    ledpwm_set_intensity(LED_RED, 1.0f);
}

void indicate_running(void) {
    // Breathing effect
    static float phase = 0.0f;
    phase += 0.01f;
    if (phase > 2.0f * M_PI) phase = 0.0f;

    float intensity = 0.5f + 0.5f * sinf(phase);
    ledpwm_set_intensity(LED_GREEN, intensity);
}

void battery_level_indicator(float battery_percent) {
    // LED bar graph
    int num_leds = (int)(battery_percent / 25.0f);  // 0-4 LEDs

    ledpwm_led_off(LED_HW1);
    ledpwm_led_off(LED_HW2);
    ledpwm_led_off(LED_HW3);
    ledpwm_led_off(LED_HW4);

    if (num_leds >= 1) ledpwm_led_on(LED_HW1);
    if (num_leds >= 2) ledpwm_led_on(LED_HW2);
    if (num_leds >= 3) ledpwm_led_on(LED_HW3);
    if (num_leds >= 4) ledpwm_led_on(LED_HW4);
}
```

### PWM Characteristics

**PWM Frequency:** Determined by `LEDPWM_CNT_TOP` (typically 200 → ~1kHz)
**Resolution:** 8-bit effective (0-200 steps)
**Update Rate:** Synchronous with timer ISR (1kHz typical)
