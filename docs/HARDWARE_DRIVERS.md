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

---

## Servo Decoder

### Overview

The servo decoder (`servo_dec.c/h`) captures and decodes standard RC servo PWM signals (1-2ms pulses). This allows using RC receivers as input devices for motor control.

**Location:** `driver/servo_dec.c`, `driver/servo_dec.h`

### RC Servo PWM Protocol

#### Theory of Operation

Standard RC servo signals:
- **Frame Rate:** 50Hz (20ms period) typical, can vary 40-200Hz
- **Pulse Width:** 1000µs (min) to 2000µs (max), 1500µs (center)
- **Signal:** Positive-going pulses on idle-low signal
- **Channels:** Typically 1-8 channels, each on separate wire

**Timing Diagram:**
```
           ┌──────┐              ┌──────┐
Signal     │      │              │      │
       ────┘      └──────────────┘      └─────
           
           ├──────┤
           1000µs  = Full reverse/minimum
           
           ├────────────┤
              1500µs    = Center/neutral
           
           ├──────────────────┤
                  2000µs      = Full forward/maximum
           
           ├──────────────────────────────┤
                    20ms                  = Frame period
```

**Decoded Range:**
- **-1.0:** 1000µs (full reverse)
- **0.0:** 1500µs (center/neutral)
- **+1.0:** 2000µs (full forward)

### API Reference

#### Initialization

```c
void servodec_init(void (*d_func)(void));
```

Initializes servo decoder:
- Configures input capture timer
- Sets up interrupt handler
- Starts pulse measurement

**Parameters:**
- `d_func` - Callback function called on each pulse reception (can be NULL)

**Example:**
```c
void pulse_received_callback(void) {
    // Called when new pulse detected
    // Can be used for timeout reset or LED blink
}

servodec_init(pulse_received_callback);
```

#### Stop

```c
void servodec_stop(void);
```

Stops servo decoder and releases timer resources.

#### Configuration

```c
void servodec_set_pulse_options(float start, float end, bool median_filter);
```

Configures pulse width mapping and filtering.

**Parameters:**
- `start` - Pulse width (ms) for -1.0 output (typically 1.0)
- `end` - Pulse width (ms) for +1.0 output (typically 2.0)
- `median_filter` - Enable 3-sample median filter

**Example:**
```c
// Standard servo range: 1-2ms
servodec_set_pulse_options(1.0f, 2.0f, true);

// Extended range: 0.9-2.1ms
servodec_set_pulse_options(0.9f, 2.1f, true);

// Reversed direction
servodec_set_pulse_options(2.0f, 1.0f, false);
```

#### Reading Decoded Value

```c
float servodec_get_servo(int servo_num);
```

Returns decoded servo position.

**Parameters:**
- `servo_num` - Channel number (0-7)

**Returns:** Decoded value (-1.0 to +1.0), or 0.0 if no signal

**Example:**
```c
float throttle = servodec_get_servo(0);  // Read channel 0
float steering = servodec_get_servo(1);  // Read channel 1

// Apply to motor control
mc_interface_set_duty(throttle);
```

#### Pulse Timing

```c
float servodec_get_last_pulse_len(int servo_num);
```

Returns raw pulse length in milliseconds.

**Returns:** Pulse width in ms (typically 1.0-2.0)

```c
uint32_t servodec_get_time_since_update(void);
```

Time since last pulse received (for timeout detection).

**Returns:** Milliseconds since last pulse

**Example:**
```c
if (servodec_get_time_since_update() > 500) {
    // No signal for 500ms, stop motor
    mc_interface_release_motor();
}
```

#### Status Check

```c
bool servodec_is_running(void);
```

**Returns:** `true` if decoder is initialized and running

### Practical Application

#### RC Receiver Control

```c
#include "servo_dec.h"
#include "mc_interface.h"
#include "app.h"

#define SERVO_TIMEOUT_MS  500

void rc_control_init(void) {
    // Initialize servo decoder
    servodec_init(NULL);

    // Configure for standard servo range with filtering
    servodec_set_pulse_options(1.0f, 2.0f, true);
}

void rc_control_update(void) {
    // Check for signal timeout
    if (servodec_get_time_since_update() > SERVO_TIMEOUT_MS) {
        // No RC signal, stop motor
        mc_interface_release_motor();
        return;
    }

    // Read throttle channel (channel 0)
    float throttle = servodec_get_servo(0);

    // Apply deadband (neutral zone)
    if (fabsf(throttle) < 0.05f) {
        throttle = 0.0f;
    }

    // Set motor duty cycle
    mc_interface_set_duty(throttle);
}

// Call from periodic thread (e.g., 100Hz)
void rc_control_thread(void *arg) {
    while (1) {
        rc_control_update();
        chThdSleepMilliseconds(10);
    }
}
```

#### Multi-Channel RC Input

```c
void multi_channel_rc_control(void) {
    // Channel assignments
    float ch0 = servodec_get_servo(0);  // Throttle
    float ch1 = servodec_get_servo(1);  // Steering
    float ch2 = servodec_get_servo(2);  // Mode switch
    float ch3 = servodec_get_servo(3);  // Auxiliary

    // Check timeout on primary channel
    if (servodec_get_time_since_update() > 500) {
        mc_interface_release_motor();
        return;
    }

    // 3-position mode switch on channel 2
    int mode;
    if (ch2 < -0.5f) {
        mode = 0;  // Position 1
    } else if (ch2 > 0.5f) {
        mode = 2;  // Position 3
    } else {
        mode = 1;  // Position 2 (center)
    }

    switch (mode) {
    case 0:
        // Current control mode
        mc_interface_set_current(ch0 * 30.0f);  // ±30A
        break;

    case 1:
        // Duty cycle mode
        mc_interface_set_duty(ch0);
        break;

    case 2:
        // Speed control mode
        mc_interface_set_pid_speed(ch0 * 5000.0f);  // ±5000 ERPM
        break;
    }
}
```

### Hardware Considerations

**Input Pin Requirements:**
- Connect to timer input capture pin (defined in hardware config)
- 5V tolerant GPIO recommended (RC receivers typically output 3.3V or 5V)
- No external pull-up/down needed (configured internally)

**Supported Frequencies:**
- Standard: 50Hz (20ms period)
- High-speed: up to 200Hz (5ms period)
- Works with SBUS and other fast RC protocols after conversion

---

## PWM Servo Output

### Overview

The PWM servo output driver (`pwm_servo.c/h`) generates RC servo PWM signals (1-2ms pulses at 50Hz) for controlling external servos. This can be used for steering control, actuators, or other servo-driven mechanisms.

**Location:** `driver/pwm_servo.c`, `driver/pwm_servo.h`

### API Reference

#### Initialization

```c
uint32_t pwm_servo_init(uint32_t freq_hz, float duty);
```

Initializes PWM output in generic PWM mode.

**Parameters:**
- `freq_hz` - PWM frequency in Hz
- `duty` - Initial duty cycle (0.0-1.0)

**Returns:** Actual frequency achieved

**Example:**
```c
// 10kHz PWM at 50% duty
uint32_t actual_freq = pwm_servo_init(10000, 0.5f);
```

```c
void pwm_servo_init_servo(void);
```

Initializes in servo mode (50Hz, 1.5ms pulse).

**Example:**
```c
pwm_servo_init_servo();  // Standard RC servo mode
```

#### Control

```c
float pwm_servo_set_duty(float duty);
```

Sets PWM duty cycle in generic mode.

**Parameters:**
- `duty` - Duty cycle 0.0 (0%) to 1.0 (100%)

**Returns:** Actual duty cycle set

```c
void pwm_servo_set_servo_out(float output);
```

Sets servo position in servo mode.

**Parameters:**
- `output` - Servo position -1.0 (1ms) to +1.0 (2ms), 0.0 = center (1.5ms)

**Example:**
```c
pwm_servo_set_servo_out(-1.0f);  // Full left (1ms)
pwm_servo_set_servo_out(0.0f);   // Center (1.5ms)
pwm_servo_set_servo_out(+1.0f);  // Full right (2ms)
```

#### Stop

```c
void pwm_servo_stop(void);
```

Stops PWM generation and releases timer.

#### Status

```c
bool pwm_servo_is_running(void);
```

**Returns:** `true` if PWM output is active

### Practical Applications

#### Steering Servo Control

```c
#include "pwm_servo.h"
#include "mc_interface.h"

void steering_init(void) {
    pwm_servo_init_servo();
    pwm_servo_set_servo_out(0.0f);  // Center position
}

void update_steering(float motor_current) {
    // Derive steering angle from motor current
    // e.g., for differential steering
    float steering_angle = motor_current / 20.0f;  // ±20A = ±1.0

    // Clamp to valid range
    if (steering_angle > 1.0f) steering_angle = 1.0f;
    if (steering_angle < -1.0f) steering_angle = -1.0f;

    pwm_servo_set_servo_out(steering_angle);
}
```

#### Generic PWM Output

```c
// LED dimming with variable frequency PWM
void led_pwm_init(void) {
    pwm_servo_init(1000, 0.0f);  // 1kHz PWM, initially off
}

void set_led_brightness(float brightness) {
    pwm_servo_set_duty(brightness);  // 0.0-1.0
}

// Motor speed control with 20kHz PWM
void blower_motor_init(void) {
    pwm_servo_init(20000, 0.0f);  // 20kHz PWM
}

void set_blower_speed(float speed_percent) {
    pwm_servo_set_duty(speed_percent / 100.0f);
}
```

---

## NRF24L01+ Wireless Driver

### Overview

The NRF24L01+ driver provides 2.4GHz wireless communication using the Nordic Semiconductor NRF24L01+ transceiver chip. It enables remote control and telemetry for VESC applications.

**Location:** `driver/nrf/`

### Architecture

The NRF driver has multiple layers:

```
┌─────────────────────────────────────┐
│   nrf_driver.c/h                    │  High-level interface
│   - Pairing                         │  - Packet routing
│   - Configuration                   │  - Application integration
└────────────┬────────────────────────┘
             │
┌────────────▼────────────────────────┐
│   rfhelp.c/h                        │  Helper functions
│   - CRC handling                    │  - Data send/receive
│   - Address management              │
└────────────┬────────────────────────┘
             │
┌────────────▼────────────────────────┐
│   rf.c/h                            │  Low-level register access
│   - SPI communication               │  - NRF24 register read/write
│   - Mode control (TX/RX)            │
└────────────┬────────────────────────┘
             │
┌────────────▼────────────────────────┐
│   spi_sw.c/h                        │  Software SPI (bit-bang)
└─────────────────────────────────────┘
```

### NRF24L01+ Overview

**Features:**
- **Frequency:** 2.4-2.525 GHz (125 channels)
- **Data Rate:** 250kbps, 1Mbps, or 2Mbps
- **Range:** 10-100m (depending on power and environment)
- **Auto-Acknowledgment:** Hardware ACK with auto-retry
- **Multi-ceiver:** 6 data pipes for star topology
- **Packet Size:** 1-32 bytes
- **CRC:** Hardware 1 or 2 byte CRC

**Power Levels:**
- 0dBm (1mW)
- -6dBm
- -12dBm
- -18dBm

### Data Structures

```c
typedef struct {
    NRF_SPEED speed;         // Data rate
    NRF_POWER power;         // TX power
    NRF_CRC crc_type;        // CRC configuration
    NRF_RETR_DELAY retry_delay;
    unsigned char retries;   // Auto-retry count
    unsigned char channel;   // RF channel (0-125)
    unsigned char address[3];
    bool send_crc_ack;
} nrf_config;
```

### High-Level API

#### Initialization

```c
bool nrf_driver_init(void);
```

Initializes NRF24L01+ driver:
- Configures SPI interface
- Resets and configures NRF chip
- Sets up RX/TX modes
- Starts background thread

**Returns:** `true` on success

```c
void nrf_driver_init_ext_nrf(void);
```

Initializes external NRF module (if HW_HAS_NRF24_EXT defined).

```c
void nrf_driver_stop(void);
```

Stops NRF driver and powers down radio.

#### Pairing

```c
void nrf_driver_start_pairing(int ms);
```

Starts pairing mode for specified duration.

**Parameters:**
- `ms` - Pairing timeout in milliseconds

**Example:**
```c
// Enter pairing mode for 30 seconds
nrf_driver_start_pairing(30000);

// User presses button on remote
// Remote sends pairing request
// VESC stores remote address
```

```c
bool nrf_driver_is_pairing(void);
```

**Returns:** `true` if currently in pairing mode

#### Data Transfer

```c
void nrf_driver_send_buffer(unsigned char *data, unsigned int len);
```

Sends data packet over NRF.

**Parameters:**
- `data` - Data buffer to send
- `len` - Data length (max 32 bytes)

**Example:**
```c
// Send telemetry data
struct telemetry {
    float battery_voltage;
    float motor_current;
    int32_t erpm;
} telem;

telem.battery_voltage = mc_interface_get_input_voltage_filtered();
telem.motor_current = mc_interface_get_tot_current_filtered();
telem.erpm = mc_interface_get_rpm();

nrf_driver_send_buffer((uint8_t*)&telem, sizeof(telem));
```

```c
void nrf_driver_process_packet(unsigned char *buf, unsigned char len);
```

Processes received packet (called by driver internally).

#### Control

```c
void nrf_driver_pause(int ms);
```

Temporarily pauses NRF communication.

**Parameters:**
- `ms` - Pause duration in milliseconds

```c
bool nrf_driver_ext_nrf_running(void);
```

**Returns:** `true` if external NRF module is active

### Low-Level RF API

The `rf.c/h` module provides direct register access:

```c
void rf_init(void);                           // Initialize NRF chip
void rf_set_frequency(int freq);              // Set channel (2400 + freq MHz)
void rf_set_power(NRF_POWER power);           // Set TX power
void rf_set_speed(NRF_SPEED speed);           // Set data rate
void rf_mode_rx(void);                        // Enter RX mode
void rf_mode_tx(void);                        // Enter TX mode
void rf_power_down(void);                     // Power down radio
void rf_power_up(void);                       // Power up radio

// Data transfer
void rf_write_tx_payload(const char *data, int length);
void rf_read_rx_payload(char *data, int length);
int rf_status(void);                          // Read status register
```

### Practical Application

#### Remote Control Receiver

```c
#include "nrf_driver.h"
#include "mc_interface.h"

void nrf_remote_init(void) {
    if (!nrf_driver_init()) {
        // NRF init failed
        return;
    }

    // Check if pairing button pressed
    if (button_is_pressed()) {
        nrf_driver_start_pairing(30000);  // 30 second pairing window
    }
}

// Packet format from remote
struct remote_packet {
    uint8_t throttle;    // 0-255
    uint8_t brake;       // 0-255
    uint8_t buttons;     // Button flags
    uint16_t checksum;
};

void nrf_packet_handler(unsigned char *buf, unsigned char len) {
    if (len != sizeof(struct remote_packet)) {
        return;  // Invalid packet size
    }

    struct remote_packet *pkt = (struct remote_packet*)buf;

    // Verify checksum
    uint16_t calc_crc = crc16(buf, len - 2);
    if (calc_crc != pkt->checksum) {
        return;  // CRC error
    }

    // Convert throttle to -1.0 to +1.0
    float throttle = ((float)pkt->throttle - 127.5f) / 127.5f;
    float brake = pkt->brake / 255.0f;

    // Apply control
    if (brake > 0.1f) {
        mc_interface_set_brake_current(brake * 30.0f);
    } else {
        mc_interface_set_current(throttle * 50.0f);
    }

    // Reset timeout (signal received)
    app_disable_output(1000);  // 1 second timeout
}
```

### Configuration Example

```c
#include "rfhelp.h"

void configure_nrf(void) {
    nrf_config conf = {
        .speed = NRF_SPEED_1M,           // 1Mbps
        .power = NRF_POWER_0DBM,         // Max power
        .crc_type = NRF_CRC_2B,          // 2-byte CRC
        .retry_delay = NRF_RETR_DELAY_1000US,
        .retries = 3,
        .channel = 76,                   // 2.476 GHz
        .address = {0xC6, 0xC5, 0xC4},
        .send_crc_ack = true
    };

    rfhelp_update_conf(&conf);
    rfhelp_restart();
}
```

---

## LoRa SX1278 Driver

### Overview

The SX1278 driver provides long-range wireless communication using the Semtech SX1278 LoRa (Long Range) transceiver. LoRa offers much greater range than NRF24L01+ at the cost of lower data rates.

**Location:** `driver/lora/`

### LoRa Overview

**LoRa (Long Range) Technology:**
- **Modulation:** Chirp Spread Spectrum (CSS)
- **Frequency:** 433MHz, 868MHz, or 915MHz (regional)
- **Range:** 2-15km line-of-sight, 500m-2km urban
- **Data Rate:** 0.3-50 kbps (configurable)
- **Power:** Up to +20dBm (100mW)

**Key LoRa Parameters:**

1. **Spreading Factor (SF):** 6-12
   - Higher SF = longer range, lower data rate
   - SF7: ~5.5 kbps, shorter range
   - SF12: ~250 bps, maximum range

2. **Bandwidth:** 7.8kHz - 500kHz
   - Lower BW = better sensitivity, lower data rate

3. **Coding Rate:** 4/5, 4/6, 4/7, 4/8
   - Error correction redundancy

### Data Structures

```c
typedef struct {
    uint64_t frequency;      // Frequency in Hz
    uint8_t power;           // TX power level
    uint8_t LoRa_SF;         // Spreading factor
    uint8_t LoRa_BW;         // Bandwidth
    uint8_t LoRa_CR;         // Coding rate
    uint8_t LoRa_CRC_sum;    // CRC enable/disable
    uint8_t packetLength;    // Max packet size

    SX1278_Status_t status;  // Current status
    uint8_t rxBuffer[256];   // Receive buffer
    uint8_t readBytes;       // Bytes available
} SX1278_t;
```

### API Reference

#### Initialization

```c
void lora_init(void);
```

Initializes LoRa module (defined in `lora.c`).

```c
void SX1278_init(SX1278_t *module,
                  uint64_t frequency,
                  uint8_t power,
                  uint8_t LoRa_SF,
                  uint8_t LoRa_BW,
                  uint8_t LoRa_CR,
                  uint8_t LoRa_CRC_sum,
                  uint8_t packetLength);
```

Initializes SX1278 module with parameters.

**Parameters:**
- `frequency` - Frequency in Hz (e.g., 433000000 for 433MHz)
- `power` - Power level: `SX1278_POWER_20DBM`, `_17DBM`, `_14DBM`, `_11DBM`
- `LoRa_SF` - Spreading factor: `SX1278_LORA_SF_7` through `SX1278_LORA_SF_12`
- `LoRa_BW` - Bandwidth: `SX1278_LORA_BW_125KHZ`, `_250KHZ`, `_500KHZ`, etc.
- `LoRa_CR` - Coding rate: `SX1278_LORA_CR_4_5` through `SX1278_LORA_CR_4_8`
- `LoRa_CRC_sum` - CRC: `SX1278_LORA_CRC_EN` or `SX1278_LORA_CRC_DIS`
- `packetLength` - Max packet size (1-256 bytes)

**Example:**
```c
SX1278_t lora;

// Long range configuration
SX1278_init(&lora,
    433000000,                  // 433 MHz
    SX1278_POWER_20DBM,        // 20dBm (100mW)
    SX1278_LORA_SF_12,         // SF12 (maximum range)
    SX1278_LORA_BW_125KHZ,     // 125kHz bandwidth
    SX1278_LORA_CR_4_8,        // 4/8 coding rate
    SX1278_LORA_CRC_EN,        // Enable CRC
    64);                        // 64 byte packets
```

#### Transmit

```c
int SX1278_transmit(SX1278_t *module,
                     uint8_t *txBuf,
                     uint8_t length,
                     uint32_t timeout);
```

Transmits data packet.

**Parameters:**
- `txBuf` - Data to transmit
- `length` - Data length (up to packetLength)
- `timeout` - TX timeout in ms

**Returns:** 1 on success, 0 on timeout

**Example:**
```c
uint8_t data[] = "Hello LoRa!";
int result = SX1278_transmit(&lora, data, strlen((char*)data), 1000);
if (result == 1) {
    // Transmission successful
}
```

#### Receive

```c
int SX1278_receive(SX1278_t *module, uint8_t length, uint32_t timeout);
```

Enters receive mode and waits for packet.

**Parameters:**
- `length` - Expected packet length
- `timeout` - RX timeout in ms

**Returns:** 1 if packet received, 0 on timeout

```c
uint8_t SX1278_available(SX1278_t *module);
```

Returns number of bytes in receive buffer.

```c
uint8_t SX1278_read(SX1278_t *module, uint8_t *rxBuf, uint8_t length);
```

Reads received data.

**Parameters:**
- `rxBuf` - Buffer to store received data
- `length` - Maximum bytes to read

**Returns:** Number of bytes read

**Example:**
```c
// Start receiving
if (SX1278_receive(&lora, 64, 5000) == 1) {
    // Packet received within 5 seconds
    uint8_t bytes_available = SX1278_available(&lora);

    uint8_t rxbuf[64];
    uint8_t bytes_read = SX1278_read(&lora, rxbuf, sizeof(rxbuf));

    // Process received data
    process_packet(rxbuf, bytes_read);
}
```

#### Signal Strength

```c
uint8_t SX1278_RSSI_LoRa(void);
uint8_t SX1278_RSSI(void);
```

Returns RSSI (Received Signal Strength Indicator) value.

**Returns:** RSSI value (lower = stronger signal)

#### Power Management

```c
void SX1278_standby(SX1278_t *module);  // Low power standby
void SX1278_sleep(SX1278_t *module);    // Sleep mode
```

### Practical Application

#### Long-Range Telemetry

```c
#include "lora.h"

SX1278_t lora_module;

void telemetry_lora_init(void) {
    // Configure for balanced range/speed
    SX1278_init(&lora_module,
        433000000,                  // 433 MHz
        SX1278_POWER_20DBM,        // Maximum power
        SX1278_LORA_SF_10,         // SF10 (good range)
        SX1278_LORA_BW_125KHZ,     // 125kHz BW
        SX1278_LORA_CR_4_5,        // 4/5 CR (less redundancy)
        SX1278_LORA_CRC_EN,
        32);                        // 32 byte packets
}

struct telemetry_packet {
    float battery_v;
    float motor_current;
    int32_t rpm;
    int16_t temp_motor;
    int16_t temp_mosfet;
    uint32_t timestamp;
};

void send_telemetry(void) {
    struct telemetry_packet telem;

    telem.battery_v = mc_interface_get_input_voltage_filtered();
    telem.motor_current = mc_interface_get_tot_current_filtered();
    telem.rpm = mc_interface_get_rpm();
    telem.temp_motor = mc_interface_temp_motor_filtered();
    telem.temp_mosfet = mc_interface_temp_fet_filtered();
    telem.timestamp = chVTGetSystemTime();

    // Transmit (takes ~500ms at SF10, 125kHz BW)
    SX1278_transmit(&lora_module, (uint8_t*)&telem, sizeof(telem), 2000);
}

void receive_commands(void) {
    // Listen for incoming commands
    if (SX1278_receive(&lora_module, 32, 100) == 1) {
        uint8_t cmd_buf[32];
        uint8_t len = SX1278_read(&lora_module, cmd_buf, sizeof(cmd_buf));

        // Process command packet
        handle_command(cmd_buf, len);
    }
}

// Call periodically
void lora_task(void) {
    static uint32_t last_tx = 0;
    uint32_t now = chVTGetSystemTime();

    // Send telemetry every 5 seconds
    if (now - last_tx > 5000) {
        send_telemetry();
        last_tx = now;
    }

    // Check for incoming commands
    receive_commands();
}
```

### Range vs Data Rate

**Configuration Trade-offs:**

| SF | BW (kHz) | Data Rate | Range | Air Time (32 bytes) |
|----|----------|-----------|-------|---------------------|
| 7  | 125      | ~5.5 kbps | Short | ~56 ms              |
| 9  | 125      | ~1.8 kbps | Medium| ~185 ms             |
| 12 | 125      | ~250 bps  | Maximum| ~1400 ms           |
| 7  | 250      | ~11 kbps  | Short | ~28 ms              |
| 10 | 125      | ~980 bps  | Long  | ~370 ms             |

**Guidelines:**
- **Telemetry (low data rate):** SF10-12, 125kHz BW, 4/8 CR
- **Control (faster updates):** SF7-9, 250kHz BW, 4/5 CR
- **Maximum range:** SF12, 125kHz BW, 4/8 CR, high power
- **Urban environment:** Higher SF, lower BW

### Hardware Considerations

**Antenna:**
- **433MHz:** ~17cm quarter-wave
- **868MHz:** ~8.6cm quarter-wave
- **915MHz:** ~8.2cm quarter-wave
- Use proper 50Ω antenna for best performance

**Power Consumption:**
- TX at 20dBm: ~120mA
- RX mode: ~12mA
- Standby: ~1.5µA
- Sleep: ~0.2µA

**Regional Restrictions:**
- **433MHz:** ISM band (most regions)
- **868MHz:** Europe
- **915MHz:** North America, Australia
- Check local regulations for power limits and duty cycle

---

## Summary

The `/driver/` directory provides essential low-level drivers:

**Core Infrastructure:**
- EEPROM emulation enables persistent configuration storage
- Timer utilities provide precise timing for control loops
- LED PWM offers user feedback

**Communication Protocols:**
- I2C bit-bang for sensors and peripherals
- SPI bit-bang for encoders and external devices

**Signal Processing:**
- Servo decoder for RC receiver input
- PWM servo output for actuators

**Wireless:**
- NRF24L01+ for short-range, high-speed communication
- LoRa SX1278 for long-range telemetry

These drivers form the foundation for hardware interaction, enabling the VESC firmware to interface with a wide variety of peripherals and sensors.
