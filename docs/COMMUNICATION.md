# Communication System Documentation

## Table of Contents

1. [Introduction](#introduction)
2. [Communication Architecture](#communication-architecture)
3. [Packet Format and Encoding](#packet-format-and-encoding)
4. [USB Communication](#usb-communication)
5. [CAN Bus Communication](#can-bus-communication)
6. [Command Protocol](#command-protocol)
7. [Implementation Files](#implementation-files)
8. [Practical Examples](#practical-examples)

---

## Introduction

The `/comm/` directory implements the communication layer that allows external devices (computers, microcontrollers, other VESCs) to control and monitor the VESC. This is how VESC Tool communicates with the hardware, and how multiple VESCs coordinate in multi-motor systems.

**Key Features:**
- Multiple communication interfaces (USB, CAN, NRF, LoRa)
- Reliable packet-based protocol with CRC error detection
- Support for 100+ different commands
- CAN bus multi-device networking
- Command forwarding for multi-VESC systems
- Real-time telemetry and configuration

**Communication Interfaces:**
- **USB** - Primary interface for VESC Tool
- **CAN Bus** - Multi-device networking and control
- **NRF24L01+** - 2.4GHz wireless (optional hardware)
- **LoRa** - Long-range wireless (optional hardware)
- **UART** - Serial communication

---

## Communication Architecture

### Protocol Stack

```
┌─────────────────────────────────────────┐
│  Application Layer                      │
│  (VESC Tool, Custom Apps)               │
└───────────────┬─────────────────────────┘
                │
┌───────────────▼─────────────────────────┐
│  Command Layer (commands.c)             │
│  - Command processing                   │
│  - Parameter extraction                 │
│  - Response generation                  │
└───────────────┬─────────────────────────┘
                │
┌───────────────▼─────────────────────────┐
│  Packet Layer (packet.c)                │
│  - Packet framing                       │
│  - CRC validation                       │
│  - Byte stuffing                        │
└───────────────┬─────────────────────────┘
                │
        ┌───────┴────────┐
        │                │
┌───────▼──────┐  ┌──────▼──────┐
│  USB/UART    │  │  CAN Bus    │
│  (Serial)    │  │  (comm_can) │
└──────────────┘  └─────────────┘
```

### Communication Modes

The VESC supports 4 CAN modes (configurable in VESC Tool):

| Mode | Description |
|------|-------------|
| **VESC** | Default CAN protocol. Supports multi-VESC configurations and command forwarding |
| **UAVCAN** | DroneCAN/UAVCAN protocol for drone applications |
| **Comm Bridge** | Forwards CAN frames to commands (debugging/generic CAN interface) |
| **Unused** | CAN frames ignored (for custom Lisp scripts) |

---

## Packet Format and Encoding

### Packet Structure

All communication (USB, UART, CAN) uses a packet-based protocol with the following structure:

```
┌─────┬─────┬────────┬─────────────┬──────┬─────┐
│Start│ Len │ Len    │   Payload   │ CRC  │ End │
│Byte │ LSB │ MSB    │   (N bytes) │(2/4) │Byte │
└─────┴─────┴────────┴─────────────┴──────┴─────┘
```

**Start Byte:**
- `0x02` - Short packet (payload ≤ 256 bytes, 16-bit CRC)
- `0x03` - Long packet (payload > 256 bytes, 16-bit CRC)
- `0x04` - Long packet with 32-bit CRC

**Length:**
- 1 byte for short packets
- 2 bytes (big-endian) for long packets

**Payload:**
- First byte: Command ID (`COMM_PACKET_ID`)
- Remaining bytes: Command-specific data

**CRC:**
- 16-bit CRC for most packets
- 32-bit CRC for critical operations (firmware updates)

**End Byte:**
- `0x03` - Packet terminator

### Example Packet

Setting motor current to 10A:

```
Start: 0x02
Len:   0x05                    (5 bytes payload)
Cmd:   0x06                    (COMM_SET_CURRENT)
Data:  0x00 0x00 0x27 0x10    (10.0 * 1000 = 10000)
CRC:   0xXX 0xXX              (calculated)
End:   0x03
```

### Byte Stuffing

To prevent confusion with start/end bytes in payload:
- All instances of `0x02` and `0x03` in payload/CRC are escaped
- Escape sequence: Insert `0x00` before the byte
- Example: `0x02` → `0x00 0x02`, `0x03` → `0x00 0x03`

### CRC Calculation

**16-bit CRC (CCITT):**
- Polynomial: 0x1021
- Initial value: 0x0000
- Calculated over payload only

**Implementation:**
```c
unsigned short crc16(unsigned char *buf, unsigned int len) {
    unsigned short cksum = 0;
    for (unsigned int i = 0; i < len; i++) {
        cksum = (cksum << 8) ^ crc16_tab[((cksum >> 8) ^ *buf++) & 0xFF];
    }
    return cksum;
}
```

---

## USB Communication

### USB Serial CDC

The VESC uses USB Communication Device Class (CDC) for serial communication.

**Configuration:**
- Baud rate: Ignored (USB doesn't use baud rate)
- Data bits: 8
- Parity: None
- Stop bits: 1

**Implementation:** `comm/comm_usb_serial.c/.h`

### USB Endpoints

- **EP1 OUT**: Receive data from host
- **EP1 IN**: Transmit data to host
- **Buffer size**: 64 bytes per transfer

### Connection Sequence

1. Host enumerates USB device
2. VESC presents as CDC serial port
3. Host opens serial port
4. Bidirectional packet communication established

**No handshake required** - Just open port and start sending packets

---

## CAN Bus Communication

### CAN Bus Overview

Controller Area Network (CAN) is a robust multi-master serial bus originally designed for automotive applications.

**Key Characteristics:**
- Multi-master bus (any device can initiate transmission)
- Message priority based on ID (lower ID = higher priority)
- Built-in error detection and automatic retransmission
- Differential signaling (CAN_H, CAN_L) for noise immunity
- Up to 1 Mbps (standard), configurable in VESC

### VESC CAN Frame Format

The VESC uses **29-bit Extended IDs** for all CAN frames:

```
Extended ID (29 bits):
┌──────────┬────────────┬──────────┐
│ B28-B16  │   B15-B8   │  B7-B0   │
│ (Unused) │ Command ID │ VESC ID  │
└──────────┴────────────┴──────────┘
```

**Components:**
- **VESC ID** (bits 0-7): Target device ID (0-255)
- **Command ID** (bits 8-15): CAN command type
- **Unused** (bits 16-28): Reserved (set to 0)

### CAN Baud Rates

Configurable in VESC Tool. Common rates:

| Baud Rate | Recommended Use |
|-----------|-----------------|
| 125 kbps | Long cables (>20m) |
| 250 kbps | Standard installations |
| 500 kbps | Short cables, high-speed |
| 1 Mbps | Very short cables (<5m) |

### CAN Bus Wiring

**Standard CAN Bus:**
```
VESC 1          VESC 2          VESC N
  │               │               │
CAN_H ────────── CAN_H ────────── CAN_H
CAN_L ────────── CAN_L ────────── CAN_L
GND   ────────── GND   ────────── GND
  │               │               │
 120Ω            (no             120Ω
  │              termination)      │
 Term.                           Term.
```

**Important:**
- Use twisted pair cable (CAN_H and CAN_L twisted together)
- 120Ω termination resistors at **both ends** of bus
- Maximum cable length depends on baud rate:
  - 1 Mbps: 40m
  - 500 kbps: 100m
  - 250 kbps: 250m
  - 125 kbps: 500m

### Simple CAN Commands

Single-frame commands that fit in one CAN message (8 bytes data).

**Format:**
- **4 bytes**: 32-bit signed integer (big-endian)
- **Scaling**: Each command has specific scaling factor

#### Available Simple CAN Commands

| Command | ID | Scaling | Unit | Range | Description |
|---------|-----|---------|------|-------|-------------|
| `CAN_PACKET_SET_DUTY` | 0 | 100000 | % | -1.0 to 1.0 | Set PWM duty cycle |
| `CAN_PACKET_SET_CURRENT` | 1 | 1000 | A | -max to +max | Set motor current |
| `CAN_PACKET_SET_CURRENT_BRAKE` | 2 | 1000 | A | 0 to max | Set brake current |
| `CAN_PACKET_SET_RPM` | 3 | 1 | ERPM | -max to +max | Set target RPM |
| `CAN_PACKET_SET_POS` | 4 | 1000000 | degrees | 0 to 360 | Set target position |
| `CAN_PACKET_SET_CURRENT_REL` | 10 | 100000 | % | -1.0 to 1.0 | Set relative current |
| `CAN_PACKET_SET_CURRENT_BRAKE_REL` | 11 | 100000 | % | 0 to 1.0 | Set relative brake |
| `CAN_PACKET_SET_CURRENT_HANDBRAKE` | 12 | 1000 | A | 0 to max | Engage handbrake |
| `CAN_PACKET_SET_CURRENT_HANDBRAKE_REL` | 13 | 100000 | % | 0 to 1.0 | Relative handbrake |

#### CAN Status Messages

VESCs periodically broadcast status messages:

**CAN_PACKET_STATUS (ID 9):**
- ERPM (4 bytes)
- Current (2 bytes, 0.01A resolution)
- Duty cycle (2 bytes, 0.001 resolution)

**CAN_PACKET_STATUS_2 (ID 14):**
- Amp hours (4 bytes, 0.0001 Ah resolution)
- Amp hours charged (4 bytes)

**CAN_PACKET_STATUS_3 (ID 15):**
- Watt hours (4 bytes, 0.0001 Wh resolution)
- Watt hours charged (4 bytes)

**CAN_PACKET_STATUS_4 (ID 16):**
- Temp FET (2 bytes, 0.01°C resolution)
- Temp Motor (2 bytes)
- Current in (2 bytes, 0.01A resolution)
- PID position (2 bytes, 0.01° resolution)

**CAN_PACKET_STATUS_5 (ID 27):**
- Tachometer (4 bytes, steps)
- Input voltage (2 bytes, 0.01V resolution)

**CAN_PACKET_STATUS_6 (ID 28):**
- ADC1, ADC2, ADC3 (2 bytes each, 0.001V resolution)
- PPM input (2 bytes, 0.001 resolution)

### Multi-Frame CAN Commands

For complex commands that don't fit in a single CAN frame, the VESC supports multi-frame transmission:

**CAN_PACKET_FILL_RX_BUFFER (ID 5):**
- Fills receive buffer with data chunks
- Each frame carries up to 8 bytes

**CAN_PACKET_PROCESS_RX_BUFFER (ID 7):**
- Processes accumulated buffer as a standard command
- Supports all `COMM_PACKET_ID` commands

This allows sending complete configuration updates, firmware, etc. over CAN.

### CAN Forwarding

The VESC can forward commands over CAN to other devices:

**Use Case:**
- Connect to VESC #1 via USB
- VESC #1 forwards commands to VESC #2, #3, etc. via CAN
- Control entire multi-motor system from single USB connection

**Implementation:**
- VESC Tool sends `COMM_FORWARD_CAN` with target ID
- Local VESC forwards packet to CAN bus
- Target VESC executes command and responds

---

## Command Protocol

### Command Categories

The VESC supports 160+ commands organized into categories:

#### Motor Control Commands

| Command | ID | Description |
|---------|-----|-------------|
| `COMM_SET_DUTY` | 5 | Set duty cycle |
| `COMM_SET_CURRENT` | 6 | Set motor current |
| `COMM_SET_CURRENT_BRAKE` | 7 | Set brake current |
| `COMM_SET_RPM` | 8 | Set target RPM |
| `COMM_SET_POS` | 9 | Set target position |
| `COMM_SET_HANDBRAKE` | 10 | Engage handbrake |
| `COMM_SET_CURRENT_REL` | 84 | Set relative current |

#### Get Values Commands

| Command | ID | Description |
|---------|-----|-------------|
| `COMM_GET_VALUES` | 4 | Get all motor values (comprehensive) |
| `COMM_GET_VALUES_SELECTIVE` | 50 | Get specific values (reduced data) |
| `COMM_GET_VALUES_SETUP` | 47 | Get setup-specific values |
| `COMM_GET_DECODED_PPM` | 31 | Get PPM input value |
| `COMM_GET_DECODED_ADC` | 32 | Get ADC input values |
| `COMM_GET_IMU_DATA` | 65 | Get IMU data |
| `COMM_GET_STATS` | 128 | Get statistics (avg/max values) |

#### Configuration Commands

| Command | ID | Description |
|---------|-----|-------------|
| `COMM_SET_MCCONF` | 13 | Set motor configuration |
| `COMM_GET_MCCONF` | 14 | Get motor configuration |
| `COMM_GET_MCCONF_DEFAULT` | 15 | Get default motor config |
| `COMM_SET_APPCONF` | 16 | Set application configuration |
| `COMM_GET_APPCONF` | 17 | Get application configuration |
| `COMM_SET_MCCONF_TEMP` | 48 | Set temporary motor config |

#### Detection/Calibration Commands

| Command | ID | Description |
|---------|-----|-------------|
| `COMM_DETECT_MOTOR_PARAM` | 24 | Auto-detect all motor parameters |
| `COMM_DETECT_MOTOR_R_L` | 25 | Measure resistance and inductance |
| `COMM_DETECT_MOTOR_FLUX_LINKAGE` | 26 | Measure flux linkage |
| `COMM_DETECT_ENCODER` | 27 | Detect encoder offset/direction |
| `COMM_DETECT_HALL_FOC` | 28 | Detect Hall sensor table |
| `COMM_DETECT_APPLY_ALL_FOC` | 58 | Run full FOC auto-detection |

#### Terminal/Debug Commands

| Command | ID | Description |
|---------|-----|-------------|
| `COMM_TERMINAL_CMD` | 20 | Send terminal command |
| `COMM_PRINT` | 21 | Print message to terminal |
| `COMM_SAMPLE_PRINT` | 19 | Sample and print waveforms |
| `COMM_ROTOR_POSITION` | 22 | Send rotor position |
| `COMM_ALIVE` | 30 | Keepalive/ping |

#### Lisp Commands

| Command | ID | Description |
|---------|-----|-------------|
| `COMM_LISP_READ_CODE` | 130 | Read Lisp code from VESC |
| `COMM_LISP_WRITE_CODE` | 131 | Write Lisp code to VESC |
| `COMM_LISP_ERASE_CODE` | 132 | Erase Lisp code |
| `COMM_LISP_SET_RUNNING` | 133 | Start/stop Lisp script |
| `COMM_LISP_GET_STATS` | 134 | Get Lisp interpreter stats |
| `COMM_LISP_REPL_CMD` | 138 | Send REPL command |

#### Firmware Update Commands

| Command | ID | Description |
|---------|-----|-------------|
| `COMM_JUMP_TO_BOOTLOADER` | 0 | Enter bootloader mode |
| `COMM_ERASE_NEW_APP` | 1 | Erase application flash |
| `COMM_WRITE_NEW_APP_DATA` | 2 | Write firmware data |
| `COMM_WRITE_NEW_APP_DATA_LZO` | 81 | Write compressed firmware |
| `COMM_REBOOT` | 29 | Reboot device |

#### BMS Commands

| Command | ID | Description |
|---------|-----|-------------|
| `COMM_BMS_GET_VALUES` | 96 | Get BMS values |
| `COMM_BMS_SET_CHARGE_ALLOWED` | 97 | Allow/disallow charging |
| `COMM_BMS_SET_BALANCE_OVERRIDE` | 98 | Override balancing |
| `COMM_BMS_FORCE_BALANCE` | 100 | Force cell balancing |

### Data Serialization

Commands use big-endian byte order for multi-byte values.

**Helper Functions (in buffer.c):**

```c
// Appending data to buffer
void buffer_append_int16(uint8_t* buffer, int16_t number, int32_t *index);
void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index);
void buffer_append_uint16(uint8_t* buffer, uint16_t number, int32_t *index);
void buffer_append_uint32(uint8_t* buffer, uint32_t number, int32_t *index);
void buffer_append_float16(uint8_t* buffer, float number, float scale, int32_t *index);
void buffer_append_float32(uint8_t* buffer, float number, float scale, int32_t *index);

// Reading data from buffer
int16_t buffer_get_int16(const uint8_t *buffer, int32_t *index);
int32_t buffer_get_int32(const uint8_t *buffer, int32_t *index);
uint16_t buffer_get_uint16(const uint8_t *buffer, int32_t *index);
uint32_t buffer_get_uint32(const uint8_t *buffer, int32_t *index);
float buffer_get_float16(const uint8_t *buffer, float scale, int32_t *index);
float buffer_get_float32(const uint8_t *buffer, float scale, int32_t *index);
```

### Command Timeout

**Default Timeout:** 1000ms (1 second)

If no command is received within timeout period, the motor is released (stopped). This is a critical safety feature.

**Configuration:**
- Set in VESC Tool: App Settings → General → Timeout
- Set to 0 to disable (NOT recommended)
- Set timeout brake current for controlled deceleration

**Best Practice:**
Send commands continuously at 50-100 Hz to prevent timeout, even if command value hasn't changed.

---

## Implementation Files

### packet.c/.h (7.3KB)

**Purpose:** Low-level packet encoding/decoding with CRC validation

**Key Functions:**

```c
// Initialize packet handler
void packet_init(
    void (*send_func)(unsigned char *data, unsigned int len),
    void (*process_func)(unsigned char *data, unsigned int len),
    PACKET_STATE_t *state
);

// Process incoming byte (state machine)
void packet_process_byte(uint8_t rx_data, PACKET_STATE_t *state);

// Send packet with framing and CRC
void packet_send_packet(unsigned char *data, unsigned int len, PACKET_STATE_t *state);

// Reset packet state
void packet_reset(PACKET_STATE_t *state);
```

**How it Works:**

1. **Receiving:**
   - `packet_process_byte()` is called for each received byte
   - State machine tracks packet assembly
   - Validates CRC when complete
   - Calls `process_func()` callback with payload

2. **Sending:**
   - `packet_send_packet()` adds framing bytes
   - Calculates and appends CRC
   - Performs byte stuffing
   - Calls `send_func()` to transmit

**Packet State Machine:**

```
IDLE → START_BYTE → LEN → PAYLOAD → CRC → END_BYTE → PROCESS
  ↑                                                       |
  └───────────────────────────────────────────────────────┘
```

---

### commands.c/.h (Large - command processing)

**Purpose:** High-level command processing and response generation

**Key Functions:**

```c
// Initialize command handler
void commands_init(void);

// Process received packet
void commands_process_packet(
    unsigned char *data,
    unsigned int len,
    void(*reply_func)(unsigned char *data, unsigned int len)
);

// Send response packet
void commands_send_packet(unsigned char *data, unsigned int len);

// Send to last source (USB/CAN/NRF)
void commands_send_packet_can_last(unsigned char *data, unsigned int len);

// Printf to VESC Tool console
int commands_printf(const char* format, ...);

// Send motor values
void commands_send_values(void);
void commands_send_values_setup(void);

// Send configuration
void commands_send_mcconf(COMM_PACKET_ID packet_id, mc_configuration* mcconf,
    void(*reply_func)(unsigned char* data, unsigned int len));
void commands_send_appconf(COMM_PACKET_ID packet_id, app_configuration* appconf,
    void(*reply_func)(unsigned char* data, unsigned int len));
```

**Command Processing Flow:**

```c
void commands_process_packet(unsigned char *data, unsigned int len,
        void(*reply_func)(unsigned char *data, unsigned int len)) {

    COMM_PACKET_ID packet_id = data[0];  // First byte is command ID

    switch (packet_id) {
        case COMM_SET_CURRENT: {
            int32_t ind = 1;  // Skip command byte
            float current = buffer_get_float32(data, 1000.0, &ind);
            mc_interface_set_current(current);
            // No response for simple set commands
            break;
        }

        case COMM_GET_VALUES: {
            // Build response with current values
            uint8_t send_buffer[128];
            int32_t ind = 0;
            send_buffer[ind++] = COMM_GET_VALUES;
            buffer_append_float16(send_buffer, mc_interface_temp_fet_filtered(), 1e1, &ind);
            buffer_append_float16(send_buffer, mc_interface_temp_motor_filtered(), 1e1, &ind);
            buffer_append_float32(send_buffer, mc_interface_get_tot_current(), 1e2, &ind);
            // ... more values ...
            reply_func(send_buffer, ind);
            break;
        }

        // ... 160+ more commands ...
    }
}
```

---

### comm_can.c/.h (60KB)

**Purpose:** CAN bus communication and multi-device coordination

**Key Functions:**

```c
// Initialize CAN interface
void comm_can_init(void);

// Set CAN baud rate
void comm_can_set_baud(CAN_BAUD baud, int delay_msec);

// Transmit CAN frame
msg_t comm_can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len);

// Set callbacks for CAN reception
void comm_can_set_sid_rx_callback(bool (*p_func)(uint32_t id, uint8_t *data, uint8_t len));
void comm_can_set_eid_rx_callback(bool (*p_func)(uint32_t id, uint8_t *data, uint8_t len));

// Send motor control commands over CAN
void comm_can_set_duty(uint8_t controller_id, float duty);
void comm_can_set_current(uint8_t controller_id, float current);
void comm_can_set_rpm(uint8_t controller_id, float rpm);
void comm_can_set_pos(uint8_t controller_id, float pos);

// Send status messages
void comm_can_send_status1(uint8_t id, bool replace);
void comm_can_send_status2(uint8_t id, bool replace);
void comm_can_send_status3(uint8_t id, bool replace);

// Get status from other VESCs
can_status_msg *comm_can_get_status_msg_id(int id);
can_status_msg_2 *comm_can_get_status_msg_2_id(int id);
```

**CAN Status Caching:**

The VESC caches status messages from other devices on the bus:

```c
// Get status from VESC with ID 5
can_status_msg *status = comm_can_get_status_msg_id(5);
if (status != NULL) {
    float rpm = status->rpm;
    float current = status->current;
    float duty = status->duty;
}
```

---

### comm_usb.c/.h

**Purpose:** USB interface management

**Key Features:**
- USB CDC (Communication Device Class) implementation
- Automatic enumeration
- Bidirectional packet communication
- Buffer management

---

## Practical Examples

### Example 1: Setting Motor Current via USB (Python)

```python
import serial
import struct

def crc16(data):
    """Calculate CRC16-CCITT"""
    crc = 0
    for byte in data:
        crc = ((crc << 8) & 0xFF00) ^ crc16_tab[((crc >> 8) ^ byte) & 0xFF]
    return crc & 0xFFFF

def send_command(ser, command_id, data=b''):
    """Send VESC command packet"""
    payload = bytes([command_id]) + data
    payload_len = len(payload)

    # Build packet
    if payload_len <= 256:
        packet = bytes([0x02, payload_len]) + payload  # Short packet
    else:
        packet = bytes([0x03, payload_len >> 8, payload_len & 0xFF]) + payload

    # Calculate CRC
    crc = crc16(payload)
    packet += struct.pack('>H', crc)

    # Add end byte
    packet += bytes([0x03])

    # Send
    ser.write(packet)

# Connect to VESC
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

# Set motor current to 10A
COMM_SET_CURRENT = 6
current = 10.0  # Amps
current_scaled = int(current * 1000)  # Scale by 1000
data = struct.pack('>i', current_scaled)  # Big-endian 32-bit int
send_command(ser, COMM_SET_CURRENT, data)

print("Motor current set to 10A")
```

### Example 2: Reading Motor Values via USB (Python)

```python
def receive_packet(ser):
    """Receive and validate VESC packet"""
    # Wait for start byte
    while True:
        start = ser.read(1)
        if start in [b'\x02', b'\x03', b'\x04']:
            break

    # Get length
    if start == b'\x02':
        length = ord(ser.read(1))
    else:
        length = struct.unpack('>H', ser.read(2))[0]

    # Read payload
    payload = ser.read(length)

    # Read CRC
    crc_received = struct.unpack('>H', ser.read(2))[0]

    # Read end byte
    end = ser.read(1)

    # Validate
    crc_calc = crc16(payload)
    if crc_calc == crc_received and end == b'\x03':
        return payload
    else:
        return None

# Request motor values
COMM_GET_VALUES = 4
send_command(ser, COMM_GET_VALUES)

# Receive response
response = receive_packet(ser)
if response and response[0] == COMM_GET_VALUES:
    ind = 1
    temp_fet = struct.unpack('>h', response[ind:ind+2])[0] / 10.0
    ind += 2
    temp_motor = struct.unpack('>h', response[ind:ind+2])[0] / 10.0
    ind += 2
    motor_current = struct.unpack('>i', response[ind:ind+4])[0] / 100.0
    ind += 4
    input_current = struct.unpack('>i', response[ind:ind+4])[0] / 100.0
    ind += 4

    print(f"FET Temp: {temp_fet}°C")
    print(f"Motor Temp: {temp_motor}°C")
    print(f"Motor Current: {motor_current}A")
    print(f"Input Current: {input_current}A")
```

### Example 3: CAN Bus Control (Arduino/C)

```c
#include <mcp_can.h>

MCP_CAN CAN0(10);  // CS pin 10

void setup() {
    // Initialize CAN at 500kbps
    CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ);
    CAN0.setMode(MCP_NORMAL);
}

void can_set_current(uint8_t vesc_id, float current) {
    // Command ID for SET_CURRENT
    uint8_t cmd_id = 1;  // CAN_PACKET_SET_CURRENT

    // Build extended ID: (cmd_id << 8) | vesc_id
    uint32_t can_id = (cmd_id << 8) | vesc_id;

    // Scale current (1000x) and convert to big-endian
    int32_t current_scaled = (int32_t)(current * 1000.0);
    uint8_t data[4];
    data[0] = (current_scaled >> 24) & 0xFF;
    data[1] = (current_scaled >> 16) & 0xFF;
    data[2] = (current_scaled >> 8) & 0xFF;
    data[3] = current_scaled & 0xFF;

    // Send CAN frame
    CAN0.sendMsgBuf(can_id, 1, 4, data);  // 1 = extended ID
}

void loop() {
    // Set VESC ID 5 to 15A motor current
    can_set_current(5, 15.0);

    delay(20);  // Send at 50Hz to prevent timeout
}
```

### Example 4: Multi-VESC Coordination (C)

```c
// Control multiple VESCs synchronized via CAN
void sync_motors(float throttle) {
    // Calculate motor currents based on throttle
    float current = throttle * 50.0;  // 0-50A

    // Send commands to all VESCs simultaneously
    comm_can_set_current(1, current);      // Front left
    comm_can_set_current(2, current);      // Front right
    comm_can_set_current(3, current);      // Rear left
    comm_can_set_current(4, current);      // Rear right

    // Read status from all VESCs
    can_status_msg *status1 = comm_can_get_status_msg_id(1);
    can_status_msg *status2 = comm_can_get_status_msg_id(2);
    can_status_msg *status3 = comm_can_get_status_msg_id(3);
    can_status_msg *status4 = comm_can_get_status_msg_id(4);

    if (status1 && status2 && status3 && status4) {
        // Calculate average RPM
        float avg_rpm = (status1->rpm + status2->rpm +
                        status3->rpm + status4->rpm) / 4.0;

        // Detect if any motor is out of sync
        float max_deviation = fmaxf(
            fabsf(status1->rpm - avg_rpm),
            fmaxf(fabsf(status2->rpm - avg_rpm),
                 fmaxf(fabsf(status3->rpm - avg_rpm),
                      fabsf(status4->rpm - avg_rpm)))
        );

        if (max_deviation > 1000.0) {
            // Motors out of sync - take action
            commands_printf("Warning: Motor sync deviation: %.0f ERPM", max_deviation);
        }
    }
}
```

### Example 5: Custom Application Data Handler

```c
// Register custom data handler for app-specific communication
void my_app_data_handler(unsigned char *data, unsigned int len) {
    // Process custom application data
    if (len > 0) {
        uint8_t custom_cmd = data[0];

        switch (custom_cmd) {
            case 0x01:  // Custom command 1
                // Handle custom command
                float value = buffer_get_float32(data, 1000.0, &ind);
                // Process value...
                break;

            case 0x02:  // Custom command 2
                // Another custom command
                break;
        }
    }
}

void app_init(void) {
    // Register handler
    commands_set_app_data_handler(my_app_data_handler);

    // Now COMM_CUSTOM_APP_DATA packets will be routed here
}

// Send custom application data from VESC Tool
void send_custom_data_from_tool() {
    uint8_t buffer[32];
    int32_t ind = 0;

    buffer[ind++] = COMM_CUSTOM_APP_DATA;
    buffer[ind++] = 0x01;  // Custom command ID
    buffer_append_float32(buffer, 42.5, 1000.0, &ind);

    commands_send_packet(buffer, ind);
}
```

---

## Summary

The VESC communication system provides:

✅ **Multiple Interfaces:**
- USB/UART for computer connection
- CAN bus for multi-device networking
- Wireless options (NRF, LoRa)

✅ **Robust Protocol:**
- CRC error detection
- Byte stuffing for reliable framing
- Timeout safety

✅ **Comprehensive Commands:**
- 160+ commands for all functions
- Motor control, configuration, detection, debugging
- Firmware updates, Lisp scripting, BMS integration

✅ **CAN Networking:**
- Multi-master bus
- Simple and complex command support
- Status message broadcasting
- Command forwarding

✅ **Production Ready:**
- Used in thousands of commercial products
- Well-tested and reliable
- Extensive VESC Tool integration

**Key Files Reference:**
- `comm/packet.c/.h` - Packet framing and CRC (packet.c:45)
- `comm/commands.c/.h` - Command processing (commands.c:26)
- `comm/comm_can.c/.h` - CAN bus implementation (comm_can.c:30)
- `comm/comm_usb_serial.c/.h` - USB serial interface (comm_usb_serial.c)
- `documentation/comm_can.md` - Extended CAN protocol documentation

