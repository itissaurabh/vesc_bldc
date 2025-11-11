# VESC CAN Protocol Guide

## Document Information
- **File**: CAN Protocol and Message Implementation Guide
- **Author**: Documentation generated from VESC firmware analysis
- **Target Audience**: Firmware developers, CAN integration engineers
- **Prerequisites**: Understanding of CAN bus, motor control basics

---

## Table of Contents

**Part 1: CAN Protocol Overview**
- CAN Bus Architecture
- Message Structure and Addressing
- Complete Message Type Reference (69 message types)
- Message Categories and Use Cases

**Part 2: Adding New CAN Messages**
- Implementation Workflow
- Code Modifications Required
- Message Handler Implementation
- Testing and Debugging

---

# Part 1: CAN Protocol Overview

## 1. VESC CAN Architecture

### 1.1 What is VESC CAN?

VESC uses **CAN bus (Controller Area Network)** for:
- **Multi-ESC coordination** (e.g., dual motor skateboards, 4WD vehicles)
- **Status broadcasting** (ERPM, current, temperature)
- **Remote control** (set current, duty, position from master VESC)
- **BMS integration** (battery management system communication)
- **IO board communication** (external ADC/GPIO expansion)
- **GNSS data** (GPS position forwarding)

### 1.2 Hardware Setup

```
Typical Multi-VESC CAN Network:

VESC 1 (ID=0)          VESC 2 (ID=1)          IO Board (ID=10)
  [CAN_H]────────────────[CAN_H]────────────────[CAN_H]
  [CAN_L]────────────────[CAN_L]────────────────[CAN_L]
     |                      |                      |
   120Ω                   (none)                 120Ω
  terminator                                   terminator
```

**Termination**: 120Ω resistors at both ends of bus

**Baud Rate Configuration**:
```c
app_conf->can_baud_rate = CAN_BAUD_500K;  // 500 kbps (most common)
// Options: 125K, 250K, 500K, 1M, 10K, 20K, 50K, 75K
```

---

## 2. Message Structure

### 2.1 Extended ID (29-bit) Format

VESC uses CAN **Extended ID** frames:

```
29-bit Extended ID Breakdown:
┌──────────────────────────────────────────────────┐
│ Bits 28-8: Command/Packet Type (21 bits)        │
│ Bits 7-0:  Destination Controller ID (8 bits)   │
└──────────────────────────────────────────────────┘

Example:
  CAN_PACKET_SET_CURRENT (1) to Controller 5:
  EID = (1 << 8) | 5 = 0x105
```

**Construction**:
```c
uint32_t eid = (packet_type << 8) | controller_id;
```

**Extraction**:
```c
uint8_t id = eid & 0xFF;              // Controller ID
CAN_PACKET_ID cmd = (eid >> 8);       // Command type
```

### 2.2 Data Payload

**CAN Frame**: Up to **8 bytes** of data per message

**Encoding Helpers** (in `buffer.h/c`):
- `buffer_append_int32()` - 32-bit integer (4 bytes)
- `buffer_append_float32()` - 32-bit float with scaling
- `buffer_append_float16()` - 16-bit float with scaling
- `buffer_get_*()` - Corresponding decoders

**Example**:
```c
// Send current command (scaled by 1000)
uint8_t buffer[4];
int32_t send_index = 0;
buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
comm_can_transmit_eid(controller_id | (CAN_PACKET_SET_CURRENT << 8),
                       buffer, send_index);
```

### 2.3 Long Messages

For messages >8 bytes, VESC uses **fragmentation**:

```
Send 100-byte buffer to ID=5:

Step 1: Fill RX buffer in chunks
  CAN_PACKET_FILL_RX_BUFFER [offset=0,  data[7 bytes]]
  CAN_PACKET_FILL_RX_BUFFER [offset=7,  data[7 bytes]]
  CAN_PACKET_FILL_RX_BUFFER [offset=14, data[7 bytes]]
  ...

Step 2: Process complete buffer
  CAN_PACKET_PROCESS_RX_BUFFER [sender_id, len, CRC]

Step 3: VESC processes buffer contents
  → Calls commands handler with full buffer
```

---

## 3. Complete Message Type Reference

VESC supports **69 CAN message types** (IDs 0-68).

### 3.1 Motor Control Commands (0-4, 10-13)

| ID | Name | Data Format | Purpose |
|----|------|-------------|---------|
| 0 | `SET_DUTY` | int32 (duty × 100000) | Set duty cycle |
| 1 | `SET_CURRENT` | int32 (current × 1000) | Set motor current (A) |
| 2 | `SET_CURRENT_BRAKE` | int32 (current × 1000) | Set brake current (A) |
| 3 | `SET_RPM` | int32 (ERPM) | Set speed (ERPM) |
| 4 | `SET_POS` | int32 (pos × 1000000) | Set position (degrees) |
| 10 | `SET_CURRENT_REL` | float32 (×100000) | Relative current [-1, 1] |
| 11 | `SET_CURRENT_BRAKE_REL` | float32 (×100000) | Relative brake [0, 1] |
| 12 | `SET_CURRENT_HANDBRAKE` | float32 (×1000) | Handbrake current (A) |
| 13 | `SET_CURRENT_HANDBRAKE_REL` | float32 (×100000) | Relative handbrake [0, 1] |

**Example Usage**:
```c
// Set 20A on VESC ID=2
comm_can_set_current(2, 20.0);

// Set 50% relative current on VESC ID=3
comm_can_set_current_rel(3, 0.5);
```

---

### 3.2 Buffer Management (5-8)

| ID | Name | Data Format | Purpose |
|----|------|-------------|---------|
| 5 | `FILL_RX_BUFFER` | [offset:1, data:7] | Fill receive buffer (short offset) |
| 6 | `FILL_RX_BUFFER_LONG` | [offset:2, data:6] | Fill receive buffer (long offset) |
| 7 | `PROCESS_RX_BUFFER` | [sender, send, len:2, CRC:2] | Process buffered data |
| 8 | `PROCESS_SHORT_BUFFER` | [sender, send, data:n] | Process short buffer (<= 6 bytes) |

**Purpose**: Send commands longer than 8 bytes over CAN
- Use case: Forward UART commands, complex configurations

---

### 3.3 Status Messages (9, 14-16, 27, 58)

| ID | Name | Data | Content |
|----|------|------|---------|
| 9 | `STATUS` | 8 bytes | ERPM, current, duty |
| 14 | `STATUS_2` | 8 bytes | Amp hours, amp hours charged |
| 15 | `STATUS_3` | 8 bytes | Watt hours, watt hours charged |
| 16 | `STATUS_4` | 8 bytes | Temp FET, temp motor, current_in, PID pos |
| 27 | `STATUS_5` | 8 bytes | Tachometer, voltage in |
| 58 | `STATUS_6` | 8 bytes | ADC1, ADC2, ADC3, PPM |

**Broadcast Rate**: Configurable (10-1000 Hz)

**Configuration**:
```c
app_conf->can_status_rate_1 = 50;         // 50 Hz for status 1-3
app_conf->can_status_rate_2 = 10;         // 10 Hz for status 4-6
app_conf->can_status_msgs_r1 = 0x07;      // Enable STATUS_1/2/3 (bits 0-2)
app_conf->can_status_msgs_r2 = 0x38;      // Enable STATUS_4/5/6 (bits 3-5)
```

**Data Structures** (in `comm/comm_can.c`):
```c
typedef struct {
    int id;              // Controller ID
    systime_t rx_time;   // Last received time
    float rpm;
    float current;
    float duty;
} can_status_msg;

typedef struct {
    int id;
    systime_t rx_time;
    float amp_hours;
    float amp_hours_charged;
} can_status_msg_2;
// ... (status_msg_3, _4, _5, _6 similarly defined)
```

---

### 3.4 Network Management (17-18, 31, 57, 63)

| ID | Name | Data | Purpose |
|----|------|------|---------|
| 17 | `PING` | - | Request PONG response |
| 18 | `PONG` | [hw_type] | Response to PING |
| 31 | `SHUTDOWN` | - | Remote shutdown |
| 57 | `NOTIFY_BOOT` | - | Notify network of boot/restart |
| 63 | `UPDATE_BAUD` | [baud_rate] | Change CAN baud rate |

**Ping Example**:
```c
HW_TYPE hw;
if (comm_can_ping(5, &hw)) {
    // VESC ID=5 is online
    if (hw == HW_TYPE_VESC) {
        // It's a VESC (not IO board or BMS)
    }
}
```

---

### 3.5 Configuration (19-26, 29-30, 55)

| ID | Name | Purpose |
|----|------|---------|
| 19 | `DETECT_APPLY_ALL_FOC` | Trigger FOC detection on all VESCs |
| 20 | `DETECT_APPLY_ALL_FOC_RES` | Detection result response |
| 21 | `CONF_CURRENT_LIMITS` | Set current limits |
| 22 | `CONF_STORE_CURRENT_LIMITS` | Save current limits to EEPROM |
| 23 | `CONF_CURRENT_LIMITS_IN` | Set input current limits |
| 24 | `CONF_STORE_CURRENT_LIMITS_IN` | Save input limits |
| 25 | `CONF_FOC_ERPMS` | Set FOC ERPM parameters |
| 26 | `CONF_STORE_FOC_ERPMS` | Save FOC ERPM |
| 29 | `CONF_BATTERY_CUT` | Set battery cutoff voltages |
| 30 | `CONF_STORE_BATTERY_CUT` | Save battery cutoff |
| 55 | `UPDATE_PID_POS_OFFSET` | Update PID position offset |

---

### 3.6 IO Board Messages (32-37)

| ID | Name | Data | Purpose |
|----|------|------|---------|
| 32 | `IO_BOARD_ADC_1_TO_4` | 4× float16 | ADC channels 1-4 values |
| 33 | `IO_BOARD_ADC_5_TO_8` | 4× float16 | ADC channels 5-8 values |
| 34 | `IO_BOARD_ADC_9_TO_12` | 4× float16 | ADC channels 9-12 values |
| 35 | `IO_BOARD_DIGITAL_IN` | uint64 | Digital input states (64 bits) |
| 36 | `IO_BOARD_SET_OUTPUT_DIGITAL` | [pin, state] | Set digital output |
| 37 | `IO_BOARD_SET_OUTPUT_PWM` | [pin, duty] | Set PWM output |

**Use Case**: External ADC/GPIO expansion for custom inputs

---

### 3.7 BMS Messages (38-54, 64-68)

| ID | Name | Data | Content |
|----|------|------|---------|
| 38 | `BMS_V_TOT` | float32 | Total battery voltage |
| 39 | `BMS_I` | float32 | Battery current |
| 40 | `BMS_AH_WH` | 2× float32 | Amp-hours, watt-hours |
| 41 | `BMS_V_CELL` | [cell_id, float32] | Individual cell voltage |
| 42 | `BMS_BAL` | uint64 | Balancing state bitfield |
| 43 | `BMS_TEMPS` | 3× float16, uint8 | 3× temp sensors + ID |
| 44 | `BMS_HUM` | float16, float16 | Humidity, temp |
| 45 | `BMS_SOC_SOH_TEMP_STAT` | ... | State of charge, health, status |
| 48-52 | `BMS_HW_DATA_1-5` | ... | Hardware-specific BMS data |
| 53 | `BMS_AH_WH_CHG_TOTAL` | 2× float32 | Total charged Ah/Wh |
| 54 | `BMS_AH_WH_DIS_TOTAL` | 2× float32 | Total discharged Ah/Wh |
| 64-68 | `BMS_STATUS_1-5` | ... | BMS status packets |

**BMS Integration**: VESC can act as BMS or receive data from external BMS

---

### 3.8 Power Switch (PSW) Messages (46-47)

| ID | Name | Data | Purpose |
|----|------|------|---------|
| 46 | `PSW_STAT` | [state, V_in, V_out] | Power switch status |
| 47 | `PSW_SWITCH` | [on/off] | Control power switch |

---

### 3.9 Encoder/Position (28, 56)

| ID | Name | Data | Purpose |
|----|------|------|---------|
| 28 | `POLL_TS5700N8501_STATUS` | - | Request TS5700N encoder status |
| 56 | `POLL_ROTOR_POS` | - | Request rotor position |

---

### 3.10 GNSS/GPS Messages (59-62)

| ID | Name | Data | Content |
|----|------|------|---------|
| 59 | `GNSS_TIME` | [ms_today:4, yy:2, mo, dd] | GPS time |
| 60 | `GNSS_LAT` | double | Latitude |
| 61 | `GNSS_LON` | double | Longitude |
| 62 | `GNSS_ALT_SPEED_HDOP` | 3× float32 | Altitude, speed, HDOP |

**Use Case**: Forward GPS data from GNSS module to all VESCs on network

---

## 4. Message Categories

### 4.1 By Direction

**Commands (Master → Slave)**:
- Motor control (0-4, 10-13)
- Configuration (19-30, 55)
- IO control (36-37)
- PSW control (47)

**Status (Slave → Master/Broadcast)**:
- Motor status (9, 14-16, 27, 58)
- BMS data (38-54, 64-68)
- IO board data (32-35)
- PSW status (46)

**Bidirectional**:
- Ping/Pong (17-18)
- Buffer transfer (5-8)
- GNSS (59-62)

### 4.2 By Update Rate

**High Frequency (50-1000 Hz)**:
- STATUS (9) - ERPM, current, duty
- Motor control commands (0-4, 10-13)

**Medium Frequency (10-50 Hz)**:
- STATUS_2-6 (14-16, 27, 58)
- BMS data (38-45)

**Low Frequency / On-Demand**:
- Configuration (19-30)
- PING/PONG (17-18)
- Shutdown (31)

**Event-Driven**:
- Buffer transfer (5-8)
- NOTIFY_BOOT (57)

---

## 5. CAN Threading Architecture

### 5.1 Thread Overview

**4 CAN Threads** (in `comm/comm_can.c`):

| Thread | Priority | Rate | Purpose |
|--------|----------|------|---------|
| `cancom_read` | NORMALPRIO+1 | Blocking | Receive CAN frames |
| `cancom_process` | NORMALPRIO | Event-driven | Decode messages |
| `cancom_status` | NORMALPRIO | Configurable | Send STATUS 1-3 |
| `cancom_status_2` | NORMALPRIO | Configurable | Send STATUS 4-6 |

**Dual Motor** (+1 thread):
- `cancom_status_internal`: Send status for motor 2

### 5.2 Receive Flow

```
CAN Hardware RX
      ↓
cancom_read_thread (NORMALPRIO+1)
  ├─ canReceive() - Wait for CAN frame
  ├─ Store in m_rx_state ring buffer
  └─ Signal cancom_process_thread
      ↓
cancom_process_thread (NORMALPRIO)
  ├─ Fetch frame from ring buffer
  ├─ Check SID vs EID
  ├─ if (EID):
  │   ├─ Try BMS handler
  │   ├─ Try custom callback (eid_callback)
  │   ├─ Try LispBM handler
  │   └─ decode_msg() - Main VESC handler
  │       ├─ Extract ID and command
  │       ├─ Switch on CAN_PACKET_ID
  │       └─ Execute command
  └─ if (SID):
      ├─ Try custom callback (sid_callback)
      ├─ Try BMS handler
      └─ Try LispBM handler
```

### 5.3 Transmit Flow

```
Application Code
      ↓
comm_can_set_*() functions
      ↓
comm_can_transmit_eid()
      ↓
comm_can_transmit_eid_replace()
  ├─ Build CAN frame
  ├─ chMtxLock(&can_mtx) - Thread-safe
  ├─ canTransmit() - Hardware TX
  └─ chMtxUnlock(&can_mtx)
      ↓
CAN Hardware TX
```

---

## 6. Code Locations

### 6.1 Key Files

| File | Purpose |
|------|---------|
| `comm/comm_can.c` | Main CAN implementation |
| `comm/comm_can.h` | CAN API functions |
| `datatypes.h:1133-1204` | CAN_PACKET_ID enum |
| `buffer.h/c` | Data encoding/decoding |

### 6.2 Key Functions

**Send Functions** (all in `comm_can.c`):
```c
void comm_can_set_duty(uint8_t id, float duty);
void comm_can_set_current(uint8_t id, float current);
void comm_can_set_current_rel(uint8_t id, float current_rel);
void comm_can_set_rpm(uint8_t id, float rpm);
void comm_can_set_pos(uint8_t id, float pos);
// ... [more]

void comm_can_transmit_eid(uint32_t id, uint8_t *data, uint8_t len);
msg_t comm_can_transmit_eid_replace(uint32_t id, uint8_t *data,
                                     uint8_t len, bool replace, int interface);
```

**Receive Handler**:
```c
static void decode_msg(uint32_t eid, uint8_t *data8, int len, bool is_replaced);
```

**Status Broadcast**:
```c
void comm_can_send_status1(uint8_t id, bool replace);
void comm_can_send_status2(uint8_t id, bool replace);
// ... [status 3-6]
```

**Data Retrieval** (get status from other VESCs):
```c
can_status_msg *comm_can_get_status_msg_index(int index);
can_status_msg_2 *comm_can_get_status_msg_2_index(int index);
// ... [more]
```

---

## Summary of Part 1

**What We Covered**:
- ✅ VESC CAN architecture and network setup
- ✅ Extended ID message structure (29-bit)
- ✅ Complete reference of 69 message types
- ✅ Message categorization (control, status, config, BMS, IO, GNSS)
- ✅ Threading architecture (4 threads)
- ✅ RX/TX flow diagrams
- ✅ Code locations and key functions

**What's Next (Part 2)**:
- Adding new CAN message types
- Implementation workflow
- Detailed code modifications
- Testing procedures

---

## Document End (Part 1 of 2)

**Revision**: 1.0
**Date**: November 11, 2025

---

# Part 2: Adding New CAN Messages

## Document Information
- **File**: Part 2 - Implementation Guide
- **Added**: November 11, 2025
- **Prerequisites**: Part 1 (CAN protocol overview)

---

## 1. Implementation Overview

### 1.1 Steps to Add a New CAN Message

```
┌────────────────────────────────────────────────────┐
│ Step 1: Define message ID in datatypes.h          │
├────────────────────────────────────────────────────┤
│ Step 2: Implement send function in comm_can.c     │
├────────────────────────────────────────────────────┤
│ Step 3: Implement receive handler in comm_can.c   │
├────────────────────────────────────────────────────┤
│ Step 4: Add API function to comm_can.h            │
├────────────────────────────────────────────────────┤
│ Step 5: (Optional) Add LispBM bindings            │
├────────────────────────────────────────────────────┤
│ Step 6: Test on hardware                          │
└────────────────────────────────────────────────────┘
```

### 1.2 Example: Adding CAN_PACKET_CUSTOM_DATA

We'll implement a new message to send custom sensor data:
- **Message ID**: 69 (next available)
- **Purpose**: Send 3× float values (temperature, pressure, humidity)
- **Direction**: Broadcast from sensor node
- **Update rate**: 10 Hz

---

## 2. Step 1: Define Message ID

**File**: `datatypes.h`

**Location**: Around line 1203 (after existing CAN_PACKET_* definitions)

```c
typedef enum {
    CAN_PACKET_SET_DUTY                     = 0,
    // ... [existing 69 messages] ...
    CAN_PACKET_BMS_STATUS_5                 = 68,
    
    // === ADD NEW MESSAGE HERE ===
    CAN_PACKET_CUSTOM_SENSOR_DATA           = 69,  // ← New message ID
    
    CAN_PACKET_MAKE_ENUM_32_BITS = 0xFFFFFFFF,
} CAN_PACKET_ID;
```

**Important**:
- Choose next available ID (currently 69)
- Use descriptive name with `CAN_PACKET_` prefix
- Add comment explaining purpose
- Place before `CAN_PACKET_MAKE_ENUM_32_BITS`

---

## 3. Step 2: Implement Send Function

**File**: `comm/comm_can.c`

**Location**: Add after existing `comm_can_set_*` functions (around line 900)

```c
/**
 * Send custom sensor data
 *
 * @param controller_id
 * The ID of the VESC/device sending the data.
 *
 * @param temperature
 * Temperature in degrees Celsius.
 *
 * @param pressure
 * Pressure in kPa.
 *
 * @param humidity
 * Relative humidity in %.
 */
void comm_can_send_sensor_data(uint8_t controller_id,
                                 float temperature,
                                 float pressure,
                                 float humidity) {
    int32_t send_index = 0;
    uint8_t buffer[8];
    
    // Pack 3 floats into 8 bytes (compressed format)
    buffer_append_float16(buffer, temperature, 1e2, &send_index);  // ±327°C, 0.01° res
    buffer_append_float16(buffer, pressure, 1e1, &send_index);     // ±3276 kPa, 0.1 kPa res
    buffer_append_float16(buffer, humidity, 1e2, &send_index);     // ±327%, 0.01% res
    
    // Optional: Add timestamp or status byte
    buffer[send_index++] = 0;  // Reserved/status byte
    buffer[send_index++] = 0;  // Reserved byte
    
    // Transmit
    comm_can_transmit_eid_replace(controller_id |
            ((uint32_t)CAN_PACKET_CUSTOM_SENSOR_DATA << 8),
            buffer, send_index, true, 0);
}
```

**Data Encoding Notes**:
- `buffer_append_float16()`: 16-bit float with scaling
  - Format: `int16_t = value × scale`
  - Range: ±32767 / scale
  - Example: `temperature × 100` → -327 to +327°C with 0.01° resolution
- `buffer_append_float32()`: 32-bit float (4 bytes, higher precision)
- `buffer_append_int32()`: 32-bit integer (4 bytes)
- `buffer_append_int16()`: 16-bit integer (2 bytes)

**Transmit Options**:
- `comm_can_transmit_eid()`: Basic transmit
- `comm_can_transmit_eid_replace()`: Replace if already in TX queue (prevents flooding)
  - `replace = true`: Recommended for periodic status messages
  - `replace = false`: For commands that must not be lost

---

## 4. Step 3: Implement Receive Handler

**File**: `comm/comm_can.c`

**Location**: In `decode_msg()` function, around line 1850 (after existing `case` statements)

```c
static void decode_msg(uint32_t eid, uint8_t *data8, int len, bool is_replaced) {
    int32_t ind = 0;
    uint8_t id = eid & 0xFF;
    CAN_PACKET_ID cmd = eid >> 8;
    
    // ... [existing code] ...
    
    // Existing message handlers
    switch (cmd) {
    case CAN_PACKET_SET_DUTY:
        // ... [existing handler] ...
        break;
        
    // ... [many more existing cases] ...
    
    // === ADD NEW HANDLER HERE ===
    case CAN_PACKET_CUSTOM_SENSOR_DATA: {
        ind = 0;
        float temperature = buffer_get_float16(data8, 1e2, &ind);
        float pressure = buffer_get_float16(data8, 1e1, &ind);
        float humidity = buffer_get_float16(data8, 1e2, &ind);
        uint8_t status = data8[ind++];
        uint8_t reserved = data8[ind++];
        
        // Process received data
        // Option 1: Store in global structure
        g_custom_sensor_data[id].temperature = temperature;
        g_custom_sensor_data[id].pressure = pressure;
        g_custom_sensor_data[id].humidity = humidity;
        g_custom_sensor_data[id].rx_time = chVTGetSystemTime();
        
        // Option 2: Call handler function
        // handle_custom_sensor_data(id, temperature, pressure, humidity);
        
        // Option 3: Forward to LispBM
        // lispif_process_custom_sensor(id, temperature, pressure, humidity);
        
    } break;
    
    default:
        break;
    }
}
```

**Storage Structure** (add near top of `comm_can.c`):

```c
// Custom sensor data storage
typedef struct {
    float temperature;
    float pressure;
    float humidity;
    systime_t rx_time;      // When data was last received
    int id;                 // Controller ID
} custom_sensor_data;

static custom_sensor_data g_custom_sensor_data[CAN_STATUS_MSGS_TO_STORE];
```

**Initialization** (in `comm_can_init()`):

```c
void comm_can_init(void) {
    // ... [existing init code] ...
    
    // Initialize custom sensor data
    for (int i = 0; i < CAN_STATUS_MSGS_TO_STORE; i++) {
        g_custom_sensor_data[i].id = -1;
        g_custom_sensor_data[i].temperature = 0.0;
        g_custom_sensor_data[i].pressure = 0.0;
        g_custom_sensor_data[i].humidity = 0.0;
    }
    
    // ... [rest of init] ...
}
```

---

## 5. Step 4: Add API Functions

**File**: `comm/comm_can.h`

**Location**: Around line 100 (after existing function declarations)

```c
// Existing declarations
void comm_can_set_duty(uint8_t controller_id, float duty);
void comm_can_set_current(uint8_t controller_id, float current);
// ... [many more] ...

// === ADD NEW DECLARATIONS HERE ===

// Send custom sensor data
void comm_can_send_sensor_data(uint8_t controller_id,
                                float temperature,
                                float pressure,
                                float humidity);

// Get received sensor data
custom_sensor_data *comm_can_get_sensor_data(int controller_id);
custom_sensor_data *comm_can_get_sensor_data_index(int index);
```

**Getter Implementation** (in `comm_can.c`):

```c
/**
 * Get custom sensor data by controller ID
 */
custom_sensor_data *comm_can_get_sensor_data(int controller_id) {
    for (int i = 0; i < CAN_STATUS_MSGS_TO_STORE; i++) {
        if (g_custom_sensor_data[i].id == controller_id) {
            return &g_custom_sensor_data[i];
        }
    }
    return NULL;
}

/**
 * Get custom sensor data by array index
 */
custom_sensor_data *comm_can_get_sensor_data_index(int index) {
    if (index < CAN_STATUS_MSGS_TO_STORE) {
        return &g_custom_sensor_data[index];
    }
    return NULL;
}
```

---

## 6. Step 5: (Optional) Add LispBM Bindings

**File**: `lispBM/lispif_vesc_extensions.c`

**Location**: Around line 6100 (after existing CAN-related extensions)

```c
// Send custom sensor data from LispBM
static lbm_value ext_can_send_sensor_data(lbm_value *args, lbm_uint argn) {
    LBM_CHECK_ARGN_NUMBER(4);
    
    uint8_t id = lbm_dec_as_u32(args[0]);
    float temp = lbm_dec_as_float(args[1]);
    float pressure = lbm_dec_as_float(args[2]);
    float humidity = lbm_dec_as_float(args[3]);
    
    comm_can_send_sensor_data(id, temp, pressure, humidity);
    
    return ENC_SYM_TRUE;
}

// Get custom sensor data in LispBM
static lbm_value ext_can_get_sensor_data(lbm_value *args, lbm_uint argn) {
    LBM_CHECK_ARGN_NUMBER(1);
    
    int id = lbm_dec_as_i32(args[0]);
    custom_sensor_data *data = comm_can_get_sensor_data(id);
    
    if (data == NULL || data->id != id) {
        return ENC_SYM_NIL;
    }
    
    // Return as list: (temp pressure humidity age_ms)
    lbm_value res = ENC_SYM_NIL;
    res = lbm_cons(lbm_enc_float(ST2MS(chVTTimeElapsedSinceX(data->rx_time))), res);
    res = lbm_cons(lbm_enc_float(data->humidity), res);
    res = lbm_cons(lbm_enc_float(data->pressure), res);
    res = lbm_cons(lbm_enc_float(data->temperature), res);
    
    return res;
}

// Register extensions (in lispif_load_vesc_extensions())
void lispif_load_vesc_extensions(bool main_found) {
    // ... [existing registrations] ...
    
    lbm_add_extension("can-send-sensor", ext_can_send_sensor_data);
    lbm_add_extension("can-get-sensor", ext_can_get_sensor_data);
}
```

**LispBM Usage**:
```lisp
; Send sensor data (ID=5, temp=25.5°C, pressure=101.3kPa, humidity=65%)
(can-send-sensor 5 25.5 101.3 65.0)

; Read sensor data from ID=3
(define data (can-get-sensor 3))
; data = (25.5 101.3 65.0 234)  ; (temp pressure humidity age_ms)

(if (not-eq data nil)
    (let ((temp (first data))
          (pressure (second data))
          (humidity (third data))
          (age (fourth data)))
        (print (str-from-n temp "Temperature: %.1f C"))))
```

---

## 7. Complete Example: Motor Torque Command

Let's implement a more complex example: **CAN_PACKET_SET_TORQUE_LIMITS**

### 7.1 Define Message ID

```c
// datatypes.h
CAN_PACKET_SET_TORQUE_LIMITS = 70,
```

### 7.2 Implement Send Function

```c
// comm_can.c
void comm_can_set_torque_limits(uint8_t controller_id,
                                  float torque_max_positive,
                                  float torque_max_negative,
                                  bool store) {
    int32_t send_index = 0;
    uint8_t buffer[8];
    
    buffer_append_float16(buffer, torque_max_positive, 1e2, &send_index);  // Nm × 100
    buffer_append_float16(buffer, torque_max_negative, 1e2, &send_index);  // Nm × 100
    buffer[send_index++] = store ? 1 : 0;  // Store to EEPROM flag
    
    comm_can_transmit_eid_replace(controller_id |
            ((uint32_t)CAN_PACKET_SET_TORQUE_LIMITS << 8),
            buffer, send_index, true, 0);
}
```

### 7.3 Implement Receive Handler

```c
// comm_can.c - in decode_msg()
case CAN_PACKET_SET_TORQUE_LIMITS: {
    ind = 0;
    float torque_pos = buffer_get_float16(data8, 1e2, &ind);
    float torque_neg = buffer_get_float16(data8, 1e2, &ind);
    bool store = data8[ind++] != 0;
    
    // Validate limits
    utils_truncate_number_abs(&torque_pos, 500.0);  // Max ±500 Nm
    utils_truncate_number_abs(&torque_neg, 500.0);
    
    // Apply to motor configuration
    mc_configuration *mcconf = mempools_alloc_mcconf();
    *mcconf = *mc_interface_get_configuration();
    
    mcconf->l_current_max = torque_pos / mcconf->foc_motor_flux_linkage;
    mcconf->l_current_min = torque_neg / mcconf->foc_motor_flux_linkage;
    
    mc_interface_set_configuration(mcconf);
    
    if (store) {
        conf_general_store_mc_configuration(mcconf, mc_interface_get_motor_thread() == 2);
    }
    
    mempools_free_mcconf(mcconf);
    timeout_reset();  // Reset timeout (this is a control command)
} break;
```

### 7.4 Add to Header

```c
// comm_can.h
void comm_can_set_torque_limits(uint8_t controller_id,
                                  float torque_max_positive,
                                  float torque_max_negative,
                                  bool store);
```

---

## 8. Testing New Messages

### 8.1 Hardware Testing Procedure

#### Step 1: Build and Flash

```bash
# Build firmware with new message
make

# Flash to VESC
make upload
```

#### Step 2: Terminal Testing

```c
// Add test command in main.c or comm_can.c
void terminal_test_can_sensor(int argc, const char **argv) {
    if (argc == 4) {
        float temp = strtof(argv[0], NULL);
        float pressure = strtof(argv[1], NULL);
        float humidity = strtof(argv[2], NULL);
        
        comm_can_send_sensor_data(app_get_configuration()->controller_id,
                                   temp, pressure, humidity);
        commands_printf("Sent: T=%.1f P=%.1f H=%.1f", temp, pressure, humidity);
    } else {
        commands_printf("Usage: test_can_sensor <temp> <pressure> <humidity>");
    }
}

// Register in hw_init_gpio() or comm_can_init()
terminal_register_command_callback("test_can_sensor",
                                   "Test custom CAN sensor message",
                                   0, terminal_test_can_sensor);
```

**Usage**:
```
VESC Terminal → test_can_sensor 25.5 101.3 65.0
```

#### Step 3: Monitor Reception

```c
// Add receive monitor command
void terminal_show_can_sensor(int argc, const char **argv) {
    commands_printf("=== Custom Sensor Data ===");
    for (int i = 0; i < CAN_STATUS_MSGS_TO_STORE; i++) {
        custom_sensor_data *data = comm_can_get_sensor_data_index(i);
        if (data->id >= 0) {
            float age_s = ST2MS(chVTTimeElapsedSinceX(data->rx_time)) / 1000.0;
            commands_printf("ID %d: T=%.1f P=%.1f H=%.1f (%.1fs ago)",
                           data->id,
                           data->temperature,
                           data->pressure,
                           data->humidity,
                           age_s);
        }
    }
}

terminal_register_command_callback("show_can_sensor",
                                   "Show received CAN sensor data",
                                   0, terminal_show_can_sensor);
```

### 8.2 LispBM Testing

```lisp
; Send sensor data every second
(loopwhile t
    (progn
        (can-send-sensor 0 (+ 20 (rand 10)) 101.3 (+ 50 (rand 20)))
        (sleep 1.0)))

; Monitor received data
(loopwhile t
    (progn
        (define data (can-get-sensor 1))
        (if (not-eq data nil)
            (print (list "Sensor 1:" data)))
        (sleep 0.5)))
```

### 8.3 Oscilloscope/CAN Analyzer

**Verify CAN Frame**:
```
Expected frame:
  ID: 0x4500 (CAN_PACKET_CUSTOM_SENSOR_DATA=69 << 8 | controller_id=0)
  DLC: 8
  Data: [temp_H, temp_L, pres_H, pres_L, hum_H, hum_L, 0x00, 0x00]
  
Example:
  25.5°C, 101.3 kPa, 65.0%
  → temp = 25.5 × 100 = 2550 = 0x09F6
  → pres = 101.3 × 10 = 1013 = 0x03F5
  → hum = 65.0 × 100 = 6500 = 0x1964
  
  Data: [09 F6 03 F5 19 64 00 00]
```

---

## 9. Best Practices

### 9.1 Message Design

**DO**:
- ✅ Use descriptive names (CAN_PACKET_SET_TORQUE, not CAN_PACKET_CTRL)
- ✅ Pack data efficiently (use float16 when possible)
- ✅ Include versioning/status byte if protocol may evolve
- ✅ Document units and scaling factors in comments
- ✅ Validate received data (range checks)

**DON'T**:
- ❌ Reuse existing message IDs
- ❌ Send high-rate messages (>1 kHz) without consideration
- ❌ Forget to handle dual-motor systems
- ❌ Skip timeout_reset() for control commands
- ❌ Use float32 for all data (wastes bandwidth)

### 9.2 Performance Considerations

**CAN Bus Loading**:
```
CAN Frame overhead: ~44 bits (start, ID, CRC, ACK, EOF, IFS)
Data: 0-8 bytes = 0-64 bits
Total: 44-108 bits per frame

At 500 kbps CAN:
  Max theoretical: ~4600 frames/sec
  Practical max: ~2000 frames/sec (accounting for bus arbitration)

Example:
  6× STATUS messages @ 50 Hz = 300 frames/sec = 15% bus load
  10× Control commands @ 100 Hz = 1000 frames/sec = 50% bus load
```

**Recommendations**:
- Periodic status: 10-50 Hz
- Control commands: up to 500 Hz (only when needed)
- Burst transfers (buffer fill): Avoid unless necessary

### 9.3 Dual Motor Handling

If message applies to specific motor:

```c
// In decode_msg()
#ifdef HW_HAS_DUAL_MOTORS
    int motor_last = mc_interface_get_motor_thread();
    int id1 = app_get_configuration()->controller_id;
    int id2 = utils_second_motor_id();
    
    if (id == id2) {
        mc_interface_select_motor_thread(2);
    } else {
        mc_interface_select_motor_thread(1);
    }
#endif

    // ... [process message for selected motor] ...

#ifdef HW_HAS_DUAL_MOTORS
    mc_interface_select_motor_thread(motor_last);  // Restore
#endif
```

---

## 10. Debugging

### 10.1 Common Issues

| Problem | Likely Cause | Solution |
|---------|--------------|----------|
| Message not received | Wrong ID in switch | Check `CAN_PACKET_ID` value |
| Garbled data | Encoding/decoding mismatch | Verify scale factors match |
| Bus errors | Termination issue | Check 120Ω resistors |
| Dropped messages | Bus overload | Reduce message rate |
| No transmission | `replace=true` drops msg | Use `replace=false` |

### 10.2 Debug Logging

**Add to send function**:
```c
void comm_can_send_sensor_data(...) {
    // ... [encoding] ...
    
#ifdef DEBUG_CAN
    commands_printf("CAN TX: ID=0x%X, len=%d, data=[%02X %02X ...]",
                   controller_id | (CAN_PACKET_CUSTOM_SENSOR_DATA << 8),
                   send_index, buffer[0], buffer[1]);
#endif
    
    comm_can_transmit_eid_replace(...);
}
```

**Add to receive handler**:
```c
case CAN_PACKET_CUSTOM_SENSOR_DATA: {
#ifdef DEBUG_CAN
    commands_printf("CAN RX: Custom sensor from ID=%d", id);
#endif
    // ... [decode] ...
}
```

**Enable**:
```c
// In makefile or build system
CFLAGS += -DDEBUG_CAN
```

---

## 11. Summary

### 11.1 Quick Reference Checklist

When adding new CAN message:

- [ ] **Step 1**: Add to `CAN_PACKET_ID` enum in `datatypes.h`
- [ ] **Step 2**: Implement `comm_can_send_xxx()` in `comm_can.c`
- [ ] **Step 3**: Add `case CAN_PACKET_XXX:` in `decode_msg()`
- [ ] **Step 4**: Declare function in `comm_can.h`
- [ ] **Step 5**: (Optional) Add LispBM bindings
- [ ] **Step 6**: Add terminal test command
- [ ] **Step 7**: Test on hardware (TX and RX)
- [ ] **Step 8**: Verify with CAN analyzer
- [ ] **Step 9**: Document in code comments
- [ ] **Step 10**: Update this guide if needed!

### 11.2 Files Modified Summary

For each new message type:

| File | What to Add |
|------|-------------|
| `datatypes.h` | `CAN_PACKET_ID` enum entry |
| `comm/comm_can.c` | Send function, receive handler, storage struct, init code |
| `comm/comm_can.h` | Function declarations |
| `lispBM/lispif_vesc_extensions.c` | (Optional) LispBM bindings |

---

## 12. Advanced Topics

### 12.1 Custom Callbacks

For application-specific handling without modifying `comm_can.c`:

```c
// In your application code
bool my_can_eid_callback(uint32_t id, uint8_t *data, uint8_t len) {
    CAN_PACKET_ID cmd = id >> 8;
    uint8_t controller_id = id & 0xFF;
    
    if (cmd == CAN_PACKET_CUSTOM_SENSOR_DATA) {
        // Handle custom message
        // Return true to indicate message was handled
        return true;
    }
    
    return false;  // Not handled, let default handler process
}

// Register callback
comm_can_set_eid_rx_callback(my_can_eid_callback);
```

### 12.2 SID (Standard ID) Messages

VESC primarily uses Extended ID, but SID callback is available:

```c
bool my_can_sid_callback(uint32_t id, uint8_t *data, uint8_t len) {
    // Handle 11-bit standard ID messages
    if (id == 0x123) {
        // Process message
        return true;
    }
    return false;
}

comm_can_set_sid_rx_callback(my_can_sid_callback);
```

---

## Complete Guide Summary

### What We Covered Across Both Parts

**Part 1: Protocol Overview**
- ✅ VESC CAN architecture and network setup
- ✅ 29-bit Extended ID message structure
- ✅ Complete reference of 69 message types
- ✅ Message categories (control, status, config, BMS, IO, GNSS)
- ✅ Threading architecture (4 CAN threads)
- ✅ RX/TX flow diagrams

**Part 2: Implementation Guide** (This Part)
- ✅ Step-by-step workflow for adding messages
- ✅ Complete code examples (send, receive, LispBM)
- ✅ Data encoding/decoding techniques
- ✅ Testing procedures (terminal, LispBM, hardware)
- ✅ Best practices and performance tips
- ✅ Debugging techniques
- ✅ Dual motor and custom callback handling

---

## Document End (Part 2 of 2 - Complete)

**Complete CAN Protocol Guide**
**Revision**: 1.0
**Date**: November 11, 2025

**Total Pages**: Part 1 (503 lines) + Part 2 (700+ lines) = 1200+ lines

**Coverage**:
- ✅ CAN protocol structure and addressing
- ✅ All 69 message types documented
- ✅ Complete implementation workflow
- ✅ Code examples for sending and receiving
- ✅ LispBM integration
- ✅ Testing and debugging procedures
- ✅ Best practices and performance guidelines

