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
