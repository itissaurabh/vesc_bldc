# ChibiOS Integration in VESC Firmware

**Purpose:** Comprehensive guide to understanding ChibiOS RTOS and its integration in VESC firmware

## Table of Contents
1. [ChibiOS Basics](#chibios-basics)
2. [RTOS Fundamental Concepts](#rtos-fundamental-concepts)
3. [ChibiOS Architecture](#chibios-architecture)
4. [Thread Management in VESC](#thread-management-in-vesc)
5. [All Threads in VESC Firmware](#all-threads-in-vesc-firmware)
6. [Synchronization Primitives](#synchronization-primitives)
7. [Interrupt Integration](#interrupt-integration)
8. [Motor Control Thread Architecture](#motor-control-thread-architecture)
9. [Learning Resources](#learning-resources)

---

## ChibiOS Basics

### What is ChibiOS?

**ChibiOS/RT** (Real-Time) is a compact and fast real-time operating system (RTOS) supporting multiple architectures. The VESC firmware uses **ChibiOS 3.0.5**.

**Key Features:**
- **Small footprint:** ~6KB ROM, ~300 bytes RAM
- **Fast:** Context switch in microseconds
- **Preemptive:** Higher priority tasks interrupt lower priority tasks
- **Static memory allocation:** No malloc/free at runtime
- **Tick-less scheduling:** Can sleep in low-power modes
- **Rich API:** Threads, semaphores, mutexes, events, timers, memory pools

**Why RTOS for Motor Control?**

Motor control requires:
1. **Real-time response:** ADC sampling at 10-40 kHz
2. **Concurrent operations:** Motor control + communication + UI + logging
3. **Priority scheduling:** Critical motor control must preempt non-critical tasks
4. **Deterministic timing:** Predictable response times
5. **Resource sharing:** Safe access to shared hardware (SPI, I2C, UART)

---

## RTOS Fundamental Concepts

### 1. Threads (Tasks)

A **thread** is an independent execution context with its own:
- **Stack:** Local variables and function call history
- **Priority:** Determines scheduling order
- **State:** Running, Ready, Suspended, Sleeping

**Thread States:**

```
         ┌──────────┐
  spawn  │  READY   │ ◄─── wakeup ──┐
    │    └──────────┘                │
    │         │                      │
    │    schedule                    │
    │         │                      │
    ▼         ▼                      │
┌──────────────┐              ┌───────────┐
│   RUNNING    │ ──────────►  │  SLEEPING │
└──────────────┘   sleep      └───────────┘
    │         │                      ▲
    │    preempt                     │
    │         │                      │
    │         ▼                      │
    │    ┌──────────┐                │
    │    │  READY   │ ───────────────┘
    │    └──────────┘
    │
  exit
    │
    ▼
┌──────────┐
│TERMINATED│
└──────────┘
```

### 2. Scheduling

**Priority-based preemptive scheduling:**
- Each thread has a priority (0 = lowest, 255 = highest)
- Highest priority ready thread always runs
- Equal priority threads round-robin (time slicing)
- Lower priority threads only run when higher are blocked

**Priority Levels in VESC:**
```c
#define IDLEPRIO        1    // Idle thread
#define LOWPRIO         2    // Background tasks
#define NORMALPRIO      64   // Normal priority
#define HIGHPRIO        127  // High priority
#define EMERGENCY_PRIO  255  // Critical ISR threads
```

### 3. Synchronization Primitives

**Why synchronization?**
- Multiple threads accessing shared resources cause **race conditions**
- Need mechanisms to coordinate thread execution

**ChibiOS Primitives:**

| Primitive | Purpose | Use Case |
|-----------|---------|----------|
| **Mutex** | Mutual exclusion | Protect shared data (e.g., configuration) |
| **Semaphore** | Signaling/counting | Resource availability (e.g., buffer slots) |
| **Event** | Notification | Wake thread on condition (e.g., ADC complete) |
| **Mailbox** | Message passing | Send data between threads |
| **Condition Variable** | Complex conditions | Wait for specific state |
| **Virtual Timer** | One-shot timer | Delayed actions |

### 4. Memory Management

**Static Allocation (VESC approach):**
```c
// Declare working area (stack) for thread
static THD_WORKING_AREA(my_thread_wa, 512);  // 512 bytes stack

// Create thread
chThdCreateStatic(my_thread_wa, sizeof(my_thread_wa),
                  NORMALPRIO, my_thread, NULL);
```

**Advantages:**
- No heap fragmentation
- Deterministic memory usage
- No malloc failures at runtime
- Stack overflow detection possible

**Memory Pools:**
VESC uses memory pools for temporary large structures:
```c
mc_configuration *mcconf = mempools_alloc_mcconf();
// Use mcconf...
mempools_free_mcconf(mcconf);
```

### 5. Time Management

**System Tick:**
- Configured at 1000 Hz (1ms per tick) in VESC
- Drives scheduling and timeouts

**Time Functions:**
```c
chThdSleepMilliseconds(100);    // Sleep 100ms
chThdSleepMicroseconds(500);    // Sleep 500µs
systime_t now = chVTGetSystemTime();  // Get current tick
```

---

## ChibiOS Architecture

### Core Components

```
┌─────────────────────────────────────────────────┐
│              Application Layer                   │
│   (VESC motor control, apps, communication)     │
└─────────────────────────────────────────────────┘
                    │
┌─────────────────────────────────────────────────┐
│              ChibiOS/RT Kernel                   │
├─────────────────────────────────────────────────┤
│  • Scheduler (preemptive, priority-based)       │
│  • Thread Management                             │
│  • Synchronization Primitives                    │
│  • Time Management (Virtual Timers)             │
│  • Memory Pools                                  │
│  • System Calls                                  │
└─────────────────────────────────────────────────┘
                    │
┌─────────────────────────────────────────────────┐
│              ChibiOS/HAL                         │
│  (Hardware Abstraction Layer)                    │
├─────────────────────────────────────────────────┤
│  • PAL (GPIO)           • SPI                   │
│  • Serial               • I2C                   │
│  • USB                  • ADC                   │
│  • CAN                  • PWM                   │
└─────────────────────────────────────────────────┘
                    │
┌─────────────────────────────────────────────────┐
│              STM32 Hardware                      │
│  (ARM Cortex-M4, peripherals)                   │
└─────────────────────────────────────────────────┘
```

### Configuration (chconf.h, halconf.h)

Key settings in `chconf.h`:
```c
#define CH_CFG_ST_FREQUENCY                 1000    // 1kHz tick
#define CH_CFG_TIME_QUANTUM                 20      // 20ms timeslice
#define CH_CFG_MEMCORE_SIZE                 0       // No heap
#define CH_CFG_NO_IDLE_THREAD               FALSE   // Enable idle thread
#define CH_CFG_OPTIMIZE_SPEED               TRUE    // Speed over size
```

---

## Thread Management in VESC

### Thread Creation Pattern

**Step 1: Declare working area (stack)**
```c
static THD_WORKING_AREA(my_thread_wa, 512);
```
- Allocates 512 bytes for thread stack
- `static` ensures it stays in memory
- Must be large enough for local variables + nested calls

**Step 2: Define thread function**
```c
static THD_FUNCTION(my_thread, arg) {
    (void)arg;  // Unused parameter

    chRegSetThreadName("MyThread");  // Set name for debugging

    for (;;) {  // Infinite loop
        // Thread work here

        chThdSleepMilliseconds(10);  // Sleep 10ms
    }
}
```

**Step 3: Create and start thread**
```c
chThdCreateStatic(my_thread_wa, sizeof(my_thread_wa),
                  NORMALPRIO, my_thread, NULL);
```
- `NORMALPRIO`: Thread priority
- `my_thread`: Function pointer
- `NULL`: Argument passed to thread

### Thread Lifecycle

```c
// Thread creation (in main or init function)
thread_t *tp = chThdCreateStatic(...);

// Thread runs continuously
// Inside thread:
for (;;) {
    // Do work

    // Sleep (yields CPU to other threads)
    chThdSleepMilliseconds(100);

    // Or wait on event/semaphore
    chEvtWaitAny(ALL_EVENTS);
}

// Thread termination (rare in embedded systems)
chThdExit(MSG_OK);
```

---

## All Threads in VESC Firmware

### System-Level Threads (main.c)

#### 1. LED Thread - Lines 111-155
```c
static THD_WORKING_AREA(led_thread_wa, 256);
static THD_FUNCTION(led_thread, arg) {
    chRegSetThreadName("Main LED");

    for (;;) {
        // Update status LEDs based on motor state
        // Flash fault codes on red LED
        chThdSleepMilliseconds(10);  // 100 Hz update
    }
}
```

**Purpose:** Visual feedback via LEDs
- **Priority:** NORMALPRIO
- **Stack:** 256 bytes
- **Rate:** 100 Hz
- **Responsibilities:**
  - Blink green LED when motor running
  - Flash red LED fault codes
  - LED patterns for different states

**Created:** `main.c:327`

---

#### 2. Periodic Thread - Lines 157-209
```c
static THD_WORKING_AREA(periodic_thread_wa, 256);
static THD_FUNCTION(periodic_thread, arg) {
    chRegSetThreadName("Main periodic");

    for (;;) {
        // Report motor detection position via CAN
        // HSI temperature compensation
        chThdSleepMilliseconds(1000);  // 1 Hz
    }
}
```

**Purpose:** Low-priority periodic tasks
- **Priority:** NORMALPRIO
- **Stack:** 256 bytes
- **Rate:** 1 Hz
- **Responsibilities:**
  - Motor detection position reporting
  - HSI oscillator temperature compensation
  - Periodic CAN messages

**Created:** `main.c:328`

---

#### 3. Flash Integrity Check Thread - Lines 96-109
```c
static THD_WORKING_AREA(flash_integrity_check_thread_wa, 256);
static THD_FUNCTION(flash_integrity_check_thread, arg) {
    chRegSetThreadName("Flash check");

    for (;;) {
        // Calculate CRC of configuration in flash
        // Compare with stored CRC
        // Reset if corruption detected
        chThdSleepMilliseconds(6000);  // Every 6 seconds
    }
}
```

**Purpose:** Detect flash memory corruption
- **Priority:** LOWPRIO (background task)
- **Stack:** 256 bytes
- **Rate:** 0.17 Hz (every 6 seconds)
- **Responsibilities:**
  - CRC check of motor/app configuration
  - Automatic reset on corruption
  - Fault code reporting

**Created:** `main.c:329`

---

### Motor Control Threads (mc_interface.c)

#### 4. Timer Thread - Lines 2689-2702
```c
static THD_FUNCTION(timer_thread, arg) {
    chRegSetThreadName("mcif timer");

    for (;;) {
        run_timer_tasks(&m_motor_1);
#ifdef HW_HAS_DUAL_MOTORS
        run_timer_tasks(&m_motor_2);
#endif
        chThdSleepMilliseconds(1);  // 1000 Hz
    }
}
```

**Purpose:** 1ms periodic motor control tasks
- **Priority:** NORMALPRIO
- **Stack:** 512 bytes
- **Rate:** 1000 Hz
- **Responsibilities:**
  - Update dynamic current limits (temperature, voltage, RPM)
  - Filter battery voltage
  - Clear faults when conditions resolve
  - Update odometer and runtime
  - Monitor current sensor offsets
  - Control auxiliary output
  - Check encoder faults

**Created:** `mc_interface_init():184`

---

#### 5. Sample Send Thread - Lines 2864-2900
```c
static THD_FUNCTION(sample_send_thread, arg) {
    chRegSetThreadName("SampleSender");
    sample_send_tp = chThdGetSelfX();

    for (;;) {
        chEvtWaitAny((eventmask_t) 1);  // Wait for signal

        // Send collected ADC samples over communication
        for (int i = 0; i < len; i++) {
            send_sample_block(i, offset);
        }
    }
}
```

**Purpose:** Debug sample transmission
- **Priority:** NORMALPRIO
- **Stack:** 512 bytes
- **Rate:** Event-driven (triggered by sampling complete)
- **Responsibilities:**
  - Wait for sampling complete signal
  - Transmit ADC waveform data
  - Send current, voltage, phase angle samples

**Created:** `mc_interface_init():191`

**Synchronization:** Event-driven via `chEvtSignal()`

---

#### 6. Fault Stop Thread - Lines 2902-2996
```c
static THD_FUNCTION(fault_stop_thread, arg) {
    chRegSetThreadName("Fault Stop");
    fault_stop_tp = chThdGetSelfX();

    for (;;) {
        chEvtWaitAny((eventmask_t) 1);  // Wait for fault signal

        // Log fault data
        // Stop PWM output
        // Set fault state
    }
}
```

**Purpose:** Emergency motor shutdown on faults
- **Priority:** NORMALPRIO (but signaled from high-priority ISR)
- **Stack:** 512 bytes
- **Rate:** Event-driven (triggered by fault detection)
- **Responsibilities:**
  - Stop motor PWM immediately
  - Log comprehensive fault data
  - Set ignore iterations timer
  - Report fault via terminal

**Created:** `mc_interface_init():198`

**Critical Path:** ISR → Event Signal → Thread wakes → Motor stops

---

#### 7. Statistics Thread - Lines 2804-2817
```c
static THD_FUNCTION(stat_thread, arg) {
    chRegSetThreadName("StatCounter");

    for (;;) {
        update_stats(&m_motor_1);
#ifdef HW_HAS_DUAL_MOTORS
        update_stats(&m_motor_2);
#endif
        chThdSleepMilliseconds(10);  // 100 Hz
    }
}
```

**Purpose:** Performance statistics collection
- **Priority:** NORMALPRIO
- **Stack:** 256 bytes
- **Rate:** 100 Hz
- **Responsibilities:**
  - Track average/max power, speed, current
  - Monitor temperatures
  - Calculate running averages

**Created:** `mc_interface_init():205`

---

### Timeout Thread (timeout.c)

#### 8. Timeout Thread - Lines 192-298
```c
static THD_FUNCTION(timeout_thread, arg) {
    chRegSetThreadName("Timeout");

    for (;;) {
        // Check if timeout expired
        // Apply brake current if timeout
        // Check kill switch
        chThdSleepMilliseconds(10);  // 100 Hz
    }
}
```

**Purpose:** Watchdog for loss of control input
- **Priority:** HIGHPRIO (safety critical)
- **Stack:** 256 bytes
- **Rate:** 100 Hz
- **Responsibilities:**
  - Monitor last command timestamp
  - Apply brake current on timeout
  - Check hardware kill switch
  - BMS timeout monitoring

**Created:** `timeout_init():152`

---

### Application Threads

#### 9. PPM Thread (app_ppm.c)
```c
static THD_FUNCTION(ppm_thread, arg) {
    chRegSetThreadName("APP_PPM");

    for (;;) {
        // Read decoded PPM pulse width
        // Apply throttle curve and ramping
        // Send motor command
        chThdSleepMilliseconds(10);  // 100 Hz
    }
}
```

**Purpose:** RC PPM receiver input processing
- **Priority:** NORMALPRIO
- **Rate:** 100 Hz
- **Responsibilities:**
  - Read pulse width from servodec
  - Apply exponential curve
  - Ramp up/down smoothly
  - Cruise control logic

---

#### 10. ADC Thread (app_adc.c)
```c
static THD_FUNCTION(adc_thread, arg) {
    chRegSetThreadName("APP_ADC");

    for (;;) {
        // Read ADC voltages (throttle, brake)
        // Read button states
        // Apply curves and limits
        // Send motor command
        chThdSleepMilliseconds(10);  // 100 Hz
    }
}
```

**Purpose:** Analog throttle/brake input
- **Priority:** NORMALPRIO
- **Rate:** 100 Hz
- **Responsibilities:**
  - Read ADC1/ADC2 voltages
  - Read buttons (cruise, reverse)
  - Voltage-to-current mapping
  - Safe start logic

---

#### 11. PAS Thread (app_pas.c)
```c
static THD_FUNCTION(pas_thread, arg) {
    chRegSetThreadName("APP_PAS");

    for (;;) {
        // Measure pedal cadence (RPM)
        // Calculate assist current
        // Apply ramping
        chThdSleepMilliseconds(10);  // 100 Hz
    }
}
```

**Purpose:** E-bike pedal assist sensor
- **Priority:** NORMALPRIO
- **Rate:** 100 Hz
- **Responsibilities:**
  - Measure time between magnet pulses
  - Calculate pedal RPM
  - Map RPM to assist level
  - Ramp assist smoothly

---

#### 12. Nunchuk Thread (app_nunchuk.c)
```c
static THD_FUNCTION(nunchuk_thread, arg) {
    chRegSetThreadName("APP_NUNCHUK");

    for (;;) {
        // Read I2C data from Nunchuk
        // Decode joystick and buttons
        // Apply curves
        // Send motor command
        chThdSleepMilliseconds(10);  // 100 Hz
    }
}
```

**Purpose:** Wii Nunchuk joystick control
- **Priority:** NORMALPRIO
- **Rate:** 100 Hz
- **Responsibilities:**
  - I2C communication with Nunchuk
  - Joystick position decoding
  - Button state (C, Z)
  - Reverse direction logic

---

### Communication Threads

#### 13. USB Serial Thread (comm_usb.c)
```c
static THD_FUNCTION(usb_serial_thread, arg) {
    chRegSetThreadName("USB-serial");

    for (;;) {
        // Wait for USB data
        // Process packets
        // Send responses
        chThdSleepMilliseconds(1);
    }
}
```

**Purpose:** USB CDC (Virtual COM port) communication
- **Priority:** NORMALPRIO
- **Rate:** 1000 Hz (polling)
- **Responsibilities:**
  - USB packet reception
  - VESC protocol parsing
  - Command execution
  - Response transmission

---

#### 14. CAN RX Thread (comm_can.c)
```c
static THD_FUNCTION(canrx_thread, arg) {
    chRegSetThreadName("CAN RX");

    for (;;) {
        // Wait for CAN message
        // Parse CAN ID and data
        // Execute CAN command
        // Send response if needed
    }
}
```

**Purpose:** CAN bus message reception
- **Priority:** NORMALPRIO
- **Rate:** Event-driven (message arrival)
- **Responsibilities:**
  - Receive CAN frames
  - Decode VESC CAN protocol
  - Execute remote commands
  - Forward status messages

---

#### 15. CAN Status Send Thread (comm_can.c)
```c
static THD_FUNCTION(cancom_status_thread, arg) {
    chRegSetThreadName("CAN status");

    for (;;) {
        // Send periodic CAN status messages
        chThdSleepMilliseconds(10);  // 100 Hz (configurable)
    }
}
```

**Purpose:** Periodic CAN status transmission
- **Priority:** NORMALPRIO
- **Rate:** 10-1000 Hz (configurable)
- **Responsibilities:**
  - Send RPM, current, duty cycle
  - Send temperatures
  - Send energy consumption

---

### Other Subsystem Threads

#### 16. IMU Thread (imu/imu.c)
```c
static THD_FUNCTION(imu_thread, arg) {
    chRegSetThreadName("IMU");

    for (;;) {
        // Read accelerometer/gyroscope
        // Run AHRS algorithm (Mahony/Madgwick)
        // Update orientation quaternion
        chThdSleepMilliseconds(1);  // 1000 Hz
    }
}
```

**Purpose:** Inertial measurement unit processing
- **Priority:** NORMALPRIO
- **Rate:** 1000 Hz
- **Responsibilities:**
  - I2C/SPI sensor reading
  - Sensor fusion (accelerometer + gyroscope + magnetometer)
  - Quaternion/Euler angle calculation
  - Tilt compensation

---

#### 17. NRF Driver Thread (driver/nrf/nrf_driver.c)
```c
static THD_FUNCTION(nrf_thread, arg) {
    chRegSetThreadName("NRF");

    for (;;) {
        // Process NRF24L01+ wireless packets
        // ACK handling
        // Retry logic
        chThdSleepMilliseconds(2);  // 500 Hz
    }
}
```

**Purpose:** NRF24L01+ 2.4GHz wireless transceiver
- **Priority:** NORMALPRIO
- **Rate:** 500 Hz
- **Responsibilities:**
  - SPI communication with NRF chip
  - Packet transmission/reception
  - Pairing and channel hopping
  - RSSI monitoring

---

#### 18. Encoder Thread (encoder/encoder.c)
```c
static THD_FUNCTION(encoder_thread, arg) {
    chRegSetThreadName("Encoder");

    for (;;) {
        // Read encoder position (SPI/ABI)
        // Apply sin/cos correction
        // Update position estimate
        chThdSleepMilliseconds(1);  // 1000 Hz
    }
}
```

**Purpose:** Position sensor reading (for some encoder types)
- **Priority:** NORMALPRIO
- **Rate:** 1000 Hz
- **Responsibilities:**
  - SPI reading for AS5047, MT6816
  - ABI quadrature decoding
  - Sin/cos correction table application
  - Index pulse alignment

---

#### 19. LispBM Thread (lispBM/lispif.c)
```c
static THD_FUNCTION(eval_thread, arg) {
    chRegSetThreadName("Lisp");

    for (;;) {
        // Execute Lisp bytecode
        // Handle extensions (motor control, sensors)
        // GC if needed
        lbm_run_eval();
    }
}
```

**Purpose:** LispBM scripting engine execution
- **Priority:** NORMALPRIO
- **Rate:** Continuous (no sleep, yields via eval quota)
- **Responsibilities:**
  - Lisp code evaluation
  - Extension function calls
  - Garbage collection
  - Event handling

---

## Thread Priority Summary

```
Priority 255 (EMERGENCY_PRIO)
    └─ (Reserved for critical ISR threads)

Priority 127 (HIGHPRIO)
    └─ Timeout Thread            [Safety critical]

Priority 64 (NORMALPRIO) - Most threads
    ├─ LED Thread                [Visual feedback]
    ├─ Periodic Thread           [Background tasks]
    ├─ Timer Thread              [Motor control periodic]
    ├─ Sample Send Thread        [Debug telemetry]
    ├─ Fault Stop Thread         [Emergency shutdown]
    ├─ Statistics Thread         [Performance tracking]
    ├─ PPM Thread                [RC input]
    ├─ ADC Thread                [Analog input]
    ├─ PAS Thread                [Pedal assist]
    ├─ Nunchuk Thread            [Joystick input]
    ├─ USB Serial Thread         [USB communication]
    ├─ CAN RX Thread             [CAN reception]
    ├─ CAN Status Thread         [CAN transmission]
    ├─ IMU Thread                [Sensor fusion]
    ├─ NRF Thread                [Wireless]
    ├─ Encoder Thread            [Position sensing]
    └─ LispBM Thread             [Scripting]

Priority 2 (LOWPRIO)
    └─ Flash Integrity Thread    [Background CRC check]

Priority 1 (IDLEPRIO)
    └─ Idle Thread               [CPU usage measurement, sleep]
```

---

## Synchronization Primitives

### 1. Events (Most Common in VESC)

**Event Flags:** Wake a thread when condition occurs

```c
// Thread declaration
static thread_t *sample_send_tp;

// In thread function
sample_send_tp = chThdGetSelfX();  // Get own thread pointer

for (;;) {
    chEvtWaitAny((eventmask_t) 1);  // Wait for event bit 0

    // Process event
}

// From another thread or ISR
chEvtSignal(sample_send_tp, (eventmask_t) 1);  // Wake the thread
```

**Use Cases in VESC:**
- Fault detection → Fault Stop Thread
- Sampling complete → Sample Send Thread
- Packet received → USB/CAN threads

---

### 2. Mutexes (Mutual Exclusion)

**Purpose:** Protect shared data from concurrent access

```c
static mutex_t config_mutex;

// Initialize
chMtxObjectInit(&config_mutex);

// Thread 1
chMtxLock(&config_mutex);
// Access shared configuration
chMtxUnlock(&config_mutex);

// Thread 2
chMtxLock(&config_mutex);
// Access shared configuration
chMtxUnlock(&config_mutex);
```

**Priority Inheritance:** If high-priority thread waits on mutex held by low-priority thread, low-priority thread inherits high priority temporarily (prevents priority inversion).

**Use Cases in VESC:**
- Configuration structures
- SPI/I2C bus access
- Statistics counters

---

### 3. Semaphores (Counting)

**Purpose:** Resource availability or signaling

```c
static semaphore_t buffer_sem;

// Initialize with count
chSemObjectInit(&buffer_sem, 10);  // 10 available slots

// Consumer thread
chSemWait(&buffer_sem);  // Decrement, block if zero
// Use resource
chSemSignal(&buffer_sem);  // Increment

// Producer thread
if (chSemWaitTimeout(&buffer_sem, MS2ST(100)) == MSG_OK) {
    // Got slot within 100ms
} else {
    // Timeout
}
```

**Use Cases:**
- Buffer management
- Resource pools
- Inter-thread signaling

---

### 4. Virtual Timers

**Purpose:** One-shot or periodic callbacks (without dedicated thread)

```c
static virtual_timer_t output_vt;

// Callback function
static void output_vt_cb(void *arg) {
    output_disabled_now = false;
}

// Initialize
chVTObjectInit(&output_vt);

// Set timer (one-shot)
chVTSet(&output_vt, MS2ST(500), output_vt_cb, NULL);  // Fire in 500ms

// Cancel timer
chVTReset(&output_vt);
```

**Use Cases in VESC:**
- Output disable timeout (app.c)
- Delayed operations without threads
- Periodic callbacks

---

### 5. Mailboxes (Message Passing)

**Purpose:** Send data between threads (not heavily used in VESC)

```c
#define MB_SIZE 8
static msg_t mb_buffer[MB_SIZE];
static mailbox_t mb;

// Initialize
chMBObjectInit(&mb, mb_buffer, MB_SIZE);

// Sender
msg_t msg = 0x1234;
chMBPost(&mb, msg, TIME_INFINITE);

// Receiver
msg_t msg;
chMBFetch(&mb, &msg, TIME_INFINITE);
```

---

## Interrupt Integration

### ISR to Thread Communication

**Problem:** Can't do complex work in ISR (must be fast)

**Solution:** ISR signals thread via event

```c
// High-priority ISR
CH_IRQ_HANDLER(ADC_IRQHandler) {
    CH_IRQ_PROLOGUE();

    // Quick: Read ADC value, store in buffer
    adc_buffer[adc_index++] = ADC1->DR;

    // Signal thread (no blocking calls!)
    chSysLockFromISR();
    chEvtSignalI(adc_thread_tp, (eventmask_t) 1);
    chSysUnlockFromISR();

    CH_IRQ_EPILOGUE();
}

// Normal-priority thread
static THD_FUNCTION(adc_thread, arg) {
    for (;;) {
        chEvtWaitAny((eventmask_t) 1);  // Wait for ISR signal

        // Process buffer (can take time)
        process_adc_samples(adc_buffer, adc_index);
    }
}
```

**Key ISRs in VESC:**
1. **TIM1 Update ISR:** PWM cycle complete → `mc_interface_mc_timer_isr()`
2. **ADC Injection ISR:** Current samples ready → FOC calculations
3. **CAN RX ISR:** CAN message received → Signal CAN thread
4. **USB ISR:** USB data available → Signal USB thread

---

### Motor Control ISR Flow

**Most Critical Path:**

```
TIM1 PWM Timer (10-40 kHz)
    │
    ├─► TIM1_UP_IRQHandler (ISR)
    │       │
    │       └─► mc_interface_mc_timer_isr()
    │               │
    │               ├─► Update voltage filters (fast)
    │               ├─► Check absolute current limit (fast)
    │               ├─► Check gate driver fault (fast)
    │               ├─► Accumulate energy counters (fast)
    │               ├─► Voltage fault integration (fast)
    │               └─► Signal Fault Stop Thread if fault (event)
    │
    └─► ADC_IRQHandler (ADC complete, triggered by TIM1)
            │
            └─► mcpwm_foc_adc_int_handler()
                    │
                    ├─► Read ADC values (phase currents)
                    ├─► Clarke transform (fast math)
                    ├─► Park transform (fast math)
                    ├─► PI current control (fast math)
                    ├─► Inverse Park (fast math)
                    ├─► Space Vector Modulation (fast math)
                    └─► Update TIM1 CCR registers (PWM duty)

ISR Duration: 5-20 µs (must complete before next PWM cycle!)
```

**Why ISR instead of thread?**
- **Latency:** ISR preempts everything immediately
- **Jitter:** ISR timing very consistent (< 1µs jitter)
- **Frequency:** 40 kHz too fast for thread switching overhead

---

## Motor Control Thread Architecture

### Thread Interaction Diagram

```
┌───────────────────────────────────────────────────────────┐
│                    MAIN THREAD                            │
│  • System initialization                                  │
│  • Create all threads                                     │
│  • Start scheduler                                        │
└───────────────────────────────────────────────────────────┘
                        │
        ┌───────────────┼───────────────┐
        │               │               │
        ▼               ▼               ▼
┌──────────────┐ ┌──────────────┐ ┌──────────────┐
│ APP THREADS  │ │MOTOR CONTROL │ │   COMM       │
│              │ │   THREADS    │ │  THREADS     │
├──────────────┤ ├──────────────┤ ├──────────────┤
│ PPM Thread   │ │ Timer Thread │ │ USB Thread   │
│ ADC Thread   │ │ Sample Send  │ │ CAN RX Thread│
│ PAS Thread   │ │ Fault Stop   │ │ CAN TX Thread│
│ Nunchuk      │ │ Statistics   │ └──────────────┘
└──────────────┘ └──────────────┘
        │               │               │
        │               │               │
        └───────────────┼───────────────┘
                        │
                        ▼
             ┌────────────────────┐
             │ mc_interface_set_* │
             │  (Motor Commands)  │
             └────────────────────┘
                        │
                        ▼
             ┌────────────────────┐
             │  mcpwm_foc_set_*   │
             │  (FOC Implementation)|
             └────────────────────┘
                        │
                        ▼
        ┌───────────────────────────────┐
        │  TIM1 ISR (40 kHz)            │
        │  • Read ADC                   │
        │  • FOC algorithm              │
        │  • Update PWM                 │
        │  • mc_interface_mc_timer_isr()│
        └───────────────────────────────┘
                        │
                        ▼
             ┌────────────────────┐
             │  Hardware (Motor)  │
             └────────────────────┘
```

### Data Flow Example: PPM Control

```
1. Servo decoder captures PPM pulse (hardware timer)
        │
        ▼
2. PPM Thread wakes up (100 Hz)
        │
        ├─ Read pulse width: 1.5ms
        ├─ Map to [-1, +1]: 0.0 (center)
        ├─ Apply exponential curve
        ├─ Check safe start
        └─ Apply ramping
        │
        ▼
3. Call mc_interface_set_duty(0.5)
        │
        ├─ Check if input allowed (mc_interface_try_input)
        ├─ Apply direction multiplier
        └─ Call mcpwm_foc_set_duty(0.5)
        │
        ▼
4. mcpwm_foc_set_duty() stores target
        │
        ▼
5. TIM1 ISR fires (40 kHz)
        │
        ├─ ADC reads phase currents
        ├─ FOC algorithm calculates voltage
        ├─ Ramps toward duty target
        └─ Updates PWM registers
        │
        ▼
6. Motor turns at 50% voltage
```

### Thread Safety in VESC

**Problem:** Multiple threads calling motor control functions

**Solution 1: Thread-Local Motor Selection**
```c
// Each thread selects its motor
mc_interface_select_motor_thread(1);  // Control motor 1
mc_interface_set_current(10.0);

// Internal: motor_now() reads thread-local variable
static volatile motor_if_state_t *motor_now(void) {
    return chThdGetSelfX()->motor_selected == 1 ? &m_motor_1 : &m_motor_2;
}
```

**Solution 2: Lock System**
```c
// Lock motor control (safety system can lock)
mc_interface_lock();

// All control commands ignored unless:
mc_interface_lock_override_once();  // Allow one command
mc_interface_set_current(5.0);      // This works
mc_interface_set_current(10.0);     // Blocked again

mc_interface_unlock();  // Unlock permanently
```

**Solution 3: Atomic Operations**
```c
// In ISR: Use ChibiOS lock/unlock
utils_sys_lock_cnt();   // Disable interrupts, count nesting
// Critical section
utils_sys_unlock_cnt(); // Re-enable if count reaches 0
```

---

## Learning Resources

### Official ChibiOS Documentation

1. **ChibiOS/RT Documentation**
   - http://www.chibios.org/dokuwiki/doku.php?id=chibios:documentation:start
   - Official manual with all APIs

2. **ChibiOS Book (Free PDF)**
   - http://www.chibios.org/dokuwiki/doku.php?id=chibios:book:start
   - Comprehensive guide to RTOS concepts

3. **API Reference**
   - http://www.chibios.org/dokuwiki/doku.php?id=chibios:documentation:rt
   - Function-by-function documentation

### VESC-Specific Resources

1. **VESC Forums**
   - https://vesc-project.com/forum
   - Community discussions about firmware

2. **Benjamin Vedder's GitHub**
   - https://github.com/vedderb/bldc
   - Source code with comments

3. **VESC Tool**
   - Study the Lisp examples showing motor control from scripts

### Recommended Reading Order

1. **Start Here:** ChibiOS Concepts (threads, scheduling)
2. **Then:** Synchronization primitives (events, mutexes)
3. **Next:** VESC thread structure (this document)
4. **Finally:** Deep dive into specific subsystems (FOC, applications)

### Example Code to Study

**Simple Thread Example:**
```c
// applications/app_ppm.c - Good example of periodic thread
```

**Event-Driven Example:**
```c
// mc_interface.c - fault_stop_thread uses events
```

**ISR Integration:**
```c
// mcpwm_foc.c - ADC ISR to FOC algorithm
```

**Multi-Thread Coordination:**
```c
// comm_can.c - RX and TX threads with mailbox
```

---

## Common ChibiOS Patterns in VESC

### Pattern 1: Periodic Task
```c
static THD_FUNCTION(my_thread, arg) {
    chRegSetThreadName("MyThread");

    for (;;) {
        // Do work
        do_periodic_task();

        // Sleep for period
        chThdSleepMilliseconds(10);  // 100 Hz
    }
}
```

### Pattern 2: Event-Driven Task
```c
static thread_t *event_thread_tp;

static THD_FUNCTION(event_thread, arg) {
    event_thread_tp = chThdGetSelfX();

    for (;;) {
        chEvtWaitAny(ALL_EVENTS);  // Block until signaled

        // Process event
        handle_event();
    }
}

// Somewhere else
chEvtSignal(event_thread_tp, (eventmask_t) 1);
```

### Pattern 3: Producer-Consumer
```c
// Producer thread
for (;;) {
    data_t item = produce_data();
    chMBPost(&mailbox, (msg_t)item, TIME_INFINITE);
}

// Consumer thread
for (;;) {
    msg_t item;
    chMBFetch(&mailbox, &item, TIME_INFINITE);
    consume_data((data_t)item);
}
```

### Pattern 4: Watchdog
```c
static THD_FUNCTION(watchdog_thread, arg) {
    systime_t last_ping = chVTGetSystemTime();

    for (;;) {
        if (UTILS_AGE_S(last_ping) > 1.0) {
            // Timeout! Take action
            emergency_stop();
        }

        chThdSleepMilliseconds(10);
    }
}

// Main control thread periodically pings
void keep_alive() {
    last_ping = chVTGetSystemTime();
}
```

---

## Next Steps

To fully understand VESC firmware, study in this order:

1. **✅ This document** - ChibiOS basics and thread structure
2. **TODO: FOC Control Thread** - How mcpwm_foc.c implements FOC algorithm in ISR
3. **TODO: Application Threads** - Deep dive into PPM, ADC, PAS threads
4. **TODO: Communication Protocol** - USB/CAN thread interaction
5. **TODO: Configuration Storage** - Flash write synchronization

---

**Summary:**

- **ChibiOS** is the RTOS providing preemptive multithreading
- **~19 threads** run concurrently in VESC firmware
- **Priority-based scheduling** ensures critical tasks run first
- **Events and mutexes** coordinate thread execution
- **ISR → Thread** pattern separates fast (ISR) and slow (thread) work
- **Motor control** runs mostly in 40 kHz ISR, with helper threads
- **Applications** read input at 100 Hz and send commands to motor control

This architecture allows VESC to:
- Control motor at 40 kHz (FOC loop)
- Process user input at 100 Hz (applications)
- Communicate via USB/CAN (event-driven)
- Monitor safety conditions (timeout, faults)
- Log data and run scripts (background threads)

All while maintaining real-time performance and deterministic response!
