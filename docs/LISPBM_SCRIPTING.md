# LispBM Scripting Engine Documentation

## Overview

LispBM is an embedded Lisp interpreter integrated into VESC firmware, enabling powerful runtime customization and scripting without recompiling firmware. Written by Joel Svensson, it provides a sandboxed environment for user applications.

**Location:** `lispBM/`

### Why LispBM in VESC?

**Traditional Approach (Without LispBM):**
- Modify C source code
- Recompile entire firmware
- Flash new firmware to VESC
- Risk bricking device with bugs
- Difficult for non-programmers

**With LispBM:**
- Write scripts in Lisp language
- Upload to VESC via VESC Tool
- Live testing and debugging
- Sandboxed execution (can't brick device)
- Hot-reload without firmware reflashing
- Access to all VESC functions and sensors

### Key Features

1. **Development in VESC Tool**
   - Live REPL (Read-Eval-Print Loop)
   - Variable monitoring and plotting
   - CPU and memory usage monitoring
   - Syntax highlighting

2. **Sandboxed Execution**
   - Runs in isolated environment
   - Cannot crash main firmware
   - Heap and stack limits enforced
   - Automatic recovery on errors

3. **Persistent Storage**
   - Scripts stored in flash memory
   - Auto-start on boot
   - Survive power cycles

4. **Full VESC Access**
   - Motor control commands
   - Sensor readings (IMU, encoders, ADC)
   - CAN bus communication
   - GPIO control
   - Configuration access

5. **Real-Time Capable**
   - Low overhead (~1-5% CPU typical)
   - Deterministic garbage collection
   - Suitable for control loops

---

## Quick Start

### Hello World Example

```lisp
(print "Hello from VESC LispBM!")
```

### Reading Battery Voltage

```lisp
(defun read-voltage ()
    (let ((v-bat (get-vin)))
        (print (str-from-n v-bat "Battery: %.2f V"))))

; Run once
(read-voltage)
```

### Blinking LED

```lisp
(defun blink-forever ()
    (loopwhile t
        (progn
            (set-gpio 1 1)  ; LED on
            (sleep 0.5)
            (set-gpio 1 0)  ; LED off
            (sleep 0.5))))

; Start background task
(spawn blink-forever)
```

### Simple Speed Controller

```lisp
(defun speed-control ()
    (loopwhile t
        (progn
            ; Read ADC throttle (0.0 to 1.0)
            (let ((throttle (get-adc 1)))
                ; Set speed (ERPM)
                (set-rpm (* throttle 5000.0)))
            (sleep 0.01))))  ; 100Hz update rate

(spawn speed-control)
```

---

## Documentation Structure

The lispBM system has extensive built-in documentation:

### Core Language Reference

**File:** `lispBM/lispBM/doc/lbmref.md`

Complete language reference covering:
- Data types (numbers, symbols, lists, arrays)
- Control structures (if, cond, match, loops)
- Functions and lambda
- Let bindings and closures
- Built-in functions (math, list operations, etc.)
- Memory management
- Concurrency

**Essential Reading** for understanding Lisp syntax and semantics.

### VESC-Specific Extensions

**File:** `lispBM/README.md` (this repository's copy)

Comprehensive documentation (190KB+) of VESC-specific functions:

**Major Categories:**
- Motor Control (`set-current`, `set-duty`, `set-rpm`, `set-pos`, `set-brake`)
- Sensor Reading (`get-vin`, `get-temp-fet`, `get-temp-motor`, `get-duty`, `get-rpm`)
- IMU Access (`get-imu-rpy`, `get-imu-quat`, `get-imu-acc`, `get-imu-gyro`, `get-imu-mag`)
- Encoder Reading (`get-encoder`, `set-encoder`)
- GPIO Control (`gpio-configure`, `gpio-write`, `gpio-read`, `gpio-analog-read`)
- CAN Bus (`canset-current`, `canset-duty`, `can-list-devs`)
- Events (`register-event-handler`)
- Configuration Access (`conf-get`, `conf-set`)
- Math and Utils (`sin`, `cos`, `sqrt`, `abs`, filtering functions)
- UART/I2C/SPI Communication
- Logging and Plotting (`log-start`, `log-config-field`, `send-data`)

### Dynamic Libraries

**File:** `lispBM/lispBM/doc/dynref.md`

Additional libraries loaded on-demand:
- Regular expressions
- String operations
- Array operations
- Hash tables
- Persistent storage

### Gotchas and Caveats

**File:** `lispBM/lispBM/doc/gotchas.md`

Common pitfalls and important notes about LispBM behavior.

---

## Architecture

### Integration Layer

```
┌─────────────────────────────────────────┐
│          VESC Tool (PC)                 │
│    - Code Editor                        │
│    - REPL Console                       │
│    - Live Monitoring                    │
└─────────────┬───────────────────────────┘
              │ USB/CAN
┌─────────────▼───────────────────────────┐
│          VESC Firmware                  │
│  ┌───────────────────────────────────┐  │
│  │  lispif.c - Integration Layer     │  │
│  │  - Command processing             │  │
│  │  - Event routing                  │  │
│  │  - Thread management              │  │
│  └──────────┬────────────────────────┘  │
│             │                            │
│  ┌──────────▼───────────────────────┐   │
│  │  lispif_vesc_extensions.c        │   │
│  │  - Motor control bindings        │   │
│  │  - Sensor access bindings        │   │
│  │  - CAN/GPIO/etc. bindings        │   │
│  └──────────┬───────────────────────┘   │
│             │                            │
│  ┌──────────▼───────────────────────┐   │
│  │  lispBM Core Interpreter         │   │
│  │  - Parser                        │   │
│  │  - Evaluator                     │   │
│  │  - Garbage Collector             │   │
│  │  - Heap/Stack Manager            │   │
│  └──────────────────────────────────┘   │
└─────────────────────────────────────────┘
```

### Key Files

**Integration Layer:**
- `lispif.c/h` - Main integration interface, initialization, command processing
- `lispif_vesc_extensions.c` - All VESC-specific function bindings (196KB!)
- `lispif_c_lib.c` - C library integration
- `lbm_vesc_utils.c/h` - Utility functions for VESC extensions

**Core LispBM (in lispBM/lispBM/ subdirectory):**
- Upstream LispBM interpreter
- Maintained by Joel Svensson
- Generic embedded Lisp implementation

### Memory Management

**Heap:**
- Stores Lisp objects (cons cells, arrays, symbols)
- Default size: configurable (typically 16-64KB)
- Garbage collected automatically
- Monitors usage via VESC Tool

**Stack:**
- Function call stack
- Default size: 256-1024 words
- Overflow causes exception

**Symbol Table:**
- Stores symbol names
- Fixed size at initialization

**Code Storage:**
- Lisp code stored in flash memory
- Loaded on boot
- Max size: ~64KB (depends on firmware variant)

---

## Practical Examples

### 1. Soft Start

Gradually ramp current to prevent jerky starts:

```lisp
(defun soft-start (target-current ramp-time)
    (let ((steps 100)
          (step-time (/ ramp-time steps))
          (current-step (/ target-current steps)))
        (looprange i 0 steps
            (progn
                (set-current (* i current-step))
                (sleep step-time)))))

; Ramp to 30A over 2 seconds
(soft-start 30.0 2.0)
```

### 2. Temperature-Based Derating

Reduce current limit when MOSFETs get hot:

```lisp
(defun temp-monitor ()
    (loopwhile t
        (let ((temp-fet (get-temp-fet)))
            (if (> temp-fet 80.0)
                ; Over 80°C, reduce to 50% current
                (set-current-rel 0.5)
                ; Under 80°C, allow full current
                (set-current-rel 1.0)))
        (sleep 0.1)))  ; Check every 100ms

(spawn temp-monitor)
```

### 3. Battery Voltage Monitor

Log and alert on low battery:

```lisp
(def low-voltage 42.0)   ; 10S LiPo minimum
(def alert-active 0)

(defun voltage-monitor ()
    (loopwhile t
        (let ((v-bat (get-vin)))
            (if (and (< v-bat low-voltage) (= alert-active 0))
                (progn
                    (set alert-active 1)
                    (print "WARNING: Low battery!")
                    (set-current 0.0))  ; Stop motor
                nil))
        (sleep 1.0)))  ; Check every second

(spawn voltage-monitor)
```

### 4. Custom Cruise Control

Maintain constant speed using PID controller:

```lisp
(def target-rpm 3000.0)
(def kp 0.1)
(def ki 0.01)
(def integral 0.0)

(defun cruise-control ()
    (loopwhile t
        (let* ((current-rpm (get-rpm))
               (error (- target-rpm current-rpm))
               (new-integral (+ integral (* error 0.01))))  ; dt = 0.01s
            (progn
                (set integral new-integral)
                (let ((output (+ (* kp error) (* ki integral))))
                    (set-current output))))
        (sleep 0.01)))  ; 100Hz control loop

(spawn cruise-control)
```

### 5. IMU-Based Tilt Detection

Detect if vehicle tips over:

```lisp
(defun tilt-monitor ()
    (loopwhile t
        (let* ((rpy (get-imu-rpy))
               (roll (first rpy))
               (pitch (second rpy)))
            (if (or (> (abs roll) 45.0) (> (abs pitch) 45.0))
                (progn
                    (print "TILT DETECTED - STOPPING")
                    (set-brake 50.0))
                nil))
        (sleep 0.05)))  ; 20Hz monitoring

(spawn tilt-monitor)
```

### 6. CAN Bus Multi-Motor Control

Synchronize multiple VESCs:

```lisp
; List of VESC CAN IDs
(def vesc-ids '(10 11 12))

; Set same current to all VESCs
(defun set-all-current (current)
    (map (fn (id) (canset-current id current)) vesc-ids))

; Main control loop
(defun multi-motor-control ()
    (loopwhile t
        (let ((throttle (get-adc 1)))
            (set-all-current (* throttle 40.0)))
        (sleep 0.01)))

(spawn multi-motor-control)
```

### 7. Data Logging

Log multiple variables for analysis:

```lisp
(defun setup-logging ()
    (progn
        (log-config-field 0 "rpm" 1)           ; Field 0: RPM
        (log-config-field 1 "current" 1)      ; Field 1: Current
        (log-config-field 2 "voltage" 1)      ; Field 2: Voltage
        (log-config-field 3 "temp_fet" 1)     ; Field 3: FET temp
        (log-start 100 0.01)))                ; 100 samples at 10ms interval

(defun log-data ()
    (loopwhile t
        (progn
            (send-data 0 (get-rpm))
            (send-data 1 (get-current))
            (send-data 2 (get-vin))
            (send-data 3 (get-temp-fet)))
        (sleep 0.01)))

(setup-logging)
(spawn log-data)
```

### 8. Event-Driven Programming

Respond to specific events:

```lisp
; Event handler for CAN frames
(defun can-handler (can-id data)
    (match can-id
        (0x10 (progn
            (print "Received from ID 16")
            (print data)))
        (0x20 (set-current (bufget-f32 data 0)))
        (_ nil)))  ; Default case

; Register event handler
(event-register-handler (fn (event data)
    (match event
        (event-can-rx (can-handler (first data) (second data)))
        (_ nil))))
```

---

## Best Practices

### 1. Use Background Tasks (spawn)

**Good:**
```lisp
(defun my-loop ()
    (loopwhile t
        (progn
            ; Do work
            (sleep 0.01))))

(spawn my-loop)  ; Runs in background
```

**Bad:**
```lisp
(loopwhile t     ; Blocks REPL and other scripts!
    (progn
        ; Do work
        (sleep 0.01)))
```

### 2. Always Include Sleep in Loops

**Good:**
```lisp
(loopwhile t
    (progn
        (do-something)
        (sleep 0.001)))  ; 1ms = 1000Hz max
```

**Bad:**
```lisp
(loopwhile t
    (do-something))  ; 100% CPU usage!
```

### 3. Handle Errors Gracefully

```lisp
(defun safe-read-sensor ()
    (let ((value (trap (read-sensor))))
        (if (eq value 'type-error)
            (progn
                (print "Sensor read failed")
                0.0)  ; Default value
            value)))
```

### 4. Limit Print Statements

Excessive printing slows down execution and floods console:

```lisp
; Bad: Print every iteration
(loopwhile t
    (progn
        (print (get-rpm))  ; Every 1ms!
        (sleep 0.001)))

; Good: Print occasionally
(def print-counter 0)
(loopwhile t
    (progn
        (set print-counter (+ print-counter 1))
        (if (= (mod print-counter 1000) 0)
            (print (get-rpm)))  ; Every 1000 iterations
        (sleep 0.001)))
```

### 5. Monitor Memory Usage

```lisp
; Check heap usage
(print (str-from-n (mem-num-free) "Free heap: %d words"))

; Check stack depth
(print (str-from-n (stack-size) "Stack depth: %d"))
```

### 6. Clean Shutdown

Allow graceful termination:

```lisp
(def running 1)

(defun my-app ()
    (loopwhile (= running 1)
        (progn
            ; Do work
            (sleep 0.01))))

; To stop:
; (set running 0)
```

---

## Performance Considerations

### Typical Performance

- **Simple scripts:** <1% CPU usage
- **Control loops (100Hz):** 1-3% CPU
- **Heavy processing:** 5-10% CPU
- **Maximum sustainable:** ~20% CPU (leaves headroom for motor control)

### Optimization Tips

1. **Use Local Variables**
   ```lisp
   ; Faster
   (let ((rpm (get-rpm)))
       (if (> rpm 1000) ...))

   ; Slower (calls get-rpm twice)
   (if (> (get-rpm) 1000) ...)
   ```

2. **Minimize Allocations**
   - Reuse variables instead of creating new ones
   - Avoid unnecessary list operations in tight loops

3. **Batch Operations**
   ```lisp
   ; Better: One CAN message
   (can-send-extended 0x100 (list 1 2 3 4 5 6 7 8))

   ; Worse: Eight CAN messages
   (map (fn (x) (can-send-byte 0x100 x)) (list 1 2 3 4 5 6 7 8))
   ```

4. **Appropriate Sleep Values**
   - Motor control: 0.001-0.01s (1kHz-100Hz)
   - Monitoring: 0.1-1.0s
   - Don't sleep less than needed

---

## Debugging

### Using REPL

VESC Tool provides live REPL:

```lisp
; Define function
> (defun test () (print "test"))
> test

; Call it
> (test)
> "test"

; Inspect variables
> my-variable
> 42

; Check if symbol exists
> my-func
> unbound-symbol  ; Not defined

; Evaluate expressions
> (+ 1 2 3)
> 6
```

### Print Debugging

```lisp
(defun debug-values ()
    (progn
        (print (str-from-n (get-rpm) "RPM: %.1f"))
        (print (str-from-n (get-duty) "Duty: %.3f"))
        (print (str-from-n (get-current) "Current: %.2f A"))))
```

### Error Messages

Common errors and solutions:

**"Out of memory"**
- Reduce heap allocations
- Free unused variables
- Simplify data structures

**"Stack overflow"**
- Reduce recursion depth
- Use iteration instead of recursion
- Check for infinite loops

**"Type error"**
- Check function argument types
- Use `to-i`, `to-f`, `to-str` for conversion

**"Evaluation error"**
- Syntax error in code
- Undefined function or variable
- Check parentheses balance

---

## Integration with VESC Tool

### Uploading Code

1. Open VESC Tool
2. Connect to VESC
3. Navigate to "Lisp" tab
4. Write or load script
5. Click "Upload" to send to VESC
6. Click "Run" to execute

### Live Monitoring

VESC Tool can plot variables in real-time:

```lisp
(log-start 500 0.01)  ; 500 samples at 10ms

; In loop:
(send-data 0 (get-rpm))
(send-data 1 (get-current))
```

Then plot fields 0 and 1 in VESC Tool.

### Persistent Storage

Code is stored in flash and survives reboots:

- "Upload" - Sends code but doesn't save
- "Write to Flash" - Saves code to non-volatile memory
- "Erase" - Removes code from flash

### Remote REPL

Access REPL over CAN bus or Bluetooth from VESC Tool.

---

## Use Cases

### Custom Applications

1. **E-Bike Controller**
   - PAS (Pedal Assist) with custom curves
   - Throttle/brake blending
   - Regenerative braking logic

2. **Robotics**
   - Multi-axis coordination
   - Sensor fusion
   - Autonomous navigation

3. **UAV/Drones**
   - Motor synchronization
   - Battery management
   - Telemetry

4. **Electric Vehicles**
   - Traction control
   - Cruise control
   - Regenerative braking
   - Dashboard integration

5. **Custom Protocols**
   - Proprietary CAN protocols
   - Custom remote controls
   - Integration with external systems

6. **Testing and Calibration**
   - Automated test sequences
   - Parameter sweeps
   - Data collection

---

## Safety Considerations

### Sandboxing Limitations

While LispBM is sandboxed, it can still:
- Command full motor power
- Access all hardware
- Modify configuration
- Communicate on CAN bus

**Always test carefully!**

### Safe Development

1. **Start with Low Limits**
   ```lisp
   (conf-set 'l_current_max 10.0)  ; Limit to 10A while testing
   ```

2. **Implement Watchdogs**
   ```lisp
   (defun watchdog ()
       (if (> (get-temp-fet) 90.0)
           (set-current 0.0)
           nil))
   ```

3. **Test Without Load First**
   - Verify logic with motor disconnected
   - Use REPL to step through code
   - Monitor variables before enabling power

4. **Emergency Stop**
   ```lisp
   (def e-stop 0)
   (if (= e-stop 1)
       (progn
           (set-brake 100.0)
           (yield)))
   ```

5. **Gradual Testing**
   - Test at low speeds first
   - Increase current limits gradually
   - Monitor temperatures

---

## Summary

LispBM provides powerful runtime customization for VESC:

**Advantages:**
- No firmware recompilation needed
- Safe sandboxed environment
- Live debugging and testing
- Persistent across reboots
- Access to all VESC features
- Suitable for control loops

**Limitations:**
- Lower performance than C code
- Limited memory (heap/stack)
- Learning curve for Lisp syntax

**Best For:**
- Custom control algorithms
- Application-specific logic
- Prototyping and testing
- Integration with external systems
- Advanced users needing customization

**Documentation:**
- Core Language: `lispBM/lispBM/doc/lbmref.md`
- VESC Extensions: `lispBM/README.md`
- Dynamic Libraries: `lispBM/lispBM/doc/dynref.md`
- Gotchas: `lispBM/lispBM/doc/gotchas.md`

**For the complete VESC LispBM reference with all available functions, see `lispBM/README.md` in this repository.**

The LispBM integration transforms VESC from a motor controller into a programmable embedded platform for advanced motor control applications.
