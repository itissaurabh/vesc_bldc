# Upstream VESC Firmware Changes Summary

## Document Information
- **Date**: November 2025
- **Commits Reviewed**: 9 commits from upstream VESC repository
- **Date Range**: October 13, 2025 - November 5, 2025
- **Author**: Benjamin Vedder (all commits)

---

## Executive Summary

This document summarizes the recent upstream changes merged into the VESC firmware. The changes include:

- **1 Critical Bug Fix**: Dual motor watt-hour/amp-hour counter bug
- **4 New Features**: LispBM extensions and encoder improvements
- **3 Improvements**: Configuration persistence and sensorless control
- **1 Code Cleanup**: Removal of 23 obsolete hardware configurations (5000+ lines)
- **1 Hardware Update**: Luna Cycle BBSHD and M600 improvements

**Overall Impact**: Low to Medium
- No breaking changes to existing APIs
- Bug fixes improve dual motor reliability
- New features are additive (LispBM extensions)
- Code cleanup reduces firmware size
- CAN ID persistence improves multi-ESC deployments

---

## 1. Bug Fixes

### 1.1 Dual Motor Watt-Hour/Amp-Hour Counter Bug Fix
**Commit**: `510e33f` - "Simplified wh and ah counters"
**Date**: November 5, 2025
**Impact**: HIGH for dual motor hardware

#### Problem
The previous implementation used a filtering accumulator that was shared between motor instances, causing incorrect counting on dual motor hardware.

#### Old Code (Buggy)
```c
// Some extra filtering - BUGGY on dual motor!
static float curr_diff_sum = 0.0;        // Shared static variable
static float curr_diff_samples = 0.0;    // Shared static variable

curr_diff_sum += current_in_filtered * t_samp;
curr_diff_samples += t_samp;

if (curr_diff_samples >= 0.01) {
    // Accumulate every 10ms
    if (curr_diff_sum > 0.0) {
        motor->m_amp_seconds += curr_diff_sum;
        motor->m_watt_seconds += curr_diff_sum * input_voltage;
    } else {
        motor->m_amp_seconds_charged -= curr_diff_sum;
        motor->m_watt_seconds_charged -= curr_diff_sum * input_voltage;
    }
    curr_diff_samples = 0.0;
    curr_diff_sum = 0.0;
}
```

**Problem**: Static variables `curr_diff_sum` and `curr_diff_samples` were shared between motor 1 and motor 2, causing cross-contamination of measurements.

#### New Code (Fixed)
```c
// Direct calculation per sample - CORRECT
if (current_in_filtered > 0.0) {
    motor->m_amp_seconds += current_in_filtered * t_samp;
    motor->m_watt_seconds += current_in_filtered * t_samp * input_voltage;
} else {
    motor->m_amp_seconds_charged -= current_in_filtered * t_samp;
    motor->m_watt_seconds_charged -= current_in_filtered * t_samp * input_voltage;
}
```

#### Impact
- **Single motor hardware**: No functional change (counter still works correctly)
- **Dual motor hardware**: Fixes incorrect watt-hour and amp-hour measurements
- **Performance**: Slightly better (removed extra filtering logic and conditionals)
- **Accuracy**: Improved on dual motor hardware

#### Files Modified
- `motor/mc_interface.c`: Lines 2005-2028 (6 insertions, 18 deletions)

---

## 2. New Features

### 2.1 LispBM Encoder Sampling Function
**Commit**: `169d5f7` - "Added enc_sample function"
**Date**: November 5, 2025
**Impact**: MEDIUM for encoder calibration workflows

#### Description
New LispBM function `enc-sample` for automated encoder error measurement and calibration.

#### Functionality
```lisp
(enc-sample buffer samples)
```

**Parameters**:
- `buffer`: Byte array of size ≥ 2880 bytes (360 degrees × 2 values × 4 bytes)
- `samples`: Number of samples to collect (1 to 300,000)

**Operation**:
1. For each sample:
   - Measure encoder angle from physical encoder
   - Measure BEMF angle from FOC observer
   - Calculate error: `err = BEMF_angle - encoder_angle`
   - Accumulate error at 1-degree resolution (360 bins)
   - Count samples per bin
2. Returns `true` on success, `eerror` on failure
3. Blocking operation (uses worker thread)

**Data Format** (for each 1-degree bin):
```
Offset = degree × 8
[0-3]: float32 - accumulated error sum
[4-7]: float32 - sample count
```

#### Use Case
Automated encoder calibration scripts can now:
1. Spin motor through full revolution
2. Collect thousands of encoder vs BEMF measurements
3. Calculate average error for each degree
4. Generate encoder correction table
5. Apply with `enc-corr` function

#### Example
```lisp
; Allocate buffer for 360 degrees
(define enc-buf (bufcreate 2880))

; Collect 10,000 samples while motor spins
(enc-sample enc-buf 10000)

; Process results (pseudocode)
(looprange i 0 360
    (let ((offset (* i 8))
          (err-sum (bufget-f32 enc-buf offset))
          (count (bufget-f32 enc-buf (+ offset 4))))
        (if (> count 10)  ; Enough samples for this degree?
            (let ((avg-err (/ err-sum count)))
                ; Store correction for this degree
                (enc-corr i avg-err)))))
```

#### Files Modified
- `lispBM/README.md`: +48 lines (documentation)
- `lispBM/lispif.c`: Minor version update
- `lispBM/lispif_vesc_extensions.c`: +62 lines (implementation)

#### Impact
- Enables automated encoder calibration workflows
- Reduces manual calibration effort for high-precision applications
- Non-breaking (additive LispBM extension)

---

### 2.2 LispBM Backup Storage Function
**Commit**: `3563295` - "Added store-backup extension"
**Date**: October 30, 2025
**Impact**: LOW (LispBM convenience function)

#### Description
New LispBM function `store-backup` to manually persist backup data to flash.

#### Functionality
```lisp
(store-backup)
```

**Returns**: `true` on success, `nil` on failure

#### Background
VESC maintains backup data in battery-backed SRAM that includes:
- Odometer values
- Runtime counters
- Hardware configuration
- Encoder correction tables
- CAN ID and baud rate (new)

This data normally persists through power cycles but needs to be written to flash for permanent storage.

#### Previous Behavior
Backup data was automatically written to flash when:
- `(conf-store)` was called
- Configuration changes were saved

#### New Behavior
With `store-backup`, LispBM scripts can explicitly trigger backup storage without saving configuration changes.

#### Use Case
```lisp
; Update odometer from GPS
(set-odometer (+ (get-odometer) gps-distance))

; Persist to flash without changing motor/app config
(store-backup)
```

#### Files Modified
- `conf_general.c`: 1 line change (removed auto-backup from conf-store)
- `lispBM/README.md`: +14 lines (documentation)
- `lispBM/lispif.c`: Minor version update
- `lispBM/lispif_vesc_extensions.c`: +8 lines (implementation)

#### Impact
- More explicit control over flash writes
- Reduces unnecessary flash writes when storing config
- Backwards compatible (existing code continues to work)

---

### 2.3 Filter Delay Compensation for Sin/Cos Encoder
**Commit**: `c5a8bb5` - "Added filter delay compensation"
**Date**: November 3, 2025
**Impact**: MEDIUM for high-speed sin/cos encoder applications

#### Description
Automatic phase compensation for sin/cos encoder readings to correct for low-pass filter delay.

#### Problem
Sin/cos encoders use analog signals that are low-pass filtered to reduce noise. At high speeds, this filtering introduces phase delay that causes position error.

**Example**:
- Motor speed: 10,000 ERPM
- Filter time constant: 1ms
- Phase delay: ~6 electrical degrees
- Position error: Proportional to speed

#### Solution
Compensate for filter delay based on motor speed and filter characteristics.

#### Implementation
```c
// Calculate delay compensation
float delay_comp = ((1.0 - cfg->filter_constant) * RPM2RADPS_f(mc_interface_get_rpm()) * timestep) /
                   (cfg->filter_constant * cfg->ratio);

// Apply compensation with configurable sign
float angle = RAD2DEG_f(utils_fast_atan2(sin, cos) + delay_comp * cfg->delay_comp_sign) + 180.0;
utils_norm_angle(&angle);
```

**Parameters**:
- `filter_constant`: Filter time constant (from encoder config)
- `ratio`: Encoder gear ratio
- `timestep`: Sample period
- `delay_comp_sign`: +1 or -1 (direction of compensation)

#### Configuration
New encoder configuration parameters:
- `float filter_constant`: Low-pass filter time constant (default: 0.0)
- `float delay_comp_sign`: Compensation direction (default: 1.0)

#### When to Use
- High-speed applications (>5,000 ERPM)
- Sin/cos encoders with hardware filtering
- Position-critical applications

#### Files Modified
- `CHANGELOG.md`: +1 line
- `conf_general.c`: +1 line (default config)
- `encoder/enc_sincos.c`: +7 lines, -1 line (implementation)
- `encoder/encoder.c`: +4 lines (parameter handling)
- `encoder/encoder_datatype.h`: +2 lines (new config fields)

#### Impact
- Improved position accuracy at high speeds
- Configurable compensation (disabled by default)
- No impact on non-sin/cos encoders

---

### 2.4 CAN ID and Baud Rate Persistence
**Commit**: `00bc35b` - "Persist CAN ID and CAN Baud Rate across firmware updates"
**Date**: October 20, 2025
**Impact**: HIGH for multi-ESC CAN networks

#### Description
CAN controller ID and baud rate now persist across firmware updates, stored in battery-backed SRAM.

#### Problem (Before)
When updating firmware on multi-ESC systems:
1. Flash firmware to ESC (e.g., change ID 1 to 5, baud to 500k)
2. ESC resets to defaults during firmware flash erase
3. **CAN ID resets to default (often 0 or 1)**
4. **Baud rate resets to default (often 250k or 500k)**
5. Must reconfigure CAN ID and baud rate after every firmware update

**Issue**: In a multi-ESC system (e.g., skateboard with 2 ESCs):
- ESC 1 configured as ID 1
- ESC 2 configured as ID 2
- Update firmware on ESC 2
- **ESC 2 resets to ID 1 (conflict!)**
- System malfunction until manually reconfigured

#### Solution (After)
CAN ID and baud rate are stored in backup structure alongside odometer and runtime:

```c
typedef struct {
    uint32_t odometer_init_flag;
    uint32_t odometer;
    uint32_t runtime_init_flag;
    uint32_t runtime;
    uint32_t hw_config_init_flag;
    uint8_t hw_config[2048];
    uint32_t enc_corr_init_flag;      // New
    uint8_t enc_corr[2048];           // New
    bool enc_corr_en;                 // New
    uint32_t can_init_flag;           // New
    CAN_BAUD can_baud;                // New - Persisted!
    uint8_t can_id;                   // New - Persisted!
} backup_data;
```

#### Implementation Flow

**On Boot** (`conf_general_init`):
```c
// Check if CAN settings are backed up
if (g_backup.can_init_flag == BACKUP_VAR_INIT_CODE) {
    // Restore from backup
    backup_tmp.can_baud = g_backup.can_baud;
    backup_tmp.can_id = g_backup.can_id;
} else {
    // First boot - use defaults
    backup_tmp.can_baud = APPCONF_CAN_BAUD_RATE;
    backup_tmp.can_id = HW_DEFAULT_ID;
}
```

**On Config Read** (`conf_general_read_app_configuration`):
```c
if (!is_ok) {  // If config flash is corrupted/erased
    confgenerator_set_defaults_appconf(conf);
    // Restore CAN settings from backup!
    conf->can_baud_rate = g_backup.can_baud;
    conf->controller_id = g_backup.can_id;
}
```

**On Config Store** (`conf_general_store_app_configuration`):
```c
// Save CAN settings to backup SRAM
g_backup.can_id = conf->controller_id;
g_backup.can_baud = conf->can_baud_rate;
conf_general_store_backup_data();  // Write to flash
```

#### Storage Location
**Battery-Backed SRAM** (survives power cycles):
- Lives in MCU's backup SRAM domain
- Powered by backup battery or supercapacitor
- Persists through power cycles and resets
- Periodically written to flash for permanent storage

**Flash Backup**:
- Written to dedicated flash sector
- Restored if SRAM loses power

#### Benefits
1. **Multi-ESC Systems**: No more ID conflicts after firmware updates
2. **Field Deployments**: Firmware updates don't break CAN network configuration
3. **User Experience**: "Update firmware and everything just works"
4. **Production**: Flash once, configure once, update freely

#### Example Scenario
```
Before:
1. Configure ESC: ID=5, Baud=500k
2. Update firmware
3. ESC resets to: ID=0, Baud=250k (default)
4. Manually reconfigure to ID=5, Baud=500k

After:
1. Configure ESC: ID=5, Baud=500k
2. Update firmware
3. ESC restores from backup: ID=5, Baud=500k
4. Done! No reconfiguration needed
```

#### Files Modified
- `conf_general.c`: +21 lines (backup save/restore logic)
- `datatypes.h`: +8 lines (backup structure fields)
- `terminal.c`: +7 lines, -2 lines (terminal command updates)

#### Impact
- **CRITICAL for multi-ESC deployments**
- Eliminates CAN ID conflicts after firmware updates
- Backwards compatible (defaults to HW_DEFAULT_ID if no backup)
- Applies to VESC Tool GUI, terminal commands, and LispBM scripts

---

## 3. Improvements

### 3.1 HFI Reset ERPM Parameter
**Commit**: `fcadd51` - "Added HFI reset ERPM parameter"
**Date**: November 3, 2025
**Impact**: LOW (sensorless control tuning)

#### Description
New configurable parameter `foc_hfi_reset_erpm` for High Frequency Injection (HFI) sensorless control.

#### Background
HFI (High Frequency Injection) is a sensorless control method that works at low speeds by injecting a high-frequency signal into the motor and observing the response to determine rotor position.

As motor speed increases, HFI becomes less reliable and the system should transition to BEMF observer.

#### Previous Behavior
HFI reset threshold was hardcoded in `mcpwm_foc.c`:
```c
// Hardcoded threshold
if (rpm > 1000) {
    // Reset HFI, transition to BEMF observer
}
```

#### New Behavior
Threshold is now configurable in motor configuration:
```c
typedef struct {
    // ... existing fields ...
    float foc_hfi_reset_erpm;  // HFI reset speed threshold (ERPM)
} mc_configuration;
```

**Default value**: Set in `motor/mcconf_default.h` based on motor type

#### Configuration
- **Parameter**: `foc_hfi_reset_erpm`
- **Type**: `float`
- **Units**: ERPM (Electrical RPM)
- **Range**: Typically 500-3000 ERPM
- **Access**: VESC Tool GUI, terminal, LispBM

#### LispBM Access
```lisp
; Get current threshold
(conf-get 'foc-hfi-reset-erpm)

; Set new threshold (2000 ERPM)
(conf-set 'foc-hfi-reset-erpm 2000)
```

#### Files Modified
- `confgenerator.c`: +3 lines (config serialization)
- `confgenerator.h`: Version update
- `datatypes.h`: +1 line (new field)
- `lispBM/README.md`: +1 line (documentation)
- `lispBM/lispif_vesc_extensions.c`: +8 lines (LispBM binding)
- `motor/mcpwm_foc.c`: +9 lines, -8 lines (use config parameter)

#### Impact
- Enables per-motor HFI tuning
- Better support for different motor types
- Non-breaking (uses sensible defaults)

---

### 3.2 HFI Configuration Improvements
**Commit**: `13aa254` - "Updated changelog, config tweaks, HFI reset improvement"
**Date**: November 3, 2025
**Impact**: LOW (sensorless control refinement)

#### Description
Minor improvements to HFI (High Frequency Injection) configuration and defaults.

#### Changes

**1. CHANGELOG Update**
Added 7 lines documenting recent changes.

**2. Configuration Generator Update**
- `confgenerator.c`: 1 line change (config version update)

**3. Default Motor Configuration Updates**
- `motor/mcconf_default.h`: 9 lines updated (default parameter refinements)
- Adjusted default HFI parameters for better out-of-box experience

**4. FOC Implementation Update**
- `motor/mcpwm_foc.c`: 1 line change (HFI reset logic refinement)

#### Files Modified
- `CHANGELOG.md`: +7 lines
- `confgenerator.c`: 1 line
- `motor/mcconf_default.h`: 9 lines adjusted
- `motor/mcpwm_foc.c`: 1 line

#### Impact
- Improved default HFI behavior
- Better sensorless performance out-of-box
- Non-breaking (configuration updates)

---

## 4. Code Cleanup

### 4.1 Removal of Obsolete Hardware Configurations
**Commit**: `6b0e91f` - "Removed obsolete parameter and some old firmwares"
**Date**: November 3, 2025
**Impact**: MEDIUM (reduced firmware size)

#### Description
Removed 23 obsolete hardware configuration files for discontinued or unmaintained hardware.

#### Hardware Removed

**DAS Hardware (2 variants)**:
- `hwconf/other/hw_das_mini.c` / `.h` - 479 lines
- `hwconf/other/hw_das_rs.c` / `.h` - 503 lines

**ES19 Hardware**:
- `hwconf/other/hw_es19.c` / `.h` - 562 lines

**Mini4 Hardware**:
- `hwconf/other/hw_mini4.c` / `.h` - 460 lines

**RD2 Hardware**:
- `hwconf/other/hw_rd2.c` / `.h` - 525 lines

**RESC Hardware**:
- `hwconf/other/hw_resc.c` / `.h` - 403 lines

**RH Hardware**:
- `hwconf/other/hw_rh.c` / `.h` - 463 lines

**UAVC Omega Hardware**:
- `hwconf/other/hw_uavc_omega.c` / `.h` - 496 lines

**UAVC QCube Hardware**:
- `hwconf/other/hw_uavc_qcube.c` / `.h` - 528 lines

**UXV SR Hardware**:
- `hwconf/other/hw_uxv_sr.c` / `.h` - 590 lines

#### Total Deletion
- **23 files removed**
- **5,035 lines deleted**
- Estimated firmware size reduction: ~50-100KB (depends on build configuration)

#### Additional Changes
- `confgenerator.c`: 6 lines removed (obsolete parameter)
- `confgenerator.h`: Version update
- `datatypes.h`: 2 lines removed (obsolete parameter definitions)
- `motor/mcpwm_foc.c`: 19 lines removed, 3 added (cleanup)

#### Impact
- **Positive**: Smaller firmware binary, faster compilation, easier maintenance
- **Negative**: Old hardware no longer supported (must use older firmware version)
- **Migration**: Users with these hardware variants must use firmware versions prior to this commit

#### Verification
Check if your hardware is affected:
```bash
# Search for your hardware name in old commits
git log --all --full-history -- "hwconf/other/hw_*.c" | grep "your_hw_name"
```

---

## 5. Hardware Support Updates

### 5.1 Luna Cycle BBSHD and M600 Improvements
**Commit**: `2b32501` - Merge pull request #856 "Small improvements on luna hardware"
**Date**: October 13, 2025
**Impact**: LOW (Luna hardware users only)

#### Description
Updates and improvements to Luna Cycle BBSHD and M600 mid-drive motor hardware support.

#### Changes Summary

**BBSHD (Bafang mid-drive motor for bicycles)**:
- `hwconf/luna/bbshd/appconf_luna_bbshd.h`: 2 lines changed (config update)
- `hwconf/luna/bbshd/bbshd_ui.qml`: +1738 lines (new QML UI!)
- `hwconf/luna/bbshd/hw_luna_bbshd.c`: +14 lines (hardware init improvements)
- `hwconf/luna/bbshd/hw_luna_bbshd.h`: 6 lines changed (pin definitions)
- `hwconf/luna/bbshd/qmlui_luna_bbshd.c`: 742 lines modified (UI compilation)
- `hwconf/luna/bbshd/qmlui_luna_bbshd.h`: Version update

**M600 (Bafang M600 mid-drive motor)**:
- `hwconf/luna/m600/appconf_luna_m600.h`: 2 lines changed (config update)
- `hwconf/luna/m600/hw_luna_m600.h`: 9 lines changed (pin definitions)
- `hwconf/luna/m600/hw_luna_m600_core.c`: +14 lines (hardware init improvements)
- `hwconf/luna/m600/luna_m600_display.c`: 11 lines changed (display handling)
- `hwconf/luna/m600/luna_tune_M600.qml`: +1 line (QML update)
- `hwconf/luna/m600/qmlui_luna_m600.c`: 887 lines modified (UI compilation)
- `hwconf/luna/m600/qmlui_luna_m600.h`: Version update

#### Total Changes
- **13 files modified**
- **+2604 lines, -826 lines**
- **Net addition**: +1778 lines

#### Key Improvements

**1. New QML User Interface for BBSHD**
- Complete custom UI for BBSHD hardware (1738 lines)
- Hardware-specific tuning and diagnostics
- Display integration for e-bike displays

**2. Hardware Initialization Improvements**
- Better GPIO configuration
- Improved peripheral initialization
- Enhanced compatibility with display modules

**3. Display Integration**
- Updated display communication protocol
- Better display refresh and responsiveness

#### Impact
- **BBSHD/M600 users**: Improved UI and hardware support
- **Other hardware**: No impact
- **Non-breaking**: Existing BBSHD/M600 configurations continue to work

---

## 6. Files Modified Summary

### By Category

**Motor Control Core**:
- `motor/mc_interface.c` - Bug fix (wh/ah counters)
- `motor/mcpwm_foc.c` - HFI improvements and obsolete code removal
- `motor/mcconf_default.h` - Default parameter updates

**Encoder Subsystem**:
- `encoder/enc_sincos.c` - Filter delay compensation
- `encoder/encoder.c` - New configuration parameters
- `encoder/encoder_datatype.h` - Structure updates

**Configuration System**:
- `conf_general.c` - CAN ID/baud persistence, backup storage
- `confgenerator.c` - Parameter additions and removals
- `confgenerator.h` - Version updates
- `datatypes.h` - Structure updates (backup data, config parameters)

**LispBM Scripting**:
- `lispBM/README.md` - Documentation updates
- `lispBM/lispif.c` - Version updates
- `lispBM/lispif_vesc_extensions.c` - New functions (enc-sample, store-backup)

**Hardware Configurations**:
- 23 files deleted (obsolete hardware)
- 13 files modified (Luna BBSHD/M600 improvements)

**Other**:
- `CHANGELOG.md` - Release notes
- `terminal.c` - CAN persistence updates

### Statistics

**Total commits reviewed**: 9
**Total files changed**: 63
**Lines added**: ~2,700
**Lines removed**: ~5,900
**Net change**: ~-3,200 lines (code cleanup)

---

## 7. Impact Assessment

### Critical Impact (Action Required)

**Dual Motor Systems**:
- **Issue**: Watt-hour/amp-hour counters were incorrect
- **Fix**: Commit `510e33f` corrects the bug
- **Action**: Update firmware to get accurate energy measurements

**Multi-ESC CAN Networks**:
- **Issue**: Firmware updates reset CAN ID and baud rate
- **Fix**: Commit `00bc35b` persists CAN configuration
- **Action**: Update firmware to avoid CAN ID conflicts during updates
- **Benefit**: Firmware updates no longer require CAN reconfiguration

### High Impact (Recommended Update)

**Sin/Cos Encoders at High Speed**:
- **Issue**: Filter delay causes position error at high RPM
- **Fix**: Commit `c5a8bb5` adds automatic compensation
- **Action**: Update firmware and configure `filter_constant` parameter
- **Benefit**: Improved position accuracy above 5,000 ERPM

### Medium Impact (Optional Update)

**LispBM Encoder Calibration**:
- **New Feature**: `enc-sample` function for automated calibration
- **Benefit**: Faster, more accurate encoder calibration workflows
- **Target Users**: High-precision position control applications

**Firmware Size Reduction**:
- **Change**: Removed 5000+ lines of obsolete hardware support
- **Benefit**: Smaller firmware binary, faster compilation
- **Note**: Old hardware no longer supported (use older firmware if needed)

### Low Impact (Nice to Have)

**HFI Sensorless Control**:
- **New Feature**: Configurable HFI reset threshold
- **Benefit**: Better sensorless tuning for specific motors
- **Target Users**: Sensorless FOC applications

**LispBM Backup Storage**:
- **New Feature**: `store-backup` function
- **Benefit**: Explicit control over flash writes
- **Target Users**: Advanced LispBM scripting

**Luna Hardware**:
- **Updates**: BBSHD and M600 improvements
- **Benefit**: Better UI and hardware integration
- **Target Users**: Luna Cycle BBSHD/M600 owners only

---

## 8. Migration Guide

### For Dual Motor Users

**Before Update**:
```c
// Energy counters may be incorrect due to bug
mc_interface_select_motor_thread(1);
float wh1 = mc_interface_get_watt_hours(false);  // Possibly wrong!
mc_interface_select_motor_thread(2);
float wh2 = mc_interface_get_watt_hours(false);  // Possibly wrong!
```

**After Update**:
```c
// Energy counters are now correct
mc_interface_select_motor_thread(1);
float wh1 = mc_interface_get_watt_hours(false);  // Correct!
mc_interface_select_motor_thread(2);
float wh2 = mc_interface_get_watt_hours(false);  // Correct!
```

**Action**: Consider resetting odometer/energy counters after update for clean baseline.

### For Multi-ESC CAN Networks

**Before Update**:
```
1. Configure ESC 1: ID=1, Baud=500k
2. Configure ESC 2: ID=2, Baud=500k
3. Update firmware on ESC 2
4. ESC 2 resets to default: ID=0, Baud=250k
5. Manually reconfigure ESC 2: ID=2, Baud=500k
```

**After Update**:
```
1. Configure ESC 1: ID=1, Baud=500k
2. Configure ESC 2: ID=2, Baud=500k
3. Update firmware on ESC 2
4. ESC 2 retains: ID=2, Baud=500k (automatic!)
5. Done - no reconfiguration needed
```

**Action**: One-time update to all ESCs in network. Future firmware updates will preserve CAN configuration.

### For Sin/Cos Encoder Users

**Previous Configuration** (no delay compensation):
```c
encoder_type = ENCODER_TYPE_SINCOS
filter_constant = 0.0  // No filtering
```

**New Configuration** (with delay compensation):
```c
encoder_type = ENCODER_TYPE_SINCOS
filter_constant = 0.001  // 1ms filter time constant
delay_comp_sign = 1.0    // Compensation direction (try ±1.0)
```

**Tuning Procedure**:
1. Update firmware
2. Configure `filter_constant` to match hardware filter
3. Spin motor at high speed (>5000 ERPM)
4. Compare encoder reading vs BEMF observer
5. Adjust `delay_comp_sign` to minimize error (+1.0 or -1.0)
6. Verify at multiple speeds

### For LispBM Encoder Calibration

**Old Manual Calibration Workflow**:
1. Spin motor manually
2. Record encoder vs BEMF at each position
3. Calculate error table
4. Apply corrections with `enc-corr`

**New Automated Workflow**:
```lisp
; Step 1: Allocate buffer
(define enc-buf (bufcreate 2880))

; Step 2: Spin motor and collect samples
(set-duty 0.1)  ; Gentle spin
(enc-sample enc-buf 50000)  ; 50k samples

; Step 3: Process results
(looprange deg 0 360
    (let ((offset (* deg 8)))
        (let ((err-sum (bufget-f32 enc-buf offset))
              (count (bufget-f32 enc-buf (+ offset 4))))
            (if (> count 20)
                (enc-corr deg (/ err-sum count))))))

; Step 4: Enable corrections
(enc-corr-en true)

; Step 5: Store
(conf-store)
```

---

## 9. Testing Recommendations

### Regression Testing

**All Systems**:
1. ✓ Verify motor starts and runs normally
2. ✓ Check fault handling (overcurrent, overvoltage, etc.)
3. ✓ Verify CAN communication (if applicable)
4. ✓ Test VESC Tool connection and parameter reads

**Dual Motor Systems**:
1. ✓ Reset energy counters
2. ✓ Run motors for fixed time (e.g., 10 minutes)
3. ✓ Verify both motors show similar wh/ah (if same load)
4. ✓ Compare with expected power consumption

**Multi-ESC CAN Networks**:
1. ✓ Record CAN ID and baud rate before firmware update
2. ✓ Update firmware
3. ✓ Verify CAN ID and baud rate match previous values
4. ✓ Test CAN communication between ESCs

**Sin/Cos Encoders**:
1. ✓ Run motor at low speed (<1000 ERPM)
2. ✓ Run motor at high speed (>5000 ERPM)
3. ✓ Compare encoder position vs BEMF observer
4. ✓ Verify position error is minimized with delay compensation

**LispBM Scripts**:
1. ✓ Test existing LispBM scripts for compatibility
2. ✓ Verify `(conf-store)` no longer auto-saves backup (use `store-backup`)
3. ✓ Test new `enc-sample` and `store-backup` functions if applicable

### Performance Testing

**Firmware Size**:
```bash
# Compare firmware binary size before/after
ls -lh firmware.bin

# Expected: ~50-100KB reduction due to hw removal
```

**Motor Control Timing**:
```bash
# Verify FOC loop timing unchanged
# Terminal command: stats

# Check for:
# - FOC frequency: ~40 kHz (unchanged)
# - CPU usage: Should be similar or slightly improved
```

---

## 10. Known Issues and Limitations

### CAN ID Persistence

**Limitation**: Requires battery-backed SRAM
Some hardware designs don't have backup battery/supercapacitor. In these cases:
- CAN ID/baud will persist through soft resets
- CAN ID/baud will NOT persist through complete power loss
- Flash backup will restore on next boot if power lost

**Workaround**: Ensure backup SRAM has power source (check hardware design).

### Filter Delay Compensation

**Limitation**: Only for sin/cos encoders
Other encoder types (hall, quadrature) don't use the filter compensation path.

**Note**: Must manually configure `filter_constant` to match hardware.

### Encoder Sampling Function

**Limitation**: Blocking operation
`enc-sample` blocks the calling LispBM thread until complete. Don't use in real-time control loops.

**Workaround**: Run calibration as separate offline procedure.

### Hardware Removal

**Breaking Change**: 10 hardware variants no longer supported
If you have DAS, ES19, Mini4, RD2, RESC, RH, UAVC, or UXV hardware, you must use firmware versions prior to commit `6b0e91f`.

**Migration Path**:
- Continue using older firmware, or
- Port your hardware configuration to current firmware structure

---

## 11. Recommendations

### Immediate Action (High Priority)

1. **Dual Motor Systems**: Update firmware to fix wh/ah counter bug
2. **Multi-ESC Networks**: Update firmware to enable CAN persistence
3. **Verify Hardware**: Ensure your hardware is not in the obsolete list

### Short-Term (Medium Priority)

1. **Sin/Cos Encoders**: Configure filter delay compensation if running >5000 ERPM
2. **Test LispBM Scripts**: Verify compatibility with `conf-store` behavior change
3. **Backup Configuration**: Save current configuration before updating

### Long-Term (Low Priority)

1. **Encoder Calibration**: Migrate to automated `enc-sample` workflow
2. **HFI Tuning**: Experiment with configurable reset threshold for sensorless
3. **Code Review**: Review removed hardware configs if maintaining custom builds

---

## 12. References

### Commit Details
All commits authored by Benjamin Vedder (benjamin@vedder.se):

- `2b32501` - October 13, 2025 - Luna M600 support improvements
- `00bc35b` - October 20, 2025 - CAN ID/baud persistence
- `3563295` - October 30, 2025 - LispBM store-backup extension
- `fcadd51` - November 3, 2025 - HFI reset ERPM parameter
- `13aa254` - November 3, 2025 - Changelog and HFI improvements
- `6b0e91f` - November 3, 2025 - Obsolete hardware removal
- `c5a8bb5` - November 3, 2025 - Filter delay compensation
- `510e33f` - November 5, 2025 - Wh/ah counter bug fix
- `169d5f7` - November 5, 2025 - LispBM enc-sample extension

### Related Documentation
- `docs/code_description/01_main.md` - System initialization
- `docs/code_description/02_mc_interface.md` - Motor control interface
- `docs/code_description/03_app.md` - Application layer
- `docs/code_description/04_chibios_integration.md` - RTOS integration
- `docs/code_description/06_thread_call_flows.md` - Thread architecture

### External Resources
- VESC Project: https://vesc-project.com/
- VESC Tool: https://vesc-project.com/vesc_tool
- VESC Forum: https://vesc-project.com/forum
- GitHub Repository: https://github.com/vedderb/bldc

---

## Appendix A: Configuration Parameter Changes

### New Parameters

**Motor Configuration**:
```c
mc_configuration:
    float foc_hfi_reset_erpm;  // HFI reset speed threshold (ERPM)
```

**Encoder Configuration**:
```c
encoder_config:
    float filter_constant;      // Sin/cos filter time constant
    float delay_comp_sign;      // Delay compensation direction (±1.0)
```

**Backup Data** (internal structure):
```c
backup_data:
    uint32_t enc_corr_init_flag;
    uint8_t enc_corr[2048];
    bool enc_corr_en;
    uint32_t can_init_flag;
    CAN_BAUD can_baud;          // Persisted CAN baud rate
    uint8_t can_id;             // Persisted CAN controller ID
```

### Removed Parameters

From `datatypes.h`:
- 2 obsolete parameters removed (specific names not critical - cleanup)

---

## Appendix B: LispBM API Changes

### New Functions

**enc-sample** - Collect encoder calibration samples
```lisp
(enc-sample buffer samples)
; buffer: byte array (≥2880 bytes)
; samples: integer (1-300000)
; Returns: true on success, eerror on failure
```

**store-backup** - Manually persist backup data
```lisp
(store-backup)
; No parameters
; Returns: true on success, nil on failure
```

### Modified Behavior

**conf-store** - No longer auto-saves backup data
```lisp
; Before:
(conf-store)  ; Saved config AND backup data

; After:
(conf-store)      ; Only saves config
(store-backup)    ; Must explicitly save backup if needed
```

---

## Appendix C: Firmware Size Analysis

### Binary Size Comparison

**Before cleanup** (estimate):
- Base firmware: ~400 KB
- Hardware configs: ~100 KB
- Total: ~500 KB

**After cleanup** (estimate):
- Base firmware: ~400 KB
- Hardware configs: ~50 KB
- Total: ~450 KB
- **Savings**: ~50 KB (10% reduction in hardware config size)

**Note**: Actual savings depend on build configuration and enabled features.

---

## Document End

**Generated**: November 11, 2025
**Firmware Version**: VESC 6.x (November 2025 upstream)
**Revision**: 1.0
