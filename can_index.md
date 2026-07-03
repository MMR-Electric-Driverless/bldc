# CAN Status Message Index (for DBC configuration)

Reference for building a `.dbc` for the VESC status frames emitted by this
firmware, including the extra FOC/braking-debug frames (STATUS_8..12) added in
commit `0cf3fc4a` and trimmed afterwards. Layouts here match
[comm/comm_can.c](../comm/comm_can.c) as currently in the tree.

## Frame identity

All status frames use the VESC **extended 29-bit** identifier:

```
EID = (CAN_PACKET_ID << 8) | VESC_ID
```

- `VESC_ID` is the controller id (0..255), the low byte.
- `CAN_PACKET_ID` is the message type, the next byte.
- In a DBC these are **Extended** frames. If you want one DBC per node, hard-code
  `VESC_ID`; if you want a node-agnostic DBC, define the messages at
  `CAN_PACKET_ID << 8` and mask off the low byte, or use a multiplexed/wildcard
  scheme in your tool.

| Message   | CAN_PACKET_ID | Base EID (VESC_ID=0) | DLC | Enable bit in VESC Tool |
|-----------|---------------|----------------------|-----|-------------------------|
| STATUS_1  | 9  (0x09)     | 0x0900               | 8   | yes (rate 1)            |
| STATUS_2  | 14 (0x0E)     | 0x0E00               | 8   | yes                     |
| STATUS_3  | 15 (0x0F)     | 0x0F00               | 8   | yes                     |
| STATUS_4  | 16 (0x10)     | 0x1000               | 8   | yes                     |
| STATUS_5  | 27 (0x1B)     | 0x1B00               | 8   | yes                     |
| STATUS_6  | 58 (0x3A)     | 0x3A00               | 8   | yes                     |
| STATUS_7  | 69 (0x45)     | 0x4500               | 5   | no — always sent        |
| STATUS_8  | 70 (0x46)     | 0x4600               | 7   | no — always sent        |
| STATUS_9  | 71 (0x47)     | 0x4700               | 8   | no — always sent        |
| STATUS_10 | 72 (0x48)     | 0x4800               | 5   | no — always sent        |
| STATUS_11 | 73 (0x49)     | 0x4900               | 8   | no — always sent        |
| STATUS_12 | 74 (0x4A)     | 0x4A00               | 8   | no — always sent        |

STATUS_7..12 are transmitted unconditionally at the `can_status_rate_1` cadence
(and at 500 Hz on the internal thread for the second motor). STATUS_1..6 honor
the VESC-Tool enable mask.

## Signal encoding conventions

Every multi-byte value is written MSB-first, so in DBC terms:

- **Byte order:** Motorola / big-endian (MSB first).
- **Value type:** signed for all `int16`/`int32`/`float16` fields (the firmware's
  `buffer_append_float16` is just `(int16_t)(value * scale)` big-endian). `uint8`
  fields (fault, control_mode, flags) are unsigned.
- **DBC factor / offset:** `factor = 1 / scale`, `offset = 0`. The `scale` column
  below is the firmware multiplier; e.g. `scale = 100` → `factor = 0.01`.
- **Motorola start bit:** for a byte-aligned big-endian signal at byte offset `N`,
  the MSB start bit is `N*8 + 7` (given in the tables). Some editors let you set
  start byte + byte order visually instead — either is fine.

## Messages

### STATUS_1 — 0x0900 (DLC 8)

| Signal          | Bytes | Start bit | Type  | scale | factor | Unit |
|-----------------|-------|-----------|-------|-------|--------|------|
| erpm            | 0..3  | 7         | int32 | 1     | 1      | ERPM |
| current_motor   | 4..5  | 39        | int16 | 10    | 0.1    | A    |
| duty            | 6..7  | 55        | int16 | 1000  | 0.001  | ratio (-1..1) |

### STATUS_2 — 0x0E00 (DLC 8)

| Signal              | Bytes | Start bit | Type  | scale | factor | Unit |
|---------------------|-------|-----------|-------|-------|--------|------|
| amp_hours           | 0..3  | 7         | int32 | 10000 | 1e-4   | Ah   |
| amp_hours_charged   | 4..7  | 39        | int32 | 10000 | 1e-4   | Ah   |

### STATUS_3 — 0x0F00 (DLC 8)

| Signal               | Bytes | Start bit | Type  | scale | factor | Unit |
|----------------------|-------|-----------|-------|-------|--------|------|
| watt_hours           | 0..3  | 7         | int32 | 10000 | 1e-4   | Wh   |
| watt_hours_charged   | 4..7  | 39        | int32 | 10000 | 1e-4   | Wh   |

### STATUS_4 — 0x1000 (DLC 8)

| Signal        | Bytes | Start bit | Type  | scale | factor | Unit |
|---------------|-------|-----------|-------|-------|--------|------|
| temp_fet      | 0..1  | 7         | int16 | 10    | 0.1    | °C   |
| temp_motor    | 2..3  | 23        | int16 | 10    | 0.1    | °C   |
| current_in    | 4..5  | 39        | int16 | 10    | 0.1    | A    |
| pid_pos       | 6..7  | 55        | int16 | 50    | 0.02   | deg  |

### STATUS_5 — 0x1B00 (DLC 8)

| Signal        | Bytes | Start bit | Type  | scale | factor | Unit |
|---------------|-------|-----------|-------|-------|--------|------|
| tachometer    | 0..3  | 7         | int32 | 1     | 1      | counts |
| v_in          | 4..5  | 39        | int16 | 10    | 0.1    | V    |
| reserved      | 6..7  | 55        | int16 | —     | —      | (always 0) |

### STATUS_6 — 0x3A00 (DLC 8)

| Signal      | Bytes | Start bit | Type  | scale | factor | Unit |
|-------------|-------|-----------|-------|-------|--------|------|
| adc_ext1    | 0..1  | 7         | int16 | 1000  | 1e-3   | V    |
| adc_ext2    | 2..3  | 23        | int16 | 1000  | 1e-3   | V    |
| adc_ext3    | 4..5  | 39        | int16 | 1000  | 1e-3   | V    |
| servo       | 6..7  | 55        | int16 | 1000  | 1e-3   | ratio |

### STATUS_7 — 0x4500 (DLC 5) — measured currents + fault

| Signal   | Bytes | Start bit | Type  | scale | factor | Unit |
|----------|-------|-----------|-------|-------|--------|------|
| id_meas  | 0..1  | 7         | int16 | 100   | 0.01   | A    |
| iq_meas  | 2..3  | 23        | int16 | 100   | 0.01   | A    |
| fault    | 4     | 39        | uint8 | 1     | 1      | enum (mc_fault_code) |

### STATUS_8 — 0x4600 (DLC 7) — current setpoints / MTPA

| Signal        | Bytes | Start bit | Type  | scale | factor | Unit |
|---------------|-------|-----------|-------|-------|--------|------|
| id_target     | 0..1  | 7         | int16 | 100   | 0.01   | A    |
| iq_target     | 2..3  | 23        | int16 | 100   | 0.01   | A    |
| i_fw          | 4..5  | 39        | int16 | 100   | 0.01   | A    |
| control_mode  | 6     | 55        | uint8 | 1     | 1      | enum (see below) |

Final d/q targets after MTPA + field weakening + current limiting; `i_fw` is the
field-weakening current magnitude. (The old flags byte was removed.)

### STATUS_9 — 0x4700 (DLC 8) — FOC voltages + bus

| Signal          | Bytes | Start bit | Type  | scale | factor | Unit |
|-----------------|-------|-----------|-------|-------|--------|------|
| vd              | 0..1  | 7         | int16 | 100   | 0.01   | V    |
| vq              | 2..3  | 23        | int16 | 100   | 0.01   | V    |
| duty_filtered   | 4..5  | 39        | int16 | 1000  | 0.001  | ratio (-1..1) |
| v_bus           | 6..7  | 55        | int16 | 100   | 0.01   | V    |

`vd`/`vq` are the PI-loop output voltages (post decoupling/bemf feedforward and
saturation). `mod_d = vd * 1.5 / v_bus`, `mod_q = vq * 1.5 / v_bus`.
`duty_filtered` is the smoothed duty (distinct from STATUS_1's instantaneous
`duty`). `v_bus` also lets you reconstruct
`max_v_mag = (1/sqrt3) * max_duty * v_bus * overmod`.

### STATUS_10 — 0x4800 (DLC 5) — braking / loss-of-control

| Signal   | Bytes | Start bit | Type  | scale | factor | Unit |
|----------|-------|-----------|-------|-------|--------|------|
| v_mag    | 0..1  | 7         | int16 | 100   | 0.01   | V    |
| bemf     | 2..3  | 23        | int16 | 100   | 0.01   | V    |
| flags    | 4     | 39        | uint8 | 1     | 1      | bitfield (see below) |

`v_mag = sqrt(vd^2 + vq^2)` (applied voltage magnitude). `bemf = omega_e * psi_m`.
(`max_v_mag` and `br_no_duty_samples` were removed — max_v_mag is reconstructable
from STATUS_9's `v_bus`.)

**flags bitfield** (define as three 1-bit signals at byte 4):

| Bit | Name         | Meaning |
|-----|--------------|---------|
| 0   | shorted      | phases shorted (control_duty = duty forced to 0, not actively driving) |
| 1   | vd_saturated | `abs(vd_presat) > max_v_mag` (d-axis hit the voltage budget) |
| 2   | vq_saturated | `abs(vq_presat) > max_vq`, or the final 2D vector clamp bit — iq is voltage-limited |

Start bits for the flag bits (Motorola): bit0 = start bit 39, bit1 = 38, bit2 = 37.

### STATUS_11 — 0x4900 (DLC 8) — setpoint inputs + PI integrators

| Signal   | Bytes | Start bit | Type  | scale | factor | Unit |
|----------|-------|-----------|-------|-------|--------|------|
| id_set   | 0..1  | 7         | int16 | 100   | 0.01   | A    |
| iq_set   | 2..3  | 23        | int16 | 100   | 0.01   | A    |
| vd_int   | 4..5  | 39        | int16 | 100   | 0.01   | V    |
| vq_int   | 6..7  | 55        | int16 | 100   | 0.01   | V    |

`id_set`/`iq_set` are the commanded currents BEFORE MTPA/FW/limits (pair with
STATUS_8 targets to localize a divergence). `vd_int`/`vq_int` are the PI
integrator states (windup indicator when vd/vq saturate).

### STATUS_12 — 0x4A00 (DLC 8) — sensorless observer health

| Signal       | Bytes | Start bit | Type  | scale | factor | Unit   |
|--------------|-------|-----------|-------|-------|--------|--------|
| flux_mag     | 0..1  | 7         | int16 | 10000 | 1e-4   | Wb     |
| lambda_est   | 2..3  | 23        | int16 | 10000 | 1e-4   | Wb     |
| phase_used   | 4..5  | 39        | int16 | 10    | 0.1    | deg    |
| speed_fast   | 6..7  | 55        | int16 | 1     | 1      | rad/s  |

`flux_mag = NORM2(observer x1, x2)` (held near `foc_motor_flux_linkage` when
locked). `lambda_est` is the adaptive flux estimate. `phase_used` is the electrical
angle fed to the inverse Park transform. `speed_fast` is the fast electrical speed
estimate; note `bemf` (STATUS_10) == `speed_fast * foc_motor_flux_linkage`.

## Enum: control_mode (STATUS_8 byte 6)

From `mc_control_mode` in [datatypes.h](../datatypes.h):

| Value | Name |
|-------|------|
| 0 | CONTROL_MODE_DUTY |
| 1 | CONTROL_MODE_SPEED |
| 2 | CONTROL_MODE_CURRENT |
| 3 | CONTROL_MODE_CURRENT_BRAKE |
| 4 | CONTROL_MODE_POS |
| 5 | CONTROL_MODE_HANDBRAKE |
| 6 | CONTROL_MODE_OPENLOOP |
| 7 | CONTROL_MODE_OPENLOOP_PHASE |
| 8 | CONTROL_MODE_OPENLOOP_DUTY |
| 9 | CONTROL_MODE_OPENLOOP_DUTY_PHASE |
| 10 | CONTROL_MODE_NONE |

## Notes / gotchas

- **Signed float16 range:** `int16` saturates at ±32767 counts. With `scale = 100`
  that caps a signal at ±327.67 (A or V); with `scale = 10000` (flux) at ±3.2767 Wb.
  Values beyond that wrap — fine for the intended ranges here.
- **STATUS_6 is packet id 58 (0x3A), not 0x1C.** Easy to misremember as sequential.
- **DLC < 8:** STATUS_7 (5), STATUS_8 (7) and STATUS_10 (5) send short frames. Set
  the message length accordingly in the DBC or the trailing bytes will read garbage.
- These debug frames are firmware-specific to this branch; a stock VESC only emits
  STATUS_1..7 (and STATUS_7 only on recent firmware).
