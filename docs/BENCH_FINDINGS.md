# Bench Findings: 14-Sensor Array

Measurements taken on 2026-10-07 with the robot on USB serial (`sensor_14array_test`).
Raw recordings are in [`bench_data/`](../bench_data/), the sensor board Gerbers in
[`hardware/sensor_board_gerbers/`](../hardware/sensor_board_gerbers/).

## 1. Sensor board layout (from the Gerbers)

Extracted from the KiCad 10 Gerber X2 pad attributes with
[`hardware/parse_gerber_pads.py`](../hardware/parse_gerber_pads.py).
Board: 129 × 59 mm, 2 layers. Sensors D1–D14, IR LEDs L1–L14, MUX U1, connector J1.

| Index | Part | MUX input | X from centre (mm) | Set back from front row (mm) |
|---|---|---|---|---|
| S0 | D1 | I0 | −60.24 (left) | 35 |
| S1 | D2 | I1 | −50.24 | 20 |
| S2 | D3 | I2 | −40.24 | 10 |
| S3 | D4 | I3 | −30.24 | 0 |
| S4 | D5 | I4 | −22.62 | 0 |
| S5 | D6 | I5 | −12.62 | 0 |
| S6 | D7 | I6 | −5.00 | 0 |
| S7 | D8 | **I8** | +5.00 | 0 |
| S8 | D9 | I9 | +12.62 | 0 |
| S9 | D10 | I10 | +22.62 | 0 |
| S10 | D11 | I11 | +30.24 | 0 |
| S11 | D12 | I12 | +40.24 | 10 |
| S12 | D13 | I13 | +50.24 | 20 |
| S13 | D14 | I14 | +60.24 (right) | 35 |

* **MUX inputs I7 and I15 are not connected.** Firmware must map index → `{0,1,2,3,4,5,6,8,9,10,11,12,13,14}`.
  `line_follower_mux14` originally read I0–I13 directly (fixed in `sensors.cpp`).
* D1 is on the robot's **left** (confirmed on the robot).
* Sensor spacing is **not uniform**: steps alternate 7.62 mm / 10 mm in the front row; the outer 3 per side curve back (arc).
* Each sensor output has a pull-up `RDx` to 3.3 V; each LED has a series resistor `RLx`.
* J1 pinout: 1 GND, 2 S0, 3 S1, 4 COM, 5 +3.3V, 6 S3, 7 S2.
* In the firmware, positions are stored **left = positive** (`SENSOR_X` in `apex_lfr`).

## 2. White / black levels (12-bit ADC)

Track: black line, **30 mm wide**, on white. Higher reading = black.

| Sensor | White | Black | Range | Noise (std) | Status |
|---|---|---|---|---|---|
| S0 (D1) | 24 | 90 | **67** | ~1.5 | ❌ faulty |
| S1 (D2) | 1656 | 3625 | 1969 | 5.4 | OK |
| S2 (D3) | 1311 | 3639 | 2328 | 2.3 | good |
| S3–S7 | 256–446 | 3402–3544 | 3050–3255 | ≤ 2.3 | excellent |
| S8 (D9) | 24 | 181 | **157** | ~2.8 | ❌ faulty |
| S9, S10 | 437–496 | 3479–3535 | ~3040 | ≤ 1.4 | excellent |
| S11 (D12) | 1035 | 3611 | 2576 | 2.7 | good |
| S12 (D13) | 1238 | 3468 | 2229 | 8.1 | OK |
| S13 (D14) | 1437 | 3546 | 2110 | 14.3 | OK (noisiest on white) |

* Noise is 1–2 counts on a 2000–3300 count range: hundreds of distinguishable levels per sensor.
* White levels vary 256–1656 between sensors, so **per-sensor calibration is required**.
  The swept-back outer sensors (S1, S2, S11–S13) read 10–25% of range above their minimum on plain white.

## 3. Hardware fault: D1 (S0) and D9 (S8)

Both read ~24 on white and barely rise on black. With fast ADC reads, S8 also holds the previous channel's
voltage (702 at 0 µs settle → 214 at 50 µs, still falling), i.e. nothing actively drives the node.
**Most likely cause: pull-up RD1 / RD9 open** (unsoldered, cracked joint, or wrong value).
Check: on black, the sensor output should measure ~2.8 V like the working channels.

Until repaired, both are disabled in firmware (`SENSOR_DEAD`). With a 30 mm line, 3–4 sensors always see
the line, so the S8 gap causes no blind spot (verified in the slide test).

## 4. ADC timing and MUX settling

| Measurement | Value |
|---|---|
| `analogRead()` (STM32duino, re-inits ADC each call) | **158 µs** |
| Direct HAL read, ADC configured once (47.5-cycle sample) | **3.2 µs** |
| All 14 sensors, old code (3 × `analogRead` + 50 µs settle) | ≈ 7.3 ms → ~135 Hz |
| All 14 sensors, fast read, 10 µs settle | ≈ 0.26 ms |

Settle test with the line under the centre (black/white neighbours):
* Good sensors: 0 µs vs 50 µs differ by only 2–10 counts (< 0.3% of range); at **10 µs** it is 1–3 counts (noise level).
* Chosen: **10 µs settle, 2-sample average**.

## 5. Line position (slide test)

Position = weighted average of normalised values × sensor X (mm):

1. Normalise each sensor to 0–1000 with its calibrated white/black.
2. Subtract a **25% noise floor** and rescale to 0–1000. Without this, the outer sensors' white offset produced a
   false position (~+28 mm) with no line under the robot.
3. Line seen if any sensor > 400; otherwise "lost" and the last side is held at ±60 mm.

Result: smooth tracking from +50 mm (left edge, S1) to −60 mm (right edge, S13), clean "lost" detection, smooth
across the dead S8. The previous binary (on/off) method moved in 4–13 mm steps and sometimes stuck.
Position noise is well under 0.1 mm, so only a light low-pass on the D term is needed.

## 6. Control loop and gains

* Fixed 1 kHz loop; D = change in mm per 10 ms, filtered `0.7·old + 0.3·new`.
* The SW-I2C OLED takes tens of ms per frame, so it is not refreshed during a run.
* **Gain scale:** the old error was Σ(weight ±13 × value 0–1000) / active ≈ ±13,000, not ±13. Equivalent
  starting gains on the mm scale (factor ≈ 217): **Kp ≈ 6.5, Kd ≈ 43**. Not yet tuned on track.

## 7. Open items

- [ ] Repair RD1 / RD9, then set `SENSOR_DEAD` back to `false` for S0 and S8.
- [ ] Wheels-up direction check with the corrected gains (line left → left wheel slows).
- [ ] Confirm button pins: `apex_lfr` / `sensor_14array_test` use BACK=PB5, UP=PB3; `line_follower_mux14` has them swapped.
- [ ] Tune Kp / Kd / speed on track.
- [ ] Optionally store calibration and gains in flash so they survive a reset.
