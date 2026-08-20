# BYD Dolphin — Longitudinal ACC_CMD (0x32E) Findings

**Date:** 2026-08-19 · **Car:** BYD Dolphin (`CAR.BYD_SEAL`, bumpbump v10.1.0)
**All live-TX work performed with the car on a lift, all four wheels off the ground.**

Three phases were run. Every claim below is grounded in a capture file included
under `data/`. Where the data does not settle a question, that is stated rather
than smoothed over.

---

## Section 1 — Shadow mode: real factory ACC vs. unmodified bukapilot ACC

### How the comparison was made

`--shadow` creates `/tmp/LONGITUDINAL_SHADOW`, which makes the patched
carcontroller call `create_accel_command()` and **log its output without
appending it to `can_sends`**. The packed bytes are identical either way, so the
frame bukapilot *would* transmit is captured at zero bus traffic.

Simultaneously the capture script logs the factory's real 0x32E frames arriving
on bus 0 (`k="rx"`, `src=0`). Both sides are decoded through
`byd_acc_cmd_decode.py`, which reads signal positions from
`byd_general_pt.dbc` at runtime rather than hardcoding them.

**This compares bukapilot's existing, unmodified packer against the factory's
real frames. No attempt was made to rewrite the packer to match.** The
divergences below are pre-existing properties of the port, not artefacts of this
session's changes.

### Field-by-field divergence

From `data/phase3_engaged_injection.jsonl` (72 TX frames vs 670 factory RX frames):

| Field | bukapilot sends | Factory sends |
|---|---|---|
| `ENGAGE_BIT` | `0` (always) | `1` |
| `ACCEL_FACTOR` | `11, 12` | `7, 8, 9, 13, 14` |
| `DECEL_FACTOR` | `8` (always) | `0, 2, 3, 4, 5, 6` |
| `SET_ME_25_2` | `25` (always) | `25, 26, 27` |
| `SET_ME_1` | `1` (always) | `0, 1` |
| `ACC_ON_1` / `ACC_ON_2` | `1` | `0, 1` |
| `CMD_REQ_ACTIVE_LOW` | `0` | `0, 1` |
| `ACC_REQ_NOT_STANDSTILL` | `1` | `0, 1` |

Two findings worth separating from the rest:

- **`SET_ME_25_2` and `SET_ME_1` are not constants on the real car.** The port
  models them as fixed; the factory varies them. The DBC/port names are
  misleading.
- **`ACCEL_FACTOR` and `DECEL_FACTOR` use different value vocabularies
  entirely** — not merely different values within a shared range.

`ACC_ON_1` and `ACC_ON_2` also disagree with each other in factory frames
(609 vs 75 occurrences of `1` across 5999 frames in
`data/phase1_factory_rx_engage_bit.jsonl`), so they are not the duplicate pair
the naming implies. bukapilot sets both to the same value.

### ENGAGE_BIT deep-dive

`ENGAGE_BIT` (DBC bit 47) is defined in the DBC but **never set by
`create_accel_command()`**. It was initially the prime suspect for why an
injected frame might be ignored. Analysis of 5999 factory frames
(`data/phase1_factory_rx_engage_bit.jsonl`) shows it is **not an engagement
flag**.

Cross-tabulation against factory `ACC_ON_1`:

| ACC_ON_1 | ENGAGE_BIT | count | pct |
|---:|---:|---:|---:|
| 0 | 0 | 579 | 9.65% |
| 0 | 1 | 4811 | 80.20% |
| 1 | 0 | **0** | **0.00%** |
| 1 | 1 | 609 | 10.15% |

Against `carState.cruiseState.enabled` (nearest-timestamp join, median gap
14.4 ms, p95 26.6 ms against an 18 Hz source):

| cruiseEnabled | ENGAGE_BIT | count | pct |
|---:|---:|---:|---:|
| 0 | 0 | 579 | 9.65% |
| 0 | 1 | 4496 | 74.95% |
| 1 | 0 | **0** | **0.00%** |
| 1 | 1 | 924 | 15.40% |

```
P(ENGAGE_BIT=1 | cruiseEnabled=0) = 0.886
P(ENGAGE_BIT=1 | cruiseEnabled=1) = 1.000
```

**Brake correlation.** All five `ENGAGE_BIT=0` windows coincide with brake
application:

```
P(ENGAGE_BIT=0 | brakePressed) = 0.995   (n=571)
P(ENGAGE_BIT=0 | gasPressed)   = 0.000   (n=2646)
P(ENGAGE_BIT=0 | neither)      = 0.004   (n=2782)
```

**Conclusion: `ENGAGE_BIT` is an arming / not-inhibited signal, not an
engagement-state flag.** It is high 90.3% of the time including 4496 frames with
cruise off; it drops only under braking. The empty `(1,0)` cell in both tables
means it is a strict precondition — cruise never engages and `ACC_ON_1` never
sets while it is low — which is the signature of a permissive bit.

**Explicitly flagged as weak:** ENGAGE_BIT went high ~14.85 s before the single
`cruiseEnabled 0→1` transition in the capture. **This rests on one transition
and is not a characterised delay.** Do not cite it as a timing figure. The
`1→0` transition coincided with cruise disengaging (within 22 ms), but only
because the brake caused both.

---

## Section 2 — Engage injection: strategy and result

### Safety strategy (the actual gate stack)

**Physical precondition — human-verified, not software-checkable.** Wheel speed,
yaw rate and every CAN signal look identical on a lift and on a road. No script
can tell them apart; that is precisely why the lift makes this safe. The
operator confirms by eye and types `car is on the lift` before the enable file
is created.

**Per-frame gate** (`byd_longitudinal_kill_gate.py`), re-asked every control
frame — nothing arms persistently. All must pass:

1. `/tmp/LONGITUDINAL_KILL` absent — checked first and independently, so it
   works even if the monitor has hung.
2. `/tmp/LONGITUDINAL_CAPTURE_ENABLE` present — owned solely by the capture
   script, removed on every exit path.
3. `/tmp/longitudinal_health.json` fresher than 400 ms **and** reporting bus-0
   `TEC == 0` with `busOff`/`errorPassive`/`errorWarning` all clear.
4. Optional `--engaged-only`: the car must be engaged this frame.

**Fail-closed on everything.** Missing file, stale file, bad JSON, missing key,
`OSError` — every path returns `False`. There is no code path returning `True`
as a fallback. This deliberately **inverts** the steering hook's convention:
that one falls back to stock because stock lateral is the safe state; here the
safe state is *not transmitting*, so an import failure must suppress TX.

**Redundant abort**, independent of the per-frame gate: the capture script
deletes the enable file itself the instant it observes a fault, at 20 Hz.

**Manual kill path**, independent of both: `touch /tmp/LONGITUDINAL_KILL` from a
second shell. Plus, under `--engaged-only`, the brake pedal becomes a direct TX
kill because it drops engagement. `/tmp` is wiped on reboot, so no gate file can
survive a restart.

### Result

```
TX window        1.954s .. 3.371s   (1.417s, 72 frames, 50.1 Hz)
TEC 0 -> 84      in 1.430s of transmitting   (~59 TEC/s)
ABORT            t=3.393s on TEC=84
busOff           NEVER reached
after abort      TEC decayed to 0, bus fully recovered, 17 total errors
```

Extrapolating 59 TEC/s: error-passive (128) at ~2.2 s, bus-off (256) at
**~4.4 s**. The abort cut transmission roughly **3 seconds short of bus-off**.
The engaged-only window did not prevent the collision — the gate did.

**Frames did reach the bus.** TEC cannot accumulate without driving the wire.
Solving `8E − S = 84` with 72 successes gives ~19 errors; `totalTxLostCnt` was
`0/s`. So ~72 well-formed frames with valid BYD checksums reached every node on
bus 0. (Our payloads never appear in the `can` RX stream — **expected**, since a
CAN controller does not self-receive. That absence is not evidence of
non-delivery.)

**No evidence the ECU acted on them.** The car followed the factory command
throughout: factory `ACCEL_CMD` went 0→23 before we started, held 23 through the
injection, decayed afterward; `vEgo` climbed 4.54 → 7.09 m/s while we commanded
`ACCEL_CMD` 0–2. No disengage, no fault (`events=[]`, `alert1=''` throughout).
**Unresolved:** `aEgo` was low during injection (+0.08, +0.14) and rose sharply
after (+0.51, +0.89) — consistent with partial suppression, but confounded with
the normal ACC engagement ramp, from one run with no control. Do not cite in
either direction.

**Scope caveat:** injection occurred during the *engage transition* — factory
`ACC_ON_1` was still 0 throughout the TX window, going 1 only at t≈4–5 s.
Steady-state engaged injection was not tested.

### Mechanism: bit-3 divergence, and why parked was slower

Measured across all 72 transmitted frames against the nearest factory frame:

```
bukapilot byte0 (ACCEL_CMD) : 100, 101, 102
factory   byte0             : 123
byte0 equal                 : 0/72
first differing BIT         : 3   (min = median = max, all 72 frames)
```

Identical CAN IDs tie through arbitration; the data phase then decides. The
first bit where one node transmits recessive while another transmits dominant is
a bit error. Diverging at **bit 3 — the 4th data bit on the wire** — produces a
collision on nearly every overlap of the two 50 Hz streams.

**Parked, by contrast, bukapilot and the factory both send byte0 = 100 —
identical.** The frames only diverge deep in the payload, so far fewer overlaps
produce an error. That explains why the 07-27 parked test took *tens of minutes*
to reach bus-off while this test reached two-thirds of the way in ~1.4 s.
Engagement removes the accidental protection that byte-0 agreement provided.

### Payload correctness does NOT explain or fix this collision

This point must not blur with Section 1. The collision is **arbitration-level
and payload-independent in kind**: two nodes transmitting the same CAN ID
concurrently will collide whenever their data differs *anywhere*. Section 1's
findings (ENGAGE_BIT, the FACTOR vocabularies, the non-constant `SET_ME_*`
fields) explain why the ECU would likely *reject* our frame as malformed. They
are a **separate, additional** problem.

Making the payload byte-identical would not grant control either — see the
bit-identical echo note in Section 3. Fixing Section 1 does not make Section 2's
problem go away.

### Known limitation in the logging tool (being fixed)

The capture script's 1 Hz summary for this run reported
`TEC stayed 0 the whole run` / `ONSET: none` / `RESULT: NO FAULT within 3
minutes` — **all false.** TEC is a gauge that decays (−1 per successful TX), and
the entire 0→84→0 excursion happened between two 1 Hz samples. Only the 20 Hz
fast path observed it, which is why the abort fired correctly.

**The `abort` line in that run's summary is trustworthy; the ONSET/RESULT lines
are not.** This report is written from the 20 Hz data and the full-rate JSONL,
not from those lines. The bug is fixed in the version shipped here (fast-path
onset tracking, `PEAK TEC` reporting), with regression coverage as case 8 of
`laptop-side/tests/test_capture_session.py`.

---

## Section 3 — Conclusion

**Dual-transmitter injection onto 0x32E remains non-viable**, now with the added
measured finding that **engagement makes the collision faster, not milder** —
~59 TEC/s and ~4.4 s to projected bus-off, versus tens of minutes when parked —
because engagement destroys the byte-0 agreement that was incidentally limiting
collisions.

### Scope of this conclusion

This evaluates **one approach: injecting alongside a live, untouched factory ACC
ECU.** It does **not** evaluate, and does not rule out, a configuration in which
the factory ACC source is silenced or isolated first — the relay-style approach
that works for steering.

That path remains **untested and unattempted, not disproven.** The 07-29 UDS
investigation (CONTEXT.md) found standard-address diagnostic silencing is not
reachable on any Panda-visible bus and that Kedua's `set_obd()` is a
non-functional stub. Silencing would therefore require either physical
disconnection of the ACC ECU or a dealer-obtained diagnostic address. Neither
was attempted.

### Open items

1. **Logging-tool bug fix** — done in the version shipped here, but the fixed
   `byd_long_conflict_capture.py` was **never redeployed to the device** (the
   device was fully reverted after the test). Any future run must deploy this
   version, not the one that produced the misleading 12:41 summary.
2. **Bit-identical echo approach** — discussed, never built. Transmitting a
   frame byte-identical to the factory's, including a real-time replica of its
   COUNTER and CHECKSUM, would be theoretically collision-free. It also confers
   **zero control authority** — an exact copy of what the factory already says
   changes nothing — so it is a curiosity, not a path to longitudinal control.
3. **ECU silencing is the real prerequisite.** No amount of payload correction
   or injection timing addresses the fundamental problem of two transmitters on
   one ID. This is the actual blocker for any future attempt at real
   longitudinal control on this car.

### Reproducing the analysis

```bash
cd bumpbump_clone
python3 ../claude/byd_acc_cmd_decode.py --selftest      # 51 signal round-trips
python3 ../claude/byd_acc_cmd_decode.py --sweep         # offline reference table
python3 ../claude/byd_acc_cmd_decode.py --capture ../Modified_Kommu/LongitudinalCapture_20260819/data/phase3_engaged_injection.jsonl
python3 ../claude/byd_acc_cmd_decode.py --shadow  ../Modified_Kommu/LongitudinalCapture_20260819/data/phase2_shadow_parked.jsonl
```
