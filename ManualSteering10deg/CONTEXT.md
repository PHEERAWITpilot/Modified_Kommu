# Kommu.AI — Context for AI Assistants

> **LAST UPDATED: 2026-07-09 (staging_v2 decision revision).** The project reset
> its foundation (see "WHY WE RESET"), then discovered that stock `staging_v2`
> already runs code functionally identical to bumpbump's `cam_lka` for the
> Dolphin's lateral control — verified line-by-line, only cosmetic differences.
> **Decision: stay on `staging_v2`. Do NOT migrate to bumpbump.** The Dolphin's
> known engagement bug (`cancel_pressed`/`acc_pressed` bit collision, unpatched
> on `staging_v2`) has a verified, low-cost workaround: engage via the **ICC
> steering-wheel button**, not the SET-/RES+ knob (see "ENGAGEMENT WORKAROUND"
> below). We are commanding a **desired steering angle** into the existing
> carcontroller's own encoder, directly on `staging_v2`, rather than hand-writing
> steering frames or waiting on a bumpbump port. Read "WHAT WE KNOW (carries
> forward)" first; that is the real starting capital. Read "WHY WE RESET" once,
> then move on — do not re-litigate it.

---

## THE GOAL (one sentence)
Get the BYD Dolphin steering under our control **directly on stock `staging_v2`**
by commanding a **desired steering angle** through the existing carcontroller's
own encoder — reusing everything we already learned about the car, and NOT
rebuilding CAN encoding, checksums, or fingerprints from zero, and NOT migrating
to bumpbump (decided unnecessary — see below).

## WHY WE RESET (read once, then move on)
Earlier work ran on a hand-built "flat" snapshot port and chased a long
controlsMismatch/engagement-drop investigation. That line of debugging reached a
point of diminishing returns: the evidence indicated the Panda firmware never
actually changed across the drives in question, so the "why did it work then but
not now" question would not resolve into a clean, actionable fix. Rather than keep
excavating a stale port and firmware forensics, the developer chose to **refresh
the whole foundation** — allow the device auto-update to bring a clean modern base.
The original plan was to then port onto bumpbump `cam_lka`; after the reset, that
turned out to be unnecessary (see "DECISION: STAY ON staging_v2" below) — stock
`staging_v2` already runs functionally equivalent code. The old flat-port
debugging history is intentionally NOT carried into this document. If a future
note tempts you back into cancel_pressed / reflash / "how did Test 1.0 hold 80
min" — that is the OLD problem space. Don't. The new path sidesteps it by
starting clean, and the ICC-button workaround (below) handles the one part of
that old bug that still matters day-to-day.

---

## WHAT WE KNOW (carries forward — this is the starting capital)

These facts were verified firsthand on the real Dolphin over prior phases. They
are vehicle/hardware truths, independent of any particular software port, so they
remain valid on the fresh base. Trust them; re-confirm only if something visibly
conflicts.

### The car & hardware
- Car: **BYD Dolphin** (RHD, EV, ~1580 kg, wheelbase ~2.70 m).
- ADAS device: **Kommu KommuAssist 2** (RK3588, runs bukapilot, a fork of openpilot).
- CAN interface: **Panda board, type `kedua`** (Kommu custom), on the OBD-II port.
- A **KommuRelay** sits between the BYD factory ADAS camera and the Kommu unit —
  this is what lets us inject our steering frame in place of the camera's.
- Steering is **ANGLE-controlled** (steerControlType = angle), NOT torque. This is
  the single most important architectural fact: we command a desired ANGLE.

### CAN facts (validated on the real car)
- DBC: **`byd_general_pt.dbc`** (shared across BYD models) — confirmed accurate.
- **Steering command:** `STEERING_MODULE_ADAS`, ID **0x1E2 (482)**, 8 bytes.
- **Steering angle readback:** `STEER_MODULE_2`, ID **0x11F (287)**, 5 bytes
  (Dolphin-specific — the Seal's is 8 bytes).
- **ACC / longitudinal:** `ACC_CMD`, ID **0x32E (814)**, 8 bytes.
- **Pedals:** ID **834** — gas = data[0] > 0, brake = data[1] > 0.
- **Wheel speed:** ID **496** (used for vehicle_moving / speed in the safety model).
- **Buttons:** `PCM_BUTTONS` ID **0x3B0 (944)** — SET=data[0] bit4, RES=data[0] bit3,
  LKAS/ICC=data[0] bit6, ACC_ON=data[2] bit3. **No cancel button exists** on the
  Dolphin — disengage is via the **brake pedal** (834) only.
- ⚠️ **Two physically separate steering-wheel buttons both engage cruise, on
  DIFFERENT bits of the SAME 0x3B0 message (verified 2026-07-09):**
  - **SET-/RES+ combo** (the main knob) — sets `SET`/`RES`/`ACC_ON` bits.
    Engaging this way sets `data[2] bit3` (`ACC_ON`), which is the EXACT bit
    the `cancel_pressed`/`acc_pressed` collision bug is keyed on — every
    engagement via this path has historically triggered `controlsMismatch`
    seconds later, while the raw `byd.h` bug remains unpatched.
  - **ICC button** (steering-wheel icon, separate from the knob cluster) —
    sets `data[0] bit6` only (`icc_pressed` in `byd.h`, already OR'd into
    `controls_allowed` alongside `set_pressed`/`res_pressed`/`acc_pressed`).
    Confirmed via real drive + CAN capture: engaging through ICC does NOT
    touch `data[2] bit3` at all, so it structurally never exercises the
    collision bug's trigger condition. Verified result: 94.4s continuous
    clean engagement, zero mismatch/fault events, ending only in a genuine
    driver-initiated disengage (pedal + steer override).
  - **Practical workaround (not a guarantee):** prefer the ICC button over
    SET-/RES+ while the raw collision bug is unpatched on whatever branch is
    live. This does NOT prove ICC is immune in all cases — only that ACC_ON
    happening to be 0 during an ICC press avoids the bug that time; if
    ACC_ON were independently high at the same moment, the collision could
    still theoretically fire. Still, it's a real, low-cost, evidence-backed
    behavioral mitigation, distinct from any firmware fix or masking.
- **Counter:** 0–15 cycling. ID 482 upper nibble of byte[6]; ID 814 lower nibble.
- **STEER_ANGLE scale: degrees × 10 → raw.** e.g. 7.5° → raw 75 (0x4B).
- Dolphin vs Seal fingerprint: ~96% overlap; Dolphin-unique IDs include 792, 1050,
  1296. Real Dolphin fingerprint = 72 CAN IDs (captured Phase 2).

### The checksum (do NOT hand-roll — verified 2026-07-02)
- The real algorithm is **`byd_checksum(address, sig, d)`** — a **nibble-sum keyed
  on byte_key = 0xAF**. It is **computed automatically by opendbc's CANPacker** from
  the DBC (SignalType.BYD_CHECKSUM). `create_can_steer_command` does NOT call it by
  hand; the packer applies it.
- To VERIFY frames offline, decode through **opendbc's own CANParser/CANPacker**,
  **never cantools** (cantools rejects `byd_general_pt.dbc` — it has a C-style
  `/* comment */` opendbc parses fine).
- ⚠️ A disproven `(sum ^ 0xFF)` formula appears in some OLD helper scripts
  (display-only). It is WRONG. Do not reintroduce or copy it.

### Known-good CAN test frames (verify any encode against these)
```
ID 482 (0x1E2) STEERING_MODULE_ADAS — 8 bytes:
  2c51cb8bff64efda → checksum=0xDA, counter=14
  2c51cb8bff64ffca → checksum=0xCA, counter=15
  2c51cb8bff647f4a → checksum=0x4A, counter=7
ID 814 (0x32E) ACC_CMD — 8 bytes:
  646464805048f9c2 → checksum=0xC2, counter=9
  646464805048fac1 → checksum=0xC1, counter=10
  646464805048f2c9 → checksum=0xC9, counter=2
```

### The steering/longitudinal split (why lateral-only is valid)
Confirmed by a CAN source-bus capture + read-only audit — steering and speed are
genuinely SEPARATE sources on the Dolphin, so steering can be developed in isolation:
- **STEERING (482):** camera-origin (arrives bus 2); openpilot injects its
  replacement on **bus 0** via the relay. **This is the path we control.**
- **LONGITUDINAL (814 ACC_CMD, 834 pedals):** car-native, bus 0, emitted by the
  car's own powertrain ACC ECU. Openpilot only READS these. We do NOT transmit them.
- **No radar in play:** `byd_radar_fd.dbc` doesn't exist; radar_interface is an
  empty stub; radarOffCan = True.
- **Speed stays with the car's STOCK ACC.** We do lateral (steering) only. This is
  the correct, confirmed architecture — not a limitation to engineer around.

### Two hard physical gates (confirmed, do not try to defeat)
1. **The EPS rejects angle commands at standstill.** There is NO parked-steering.
   Any steering test requires the car ENGAGED and MOVING. Parked captures only ever
   show the neutral frame (steer_req = False).
2. **The Panda only allows steering when stock ACC is engaged** (controls_allowed).
   Engagement must come through the legitimate path (stock ACC on). Never spoof it.

### Proven capability (the reason this is not from scratch)
- We can **encode a valid 0x1E2 steering frame** that is byte-for-byte identical to
  bukapilot's own live bus-0 output (verified via read-only compare, counter +
  checksum masked).
- We have a working **desired-angle → encoder → bus 0** data path concept (the
  "substitute one number" approach) validated on the older port.
- **Angle command is very likely sufficient to steer the car** — we do NOT need to
  author the full steering command by hand; we feed the desired angle into the
  car port's OWN encoder and let it build the frame.

---

## DECISION: STAY ON staging_v2 (do not migrate to bumpbump)

### Why bumpbump turned out to be unnecessary
After the reset, `staging_v2`'s BYD carcontroller was compared line-by-line
against bumpbump's `cam_lka/carcontroller.py` (2026-07-09). Result: **functionally
identical** for the Dolphin's lateral control —
- Same `_compute_apply_angle` method, same tuple return
  `(apply_angle, steer_angle_limited)`, same 2Hz lowpass → speed-scheduled
  `apply_std_steer_angle_limits` → ±10° measured-angle clamp chain.
- Same `create_can_steer_command(packer, steer_angle, steer_req, is_standstill,
  ecu_fault, recovery_btn)` signature.
- Same angle control (`steeringAngleDeg`, not torque).
- Same `CarControllerParams` (`STEER_ANGLE_MAX=120°`, same speed-scheduled rate
  tables), same `CAR.BYD_SEAL`-with-Dolphin-docs arrangement (no `CAR.BYD_DOLPHIN`
  enum on either branch).
- Only differences: file layout (flat vs `cam_lka/`/`mpc_lka/` split — cosmetic)
  and two platform-constant-set names (`BYD_ATTO_STYLE_PLATFORMS`,
  `BYD_OP_LONG_PLATFORMS`) that don't affect the lateral-only Dolphin path.

**Conclusion: there is no `cam_lka` porting task.** The angle-command
architecture the project was planning to bring over from bumpbump is already
running, as-is, on stock `staging_v2`. The desired-angle substitution hook can
be wired directly into `staging_v2`'s existing `carcontroller.py`.

### The one real difference: byd_relax_controls
Bumpbump's `byd.h` has `byd_relax_controls` — a mechanism that unconditionally
forces `controls_allowed = true` at the end of every `byd_rx_hook()` call for
safetyParam 2/4, which masks the `cancel_pressed`/`acc_pressed` bit-collision bug
(see below). **`staging_v2`'s `byd.h` has NO such mechanism** — the bug is raw
and unpatched here. This was a deliberate, considered trade (see "ENGAGEMENT
WORKAROUND" below) — not an oversight.

---

## ENGAGEMENT WORKAROUND (do this, don't reflash)

### The bug, briefly
`byd.h` reads `acc_pressed` and `cancel_pressed` from the SAME bit
(`data[2] bit3` on PCM_BUTTONS, CAN 0x3B0/944). The Dolphin has no cancel
button, so this bit should only ever mean "ACC pressed" — but the code doesn't
know that, and treats every rising edge as both "engage" and "cancel"
simultaneously. Engaging via the **SET-/RES+ combo knob** sets this exact bit,
which is why that path has historically triggered `controlsMismatch` seconds
after every engagement.

### The workaround — verified 2026-07-09, real drive data
There are **two physically separate buttons that both engage cruise**, on
different bits of the SAME 0x3B0 message:
- **SET-/RES+ combo** → sets `data[2] bit3` (`ACC_ON`) → hits the bug.
- **ICC button** (steering-wheel icon, separate from the SET/RES/knob cluster)
  → sets `data[0] bit6` only (`icc_pressed` in `byd.h`, already OR'd into
  `controls_allowed`) → **does NOT touch `data[2] bit3` at all**.

Verified via real CAN capture (route `2026-07-09--03-36-48`, t≈511s):
engaging via ICC held `latActive`/`controlsAllowed` continuously for **94.4
seconds**, zero mismatch/fault events, ending only in a genuine
driver-initiated disengage (brake/gas + steer override).

**Rule: always engage via the ICC steering-wheel button, never SET-/RES+,**
until/unless `byd_relax_controls` (or an equivalent structural fix) is ported
later. This is a real, low-cost, evidence-backed mitigation — not a proof the
bug is fixed. If `ACC_ON` ever happened to be independently high at the exact
moment of an ICC press, the collision could theoretically still fire; this just
means the ICC path never has to set that bit itself.

### Terminal monitor shows this live
`claude/byd_terminal_monitor.py` (deployed at `/data/openpilot/`) displays all
four PCM_BUTTONS bits (SET/RES/ACC_ON/ICC) plus a derived label —
`ENGAGE (ACC_ON)` vs `ENGAGE (ICC)` vs `ACC+`/`ACC-` vs `UNKNOWN: ...` for
anything unexpected. Run it during any drive/test session to confirm which path
is actually being used to engage. See "MONITORING TOOLS" below.

---

## HOW WE COMMAND STEERING (verified directly on staging_v2)

### The core mechanism
`staging_v2`'s BYD carcontroller already builds and encodes the steering frame
correctly. Our job is to substitute ONE input: the **desired steering angle**,
fed in BEFORE the limiter chain, so it rides the car port's own tested path
(lowpass → standard angle limits → measured-angle clamp → encoder → bus 0).
Per-frame, the correct CarControl for the Dolphin is:
```
CC.enabled = True, CC.latActive = True, CC.longActive = False,
CC.actuators.accel = 0.0, CC.actuators.steeringAngleDeg = <angle>   # NOT torque
```
The controller reads `steeringAngleDeg` and IGNORES `actuators.torque` (it's the
angle car). Any example that writes torque is a NO-OP here — borrow
input-mapping ideas from such examples, never their torque line.

### Carcontroller specifics worth knowing before you touch it
- `_compute_apply_angle` is a METHOD returning a TUPLE
  `(apply_angle, steer_angle_limited)`, with a 2 Hz lowpass pre-stage, the shared
  `apply_std_steer_angle_limits` against a typed `AngleSteeringLimits`
  (`CarControllerParams.ANGLE_LIMITS`), then a ±10°-of-measured clamp.
- `create_can_steer_command(packer, steer_angle, steer_req, is_standstill,
  ecu_fault, recovery_btn)` — checksum + counter are NOT computed inside; the
  packer auto-applies them.
- The encoder was VERIFIED on this tree: it round-trips all six known-good frames
  BYTE-IDENTICAL through opendbc's own CANPacker/CANParser.
- Rate/angle limits live in `values.py` as SPEED-SCHEDULED tables (breakpoints
  0/5/15 m/s) plus `STEER_ANGLE_MAX = 120°`.
- ⚠️ `CAR.BYD_SEAL` CarSpecs are SEAL placeholders (mass ~2180-2316, wheelbase =
  2.92, steerRatio = 16.0) — NOT the Dolphin's real 1580 kg / 2.70 m.
  Override/verify these before trusting any vehicle-model math. There is NO
  separate `CAR.BYD_DOLPHIN` enum — the Dolphin fingerprints as `CAR.BYD_SEAL`
  (a "BYD Dolphin 2023-26" docs entry sits nested under it), reached today via
  the `CarName` override param (`/data/params/d/CarName` = "BYD Dolphin
  2023-26") since `fingerprints.py` has no dedicated Dolphin fingerprint table.
- A previously-known regression (`×1.02` steer-angle scale factor dropped, net
  deg→raw = 10.0 not 10.2) is present here too — negligible (~2%) for a
  nudge-and-watch manual tool. Do NOT add a compensating factor; let
  liveParameters re-calibrate on the first drive.
- **Separate, standing issue (unrelated to the above, not yet fixed):** the
  Panda's real hard steering ceiling (`byd.h`'s `max_angle=450` raw → **45°**)
  is far tighter than what `values.py` believes it can command
  (`STEER_ANGLE_MAX=120°`), and the rate-limit tables diverge too (Panda allows
  `[28,26,22]` both directions; Python self-limits to `[6,4,3]` up / `[8,6,4]`
  down). If the desired-angle hook ever commands close to or above 45°, expect
  the Panda to reject/block that TX (`safetyTxBlocked` will rise) — this is a
  live, standing gap, not something the reset changed.

## MONITORING TOOLS

### Terminal car-state monitor (built 2026-07-09, deployed and verified live)
`claude/byd_terminal_monitor.py` — a plain-curses (no display needed) live
dashboard, safe to run alongside bukapilot with zero USB/CAN contention (pure
`cereal.messaging.SubMaster` subscriber — confirmed no `Panda(...)` usage
anywhere). Shows: speed, steering angle, gear (P/D/N/R + fallback for
sport/low/brake/eco/manumatic/unknown), `LAT`/`LONG`/`CRUISE` as three
independent always-visible fields, pedals, PCM_BUTTONS raw bits (SET/RES/
ACC_ON/ICC) plus the derived engage-path label, last `onroadEvents` name, and a
live CAN panel (0x11F/0x1E2/0x32E) with the REAL `byd_checksum` (0xAF nibble-sum
— verified against all 6 known-good frames, not the disproven `sum^0xFF`
formula some old scripts still carry).
```bash
ssh kommu@<ip>
python3 /data/openpilot/byd_terminal_monitor.py
```
Quit with `q`/Esc/Ctrl-C. Requires a real interactive SSH shell (curses needs
a TTY) — don't run it as a one-shot `ssh host 'python3 ...'` remote command.

### Device access
- Device runs at a **DYNAMIC IP** (varies by network — home WiFi vs phone hotspot).
  Do NOT hardcode it. Find it via the KommuAI app or `nmap -sn <subnet>/24`.
- `ssh kommu@<ip>` (username is **kommu**, not comma). Password fallback = the
  dongle ID if key auth fails.
- Device openpilot path: `/data/openpilot`. BYD files under
  `/data/openpilot/opendbc_repo/opendbc/car/byd/`.
- Deploy with `scp`; file changes take effect after a reboot.

### Bukapilot holds the Panda (USB) while running
- Any script that opens the Panda DIRECTLY (firmware read, raw CAN scan) needs
  bukapilot STOPPED first (`pkill -f manager.py`), or you get LIBUSB_ERROR_BUSY.
- Reading bukapilot's **cereal** stream (carState, carControl, etc.) does NOT
  conflict — bukapilot stays running for that.
- Restart: `cd /data/openpilot && nohup ./launch_openpilot.sh &`
  (source `/etc/profile` first in a plain SSH shell so the venv is active).
- **Only stop bukapilot while PARKED.** The car reverts to stock driving when it stops.

### Cereal schema (bukapilot v10.x — where fields actually live)
- `latActive`, `longActive` → **carControl** (NOT controlsState)
- `enabled`, `alertText1`, `state` → **selfdriveState**
- `vEgo`, `steeringAngleDeg`, `cruiseState.enabled`, `brakePressed`, `gasPressed`
  → **carState**
- `steerRatio`, `stiffnessFactor`, `angleOffsetAverageDeg` → **liveParameters**
  (auto-calibrated while driving)
- CAN frames → **can** (pkt.address, pkt.dat, pkt.src)
- Log read pattern (verified): decompress `rlog.zst` with zstandard, then
  `capnp_log.Event.read(f)` in a loop. Do NOT use LogReader.

### Rules when writing CAN-related code (non-negotiable)
- NEVER invent CAN encode/decode/checksum. Read the existing carcontroller
  implementation first and reuse it.
- Verify any encode against the known-good frames above, through opendbc's own
  CANParser — never cantools.
- `packer.make_can_msg` returns a **3-tuple** (addr, dat, bus) on this device.

### Auto-updater — verified mechanism (tested directly 2026-07-08, do not re-derive)
The updater is `system/updated/updated.py`, run by `manager.py` as a gated child
process (`only_offroad` — needs `IsOffroad=1`/`IsOnroad=0`, the normal parked
state). It fetches `origin/<UpdaterTargetBranch>` and, on a real mismatch,
force-resets with `git checkout --force -B <branch> FETCH_HEAD`.

**To DISABLE (stop any reset from happening):**
```bash
printf '1' > /data/params/d/DisableUpdates
```
Confirmed twice by direct test (reboot with real local commits + test files
present): with this set, NOTHING is touched, across a full reboot. `updated.py`
checks this param first (`updated.py:456`) and exits immediately if set. This is
the reliable way to freeze the base while developing — trust it.

**To RE-ENABLE and actually force a reset (e.g. to intentionally return to a clean
branch), removing `DisableUpdates` is NOT enough by itself:**
1. `rm -f /data/params/d/DisableUpdates`
2. Set the target branch through the **params API, not a raw shell redirect** —
   a plain `echo > /data/params/d/UpdaterTargetBranch` was observed to silently
   NOT persist (reverted with no reboot involved). Use:
   ```bash
   python3 -c "
   from openpilot.common.params import Params
   Params().put('UpdaterTargetBranch', 'staging_v2')
   "
   ```
3. Force an immediate fetch check (don't just wait — the daemon's own cycle can
   take a long time to align): `bash /data/openpilot/scripts/update_now.sh`
   (sends SIGHUP to the updater process).
4. **Watch for the real "ready" signal before rebooting** — do NOT reboot just
   because you removed the disable flag:
   ```bash
   cat /data/params/d/UpdaterFetchAvailable   # want: 1
   cat /data/params/d/UpdaterNewDescription   # want: shows the REMOTE branch/hash, non-empty
   ```
   If `UpdaterNewDescription` is empty, no fetch has completed yet and a reboot
   will likely do nothing (observed directly — rebooting with these still empty
   left the branch/commits completely unchanged). Re-run step 3 and wait
   (10–15s at a time) until `UpdaterNewDescription` shows the real target hash.
5. Only once that's populated, `sudo reboot` — the reset will actually apply.

**Blast radius of a real reset (verified directly, more precise than "git wipes
tracked files"):** it touches (a) EVERYTHING under `/data/openpilot`, tracked or
not — this is a directory swap via a staging area (`/data/safe_staging`), not a
plain `git checkout`, so untracked files inside the repo are NOT safe — and
(b) loose files sitting directly in `/data`'s root. It does **NOT** touch existing
subdirectories under `/data` that aren't part of the openpilot/params/staging
machinery (confirmed: `/data/docker`, `/data/misc`, `/data/ssh`, `/data/stats`
survived a real reset untouched, with original timestamps).
**Practical rule:** keep backups/scratch work in a dedicated subdirectory under
`/data` (e.g. `/data/kommu_backup/`) — never as loose files in `/data` root, and
never inside `/data/openpilot` — if you want it to survive an update you didn't
mean to disable.

### Backing up work (the device canNOT push to GitHub)
The device has no working GitHub push key. The durable backup method: `git bundle`
on the device → `scp` the bundle to the laptop → push from the laptop (which is
authenticated). Personal backup remote: **PHEERAWITpilot/dolphin_port**. Never push
to kommuai/*.

---

## SAFETY (the steering path is the high-care path)
The steering frame (0x1E2) is the ONE thing we actively transmit and the manual
tool moves. "Different bus" or "brake covered" do NOT downgrade the care here.
- The manual-steer approach changes ONE number (desired angle) fed into the car
  port's OWN encoder. It NEVER encodes CAN itself, NEVER opens the Panda, NEVER
  manufactures engagement, NEVER defeats the Panda gate or the EPS standstill gate.
- It only acts when the car is genuinely lat_active AND moving AND stock ACC is
  engaged AND an explicit enable file is present AND the target is fresh. Any gate
  failing ⇒ stock steering, unchanged.
- Kill paths: `rm /tmp/MANUAL_STEER_ENABLE` (instant revert to stock) + the brake
  pedal. Everyone present must know both before the car moves.
- Longitudinal is OUT of scope (lateral-only). Do NOT enable
  openpilotLongitudinalControl or transmit on 814 / 834.
- The live moving-car test is the DEVELOPER'S in-person call, not AI-authorized.
  AI builds and validates through the read-only stages; the person in the seat
  decides when a real frame goes to a moving wheel.

---

## PROJECT PHASES
- [x] Phase 1 — Simulation & CAN prep
- [x] Phase 2 — Hardware arrival & CAN fingerprinting
- [x] Phase 3 — Car port on device (confirmed: stock `staging_v2` already works,
      no bumpbump/cam_lka migration needed — see "DECISION" above)
- [ ] Phase 4 — Lateral (steering) control + desired-angle command  ← WHERE WE ARE
- [ ] Phase 5 — Longitudinal (speed) — deferred
- [ ] Phase 6 — Full autonomy & refinement

## STARTING POINT FOR THE NEXT SESSION
See the companion **SESSION_HANDOFF** for the concrete first steps. In short: the
Dolphin already stands up on stock `staging_v2` (fingerprint → `CAR.BYD_SEAL` via
the `CarName` override, safetyParam 2, angle control, confirmed healthy and able
to engage cleanly via the ICC-button workaround). The remaining real work is
wiring the desired-angle substitution hook into `staging_v2`'s existing
carcontroller and validating it read-only before anything moves.
