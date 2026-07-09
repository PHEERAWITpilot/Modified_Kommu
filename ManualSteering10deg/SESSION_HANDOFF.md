# Kommu.AI — Session Hand-off (2026-07-09) → START HERE

Read this first, then read **CONTEXT.md** (2026-07-09, "staging_v2 decision
revision" — header says so explicitly). This file is the "where we are and
what to do next" pointer; CONTEXT.md is the durable reference.

---

## ⚠️ FIRST: what NOT to do
Two traps to avoid opening this session:
1. **Do not investigate controlsMismatch, cancel_pressed, or Panda reflashing.**
   This was resolved practically (see below) — don't reopen it.
2. **Do not plan or start a bumpbump/cam_lka migration.** This was seriously
   considered and explicitly DECLINED — stock `staging_v2` already runs code
   functionally identical to bumpbump's `cam_lka` for the Dolphin's lateral
   control (verified line-by-line). There is no porting task. If you find
   yourself about to clone bumpbump or reference a `cam_lka/` subdirectory on
   the device — stop, re-read CONTEXT.md's "DECISION: STAY ON staging_v2"
   section, you're about to redo work that was already ruled unnecessary.

Your job this session is to **build the desired-angle substitution hook
directly into `staging_v2`'s existing carcontroller.** Nothing else needs to
happen first.

---

## 30-second summary of where things stand
- Device was factory-reset (auto-update) to stock `staging_v2 @ ab87942`.
  Confirmed clean — no local `dolphin-port` work carried over (safely backed
  up off-device beforehand: bundle + `PHEERAWITpilot/dolphin_port` on GitHub).
- The Dolphin stands up on `staging_v2` right now via a `CarName` param
  override ("BYD Dolphin 2023-26" → fingerprints as `CAR.BYD_SEAL`,
  safetyParam 2, angle control). Confirmed working, confirmed engages and
  drives normally (full real drives analyzed 2026-07-08/09).
- The `cancel_pressed`/`acc_pressed` collision bug is present and unpatched in
  this branch's `byd.h` — but there's a real, verified workaround: **engage via
  the ICC steering-wheel button, not SET-/RES+.** ICC doesn't touch the buggy
  bit at all. Verified: 94.4s clean continuous engagement, zero mismatches.
- A terminal-based live monitor (`claude/byd_terminal_monitor.py`, already
  deployed to `/data/openpilot/`) shows engagement state, gear, speed, steering
  angle, and both engage paths distinctly (`ENGAGE (ACC_ON)` vs
  `ENGAGE (ICC)`) — use it during any test/drive.

---

## THE APPROACH (one line)
bukapilot runs, unmodified, on `staging_v2`; our tool substitutes the desired
angle inside the existing carcontroller — bukapilot owns the loop, encoder,
counter, checksum, Panda, and safety gating, and we inherit all of it, tested.

---

## NEXT STEPS (in order)

### 1. Wire the desired-angle substitution into staging_v2's carcontroller
This is the one real remaining build task. The hook substitutes ONE value —
the desired steering angle — BEFORE `_compute_apply_angle`'s limiter chain,
defaulting to a no-op (byte-identical to stock) when inactive.
- **READ-ONLY analysis first**: open `staging_v2`'s actual
  `opendbc_repo/opendbc/car/byd/carcontroller.py` on the device, quote the
  real current `_compute_apply_angle` signature and the `update()` call site.
  (A verified patch already exists from earlier work against the near-identical
  cam_lka file — check whether it applies to this file too; expect it to,
  given the two are functionally the same, but verify, don't assume.)
- Propose a REVIEWABLE patch adding an optional `desired_override` (default
  `None` ⇒ stock behaviour, proven byte-identical via 100k-case randomized
  test earlier this project). Do NOT apply blind.
- Apply on a working copy, import-verify, then a PARKED read-only dry-run with
  the enable file ABSENT before anything is ever armed.
- Command shape is ANGLE, not torque: `CC.enabled=True, latActive=True,
  longActive=False, accel=0.0, actuators.steeringAngleDeg=<angle>`.

### 2. Respect the 45°/120° gap when choosing test angles
The Panda's real hard ceiling is 45° (not the 120° the software believes it
has). Keep any early test angles well under that, and watch
`safetyTxBlocked` in the terminal monitor's CAN panel — if it climbs, the
commanded angle is too aggressive for what the Panda will actually pass.

### 3. Always engage via ICC during testing
Non-negotiable for now: SET-/RES+ still hits the raw collision bug. Use the
ICC steering-wheel button to engage, and confirm via the terminal monitor
(`ENGAGE (ICC)` label) that this is actually what happened before trusting
any test data that follows.

### 4. Validate read-only, then (developer-led) live
- With the car genuinely engaged (via ICC) and moving, confirm the ACTIVE
  steering frame (real angle, steer_req True) matches the live car — still
  read-only.
- Only then is a live moving-car nudge test on the table, and that is the
  DEVELOPER'S in-person call (car moving, closed/empty area, hands ready,
  both kill paths briefed: `rm /tmp/MANUAL_STEER_ENABLE` + brake pedal).
  NOT AI-authorized.

---

## HARD RULES (from CONTEXT — non-negotiable)
- The tool changes ONE number (desired angle) fed into the existing carcontroller's
  OWN encoder. It never encodes CAN itself, never opens the Panda, never
  manufactures engagement, never defeats the Panda gate or EPS standstill gate.
- Lateral-only. Do NOT enable longitudinal (no ACC_CMD/814 or pedal/834 TX).
- Never hand-roll CAN encoding/checksum — reuse the existing implementation,
  verify against the six known-good frames via opendbc's CANPacker/CANParser,
  never cantools. (Note: some OLD scripts still carry a disproven
  `sum^0xFF` checksum formula — CONTEXT.md flags exactly where; don't copy it.)
- The car MUST be moving and stock ACC MUST be engaged (via ICC) for steering
  to reach the wheel. There is no parked steering. Don't try to defeat either
  gate.
- `DisableUpdates` should currently be set to `1` on the device — confirm
  before starting work, so the auto-updater doesn't reset anything mid-session.

## HOW TO START
1. Ensure this CONTEXT.md (2026-07-09 revision) is in the working folder.
2. Have the assistant read CONTEXT.md first, then this hand-off.
3. Tell it: "Continue Kommu.AI — desired-angle command on staging_v2. Start
   with NEXT STEP 1: read-only analysis of staging_v2's real carcontroller.py,
   then verify/adapt the existing desired_override patch for it."
