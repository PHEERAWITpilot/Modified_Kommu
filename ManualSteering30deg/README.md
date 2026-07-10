# ManualSteering30deg — two-tier angle cap milestone (2026-07-10)

Snapshot of the manual-steer desired-angle substitution system after adding a
second, wider angle tier on top of the working 10deg/20deg baseline.

## What's new since ManualSteering10deg
- Two-tier angle cap across all three layers (sender, writer, hook):
  - NORMAL tier: ±20° (the prior working default)
  - HIGH tier: ±30° (new — 15° margin to the Panda's confirmed 45° hard ceiling)
- New `H` key on the sender: toggles HIGH range, but ONLY has effect when
  `M` (override) is already ON. Pressing H while M is OFF does nothing.
- `M` OFF auto-clears `H` — HIGH state never survives an override-off.
- Reconnecting the sender always resets HIGH to OFF — never inherited across
  sessions.
- `active`/`high_range` are independently re-verified as a matched pair at
  every layer (sender's send_angle, writer's atomic_write_target, hook's
  _read_target) — high_range can never be True when active is False, checked
  redundantly rather than trusted from a single source.
- Writer now performs an explicit belt-and-suspenders write of
  `{active:false, high_range:false, angle_deg:0.0}` on disconnect, on top of
  the existing 200ms staleness guard.

## Verified
- 8/8 local unit tests passed (boundary conditions: missing field, false-but-
  present, active/high_range mismatch both directions, symmetric negative
  clamp, in-range passthrough).
- Live-tested at NORMAL (20°) and HIGH (30°) tiers via ICC engagement,
  low-speed closed-area test, 2026-07-10.

## Layout
- `device-side/` — files deployed on the Kommu device at their listed paths.
- `laptop-side/` — files run on the operator's laptop (manual_steer_sender.py).

## Not yet attempted
- Any tier beyond ±30°. The Panda's real hard ceiling is 45° (byd.h
  max_angle=450 raw) — 30° was chosen as a deliberate intermediate step, not
  a final target. Do not add a third tier without the same incremental,
  single-step validation process used to get from 20° to 30°.
