# PROPOSED CONTEXT.md change — NOT APPLIED

## Status of what is already in CONTEXT.md

Two edits were **already applied** during the session:

1. `## SAFETY` — the lift-only carve-out replacing the blanket
   "Longitudinal is OUT of scope" line.
2. A new standalone section at line ~847,
   `## LONGITUDINAL, ENGAGED: injection is WORSE when engaged (2026-08-19, LIFT)`,
   plus a strike-through cross-link in the 07-27 section pointing at it.

Note the section is named **"LONGITUDINAL, ENGAGED"**, not "LONGITUDINAL
RE-OPENED" — no section by the latter name exists.

So the substance is recorded. What follows is the *alternative* framing the task
asked for: folding a short summary into the existing 07-27 section as a **dated
addendum**, matching how the 07-27 and 07-29 sections are written.

## The choice to make

These two are redundant. Pick one:

- **(A) Keep as-is.** The standalone section carries the full mechanism, the
  data, and the caveats. The 07-27 section already cross-links to it. Nothing
  further needed — this is what the tree currently has.
- **(B) Apply the addendum below** and shrink the standalone section to just the
  mechanism + data tables, so the *verdict* lives with the 07-27 verdict and the
  *evidence* lives separately.
- **(C) Apply the addendum and delete the standalone section**, keeping only the
  data files in `Modified_Kommu/LongitudinalCapture_20260819/` as backing. Most
  compact; loses the ENGAGE_BIT analysis from CONTEXT.md.

I'd suggest **(A)** — the finding is substantial enough to warrant its own
section, and the existing cross-link already prevents anyone reading 07-27 in
isolation. (B) and (C) are offered because the task asked for the addendum form.

## The addendum diff, if you choose (B) or (C)

Insert immediately before the final line of the 07-27 section
(`not out-shouted. Full data: memory byd_814_busoff_experiment.`):

```diff
 injecting 0x32E is confirmed non-viable; longitudinal would need the
 factory ACC source silenced/isolated first (relay-style, like steering),
 not out-shouted. Full data: memory `byd_814_busoff_experiment`.
+
+### ADDENDUM 2026-08-19 (lift, engaged): engagement makes it WORSE
+Ran the same injection with the car on a lift and cruise ENGAGED, TX confined
+to the engaged window (`--capture --engaged-only`). 72 frames over 1.417s at
+50Hz drove bus-0 TEC 0 -> 84 (~59 TEC/s) before the gate aborted at t=3.393s.
+No bus-off: TEC decayed to 0, bus recovered. Extrapolated bus-off ~4.4s, so the
+abort cut it ~3s short.
+MECHANISM: parked, bukapilot and the factory both send ACCEL_CMD byte0=100 --
+identical -- so frames diverge only deep in the payload and collisions are rare
+(hence 07-27's tens-of-minutes). Engaged, the factory sends 123 while bukapilot
+sends 100-102, so ALL 72/72 frames diverged at BIT 3 (4th data bit on the wire)
+-> collision on nearly every overlap. Engagement removes the accidental
+byte-0 protection.
+Frames DID reach the bus (TEC cannot accumulate without driving the wire; ~19
+errors / ~72 successes). No evidence the ECU obeyed: the car tracked the
+factory's +23 command, vEgo 4.54 -> 7.09 m/s, while we commanded 0-2.
+SCOPE: this tests injection alongside a LIVE, UNTOUCHED factory ECU. The
+silenced/isolated-ECU path (see the 07-29 UDS section) remains UNTESTED, not
+disproven. Payload correctness does NOT fix this -- the collision is
+arbitration-level and payload-independent in kind.
+Full data + report: `Modified_Kommu/LongitudinalCapture_20260819/`.
```

If applying (B) or (C), also revert the strike-through cross-link added earlier
in that section, since the addendum then answers the question in place:

```diff
-`immediateDisable | PERMANENT`) until reboot. ~~**Not tested: whether
-engaging cruise changes this**~~ — all runs here were parked/idle. **ANSWERED
-2026-08-19 (lift): engaging makes it WORSE, ~59 TEC/s vs tens-of-minutes
-parked, because parked the two frames share an identical byte 0 and engaged
-they diverge at bit 3. See the next section.** Simply
+`immediateDisable | PERMANENT`) until reboot. **Not tested here: whether
+engaging cruise changes this** — all runs in this section were parked/idle;
+see the 2026-08-19 addendum below. Simply
```
