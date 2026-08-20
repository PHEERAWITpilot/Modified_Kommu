# LongitudinalCapture_20260819

Instrumentation harness and captured evidence for the BYD Dolphin longitudinal
`ACC_CMD` (0x32E / 814) investigation, 2026-08-19.

**Outcome: 0x32E injection alongside the live factory ACC ECU is confirmed
non-viable. Engagement makes the collision faster, not milder.**
Full analysis in [FINDINGS.md](FINDINGS.md).

> **The device is fully reverted.** Nothing from this folder is deployed. The
> car is back to stock: `openpilotLongitudinalControl = False`
> (`interface.py` md5 `a9504d5f27941b1014679d7a2a989392`), `values.py` stock,
> `cam_lka/carcontroller.py` restored to `5f21205730283edaf4fbc05c44a67807`,
> and the gate + capture scripts removed from `/data/openpilot`.

---

## Contents

```
device-side/
  byd_longitudinal_kill_gate.py    NEW. Per-frame TX gate. Fail-closed.
  byd_long_conflict_capture.py     EXTENDED from the 07-27 read-only script.
  carcontroller.patch              THE REVIEWABLE ARTEFACT — diff vs stock.
  carcontroller.py                 Result of applying that patch (for reference).
laptop-side/
  byd_acc_cmd_decode.py            NEW. DBC-driven decode / sweep / compare.
  tests/run_all.sh                 Pre-deploy gate: 4 suites, all must pass.
  tests/test_longitudinal_gate.py  Drives the REAL patched CarController.update().
  tests/test_capture_session.py    Capture script against a stubbed cereal.
data/
  phase1_factory_rx_engage_bit.jsonl   5999 factory RX frames + 2156 carState
  phase2_shadow_parked.jsonl           72 shadow frames (parked, zero TX)
  phase3_engaged_injection.jsonl       72 TX + 670 RX + 239 carState + abort
FINDINGS.md                        The report.
```

## The carcontroller patch

`carcontroller.patch` is a unified diff against the **stock device file**, not a
copy of the modified file, so it can be reviewed against upstream. Verified:
applying it to stock reproduces the deployed file byte-for-byte.

```
stock    5f21205730283edaf4fbc05c44a67807
patched  13a240fd35a549f3250573ff60d47ba2
```

Two changes, purely additive apart from one line becoming a gated append:

1. Guarded import of the kill gate, **falling back to closed** (not to stock) on
   any import failure.
2. Gates only the `can_sends.append`, leaving branch selection on config alone.

### Two traps this patch encodes — read before modifying

**Do not fold the gate into the config `if`.** Writing
`if <config> and longitudinal_tx_allowed():` falls through to the `else` branch
and starts emitting resume-`send_buttons` every time the gate closes — the
safety mechanism inventing a new TX path. Branch selection must stay on config;
only the append is gated. Regression-covered.

**Do not add `CAR.BYD_SEAL` to `BYD_OP_LONG_PLATFORMS`.** That tuple is shared
with `cam_lka/carstate.py:213-215`, where it makes the **camera-bus** CANParser
require `ACC_CMD`(814) + `ACC_HUD_ADAS`(813) at 50 Hz. Behind the KommuRelay
they do not arrive at that rate. Doing this on 2026-08-19 produced `canError` —
displayed as the misleading alert **"Unknown Vehicle Variant"** — with
`IMMEDIATE_DISABLE` and a red LED. The patch recognises `BYD_SEAL` in the
carcontroller only, leaving `values.py` stock.

## What was validated

Offline, no car — 4 suites. **These are archived copies.** They import
`opendbc`/`cereal` from the `bumpbump_clone` tree, which lives in the Kommu.AI
working tree, not in this repo — so run them from there, not from this folder:

```bash
cd ~/Desktop/Kommu.AI && bash claude/tests/run_all.sh
```


| Suite | Cases | Covers |
|---|---|---|
| gate self-test | 34 | every gate branch, staleness, engaged-only, fail-closed |
| decoder round-trip | 51 | decode vs the real packer, all signals |
| carcontroller gating | 29 | real `update()`: closed ⇒ no ACC_CMD **and** no stray buttons |
| capture session | 27 | enable/health lifecycle, abort, teardown on exception |

On the car: Stage 1 (config live, gate shut ⇒ zero TX, alerts clean), Stage 2
(shadow ⇒ payloads logged, still zero TX — proving the branch executes and the
gate is what stops it), Stage 3 (lift, live injection, aborted on TEC=84 at
t=3.393 s, no bus-off, bus recovered).

## Data files

~1.0 MB total, kept in-repo because every quantitative claim in FINDINGS.md is
derived from them and the device copies have since been deleted. They are the
only surviving record of these runs.

**The `.log` files were not preserved** — they were removed from the device
during the full revert before being copied off. No loss of information: the
`.log` was a 1 Hz text summary recording only `data[0]`, whereas the `.jsonl`
holds every frame with the complete 8-byte payload. The 1 Hz summary of the
Stage 3 run was also affected by the reporting bug described in FINDINGS.md
§2, so it would have been actively misleading to preserve as reference.

## If this is ever re-run

1. `cd ~/Desktop/Kommu.AI && bash claude/tests/run_all.sh` — must print ALL PASS.
2. Deploy gate + capture script + patched carcontroller; **reboot** (a running
   bukapilot caches Python modules at import).
3. Flip `interface.py` only. Back it up first; `values.py` stays stock.
4. Stage 1 read-only check: alerts clean, `opTX = 0/s`, bus-0 `TEC = 0`.
5. Stage 2 `--shadow`: confirms the branch executes at zero TX.
6. Stage 3 `--capture --engaged-only`: **lift only**, brake covered.
7. Revert `interface.py`, reboot, verify md5.

Deploy the **fixed** `byd_long_conflict_capture.py` from this folder — the
version that ran on 2026-08-19 had the 1 Hz reporting bug.
