# ManualSteering60deg — ±45°/±60° two-tier cap on 70° firmware (2026-08-26)

Manual desired-angle substitution for the BYD Dolphin, at the widest tier
validated so far. Same architecture as ManualSteering10deg / 30deg: the tool
substitutes ONE number — the desired steering angle — into the car port's own
encoder. It never encodes CAN, never opens the Panda, never manufactures
engagement.

## What's new since ManualSteering30deg

- **Tier caps raised**: NORMAL 20° → **45°**, HIGH 30° → **60°**
  (`manual_steer_hook.py`, `MANUAL_ABS_MAX_DEG_NORMAL` / `_HIGH`).
- **Requires 70° Panda firmware.** 30deg ran under a 45° ceiling; both tiers
  here exceed it. See "Raising the steering limit" below — this is a
  prerequisite, not an optional extra.
- **Carcontroller rebased onto the `cam_lka` split.** The 30deg carcontroller
  predates it (`opendbc.car.byd.bydcan`); this one targets
  `opendbc.car.byd.cam_lka.bydcan` and the Seal 6-era `send_steer`/`steer_req`
  structure. **The 30deg patch does not apply to current bumpbump.**

## Layout

```
device-side/
  carcontroller.py      patched — deploy to opendbc_repo/opendbc/car/byd/cam_lka/
  carcontroller.patch   the reviewable diff vs stock (see below)
  manual_steer_hook.py  deploy to opendbc_repo/opendbc/car/byd/
  manual_steer_writer.py          "
laptop-side/
  manual_steer_sender.py
```

`carcontroller.patch` is a unified diff against the **stock device file**, so it
can be reviewed against upstream rather than eyeballed as a whole file:

```
stock    5f21205730283edaf4fbc05c44a67807
patched  8b109067cd27fbcd92917a28e2d1706c
```

Three edits, purely additive apart from one line becoming a two-line
`desired_in` selection: guarded hook import (falls back to a no-op stub so a
missing hook leaves stock behaviour byte-for-byte unchanged), a
`desired_override=None` 4th parameter on `_compute_apply_angle`, and the hook
call at the substitution site.

---

## Raising the steering limit (the 70° firmware prerequisite)

The Panda enforces its own angle ceiling in firmware, independently of anything
in Python. Exceed it and the frame is rejected — the tool will appear to do
nothing above the cap.

**Where it lives:** `opendbc_repo/opendbc/safety/modes/byd.h`, field
`.max_angle`, with `.angle_deg_to_can = 10`. So the value is **degrees × 10**:

| `.max_angle` | ceiling |
|---:|---:|
| 450 | 45° |
| **700** | **70°** ← required for this tier |

Leave `.angle_rate_up_lookup` / `_down_lookup` alone unless separately deciding
to change rate limits — they are a different concern.

**How the flash happens:** `byd.h` compiles into the Panda firmware. At boot,
`pandad` compares the on-Panda signature against the tree-built firmware's
signature (first 8 bytes of the last 128 bytes of
`panda/board/obj/panda_h7.bin.signed`) and **auto-flashes on mismatch**. So:

> edit `byd.h` → rebuild → **reboot** → auto-flash

The reboot is the load-bearing step. Until it happens the Panda is still running
the old ceiling no matter what the source says.

### Route A — restore a verified binary (preferred, deterministic)

If the target firmware already exists as a saved artifact, skip the toolchain:

```bash
IP=<device-ip>
ssh kommu@$IP "cp /data/byd_fw_backup_8c847fa6_20260811_70deg/panda_h7.bin.signed \
                  /data/openpilot/panda/board/obj/panda_h7.bin.signed"
# fix the source so the tree matches the binary — ONE line, anchored:
ssh kommu@$IP "sed -i 's/^      \.max_angle = 450,\$/      .max_angle = 700,/' \
                  /data/openpilot/opendbc_repo/opendbc/safety/modes/byd.h"
ssh kommu@$IP "sudo reboot"
```

Verify the signature of the staged binary **before** rebooting, and the running
signature **after**:

```bash
# staged binary:
ssh kommu@$IP "tail -c 128 /data/openpilot/panda/board/obj/panda_h7.bin.signed | head -c 8 | od -An -tx1 | tr -d ' \n'"
# running firmware, after reboot:
ssh kommu@$IP "head -c 8 /data/params/d/PandaSignatures | od -An -tx1 | tr -d ' \n'"
```

Both must read the target signature — `8c847fa64fc75e40` for this 70° build.
The 45° baseline is `b5893e59d1d576f7`.

### Route B — rebuild from source

Needed when moving to a ceiling that has no saved artifact.

```bash
ssh kommu@$IP "cd /data/openpilot/panda && PATH=/usr/local/venv/bin:\$PATH \
  VIRTUAL_ENV=/usr/local/venv PYTHONPATH=/data/openpilot \
  /usr/local/venv/bin/scons -j4"
```

> ⚠️ **The venv MUST be on `PATH`.** scons runs `crypto/sign.py` via
> `#!/usr/bin/env python3`, but `pycryptodome` lives **only** in
> `/usr/local/venv`. Without it, signing fails with
> `ModuleNotFoundError: No module named 'Crypto'` — and a failed sign step
> **deletes** `obj/panda_h7.bin.signed`. Certs were never the problem on
> bumpbump; this PATH issue was.

The build is deterministic — restoring `byd.h` to a previous value reproduces a
byte-identical signed binary. Rollback therefore works either way: rebuild from
source, or restore the saved binary.

### Discipline

- **Raise one validated tier at a time.** 10° → 20° → 20°/30° → 45°/60°. Do not
  jump toward the ceiling.
- **Back up before changing.** Save the current `panda_h7.bin.signed` and
  `byd.h` to `/data/kommu_backup/` first, and record the signature.
- **A firmware reflash does NOT wipe `/data/openpilot`** — the tool files
  survive with unchanged md5s. That is distinct from a device re-provision or
  the updater's tree-swap, both of which DO wipe deployed files.

---

## Deploying the tool

```bash
IP=<device-ip>
BYD=/data/openpilot/opendbc_repo/opendbc/car/byd
scp device-side/manual_steer_hook.py   kommu@$IP:$BYD/
scp device-side/manual_steer_writer.py kommu@$IP:$BYD/
scp device-side/carcontroller.py       kommu@$IP:$BYD/cam_lka/
ssh kommu@$IP "sudo reboot"        # required — a running bukapilot caches imports
```

Also seed masters outside the updater's blast radius, or the next update swap
takes them:

```bash
ssh kommu@$IP "mkdir -p /data/kommu_tools/manual_steer_60deg"
scp device-side/*.py kommu@$IP:/data/kommu_tools/manual_steer_60deg/
```

**Verify the hook actually imported** — the guarded fallback is silent, and a
stub is indistinguishable from the real thing at runtime:

```bash
ssh kommu@$IP "bash -lc 'cd /data/openpilot && python3 -c \
  \"import opendbc.car.byd.cam_lka.carcontroller as cc; print(cc.manual_desired_angle.__module__)\"'"
```

Must print `opendbc.car.byd.manual_steer_hook`. If it prints
`...cam_lka.carcontroller`, the stub is active and the tool will do nothing.

## Running

The writer does **not** auto-start (it isn't in `manager.py`'s process list) and
`/tmp` is wiped on every reboot, so it needs starting each session:

```bash
ssh kommu@$IP "cd /data/openpilot/opendbc_repo/opendbc/car/byd && \
  screen -dmS msteer bash -c 'python3 -u manual_steer_writer.py --verbose > /tmp/msteer_writer.log 2>&1'"
python3 laptop-side/manual_steer_sender.py --host $IP
```

If the screen session vanishes immediately, the writer file is missing — check
`/tmp/msteer_writer.log`.

## Gates and kill paths

Nothing moves until **every** gate passes, re-checked each frame: `lat_active`
(car genuinely steering) + enable file + speed ≥ 0.30 m/s + fresh target
(<200 ms) + the driver's explicit `M` toggle. `H` adds the 60° tier and only
has effect when `M` is already on; `M` off auto-clears `H`.

Kill paths, all independent — everyone in the car must know at least the brake:

- **brake pedal** — kills `lat_active`, fastest and most reliable
- press `M` again on the sender
- quit the sender (sends a final inactive packet; the writer's disconnect
  handler also removes the enable file)
- network drop — the target goes stale within 200 ms regardless

## Verified

- 70° firmware restored and flashed 2026-08-26, running signature
  `8c847fa64fc75e40`; `byd.h` source matches at `.max_angle = 700`.
- All three device files deployed and md5-verified; hook confirmed importing
  the real module, not the fallback stub; bukapilot healthy, no crash loop.

## Not yet done

- **Live validation at these caps.** The tier caps and firmware are in place and
  the plumbing is verified, but ±45°/±60° has not been driven. NORMAL here is
  more than double 30deg's 20° — treat the first `M` press accordingly, before
  `H` is anywhere in play. Live moving-car testing is the developer's in-person
  call, per CONTEXT.md's SAFETY section.
- **Known doc nit:** `manual_steer_hook.py` line 153 still carries a stale
  comment reading `(NORMAL, 20 deg)`, left over from the 30deg copy. The code is
  correct at `45.0`; only the comment is wrong. Left unedited deliberately so
  this file's md5 keeps matching the deployed and validated copy.
