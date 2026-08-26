# Panda FW Baseline — bumpbump 70° (2026-08-11) — NEW KNOWN-GOOD

Supersedes the 45° baseline `b5893e59d1d576f7` (PandaFW_b5893e_20260804_bumpbump).
Deliberate firmware ceiling increase 45° → 70°. Device (kommu@172.20.10.3) left on this firmware.

## Documented baseline
- branch          : bumpbump
- commit          : d2876136ca5460b88e7eb14beeb58f4542a5c638
- openpilot ver   : 10.1.0
- carFingerprint  : BYD_SEAL (CarName "BYD Dolphin")
- Panda MCU       : H7 (Kedua / internal), USB 3801:ddcc
- Panda signature : 8c847fa64fc75e40   (first 8 bytes of last 128 bytes of panda_h7.bin.signed)
- steer ceiling   : 70°  (byd.h max_angle=700, angle_deg_to_can=10); rate lookups unchanged {{0,5,15},{28,26,22}}

## Files (md5)
- 228cb9aae8be4535c4d9689fd9d99ca1  panda_h7.bin.signed    (app fw — signature 8c847fa64fc75e40)
- 3d7ab0e7f7ee0a1f7c8f03facc5379fb  bootstub.panda_h7.bin  (bootstub, unchanged from 45° baseline)
- 6068d29b7f70587968d6962898e9e867  byd.h                  (safety, 70°)

## Locations (all md5-verified identical, 2026-08-11)
- device : /data/byd_fw_backup_8c847fa6_20260811_70deg/
- repo   : Modified_Kommu/PandaFW_8c847fa6_20260811_70deg_bumpbump/
- laptop : ~/Downloads/PandaFW_8c847fa6_20260811_70deg_bumpbump/

## Build (on device)
cd /data/openpilot/panda && PATH=/usr/local/venv/bin:$PATH VIRTUAL_ENV=/usr/local/venv \
  PYTHONPATH=/data/openpilot /usr/local/venv/bin/scons -j4
(venv MUST be on PATH — sign.py shebang uses env python3, and pycryptodome is only in the venv.)

## Prior baseline (rollback target if ever needed)
- 45° : b5893e59d1d576f7, PandaFW_b5893e_20260804_bumpbump/, panda_h7.bin.signed md5 127e5cd54369e7f125fd9b1ccf3d7405
- on-device 45° byd.h preserved at opendbc_repo/opendbc/safety/modes/byd.h.orig45_20260811

## NOTE — ceiling only
This raises the Panda HARD LIMIT to 70°. It does NOT change what the software commands.
The manual-steer tool's own commanded-angle cap is a separate layer (its tier values live in
sender/writer/hook) and was untouched by this change.
