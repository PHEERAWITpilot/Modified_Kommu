# Panda FW Baseline — bumpbump (2026-08-04)

Rollback artifact for the KommuAssist 2 (kommu@172.20.10.3) after moving to bumpbump.
This is the firmware currently flashed on the Panda. Re-created because the factory
reset destroyed the prior on-device backups — off-device copies (this dir + ~/Downloads)
are what actually survive a reset.

## Documented baseline
- branch          : bumpbump
- commit          : d2876136ca5460b88e7eb14beeb58f4542a5c638
- openpilot ver   : 10.1.0
- carFingerprint  : BYD_SEAL   (CarName = "BYD Dolphin"; forced via app car-selection)
- Panda MCU       : H7 (Kedua / internal), USB 3801:ddcc
- Panda signature : b5893e59d1d576f7   (first 8 bytes of last 128 bytes of panda_h7.bin.signed)
- steer limit     : 45°  (byd.h max_angle=450, angle_deg_to_can=10)

## Files (md5)
- 127e5cd54369e7f125fd9b1ccf3d7405  panda_h7.bin.signed      (app firmware — signature b5893e59d1d576f7)
- 3d7ab0e7f7ee0a1f7c8f03facc5379fb  bootstub.panda_h7.bin    (bootstub, needed for a full flash)
- eb48e326f23141cd1d3bb4c2657e8f7d  byd.h                    (safety, 45°)

## Locations (all md5-verified identical, 2026-08-04)
- device : /data/byd_fw_backup_20260804_bumpbump/
- repo   : Modified_Kommu/PandaFW_b5893e_20260804_bumpbump/
- laptop : ~/Downloads/PandaFW_b5893e_20260804_bumpbump/

## How the signature is derived
Panda.get_signature_from_firmware(fn): last 128 bytes of the .signed file; pandad logs .hex()[:16].
Verified on device: computed = b5893e59d1d576f7 == expected. MATCH.
