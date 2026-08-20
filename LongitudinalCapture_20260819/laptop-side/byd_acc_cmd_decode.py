#!/usr/bin/env python3
"""
byd_acc_cmd_decode.py

Turns raw ACC_CMD (0x32E / 814) payload bytes into the signal table — the
actual deliverable of the lift capture harness. Runs on the LAPTOP; needs only
the DBC, no cereal, no device, no car.

Signal layout is read from byd_general_pt.dbc at runtime rather than hardcoded,
so it cannot drift out of sync with the packer.

MODES
  --sweep
      Offline reference table. Runs the real create_accel_command() across a
      range of accel values in both engaged and disengaged states and decodes
      each result. Needs no capture at all — this is what OP *would* send.
      Use it to know what to expect before the lift run, and to diff against
      what the lift run actually produced.

  --shadow FILE
      Decode /tmp/longitudinal_shadow.jsonl from a Stage-2 shadow run (the
      zero-TX capture). This is the primary deliverable path.

  --capture FILE
      Decode a Stage-2/3 long_capture_*.jsonl. Separates OP's TX frames from
      the factory's bus-0 RX frames and prints both tables, then a field-by-
      field comparison — which is how you tell whether OP's frame is even
      shaped like something the ACC ECU would accept.

  --hex AABBCC...
      Decode one payload from the command line.

EXAMPLES
  python3 claude/byd_acc_cmd_decode.py --sweep
  python3 claude/byd_acc_cmd_decode.py --shadow shadow.jsonl
  python3 claude/byd_acc_cmd_decode.py --capture long_capture_20260819_1200.jsonl
  python3 claude/byd_acc_cmd_decode.py --hex 666666884b18f0f2
  python3 claude/byd_acc_cmd_decode.py --selftest
"""

import argparse
import json
import os
import re
import sys

ACC_CMD = 814

DEFAULT_DBC = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "bumpbump_clone", "opendbc_repo", "opendbc", "dbc", "byd_general_pt.dbc")

# Print order: the engage/state bits first (what the brief cares about), then
# the accel payload, then the constants, then counter/checksum.
FIELD_ORDER = [
    "ACC_ON_1", "ACC_ON_2", "CMD_REQ_ACTIVE_LOW", "ENGAGE_BIT",
    "ACC_REQ_NOT_STANDSTILL", "ACC_CONTROLLABLE_AND_ON",
    "ACC_OVERRIDE_OR_STANDSTILL", "STANDSTILL_STATE", "STANDSTILL_RESUME",
    "ACCEL_CMD", "ACCEL_FACTOR", "DECEL_FACTOR",
    "SET_ME_25_1", "SET_ME_25_2", "SET_ME_X8", "SET_ME_1", "SET_ME_XF",
    "COUNTER", "CHECKSUM",
]

SG_RE = re.compile(
    r'^\s*SG_\s+(\w+)\s*:\s*(\d+)\|(\d+)@([01])([+-])\s*\(([^,]+),([^)]+)\)')


def load_signals(dbc_path, msg_id=ACC_CMD):
    """Parse the SG_ lines of one BO_ out of a DBC. Returns list of dicts."""
    sigs = []
    in_msg = False
    with open(dbc_path) as f:
        for line in f:
            if line.startswith("BO_ "):
                in_msg = line.split()[1] == str(msg_id)
                continue
            if not in_msg:
                continue
            if line.strip() and not line.startswith((" ", "\t")):
                break
            m = SG_RE.match(line)
            if m:
                name, start, length, order, sign, factor, offset = m.groups()
                sigs.append({
                    "name": name,
                    "start": int(start),
                    "length": int(length),
                    "little": order == "1",
                    "signed": sign == "-",
                    "factor": float(factor),
                    "offset": float(offset),
                })
    if not sigs:
        sys.exit(f"no signals found for message {msg_id} in {dbc_path}")
    return sigs


def _raw_le(dat, start, length):
    val = int.from_bytes(dat, "little")
    return (val >> start) & ((1 << length) - 1)


def _raw_be(dat, start, length):
    """Motorola/big-endian: start is the MSB in LSB0 sawtooth numbering."""
    val = 0
    bit = start
    for _ in range(length):
        val = (val << 1) | ((dat[bit // 8] >> (bit % 8)) & 1)
        bit = bit + 15 if bit % 8 == 0 else bit - 1
    return val


def decode(dat, sigs):
    """dat: 8 bytes. Returns {name: physical_value} plus _raw_{name}."""
    if len(dat) != 8:
        raise ValueError(f"expected 8 bytes, got {len(dat)}")
    out = {}
    for s in sigs:
        raw = (_raw_le(dat, s["start"], s["length"]) if s["little"]
               else _raw_be(dat, s["start"], s["length"]))
        if s["signed"] and raw >= (1 << (s["length"] - 1)):
            raw -= (1 << s["length"])
        out[s["name"]] = raw * s["factor"] + s["offset"]
        out["_raw_" + s["name"]] = raw
    return out


def _fmt(v):
    if isinstance(v, float) and v == int(v):
        return str(int(v))
    return f"{v:g}" if isinstance(v, float) else str(v)


def ordered_names(sigs):
    known = [n for n in FIELD_ORDER if any(s["name"] == n for s in sigs)]
    rest = sorted(s["name"] for s in sigs if s["name"] not in FIELD_ORDER)
    return known + rest


def print_table(rows, sigs, label_hdr="payload"):
    """rows: list of (label, hexstr, decoded_dict)."""
    names = ordered_names(sigs)
    widths = {n: max(len(n), 4) for n in names}
    lab_w = max([len(label_hdr)] + [len(r[0]) for r in rows])
    hex_w = 16
    hdr = (f"{label_hdr:<{lab_w}}  {'hex':<{hex_w}}  " +
           "  ".join(f"{n:>{widths[n]}}" for n in names))
    print(hdr)
    print("-" * len(hdr))
    for label, hx, d in rows:
        print(f"{label:<{lab_w}}  {hx:<{hex_w}}  " +
              "  ".join(f"{_fmt(d[n]):>{widths[n]}}" for n in names))


def collapse(rows, sigs):
    """Report which fields are constant across rows and which vary."""
    names = ordered_names(sigs)
    const, vary = {}, {}
    for n in names:
        vals = {_fmt(d[n]) for _, _, d in rows}
        (const if len(vals) == 1 else vary)[n] = sorted(vals)
    print("\nCONSTANT across all frames:")
    for n, v in const.items():
        print(f"  {n:<28} = {v[0]}")
    print("\nVARYING:")
    for n, v in vary.items():
        shown = ", ".join(v[:8]) + (" ..." if len(v) > 8 else "")
        print(f"  {n:<28} : {len(v):>3} distinct  [{shown}]")


# ---------------------------------------------------------------------------


def mode_sweep(sigs, args):
    """Reference table straight from the real packer. No car involved."""
    sys.path.insert(0, os.path.join(os.path.dirname(DEFAULT_DBC), "..", "..", ".."))
    try:
        from opendbc.can.packer import CANPacker
        from opendbc.car.byd.cam_lka.bydcan import create_accel_command
    except ImportError as e:
        sys.exit(f"cannot import opendbc for --sweep ({e}).\n"
                 f"Run from the repo root, or use --shadow/--capture/--hex instead.")

    packer = CANPacker("byd_general_pt")
    rows = []
    # ACCEL_MULT[BYD_SEAL] == 1, so accel passes through unscaled.
    cases = [("disengaged", a, False, False) for a in (0.0,)]
    cases += [("engaged", a, True, False)
              for a in (-4.0, -2.0, -0.5, 0.0, 0.5, 1.0, 2.0, 3.0)]
    cases += [("brake_hold", 0.0, True, True)]
    for label, accel, enabled, hold in cases:
        addr, dat, bus = create_accel_command(packer, accel, enabled,
                                              args.accel_mult, hold)
        assert addr == ACC_CMD, addr
        rows.append((f"{label} a={accel:+.1f}", bytes(dat).hex(), decode(bytes(dat), sigs)))

    print("ACC_CMD reference table — generated offline from create_accel_command()")
    print(f"(accel_mult={args.accel_mult}; COUNTER advances per call, CHECKSUM follows it)\n")
    print_table(rows, sigs, "case")
    collapse(rows, sigs)
    print("\nNOTE: ENGAGE_BIT is defined in the DBC but never set by")
    print("create_accel_command() — OP's frame leaves it 0. Compare against the")
    print("factory frame (--capture) to see whether the real ACC ECU sets it.")


def _iter_jsonl(path):
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                yield json.loads(line)
            except json.JSONDecodeError:
                continue  # tolerate a torn final line from an aborted run


def mode_shadow(sigs, args):
    rows = []
    for rec in _iter_jsonl(args.shadow):
        hx = rec.get("hex")
        if not hx or rec.get("addr") not in (ACC_CMD, None):
            continue
        d = decode(bytes.fromhex(hx), sigs)
        lab = (f"t={rec.get('t', 0) % 10000:8.2f} a={rec.get('accel', 0):+.2f} "
               f"la={int(bool(rec.get('long_active')))} v={rec.get('v_ego', 0):5.1f}")
        rows.append((lab, hx, d))
    if not rows:
        sys.exit(f"no ACC_CMD payloads found in {args.shadow}")
    print(f"SHADOW capture — {len(rows)} frames from {args.shadow}")
    print("(these were computed by the car but NEVER transmitted)\n")
    print_table(rows[:args.limit], sigs, "sample")
    if len(rows) > args.limit:
        print(f"... {len(rows) - args.limit} more (raise --limit to see them)")
    collapse(rows, sigs)


def mode_capture(sigs, args):
    tx, rx = [], []
    for rec in _iter_jsonl(args.capture):
        if rec.get("addr") != ACC_CMD or "hex" not in rec:
            continue
        try:
            d = decode(bytes.fromhex(rec["hex"]), sigs)
        except ValueError:
            continue
        lab = f"t={rec.get('t', 0) % 10000:8.2f}"
        (tx if rec.get("k") == "tx" else rx).append((lab, rec["hex"], d))

    print(f"CAPTURE — {args.capture}")
    print(f"  OP TX frames (sendcan) : {len(tx)}")
    print(f"  factory RX frames (bus0): {len(rx)}\n")

    for name, rows in (("OP TX (what bukapilot sent)", tx),
                       ("FACTORY RX (the car's own ACC_CMD on bus 0)", rx)):
        if not rows:
            print(f"--- {name}: none ---\n")
            continue
        print(f"--- {name} ---")
        print_table(rows[:args.limit], sigs, "sample")
        if len(rows) > args.limit:
            print(f"... {len(rows) - args.limit} more")
        collapse(rows, sigs)
        print()

    if tx and rx:
        print("=" * 70)
        print("OP TX vs FACTORY RX — fields where the two sources disagree")
        print("=" * 70)
        names = ordered_names(sigs)
        for n in names:
            if n in ("COUNTER", "CHECKSUM"):
                continue  # expected to differ every frame
            t = {_fmt(d[n]) for _, _, d in tx}
            r = {_fmt(d[n]) for _, _, d in rx}
            if t != r:
                print(f"  {n:<28} OP={sorted(t)[:6]}  factory={sorted(r)[:6]}")
        print("\nFields not listed above matched between OP and the factory.")


def mode_hex(sigs, args):
    dat = bytes.fromhex(args.hex.replace(" ", ""))
    print_table([("given", dat.hex(), decode(dat, sigs))], sigs)


def mode_selftest(sigs, args):
    """Round-trip the decoder against the real packer: pack known values,
    decode them back, assert every signal matches what went in."""
    sys.path.insert(0, os.path.join(os.path.dirname(DEFAULT_DBC), "..", "..", ".."))
    try:
        from opendbc.can.packer import CANPacker
    except ImportError as e:
        sys.exit(f"--selftest needs opendbc importable ({e})")

    packer = CANPacker("byd_general_pt")
    cases = [
        {"ACCEL_CMD": 0, "ACC_ON_1": 1, "ACC_ON_2": 1, "ACCEL_FACTOR": 11,
         "DECEL_FACTOR": 8, "CMD_REQ_ACTIVE_LOW": 0, "SET_ME_25_1": 25,
         "SET_ME_25_2": 25, "SET_ME_X8": 8, "SET_ME_1": 1, "SET_ME_XF": 0xF,
         "ACC_REQ_NOT_STANDSTILL": 1, "ACC_CONTROLLABLE_AND_ON": 1,
         "ACC_OVERRIDE_OR_STANDSTILL": 0, "STANDSTILL_STATE": 0,
         "STANDSTILL_RESUME": 0, "ENGAGE_BIT": 0},
        {"ACCEL_CMD": -80, "ACC_ON_1": 0, "ACC_ON_2": 0, "ACCEL_FACTOR": 0,
         "DECEL_FACTOR": 0, "CMD_REQ_ACTIVE_LOW": 1, "SET_ME_25_1": 25,
         "SET_ME_25_2": 25, "SET_ME_X8": 8, "SET_ME_1": 1, "SET_ME_XF": 0xF,
         "ACC_REQ_NOT_STANDSTILL": 0, "ACC_CONTROLLABLE_AND_ON": 0,
         "ACC_OVERRIDE_OR_STANDSTILL": 1, "STANDSTILL_STATE": 1,
         "STANDSTILL_RESUME": 1, "ENGAGE_BIT": 1},
        {"ACCEL_CMD": 30, "ACC_ON_1": 1, "ACC_ON_2": 0, "ACCEL_FACTOR": 12,
         "DECEL_FACTOR": 15, "CMD_REQ_ACTIVE_LOW": 0, "SET_ME_25_1": 0,
         "SET_ME_25_2": 63, "SET_ME_X8": 15, "SET_ME_1": 0, "SET_ME_XF": 0,
         "ACC_REQ_NOT_STANDSTILL": 1, "ACC_CONTROLLABLE_AND_ON": 0,
         "ACC_OVERRIDE_OR_STANDSTILL": 1, "STANDSTILL_STATE": 0,
         "STANDSTILL_RESUME": 1, "ENGAGE_BIT": 1},
    ]
    npass = nfail = 0
    for i, vals in enumerate(cases):
        addr, dat, bus = packer.make_can_msg("ACC_CMD", 0, vals)
        got = decode(bytes(dat), sigs)
        print(f"case {i}: {bytes(dat).hex()}")
        for k, want in vals.items():
            g = got[k]
            if abs(g - want) < 1e-9:
                npass += 1
            else:
                nfail += 1
                print(f"   FAIL {k}: decoded {g}, packed {want}")
    print(f"\n  {npass} signal round-trips passed, {nfail} failed")
    return 1 if nfail else 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--sweep", action="store_true", help="offline reference table")
    g.add_argument("--shadow", metavar="FILE", help="decode a shadow jsonl")
    g.add_argument("--capture", metavar="FILE", help="decode a capture jsonl")
    g.add_argument("--hex", metavar="BYTES", help="decode one payload")
    g.add_argument("--selftest", action="store_true", help="round-trip vs the packer")
    ap.add_argument("--dbc", default=DEFAULT_DBC)
    ap.add_argument("--limit", type=int, default=12, help="rows to print")
    ap.add_argument("--accel-mult", type=float, default=1.0,
                    help="ACCEL_MULT for the platform (BYD_SEAL = 1)")
    args = ap.parse_args()

    sigs = load_signals(args.dbc)

    if args.selftest:
        return mode_selftest(sigs, args)
    if args.sweep:
        return mode_sweep(sigs, args)
    if args.shadow:
        return mode_shadow(sigs, args)
    if args.capture:
        return mode_capture(sigs, args)
    return mode_hex(sigs, args)


if __name__ == "__main__":
    raise SystemExit(main() or 0)
