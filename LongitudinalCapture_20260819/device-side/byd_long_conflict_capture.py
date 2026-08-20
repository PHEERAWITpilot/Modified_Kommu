#!/usr/bin/env python3
"""
BYD Dolphin — Longitudinal 0x32E Conflict / Bus-Off Capture  (READ-ONLY)
========================================================================
PARKED instrumented test. Cereal SubMaster + sub_sock only. NO Panda() usage,
NO CAN TX, no writes of any kind. Logs at 1 Hz to stdout AND a per-run
timestamped file in /data/openpilot (persists across reboots).

PURPOSE
  Demonstrate (or refute) that openpilotLongitudinalControl=True makes the
  panda's own 50 Hz TX of ACC_CMD (0x32E) collide with the car's native bus-0
  ACC source (duplicate-ID), driving bus 0's transmit-error counter to bus-off
  and eventually canBusMissing / "CAN Bus Disconnected".

SPEC DEVIATION (important — read this)
  The requested pandaStates fields canSendErrs / canRxErrs / canFwdErrs DO NOT
  EXIST in this fork's PandaState schema. This fork instead exposes the full
  per-bus CAN controller state canState0/1/2, which is strictly more informative
  (it carries the actual TEC/REC and a hard busOff flag). We use those:
     canSendErrs  ->  canState0.transmitErrorCnt (TEC gauge) + totalTxLostCnt/s
     canRxErrs    ->  canState0.receiveErrorCnt  (REC gauge) + totalRxLostCnt/s
     canFwdErrs   ->  canState0.totalFwdCnt/s (throughput) + totalErrorCnt/s
     + the decisive canState0.busOff flag and busOffCnt/s.
  DELTAS/sec are printed for cumulative counters; TEC/REC are gauges (shown raw).

  canState index == physical CAN bus number (0 = powertrain/PT-CAN where the
  0x32E conflict lives; 2 = camera bus). Verified live: bus 0 was the one that
  went busOff under the conflict while 1 and 2 stayed clean.

WHAT'S LOGGED EACH SECOND
  t          seconds since script start
  BUS0/BUS2  busOff, errorWarning/Passive, TEC, REC, and /s deltas of
             busOffCnt, totalErrorCnt, totalTxLostCnt, canCoreResetCnt
  PANDA      faultStatus, faults[], safetyModel/Param, safetyRxChecksInvalid,
             heartbeatLost, and /s deltas of safetyTxBlocked, txBufferOverflow,
             spiErrorCount   (heartbeatLost/spiErr distinguish comms-drop from bus-off)
  0x32E      stock RX rate on 'can' (src=0) + data[0];  OP TX rate on 'sendcan' + data[0]
  0x342      rate on 'can' (bus-0 canary)
  STATE      selfdriveState.alertText1 ; onroadEvents names
  MARKERS    first second any CAN error counter goes non-zero / bus goes busOff,
             and the first second canBusMissing appears.

RUN ON DEVICE:
  python3 /data/openpilot/byd_long_conflict_capture.py
  (Ctrl+C to stop, or it self-stops at --minutes, default 5.)

--------------------------------------------------------------------------
CAPTURE MODE (2026-08-18) — lift-only ACC_CMD payload harness
--------------------------------------------------------------------------
Default behaviour above is UNCHANGED and still strictly read-only. Two opt-in
flags add supervision for the lift capture (see CONTEXT.md SAFETY carve-out
and byd_longitudinal_kill_gate.py):

  --shadow    Create /tmp/LONGITUDINAL_SHADOW so carcontroller LOGS the
              engaged ACC_CMD payload to /tmp/longitudinal_shadow.jsonl
              WITHOUT transmitting it. Still zero TX. This is Stage 2, the
              primary deliverable, and needs no confirmation phrase.

  --capture   Create /tmp/LONGITUDINAL_CAPTURE_ENABLE so the kill gate may
              OPEN and real 0x32E goes on bus 0. Stage 3 only. Requires the
              typed confirmation phrase. While running, this process:
                * writes /tmp/longitudinal_health.json at ~20Hz — the gate
                  refuses TX if that file is >400ms stale, so if this process
                  dies, hangs, or is killed, TX stops on its own within 400ms;
                * independently deletes the enable file the instant bus-0 TEC
                  leaves 0 or any error flag sets (redundant with the gate's
                  own per-frame check — two paths to the same shutoff);
                * removes the enable file in a finally covering clean exit,
                  Ctrl-C, and uncaught exception.

  Both modes also write a full-rate .jsonl beside the .log: every 0x32E frame
  on BOTH topics with the complete 8-byte payload (the 1Hz text log only ever
  recorded data[0]), plus carState. That closes the "only pandaStates at 1Hz"
  resolution gap that made the 07-27 logs unable to show which frame landed.

  Manual kill from a second SSH session, works even if this process is wedged:
      touch /tmp/LONGITUDINAL_KILL
"""

import argparse
import json
import os
import sys
import time

try:
    import cereal.messaging as messaging
except ImportError:
    sys.exit("ERROR: cereal not found. Run inside device venv: "
             "cd /data/openpilot && python3 /data/openpilot/byd_long_conflict_capture.py")

ACC_CMD = 0x32E   # 814 — the contested longitudinal command
PEDAL   = 0x342   # 834 — bus-0 canary

# --- capture-mode gate files (must match byd_longitudinal_kill_gate.py) ---
ENABLE_FILE = "/tmp/LONGITUDINAL_CAPTURE_ENABLE"
KILL_FILE   = "/tmp/LONGITUDINAL_KILL"
HEALTH_FILE = "/tmp/longitudinal_health.json"
SHADOW_FILE = "/tmp/LONGITUDINAL_SHADOW"
ENGAGED_ONLY_FILE = "/tmp/LONGITUDINAL_ENGAGED_ONLY"
CONFIRM_PHRASE = "car is on the lift"
HEALTH_PERIOD_S = 0.05   # ~20Hz; gate's staleness limit is 400ms

# Abort the instant ANY of these is true on bus 0. The 128 error-passive
# threshold was never measured on this platform — 07-27 recorded onset as
# "first time TEC leaves 0", so that is the trigger.
ABORT_FLAGS = ('busOff', 'errorPassive', 'errorWarning')


def _get(obj, name, default=0):
    try:
        return getattr(obj, name)
    except Exception:
        return default


def _busrow(cs, prev, dt):
    """Format a per-bus CAN-state row with /s deltas for cumulative counters."""
    def d(field):
        cur = _get(cs, field)
        p = prev.get(field, cur)
        return (cur - p) / dt if dt > 0 else 0.0
    return (
        f"busOff={int(_get(cs,'busOff'))} "
        f"errW={int(_get(cs,'errorWarning'))} errP={int(_get(cs,'errorPassive'))} "
        f"TEC={_get(cs,'transmitErrorCnt')} REC={_get(cs,'receiveErrorCnt')} "
        f"busOffCnt+={d('busOffCnt'):.0f}/s totErr+={d('totalErrorCnt'):.0f}/s "
        f"txLost+={d('totalTxLostCnt'):.0f}/s coreReset+={d('canCoreResetCnt'):.1f}/s"
    )


def _bus_snapshot(cs):
    return {k: _get(cs, k) for k in
            ('busOffCnt', 'totalErrorCnt', 'totalTxLostCnt', 'totalRxLostCnt',
             'totalFwdCnt', 'canCoreResetCnt')}


def _write_health(cs0, ps):
    """Publish bus-0 CAN health for the kill gate. Atomic: the gate must never
    read a half-written file. Returns (tec, tripped_reason_or_None)."""
    tec = _get(cs0, 'transmitErrorCnt')
    tripped = None
    if tec:
        tripped = f"TEC={tec}"
    else:
        for f in ABORT_FLAGS:
            if _get(cs0, f):
                tripped = f
                break
    data = {
        "transmit_error_cnt": int(tec),
        "receive_error_cnt": int(_get(cs0, 'receiveErrorCnt')),
        "bus_off": bool(_get(cs0, 'busOff')),
        "error_passive": bool(_get(cs0, 'errorPassive')),
        "error_warning": bool(_get(cs0, 'errorWarning')),
        "safety_tx_blocked": int(_get(ps, 'safetyTxBlocked')),
        "ts": time.time(),
    }
    try:
        tmp = HEALTH_FILE + ".tmp"
        with open(tmp, "w") as f:
            f.write(json.dumps(data))
        os.replace(tmp, HEALTH_FILE)
    except OSError:
        pass  # gate sees a stale file and closes on its own — correct outcome
    return tec, tripped


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--minutes', type=float, default=5.0, help='auto-stop after N minutes')
    ap.add_argument('--tag', default='', help='label for the run (e.g. RUN_A_flagOFF)')
    ap.add_argument('--shadow', action='store_true',
                    help='Stage 2: log the engaged ACC_CMD payload without transmitting (zero TX)')
    ap.add_argument('--capture', action='store_true',
                    help='Stage 3: LIFT ONLY. Open the TX gate so real 0x32E goes on bus 0')
    ap.add_argument('--abort-tail', type=float, default=10.0,
                    help='keep logging N seconds after an abort, to capture the aftermath')
    ap.add_argument('--engaged-only', action='store_true',
                    help='inject ONLY while engaged, instead of continuously from process '
                         'start. Narrows exposure to the engaged window; the brake pedal '
                         'then becomes a direct TX kill. Use with --capture.')
    ap.add_argument('--out-dir', default='/data/openpilot',
                    help='where to write the .log/.jsonl (default /data/openpilot, survives reboot)')
    args = ap.parse_args()

    # ---- capture-mode preconditions, BEFORE anything is opened or created ----
    if args.capture:
        print("=" * 70)
        print("STAGE 3 — LIVE 0x32E TRANSMISSION ONTO BUS 0.")
        print("This injects ACC_CMD alongside the car's own ACC transmitter.")
        print("The 07-27 experiment showed this drives bus 0 toward bus-off")
        print("(canBusMissing / immediateDisable / PERMANENT until reboot).")
        print("")
        print("Confirm ALL of the following are physically true RIGHT NOW:")
        print("  - car is on a lift, all four wheels off the ground")
        print("  - someone is at the driver's seat / able to reach the brake pedal")
        print("  - nothing is connected to the drivetrain that the wheels could load")
        print("=" * 70)
        try:
            typed = input(f'Type exactly "{CONFIRM_PHRASE}" to proceed: ')
        except (EOFError, KeyboardInterrupt):
            sys.exit("\nno confirmation — nothing was started")
        if typed.strip() != CONFIRM_PHRASE:
            sys.exit("confirmation phrase did not match — nothing was started")

    stamp = time.strftime('%Y%m%d_%H%M%S')
    tag = ('_' + args.tag) if args.tag else ''
    log_path = os.path.join(args.out_dir, f"long_capture_{stamp}{tag}.log")
    jsonl_path = os.path.join(args.out_dir, f"long_capture_{stamp}{tag}.jsonl")

    sm = messaging.SubMaster(['pandaStates', 'selfdriveState', 'onroadEvents', 'carState'])
    sub_can  = messaging.sub_sock('can',     timeout=20)
    sub_send = messaging.sub_sock('sendcan', timeout=0)

    # per-second frame counters
    n_stock = n_optx = n_pedal = 0
    last_stock_d0 = last_optx_d0 = None

    prev_bus = {0: {}, 2: {}}
    prev_panda = {}
    first_busmissing_t = None

    # auto-stop / transition tracking
    FULL_FIELDS = ('busOff', 'busOffCnt', 'transmitErrorCnt', 'receiveErrorCnt',
                   'totalErrorCnt', 'totalTxLostCnt', 'totalRxLostCnt',
                   'canCoreResetCnt', 'totalTxCnt', 'totalRxCnt')
    def full_snap(ps):
        cs = ps.canState0
        d = {k: _get(cs, k) for k in FULL_FIELDS}
        d['txBufOvf'] = _get(ps, 'txBufferOverflow')
        return d
    tec_left_zero_t = None; tec_left_zero_val = None
    busoff_first_t = None
    fault_t = None; fault_snap = None; fault_snap_ts = None
    last_snap = None; last_snap_t = None; last_snap_ts = None
    stop_deadline = None            # set to now+10s ONLY when busOff latches
    max_optx = 0                    # runtime flag proof (0x32E on sendcan)
    first_optx_t = None             # t of the FIRST 0x32E we transmit
    total_optx = 0                  # cumulative TX frames this run
    engaged_frames = 0              # carState samples with cruise engaged
    # checkpoint mode (post-onset): snapshot every 15s until busOff or cap
    CHECKPOINT_S = 15.0
    checkpoints = []                # list of (t, ts, snap_dict)
    next_checkpoint = None          # set to onset_t + 15 at onset
    onset_totalErr = None           # totalErrorCnt at onset (for accumulated-errors report)

    # disk flag line (the SEAL-branch effective value)
    disk_flag = '(unavailable)'
    try:
        with open('/data/openpilot/opendbc_repo/opendbc/car/byd/interface.py') as _f:
            for _ln in _f:
                if 'openpilotLongitudinalControl' in _ln and '=' in _ln:
                    disk_flag = _ln.strip()   # last match = SEAL-branch line
    except OSError:
        pass

    # ---- session state for capture/shadow modes ----
    session_mode = args.capture or args.shadow or args.engaged_only
    jf = None                    # full-rate jsonl handle
    last_health = 0.0
    abort_reason = None
    # ---- FAST-PATH fault tracking (20Hz) ----------------------------------
    # The 1Hz tick below ALSO tracks onset, but it samples pandaStates once a
    # second and TEC is a gauge that DECAYS (-1 per successful TX). On
    # 2026-08-19 TEC spiked 0->84 and decayed back to 0 entirely between two
    # 1Hz samples, so the 1Hz fields reported "TEC stayed 0 / NO FAULT" for a
    # run that had actually aborted on TEC=84. These fast-path values are the
    # authoritative ones for the summary.
    fast_tec_left_zero_t = None
    fast_peak_tec = 0
    fast_first_flag = None
    fast_busoff_t = None
    abort_t = None
    enable_created = False

    if session_mode:
        # A stale KILL file from a previous run would silently block everything.
        try:
            os.remove(KILL_FILE)
        except OSError:
            pass
    if args.shadow:
        open(SHADOW_FILE, "w").close()
    if args.engaged_only:
        open(ENGAGED_ONLY_FILE, "w").close()
    if args.capture:
        # The gate ALSO requires a fresh, clean health file, so creating this
        # now does not open TX yet — it opens once the first health write lands.
        open(ENABLE_FILE, "w").close()
        enable_created = True

    start = time.monotonic()
    next_tick = start + 1.0
    end = start + args.minutes * 60.0

    def emit(f, line):
        f.write(line + "\n"); f.flush()
        sys.stdout.write(line + "\n"); sys.stdout.flush()

    def jrec(obj):
        if jf is not None:
            jf.write(json.dumps(obj) + "\n")

    with open(log_path, "a") as lf:
        if session_mode:
            jf = open(jsonl_path, "a")
        hdr = (f"# BYD long conflict capture  tag={args.tag or '(none)'}  "
               f"start={time.strftime('%H:%M:%S')}  file={log_path}")
        emit(lf, hdr)
        emit(lf, "# canSendErrs/RxErrs/FwdErrs absent in this fork -> using canState0 "
                 "TEC/REC/busOff (see header).")
        if args.shadow:
            emit(lf, f"# SHADOW mode: {SHADOW_FILE} created — carcontroller logs the ACC_CMD")
            emit(lf, "# payload to /tmp/longitudinal_shadow.jsonl but transmits NOTHING.")
        if args.capture:
            emit(lf, f"# CAPTURE mode: {ENABLE_FILE} created — TX gate MAY OPEN. LIFT ONLY.")
            emit(lf, f"# abort on first nonzero bus-0 TEC or any of {ABORT_FLAGS}")
            emit(lf, f"# manual kill from another shell:  touch {KILL_FILE}")
        if session_mode:
            emit(lf, f"# full-rate jsonl: {jsonl_path}")

        try:
            while time.monotonic() < end and (stop_deadline is None or time.monotonic() < stop_deadline):
                # drain high-rate CAN topics and tally
                for pkt in messaging.drain_sock(sub_can, wait_for_one=True):
                    for fr in pkt.can:
                        if fr.address == ACC_CMD and fr.src == 0:
                            n_stock += 1
                            if len(fr.dat): last_stock_d0 = fr.dat[0]
                            # FULL payload of the factory frame — this is the RX
                            # side of the correlation (did our frame ever land?).
                            jrec({"t": time.time(), "k": "rx", "addr": fr.address,
                                  "hex": bytes(fr.dat).hex(), "src": fr.src})
                        elif fr.address == PEDAL and fr.src == 0:
                            n_pedal += 1
                for pkt in messaging.drain_sock(sub_send, wait_for_one=False):
                    for fr in pkt.sendcan:
                        if fr.address == ACC_CMD:
                            n_optx += 1
                            total_optx += 1
                            if first_optx_t is None:
                                first_optx_t = time.monotonic() - start
                                emit(lf, f"        *** FIRST OP TX of 0x32E at "
                                         f"t={first_optx_t:.3f}s — collision exposure "
                                         f"starts HERE, not at t=0 ***")
                            if len(fr.dat): last_optx_d0 = fr.dat[0]
                            jrec({"t": time.time(), "k": "tx", "addr": fr.address,
                                  "hex": bytes(fr.dat).hex(), "src": fr.src})

                sm.update(0)

                now = time.monotonic()

                # ---- FAST PATH: health publish + redundant abort (~20Hz) ----
                # This must NOT live under the 1Hz tick below: the gate needs a
                # health file fresher than 400ms, and an abort that waited for
                # the next 1Hz tick would keep transmitting for up to a second
                # after the bus started erroring.
                if session_mode and (now - last_health) >= HEALTH_PERIOD_S:
                    last_health = now
                    if sm.recv_frame['carState']:
                        _cs = sm['carState']
                        if _cs.cruiseState.enabled:
                            engaged_frames += 1
                        jrec({"t": time.time(), "k": "carState",
                              "vEgo": round(_cs.vEgo, 3), "aEgo": round(_cs.aEgo, 3),
                              "cruiseEnabled": bool(_cs.cruiseState.enabled),
                              "cruiseSpeed": round(_cs.cruiseState.speed, 2),
                              "gasPressed": bool(_cs.gasPressed),
                              "brakePressed": bool(_cs.brakePressed),
                              "standstill": bool(_cs.standstill)})
                    if sm.recv_frame['pandaStates'] and len(sm['pandaStates']):
                        _ps = sm['pandaStates'][0]
                        _tec, _tripped = _write_health(_ps.canState0, _ps)
                        # authoritative fault record — 20Hz, catches spike-and-decay
                        if _tec > fast_peak_tec:
                            fast_peak_tec = _tec
                        if _tec and fast_tec_left_zero_t is None:
                            fast_tec_left_zero_t = now - start
                        for _f in ABORT_FLAGS:
                            if _get(_ps.canState0, _f):
                                if fast_first_flag is None:
                                    fast_first_flag = (_f, now - start)
                                if _f == 'busOff' and fast_busoff_t is None:
                                    fast_busoff_t = now - start
                        if _tripped and abort_reason is None:
                            abort_reason = _tripped
                            abort_t = now - start
                            # Independent of the gate's own per-frame check.
                            # Two paths to the same shutoff, by design.
                            try:
                                os.remove(ENABLE_FILE)
                            except OSError:
                                pass
                            enable_created = False
                            banner = ("*** ABORT: bus-0 " + _tripped +
                                      f" at t={abort_t:.2f}s -> ENABLE removed, TX gate closed ***")
                            emit(lf, "!" * 70)
                            emit(lf, banner)
                            emit(lf, "!" * 70)
                            jrec({"t": time.time(), "k": "abort", "reason": _tripped,
                                  "t_rel": abort_t})
                            # keep logging the aftermath, then stop
                            if stop_deadline is None:
                                stop_deadline = now + args.abort_tail

                if now < next_tick:
                    continue
                dt = now - (next_tick - 1.0)
                t = now - start
                next_tick += 1.0

                # ---- panda / bus state ----
                bus_txt = "pandaStates: (none yet)"
                panda_txt = ""
                events = []
                alert1 = ""
                new_error_this_tick = False

                if sm.recv_frame['pandaStates'] and len(sm['pandaStates']):
                    ps = sm['pandaStates'][0]
                    cs0 = ps.canState0
                    cs2 = ps.canState2
                    bus_txt = ("BUS0[" + _busrow(cs0, prev_bus[0], dt) + "]\n"
                               "        BUS2[" + _busrow(cs2, prev_bus[2], dt) + "]")

                    # ---- transition detection + auto-stop ----
                    tec = _get(cs0, 'transmitErrorCnt')
                    boff = bool(_get(cs0, 'busOff'))
                    last_snap = full_snap(ps); last_snap_t = t
                    last_snap_ts = time.strftime('%H:%M:%S')
                    if tec_left_zero_t is None and tec > 0:
                        tec_left_zero_t = t; tec_left_zero_val = tec
                    # ONSET = first time TEC leaves 0 (or busOff, whichever first).
                    # After onset we switch to 15s checkpoint mode (no continuous log).
                    if fault_t is None and (boff or tec > 0):
                        fault_t = t
                        fault_snap = full_snap(ps); fault_snap_ts = time.strftime('%H:%M:%S')
                        onset_totalErr = _get(cs0, 'totalErrorCnt')
                        next_checkpoint = now + CHECKPOINT_S
                        checkpoints.append((t, fault_snap_ts, fault_snap))  # onset = checkpoint 0
                        new_error_this_tick = True
                    # busOff LATCH = the real catastrophic fault: 1 final snap 10s later, stop.
                    if busoff_first_t is None and boff:
                        busoff_first_t = t
                        stop_deadline = now + 10.0

                    prev_bus[0] = _bus_snapshot(cs0)
                    prev_bus[2] = _bus_snapshot(cs2)

                    def pd(field):
                        cur = _get(ps, field)
                        p = prev_panda.get(field, cur)
                        return (cur - p) / dt if dt > 0 else 0.0
                    panda_txt = (
                        f"PANDA faultStatus={ps.faultStatus} faults={list(ps.faults)} "
                        f"model={ps.safetyModel} param={ps.safetyParam} "
                        f"rxChecksInvalid={int(_get(ps,'safetyRxChecksInvalid'))} "
                        f"heartbeatLost={int(_get(ps,'heartbeatLost'))} "
                        f"txBlocked+={pd('safetyTxBlocked'):.0f}/s "
                        f"txBufOvf+={pd('txBufferOverflow'):.0f}/s "
                        f"spiErr+={pd('spiErrorCount'):.1f}/s"
                    )
                    prev_panda = {k: _get(ps, k) for k in
                                  ('safetyTxBlocked', 'txBufferOverflow', 'spiErrorCount')}

                if sm.recv_frame['selfdriveState']:
                    alert1 = sm['selfdriveState'].alertText1
                if sm.recv_frame['onroadEvents']:
                    events = [str(e.name) for e in sm['onroadEvents']]
                    if 'canBusMissing' in events and first_busmissing_t is None:
                        first_busmissing_t = t

                # ---- 0x32E rates ----
                d0s = last_stock_d0 if last_stock_d0 is not None else '-'
                d0t = last_optx_d0 if last_optx_d0 is not None else '-'
                acc_txt = (f"0x32E stockRX={n_stock:2d}/s (d0={d0s})  "
                           f"opTX={n_optx:2d}/s (d0={d0t})   0x342={n_pedal:2d}/s")

                # ---- assemble ----
                if fault_t is None:
                    # PRE-ONSET: continuous 1 Hz logging.
                    emit(lf, f"[t={t:6.1f}s | {time.strftime('%H:%M:%S')}] {bus_txt}")
                    emit(lf, f"        {panda_txt}")
                    emit(lf, f"        {acc_txt}")
                    emit(lf, f"        alert1={alert1!r}  events={events}")
                else:
                    # POST-ONSET: quiet except 15s checkpoints + the onset/busOff markers.
                    if new_error_this_tick:
                        emit(lf, f"[t={t:6.1f}s | {fault_snap_ts}] *** ONSET: TEC left 0 "
                                 f"(TEC={tec_left_zero_val}) -> checkpoint mode every {CHECKPOINT_S:.0f}s ***")
                        emit(lf, f"        CHECKPOINT0 @ t={t:.1f}s: {fault_snap}")
                    elif next_checkpoint is not None and now >= next_checkpoint:
                        snap = full_snap(ps)
                        checkpoints.append((t, time.strftime('%H:%M:%S'), snap))
                        next_checkpoint += CHECKPOINT_S
                        emit(lf, f"[t={t:6.1f}s | {time.strftime('%H:%M:%S')}] CHECKPOINT{len(checkpoints)-1} "
                                 f"TEC={snap['transmitErrorCnt']} totErr={snap['totalErrorCnt']} "
                                 f"busOff={snap['busOff']} busOffCnt={snap['busOffCnt']}  events={events}")
                    if busoff_first_t is not None and abs(busoff_first_t - t) < 0.6:
                        emit(lf, f"        *** busOff LATCHED at t={t:.1f}s -> final snap in 10s, then stop ***")
                if first_busmissing_t is not None and abs(first_busmissing_t - t) < 0.6:
                    emit(lf, f"        *** MARKER: canBusMissing first seen at t={t:.1f}s ***")

                if n_optx > max_optx:
                    max_optx = n_optx
                n_stock = n_optx = n_pedal = 0   # reset per-second tallies

        except KeyboardInterrupt:
            pass
        finally:
            # ---- gate teardown FIRST, before any reporting can fail ----
            # Covers clean exit, Ctrl-C, and uncaught exceptions alike. Even if
            # every line below this raised, TX would still stop: the health file
            # goes stale within 400ms once this process is gone.
            for _f in (ENABLE_FILE, SHADOW_FILE, HEALTH_FILE, ENGAGED_ONLY_FILE):
                try:
                    os.remove(_f)
                except OSError:
                    pass
            enable_created = False
            if session_mode:
                emit(lf, "[capture] gate files removed — TX gate closed")
            if jf is not None:
                try:
                    jf.close()
                except OSError:
                    pass

            # post-fault snapshot = the last tick captured (~10s after fault, since
            # we stop 10s after the trigger). For a no-fault run it's just the final tick.
            post_snap = last_snap; post_snap_t = last_snap_t; post_snap_ts = last_snap_ts
            cap_min = args.minutes
            faulted = fault_t is not None
            emit(lf, "")
            emit(lf, "======================= SUMMARY =======================")
            emit(lf, f"tag              : {args.tag or '(none)'}")
            emit(lf, f"start            : {time.strftime('%H:%M:%S')} (end)   file={log_path}")
            emit(lf, f"disk flag line   : {disk_flag}")
            emit(lf, f"runtime flag     : opTX 0x32E max={max_optx}/s  "
                     f"=> openpilotLongitudinalControl {'TRUE (transmitting)' if max_optx > 0 else 'FALSE (silent)'} at runtime")
            # Prefer the 20Hz fast-path record: TEC decays, and a spike can
            # begin AND end between two 1Hz samples (seen 2026-08-19: 0->84->0).
            faulted_fast = (fast_tec_left_zero_t is not None or fast_first_flag is not None
                            or abort_reason is not None)
            emit(lf, f"TEC first left 0 : "
                     f"{'t=%.3fs (20Hz sampler)' % fast_tec_left_zero_t if fast_tec_left_zero_t is not None else ('t=%.1fs (1Hz sampler)' % tec_left_zero_t if tec_left_zero_t is not None else 'never observed non-zero')}")
            emit(lf, f"PEAK TEC (20Hz)  : {fast_peak_tec}"
                     f"{'  <-- 1Hz sampler MISSED this (spike+decay between ticks)' if (fast_peak_tec and tec_left_zero_t is None) else ''}")
            if fast_first_flag is not None:
                emit(lf, f"first error flag : {fast_first_flag[0]} at t={fast_first_flag[1]:.3f}s")
            emit(lf, f"busOff first True: "
                     f"{'t=%.1fs' % busoff_first_t if busoff_first_t is not None else 'busOff NEVER True'}")
            if not faulted:
                if faulted_fast:
                    emit(lf, f"ONSET            : t={fast_tec_left_zero_t:.3f}s (20Hz sampler; the 1Hz "
                             f"sampler saw nothing because TEC decayed back to 0 between ticks)"
                             if fast_tec_left_zero_t is not None else
                             f"ONSET            : flag-only fault, see 'first error flag' above")
                    emit(lf, f"RESULT           : FAULT — peak TEC {fast_peak_tec}, aborted "
                             f"({abort_reason}); did NOT reach busOff")
                else:
                    emit(lf, f"ONSET            : none — TEC stayed 0 for the whole run")
                    emit(lf, f"RESULT           : NO FAULT within {cap_min:g} minutes "
                             f"({'stopped early' if stop_deadline is not None else 'ran full cap'})")
                emit(lf, f"  FINAL_SNAP  @ t={post_snap_t:.1f}s: {post_snap}"
                         if post_snap is not None else "  FINAL_SNAP: (no pandaStates seen)")
            else:
                emit(lf, f"ONSET            : t={fault_t:.1f}s ({fault_snap_ts})  first TEC={tec_left_zero_val}")
                emit(lf, f"CHECKPOINTS ({len(checkpoints)}, every {CHECKPOINT_S:.0f}s from onset):")
                for i, (ct, cts, cs) in enumerate(checkpoints):
                    emit(lf, f"  cp{i} @ t={ct:5.1f}s ({cts}): "
                             f"TEC={cs['transmitErrorCnt']:>3} totErr={cs['totalErrorCnt']:>6} "
                             f"busOff={cs['busOff']} busOffCnt={cs['busOffCnt']}")
                if busoff_first_t is not None:
                    emit(lf, f"RESULT           : ESCALATED TO busOff at t={busoff_first_t:.1f}s")
                    emit(lf, f"  FINAL_SNAP (~10s post-busOff) @ t={post_snap_t:.1f}s: {post_snap}")
                else:
                    acc = (post_snap['totalErrorCnt'] - onset_totalErr) if (post_snap and onset_totalErr is not None) else '?'
                    emit(lf, f"RESULT           : errors occurring but NEVER escalated to busOff within {cap_min:g} min")
                    emit(lf, f"  total errors accumulated onset->cap: {acc}  "
                             f"(totErr {onset_totalErr} -> {post_snap['totalErrorCnt'] if post_snap else '?'})")
                    emit(lf, f"  FINAL_SNAP  @ t={post_snap_t:.1f}s: {post_snap}")
            if session_mode:
                # THE comparison metric: 07-27 measured 2-21s from process start,
                # where TX began immediately. With --engaged-only, TX begins at the
                # engage moment instead, so onset must be measured from first TX.
                emit(lf, f"injection window : "
                         f"{'ENGAGED-ONLY (TX confined to engaged frames)' if args.engaged_only else 'CONTINUOUS (TX from process start)'}")
                emit(lf, f"first OP TX      : "
                         f"{'t=%.3fs' % first_optx_t if first_optx_t is not None else 'NEVER TRANSMITTED'}")
                emit(lf, f"total OP TX      : {total_optx} frames of 0x32E")
                emit(lf, f"engaged samples  : {engaged_frames}")
                _onset = fast_tec_left_zero_t if fast_tec_left_zero_t is not None else tec_left_zero_t
                if first_optx_t is not None and _onset is not None:
                    emit(lf, f"ONSET FROM 1st TX: {_onset - first_optx_t:.3f}s to first error, "
                             f"peak TEC {fast_peak_tec}  <== compare against 07-27's 2-21s")
                elif first_optx_t is not None:
                    emit(lf, f"ONSET FROM 1st TX: no errors during {total_optx} transmitted frames")
                emit(lf, f"mode             : "
                         f"{'CAPTURE (TX gate could open)' if args.capture else 'SHADOW (zero TX)'}")
                emit(lf, f"abort            : "
                         f"{'%s at t=%.2fs' % (abort_reason, abort_t) if abort_reason else 'never tripped'}")
                emit(lf, f"full-rate jsonl  : {jsonl_path}")
                if args.shadow:
                    emit(lf, f"shadow payloads  : /tmp/longitudinal_shadow.jsonl "
                             f"(copy it off /tmp before rebooting — /tmp is wiped)")
            emit(lf, f"first canBusMissing: "
                     f"{'t=%.1fs' % first_busmissing_t if first_busmissing_t is not None else 'NONE'}")
            emit(lf, f"log file         : {log_path}")
            emit(lf, "======================================================")


if __name__ == "__main__":
    main()
