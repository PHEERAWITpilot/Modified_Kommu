#!/usr/bin/env python3
"""Exercise byd_long_conflict_capture.py's capture-mode session logic with a
stubbed cereal: enable-file lifecycle, health publishing, the redundant abort,
and finally-teardown. None of these paths have ever executed before."""
import builtins, json, os, sys, time, types, importlib.util

SCRATCH = os.environ.get("KOMMU_TEST_TMP", "/tmp/kommu_captest")
os.makedirs(SCRATCH, exist_ok=True)
CLAUDE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")

PASS = FAIL = 0
def check(d, got, want):
    global PASS, FAIL
    if got == want: PASS += 1; print(f"  PASS  {d}")
    else: FAIL += 1; print(f"  FAIL  {d}\n          got {got!r}\n          want {want!r}")

# ---- stub cereal.messaging -------------------------------------------------
class FakeCanState:
    def __init__(self): self.transmitErrorCnt=0; self.receiveErrorCnt=0
    busOff=False; errorPassive=False; errorWarning=False
    busOffCnt=0; totalErrorCnt=0; totalTxLostCnt=0; totalRxLostCnt=0
    totalFwdCnt=0; canCoreResetCnt=0; totalTxCnt=0; totalRxCnt=0

class FakePandaState:
    def __init__(self):
        self.canState0=FakeCanState(); self.canState2=FakeCanState()
    faultStatus=0; faults=[]; safetyModel=1; safetyParam=2
    safetyRxChecksInvalid=False; heartbeatLost=False
    safetyTxBlocked=0; txBufferOverflow=0; spiErrorCount=0

class FakeCruise: enabled=True; speed=20.0
class FakeCarState:
    vEgo=11.0; aEgo=0.1; gasPressed=False; brakePressed=False; standstill=False
    cruiseState=FakeCruise()

class FakeSelfdrive: alertText1=""

PS = FakePandaState()
class FakeSM:
    def __init__(self, services):
        self.services=services
        self.recv_frame={s:1 for s in services}
        self._d={'pandaStates':[PS],'carState':FakeCarState(),
                 'selfdriveState':FakeSelfdrive(),'onroadEvents':[]}
    def __getitem__(self,k): return self._d[k]
    def update(self,t=0): pass

class FakeSock:
    def __init__(self,name): self.name=name

def fake_drain(sock, wait_for_one=False):
    time.sleep(0.002)
    return []

msg = types.ModuleType("cereal.messaging")
msg.SubMaster = FakeSM
msg.sub_sock = lambda n, timeout=0: FakeSock(n)
msg.drain_sock = fake_drain
cereal = types.ModuleType("cereal"); cereal.messaging = msg
sys.modules["cereal"]=cereal; sys.modules["cereal.messaging"]=msg

spec = importlib.util.spec_from_file_location("cap", f"{CLAUDE}/byd_long_conflict_capture.py")
cap = importlib.util.module_from_spec(spec); spec.loader.exec_module(cap)

# redirect all gate/output paths into scratch
cap.ENABLE_FILE = f"{SCRATCH}/T_ENABLE"
cap.KILL_FILE   = f"{SCRATCH}/T_KILL"
cap.HEALTH_FILE = f"{SCRATCH}/T_health.json"
cap.SHADOW_FILE = f"{SCRATCH}/T_SHADOW"


OUT = f"{SCRATCH}/out"
os.makedirs(OUT, exist_ok=True)

def clean():
    for p in (cap.ENABLE_FILE, cap.KILL_FILE, cap.HEALTH_FILE, cap.SHADOW_FILE):
        try: os.remove(p)
        except OSError: pass

def run_main(argv, feed_input=None, mutate_after=None):
    """Run cap.main() with argv; optionally answer input() and mutate panda
    state from a background thread after N seconds."""
    import threading
    clean()
    PS.canState0.transmitErrorCnt = 0
    PS.canState0.busOff = PS.canState0.errorPassive = PS.canState0.errorWarning = False
    old_argv, old_input = sys.argv, builtins.input
    sys.argv = ["cap"] + argv
    if feed_input is not None:
        builtins.input = lambda prompt="": feed_input
    seen = {"enable_mid": None, "health_mid": None}
    stop = threading.Event()

    def watcher():
        t0 = time.time()
        while not stop.wait(0.02):
            if seen["enable_mid"] is None and time.time()-t0 > 0.35:
                seen["enable_mid"] = os.path.exists(cap.ENABLE_FILE)
                seen["health_mid"] = (os.path.exists(cap.HEALTH_FILE) and
                                      time.time()-os.stat(cap.HEALTH_FILE).st_mtime < 0.4)
            if mutate_after and time.time()-t0 > mutate_after[0]:
                setattr(PS.canState0, mutate_after[1], mutate_after[2])
                if len(mutate_after) > 3:          # pulse: revert after N seconds
                    time.sleep(mutate_after[3])
                    setattr(PS.canState0, mutate_after[1], 0)
                return
    th = threading.Thread(target=watcher, daemon=True); th.start()
    try:
        cap.main()
    finally:
        stop.set(); th.join(timeout=1)
        sys.argv, builtins.input = old_argv, old_input
    return seen

print("capture-session tests (stubbed cereal)\n")

# --- 1. wrong confirmation phrase must start nothing -----------------------
print("1. confirmation gate")
clean()
try:
    run_main(["--capture","--minutes","0.02","--out-dir",OUT], feed_input="yes")
    check("wrong phrase exits", False, True)
except SystemExit as e:
    check("wrong phrase exits without starting", "did not match" in str(e), True)
check("wrong phrase left NO enable file", os.path.exists(cap.ENABLE_FILE), False)

# --- 2. capture run, no fault: lifecycle + teardown ------------------------
print("\n2. capture run, clean bus")
seen = run_main(["--capture","--minutes","0.015","--out-dir",OUT],
                feed_input=cap.CONFIRM_PHRASE)
check("enable file existed DURING the run", seen["enable_mid"], True)
check("health file was fresh DURING the run", seen["health_mid"], True)
check("enable file removed after run", os.path.exists(cap.ENABLE_FILE), False)
check("health file removed after run", os.path.exists(cap.HEALTH_FILE), False)

h = [f for f in os.listdir(OUT) if f.endswith(".jsonl")]
check("jsonl written", len(h) > 0, True)
recs = [json.loads(l) for l in open(os.path.join(OUT, sorted(h)[-1]))]
check("carState records present", any(r.get("k")=="carState" for r in recs), True)

# --- 3. abort fires on first nonzero TEC -----------------------------------
print("\n3. redundant abort on TEC leaving 0")
seen = run_main(["--capture","--minutes","0.4","--abort-tail","0.3","--out-dir",OUT],
                feed_input=cap.CONFIRM_PHRASE, mutate_after=(0.5,"transmitErrorCnt",7))
check("enable file gone after abort", os.path.exists(cap.ENABLE_FILE), False)
h = sorted(f for f in os.listdir(OUT) if f.endswith(".log"))
log = open(os.path.join(OUT, h[-1])).read()
check("abort banner logged", "ABORT: bus-0 TEC=7" in log, True)
check("summary records abort", "abort            : TEC=7" in log, True)
check("run stopped early (not full 24s)", "t=  24" not in log, True)

# --- 4. abort fires on a flag with TEC still 0 -----------------------------
print("\n4. abort on errorWarning with TEC == 0")
run_main(["--capture","--minutes","0.4","--abort-tail","0.3","--out-dir",OUT],
         feed_input=cap.CONFIRM_PHRASE, mutate_after=(0.5,"errorWarning",True))
h = sorted(f for f in os.listdir(OUT) if f.endswith(".log"))
log = open(os.path.join(OUT, h[-1])).read()
check("aborted on errorWarning", "ABORT: bus-0 errorWarning" in log, True)
check("enable file gone", os.path.exists(cap.ENABLE_FILE), False)

# --- 5. shadow mode needs no phrase and creates no enable file -------------
print("\n5. shadow mode")
seen = run_main(["--shadow","--minutes","0.015","--out-dir",OUT])
check("shadow made NO enable file at any point", seen["enable_mid"], False)
check("shadow file removed on exit", os.path.exists(cap.SHADOW_FILE), False)

# --- 6. default mode is untouched: no gate files at all --------------------
print("\n6. default read-only mode")
seen = run_main(["--minutes","0.015","--out-dir",OUT])
check("no enable file", seen["enable_mid"], False)
check("no health file created", os.path.exists(cap.HEALTH_FILE), False)
check("no shadow file created", os.path.exists(cap.SHADOW_FILE), False)

# --- 7. teardown survives an exception in the loop -------------------------
print("\n7. teardown on unexpected exception")
orig = cap.messaging.drain_sock
boom = {"n":0}
def exploding(sock, wait_for_one=False):
    boom["n"] += 1
    if boom["n"] > 30: raise RuntimeError("simulated failure mid-run")
    time.sleep(0.002); return []
cap.messaging.drain_sock = exploding
try:
    run_main(["--capture","--minutes","0.4","--out-dir",OUT], feed_input=cap.CONFIRM_PHRASE)
    check("exception propagated", False, True)
except RuntimeError:
    check("exception propagated after teardown", True, True)
finally:
    cap.messaging.drain_sock = orig
check("enable file STILL removed after exception", os.path.exists(cap.ENABLE_FILE), False)
check("health file STILL removed after exception", os.path.exists(cap.HEALTH_FILE), False)

# --- 8. spike-and-decay: the 2026-08-19 reporting bug --------------------
print("\n8. TEC spike that decays back to 0 between 1Hz ticks")
# unique tag: .log files open in append mode with 1s-resolution names, so an
# untagged run can land in the same file as an earlier one and mix summaries.
run_main(["--capture","--minutes","0.4","--abort-tail","0.3","--out-dir",OUT,
          "--tag","spikedecay"],
         feed_input=cap.CONFIRM_PHRASE, mutate_after=(0.4,"transmitErrorCnt",84,0.25))
h = sorted(f for f in os.listdir(OUT) if f.endswith("spikedecay.log"))
log = open(os.path.join(OUT, h[-1])).read()
check("abort still fired on the spike", "ABORT: bus-0 TEC=84" in log, True)
check("peak TEC recorded by 20Hz sampler", "PEAK TEC (20Hz)  : 84" in log, True)
check("summary does NOT claim 'TEC stayed 0'", "TEC stayed 0 the whole run" in log, False)
check("summary does NOT claim NO FAULT", "RESULT           : NO FAULT" in log, False)
check("summary reports the fault", "RESULT           : FAULT" in log, True)

print(f"\n  {PASS} passed, {FAIL} failed")
sys.exit(0 if FAIL == 0 else 1)
