#!/usr/bin/env python3
"""HIL-F24-F item 6/7: single future-hardware orchestrator for the fmuv3
Simulation-on-Hardware bench SIM_JSON test.

Two guarded profiles select what feeds sr75_sim_json_responder.py's
--state-file over PPP to a real Pixhawk:

  --profile static  (default, unchanged since HIL-F24-F/Q): a single,
    unchanging static state row (sr75_hil_f24d_static_state_feed.py).
  --profile dynamic (HIL-F24-R1): the real SR-75 6-DOF JSBSim model,
    ground-level and motionless, engines off, RATO/throttle/controls
    pinned at zero for the whole run (sr75_hil_f24r1_dynamic_jsbsim_feed.py
    launching aircraft/sr_75_6_dof/scripts/SR75_hil_f24r1_dynamic_state_feed.xml).
    There is no feedback path from the Pixhawk into JSBSim in either
    profile -- the responder is always started without
    --jsbsim-command-target/--actuator-map (LOG_ONLY actuator mode), so
    incoming actuator commands are only ever logged, never applied to
    hardware or to JSBSim.

An additional, separately-guarded --stage r2 (HIL-F24-R2; requires
--profile dynamic) layers a READ-ONLY Pixhawk-estimator-vs-JSBSim-truth
comparison on top of an otherwise-unchanged HIL-F24-R1 dynamic run:
sr75_hil_f24r2_estimator_capture.py connects read-only to the Pixhawk's
USB MAVLink port (HEARTBEAT/GLOBAL_POSITION_INT/LOCAL_POSITION_NED/
ATTITUDE/GPS_RAW_INT/VFR_HUD/EKF_STATUS_REPORT/SYS_STATUS/STATUSTEXT --
the only messages ever sent are read-only MAV_CMD_SET_MESSAGE_INTERVAL
stream-rate requests) while tailing the same state.csv the responder
reads for JSBSim truth, and sr75_hil_f24r2_estimator_comparison.py scores
the two afterward. Nothing in --stage r2 sends PARAM_SET, arm/disarm,
mode change, mission item, RC override, or actuator command, and nothing
it reads is ever applied to JSBSim or to hardware.

Neither profile, nor --stage r2, ever arms, enables actuator/servo/engine
output, or sends AUTO/mission commands -- this orchestrator has no code
path that could.

Safety gating (item 7): --dry-run is the default. It prints the exact
plan/commands this script WOULD run and exits 0 without touching
anything. Actually starting PPP/the feeder/the responder against real
hardware requires --execute AND a confirmation flag unique to what is
selected: --confirm-static-only (profile=static, unchanged),
--confirm-dynamic-jsbsim (profile=dynamic, HIL-F24-R1), or
--confirm-estimator-comparison (profile=dynamic --stage r2, HIL-F24-R2).
No confirmation flag can substitute for another.

Execution plan (item 6), in order:
  1. Check /dev/ttyACM0 (Pixhawk USB MAVLink) and /dev/ttyUSB0 (PPP UART
     adapter) exist.
  2. Read-only SoH safety precheck (subprocess: sr75_hil_f24c_preflash_
     precheck.py) -- abort here on STOP, before touching PPP at all. Also
     records Pixhawk mode/armed-state/CH7/CH8 safety evidence.
  3. Start PPP (sr75_ppp_start.sh --background).
  4. Verify ppp0 is up and the Pixhawk PPP endpoint responds to ping --
     abort (and stop PPP) before starting SIM_JSON if this fails.
  5. Start the selected feeder (static or dynamic), then the responder
     (sr75_sim_json_responder.py, no actuator/JSBSim command target) --
     feeder before responder, as required. For --stage r2, also start the
     read-only estimator capture process.
  6. Record every artifact (precheck output, ppp status, feeder log,
     responder log/CSV, and for --profile dynamic also the raw JSBSim
     state CSV and a post-run safety-evidence recheck; for --stage r2
     also jsbsim_truth.csv, pixhawk_estimator.csv, estimator_comparison.csv,
     and estimator_summary.json/.md) under a timestamped session directory.
  7. On exit -- success, failure, or exception -- always stop the
     feeder/responder(/capture) and always stop PPP.

No PARAM_SET, arm, mission, RC override, or actuator command is ever sent.
"""
import argparse
import json
import re
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import List, Optional

LAYER2_DIR = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = LAYER2_DIR / "scripts"
SIM_JSON_DIR = LAYER2_DIR / "sim_json"
PPP_DIR = LAYER2_DIR / "ppp"
DEFAULT_SESSIONS_DIR = LAYER2_DIR / "hardware_sessions"

# HIL-F24-R2F: reuses sr75_sim_json_responder.LatestCSVReader (the exact
# HIL-F24-P/Q/S/T-hardened, monotonic-freshness-aware state.csv reader
# the responder and the R2 capture script's own TruthTailer already use)
# for check_r2c_origin_check_readiness()'s state-freshness check --
# not reimplemented.
sys.path.insert(0, str(SIM_JSON_DIR))
import sr75_sim_json_responder as responder  # noqa: E402

# HIL-F24-Q: how long the orchestrator waits, beyond --duration-s, before it
# begins the explicit shutdown sequence (stop responder, then feeder, then
# PPP) -- unchanged from the pre-existing behavior.
ORCHESTRATOR_TEST_MARGIN_S = 2.0

# HIL-F24-Q: how much longer the feeder's OWN --duration-s runs beyond the
# orchestrator's full pre-shutdown wait (--duration-s + ORCHESTRATOR_TEST_
# MARGIN_S). Previously the feeder was given exactly --duration-s and so
# exited on its own ~ORCHESTRATOR_TEST_MARGIN_S seconds *before* the
# orchestrator even began stopping anything -- during that gap the
# responder (which has no duration limit of its own) kept receiving and
# trying to answer SIM_JSON requests against a state file that had
# stopped being updated, producing NO_FRESH_STATE and then STALE_STATE
# replies (the F24-P hardware run's requests 1489-1499). With this margin,
# the feeder is still alive and actively writing for the entire window the
# responder can receive requests, and is only ever stopped by the
# explicit "stop responder, then feeder" sequence below -- never by its
# own internal timeout expiring early.
FEEDER_SHUTDOWN_MARGIN_S = 3.0

PRECHECK_SCRIPT = SCRIPTS_DIR / "sr75_hil_f24c_preflash_precheck.py"
FEEDER_SCRIPT = SIM_JSON_DIR / "sr75_hil_f24d_static_state_feed.py"
# HIL-F24-R1: live JSBSim feeder, selected only by --profile dynamic.
DYNAMIC_FEEDER_SCRIPT = SIM_JSON_DIR / "sr75_hil_f24r1_dynamic_jsbsim_feed.py"
# Must match sr75_hil_f24r1_dynamic_jsbsim_feed.py's own DEFAULT_RAW_OUTPUT
# (and aircraft/sr_75_6_dof/scripts/SR75_hil_f24r1_dynamic_state_feed.xml's
# <output name=...>) exactly -- this orchestrator runs the feeder as a
# separate subprocess, so the path is duplicated here rather than shared
# via import, the same documented-constant convention already used for
# DEFAULT_STATE_FILE elsewhere in this codebase.
DYNAMIC_FEEDER_RAW_OUTPUT = "/tmp/sr75_hil_f24r1_jsbsim_raw.csv"
RESPONDER_SCRIPT = SIM_JSON_DIR / "sr75_sim_json_responder.py"
# HIL-F24-R2: read-only estimator-vs-truth capture/comparison, selected
# only by --stage r2 (which itself requires --profile dynamic).
R2_CAPTURE_SCRIPT = SCRIPTS_DIR / "sr75_hil_f24r2_estimator_capture.py"
R2_COMPARISON_SCRIPT = SCRIPTS_DIR / "sr75_hil_f24r2_estimator_comparison.py"
# HIL-F24-R2C: read-only EKF-origin relatch precondition check, selected
# only by --stage r2c (which itself requires --profile dynamic, and
# performs everything --stage r2 does, plus the relatch workflow first).
R2B_ORIGIN_DIAGNOSTIC_SCRIPT = SCRIPTS_DIR / "sr75_hil_f24r2b_ahrs_origin_diagnostics.py"
# Matches sr75_hil_f24r2b_ahrs_origin_diagnostics.ORIGIN_MISMATCH_WARN_M --
# duplicated here (documented-constant convention, same as
# DYNAMIC_FEEDER_RAW_OUTPUT above) since the orchestrator gates on this
# value directly from the diagnostic's JSON summary rather than importing
# the diagnostic script as a library.
ORIGIN_RELATCH_MAX_MISMATCH_M = 50.0
# HIL-F24-R2E: R2C's responder starts before PPP exists, so it cannot
# bind to args.host_ip (192.168.144.2 -- not yet assigned to any
# interface). 0.0.0.0 is sr75_sim_json_responder.py's own upstream
# --listen-host default; binding to it works immediately and keeps
# working, uninterrupted, once PPP later assigns 192.168.144.2 to ppp0.
R2C_RESPONDER_LISTEN_HOST = "0.0.0.0"
# HIL-F24-R2E: how long to wait after starting the responder before the
# first liveness poll -- long enough that a synchronous startup failure
# (e.g. bind()) has certainly already happened and been reflected in
# proc.poll(), short enough not to meaningfully delay the (already slow,
# operator-blocking) relatch workflow.
RESPONDER_STARTUP_HEALTH_CHECK_S = 1.0

# HIL-F24-R2F: --stage r2c's feeder (and the JSBSim subprocess it tails)
# must stay alive, and keep producing fresh truth, through the ENTIRE
# relatch workflow -- operator power-cycle wait, device reattach,
# PPP restart/verify, SIM_JSON-reply wait, the origin diagnostic, and
# finally the R2 capture window -- not just the capture window alone
# (static/dynamic/--stage r2's feeder_duration_s, unchanged, only ever
# needs to cover that last part). Session sr75_hil_f24r2c_relatch_
# 20260806T065842Z's responder.log captured the failure this fixes
# exactly: replies_sent=1, stale_state_count=305, because the feeder
# (given the same feeder_duration_s formula as every other stage/
# profile, ~35s) stopped tens of seconds before the operator had even
# finished the manual reboot + PPP restart. r2c_feeder_duration_s()
# below computes r2c's feeder lifetime explicitly, as the sum of every
# phase's own already-configurable timeout (task 2's "explicit
# configurable duration, not an arbitrary fixed value") -- growing any
# one phase's timeout automatically grows the feeder's lifetime to
# match, with no separate constant to remember to keep in sync.
#
# The retry/timeout constants below are r2c-only. static/dynamic/--stage r2's
# original single PPP start/verify path is intentionally left untouched.
R2C_PPP_START_TIMEOUT_S = 30.0
R2C_PPP_VERIFY_TIMEOUT_S = 30.0
R2C_ORIGIN_CHECK_TIMEOUT_S = 90.0
DEFAULT_RELATCH_PPP_RETRY_ATTEMPTS = 3
DEFAULT_RELATCH_PPP_RETRY_DELAY_S = 5.0
DEFAULT_RELATCH_PPP_HEALTH_TIMEOUT_S = 30.0
DEFAULT_RELATCH_PPP_HEALTH_POLL_S = 2.0
# HIL-F24-R2F task 2: the one phase with no natural fixed value -- a
# human manually power-cycling the Pixhawk. Generous default (5 minutes)
# but explicit and operator-configurable via --relatch-operator-wait-
# budget-s, not baked into an unlabelled constant.
DEFAULT_RELATCH_OPERATOR_WAIT_BUDGET_S = 300.0
# HIL-F24-R2F task 4: readiness-gate thresholds, checked immediately
# before the origin diagnostic runs (check_r2c_origin_check_readiness()).
# Matches sr75_sim_json_responder.py's own --state-timeout-ms default
# (500.0ms) -- if the responder itself would already reject this state
# as stale, the readiness gate must say so too, not just eventually
# discover it via a confusing downstream failure.
R2C_READINESS_MAX_STATE_AGE_S = 0.5
R2C_READINESS_MIN_REPLIES = 5

PPP_START_SCRIPT = PPP_DIR / "sr75_ppp_start.sh"
PPP_STOP_SCRIPT = PPP_DIR / "sr75_ppp_stop.sh"
PPP_STATUS_SCRIPT = PPP_DIR / "sr75_ppp_status.sh"


class OrchestratorAbort(Exception):
    """Raised to unwind the plan early (adapter missing, precheck STOP,
    PPP failed) -- always caught by main() so cleanup still runs."""


@dataclass
class Step:
    name: str
    description: str
    command: Optional[List[str]] = None


def check_adapter_presence(paths: List[str]) -> dict:
    """Pure(ish)/testable: reports which of the given device paths exist.
    Does not open, configure, or otherwise touch any device."""
    import os
    return {path: os.path.exists(path) for path in paths}


def wait_for_devices(
    paths: List[str], timeout_s: float, poll_interval_s: float = 1.0,
    exists_fn=None, sleep_fn=None, now_fn=None,
) -> bool:
    """HIL-F24-R2C step 3: polls until every path in `paths` exists (the
    Pixhawk USB and PPP UART adapter reappearing after a manual power
    cycle), or timeout_s elapses. Never opens/configures/touches a
    device -- existence check only, same as check_adapter_presence().

    Pure/testable: exists_fn/sleep_fn/now_fn default to os.path.exists/
    time.sleep/time.monotonic but can be injected so tests never need a
    real device or a real wall-clock wait.
    """
    import os
    exists_fn = exists_fn or os.path.exists
    sleep_fn = sleep_fn or time.sleep
    now_fn = now_fn or time.monotonic
    deadline = now_fn() + timeout_s
    while now_fn() < deadline:
        if all(exists_fn(p) for p in paths):
            return True
        sleep_fn(poll_interval_s)
    return all(exists_fn(p) for p in paths)


def count_sim_json_replies(responder_csv_path) -> int:
    """Counts rows in responder.csv with reply_sent==1 -- the number of
    real SIM_JSON requests the responder has successfully answered so
    far. Read-only; returns 0 if the file does not exist yet."""
    import csv
    import os
    if not os.path.exists(responder_csv_path):
        return 0
    count = 0
    with open(responder_csv_path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            if row.get("reply_sent") == "1":
                count += 1
    return count


def wait_for_sim_json_replies(
    responder_csv_path, min_new_replies: int, timeout_s: float, poll_interval_s: float = 1.0,
    count_fn=None, sleep_fn=None, now_fn=None, alive_fn=None,
) -> bool:
    """HIL-F24-R2C step 4: polls responder.csv (read-only) until at least
    `min_new_replies` new reply_sent==1 rows have appeared since this
    call began, or timeout_s elapses -- confirms the freshly-restarted
    PPP link is actually carrying live SIM_JSON traffic, not just that
    the link itself is up (ppp_verify already checks that separately).

    HIL-F24-R2E task 4: if `alive_fn` is given, it is checked on every
    poll iteration; the moment it returns False (the responder process
    has died) this returns False immediately instead of waiting out the
    rest of timeout_s uselessly -- a dead responder can never produce a
    reply no matter how long we wait. Defaults to None (no process-health
    awareness), preserving the exact pre-R2E signature/behavior for any
    other caller.

    Pure/testable: count_fn/sleep_fn/now_fn/alive_fn default to
    count_sim_json_replies/time.sleep/time.monotonic/None but can be
    injected.
    """
    count_fn = count_fn or count_sim_json_replies
    sleep_fn = sleep_fn or time.sleep
    now_fn = now_fn or time.monotonic
    start_count = count_fn(responder_csv_path)
    deadline = now_fn() + timeout_s
    while now_fn() < deadline:
        if count_fn(responder_csv_path) - start_count >= min_new_replies:
            return True
        if alive_fn is not None and not alive_fn():
            return False
        sleep_fn(poll_interval_s)
    return count_fn(responder_csv_path) - start_count >= min_new_replies


def cleanup_stale_raw_jsbsim_evidence(raw_output_path) -> bool:
    """HIL-F24-R2E task 5: removes a stale raw JSBSim output file left
    over from an earlier invocation, if one is present, before this run's
    feeder starts writing to the same fixed path. Returns True if a file
    was actually removed, False if there was nothing to remove. Read/
    delete-only -- never touches state.csv, responder.csv, or anything
    else; the feeder (sr75_hil_f24r1_dynamic_jsbsim_feed.py's own
    start_jsbsim()) already does this same cleanup internally, so this is
    a defensive, harmless duplicate at the orchestrator level."""
    try:
        Path(raw_output_path).unlink()
        return True
    except FileNotFoundError:
        return False


def wait_for_process_startup_health(proc, grace_s: float = None, sleep_fn=None) -> bool:
    """HIL-F24-R2E task 3/4: a brief grace period, then a single liveness
    poll -- catches a process that exits almost immediately after
    starting (e.g. a bind() failure) *before* committing to the slow,
    operator-blocking steps that follow it (R2C's power-cycle prompt and
    device-reattach wait), rather than only discovering the failure much
    later when wait_for_sim_json_replies() finally times out.

    Pure/testable: `proc` only needs a `.poll()` method (a real
    subprocess.Popen or a fake with the same shape); grace_s/sleep_fn
    default to RESPONDER_STARTUP_HEALTH_CHECK_S/time.sleep but can be
    injected so tests never need a real process or a real sleep.
    """
    sleep_fn = sleep_fn or time.sleep
    if grace_s is None:
        grace_s = RESPONDER_STARTUP_HEALTH_CHECK_S
    sleep_fn(grace_s)
    return proc.poll() is None


def prompt_operator_power_cycle(input_fn=None, print_fn=None) -> None:
    """HIL-F24-R2C step 2: blocks for explicit operator confirmation that
    the Pixhawk has been manually power-cycled. Sends nothing to the
    Pixhawk itself -- a host-side console prompt only. input_fn/print_fn
    injectable so tests never block on real console input."""
    input_fn = input_fn or input
    print_fn = print_fn or print
    print_fn(
        "\n=== HIL-F24-R2C: manual EKF-origin relatch ===\n"
        "The dynamic feeder and responder are already running and serving\n"
        "valid SIM_JSON data.\n"
        "ACTION REQUIRED: power-cycle the Pixhawk now (remove and reapply\n"
        "power) so its very first GPS fix after boot is this bench's real\n"
        "position, not the stale/default EKF origin (see HIL_F24_R2B_\n"
        "ahrs_backend_diagnosis.md).\n"
    )
    input_fn("Press ENTER once the Pixhawk has been power-cycled and is booting back up... ")


def evaluate_origin_relatch_summary(summary: dict, max_mismatch_m: float) -> tuple:
    """Pure: given sr75_hil_f24r2b_ahrs_origin_diagnostics.py's
    --summary-json output, decides whether the relatch succeeded. HIL-
    F24-R2C gates specifically on GPS_GLOBAL_ORIGIN (not HOME_POSITION,
    which was already confirmed correct in HIL-F24-R2B and is not what
    was poisoned) being within max_mismatch_m of the given truth.
    Returns (ok: bool, reason: str)."""
    mismatch_m = summary.get("gps_global_origin_mismatch_m")
    if mismatch_m is None:
        return False, "GPS_GLOBAL_ORIGIN was not received within the diagnostic's listen window"
    if mismatch_m > max_mismatch_m:
        return False, f"GPS_GLOBAL_ORIGIN vs. truth mismatch {mismatch_m:.2f} m exceeds {max_mismatch_m} m"
    return True, f"GPS_GLOBAL_ORIGIN vs. truth mismatch {mismatch_m:.2f} m is within {max_mismatch_m} m"


def r2c_feeder_duration_s(
    operator_wait_budget_s: float, device_timeout_s: float, sim_json_timeout_s: float,
    duration_s: float, ppp_retry_attempts: int = DEFAULT_RELATCH_PPP_RETRY_ATTEMPTS,
    ppp_retry_delay_s: float = DEFAULT_RELATCH_PPP_RETRY_DELAY_S,
    ppp_health_timeout_s: float = DEFAULT_RELATCH_PPP_HEALTH_TIMEOUT_S,
) -> float:
    """HIL-F24-R2F task 1/2: --stage r2c's feeder --duration-s, computed
    as the sum of every phase of the relatch workflow the feeder must
    stay alive (and producing fresh truth) through, in order:
    operator power-cycle wait, device reattach, PPP restart, the post-
    start negotiate sleep, PPP verify, the SIM_JSON-reply wait, the
    origin diagnostic, the R2 capture window itself
    (duration_s + ORCHESTRATOR_TEST_MARGIN_S), and the same
    FEEDER_SHUTDOWN_MARGIN_S safety margin every other profile/stage's
    feeder already gets (HIL-F24-Q) so it is never stopped by its own
    internal timeout instead of the orchestrator's explicit shutdown
    sequence. Pure function of the already-configurable phase timeouts
    -- not an arbitrary fixed value (task 2)."""
    ppp_retry_attempts = max(1, int(ppp_retry_attempts))
    ppp_retry_delay_total_s = max(0, ppp_retry_attempts - 1) * ppp_retry_delay_s
    ppp_attempt_total_s = ppp_retry_attempts * (R2C_PPP_START_TIMEOUT_S + ppp_health_timeout_s)
    return (
        operator_wait_budget_s
        + device_timeout_s
        + ppp_attempt_total_s
        + ppp_retry_delay_total_s
        + sim_json_timeout_s
        + R2C_ORIGIN_CHECK_TIMEOUT_S
        + duration_s + ORCHESTRATOR_TEST_MARGIN_S
        + FEEDER_SHUTDOWN_MARGIN_S
    )


def check_r2c_origin_check_readiness(
    feeder_proc, state_csv_path, responder_csv_path,
    min_replies: int = None, max_state_age_s: float = None,
    count_fn=None, reader_factory=None,
) -> tuple:
    """HIL-F24-R2F task 4/5: the three preconditions required before the
    origin diagnostic may run, checked in this order so the first
    genuine failure is always the one reported (task 5's "exact
    reason"): (1) the feeder process is still alive -- a dead feeder can
    never produce fresh truth again; (2) state.csv's newest snapshot is
    fresh (age <= max_state_age_s, matching sr75_sim_json_responder.py's
    own --state-timeout-ms default so this gate and the responder's own
    freshness gate never disagree); (3) at least min_replies SIM_JSON
    requests have actually been answered (reply_sent=1) so far.

    Returns (ready: bool, reason: str). Pure/testable: count_fn/
    reader_factory dependency-injected (default to count_sim_json_
    replies / sr75_sim_json_responder.LatestCSVReader) so tests never
    need a real process, a real state.csv, or a real responder.csv.
    """
    min_replies = R2C_READINESS_MIN_REPLIES if min_replies is None else min_replies
    max_state_age_s = R2C_READINESS_MAX_STATE_AGE_S if max_state_age_s is None else max_state_age_s
    count_fn = count_fn or count_sim_json_replies
    reader_factory = reader_factory or responder.LatestCSVReader

    if feeder_proc is not None and feeder_proc.poll() is not None:
        return False, f"feeder process exited (exit code {feeder_proc.returncode}) before the origin check could run"

    reader = reader_factory(state_csv_path)
    try:
        _row, _line, age_s, _st = reader.read_latest()
    except responder.StateError as exc:
        return False, f"state.csv not ready: {exc}"
    if age_s > max_state_age_s:
        return False, (
            f"state.csv is stale (age {age_s * 1000.0:.1f} ms exceeds "
            f"{max_state_age_s * 1000.0:.0f} ms) -- feeder truth stopped updating"
        )

    replies = count_fn(responder_csv_path)
    if replies < min_replies:
        return False, f"only {replies} SIM_JSON replies recorded so far (need >= {min_replies})"

    return True, f"feeder alive, state.csv fresh ({age_s * 1000.0:.1f} ms old), {replies} replies recorded"


def wait_for_r2c_live_sim_json_replies(
    responder_csv_path, state_csv_path, feeder_proc,
    min_new_replies: int, timeout_s: float,
    max_state_age_s: float = None, poll_interval_s: float = 1.0,
    count_fn=None, reader_factory=None, sleep_fn=None, now_fn=None,
) -> tuple:
    """HIL-F24-R2F task 4/5: R2C-only SIM_JSON readiness wait.

    Unlike wait_for_sim_json_replies(), this is deliberately tied to the
    dynamic-truth relatch workflow: every poll proves the feeder is still
    alive and state.csv is fresh before waiting longer. If the feeder exits
    or truth becomes stale, return immediately with that exact reason.
    """
    max_state_age_s = R2C_READINESS_MAX_STATE_AGE_S if max_state_age_s is None else max_state_age_s
    count_fn = count_fn or count_sim_json_replies
    sleep_fn = sleep_fn or time.sleep
    now_fn = now_fn or time.monotonic

    start_count = count_fn(responder_csv_path)
    deadline = now_fn() + timeout_s
    while now_fn() < deadline:
        ready, reason = check_r2c_origin_check_readiness(
            feeder_proc, state_csv_path, responder_csv_path,
            min_replies=0, max_state_age_s=max_state_age_s,
            count_fn=count_fn, reader_factory=reader_factory,
        )
        if not ready:
            return False, reason

        replies = count_fn(responder_csv_path) - start_count
        if replies >= min_new_replies:
            return True, (
                f"{replies} new SIM_JSON replies recorded, feeder alive, "
                f"{reason.split(', ', 1)[1] if ', ' in reason else reason}"
            )
        sleep_fn(poll_interval_s)

    replies = count_fn(responder_csv_path) - start_count
    return False, (
        f"only {replies} new SIM_JSON replies recorded within {timeout_s}s "
        f"(need >= {min_new_replies})"
    )


def ppp_pid_file_for_device(device: str) -> Path:
    """Matches sr75_ppp_start.sh/sr75_ppp_stop.sh's per-device PID path."""
    return Path(f"/tmp/sr75_ppp_{device.replace('/', '_')}.pid")


def ppp_status_health(status_text: str, host_ip: str, pixhawk_ip: str) -> tuple:
    """Returns (healthy, reason) for sr75_ppp_status.sh output."""
    link_line = ""
    for line in status_text.splitlines():
        if re.match(r"^\s*\d+:\s+ppp0:\s+<[^>]+>", line):
            link_line = line
            break
    if "UP" not in link_line or "LOWER_UP" not in link_line:
        return False, "ppp0 is not UP/LOWER_UP"

    inet_pattern = (
        r"\binet\s+"
        + re.escape(host_ip)
        + r"(?:/\d+)?\s+peer\s+"
        + re.escape(pixhawk_ip)
        + r"(?:/\d+)?\b"
    )
    if re.search(inet_pattern, status_text) is None:
        return False, f"ppp0 local/peer addresses missing ({host_ip} peer {pixhawk_ip})"

    ping_match = re.search(r"(\d+)\s+packets transmitted,\s+(\d+)\s+(?:packets )?received", status_text)
    if ping_match is None:
        return False, "ping result missing"
    transmitted = int(ping_match.group(1))
    received = int(ping_match.group(2))
    if transmitted <= 0 or received != transmitted:
        return False, f"ping failed ({received}/{transmitted} received)"

    return True, f"ppp0 UP/LOWER_UP, {host_ip} peer {pixhawk_ip}, ping {received}/{transmitted}"


def ppp_status_progress(status_text: str) -> bool:
    """True when a not-yet-healthy status still shows link progress."""
    ping_match = re.search(r"(\d+)\s+packets transmitted,\s+(\d+)\s+(?:packets )?received", status_text)
    if ping_match is not None and int(ping_match.group(2)) > 0:
        return True
    return (
        re.search(r"^\s*\d+:\s+ppp0:\s+<", status_text, re.MULTILINE) is not None
        or re.search(r"\binet\s+\d+\.\d+\.\d+\.\d+", status_text) is not None
    )


def r2c_ppp_cleanup_steps(device: str) -> List[Step]:
    """R2C-only stale PPP cleanup before a post-reboot reconnect attempt."""
    return [
        Step("ppp_cleanup_stop", f"Stop stale pppd for {device}", ["sudo", str(PPP_STOP_SCRIPT), "--device", device]),
        Step("ppp_cleanup_pid", "Remove stale SR-75 PPP pid file", ["sudo", "rm", "-f", str(ppp_pid_file_for_device(device))]),
        Step("ppp_cleanup_ppp0", "Remove stale/down ppp0 if present", ["sudo", "ip", "link", "delete", "ppp0"]),
    ]


def should_proceed_to_hardware(
    execute: bool,
    confirm_static_only: bool,
    confirm_dynamic_jsbsim: bool = False,
    profile: str = "static",
    confirm_estimator_comparison: bool = False,
    stage: str = "none",
    confirm_origin_relatch: bool = False,
) -> tuple:
    """Pure: item 7's gating logic. --dry-run (i.e. --execute missing, or
    the selected mode's own confirmation flag missing) always keeps this
    a dry run.

    HIL-F24-R1: the confirmation flag checked is selected by `profile` --
    --confirm-static-only for profile="static" (the original, unchanged
    default) and --confirm-dynamic-jsbsim for profile="dynamic". A
    profile's own confirmation flag can never gate the other profile's
    run (e.g. --confirm-static-only alone cannot authorize a dynamic
    run), so the two guarded modes cannot be confused for one another.
    confirm_dynamic_jsbsim/profile both default such that calling this
    exactly as before HIL-F24-R1 (2 positional/keyword args) reproduces
    the original static-only behavior byte-for-byte.

    HIL-F24-R2: stage="r2" (which build_arg_parser()/main() only accept
    together with profile="dynamic" -- see main()'s own validation)
    requires --confirm-estimator-comparison specifically -- a
    confirmation unique to R2, not satisfiable by
    --confirm-dynamic-jsbsim or --confirm-static-only either.
    confirm_estimator_comparison/stage both default such that existing
    calls (2-4 args, no stage) are entirely unaffected.

    HIL-F24-R2C: stage="r2c" requires --confirm-origin-relatch
    specifically -- a confirmation unique to R2C (acknowledging the
    manual power-cycle step this stage will prompt for), not satisfiable
    by any of the other three confirm flags either. confirm_origin_
    relatch defaults such that existing calls are entirely unaffected.
    """
    if stage == "r2c":
        confirm = confirm_origin_relatch
        confirm_flag_name = "--confirm-origin-relatch"
    elif stage == "r2":
        confirm = confirm_estimator_comparison
        confirm_flag_name = "--confirm-estimator-comparison"
    elif profile == "dynamic":
        confirm = confirm_dynamic_jsbsim
        confirm_flag_name = "--confirm-dynamic-jsbsim"
    else:
        confirm = confirm_static_only
        confirm_flag_name = "--confirm-static-only"
    if execute and confirm:
        return True, ""
    if execute and not confirm:
        return False, f"--execute given without {confirm_flag_name} -- staying in dry-run"
    if confirm and not execute:
        return False, f"{confirm_flag_name} given without --execute -- staying in dry-run"
    return False, (
        "dry-run (default): pass --execute --confirm-static-only "
        "(or --profile dynamic --execute --confirm-dynamic-jsbsim, "
        "or --profile dynamic --stage r2 --execute --confirm-estimator-comparison, "
        "or --profile dynamic --stage r2c --execute --confirm-origin-relatch) "
        "to run against real hardware"
    )


def build_session_dir(
    base_dir: Path, now: Optional[datetime] = None, profile: str = "static", stage: str = "none",
) -> Path:
    stamp = (now or datetime.now(timezone.utc)).strftime("%Y%m%dT%H%M%SZ")
    if stage == "r2c":
        name = "sr75_hil_f24r2c_relatch"
    elif stage == "r2":
        name = "sr75_hil_f24r2_estimator"
    elif profile == "dynamic":
        name = "sr75_hil_f24r1_dynamic"
    else:
        name = "sr75_hil_f24f_static"
    return base_dir / f"{name}_{stamp}"


def build_execution_plan(args) -> List[Step]:
    """Pure: the full ordered plan, independent of dry-run/execute -- used
    both to print the dry-run summary and (in main()) to drive real
    execution, so the two can never drift apart."""
    ppp_start_cmd = [
        "sudo", str(PPP_START_SCRIPT),
        "--device", args.ppp_device, "--baud", str(args.ppp_baud),
        "--host-ip", args.host_ip, "--pixhawk-ip", args.pixhawk_ip, "--background",
    ]
    ppp_stop_cmd = ["sudo", str(PPP_STOP_SCRIPT), "--device", args.ppp_device]
    ppp_status_cmd = [
        str(PPP_STATUS_SCRIPT),
        "--pixhawk-ip", args.pixhawk_ip, "--responder-ip", args.host_ip,
        "--responder-port", str(args.listen_port),
    ]
    precheck_cmd = [
        sys.executable, str(PRECHECK_SCRIPT), "--pixhawk", args.pixhawk, "--baud", str(args.baud),
    ]
    # HIL-F24-Q: the feeder runs FEEDER_SHUTDOWN_MARGIN_S longer than the
    # orchestrator's total pre-shutdown wait (--duration-s +
    # ORCHESTRATOR_TEST_MARGIN_S), so it is still alive when the
    # orchestrator issues its explicit stop-responder-then-feeder sequence
    # (see main()'s `finally:` block) instead of exiting on its own first.
    # HIL-F24-R1: this margin arithmetic is identical for both profiles --
    # only which script is launched, and with which args, differs below.
    feeder_duration_s = args.duration_s + ORCHESTRATOR_TEST_MARGIN_S + FEEDER_SHUTDOWN_MARGIN_S
    profile = getattr(args, "profile", "static")
    if profile == "dynamic":
        feeder_cmd = [
            sys.executable, str(DYNAMIC_FEEDER_SCRIPT), "--output", "{state_csv}",
            "--duration-s", str(feeder_duration_s),
            # Deliberately no arguments derived from any Pixhawk/responder
            # input -- this feeder only ever reads JSBSim's own live state
            # and republishes it; there is no feedback path into JSBSim.
        ]
        feeder_description = "Start live JSBSim state feeder (HIL-F24-R1; before responder)"
    else:
        feeder_cmd = [
            sys.executable, str(FEEDER_SCRIPT), "--output", "{state_csv}",
            "--duration-s", str(feeder_duration_s), "--rate-hz", str(args.rate_hz),
        ]
        feeder_description = "Start static-state feeder (before responder)"
    responder_cmd = [
        sys.executable, str(RESPONDER_SCRIPT),
        "--listen-host", args.host_ip, "--listen-port", str(args.listen_port),
        "--state-file", "{state_csv}", "--log-csv", "{responder_csv}", "--strict",
        # Deliberately no --jsbsim-command-target / --actuator-map: this
        # keeps the responder in LOG_ONLY actuator mode, so there is no
        # actuator/PWM command path to send anywhere, matching item 9's
        # "never enable actuator output". Unconditional for both profiles.
    ]
    stage = getattr(args, "stage", "none")
    if stage == "r2c":
        # HIL-F24-R2C: feeder + responder start FIRST (before PPP even
        # starts, let alone the relatch prompt) so they are already
        # warm and serving valid SIM_JSON data by the time the operator
        # power-cycles the Pixhawk and PPP comes back up -- see
        # HIL_F24_R2C_ekf_origin_relatch.md and prompt_operator_power_
        # cycle()'s own printed rationale. PPP itself is started/verified
        # only AFTER the reattach wait, since the pre-power-cycle PPP
        # link (if any) is torn down by the power cycle regardless.
        #
        # HIL-F24-R2E: because the responder starts before PPP exists at
        # all, it cannot bind to args.host_ip (192.168.144.2) -- that
        # address isn't assigned to any local interface until pppd
        # negotiates it, so binding to it here fails immediately
        # (EADDRNOTAVAIL) and the responder process exits before the
        # relatch workflow even prompts the operator. R2C's responder
        # binds R2C_RESPONDER_LISTEN_HOST (0.0.0.0 -- the responder
        # script's own upstream default, see sr75_sim_json_responder.py's
        # --listen-host) instead: a socket already bound to 0.0.0.0
        # keeps working, with zero interruption, once 192.168.144.2 is
        # assigned to ppp0 later -- this is standard, well-defined socket
        # behavior, not a race. static/dynamic/--stage r2 are completely
        # unaffected (their own responder_cmd, built above, is untouched;
        # they only ever start the responder after ppp_verify already
        # confirmed the PPP link, and args.host_ip, up).
        r2c_responder_cmd = [
            sys.executable, str(RESPONDER_SCRIPT),
            "--listen-host", R2C_RESPONDER_LISTEN_HOST, "--listen-port", str(args.listen_port),
            "--state-file", "{state_csv}", "--log-csv", "{responder_csv}", "--strict",
        ]
        # HIL-F24-R2F: r2c's feeder gets its own, much longer --duration-s
        # -- see r2c_feeder_duration_s()'s docstring -- instead of the
        # feeder_duration_s shared by static/dynamic/--stage r2 above
        # (unchanged, task 3/6). Always the dynamic (JSBSim) feeder script,
        # since --stage r2c requires --profile dynamic (enforced in main()).
        r2c_feeder_duration_s_value = r2c_feeder_duration_s(
            operator_wait_budget_s=args.relatch_operator_wait_budget_s,
            device_timeout_s=args.relatch_device_timeout_s,
            sim_json_timeout_s=args.relatch_sim_json_timeout_s,
            duration_s=args.duration_s,
            ppp_retry_attempts=args.relatch_ppp_retry_attempts,
            ppp_retry_delay_s=args.relatch_ppp_retry_delay_s,
            ppp_health_timeout_s=args.relatch_ppp_health_timeout_s,
        )
        r2c_feeder_cmd = [
            sys.executable, str(DYNAMIC_FEEDER_SCRIPT), "--output", "{state_csv}",
            "--duration-s", str(r2c_feeder_duration_s_value),
        ]
        origin_diagnostic_cmd = [
            sys.executable, str(R2B_ORIGIN_DIAGNOSTIC_SCRIPT),
            "--pixhawk", args.pixhawk, "--baud", str(args.baud),
            "--listen-s", str(args.relatch_listen_s),
            "--truth-lat", str(args.relatch_truth_lat), "--truth-lon", str(args.relatch_truth_lon),
            "--truth-alt-m", str(args.relatch_truth_alt_m),
            "--summary-json", "{origin_relatch_summary_json}",
        ]
        plan = [
            Step("check_adapters", f"Check {args.pixhawk} and {args.ppp_device} exist", None),
            Step("precheck", "Read-only SoH safety precheck (abort on STOP, before touching PPP)", precheck_cmd),
            Step("start_feeder", feeder_description + " (HIL-F24-R2C: before the relatch step)", r2c_feeder_cmd),
            Step(
                "start_responder",
                f"Start responder in LOG_ONLY mode, bound to {R2C_RESPONDER_LISTEN_HOST} (before PPP exists)",
                r2c_responder_cmd,
            ),
            Step("prompt_power_cycle", "Prompt operator to manually power-cycle the Pixhawk (blocks for ENTER)", None),
            Step("wait_for_reattach", f"Wait for {args.pixhawk} and {args.ppp_device} to reappear", None),
            Step("ppp_start", "(Re)start PPP link now that the Pixhawk has rebooted", ppp_start_cmd),
            Step("ppp_verify", "Verify ppp0 is up and Pixhawk PPP endpoint responds to ping", ppp_status_cmd),
            Step("wait_for_sim_json_replies", "Wait for the responder to answer real post-reboot SIM_JSON requests", None),
            Step(
                "run_origin_relatch_check",
                "Read-only EKF-origin diagnostic; ABORT unless GPS_GLOBAL_ORIGIN is within "
                f"{ORIGIN_RELATCH_MAX_MISMATCH_M} m of truth (HIL-F24-R2C gate)",
                origin_diagnostic_cmd,
            ),
        ]
    else:
        plan = [
            Step("check_adapters", f"Check {args.pixhawk} and {args.ppp_device} exist", None),
            Step("precheck", "Read-only SoH safety precheck (abort on STOP, before touching PPP)", precheck_cmd),
            Step("ppp_start", "Start PPP link (only reached if --execute plus the profile's confirm flag)", ppp_start_cmd),
            Step("ppp_verify", "Verify ppp0 is up and Pixhawk PPP endpoint responds to ping", ppp_status_cmd),
            Step("start_feeder", feeder_description, feeder_cmd),
            Step("start_responder", "Start responder in LOG_ONLY mode (no actuator output)", responder_cmd),
        ]
    if stage in ("r2", "r2c"):
        # HIL-F24-R2: runs for the same window as the feeder (its own
        # margin arithmetic, so it stays alive at least as long as the
        # responder can receive requests) -- read-only the whole time.
        # Kept alive at least as long as the feeder (same margin), so it is
        # never stopped early relative to the feeder/responder -- mirrors
        # FEEDER_SHUTDOWN_MARGIN_S's rationale above. HIL-F24-R2F: for
        # r2c this is the feeder's own (much longer) r2c_feeder_duration_
        # s_value, not the shared feeder_duration_s -- r2 is unaffected.
        capture_duration_s = r2c_feeder_duration_s_value if stage == "r2c" else feeder_duration_s
        capture_cmd = [
            sys.executable, str(R2_CAPTURE_SCRIPT),
            "--pixhawk", args.pixhawk, "--baud", str(args.baud),
            "--state-file", "{state_csv}", "--duration-s", str(capture_duration_s),
            "--jsbsim-truth-csv", "{jsbsim_truth_csv}",
            "--pixhawk-estimator-csv", "{pixhawk_estimator_csv}",
            # Deliberately no PARAM_SET/arm/mode/mission/RC-override/
            # actuator argument exists on this script at all -- see its
            # own module docstring.
        ]
        comparison_cmd = [
            sys.executable, str(R2_COMPARISON_SCRIPT),
            "--truth-csv", "{jsbsim_truth_csv}", "--pixhawk-csv", "{pixhawk_estimator_csv}",
            "--comparison-csv", "{estimator_comparison_csv}",
            "--summary-json", "{estimator_summary_json}", "--summary-md", "{estimator_summary_md}",
            "--alignment-tolerance-s", str(args.r2_alignment_tolerance_s),
            "--settle-s", str(args.r2_settle_s),
        ]
        plan.append(Step(
            "start_estimator_capture",
            "Start read-only Pixhawk-estimator/JSBSim-truth capture (HIL-F24-R2; after responder)",
            capture_cmd,
        ))
        plan.append(Step(
            "run_estimator_comparison",
            "Score captured estimator output against JSBSim truth (HIL-F24-R2; after the test window)",
            comparison_cmd,
        ))
    plan.append(Step("ppp_stop", "Stop PPP (always run on exit, success or failure)", ppp_stop_cmd))
    return plan


def session_artifact_paths(session_dir) -> dict:
    """Every {placeholder} name a Step's command can reference, rendered
    relative to session_dir (a Path for a real run, or a display string
    like "<session_dir>" for the dry-run listing). Centralized so
    print_plan()/run_step()/main() can never drift out of sync on which
    placeholders exist."""
    session_dir = str(session_dir)
    return {
        "state_csv": f"{session_dir}/state.csv",
        "responder_csv": f"{session_dir}/responder.csv",
        "jsbsim_truth_csv": f"{session_dir}/jsbsim_truth.csv",
        "pixhawk_estimator_csv": f"{session_dir}/pixhawk_estimator.csv",
        "estimator_comparison_csv": f"{session_dir}/estimator_comparison.csv",
        "estimator_summary_json": f"{session_dir}/estimator_summary.json",
        "estimator_summary_md": f"{session_dir}/estimator_summary.md",
        "origin_relatch_summary_json": f"{session_dir}/origin_relatch_summary.json",
    }


def print_plan(plan: List[Step], args) -> None:
    print("Execution plan (dry-run -- nothing below has been executed):")
    placeholders = session_artifact_paths("<session_dir>")
    for i, step in enumerate(plan, start=1):
        print(f"  {i}. [{step.name}] {step.description}")
        if step.command:
            rendered = " ".join(step.command).format(**placeholders)
            print(f"       $ {rendered}")


def run_step(step: Step, session_dir: Path, log_path: Path, timeout_s: Optional[float] = None) -> int:
    rendered = [part.format(**session_artifact_paths(session_dir)) for part in step.command]
    with open(log_path, "w") as log_file:
        proc = subprocess.run(rendered, stdout=log_file, stderr=subprocess.STDOUT, timeout=timeout_s)
    return proc.returncode


def restart_r2c_ppp_with_retries(
    args, plan_by_name: dict, session_dir: Path,
    run_step_fn=run_step, sleep_fn=time.sleep, now_fn=time.monotonic, print_fn=print,
) -> tuple:
    """HIL-F24-R2G: bounded R2C-only PPP reconnect after Pixhawk reboot."""
    attempts = max(1, int(args.relatch_ppp_retry_attempts))
    last_reason = "no attempts made"
    last_start_log = None
    last_verify_log = None

    for attempt in range(1, attempts + 1):
        print_fn(f"R2C PPP reconnect attempt {attempt}/{attempts}: cleanup before start")
        for cleanup_step in r2c_ppp_cleanup_steps(args.ppp_device):
            cleanup_log = session_dir / f"ppp_r2c_attempt_{attempt}_{cleanup_step.name}.log"
            run_step_fn(cleanup_step, session_dir, cleanup_log, timeout_s=R2C_PPP_START_TIMEOUT_S)

        print_fn(f"R2C PPP reconnect attempt {attempt}/{attempts}: starting pppd")
        last_start_log = session_dir / f"ppp_r2c_attempt_{attempt}_start.log"
        start_rc = run_step_fn(plan_by_name["ppp_start"], session_dir, last_start_log, timeout_s=R2C_PPP_START_TIMEOUT_S)
        start_text = last_start_log.read_text(errors="replace") if last_start_log and last_start_log.exists() else ""
        if start_rc != 0 or "Modem hangup" in start_text:
            last_reason = f"ppp start exited {start_rc}" if start_rc != 0 else "pppd reported Modem hangup"
        else:
            deadline = now_fn() + args.relatch_ppp_health_timeout_s
            while now_fn() < deadline:
                poll_index = (
                    len(list(session_dir.glob(f"ppp_r2c_attempt_{attempt}_verify_*.log"))) + 1
                    if session_dir.exists() else 1
                )
                last_verify_log = session_dir / f"ppp_r2c_attempt_{attempt}_verify_{poll_index}.log"
                verify_rc = run_step_fn(
                    plan_by_name["ppp_verify"], session_dir, last_verify_log, timeout_s=R2C_PPP_VERIFY_TIMEOUT_S,
                )
                verify_text = last_verify_log.read_text(errors="replace") if last_verify_log.exists() else ""
                healthy, health_reason = ppp_status_health(verify_text, args.host_ip, args.pixhawk_ip)

                if verify_rc == 0 and healthy:
                    return True, f"attempt {attempt}/{attempts} succeeded: {health_reason}"

                progress = ppp_status_progress(verify_text)
                last_reason = f"ppp verify exited {verify_rc}" if verify_rc != 0 else health_reason
                if progress:
                    last_reason += " (PPP negotiation/link progress observed)"
                print_fn(
                    f"R2C PPP reconnect attempt {attempt}/{attempts}: waiting for healthy link -- {last_reason}"
                )
                sleep_fn(args.relatch_ppp_health_poll_s)

        if "Modem hangup" in start_text:
            last_reason = "pppd reported Modem hangup"

        if attempt < attempts:
            print_fn(
                f"R2C PPP reconnect attempt {attempt}/{attempts} failed: {last_reason}; "
                f"retrying after {args.relatch_ppp_retry_delay_s}s"
            )
            sleep_fn(args.relatch_ppp_retry_delay_s)

    return False, (
        f"PPP reconnect failed after {attempts} attempts; last reason: {last_reason}; "
        f"last start log: {last_start_log}; last verify log: {last_verify_log}"
    )


def record_dynamic_profile_evidence(args, session_dir: Path) -> None:
    """HIL-F24-R1 task 6: --profile dynamic-only evidence, recorded on
    every exit path (called from main()'s `finally:` block).

    - Copies JSBSim's own raw state CSV into the session directory
      (best-effort: it may not exist if the feeder never actually
      started, e.g. an early precheck/PPP abort).
    - Re-runs the same read-only safety precheck (Pixhawk mode, armed
      state, CH7/CH8 PWM) used before the test (see the "precheck" step,
      captured in precheck.log) a second time, into postcheck.log --
      giving before-and-after safety evidence for the run, not just a
      single pre-test snapshot.

    Never touches PARAM_SET, arm state, or actuator output -- this only
    ever reads.
    """
    raw_jsbsim_path = Path(DYNAMIC_FEEDER_RAW_OUTPUT)
    if raw_jsbsim_path.exists():
        try:
            shutil.copy2(raw_jsbsim_path, session_dir / "jsbsim_state.csv")
            print(f"Recorded JSBSim state: {session_dir / 'jsbsim_state.csv'}")
        except OSError as exc:
            print(f"WARNING: could not copy JSBSim raw state: {exc}")
    else:
        print(f"WARNING: JSBSim raw state file not found at {raw_jsbsim_path} -- nothing to record")

    postcheck_cmd = [sys.executable, str(PRECHECK_SCRIPT), "--pixhawk", args.pixhawk, "--baud", str(args.baud)]
    postcheck_log = session_dir / "postcheck.log"
    try:
        with open(postcheck_log, "w") as log_file:
            subprocess.run(postcheck_cmd, stdout=log_file, stderr=subprocess.STDOUT, timeout=60.0)
        print(f"Recorded post-run safety precheck: {postcheck_log}")
    except (subprocess.TimeoutExpired, OSError) as exc:
        print(f"WARNING: post-run safety precheck failed to run: {exc}")


def build_arg_parser():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--pixhawk", default="/dev/ttyACM0", help="Pixhawk USB MAVLink device")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--ppp-device", default="/dev/ttyUSB0", help="PPP UART adapter device")
    parser.add_argument("--ppp-baud", type=int, default=921600)
    parser.add_argument("--host-ip", default="192.168.144.2")
    parser.add_argument("--pixhawk-ip", default="192.168.144.14")
    parser.add_argument("--listen-port", type=int, default=9002)
    parser.add_argument("--duration-s", type=float, default=30.0)
    parser.add_argument("--rate-hz", type=float, default=50.0)
    parser.add_argument("--sessions-dir", default=str(DEFAULT_SESSIONS_DIR))
    parser.add_argument(
        "--execute", action="store_true",
        help="Actually run against hardware (also requires --confirm-static-only). Omit for a dry run.",
    )
    parser.add_argument(
        "--confirm-static-only", action="store_true",
        help="Explicit confirmation that only the static SIM_JSON test is being run (also requires --execute).",
    )
    parser.add_argument(
        "--profile", choices=("static", "dynamic"), default="static",
        help=(
            "static (default, unchanged): sr75_hil_f24d_static_state_feed.py's "
            "fixed row. dynamic (HIL-F24-R1): live SR-75 6-DOF JSBSim state via "
            "sr75_hil_f24r1_dynamic_jsbsim_feed.py, ground-level/motionless, "
            "engines off, RATO/throttle/controls pinned at zero for the whole "
            "run. Requires --confirm-dynamic-jsbsim (in place of "
            "--confirm-static-only) to actually run."
        ),
    )
    parser.add_argument(
        "--confirm-dynamic-jsbsim", action="store_true",
        help=(
            "Explicit confirmation that the live-JSBSim dynamic profile (HIL-F24-R1) "
            "is being run (also requires --execute and --profile dynamic). Does not "
            "substitute for --confirm-static-only, and vice versa."
        ),
    )
    parser.add_argument(
        "--stage", choices=("none", "r2", "r2c"), default="none",
        help=(
            "none (default, unchanged): just the selected --profile. "
            "r2 (HIL-F24-R2): also runs a read-only Pixhawk-estimator-vs-"
            "JSBSim-truth capture/comparison over the dynamic run. Requires "
            "--profile dynamic and --confirm-estimator-comparison (in place "
            "of --confirm-dynamic-jsbsim) to actually run. "
            "r2c (HIL-F24-R2C): everything --stage r2 does, preceded by a "
            "guarded manual EKF-origin relatch workflow (start feeder+"
            "responder, prompt the operator to power-cycle the Pixhawk, "
            "wait for reattach, restart PPP, wait for live SIM_JSON "
            "replies, then a read-only origin diagnostic that ABORTS the "
            "run unless GPS_GLOBAL_ORIGIN is within "
            f"{ORIGIN_RELATCH_MAX_MISMATCH_M}m of truth). Requires "
            "--profile dynamic and --confirm-origin-relatch."
        ),
    )
    parser.add_argument(
        "--confirm-estimator-comparison", action="store_true",
        help=(
            "Explicit confirmation that the read-only estimator-vs-truth comparison "
            "(HIL-F24-R2, --stage r2) is being run (also requires --execute, "
            "--profile dynamic, and --stage r2). Does not substitute for "
            "--confirm-dynamic-jsbsim or --confirm-static-only, and vice versa."
        ),
    )
    parser.add_argument(
        "--r2-alignment-tolerance-s", type=float, default=0.1,
        help="HIL-F24-R2: max host-monotonic alignment tolerance (see the comparison script)",
    )
    parser.add_argument(
        "--r2-settle-s", type=float, default=5.0,
        help="HIL-F24-R2: initial settling period excluded from steady-state scoring",
    )
    parser.add_argument(
        "--confirm-origin-relatch", action="store_true",
        help=(
            "Explicit confirmation that the guarded EKF-origin relatch workflow "
            "(HIL-F24-R2C, --stage r2c), including a manual Pixhawk power-cycle "
            "prompt, is being run (also requires --execute, --profile dynamic, "
            "and --stage r2c). Does not substitute for any other confirm flag."
        ),
    )
    parser.add_argument(
        "--relatch-listen-s", type=float, default=6.0,
        help="HIL-F24-R2C: origin diagnostic's STATUSTEXT/HOME/origin listen window, in seconds",
    )
    parser.add_argument(
        "--relatch-truth-lat", type=float, default=32.5378085,
        help="HIL-F24-R2C: truth latitude the relatched GPS_GLOBAL_ORIGIN is compared against",
    )
    parser.add_argument(
        "--relatch-truth-lon", type=float, default=74.3661944,
        help="HIL-F24-R2C: truth longitude the relatched GPS_GLOBAL_ORIGIN is compared against",
    )
    parser.add_argument(
        "--relatch-truth-alt-m", type=float, default=240.201118,
        help="HIL-F24-R2C: truth altitude (m) the relatched GPS_GLOBAL_ORIGIN is compared against",
    )
    parser.add_argument(
        "--relatch-device-timeout-s", type=float, default=120.0,
        help="HIL-F24-R2C: max wait for --pixhawk/--ppp-device to reappear after the manual power cycle",
    )
    parser.add_argument(
        "--relatch-sim-json-timeout-s", type=float, default=60.0,
        help="HIL-F24-R2C: max wait for live post-reboot SIM_JSON replies once PPP is back up",
    )
    parser.add_argument(
        "--relatch-min-replies", type=int, default=5,
        help="HIL-F24-R2C: minimum new responder replies required to consider SIM_JSON traffic live",
    )
    parser.add_argument(
        "--relatch-operator-wait-budget-s", type=float, default=DEFAULT_RELATCH_OPERATOR_WAIT_BUDGET_S,
        help=(
            "HIL-F24-R2F: budgeted time for the manual Pixhawk power-cycle prompt, counted into the "
            "--stage r2c feeder's own lifetime so JSBSim truth stays fresh through the whole relatch "
            "workflow, not just the R2 capture window."
        ),
    )
    parser.add_argument(
        "--relatch-ppp-retry-attempts", type=int, default=DEFAULT_RELATCH_PPP_RETRY_ATTEMPTS,
        help="HIL-F24-R2G: R2C-only post-reboot PPP reconnect attempts after the power cycle",
    )
    parser.add_argument(
        "--relatch-ppp-retry-delay-s", type=float, default=DEFAULT_RELATCH_PPP_RETRY_DELAY_S,
        help="HIL-F24-R2G: R2C-only delay between failed post-reboot PPP reconnect attempts",
    )
    parser.add_argument(
        "--relatch-ppp-health-timeout-s", type=float, default=DEFAULT_RELATCH_PPP_HEALTH_TIMEOUT_S,
        help="HIL-F24-R2H: R2C-only per-attempt PPP health-poll deadline after starting pppd",
    )
    parser.add_argument(
        "--relatch-ppp-health-poll-s", type=float, default=DEFAULT_RELATCH_PPP_HEALTH_POLL_S,
        help="HIL-F24-R2H: R2C-only interval between PPP health polls within one reconnect attempt",
    )
    return parser


def main():
    args = build_arg_parser().parse_args()
    print(f"=== HIL-F24-F hardware orchestrator (--profile {args.profile} --stage {args.stage}) ===")
    print("Never arms. Never sends actuator/servo output. Never runs AUTO/mission.")
    if args.profile == "dynamic":
        print(
            "HIL-F24-R1 dynamic profile: live JSBSim state, ground-level/motionless, "
            "engines off, RATO/throttle/controls pinned at zero. Responder remains "
            "LOG_ONLY -- no actuator command is ever applied to hardware or JSBSim."
        )
    if args.stage in ("r2", "r2c"):
        print(
            "HIL-F24-R2 stage: read-only Pixhawk-estimator-vs-JSBSim-truth capture/"
            "comparison layered on top of the dynamic run. No PARAM_SET, arm/disarm, "
            "mode change, mission item, RC override, or actuator command is ever sent."
        )
        if args.stage == "r2c":
            print(
                "HIL-F24-R2C: guarded EKF-origin relatch precedes the capture -- this "
                "will start the feeder/responder, then PROMPT for a manual Pixhawk "
                "power-cycle, then abort (never proceeding to capture) unless a "
                "read-only diagnostic confirms GPS_GLOBAL_ORIGIN relatched within "
                f"{ORIGIN_RELATCH_MAX_MISMATCH_M}m of truth."
            )
        if args.profile != "dynamic":
            print(f"ABORT: --stage {args.stage} requires --profile dynamic -- staying in dry-run")
            print_plan(build_execution_plan(args), args)
            return 0

    plan = build_execution_plan(args)
    proceed, reason = should_proceed_to_hardware(
        args.execute, args.confirm_static_only,
        confirm_dynamic_jsbsim=args.confirm_dynamic_jsbsim, profile=args.profile,
        confirm_estimator_comparison=args.confirm_estimator_comparison, stage=args.stage,
        confirm_origin_relatch=args.confirm_origin_relatch,
    )

    if not proceed:
        print(reason)
        print_plan(plan, args)
        return 0

    adapters = check_adapter_presence([args.pixhawk, args.ppp_device])
    for path, present in adapters.items():
        print(f"Adapter check: {path} {'FOUND' if present else 'MISSING'}")
    if not all(adapters.values()):
        print("ABORT: required serial device missing -- see HIL-F24-F adapter-replacement checklist")
        return 2

    session_dir = build_session_dir(Path(args.sessions_dir), profile=args.profile, stage=args.stage)
    session_dir.mkdir(parents=True, exist_ok=True)
    print(f"Session artifact directory: {session_dir}")

    ppp_started = False
    feeder_proc = None
    responder_proc = None
    capture_proc = None
    exit_code = 0
    try:
        plan_by_name = {step.name: step for step in plan}

        print("Running read-only SoH precheck...")
        rc = run_step(plan_by_name["precheck"], session_dir, session_dir / "precheck.log", timeout_s=60.0)
        if rc != 0:
            raise OrchestratorAbort(f"SoH precheck did not return GO (exit code {rc}); see precheck.log")

        feeder_cmd = [
            part.format(state_csv=str(session_dir / "state.csv"))
            for part in plan_by_name["start_feeder"].command
        ]
        responder_cmd = [
            part.format(state_csv=str(session_dir / "state.csv"), responder_csv=str(session_dir / "responder.csv"))
            for part in plan_by_name["start_responder"].command
        ]

        if args.stage == "r2c":
            # HIL-F24-R2C: feeder + responder start BEFORE PPP/the relatch
            # prompt at all -- see build_execution_plan()'s own comment.
            #
            # HIL-F24-R2E task 5: remove any stale raw JSBSim output left
            # over from an earlier invocation before starting this run's
            # feeder. The feeder itself already does this internally
            # (sr75_hil_f24r1_dynamic_jsbsim_feed.py's start_jsbsim()) --
            # this is a defensive, harmless duplicate at the orchestrator
            # level specific to r2c, whose longer, multi-phase, operator-
            # involving run makes stale evidence hygiene more visible/
            # important than for R1/R2's single-shot runs (left untouched
            # -- see build_execution_plan()'s r2c-only branch above).
            raw_jsbsim_path = Path(DYNAMIC_FEEDER_RAW_OUTPUT)
            if cleanup_stale_raw_jsbsim_evidence(raw_jsbsim_path):
                print(f"Removed stale raw JSBSim evidence: {raw_jsbsim_path}")

            print("Starting feeder (HIL-F24-R2C: before the relatch step)...")
            feeder_proc = subprocess.Popen(
                feeder_cmd, stdout=open(session_dir / "feeder.log", "w"), stderr=subprocess.STDOUT,
            )
            print(f"Starting responder (HIL-F24-R2C: before the relatch step, bound to {R2C_RESPONDER_LISTEN_HOST})...")
            responder_proc = subprocess.Popen(
                responder_cmd, stdout=open(session_dir / "responder.log", "w"), stderr=subprocess.STDOUT,
            )

            # HIL-F24-R2E task 3/4: confirm the responder actually started
            # (e.g. did not immediately exit on a bind() failure) *before*
            # committing to the slow, operator-blocking power-cycle
            # prompt below -- a dead responder can never answer anything
            # no matter how long the rest of this workflow waits.
            if not wait_for_process_startup_health(responder_proc):
                responder_log_tail = (session_dir / "responder.log").read_text(errors="replace")
                raise OrchestratorAbort(
                    f"responder exited immediately after starting (exit code {responder_proc.returncode}); "
                    f"see responder.log. Last output:\n{responder_log_tail}"
                )
            print("Responder is alive.")

            prompt_operator_power_cycle()

            print(f"Waiting up to {args.relatch_device_timeout_s}s for {args.pixhawk} and {args.ppp_device} to reappear...")
            reattached = wait_for_devices([args.pixhawk, args.ppp_device], timeout_s=args.relatch_device_timeout_s)
            if not reattached:
                raise OrchestratorAbort(
                    f"{args.pixhawk}/{args.ppp_device} did not reappear within "
                    f"{args.relatch_device_timeout_s}s of the power-cycle prompt"
                )
            print("Devices reattached.")

            ppp_started = True
            print("(Re)starting PPP now that the Pixhawk has rebooted...")
            ppp_ok, ppp_reason = restart_r2c_ppp_with_retries(args, plan_by_name, session_dir)
            print(f"R2C PPP reconnect: {ppp_reason}")
            if not ppp_ok:
                raise OrchestratorAbort(ppp_reason)

            print(f"Waiting up to {args.relatch_sim_json_timeout_s}s for live post-reboot SIM_JSON replies...")
            replies_seen, replies_reason = wait_for_r2c_live_sim_json_replies(
                session_dir / "responder.csv", session_dir / "state.csv", feeder_proc,
                min_new_replies=args.relatch_min_replies,
                timeout_s=args.relatch_sim_json_timeout_s,
            )
            if not replies_seen:
                if "stale" in replies_reason.lower() or "state.csv" in replies_reason:
                    raise OrchestratorAbort(f"Live SIM_JSON truth failed -- {replies_reason}")
                if feeder_proc.poll() is not None:
                    raise OrchestratorAbort(
                        f"feeder process exited (exit code {feeder_proc.returncode}) while waiting for "
                        f"SIM_JSON replies; see feeder.log"
                    )
                if responder_proc.poll() is not None:
                    raise OrchestratorAbort(
                        f"responder process exited (exit code {responder_proc.returncode}) while waiting for "
                        f"SIM_JSON replies; see responder.log"
                    )
                raise OrchestratorAbort(
                    f"Live SIM_JSON reply gate failed -- {replies_reason}; see responder.csv/responder.log"
                )
            print(f"Live SIM_JSON replies confirmed: {replies_reason}")

            # HIL-F24-R2F task 4/5: re-confirm the feeder is alive, state.csv
            # is fresh, and enough replies have been recorded -- right
            # before committing to the origin diagnostic -- and abort
            # immediately, with the exact reason, if not. Session
            # sr75_hil_f24r2c_relatch_20260806T065842Z (stale_state_
            # count=305, replies_sent=1) is exactly the failure this
            # catches early and explains clearly instead of silently
            # feeding a stale-truth run into the origin diagnostic/capture.
            readiness_ok, readiness_reason = check_r2c_origin_check_readiness(
                feeder_proc, session_dir / "state.csv", session_dir / "responder.csv",
                min_replies=args.relatch_min_replies,
            )
            print(f"Origin-check readiness: {readiness_reason}")
            if not readiness_ok:
                raise OrchestratorAbort(f"Not ready for the origin check -- {readiness_reason}")

            print("Running read-only EKF-origin relatch diagnostic (HIL-F24-R2C gate)...")
            origin_diagnostic_cmd = [
                part.format(**session_artifact_paths(session_dir))
                for part in plan_by_name["run_origin_relatch_check"].command
            ]
            with open(session_dir / "origin_relatch.log", "w") as log_file:
                subprocess.run(origin_diagnostic_cmd, stdout=log_file, stderr=subprocess.STDOUT, timeout=R2C_ORIGIN_CHECK_TIMEOUT_S)
            summary_path = session_dir / "origin_relatch_summary.json"
            if not summary_path.exists():
                raise OrchestratorAbort(
                    "Origin relatch diagnostic did not produce a summary; see origin_relatch.log"
                )
            origin_summary = json.loads(summary_path.read_text())
            origin_ok, origin_reason = evaluate_origin_relatch_summary(origin_summary, ORIGIN_RELATCH_MAX_MISMATCH_M)
            print(f"Origin relatch check: {origin_reason}")
            if not origin_ok:
                raise OrchestratorAbort(
                    f"EKF-origin relatch gate failed -- {origin_reason}; "
                    f"see origin_relatch.log and origin_relatch_summary.json. Not proceeding to capture."
                )
            print("EKF-origin relatch confirmed -- proceeding to HIL-F24-R2 capture.")
        else:
            print("Starting PPP...")
            rc = run_step(plan_by_name["ppp_start"], session_dir, session_dir / "ppp_start.log", timeout_s=30.0)
            if rc != 0:
                raise OrchestratorAbort(f"PPP failed to start (exit code {rc}); see ppp_start.log")
            ppp_started = True

            time.sleep(2.0)  # let pppd negotiate before checking link state
            print("Verifying PPP link...")
            rc = run_step(plan_by_name["ppp_verify"], session_dir, session_dir / "ppp_verify.log", timeout_s=30.0)
            verify_log = (session_dir / "ppp_verify.log").read_text()
            if rc != 0 or "0 received" in verify_log or "100% packet loss" in verify_log:
                raise OrchestratorAbort("PPP link verification failed (ppp0 not up or ping failed); see ppp_verify.log")

            print("Starting feeder (before responder)...")
            feeder_proc = subprocess.Popen(
                feeder_cmd, stdout=open(session_dir / "feeder.log", "w"), stderr=subprocess.STDOUT,
            )

            print("Starting responder (LOG_ONLY, no actuator output)...")
            responder_proc = subprocess.Popen(
                responder_cmd, stdout=open(session_dir / "responder.log", "w"), stderr=subprocess.STDOUT,
            )

        if args.stage in ("r2", "r2c"):
            print("Starting read-only estimator capture (HIL-F24-R2, after responder)...")
            capture_cmd = [
                part.format(**session_artifact_paths(session_dir))
                for part in plan_by_name["start_estimator_capture"].command
            ]
            capture_proc = subprocess.Popen(
                capture_cmd, stdout=open(session_dir / "capture.log", "w"), stderr=subprocess.STDOUT,
            )

        print(f"SIM_JSON test (--profile {args.profile}) running for {args.duration_s}s (feeder+responder); artifacts in {session_dir}")
        time.sleep(args.duration_s + ORCHESTRATOR_TEST_MARGIN_S)
        print("SIM_JSON test window complete.")

    except OrchestratorAbort as exc:
        print(f"ABORT: {exc}")
        exit_code = 3
    finally:
        # HIL-F24-Q: order matters -- responder must be stopped before the
        # feeder, so no responder request is ever received after the
        # feeder has terminated. This is only meaningful because the
        # feeder is now given FEEDER_SHUTDOWN_MARGIN_S extra --duration-s
        # (see build_execution_plan()) so it is still running (and still
        # writing) when execution reaches this point, rather than having
        # already exited on its own a couple of seconds earlier.
        # HIL-F24-R2: capture is stopped LAST, after both responder and
        # feeder, so it stays alive (and keeps recording Pixhawk estimator
        # output plus JSBSim truth) for the entire window either of them
        # could have produced data -- never stopped early relative to the
        # run it is scoring.
        for proc, label in ((responder_proc, "responder"), (feeder_proc, "feeder"), (capture_proc, "estimator capture")):
            if proc is not None and proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    proc.kill()
                print(f"Stopped {label}")
        if args.stage in ("r2", "r2c") and capture_proc is not None:
            print("Scoring estimator output against JSBSim truth (HIL-F24-R2)...")
            comparison_rc = run_step(
                plan_by_name["run_estimator_comparison"], session_dir,
                session_dir / "comparison.log", timeout_s=60.0,
            )
            if comparison_rc == 0:
                print(f"Estimator comparison PASSED all thresholds -- see {session_dir / 'estimator_summary.md'}")
            else:
                print(
                    f"Estimator comparison FAILED (or errored) -- exit code {comparison_rc}; "
                    f"see {session_dir / 'comparison.log'} and {session_dir / 'estimator_summary.md'}"
                )
                exit_code = exit_code or 4
        if args.profile == "dynamic":
            # HIL-F24-R1 task 6: record the raw JSBSim state alongside the
            # responder's own log, and re-run the read-only safety
            # precheck (mode/armed/CH7/CH8) after the test as well as
            # before, so the session directory holds before-and-after
            # safety evidence, not just a single pre-test snapshot.
            record_dynamic_profile_evidence(args, session_dir)
        if ppp_started:
            print("Stopping PPP (always run on exit)...")
            run_step(plan_by_name["ppp_stop"], session_dir, session_dir / "ppp_stop.log", timeout_s=30.0)

    print(f"All artifacts recorded under: {session_dir}")
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
