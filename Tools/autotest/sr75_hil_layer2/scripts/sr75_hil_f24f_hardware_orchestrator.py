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
import csv
import json
import math
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
VISUAL_FEEDER_JSBSIM_SCRIPT = "aircraft/sr_75_6_dof/scripts/SR75_hil_f24r3a_mp_visual_trajectory.xml"
VISUAL_FEEDER_RAW_OUTPUT = "/tmp/sr75_hil_f24r3a_mp_visual_raw.csv"
DEFAULT_VISUALIZATION_DURATION_S = 60.0
R3A_EKF_READY_TIMEOUT_S = 60.0
R3A_EKF_READY_CONSECUTIVE_REPORTS = 3
R3A_PREMOTION_DIAGNOSTIC_MESSAGES_HZ = {
    "GPS_RAW_INT": 5.0,
    "GPS2_RAW": 5.0,
    "EKF_STATUS_REPORT": 2.0,
    "ATTITUDE": 10.0,
    "AHRS2": 5.0,
    "AHRS3": 5.0,
    "RAW_IMU": 5.0,
    "SCALED_IMU": 5.0,
    "SCALED_IMU2": 5.0,
    "SCALED_IMU3": 5.0,
}
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
EKF_ATTITUDE = 1
EKF_POS_HORIZ_REL = 8
EKF_POS_HORIZ_ABS = 16
EKF_UNINITIALIZED = 1024

PPP_START_SCRIPT = PPP_DIR / "sr75_ppp_start.sh"
PPP_STOP_SCRIPT = PPP_DIR / "sr75_ppp_stop.sh"
PPP_STATUS_SCRIPT = PPP_DIR / "sr75_ppp_status.sh"


class OrchestratorAbort(Exception):
    """Raised to unwind the plan early (adapter missing, precheck STOP,
    PPP failed) -- always caught by main() so cleanup still runs."""


SUDO_CREDENTIAL_ABORT = "Run sudo -v in this terminal, then rerun."


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


def ekf_flags_ready(flags: int) -> tuple:
    if flags & EKF_UNINITIALIZED:
        return False, f"EKF_UNINITIALIZED set flags={flags}"
    if not (flags & EKF_ATTITUDE):
        return False, f"EKF attitude flag missing flags={flags}"
    if not (flags & (EKF_POS_HORIZ_REL | EKF_POS_HORIZ_ABS)):
        return False, f"EKF horizontal position flag missing flags={flags}"
    return True, f"EKF ready flags={flags}"


R3A_PREMOTION_DIAGNOSTIC_CSV_FIELDS = [
    "host_monotonic_time",
    "msg_type",
    "fix_type",
    "satellites_visible",
    "eph",
    "epv",
    "vel",
    "cog",
    "lat_deg",
    "lon_deg",
    "alt_m",
    "ekf_flags",
    "ekf_velocity_variance",
    "ekf_pos_horiz_variance",
    "ekf_pos_vert_variance",
    "ekf_compass_variance",
    "att_roll_deg",
    "att_pitch_deg",
    "att_yaw_deg",
    "att_p_deg_s",
    "att_q_deg_s",
    "att_r_deg_s",
    "ahrs2_roll_deg",
    "ahrs2_pitch_deg",
    "ahrs2_yaw_deg",
    "ahrs2_alt_m",
    "ahrs2_lat_deg",
    "ahrs2_lon_deg",
    "ahrs3_roll_deg",
    "ahrs3_pitch_deg",
    "ahrs3_yaw_deg",
    "ahrs3_alt_m",
    "ahrs3_lat_deg",
    "ahrs3_lon_deg",
    "imu_xacc",
    "imu_yacc",
    "imu_zacc",
    "imu_xgyro",
    "imu_ygyro",
    "imu_zgyro",
    "imu_xmag",
    "imu_ymag",
    "imu_zmag",
]

R3A_PREMOTION_DIAGNOSTIC_MESSAGE_TYPES = [
    "GPS_RAW_INT",
    "GPS2_RAW",
    "EKF_STATUS_REPORT",
    "ATTITUDE",
    "AHRS2",
    "AHRS3",
    "RAW_IMU",
    "SCALED_IMU",
    "SCALED_IMU2",
    "SCALED_IMU3",
]


def r3a_premotion_message_to_row(msg, host_monotonic_time: float) -> dict:
    msg_type = msg.get_type()
    row = {field: "" for field in R3A_PREMOTION_DIAGNOSTIC_CSV_FIELDS}
    row["host_monotonic_time"] = f"{host_monotonic_time:.9f}"
    row["msg_type"] = msg_type
    if msg_type in ("GPS_RAW_INT", "GPS2_RAW"):
        row["fix_type"] = getattr(msg, "fix_type", "")
        row["satellites_visible"] = getattr(msg, "satellites_visible", "")
        row["eph"] = getattr(msg, "eph", "")
        row["epv"] = getattr(msg, "epv", "")
        row["vel"] = getattr(msg, "vel", "")
        row["cog"] = getattr(msg, "cog", "")
        if hasattr(msg, "lat"):
            row["lat_deg"] = getattr(msg, "lat") * 1.0e-7
        if hasattr(msg, "lon"):
            row["lon_deg"] = getattr(msg, "lon") * 1.0e-7
        if hasattr(msg, "alt"):
            row["alt_m"] = getattr(msg, "alt") * 1.0e-3
    elif msg_type == "EKF_STATUS_REPORT":
        row["ekf_flags"] = getattr(msg, "flags", "")
        row["ekf_velocity_variance"] = getattr(msg, "velocity_variance", "")
        row["ekf_pos_horiz_variance"] = getattr(msg, "pos_horiz_variance", "")
        row["ekf_pos_vert_variance"] = getattr(msg, "pos_vert_variance", "")
        row["ekf_compass_variance"] = getattr(msg, "compass_variance", "")
    elif msg_type == "ATTITUDE":
        row["att_roll_deg"] = math.degrees(getattr(msg, "roll", 0.0))
        row["att_pitch_deg"] = math.degrees(getattr(msg, "pitch", 0.0))
        row["att_yaw_deg"] = math.degrees(getattr(msg, "yaw", 0.0))
        row["att_p_deg_s"] = math.degrees(getattr(msg, "rollspeed", 0.0))
        row["att_q_deg_s"] = math.degrees(getattr(msg, "pitchspeed", 0.0))
        row["att_r_deg_s"] = math.degrees(getattr(msg, "yawspeed", 0.0))
    elif msg_type in ("AHRS2", "AHRS3"):
        prefix = msg_type.lower()
        row[f"{prefix}_roll_deg"] = math.degrees(getattr(msg, "roll", 0.0))
        row[f"{prefix}_pitch_deg"] = math.degrees(getattr(msg, "pitch", 0.0))
        row[f"{prefix}_yaw_deg"] = math.degrees(getattr(msg, "yaw", 0.0))
        row[f"{prefix}_alt_m"] = getattr(msg, "altitude", "")
        if hasattr(msg, "lat"):
            row[f"{prefix}_lat_deg"] = getattr(msg, "lat") * 1.0e-7
        if hasattr(msg, "lng"):
            row[f"{prefix}_lon_deg"] = getattr(msg, "lng") * 1.0e-7
    elif msg_type in ("RAW_IMU", "SCALED_IMU", "SCALED_IMU2", "SCALED_IMU3"):
        row["imu_xacc"] = getattr(msg, "xacc", "")
        row["imu_yacc"] = getattr(msg, "yacc", "")
        row["imu_zacc"] = getattr(msg, "zacc", "")
        row["imu_xgyro"] = getattr(msg, "xgyro", "")
        row["imu_ygyro"] = getattr(msg, "ygyro", "")
        row["imu_zgyro"] = getattr(msg, "zgyro", "")
        row["imu_xmag"] = getattr(msg, "xmag", "")
        row["imu_ymag"] = getattr(msg, "ymag", "")
        row["imu_zmag"] = getattr(msg, "zmag", "")
    return row


def update_r3a_premotion_summary(summary: dict, msg, host_monotonic_time: float) -> None:
    msg_type = msg.get_type()
    counts = summary.setdefault("message_counts", {})
    counts[msg_type] = counts.get(msg_type, 0) + 1
    if msg_type in ("GPS_RAW_INT", "GPS2_RAW"):
        fix_type = int(getattr(msg, "fix_type", 0))
        satellites = int(getattr(msg, "satellites_visible", 0))
        gps_summary = summary["gps"].setdefault(msg_type, {
            "message_count": 0,
            "first_fix_type_ge_3_time": None,
            "first_satellite_count_time": None,
            "first_satellite_count": None,
            "last_fix_type": None,
            "last_satellites_visible": None,
            "last_eph": None,
            "last_epv": None,
            "last_vel": None,
            "last_cog": None,
            "last_lat_deg": None,
            "last_lon_deg": None,
            "last_alt_m": None,
        })
        gps_summary["message_count"] += 1
        gps_summary["last_fix_type"] = fix_type
        gps_summary["last_satellites_visible"] = satellites
        gps_summary["last_eph"] = getattr(msg, "eph", None)
        gps_summary["last_epv"] = getattr(msg, "epv", None)
        gps_summary["last_vel"] = getattr(msg, "vel", None)
        gps_summary["last_cog"] = getattr(msg, "cog", None)
        if hasattr(msg, "lat"):
            gps_summary["last_lat_deg"] = getattr(msg, "lat") * 1.0e-7
        if hasattr(msg, "lon"):
            gps_summary["last_lon_deg"] = getattr(msg, "lon") * 1.0e-7
        if hasattr(msg, "alt"):
            gps_summary["last_alt_m"] = getattr(msg, "alt") * 1.0e-3
        if fix_type >= 3 and gps_summary["first_fix_type_ge_3_time"] is None:
            gps_summary["first_fix_type_ge_3_time"] = host_monotonic_time
        if satellites > 0 and gps_summary["first_satellite_count_time"] is None:
            gps_summary["first_satellite_count_time"] = host_monotonic_time
            gps_summary["first_satellite_count"] = satellites
    elif msg_type == "EKF_STATUS_REPORT":
        flags = int(getattr(msg, "flags", 0))
        ekf_summary = summary["ekf"]
        ekf_summary["message_count"] += 1
        ekf_summary["last_flags"] = flags
        ekf_summary["last_velocity_variance"] = getattr(msg, "velocity_variance", None)
        ekf_summary["last_pos_horiz_variance"] = getattr(msg, "pos_horiz_variance", None)
        ekf_summary["last_pos_vert_variance"] = getattr(msg, "pos_vert_variance", None)
        ekf_summary["last_compass_variance"] = getattr(msg, "compass_variance", None)
        if not (flags & EKF_UNINITIALIZED) and ekf_summary["first_uninitialized_cleared_time"] is None:
            ekf_summary["first_uninitialized_cleared_time"] = host_monotonic_time
        if (flags & EKF_ATTITUDE) and ekf_summary["first_attitude_flag_time"] is None:
            ekf_summary["first_attitude_flag_time"] = host_monotonic_time
        if ((flags & (EKF_POS_HORIZ_REL | EKF_POS_HORIZ_ABS)) and
                ekf_summary["first_horizontal_position_flag_time"] is None):
            ekf_summary["first_horizontal_position_flag_time"] = host_monotonic_time
    elif msg_type == "ATTITUDE":
        summary["last_attitude"] = {
            "time": host_monotonic_time,
            "roll_deg": math.degrees(getattr(msg, "roll", 0.0)),
            "pitch_deg": math.degrees(getattr(msg, "pitch", 0.0)),
            "yaw_deg": math.degrees(getattr(msg, "yaw", 0.0)),
        }
    elif msg_type in ("AHRS2", "AHRS3"):
        summary.setdefault("last_ahrs", {})[msg_type] = {
            "time": host_monotonic_time,
            "roll_deg": math.degrees(getattr(msg, "roll", 0.0)),
            "pitch_deg": math.degrees(getattr(msg, "pitch", 0.0)),
            "yaw_deg": math.degrees(getattr(msg, "yaw", 0.0)),
        }
    elif msg_type in ("RAW_IMU", "SCALED_IMU", "SCALED_IMU2", "SCALED_IMU3"):
        summary.setdefault("last_imu", {})[msg_type] = {
            "time": host_monotonic_time,
            "xacc": getattr(msg, "xacc", None),
            "yacc": getattr(msg, "yacc", None),
            "zacc": getattr(msg, "zacc", None),
            "xgyro": getattr(msg, "xgyro", None),
            "ygyro": getattr(msg, "ygyro", None),
            "zgyro": getattr(msg, "zgyro", None),
            "xmag": getattr(msg, "xmag", None),
            "ymag": getattr(msg, "ymag", None),
            "zmag": getattr(msg, "zmag", None),
        }


def r3a_premotion_timeout_reason(
    summary: dict, last_ekf_reason: str, timeout_s: float, consecutive: int, required: int,
) -> str:
    gps_reasons = []
    for msg_type in ("GPS_RAW_INT", "GPS2_RAW"):
        gps_summary = summary.get("gps", {}).get(msg_type)
        if gps_summary is None:
            gps_reasons.append(f"{msg_type} not received")
        elif gps_summary.get("first_fix_type_ge_3_time") is None:
            gps_reasons.append(
                f"{msg_type} never reached fix_type>=3 "
                f"(last_fix_type={gps_summary.get('last_fix_type')}, "
                f"last_satellites={gps_summary.get('last_satellites_visible')})"
            )
    gps_clause = ""
    if gps_reasons:
        gps_clause = "; GPS evidence: " + "; ".join(gps_reasons)
    return (
        f"EKF readiness timed out after {timeout_s:.1f}s: {last_ekf_reason}; "
        f"consecutive_healthy={consecutive}/{required}{gps_clause}"
    )


def wait_for_consecutive_ekf_ready(
    recv_fn,
    timeout_s: float = R3A_EKF_READY_TIMEOUT_S,
    consecutive_required: int = R3A_EKF_READY_CONSECUTIVE_REPORTS,
    poll_s: float = 1.0,
    now_fn=time.monotonic,
) -> tuple:
    deadline = now_fn() + timeout_s
    consecutive = 0
    last_reason = "no EKF_STATUS_REPORT received"
    while now_fn() < deadline:
        msg = recv_fn(timeout_s=min(poll_s, max(0.0, deadline - now_fn())))
        if msg is None:
            continue
        flags = int(getattr(msg, "flags", 0))
        ok, last_reason = ekf_flags_ready(flags)
        if ok:
            consecutive += 1
            if consecutive >= consecutive_required:
                return True, f"{consecutive} consecutive healthy EKF reports; {last_reason}"
        else:
            consecutive = 0
    return False, (
        f"EKF readiness timed out after {timeout_s:.1f}s: {last_reason}; "
        f"consecutive_healthy={consecutive}/{consecutive_required}"
    )


def r3a_empty_premotion_summary(consecutive_required: int) -> dict:
    return {
        "ok": False,
        "reason": "no EKF_STATUS_REPORT received",
        "consecutive_healthy_required": consecutive_required,
        "consecutive_healthy_observed": 0,
        "message_counts": {},
        "gps": {},
        "ekf": {
            "message_count": 0,
            "first_uninitialized_cleared_time": None,
            "first_attitude_flag_time": None,
            "first_horizontal_position_flag_time": None,
            "last_flags": None,
            "last_velocity_variance": None,
            "last_pos_horiz_variance": None,
            "last_pos_vert_variance": None,
            "last_compass_variance": None,
        },
        "last_attitude": None,
        "last_ahrs": {},
        "last_imu": {},
    }


def refresh_r3a_premotion_reason(summary: dict, last_reason: str) -> str:
    ekf_summary = summary.get("ekf", {})
    if ekf_summary.get("message_count", 0) > 0:
        flags = ekf_summary.get("last_flags")
        if flags is not None:
            _ok, last_reason = ekf_flags_ready(int(flags))
        summary["reason"] = (
            f"EKF_STATUS_REPORT received but readiness not achieved: {last_reason}; "
            f"consecutive_healthy={summary.get('consecutive_healthy_observed', 0)}/"
            f"{summary.get('consecutive_healthy_required', R3A_EKF_READY_CONSECUTIVE_REPORTS)}"
        )
    return last_reason


class R3aPremotionDiagnosticCapture:
    def __init__(
        self,
        pixhawk: str,
        baud: int,
        csv_path: Path,
        summary_path: Path,
        consecutive_required: int = R3A_EKF_READY_CONSECUTIVE_REPORTS,
        rates_hz: dict = None,
        mavlink_connection_factory=None,
        now_fn=time.monotonic,
    ):
        self.pixhawk = pixhawk
        self.baud = baud
        self.csv_path = Path(csv_path)
        self.summary_path = Path(summary_path)
        self.consecutive_required = consecutive_required
        self.rates_hz = rates_hz or R3A_PREMOTION_DIAGNOSTIC_MESSAGES_HZ
        self.mavlink_connection_factory = mavlink_connection_factory
        self.now_fn = now_fn
        self.master = None
        self.csv_file = None
        self.writer = None
        self.summary = r3a_empty_premotion_summary(consecutive_required)
        self.consecutive = 0
        self.last_reason = "no EKF_STATUS_REPORT received"

    def start(self) -> None:
        from pymavlink import mavutil

        factory = self.mavlink_connection_factory or mavutil.mavlink_connection
        self.master = factory(self.pixhawk, baud=self.baud)
        self.master.wait_heartbeat(timeout=10)
        for name, hz in self.rates_hz.items():
            msg_id = getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{name}", None)
            if msg_id is None:
                continue
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                msg_id, int(1e6 / hz), 0, 0, 0, 0, 0,
            )
        self.csv_file = open(self.csv_path, "w", encoding="utf-8", newline="")
        self.writer = csv.DictWriter(self.csv_file, fieldnames=R3A_PREMOTION_DIAGNOSTIC_CSV_FIELDS)
        self.writer.writeheader()
        self.flush()

    def drain(self, duration_s: float = 0.0) -> None:
        if duration_s <= 0.0:
            while True:
                msg = self.master.recv_match(
                    type=R3A_PREMOTION_DIAGNOSTIC_MESSAGE_TYPES,
                    blocking=False,
                    timeout=0.0,
                )
                if msg is None:
                    return
                self._record(msg)
        deadline = self.now_fn() + duration_s
        while self.now_fn() <= deadline:
            timeout_s = min(0.2, max(0.0, deadline - self.now_fn()))
            msg = self.master.recv_match(
                type=R3A_PREMOTION_DIAGNOSTIC_MESSAGE_TYPES,
                blocking=True,
                timeout=timeout_s,
            )
            if msg is None:
                return
            self._record(msg)

    def wait_for_ekf_ready(self, timeout_s: float) -> tuple:
        deadline = self.now_fn() + timeout_s
        while self.now_fn() < deadline:
            msg = self.master.recv_match(
                type=R3A_PREMOTION_DIAGNOSTIC_MESSAGE_TYPES,
                blocking=True,
                timeout=min(1.0, max(0.0, deadline - self.now_fn())),
            )
            if msg is None:
                continue
            self._record(msg)
            if msg.get_type() != "EKF_STATUS_REPORT":
                continue
            flags = int(getattr(msg, "flags", 0))
            ok, self.last_reason = ekf_flags_ready(flags)
            if ok:
                self.consecutive += 1
                if self.consecutive >= self.consecutive_required:
                    self.summary["ok"] = True
                    self.summary["reason"] = f"{self.consecutive} consecutive healthy EKF reports; {self.last_reason}"
                    self.summary["consecutive_healthy_observed"] = self.consecutive
                    self.flush()
                    return True, self.summary["reason"]
            else:
                self.consecutive = 0
            self.summary["consecutive_healthy_observed"] = self.consecutive
        self.summary["reason"] = r3a_premotion_timeout_reason(
            self.summary, self.last_reason, timeout_s, self.consecutive, self.consecutive_required,
        )
        self.summary["consecutive_healthy_observed"] = self.consecutive
        self.flush()
        return False, self.summary["reason"]

    def _record(self, msg) -> None:
        host_time = self.now_fn()
        self.writer.writerow(r3a_premotion_message_to_row(msg, host_time))
        update_r3a_premotion_summary(self.summary, msg, host_time)
        self.last_reason = refresh_r3a_premotion_reason(self.summary, self.last_reason)
        self.flush()

    def flush(self) -> None:
        if self.csv_file is not None:
            self.csv_file.flush()
        with open(self.summary_path, "w", encoding="utf-8") as sf:
            json.dump(self.summary, sf, indent=2, sort_keys=True)
            sf.write("\n")

    def close(self) -> None:
        self.flush()
        if self.csv_file is not None:
            self.csv_file.close()
            self.csv_file = None
        if self.master is not None:
            close = getattr(self.master, "close", None)
            if close is not None:
                close()
            self.master = None


def wait_for_pixhawk_ekf_ready_with_premotion_diagnostics(
    pixhawk: str,
    baud: int,
    csv_path: Path,
    summary_path: Path,
    timeout_s: float = R3A_EKF_READY_TIMEOUT_S,
    consecutive_required: int = R3A_EKF_READY_CONSECUTIVE_REPORTS,
    rates_hz: dict = None,
    now_fn=time.monotonic,
) -> tuple:
    capture = R3aPremotionDiagnosticCapture(
        pixhawk, baud, csv_path, summary_path,
        consecutive_required=consecutive_required, rates_hz=rates_hz, now_fn=now_fn,
    )
    try:
        capture.start()
        return capture.wait_for_ekf_ready(timeout_s)
    finally:
        capture.close()


def wait_for_pixhawk_ekf_ready(
    pixhawk: str,
    baud: int,
    timeout_s: float = R3A_EKF_READY_TIMEOUT_S,
    consecutive_required: int = R3A_EKF_READY_CONSECUTIVE_REPORTS,
) -> tuple:
    from pymavlink import mavutil

    master = mavutil.mavlink_connection(pixhawk, baud=baud)
    master.wait_heartbeat(timeout=10)
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT,
        int(1e6 / 2.0), 0, 0, 0, 0, 0,
    )

    def recv_fn(timeout_s):
        return master.recv_match(type="EKF_STATUS_REPORT", blocking=True, timeout=timeout_s)

    return wait_for_consecutive_ekf_ready(
        recv_fn,
        timeout_s=timeout_s,
        consecutive_required=consecutive_required,
    )


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


def ppp_cleanup_steps(device: str) -> List[Step]:
    """Stale PPP cleanup before a bounded PPP readiness attempt."""
    return [
        Step("ppp_cleanup_stop", f"Stop stale pppd for {device}", ["sudo", "-n", str(PPP_STOP_SCRIPT), "--device", device]),
        Step(
            "ppp_cleanup_pid",
            "Remove stale SR-75 PPP pid file",
            ["sudo", "-n", "rm", "-f", str(ppp_pid_file_for_device(device))],
        ),
        Step("ppp_cleanup_ppp0", "Remove stale/down ppp0 if present", ["sudo", "-n", "ip", "link", "delete", "ppp0"]),
    ]


def r2c_ppp_cleanup_steps(device: str) -> List[Step]:
    """Compatibility wrapper for existing HIL-F24-R2C tests/callers."""
    return ppp_cleanup_steps(device)


def should_proceed_to_hardware(
    execute: bool,
    confirm_static_only: bool,
    confirm_dynamic_jsbsim: bool = False,
    profile: str = "static",
    confirm_estimator_comparison: bool = False,
    stage: str = "none",
    confirm_origin_relatch: bool = False,
    confirm_mp_visualization: bool = False,
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
    if stage == "r3a":
        confirm = confirm_mp_visualization
        confirm_flag_name = "--confirm-mp-visualization"
    elif stage == "r2c":
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
        "or --profile dynamic --stage r2c --execute --confirm-origin-relatch, "
        "or --profile visual --stage r3a --execute --confirm-mp-visualization) "
        "to run against real hardware"
    )


def build_session_dir(
    base_dir: Path, now: Optional[datetime] = None, profile: str = "static", stage: str = "none",
) -> Path:
    stamp = (now or datetime.now(timezone.utc)).strftime("%Y%m%dT%H%M%SZ")
    if stage == "r2c":
        name = "sr75_hil_f24r2c_relatch"
    elif stage == "r3a":
        name = "sr75_hil_f24r3a_mp_visual"
    elif stage == "r2":
        name = "sr75_hil_f24r2_estimator"
    elif profile == "visual":
        name = "sr75_hil_f24r3a_mp_visual"
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
        "sudo", "-n", str(PPP_START_SCRIPT),
        "--device", args.ppp_device, "--baud", str(args.ppp_baud),
        "--host-ip", args.host_ip, "--pixhawk-ip", args.pixhawk_ip, "--background",
    ]
    ppp_stop_cmd = ["sudo", "-n", str(PPP_STOP_SCRIPT), "--device", args.ppp_device]
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
    stage = getattr(args, "stage", "none")
    run_duration_s = args.visualization_duration_s if stage == "r3a" else args.duration_s
    feeder_duration_s = run_duration_s + ORCHESTRATOR_TEST_MARGIN_S + FEEDER_SHUTDOWN_MARGIN_S
    profile = getattr(args, "profile", "static")
    if profile == "visual":
        feeder_cmd = [
            sys.executable, str(DYNAMIC_FEEDER_SCRIPT), "--output", "{state_csv}",
            "--duration-s", str(feeder_duration_s),
            "--jsbsim-script", VISUAL_FEEDER_JSBSIM_SCRIPT,
            "--raw-output", VISUAL_FEEDER_RAW_OUTPUT,
            "--jsbsim-end", str(feeder_duration_s),
        ]
        feeder_description = "Start HIL-F24-R3A Mission Planner scripted-trajectory feeder (LOG_ONLY)"
    elif profile == "dynamic":
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
    if stage == "r3a":
        r3a_stationary_duration_s = r2c_feeder_duration_s(
            operator_wait_budget_s=args.relatch_operator_wait_budget_s,
            device_timeout_s=args.relatch_device_timeout_s,
            sim_json_timeout_s=args.relatch_sim_json_timeout_s,
            duration_s=args.r3a_ekf_readiness_timeout_s,
            ppp_retry_attempts=args.relatch_ppp_retry_attempts,
            ppp_retry_delay_s=args.relatch_ppp_retry_delay_s,
            ppp_health_timeout_s=args.relatch_ppp_health_timeout_s,
        )
        r3a_stationary_feeder_cmd = [
            sys.executable, str(FEEDER_SCRIPT), "--output", "{state_csv}",
            "--duration-s", str(r3a_stationary_duration_s),
            "--rate-hz", str(args.rate_hz),
            "--lat-deg", str(args.relatch_truth_lat),
            "--lon-deg", str(args.relatch_truth_lon),
            "--alt-m", str(args.relatch_truth_alt_m),
            "--yaw-deg", "315.0",
            "--airspeed-mps", "0.0",
        ]
        r3a_responder_cmd = [
            sys.executable, str(RESPONDER_SCRIPT),
            "--listen-host", R2C_RESPONDER_LISTEN_HOST, "--listen-port", str(args.listen_port),
            "--state-file", "{state_csv}", "--log-csv", "{responder_csv}", "--strict",
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
            Step("start_feeder", "Start stationary R3A truth feeder before Pixhawk reboot", r3a_stationary_feeder_cmd),
            Step(
                "start_responder",
                f"Start responder in LOG_ONLY mode, bound to {R2C_RESPONDER_LISTEN_HOST} (before PPP exists)",
                r3a_responder_cmd,
            ),
            Step("prompt_power_cycle", "Prompt operator to manually power-cycle the Pixhawk (blocks for ENTER)", None),
            Step("wait_for_reattach", f"Wait for {args.pixhawk} and {args.ppp_device} to reappear", None),
            Step("ppp_start", "(Re)start PPP link now that the Pixhawk has rebooted", ppp_start_cmd),
            Step("ppp_verify", "Verify ppp0 is up and Pixhawk PPP endpoint responds to ping", ppp_status_cmd),
            Step("wait_for_sim_json_replies", "Wait for the responder to answer real post-reboot SIM_JSON requests", None),
            Step(
                "run_origin_relatch_check",
                "Read-only EKF-origin diagnostic; ABORT unless GPS_GLOBAL_ORIGIN is within "
                f"{ORIGIN_RELATCH_MAX_MISMATCH_M} m of truth (HIL-F24-R3G gate)",
                origin_diagnostic_cmd,
            ),
            Step("wait_for_ekf_ready", "Wait for consecutive healthy EKF reports before visual motion", None),
        ]
    elif stage == "r2c":
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
    if stage in ("r2", "r2c", "r3a"):
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
        if getattr(args, "enable_mp_forward", False):
            capture_cmd.extend([
                "--enable-mp-forward",
                "--mp-udp-host", args.mp_udp_host,
                "--mp-udp-port", str(args.mp_udp_port),
            ])
        comparison_cmd = [
            sys.executable, str(R2_COMPARISON_SCRIPT),
            "--truth-csv", "{jsbsim_truth_csv}", "--pixhawk-csv", "{pixhawk_estimator_csv}",
            "--comparison-csv", "{estimator_comparison_csv}",
            "--summary-json", "{estimator_summary_json}", "--summary-md", "{estimator_summary_md}",
            "--alignment-tolerance-s", str(args.r2_alignment_tolerance_s),
            "--settle-s", str(0.0 if stage == "r3a" else args.r2_settle_s),
        ]
        if stage == "r3a":
            comparison_cmd.extend([
                "--health-ignore-before-s", "0.0",
                "--require-ekf-ready-after-s", "0.0",
            ])
            plan.append(Step(
                "start_motion_feeder",
                "Start 60 s smooth R3A visual trajectory after EKF readiness",
                feeder_cmd,
            ))
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
        "mp_checklist_md": f"{session_dir}/mission_planner_checklist.md",
    }


def print_plan(plan: List[Step], args) -> None:
    print("Execution plan (dry-run -- nothing below has been executed):")
    placeholders = session_artifact_paths("<session_dir>")
    for i, step in enumerate(plan, start=1):
        print(f"  {i}. [{step.name}] {step.description}")
        if step.command:
            rendered = " ".join(step.command).format(**placeholders)
            print(f"       $ {rendered}")


def check_sudo_noninteractive(run_fn=subprocess.run, timeout_s: float = 5.0) -> tuple:
    try:
        proc = run_fn(["sudo", "-n", "true"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=timeout_s)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return False, f"{SUDO_CREDENTIAL_ABORT} ({exc})"
    if proc.returncode != 0:
        return False, SUDO_CREDENTIAL_ABORT
    return True, "sudo credential available"


def run_step(step: Step, session_dir: Path, log_path: Path, timeout_s: Optional[float] = None) -> int:
    rendered = [part.format(**session_artifact_paths(session_dir)) for part in step.command]
    with open(log_path, "w") as log_file:
        try:
            proc = subprocess.run(rendered, stdout=log_file, stderr=subprocess.STDOUT, timeout=timeout_s)
        except subprocess.TimeoutExpired as exc:
            raise OrchestratorAbort(
                f"Command timed out after {timeout_s}s: {' '.join(rendered)}; log: {log_path}"
            ) from exc
    return proc.returncode


def _tail_text(path: Path, max_lines: int = 40) -> str:
    try:
        lines = path.read_text(errors="replace").splitlines()
    except OSError as exc:
        return f"<could not read {path}: {exc}>"
    return "\n".join(lines[-max_lines:])


def write_mp_visualization_checklist(session_dir: Path) -> Path:
    checklist = session_dir / "mission_planner_checklist.md"
    checklist.write_text(
        "# HIL-F24-R3A Mission Planner Checklist\n\n"
        "Safety state before and during run:\n"
        "- Mode: MANUAL\n"
        "- Armed: false\n"
        "- CH7: low/0\n"
        "- CH8: low/0\n"
        "- No AUTO, mission, RC override, actuator output, throttle, RATO or eject activity\n\n"
        "Expected visual ranges over 60 s:\n"
        "- Before visual motion: stationary truth feed at lat/lon/alt, level attitude, yaw 315 deg\n"
        "- Scored visual window starts only after origin and EKF-readiness gates pass\n"
        "- Latitude during moving segment: 32.5378085 to about 32.5394254 deg\n"
        "- Longitude during moving segment: 74.3661944 to about 74.3674623 deg\n"
        "- Altitude during moving segment: 240.2 to about 270.2 m\n"
        "- Roll during moving segment: within +/-4 deg\n"
        "- Pitch during moving segment: within +/-3 deg\n"
        "- Yaw during moving segment: about 315 to 330 deg\n"
        "- Ground displacement: about 216 m total\n",
        encoding="utf-8",
    )
    return checklist


def monitor_r3a_truth_feed(
    feeder_proc, state_csv_path: Path, feeder_log_path: Path, duration_s: float,
    max_state_age_s: float = R2C_READINESS_MAX_STATE_AGE_S,
    startup_timeout_s: float = 5.0,
    poll_interval_s: float = 0.2,
    reader_factory=None, sleep_fn=time.sleep, now_fn=time.monotonic,
) -> tuple:
    """R3A truth-feed liveness gate for the Mission Planner visual window."""
    reader_factory = reader_factory or responder.LatestCSVReader
    deadline = now_fn() + duration_s
    startup_deadline = now_fn() + startup_timeout_s
    saw_state = False
    last_fresh_reason = "no state.csv row observed yet"

    while now_fn() < deadline:
        if feeder_proc is not None and feeder_proc.poll() is not None:
            return False, (
                f"R3A truth feed failed: feeder exited early rc={feeder_proc.returncode}; "
                f"feeder.log tail:\n{_tail_text(feeder_log_path)}"
            )

        reader = reader_factory(state_csv_path)
        try:
            row, _line, age_s, _st = reader.read_latest()
        except responder.StateError as exc:
            if now_fn() >= startup_deadline:
                return False, f"R3A truth feed failed: state.csv not ready within {startup_timeout_s}s: {exc}"
        else:
            saw_state = True
            source_time = row.get("time_s", "unknown")
            if age_s > max_state_age_s:
                return False, (
                    f"R3A truth feed failed: state.csv stale age={age_s * 1000.0:.1f} ms "
                    f"exceeds {max_state_age_s * 1000.0:.0f} ms at source_time={source_time}; "
                    f"feeder_rc={feeder_proc.poll() if feeder_proc is not None else 'unknown'}; "
                    f"feeder.log tail:\n{_tail_text(feeder_log_path)}"
                )
            last_fresh_reason = f"state.csv fresh age={age_s * 1000.0:.1f} ms source_time={source_time}"

        sleep_fn(poll_interval_s)

    if not saw_state:
        return False, f"R3A truth feed failed: no state.csv row observed within {duration_s}s"
    return True, f"R3A truth feed remained live for {duration_s:.1f}s; last check: {last_fresh_reason}"


def should_run_estimator_comparison(stage: str, capture_proc, truth_feed_valid: bool) -> bool:
    return stage in ("r2", "r2c", "r3a") and capture_proc is not None and truth_feed_valid


def restart_ppp_with_retries(
    args, plan_by_name: dict, session_dir: Path, stage_label: str,
    failure_label: str = "PPP readiness",
    run_step_fn=run_step, sleep_fn=time.sleep, now_fn=time.monotonic, print_fn=print,
) -> tuple:
    """Shared bounded PPP cleanup/start/health-poll/retry readiness helper."""
    attempts = max(1, int(args.relatch_ppp_retry_attempts))
    last_reason = "no attempts made"
    last_start_log = None
    last_verify_log = None
    log_prefix = stage_label.lower()

    for attempt in range(1, attempts + 1):
        print_fn(f"{stage_label} PPP readiness attempt {attempt}/{attempts}: cleanup before start")
        for cleanup_step in ppp_cleanup_steps(args.ppp_device):
            cleanup_log = session_dir / f"ppp_{log_prefix}_attempt_{attempt}_{cleanup_step.name}.log"
            run_step_fn(cleanup_step, session_dir, cleanup_log, timeout_s=R2C_PPP_START_TIMEOUT_S)

        print_fn(f"{stage_label} PPP readiness attempt {attempt}/{attempts}: starting pppd")
        last_start_log = session_dir / f"ppp_{log_prefix}_attempt_{attempt}_start.log"
        start_rc = run_step_fn(plan_by_name["ppp_start"], session_dir, last_start_log, timeout_s=R2C_PPP_START_TIMEOUT_S)
        start_text = last_start_log.read_text(errors="replace") if last_start_log and last_start_log.exists() else ""
        if start_rc != 0 or "Modem hangup" in start_text:
            last_reason = f"ppp start exited {start_rc}" if start_rc != 0 else "pppd reported Modem hangup"
        else:
            deadline = now_fn() + args.relatch_ppp_health_timeout_s
            while now_fn() < deadline:
                poll_index = (
                    len(list(session_dir.glob(f"ppp_{log_prefix}_attempt_{attempt}_verify_*.log"))) + 1
                    if session_dir.exists() else 1
                )
                last_verify_log = session_dir / f"ppp_{log_prefix}_attempt_{attempt}_verify_{poll_index}.log"
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
                    f"{stage_label} PPP readiness attempt {attempt}/{attempts}: waiting for healthy link -- {last_reason}"
                )
                sleep_fn(args.relatch_ppp_health_poll_s)

        if "Modem hangup" in start_text:
            last_reason = "pppd reported Modem hangup"

        if attempt < attempts:
            print_fn(
                f"{stage_label} PPP readiness attempt {attempt}/{attempts} failed: {last_reason}; "
                f"retrying after {args.relatch_ppp_retry_delay_s}s"
            )
            sleep_fn(args.relatch_ppp_retry_delay_s)

    return False, (
        f"{failure_label} failed after {attempts} attempts; last reason: {last_reason}; "
        f"last start log: {last_start_log}; last verify log: {last_verify_log}"
    )


def restart_r2c_ppp_with_retries(
    args, plan_by_name: dict, session_dir: Path,
    run_step_fn=run_step, sleep_fn=time.sleep, now_fn=time.monotonic, print_fn=print,
) -> tuple:
    """HIL-F24-R2G compatibility wrapper: bounded R2C PPP reconnect."""
    return restart_ppp_with_retries(
        args, plan_by_name, session_dir, "R2C", failure_label="PPP reconnect",
        run_step_fn=run_step_fn, sleep_fn=sleep_fn, now_fn=now_fn, print_fn=print_fn,
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
    raw_jsbsim_path = Path(VISUAL_FEEDER_RAW_OUTPUT if args.profile == "visual" else DYNAMIC_FEEDER_RAW_OUTPUT)
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
        "--profile", choices=("static", "dynamic", "visual"), default="static",
        help=(
            "static (default, unchanged): sr75_hil_f24d_static_state_feed.py's "
            "fixed row. dynamic (HIL-F24-R1): live SR-75 6-DOF JSBSim state via "
            "sr75_hil_f24r1_dynamic_jsbsim_feed.py, ground-level/motionless, "
            "engines off, RATO/throttle/controls pinned at zero for the whole "
            "run. Requires --confirm-dynamic-jsbsim (in place of "
            "--confirm-static-only) to actually run. visual (HIL-F24-R3A): "
            "scripted read-only Mission Planner visualization trajectory; "
            "requires --stage r3a and --confirm-mp-visualization."
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
        "--stage", choices=("none", "r2", "r2c", "r3a"), default="none",
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
            "--profile dynamic and --confirm-origin-relatch. "
            "r3a (HIL-F24-R3A): safe Mission Planner dynamic visualization "
            "using --profile visual, LOG_ONLY responder, read-only estimator "
            "capture/comparison, MANUAL/disarmed safety precheck."
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
        "--confirm-mp-visualization", action="store_true",
        help=(
            "Explicit confirmation that the HIL-F24-R3A Mission Planner visualization "
            "profile is being run read-only (also requires --execute, --profile visual, "
            "and --stage r3a). Does not substitute for any other confirm flag."
        ),
    )
    parser.add_argument(
        "--visualization-duration-s", type=float, default=DEFAULT_VISUALIZATION_DURATION_S,
        help="HIL-F24-R3A: moving scripted trajectory duration after EKF readiness, default 60 s",
    )
    parser.add_argument(
        "--r3a-ekf-readiness-timeout-s", type=float, default=R3A_EKF_READY_TIMEOUT_S,
        help="HIL-F24-R3G: max stationary wait for EKF readiness before starting visual motion",
    )
    parser.add_argument(
        "--r3a-ekf-readiness-consecutive", type=int, default=R3A_EKF_READY_CONSECUTIVE_REPORTS,
        help="HIL-F24-R3G: consecutive healthy EKF_STATUS_REPORT samples required before visual motion",
    )
    parser.add_argument(
        "--enable-mp-forward", action="store_true",
        help=(
            "HIL-F24-R3B: while preserving Linux ownership of --pixhawk, "
            "forward received Pixhawk MAVLink telemetry one-way to Mission "
            "Planner over UDP. No UDP packets are read and no commands are "
            "written back to the Pixhawk."
        ),
    )
    parser.add_argument(
        "--mp-udp-host", default="127.0.0.1",
        help="HIL-F24-R3B: Mission Planner UDP host for one-way telemetry fanout",
    )
    parser.add_argument(
        "--mp-udp-port", type=int, default=14550,
        help="HIL-F24-R3B: Mission Planner UDP port for one-way telemetry fanout",
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
    if args.profile == "visual":
        print(
            "HIL-F24-R3A visual profile: 60s scripted JSBSim trajectory for Mission Planner display. "
            "Responder remains LOG_ONLY -- no actuator command is ever applied to hardware or JSBSim."
        )
    if args.profile == "dynamic":
        print(
            "HIL-F24-R1 dynamic profile: live JSBSim state, ground-level/motionless, "
            "engines off, RATO/throttle/controls pinned at zero. Responder remains "
            "LOG_ONLY -- no actuator command is ever applied to hardware or JSBSim."
        )
    if args.profile == "visual" and args.stage != "r3a":
        print("ABORT: --profile visual requires --stage r3a -- staying in dry-run")
        print_plan(build_execution_plan(args), args)
        return 0
    if args.stage in ("r2", "r2c", "r3a"):
        print(
            "HIL-F24 read-only stage: Pixhawk-estimator-vs-JSBSim-truth capture/"
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
        if args.stage == "r3a":
            print(
                "HIL-F24-R3A: Mission Planner should show bounded changing position, altitude and attitude "
                "while Pixhawk remains MANUAL/disarmed and the responder stays LOG_ONLY."
            )
        required_profile = "visual" if args.stage == "r3a" else "dynamic"
        if args.profile != required_profile:
            print(f"ABORT: --stage {args.stage} requires --profile {required_profile} -- staying in dry-run")
            print_plan(build_execution_plan(args), args)
            return 0

    plan = build_execution_plan(args)
    proceed, reason = should_proceed_to_hardware(
        args.execute, args.confirm_static_only,
        confirm_dynamic_jsbsim=args.confirm_dynamic_jsbsim, profile=args.profile,
        confirm_estimator_comparison=args.confirm_estimator_comparison, stage=args.stage,
        confirm_origin_relatch=args.confirm_origin_relatch,
        confirm_mp_visualization=args.confirm_mp_visualization,
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

    sudo_ok, sudo_reason = check_sudo_noninteractive()
    if not sudo_ok:
        print(f"ABORT: {sudo_reason}")
        return 3
    print(f"Sudo preflight: {sudo_reason}")

    session_dir = build_session_dir(Path(args.sessions_dir), profile=args.profile, stage=args.stage)
    session_dir.mkdir(parents=True, exist_ok=True)
    print(f"Session artifact directory: {session_dir}")
    if args.stage == "r3a":
        checklist = write_mp_visualization_checklist(session_dir)
        print(f"Mission Planner checklist: {checklist}")

    feeder_proc = None
    responder_proc = None
    capture_proc = None
    r3a_premotion_capture = None
    feeder_log_path = session_dir / "feeder.log"
    truth_feed_valid = True
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

        if args.stage in ("r2c", "r3a"):
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
            raw_jsbsim_path = Path(VISUAL_FEEDER_RAW_OUTPUT if args.stage == "r3a" else DYNAMIC_FEEDER_RAW_OUTPUT)
            if cleanup_stale_raw_jsbsim_evidence(raw_jsbsim_path):
                print(f"Removed stale raw JSBSim evidence: {raw_jsbsim_path}")

            print(f"Starting feeder (HIL-F24-{args.stage.upper()}: before the relatch step)...")
            feeder_proc = subprocess.Popen(
                feeder_cmd, stdout=open(feeder_log_path, "w"), stderr=subprocess.STDOUT,
            )
            print(
                f"Starting responder (HIL-F24-{args.stage.upper()}: before relatch, "
                f"bound to {R2C_RESPONDER_LISTEN_HOST})..."
            )
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

            print("(Re)starting PPP now that the Pixhawk has rebooted...")
            if args.stage == "r3a":
                ppp_ok, ppp_reason = restart_ppp_with_retries(args, plan_by_name, session_dir, "R3A")
                print(f"R3A PPP reconnect: {ppp_reason}")
            else:
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
            if args.stage == "r3a":
                print("Starting R3A pre-motion GPS/EKF diagnostic capture before origin check...")
                r3a_premotion_capture = R3aPremotionDiagnosticCapture(
                    args.pixhawk,
                    args.baud,
                    session_dir / "r3a_premotion_gps_ekf.csv",
                    session_dir / "r3a_premotion_gps_ekf_summary.json",
                    consecutive_required=args.r3a_ekf_readiness_consecutive,
                )
                r3a_premotion_capture.start()
                r3a_premotion_capture.drain(duration_s=1.0)

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
            if r3a_premotion_capture is not None:
                r3a_premotion_capture.drain(duration_s=0.5)
            origin_diagnostic_cmd = [
                part.format(**session_artifact_paths(session_dir))
                for part in plan_by_name["run_origin_relatch_check"].command
            ]
            with open(session_dir / "origin_relatch.log", "w") as log_file:
                subprocess.run(
                    origin_diagnostic_cmd,
                    stdout=log_file,
                    stderr=subprocess.STDOUT,
                    timeout=R2C_ORIGIN_CHECK_TIMEOUT_S,
                )
            summary_path = session_dir / "origin_relatch_summary.json"
            if not summary_path.exists():
                raise OrchestratorAbort(
                    "Origin relatch diagnostic did not produce a summary; see origin_relatch.log"
                )
            origin_summary = json.loads(summary_path.read_text())
            origin_ok, origin_reason = evaluate_origin_relatch_summary(origin_summary, ORIGIN_RELATCH_MAX_MISMATCH_M)
            print(f"Origin relatch check: {origin_reason}")
            if r3a_premotion_capture is not None:
                r3a_premotion_capture.drain(duration_s=0.5)
            if not origin_ok:
                raise OrchestratorAbort(
                    f"EKF-origin relatch gate failed -- {origin_reason}; "
                    f"see origin_relatch.log and origin_relatch_summary.json. Not proceeding to capture."
                )
            if args.stage == "r3a":
                print(
                    f"Waiting up to {args.r3a_ekf_readiness_timeout_s}s for "
                    f"{args.r3a_ekf_readiness_consecutive} consecutive healthy EKF reports..."
                )
                ekf_ok, ekf_reason = r3a_premotion_capture.wait_for_ekf_ready(args.r3a_ekf_readiness_timeout_s)
                print(f"R3A EKF readiness: {ekf_reason}")
                if not ekf_ok:
                    truth_feed_valid = False
                    raise OrchestratorAbort(f"R3A EKF readiness failed -- {ekf_reason}; not starting visual motion")

                if feeder_proc is not None and feeder_proc.poll() is None:
                    feeder_proc.terminate()
                    try:
                        feeder_proc.wait(timeout=5.0)
                    except subprocess.TimeoutExpired:
                        feeder_proc.kill()
                    print("Stopped stationary R3A feeder")
                feeder_proc = None

                motion_feeder_cmd = [
                    part.format(**session_artifact_paths(session_dir))
                    for part in plan_by_name["start_motion_feeder"].command
                ]
                feeder_log_path = session_dir / "motion_feeder.log"
                print("Starting R3A smooth visual trajectory after EKF readiness...")
                feeder_proc = subprocess.Popen(
                    motion_feeder_cmd,
                    stdout=open(feeder_log_path, "w"),
                    stderr=subprocess.STDOUT,
                )
            else:
                print("EKF-origin relatch confirmed -- proceeding to HIL-F24-R2 capture.")
        else:
            print("Starting PPP...")
            if args.stage == "r3a":
                ppp_ok, ppp_reason = restart_ppp_with_retries(args, plan_by_name, session_dir, "R3A")
                print(f"R3A PPP readiness: {ppp_reason}")
                if not ppp_ok:
                    raise OrchestratorAbort(ppp_reason)
            else:
                rc = run_step(plan_by_name["ppp_start"], session_dir, session_dir / "ppp_start.log", timeout_s=30.0)
                if rc != 0:
                    raise OrchestratorAbort(f"PPP failed to start (exit code {rc}); see ppp_start.log")

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

        if args.stage in ("r2", "r2c", "r3a"):
            print("Starting read-only estimator capture (HIL-F24-R2, after responder)...")
            capture_cmd = [
                part.format(**session_artifact_paths(session_dir))
                for part in plan_by_name["start_estimator_capture"].command
            ]
            capture_proc = subprocess.Popen(
                capture_cmd, stdout=open(session_dir / "capture.log", "w"), stderr=subprocess.STDOUT,
            )

        run_duration_s = args.visualization_duration_s if args.stage == "r3a" else args.duration_s
        print(
            f"SIM_JSON test (--profile {args.profile}) running for {run_duration_s}s "
            f"(feeder+responder); artifacts in {session_dir}"
        )
        if args.stage == "r3a":
            feed_ok, feed_reason = monitor_r3a_truth_feed(
                feeder_proc,
                session_dir / "state.csv",
                feeder_log_path,
                run_duration_s,
            )
            print(f"R3A truth-feed monitor: {feed_reason}")
            if not feed_ok:
                truth_feed_valid = False
                raise OrchestratorAbort(feed_reason)
            time.sleep(ORCHESTRATOR_TEST_MARGIN_S)
        else:
            time.sleep(run_duration_s + ORCHESTRATOR_TEST_MARGIN_S)
        print("SIM_JSON test window complete.")

    except OrchestratorAbort as exc:
        print(f"ABORT: {exc}")
        exit_code = 3
    finally:
        if r3a_premotion_capture is not None:
            try:
                r3a_premotion_capture.close()
            except Exception as exc:
                print(f"WARN: failed to close R3A pre-motion diagnostic capture cleanly: {exc}")
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
        if should_run_estimator_comparison(args.stage, capture_proc, truth_feed_valid):
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
        elif args.stage == "r3a" and capture_proc is not None and not truth_feed_valid:
            print("Skipping estimator comparison because R3A truth feed was invalid.")
        if args.profile in ("dynamic", "visual"):
            # HIL-F24-R1 task 6: record the raw JSBSim state alongside the
            # responder's own log, and re-run the read-only safety
            # precheck (mode/armed/CH7/CH8) after the test as well as
            # before, so the session directory holds before-and-after
            # safety evidence, not just a single pre-test snapshot.
            record_dynamic_profile_evidence(args, session_dir)
        print("Stopping PPP (best-effort cleanup on exit)...")
        try:
            run_step(plan_by_name["ppp_stop"], session_dir, session_dir / "ppp_stop.log", timeout_s=30.0)
        except OrchestratorAbort as exc:
            print(f"PPP cleanup failed: {exc}")
            exit_code = exit_code or 5

    print(f"All artifacts recorded under: {session_dir}")
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
