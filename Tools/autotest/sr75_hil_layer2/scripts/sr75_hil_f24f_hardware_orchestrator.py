#!/usr/bin/env python3
"""HIL-F24-F item 6/7: single future-hardware orchestrator for the fmuv3
Simulation-on-Hardware bench STATIC SIM_JSON test.

Scope is intentionally narrow: this orchestrator only ever drives the
static-level SIM_JSON test (sr75_hil_f24d_static_state_feed.py) over PPP to
a real Pixhawk. It never selects a dynamic profile, never arms, never
enables actuator output (the responder is started without
--jsbsim-command-target, so no actuator/PWM command path exists at all in
this configuration), and never sends AUTO/mission commands.

Safety gating (item 7):
  --dry-run is the default. It prints the exact plan/commands this script
  WOULD run and exits 0 without touching anything.
  Actually starting PPP/the feeder/the responder against real hardware
  requires BOTH --execute AND --confirm-static-only.

Execution plan (item 6), in order:
  1. Check /dev/ttyACM0 (Pixhawk USB MAVLink) and /dev/ttyUSB0 (PPP UART
     adapter) exist.
  2. Read-only SoH safety precheck (subprocess: sr75_hil_f24c_preflash_
     precheck.py) -- abort here on STOP, before touching PPP at all.
  3. Start PPP (sr75_ppp_start.sh --background).
  4. Verify ppp0 is up and the Pixhawk PPP endpoint responds to ping --
     abort (and stop PPP) before starting SIM_JSON if this fails.
  5. Start the static-state feeder (sr75_hil_f24d_static_state_feed.py),
     then the responder (sr75_sim_json_responder.py, no actuator/JSBSim
     command target) -- feeder before responder, as required.
  6. Record every artifact (precheck output, ppp status, feeder log,
     responder log/CSV) under a timestamped session directory.
  7. On exit -- success, failure, or exception -- always stop the
     feeder/responder and always stop PPP.

No PARAM_SET, arm, mission, RC override, or actuator command is ever sent.
"""
import argparse
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

PRECHECK_SCRIPT = SCRIPTS_DIR / "sr75_hil_f24c_preflash_precheck.py"
FEEDER_SCRIPT = SIM_JSON_DIR / "sr75_hil_f24d_static_state_feed.py"
RESPONDER_SCRIPT = SIM_JSON_DIR / "sr75_sim_json_responder.py"
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


def should_proceed_to_hardware(execute: bool, confirm_static_only: bool) -> tuple:
    """Pure: item 7's gating logic. Both flags are required; --dry-run
    (i.e. neither flag, or only one) always keeps this a dry run."""
    if execute and confirm_static_only:
        return True, ""
    if execute and not confirm_static_only:
        return False, "--execute given without --confirm-static-only -- staying in dry-run"
    if confirm_static_only and not execute:
        return False, "--confirm-static-only given without --execute -- staying in dry-run"
    return False, "dry-run (default): pass --execute --confirm-static-only to run against real hardware"


def build_session_dir(base_dir: Path, now: Optional[datetime] = None) -> Path:
    stamp = (now or datetime.now(timezone.utc)).strftime("%Y%m%dT%H%M%SZ")
    return base_dir / f"sr75_hil_f24f_static_{stamp}"


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
    feeder_cmd = [
        sys.executable, str(FEEDER_SCRIPT), "--output", "{state_csv}",
        "--duration-s", str(args.duration_s), "--rate-hz", str(args.rate_hz),
    ]
    responder_cmd = [
        sys.executable, str(RESPONDER_SCRIPT),
        "--listen-host", args.host_ip, "--listen-port", str(args.listen_port),
        "--state-file", "{state_csv}", "--log-csv", "{responder_csv}", "--strict",
        # Deliberately no --jsbsim-command-target / --actuator-map: this
        # keeps the responder in LOG_ONLY actuator mode, so there is no
        # actuator/PWM command path to send anywhere, matching item 9's
        # "never enable actuator output".
    ]
    return [
        Step("check_adapters", f"Check {args.pixhawk} and {args.ppp_device} exist", None),
        Step("precheck", "Read-only SoH safety precheck (abort on STOP, before touching PPP)", precheck_cmd),
        Step("ppp_start", "Start PPP link (only reached if --execute --confirm-static-only)", ppp_start_cmd),
        Step("ppp_verify", "Verify ppp0 is up and Pixhawk PPP endpoint responds to ping", ppp_status_cmd),
        Step("start_feeder", "Start static-state feeder (before responder)", feeder_cmd),
        Step("start_responder", "Start responder in LOG_ONLY mode (no actuator output)", responder_cmd),
        Step("ppp_stop", "Stop PPP (always run on exit, success or failure)", ppp_stop_cmd),
    ]


def print_plan(plan: List[Step], args) -> None:
    print("Execution plan (dry-run -- nothing below has been executed):")
    for i, step in enumerate(plan, start=1):
        print(f"  {i}. [{step.name}] {step.description}")
        if step.command:
            rendered = " ".join(step.command).format(
                state_csv="<session_dir>/state.csv", responder_csv="<session_dir>/responder.csv",
            )
            print(f"       $ {rendered}")


def run_step(step: Step, session_dir: Path, log_path: Path, timeout_s: Optional[float] = None) -> int:
    rendered = [
        part.format(state_csv=str(session_dir / "state.csv"), responder_csv=str(session_dir / "responder.csv"))
        for part in step.command
    ]
    with open(log_path, "w") as log_file:
        proc = subprocess.run(rendered, stdout=log_file, stderr=subprocess.STDOUT, timeout=timeout_s)
    return proc.returncode


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
    return parser


def main():
    args = build_arg_parser().parse_args()
    print("=== HIL-F24-F hardware orchestrator (static SIM_JSON test only) ===")
    print("Never arms. Never sends actuator/servo output. Never runs AUTO/mission.")

    plan = build_execution_plan(args)
    proceed, reason = should_proceed_to_hardware(args.execute, args.confirm_static_only)

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

    session_dir = build_session_dir(Path(args.sessions_dir))
    session_dir.mkdir(parents=True, exist_ok=True)
    print(f"Session artifact directory: {session_dir}")

    ppp_started = False
    feeder_proc = None
    responder_proc = None
    exit_code = 0
    try:
        plan_by_name = {step.name: step for step in plan}

        print("Running read-only SoH precheck...")
        rc = run_step(plan_by_name["precheck"], session_dir, session_dir / "precheck.log", timeout_s=60.0)
        if rc != 0:
            raise OrchestratorAbort(f"SoH precheck did not return GO (exit code {rc}); see precheck.log")

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
        feeder_cmd = [
            part.format(state_csv=str(session_dir / "state.csv"))
            for part in plan_by_name["start_feeder"].command
        ]
        feeder_proc = subprocess.Popen(
            feeder_cmd, stdout=open(session_dir / "feeder.log", "w"), stderr=subprocess.STDOUT,
        )

        print("Starting responder (LOG_ONLY, no actuator output)...")
        responder_cmd = [
            part.format(state_csv=str(session_dir / "state.csv"), responder_csv=str(session_dir / "responder.csv"))
            for part in plan_by_name["start_responder"].command
        ]
        responder_proc = subprocess.Popen(
            responder_cmd, stdout=open(session_dir / "responder.log", "w"), stderr=subprocess.STDOUT,
        )

        print(f"Static SIM_JSON test running for {args.duration_s}s (feeder+responder); artifacts in {session_dir}")
        time.sleep(args.duration_s + 2.0)
        print("Static SIM_JSON test window complete.")

    except OrchestratorAbort as exc:
        print(f"ABORT: {exc}")
        exit_code = 3
    finally:
        for proc, label in ((responder_proc, "responder"), (feeder_proc, "feeder")):
            if proc is not None and proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    proc.kill()
                print(f"Stopped {label}")
        if ppp_started:
            print("Stopping PPP (always run on exit)...")
            run_step(plan_by_name["ppp_stop"], session_dir, session_dir / "ppp_stop.log", timeout_s=30.0)

    print(f"All artifacts recorded under: {session_dir}")
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
