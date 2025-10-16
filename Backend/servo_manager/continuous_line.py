#!/usr/bin/env python3
"""
continuous_line.py
Connects to MAVLink at udp:127.0.0.1:14550 and waits for mission waypoint
reach events. Behavior:
  - On reaching configured start waypoint:
      delay (continuous_line.delay_before_on) seconds → DO_SET_SERVO ON
  - Keep servo ON while traveling to end waypoint
  - On reaching configured end waypoint:
      delay (continuous_line.delay_after_off) seconds → DO_SET_SERVO OFF

User-configurable parameters (Backend/servo_manager/config.json → continuous_line):
  - servo_number (int, default 10)
  - pwm_on (int, default 650)
  - pwm_off (int, default 1000)
  - start_wp (int, REQUIRED)
  - end_wp (int, REQUIRED)
  - delay_before_on (float, default 0.5)
  - delay_after_off (float, default 0.5)

Note: Mission seq numbers in MISSION_ITEM_REACHED may be 0-based. If you
enter 1-based waypoint numbers in the UI, start_wp=1 corresponds to seq 0.
This script treats matches as either exact or minus one to tolerate both.
"""

import json
import os
import signal
import sys
import time
from typing import Any, Dict

from pymavlink import mavutil

BASE = os.path.dirname(__file__)
CFG = os.path.join(BASE, "config.json")
CONNECTION_STRING = os.environ.get("CONTINUOUS_CONNECTION", "udp:127.0.0.1:14550")

running = True
servo_on_sent = False


def _sigterm(_sig, _frm):
    global running
    running = False


signal.signal(signal.SIGTERM, _sigterm)


def load_cfg() -> Dict[str, Any]:
    try:
        with open(CFG, "r") as f:
            return json.load(f)
    except Exception:
        return {}


def get_params() -> Dict[str, Any]:
    cfg = load_cfg().get("continuous_line", {})
    return {
        "servo_number": int(cfg.get("servo_number", 10) or 10),
        "pwm_on": int(cfg.get("pwm_on", 650) or 650),
        "pwm_off": int(cfg.get("pwm_off", 1000) or 1000),
        "start_wp": int(cfg.get("start_wp", -1) or -1),
        "end_wp": int(cfg.get("end_wp", -1) or -1),
        "delay_before_on": float(cfg.get("delay_before_on", 0.5) or 0.5),
        "delay_after_off": float(cfg.get("delay_after_off", 0.5) or 0.5),
    }


def wait_heartbeat(master: mavutil.mavfile):
    print("[continuous_line] Waiting for heartbeat...", flush=True)
    master.wait_heartbeat(timeout=10)
    print(
        f"[continuous_line] Heartbeat received from system {master.target_system} component {master.target_component}",
        flush=True,
    )


def send_do_set_servo(master: mavutil.mavfile, servo_no: int, pwm: int):
    try:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,
            float(servo_no),
            float(pwm),
            0, 0, 0, 0, 0,
        )
        print(f"[continuous_line] DO_SET_SERVO ch={servo_no} pwm={pwm}", flush=True)
    except Exception as e:
        print(f"[continuous_line] ERROR sending DO_SET_SERVO: {e}", flush=True)


def _matches_seq(reached_seq: int, wp_config: int) -> bool:
    # Tolerate 0-based vs 1-based inputs: accept exact or -1
    return reached_seq == wp_config or reached_seq == wp_config - 1


def main():
    global running, servo_on_sent
    params = get_params()
    if params["start_wp"] < 0 or params["end_wp"] < 0:
        print("[continuous_line] ERROR: start_wp and end_wp must be set in config", flush=True)
        return
    print(
        f"[continuous_line] Starting with params: start={params['start_wp']} end={params['end_wp']} servo={params['servo_number']} pwm_on={params['pwm_on']} pwm_off={params['pwm_off']} delays=({params['delay_before_on']},{params['delay_after_off']})",
        flush=True,
    )

    master = None
    while running and master is None:
        try:
            master = mavutil.mavlink_connection(CONNECTION_STRING)
            wait_heartbeat(master)
        except Exception as e:
            print(f"[continuous_line] Connection failed: {e} (retrying in 2s)", flush=True)
            master = None
            time.sleep(2)

    if not running or master is None:
        print("[continuous_line] Exiting (not running or no connection)", flush=True)
        return

    started = False
    finished = False
    last_seq = -1
    print("[continuous_line] Listening for MISSION_ITEM_REACHED messages...", flush=True)

    while running and not finished:
        try:
            msg = master.recv_match(type="MISSION_ITEM_REACHED", blocking=True, timeout=1.0)
        except Exception:
            msg = None
        if msg is None:
            continue

        try:
            seq = int(getattr(msg, "seq", -1))
        except Exception:
            seq = -1
        if seq < 0 or seq == last_seq:
            continue
        last_seq = seq

        if not started and _matches_seq(seq, params["start_wp"]):
            print(f"[continuous_line] Start WP reached (seq={seq})", flush=True)
            t = params["delay_before_on"]
            if t > 0:
                time.sleep(t)
            send_do_set_servo(master, params["servo_number"], params["pwm_on"])
            servo_on_sent = True
            started = True
            continue

        if started and _matches_seq(seq, params["end_wp"]):
            print(f"[continuous_line] End WP reached (seq={seq})", flush=True)
            t = params["delay_after_off"]
            if t > 0:
                time.sleep(t)
            send_do_set_servo(master, params["servo_number"], params["pwm_off"])
            finished = True

    print("[continuous_line] Exiting", flush=True)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("[continuous_line] Interrupted", flush=True)
