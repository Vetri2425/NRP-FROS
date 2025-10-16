#!/usr/bin/env python3
"""
interval_spray.py
Connects to MAVLink (udp:127.0.0.1:14550) and toggles a servo ON/OFF at either
time-based or distance-based intervals while the mission runs. Optionally, the
spray window can be limited between a start and end waypoint.

Configuration (Backend/servo_manager/config.json → interval_spray):
  - servo_number (int, default 10)
  - pwm_on (int, default 650)
  - pwm_off (int, default 1000)
  - toggle_mode (str: 'timer' | 'distance', default 'timer')
  - on_time_s (float, for timer mode; default 1.0)
  - off_time_s (float, for timer mode; default 1.0)
  - distance_interval_m (float, for distance mode; default 1.0)
  - start_wp (int, optional; default -1 → start immediately)
  - end_wp (int, optional; default -1 → run until stopped)

Behavior:
  - If start_wp >= 0: wait until MISSION_ITEM_REACHED for that wp (tolerate 0- or 1-based)
  - If end_wp >= 0: stop spraying after that wp is reached (ensure OFF)
  - Timer mode: alternate ON/OFF every on_time_s/off_time_s while active
  - Distance mode: toggle ON/OFF every distance_interval_m traveled while active,
                   based on GLOBAL_POSITION_INT updates (lat/lon only)
"""

import json
import math
import os
import signal
import sys
import time
from typing import Any, Dict, Optional, Tuple

from pymavlink import mavutil

BASE = os.path.dirname(__file__)
CFG = os.path.join(BASE, "config.json")
CONNECTION_STRING = os.environ.get("INTERVAL_CONNECTION", "udp:127.0.0.1:14550")

running = True


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
    cfg = load_cfg().get("interval_spray", {})
    toggle_mode = str(cfg.get("toggle_mode", "timer")).strip().lower() or "timer"
    return {
        "servo_number": int(cfg.get("servo_number", 10) or 10),
        "pwm_on": int(cfg.get("pwm_on", 650) or 650),
        "pwm_off": int(cfg.get("pwm_off", 1000) or 1000),
        "toggle_mode": toggle_mode,
        "on_time_s": float(cfg.get("on_time_s", cfg.get("spray_on_time", 1.0)) or 1.0),
        "off_time_s": float(cfg.get("off_time_s", cfg.get("interval_time", 1.0)) or 1.0),
        "distance_interval_m": float(cfg.get("distance_interval_m", 1.0) or 1.0),
        "start_wp": int(cfg.get("start_wp", -1) or -1),
        "end_wp": int(cfg.get("end_wp", -1) or -1),
    }


def wait_heartbeat(master: mavutil.mavfile):
    print("[interval_spray] Waiting for heartbeat...", flush=True)
    master.wait_heartbeat(timeout=10)
    print(
        f"[interval_spray] Heartbeat received from system {master.target_system} component {master.target_component}",
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
        print(f"[interval_spray] DO_SET_SERVO ch={servo_no} pwm={pwm}", flush=True)
    except Exception as e:
        print(f"[interval_spray] ERROR sending DO_SET_SERVO: {e}", flush=True)


def _matches_seq(reached_seq: int, wp_config: int) -> bool:
    return reached_seq == wp_config or reached_seq == wp_config - 1


def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    R = 6371000.0
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat / 2) ** 2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2) ** 2
    return 2 * R * math.asin(min(1.0, math.sqrt(a)))


def _extract_latlon(msg) -> Optional[Tuple[float, float]]:
    try:
        if msg.get_type() == "GLOBAL_POSITION_INT":
            lat = getattr(msg, "lat", None)
            lon = getattr(msg, "lon", None)
            if lat is not None and lon is not None:
                return float(lat) / 1e7, float(lon) / 1e7
        if msg.get_type() == "GPS_RAW_INT":
            lat = getattr(msg, "lat", None)
            lon = getattr(msg, "lon", None)
            if lat is not None and lon is not None and lat != 0 and lon != 0:
                return float(lat) / 1e7, float(lon) / 1e7
    except Exception:
        pass
    return None


def main():
    global running
    params = get_params()
    print(f"[interval_spray] Starting with params: {params}", flush=True)

    master = None
    while running and master is None:
        try:
            master = mavutil.mavlink_connection(CONNECTION_STRING)
            wait_heartbeat(master)
        except Exception as e:
            print(f"[interval_spray] Connection failed: {e} (retrying in 2s)", flush=True)
            master = None
            time.sleep(2)

    if not running or master is None:
        print("[interval_spray] Exiting (not running or no connection)", flush=True)
        return

    active = False  # spraying window active (between start/end)
    if params["start_wp"] < 0:
        active = True

    last_seq = -1
    servo_state_on = False
    last_toggle_time = time.monotonic()
    acc_dist = 0.0
    last_pos = None  # (lat, lon)

    print("[interval_spray] Listening for position and mission events...", flush=True)

    while running:
        try:
            msg = master.recv_match(type=[
                "MISSION_ITEM_REACHED", "GLOBAL_POSITION_INT", "GPS_RAW_INT"
            ], blocking=True, timeout=0.5)
        except Exception:
            msg = None

        if msg is None:
            # Still process time-based toggling even if no messages arrive
            if active and params["toggle_mode"] == "timer":
                now = time.monotonic()
                if servo_state_on and now - last_toggle_time >= params["on_time_s"]:
                    send_do_set_servo(master, params["servo_number"], params["pwm_off"])
                    servo_state_on = False
                    last_toggle_time = now
                elif not servo_state_on and now - last_toggle_time >= params["off_time_s"]:
                    send_do_set_servo(master, params["servo_number"], params["pwm_on"])
                    servo_state_on = True
                    last_toggle_time = now
            continue

        mtype = msg.get_type()

        # Start/end window control
        if mtype == "MISSION_ITEM_REACHED":
            try:
                seq = int(getattr(msg, "seq", -1))
            except Exception:
                seq = -1
            if seq >= 0 and seq != last_seq:
                last_seq = seq
                if not active and params["start_wp"] >= 0 and _matches_seq(seq, params["start_wp"]):
                    print(f"[interval_spray] Start window @seq={seq}", flush=True)
                    active = True
                    # reset timers/counters at window start
                    last_toggle_time = time.monotonic()
                    acc_dist = 0.0
                    last_pos = None
                    servo_state_on = False
                if active and params["end_wp"] >= 0 and _matches_seq(seq, params["end_wp"]):
                    print(f"[interval_spray] End window @seq={seq}", flush=True)
                    if servo_state_on:
                        send_do_set_servo(master, params["servo_number"], params["pwm_off"])
                        servo_state_on = False
                    active = False
                    # If both start and end are defined and reached, we can exit
                    if params["start_wp"] >= 0:
                        break
            continue

        # Position updates for distance toggling
        if active and params["toggle_mode"] == "distance":
            latlon = _extract_latlon(msg)
            if latlon is not None:
                if last_pos is not None:
                    acc_dist += _haversine_m(last_pos[0], last_pos[1], latlon[0], latlon[1])
                last_pos = latlon
                if acc_dist >= params["distance_interval_m"]:
                    # toggle
                    if servo_state_on:
                        send_do_set_servo(master, params["servo_number"], params["pwm_off"])
                        servo_state_on = False
                    else:
                        send_do_set_servo(master, params["servo_number"], params["pwm_on"])
                        servo_state_on = True
                    acc_dist = 0.0

        # Timer toggling handled also when messages are present
        if active and params["toggle_mode"] == "timer":
            now = time.monotonic()
            if servo_state_on and now - last_toggle_time >= params["on_time_s"]:
                send_do_set_servo(master, params["servo_number"], params["pwm_off"])
                servo_state_on = False
                last_toggle_time = now
            elif not servo_state_on and now - last_toggle_time >= params["off_time_s"]:
                send_do_set_servo(master, params["servo_number"], params["pwm_on"])
                servo_state_on = True
                last_toggle_time = now

    # Ensure OFF on exit
    if master and servo_state_on:
        send_do_set_servo(master, params["servo_number"], params["pwm_off"])
    print("[interval_spray] Exiting", flush=True)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("[interval_spray] Interrupted", flush=True)
