import eventlet
eventlet.monkey_patch()  # <-- ADD THIS LINE
import threading
import time
import math
import json
import base64
import socket
from collections import deque
from flask import Flask, request, jsonify
from flask_socketio import SocketIO, emit
from flask_cors import CORS
from pymavlink.dialects.v20 import common as mavlink
from dataclasses import dataclass, asdict, field
from typing import Optional, List
import os
import signal

try:
    # When running as a package (e.g., python -m Backend.server)
    from .mavlink_core import get_mavlink_core  # type: ignore
except Exception:
    # When running as a script from the Backend directory
    from mavlink_core import get_mavlink_core  # type: ignore

# --- Configuration ---
CONNECTION_STRING = 'udp:127.0.0.1:14551'  # Default connection string (can be overridden)
BAUD_RATE = 115200

# --- Flask & SocketIO Setup ---
app = Flask(__name__)
CORS(app, resources={r"/*": {"origins": "*"}})
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='eventlet',
                    ping_timeout=60, ping_interval=25)

# --- MAVLink Connection ---
master = None
is_vehicle_connected = False
current_mission = []  # Store current mission waypoints


@dataclass
class Position:
    lat: float
    lng: float


@dataclass
class CurrentState:
    # Will be set when GPS available
    position: Optional[Position] = None
    heading: float = 0.0
    battery: int = -1
    status: str = 'disarmed'
    mode: str = 'UNKNOWN'
    rtk_status: str = 'No GPS'
    signal_strength: str = 'No Link'
    current_waypoint_id: Optional[int] = None
    rc_connected: bool = False
    last_heartbeat: Optional[float] = None
    # UI expected fields
    hrms: str = '0.000'
    vrms: str = '0.000'
    imu_status: str = 'UNALIGNED'
    activeWaypointIndex: Optional[int] = None
    completedWaypointIds: List[int] = field(default_factory=list)
    distanceToNext: float = 0.0
    # Timestamp pushed to frontend (seconds since epoch)
    last_update: Optional[float] = None

    def to_dict(self) -> dict:
        d = asdict(self)
        return d


# Initialize current vehicle state
current_state = CurrentState()

pending_command_queue: deque[dict] = deque()
mavlink_io_lock = threading.Lock()
mission_upload_lock = threading.Lock()
mission_download_lock = threading.Lock()

# Real-time streaming throttle configuration
def _compute_emit_interval() -> float:
    try:
        min_ms_env = os.getenv('TELEMETRY_MIN_MS')
        if min_ms_env is not None:
            min_ms = max(1.0, float(min_ms_env))
            return max(0.001, min_ms / 1000.0)
        hz = float(os.getenv('TELEMETRY_HZ', '20'))
        return max(0.001, 1.0 / max(1.0, hz))
    except Exception:
        return 0.05  # ~20Hz


EMIT_MIN_INTERVAL = _compute_emit_interval()
last_emit_monotonic = 0.0
_emit_lock = threading.Lock()
_emit_timer: Optional[object] = None

# Pending ACK waiters
_ack_waiters_lock = threading.Lock()
_ack_waiters: dict[str, list[dict]] = {}

MISSION_LOG_HISTORY_MAX = 1000
mission_log_history: deque[dict] = deque(maxlen=MISSION_LOG_HISTORY_MAX)
mission_log_state = {
    'last_active_seq': None,
    'last_reached_seq': None,
}

COMMAND_ID_TO_NAME = {
    mavlink.MAV_CMD_COMPONENT_ARM_DISARM: 'ARM_DISARM',
    mavlink.MAV_CMD_DO_SET_MODE: 'SET_MODE',
}

COMMAND_NAME_TO_ID = {
    'WAYPOINT': mavlink.MAV_CMD_NAV_WAYPOINT,
    'NAV_WAYPOINT': mavlink.MAV_CMD_NAV_WAYPOINT,
    'TAKEOFF': mavlink.MAV_CMD_NAV_TAKEOFF,
    'NAV_TAKEOFF': mavlink.MAV_CMD_NAV_TAKEOFF,
    'LAND': mavlink.MAV_CMD_NAV_LAND,
    'NAV_LAND': mavlink.MAV_CMD_NAV_LAND,
    'SPLINE_WAYPOINT': mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT,
    'DO_SET_SERVO': mavlink.MAV_CMD_DO_SET_SERVO,
    'DO_CHANGE_SPEED': mavlink.MAV_CMD_DO_CHANGE_SPEED,
    'DO_SET_HOME': mavlink.MAV_CMD_DO_SET_HOME,
    # 🔵 ADDED: wait N seconds before continuing (used for spray duration)
    'CONDITION_DELAY': mavlink.MAV_CMD_CONDITION_DELAY,
}

COMMAND_RESULT_MAP = {
    mavlink.MAV_RESULT_ACCEPTED: ('success', 'Command accepted'),
    mavlink.MAV_RESULT_TEMPORARILY_REJECTED: ('error', 'Command temporarily rejected'),
    mavlink.MAV_RESULT_DENIED: ('error', 'Command denied'),
    mavlink.MAV_RESULT_UNSUPPORTED: ('error', 'Command unsupported'),
    mavlink.MAV_RESULT_FAILED: ('error', 'Command failed'),
    mavlink.MAV_RESULT_IN_PROGRESS: ('pending', 'Command in progress'),
}

# Mode mappings
COPTER_MODES = {
    0: 'STABILIZE', 1: 'ACRO', 2: 'ALT_HOLD', 3: 'AUTO', 4: 'GUIDED',
    5: 'LOITER', 6: 'RTL', 7: 'CIRCLE', 9: 'LAND', 11: 'DRIFT',
    13: 'SPORT', 14: 'FLIP', 15: 'AUTOTUNE', 16: 'POSHOLD', 17: 'BRAKE',
    18: 'THROW', 19: 'AVOID_ADSB', 20: 'GUIDED_NOGPS', 21: 'SMART_RTL',
    22: 'FLOWHOLD', 23: 'FOLLOW', 24: 'ZIGZAG', 25: 'SYSTEMID', 26: 'AUTOROTATE'
}

ROVER_MODES = {
    0: 'MANUAL', 1: 'ACRO', 3: 'STEERING', 4: 'HOLD', 10: 'AUTO',
    11: 'RTL', 12: 'SMART_RTL', 15: 'GUIDED', 16: 'INITIALISING'
}


# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================

def log_message(message, level='INFO'):
    """Simple logger that prints to stdout and emits to frontend."""
    try:
        ts = time.strftime('%Y-%m-%d %H:%M:%S')
        text = f"[{ts}] [{level}] {message}"
        print(text, flush=True)
        try:
            socketio.emit('server_log', {'message': text, 'level': level})
        except Exception:
            pass
    except Exception:
        try:
            print(f"[LOG-ERR] {message}", flush=True)
        except:
            pass


def _current_position() -> tuple[float | None, float | None]:
    try:
        if current_state and current_state.position:
            return current_state.position.lat, current_state.position.lng
    except Exception:
        pass
    return None, None


def _record_mission_event(
    message: str,
    *,
    status: str | None = None,
    waypoint_id: int | None = None,
    servo_action: str | None = None
) -> None:
    lat, lng = _current_position()
    entry = {
        'timestamp': time.time(),
        'message': message,
        'status': status,
        'waypointId': waypoint_id,
        'servoAction': servo_action,
        'lat': lat,
        'lng': lng,
    }
    mission_log_history.append(entry)
    try:
        socketio.emit('mission_event', entry)
    except Exception:
        pass


def safe_float(value, default=0.0) -> float:
    """Best-effort float conversion with fallback."""
    try:
        if value is None:
            return float(default)
        return float(value)
    except (TypeError, ValueError):
        return float(default)


# ============================================================
# SERVO CONTROL MISSION BUILDER (3 modes)
# ============================================================

def _is_nav_waypoint(cmd_name_or_id) -> bool:
    cid = resolve_mav_command(cmd_name_or_id)
    return command_requires_nav_coordinates(cid)

def _mk_do_set_servo(channel: int, pwm: float) -> dict:
    # DO_SET_SERVO uses param1=servo no, param2=PWM. No coordinates required.
    return {
        'command': 'DO_SET_SERVO',
        'param1': float(channel),
        'param2': float(pwm),
        'param3': 0.0,
        'param4': 0.0,
        'lat': 0.0, 'lng': 0.0, 'alt': 0.0,
        'frame': mavlink.MAV_FRAME_MISSION
    }

def _mk_condition_delay(seconds: float) -> dict:
    # Wait N seconds before proceeding
    return {
        'command': 'CONDITION_DELAY',
        'param1': float(seconds),
        'param2': 0.0, 'param3': 0.0, 'param4': 0.0,
        'lat': 0.0, 'lng': 0.0, 'alt': 0.0,
        'frame': mavlink.MAV_FRAME_MISSION
    }

def _mk_nav_wp(lat: float, lng: float, alt: float, frame: int = mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT) -> dict:
    return {
        'command': 'WAYPOINT',
        'lat': float(lat), 'lng': float(lng), 'alt': float(alt),
        'frame': int(frame),
        'param1': 0.0, 'param2': 0.0, 'param3': 0.0, 'param4': 0.0
    }

def apply_servo_modes(waypoints: list[dict], servo_config: dict) -> list[dict]:
    """
    Expand mission with DO_SET_SERVO items based on the popup config.
    Modes:
      1) MARK_AT_WAYPOINT: ON -> delay(s) -> OFF after each WP in [fromWp..toWp]
      2) CONTINUOUS_LINE: ON after startPoint, OFF after endPoint
      3) INTERVAL_SPRAY: insert mini-waypoints every intervalCm, toggling ON/OFF
    """
    if not servo_config:
        return waypoints

    mode = str(servo_config.get('mode', '')).upper()
    servo = int(safe_float(servo_config.get('servoNumber', 10), 10))
    pwm_on = float(safe_float(servo_config.get('pwmOn', 650), 650))
    pwm_off = float(safe_float(servo_config.get('pwmOff', 1000), 1000))

    # Helper: iterate original nav WPs with a 1-based index
    def _iter_nav_indexed():
        for idx, wp in enumerate(waypoints, start=1):
            yield (idx, wp)

    # -------------------------
    # Mode 1: MARK_AT_WAYPOINT
    # -------------------------
    if mode == 'MARK_AT_WAYPOINT':
        from_wp = int(safe_float(servo_config.get('fromWp', 1), 1))
        to_wp = int(safe_float(servo_config.get('toWp', len(waypoints)), len(waypoints)))
        dur_s = float(safe_float(servo_config.get('sprayDuration', 0.5), 0.5))

        new_list: list[dict] = []
        for idx, wp in _iter_nav_indexed():
            new_list.append(wp)
            if from_wp <= idx <= to_wp and _is_nav_waypoint(wp.get('command')):
                # ON right after reaching this WP, hold for dur_s, then OFF
                new_list.append(_mk_do_set_servo(servo, pwm_on))
                if dur_s > 0:
                    new_list.append(_mk_condition_delay(dur_s))
                new_list.append(_mk_do_set_servo(servo, pwm_off))
        return new_list

    # -------------------------
    # Mode 2: CONTINUOUS_LINE
    # -------------------------
    if mode == 'CONTINUOUS_LINE':
        start_pt = int(safe_float(servo_config.get('startPoint', 1), 1))
        end_pt = int(safe_float(servo_config.get('endPoint', len(waypoints)), len(waypoints)))
        if end_pt < start_pt:
            end_pt = start_pt

        new_list: list[dict] = []
        for idx, wp in _iter_nav_indexed():
            new_list.append(wp)
            if idx == start_pt and _is_nav_waypoint(wp.get('command')):
                # turn ON immediately after we "arrive" at start WP
                new_list.append(_mk_do_set_servo(servo, pwm_on))
            if idx == end_pt and _is_nav_waypoint(wp.get('command')):
                # turn OFF after end WP
                new_list.append(_mk_do_set_servo(servo, pwm_off))
        return new_list

    # -------------------------
    # Mode 3: INTERVAL_SPRAY
    # -------------------------
    if mode == 'INTERVAL_SPRAY':
        interval_cm = float(safe_float(servo_config.get('intervalCm', 30.0), 30.0))
        step_m = max(0.01, interval_cm / 100.0)  # meters

        # Prepare a new mission with sub-steps between each consecutive NAV WP
        import math
        def haversine_m(lat1, lon1, lat2, lon2):
            # rough distance in meters (good enough for path splitting)
            R = 6371000.0
            dlat = math.radians(lat2 - lat1)
            dlon = math.radians(lon2 - lon1)
            a = math.sin(dlat/2)**2 + math.cos(math.radians(lat1))*math.cos(math.radians(lat2))*math.sin(dlon/2)**2
            return 2 * R * math.asin(min(1, math.sqrt(a)))

        def lerp(a, b, t): return a + (b - a) * t

        # filter only the NAV waypoints (we preserve order and any pre-existing DO_* too)
        # but we build intervals only between NAV->NAV legs
        new_list: list[dict] = []
        toggle_on = True  # alternate ON/OFF per sub-step

        # build a list of indices of NAV waypoints in original list
        nav_indices = [i for i, w in enumerate(waypoints) if _is_nav_waypoint(w.get('command'))]

        for i, wp in enumerate(waypoints):
            new_list.append(wp)

            # if this is a NAV wp and the next NAV wp exists, insert sub-steps after this one
            if i in nav_indices:
                cur_idx_in_nav = nav_indices.index(i)
                if cur_idx_in_nav < len(nav_indices) - 1:
                    a = waypoints[nav_indices[cur_idx_in_nav]]
                    b = waypoints[nav_indices[cur_idx_in_nav + 1]]
                    lat1, lon1, alt1 = float(a.get('lat', 0)), float(a.get('lng', 0)), float(a.get('alt', 0))
                    lat2, lon2, alt2 = float(b.get('lat', 0)), float(b.get('lng', 0)), float(b.get('alt', 0))
                    dist = haversine_m(lat1, lon1, lat2, lon2)
                    steps = int(max(0, math.floor(dist / step_m)))

                    # Insert at each sub-distance: a small NAV WP + a DO_SET_SERVO toggle
                    for s in range(1, steps + 1):
                        t = s / (steps + 1)
                        lat = lerp(lat1, lat2, t)
                        lon = lerp(lon1, lon2, t)
                        alt = lerp(alt1, alt2, t)

                        # go to subpoint
                        new_list.append(_mk_nav_wp(lat, lon, alt))
                        # toggle servo value at this subpoint
                        new_list.append(_mk_do_set_servo(servo, pwm_on if toggle_on else pwm_off))
                        toggle_on = not toggle_on

        return new_list

    # Fallback: unchanged
    return waypoints

def resolve_mav_command(command_value) -> int:
    """Return MAV_CMD integer for mission item."""
    if isinstance(command_value, (int, float)):
        return int(command_value)
    if isinstance(command_value, str):
        normalized = command_value.strip().upper()
        resolved = COMMAND_NAME_TO_ID.get(normalized)
        if resolved is not None:
            return resolved
        log_message(f"Unknown mission command '{command_value}', defaulting to NAV_WAYPOINT", "WARNING")
    return mavlink.MAV_CMD_NAV_WAYPOINT


def command_requires_nav_coordinates(command_id: int) -> bool:
    """True when mission item expects lat/lon/alt."""
    return command_id in {
        mavlink.MAV_CMD_NAV_WAYPOINT,
        mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT,
        mavlink.MAV_CMD_NAV_TAKEOFF,
        mavlink.MAV_CMD_NAV_LAND,
        mavlink.MAV_CMD_NAV_LOITER_TIME,
        mavlink.MAV_CMD_NAV_LOITER_TURNS,
        mavlink.MAV_CMD_NAV_LOITER_UNLIM,
    }


def get_mode_name(mode_num, vehicle_type='ROVER'):
    """Convert mode number to mode name based on vehicle type."""
    if vehicle_type == 'ROVER':
        return ROVER_MODES.get(mode_num, f'UNKNOWN({mode_num})')
    else:
        return COPTER_MODES.get(mode_num, f'UNKNOWN({mode_num})')


def get_mode_number(mode_name, vehicle_type='ROVER'):
    """Convert mode name to mode number based on vehicle type."""
    mode_map = ROVER_MODES if vehicle_type == 'ROVER' else COPTER_MODES
    for num, name in mode_map.items():
        if name == mode_name.upper():
            return num
    return None


def derive_signal_strength(last_heartbeat: float | None, rc_connected: bool) -> str:
    """Return a human readable link quality label for the UI."""
    if last_heartbeat is None:
        return 'No Link'
    age = time.time() - last_heartbeat
    if age <= 2:
        return 'Excellent' if rc_connected else 'Good'
    if age <= 5:
        return 'Fair'
    if age <= 10:
        return 'Weak'
    return 'Lost'


# ============================================================================
# COMMAND QUEUE & ACK HANDLING
# ============================================================================

def queue_pending_command(command_name: str, sid: str | None, message: str):
    now = time.time()
    while pending_command_queue and now - pending_command_queue[0]['timestamp'] > 30:
        pending_command_queue.popleft()
    entry = {
        'command': command_name,
        'sid': sid,
        'timestamp': now,
        'message': message,
    }
    pending_command_queue.append(entry)
    if sid:
        socketio.emit('command_response', {
            'status': 'pending',
            'command': command_name,
            'message': message,
        }, room=sid)


def _notify_ack_waiters(command_name: str, response: dict) -> None:
    """Notify any threads waiting for a COMMAND_ACK for this command."""
    to_notify: list[dict] = []
    with _ack_waiters_lock:
        waiters = _ack_waiters.get(command_name)
        if waiters:
            to_notify = waiters[:]
            _ack_waiters[command_name] = []
    for w in to_notify:
        try:
            w['response'] = response
            ev = w.get('event')
            if ev is not None:
                ev.set()
        except Exception:
            pass


def wait_for_ack(command_name: str, timeout: float = 20.0) -> dict:
    """Block until a COMMAND_ACK is received or timeout."""
    event = threading.Event()
    waiter = {'event': event, 'response': None}
    with _ack_waiters_lock:
        _ack_waiters.setdefault(command_name, []).append(waiter)
    ok = event.wait(timeout)
    if not ok:
        with _ack_waiters_lock:
            lst = _ack_waiters.get(command_name)
            if lst and waiter in lst:
                lst.remove(waiter)
        return {
            'status': 'error',
            'command': command_name,
            'message': f'Command timeout ({int(timeout)}s)'
        }
    return waiter.get('response') or {
        'status': 'error',
        'command': command_name,
        'message': 'ACK wait returned no response'
    }


def resolve_pending_command(command_name: str, response: dict):
    matched = None
    for entry in list(pending_command_queue):
        if entry['command'] == command_name:
            matched = entry
            pending_command_queue.remove(entry)
            break
    if matched and matched.get('sid'):
        socketio.emit('command_response', response, room=matched['sid'])
    else:
        socketio.emit('command_response', response)


# ============================================================================
# TELEMETRY EMISSION
# ============================================================================

def emit_rover_data_now(reason: str | None = None) -> None:
    """Emit current_state to all clients with throttling bookkeeping."""
    global last_emit_monotonic
    data = get_rover_data()
    try:
        socketio.emit('rover_data', data)
        if reason:
            log_message(f"rover_data emitted ({reason}) @ {time.strftime('%H:%M:%S')}", 'INFO')
    finally:
        last_emit_monotonic = time.monotonic()


def _emit_timer_cb():
    global _emit_timer
    with _emit_lock:
        _emit_timer = None
    emit_rover_data_now(reason='throttled')


def schedule_fast_emit():
    """Emit at most EMIT_MIN_INTERVAL; schedule deferred emit if needed."""
    global _emit_timer
    now = time.monotonic()
    elapsed = now - last_emit_monotonic
    if elapsed >= EMIT_MIN_INTERVAL:
        emit_rover_data_now(reason='realtime')
        return
    delay = EMIT_MIN_INTERVAL - elapsed
    with _emit_lock:
        if _emit_timer is None:
            try:
                _emit_timer = eventlet.spawn_after(delay, _emit_timer_cb)
            except Exception:
                emit_rover_data_now(reason='realtime')


def get_rover_data():
    """Return complete rover data structure with all required fields."""
    try:
        current_state.signal_strength = derive_signal_strength(
            current_state.last_heartbeat, current_state.rc_connected
        )
    except Exception:
        pass
    return current_state.to_dict()


# ============================================================================
# MAVLINK CONNECTION & MESSAGE PROCESSING
# ============================================================================

def connect_to_drone():
    """Establishes and maintains the connection to the drone."""
    global master, is_vehicle_connected
    while True:
        try:
            if not is_vehicle_connected:
                log_message(f"Attempting to connect to vehicle on: {CONNECTION_STRING}")
                master = get_mavlink_core()
                master.connect(CONNECTION_STRING, BAUD_RATE)
                log_message("Waiting for heartbeat...")
                master.wait_heartbeat(timeout=30)
                is_vehicle_connected = True
                log_message("Vehicle connected!", "SUCCESS")
                log_message(f"Vehicle type: {master.target_system}")
                try:
                    set_message_intervals()
                except Exception as e:
                    log_message(f"Failed to set message intervals: {e}", "WARNING")
                request_data_streams()
                log_message("Emitting 'CONNECTED_TO_ROVER' to ALL clients...")
                socketio.emit('connection_status', {'status': 'CONNECTED_TO_ROVER'})
        except Exception as e:
            if is_vehicle_connected:
                log_message("Lost connection to vehicle.", "ERROR")
                is_vehicle_connected = False
                master = None
                log_message("Emitting 'WAITING_FOR_ROVER' due to lost connection...")
                socketio.emit('connection_status', {
                    'status': 'WAITING_FOR_ROVER',
                    'message': str(e)
                })
            log_message(f"Connection error: {e}", "ERROR")
        socketio.sleep(5)


def request_data_streams():
    """Request data streams from the vehicle."""
    if not master:
        return

    def _env_rate(name: str, default: float) -> int:
        try:
            val = float(os.getenv(name, str(default)))
            return int(max(0, val))
        except Exception:
            return int(default)

    streams = [
        (mavlink.MAV_DATA_STREAM_POSITION, _env_rate('STREAM_POSITION_HZ', 20)),
        (mavlink.MAV_DATA_STREAM_EXTRA1, _env_rate('STREAM_EXTRA1_HZ', 5)),
        (mavlink.MAV_DATA_STREAM_EXTRA2, _env_rate('STREAM_EXTRA2_HZ', 5)),
        (mavlink.MAV_DATA_STREAM_RC_CHANNELS, _env_rate('STREAM_RC_HZ', 5)),
        (mavlink.MAV_DATA_STREAM_RAW_SENSORS, _env_rate('STREAM_RAW_SENSORS_HZ', 5))
    ]
    for stream_id, rate in streams:
        try:
            master.mav.request_data_stream_send(
                master.target_system,
                master.target_component,
                stream_id,
                rate,
                1
            )
        except Exception as e:
            log_message(f"Error requesting data stream {stream_id}: {e}", "ERROR")


def set_message_intervals():
    """Configure MAVLink message intervals for more reliable telemetry."""
    if not master:
        return

    def send_interval(msg_id: int, hz: float):
        try:
            interval_us = int(1_000_000 / hz) if hz > 0 else -1
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                msg_id,
                interval_us,
                0, 0, 0, 0, 0
            )
        except Exception as e:
            log_message(f"Failed to set interval for msg {msg_id}: {e}", "WARNING")

    def _hz(name: str, default: float) -> float:
        try:
            return float(os.getenv(name, str(default)))
        except Exception:
            return default

    send_interval(mavlink.MAVLINK_MSG_ID_HEARTBEAT, _hz('MAV_HEARTBEAT_HZ', 5))
    send_interval(mavlink.MAVLINK_MSG_ID_COMMAND_ACK, _hz('MAV_CMD_ACK_HZ', 10))
    send_interval(mavlink.MAVLINK_MSG_ID_SYS_STATUS, _hz('MAV_SYS_STATUS_HZ', 5))
    send_interval(mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, _hz('MAV_GPOS_HZ', 20))
    send_interval(mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, _hz('MAV_GPS_RAW_HZ', 5))


def process_mavlink_messages():
    """Process incoming MAVLink messages and update vehicle data."""
    global current_state

    while True:
        if not is_vehicle_connected or not master:
            socketio.sleep(0.1)
            continue

        try:
            with mavlink_io_lock:
                msg = master.recv_match(blocking=False, timeout=0.1)
            if msg is None:
                continue

            msg_type = msg.get_type()

            if msg_type == 'HEARTBEAT':
                old_status = current_state.status
                old_mode = current_state.mode
                current_state.last_heartbeat = time.time()
                current_state.status = 'armed' if (
                    msg.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                ) else 'disarmed'
                current_state.mode = get_mode_name(msg.custom_mode)
                if current_state.status != old_status or current_state.mode != old_mode:
                    try:
                        current_state.last_update = time.time()
                    except Exception:
                        pass
                    log_message(
                        f"MODE/STATUS CHANGED: {old_mode}->{current_state.mode}, "
                        f"{old_status}->{current_state.status}",
                        "SUCCESS"
                    )
                    try:
                        emit_rover_data_now(reason='mode_change')
                    except Exception as e:
                        log_message(f"Failed to emit immediate update: {e}", "ERROR")
                schedule_fast_emit()

            elif msg_type == 'GLOBAL_POSITION_INT':
                if msg.lat != 0 and msg.lon != 0:
                    prev = current_state.position
                    new_lat = msg.lat / 1e7
                    new_lng = msg.lon / 1e7
                    pos_changed = (
                        prev is None or
                        prev.lat != new_lat or
                        prev.lng != new_lng
                    )
                    current_state.position = Position(lat=new_lat, lng=new_lng)
                    if pos_changed:
                        try:
                            current_state.last_update = time.time()
                        except Exception:
                            pass
                current_state.heading = (
                    (msg.hdg / 100.0) if getattr(msg, 'hdg', 65535) != 65535 else 0
                )
                schedule_fast_emit()

            elif msg_type == 'BATTERY_STATUS':
                if msg.battery_remaining != -1:
                    current_state.battery = msg.battery_remaining
                    schedule_fast_emit()

            elif msg_type == 'SYS_STATUS':
                if msg.battery_remaining != -1:
                    current_state.battery = msg.battery_remaining
                    schedule_fast_emit()

            elif msg_type == 'GPS_RAW_INT':
                fix_types = {
                    0: 'No GPS',
                    1: 'No Fix',
                    2: 'GPS 2D Fix',
                    3: 'GPS 3D Fix',
                    4: 'GPS DGPS',
                    5: 'RTK Float',
                    6: 'RTK Fixed'
                }
                current_state.rtk_status = fix_types.get(msg.fix_type, 'Unknown')
                if hasattr(msg, 'eph') and hasattr(msg, 'epv'):
                    current_state.hrms = (
                        f"{msg.eph / 100.0:.3f}" if msg.eph != 65535 else '0.000'
                    )
                    current_state.vrms = (
                        f"{msg.epv / 100.0:.3f}" if msg.epv != 65535 else '0.000'
                    )
                schedule_fast_emit()

            elif msg_type == 'RC_CHANNELS':
                current_state.rc_connected = msg.chan3_raw > 0
                schedule_fast_emit()

            elif msg_type == 'MISSION_CURRENT':
                try:
                    current_seq = int(getattr(msg, 'seq', -1))
                except Exception:
                    current_seq = -1

                prev_seq = current_state.activeWaypointIndex
                current_state.current_waypoint_id = current_seq
                current_state.activeWaypointIndex = current_seq

                mission_length = len(current_mission) if current_mission else 0

                if current_seq <= 0:
                    # Either mission just started or we wrapped back to the beginning.
                    current_state.completedWaypointIds = []
                    mission_log_state['last_active_seq'] = None
                    mission_log_state['last_reached_seq'] = None
                elif prev_seq is None:
                    # We don't know prior progress; assume everything before the active item is completed.
                    upper = current_seq if mission_length == 0 else min(current_seq, mission_length)
                    current_state.completedWaypointIds = list(range(1, upper + 1))
                    if upper > 0:
                        mission_log_state['last_reached_seq'] = upper - 1
                        for seq in range(upper):
                            completed_id = seq + 1
                            _record_mission_event(
                                f"Waypoint {completed_id} completed",
                                status='COMPLETED',
                                waypoint_id=completed_id
                            )
                elif current_seq > prev_seq:
                    # Advanced forward; mark everything between the previous and current index as completed.
                    for seq in range(prev_seq, current_seq):
                        completed_id = seq + 1
                        if mission_length and completed_id > mission_length:
                            break
                        if completed_id not in current_state.completedWaypointIds:
                            current_state.completedWaypointIds.append(completed_id)
                            _record_mission_event(
                                f"Waypoint {completed_id} completed",
                                status='COMPLETED',
                                waypoint_id=completed_id
                            )
                elif current_seq < prev_seq:
                    # We moved backwards (skip/RTL). Drop completed IDs beyond the new active item.
                    cutoff = current_seq
                    current_state.completedWaypointIds = [
                        wp_id for wp_id in current_state.completedWaypointIds
                        if wp_id <= cutoff
                    ]

                if current_seq >= 0 and mission_log_state['last_active_seq'] != current_seq:
                    mission_log_state['last_active_seq'] = current_seq
                    _record_mission_event(
                        f"Waypoint {current_seq + 1} active",
                        status='ACTIVE',
                        waypoint_id=current_seq + 1
                    )

                schedule_fast_emit()

            elif msg_type == 'MISSION_ITEM_REACHED':
                try:
                    reached_seq = int(getattr(msg, 'seq', -1))
                except Exception:
                    reached_seq = -1
                if reached_seq >= 0 and mission_log_state['last_reached_seq'] != reached_seq:
                    mission_log_state['last_reached_seq'] = reached_seq
                    _record_mission_event(
                        f"Waypoint {reached_seq + 1} reached",
                        status='REACHED',
                        waypoint_id=reached_seq + 1
                    )
                schedule_fast_emit()

            elif msg_type == 'AHRS2':
                if hasattr(msg, 'roll') and hasattr(msg, 'pitch'):
                    current_state.imu_status = (
                        'ALIGNED' if abs(msg.roll) < 0.1 and abs(msg.pitch) < 0.1
                        else 'ALIGNING'
                    )
                    schedule_fast_emit()

            elif msg_type == 'COMMAND_ACK':
                command_name = COMMAND_ID_TO_NAME.get(
                    msg.command, f'COMMAND_{msg.command}'
                )
                result_status, result_message = COMMAND_RESULT_MAP.get(
                    msg.result,
                    ('info', f'Result code {msg.result}')
                )
                response = {
                    'status': 'success' if result_status == 'success' else (
                        'error' if result_status == 'error' else 'pending'
                    ),
                    'command': command_name,
                    'message': result_message,
                    'result': msg.result,
                }
                if command_name == 'DO_SET_SERVO':
                    servo_state = 'SUCCESS' if response['status'] == 'success' else response['status'].upper()
                    _record_mission_event(
                        f"Servo command {result_message}",
                        status='SERVO',
                        servo_action=servo_state
                    )
                pending_match = next(
                    (entry for entry in pending_command_queue
                     if entry['command'] == command_name),
                    None
                )
                try:
                    _notify_ack_waiters(command_name, response)
                except Exception:
                    pass
                if pending_match:
                    if result_status == 'pending':
                        socketio.emit('command_response', response,
                                      room=pending_match.get('sid'))
                        pending_match['timestamp'] = time.time()
                    else:
                        resolve_pending_command(command_name, response)
                        try:
                            socketio.sleep(0.05)
                        except Exception:
                            pass
                        try:
                            emit_rover_data_now(reason='command_ack')
                        except Exception:
                            pass
                else:
                    socketio.emit('vehicle_status_text', {
                        'severity': 2 if response['status'] == 'success' else 4,
                        'text': f"{command_name}: {result_message}",
                    })

            elif msg_type == 'STATUSTEXT':
                text = (
                    msg.text.decode('utf-8', errors='ignore')
                    if isinstance(msg.text, bytes) else msg.text
                )
                severity = getattr(msg, 'severity', 6)
                log_message(
                    f"STATUSTEXT ({severity}): {text}",
                    'INFO' if severity < 4 else 'ERROR'
                )
                socketio.emit('vehicle_status_text', {
                    'severity': severity,
                    'text': text,
                })

        except Exception as e:
            log_message(f"Error processing message: {e}", "ERROR")

        socketio.sleep(0.01)


last_sent_telemetry = {}


def telemetry_loop():
    """Continuously sends telemetry data to the frontend."""
    global last_sent_telemetry, last_emit_monotonic
    fallback_interval = float(os.getenv('FALLBACK_EMIT_SEC', '60.0'))
    while True:
        if is_vehicle_connected:
            now = time.monotonic()
            if now - last_emit_monotonic >= fallback_interval:
                emit_rover_data_now(reason='fallback')
                last_sent_telemetry = get_rover_data()
        else:
            last_sent_telemetry = {}
        socketio.sleep(0.5)


# ============================================================================
# COMMAND HANDLERS
# ============================================================================

def _handle_arm_disarm(data):
    """Handle arm/disarm command."""
    if not master:
        raise Exception("No connection to vehicle")
    arm = data.get('arm', False)
    log_message(f"{'Arming' if arm else 'Disarming'} vehicle...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1 if arm else 0,
        0, 0, 0, 0, 0, 0
    )
    try:
        current_state.status = 'armed' if arm else 'disarmed'
        current_state.last_update = time.time()
        emit_rover_data_now(reason='arm_disarm')
    except Exception:
        pass
    return f"Arm command {'sent' if arm else 'cancel sent'}"


def _handle_set_mode(data):
    """Handle mode change command."""
    if not master:
        raise Exception("No connection to vehicle")
    new_mode = data.get('mode', 'MANUAL')
    mode_num = get_mode_number(new_mode)
    if mode_num is None:
        raise Exception(f"Unknown mode: {new_mode}")
    log_message(f"Setting vehicle mode to: {new_mode} (mode #{mode_num})")
    master.mav.set_mode_send(
        master.target_system,
        mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_num
    )
    try:
        current_state.mode = new_mode.upper()
        current_state.last_update = time.time()
        emit_rover_data_now(reason='set_mode')
    except Exception:
        pass
    return f"Mode change requested: {new_mode}"


def _handle_goto(data):
    """Handle goto command."""
    if not master:
        raise Exception("No connection to vehicle")
    lat = data['lat']
    lon = data['lon']
    alt = data.get('alt', 0)
    log_message(f"Sending vehicle to: {lat}, {lon}, alt: {alt}m")
    master.mav.set_position_target_global_int_send(
        0,
        master.target_system,
        master.target_component,
        mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,
        int(lat * 1e7),
        int(lon * 1e7),
        alt,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )
    return {
        'status': 'success',
        'message': f"GOTO command sent to {lat}, {lon}"
    }


def _handle_upload_mission(data):
    """Upload a mission via Socket.IO to the Pixhawk."""
    global current_mission
    if master is None:
        raise RuntimeError("Vehicle not connected")
    waypoints = data.get("waypoints")
    if not isinstance(waypoints, list) or not waypoints:
        raise ValueError("Mission upload requires waypoints")
    mission_count = len(waypoints)
    log_message(f"Mission upload initiated for {mission_count} waypoint(s)")

    # Flush stale messages
    flushed = 0
    while True:
        with mavlink_io_lock:
            pending_msg = master.recv_match(blocking=False)
        if pending_msg is None:
            break
        flushed += 1
    if flushed:
        log_message(f"Cleared {flushed} stale MAVLink messages", "DEBUG")

    overall_deadline = time.time() + max(20.0, mission_count * 3.0)

    def remaining_time() -> float:
        return max(0.0, overall_deadline - time.time())

    def wait_for_message(expected_types, description):
        timeout = remaining_time()
        if timeout <= 0:
            raise TimeoutError(f"Timeout waiting for {description}")
        with mavlink_io_lock:
            msg = master.recv_match(type=expected_types, blocking=True, timeout=timeout)
        if msg is None:
            raise TimeoutError(f"Timeout waiting for {description}")
        return msg

    try:
        log_message("Clearing existing mission on vehicle")
        master.mav.mission_clear_all_send(master.target_system, master.target_component)
        with mavlink_io_lock:
            ack = master.recv_match(type="MISSION_ACK", blocking=True, timeout=2.0)
        if ack is not None:
            ack_type = getattr(ack, "type", None)
            if ack_type != mavlink.MAV_MISSION_ACCEPTED:
                raise RuntimeError(f"Mission clear rejected (code={ack_type})")
            log_message("Vehicle acknowledged mission clear")

        log_message(f"Sending mission count ({mission_count})")
        master.mav.mission_count_send(master.target_system, master.target_component, mission_count)

        for seq, waypoint in enumerate(waypoints):
            request = wait_for_message(
                ("MISSION_REQUEST", "MISSION_REQUEST_INT"),
                f"MISSION_REQUEST for waypoint {seq}"
            )
            request_type = request.get_type()
            requested_seq = getattr(request, "seq", None)
            if requested_seq != seq:
                raise RuntimeError(f"Vehicle requested {requested_seq}, expected {seq}")

            command_id = resolve_mav_command(waypoint.get("command"))
            frame = int(safe_float(
                waypoint.get("frame", mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT),
                mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
            ))
            autocontinue = int(safe_float(waypoint.get("autocontinue", 1), 1))
            params = [
                safe_float(waypoint.get("param1", 0.0)),
                safe_float(waypoint.get("param2", 0.0)),
                safe_float(waypoint.get("param3", 0.0)),
                safe_float(waypoint.get("param4", 0.0)),
            ]

            requires_nav = command_requires_nav_coordinates(command_id)

            if requires_nav:
                try:
                    lat = float(waypoint["lat"])
                    lon = float(waypoint["lng"])
                except (KeyError, TypeError, ValueError) as exc:
                    raise ValueError(f"Waypoint {seq} missing lat/lng") from exc
                alt = float(waypoint.get("alt", 0.0))
                float_x, float_y, float_z = lat, lon, alt
                int_x, int_y = int(lat * 1e7), int(lon * 1e7)
            else:
                float_x = safe_float(waypoint.get("x", 0.0))
                float_y = safe_float(waypoint.get("y", 0.0))
                float_z = safe_float(waypoint.get("alt", waypoint.get("z", 0.0)))
                int_x, int_y = int(float_x * 1e7), int(float_y * 1e7)

            if request_type == "MISSION_REQUEST_INT":
                master.mav.mission_item_int_send(
                    master.target_system, master.target_component,
                    seq, frame, command_id, 0, autocontinue,
                    *params, int_x, int_y, float_z
                )
            else:
                master.mav.mission_item_send(
                    master.target_system, master.target_component,
                    seq, frame, command_id, 0, autocontinue,
                    *params, float_x, float_y, float_z
                )

        ack = wait_for_message("MISSION_ACK", "MISSION_ACK")
        ack_result = getattr(ack, "type", None)
        if ack_result != mavlink.MAV_MISSION_ACCEPTED:
            raise RuntimeError(f"Mission rejected (code={ack_result})")

        current_mission = list(waypoints)
        log_message(f"Mission uploaded successfully ({mission_count} waypoints)", "SUCCESS")
        return {
            "status": "success",
            "message": f"Mission uploaded ({mission_count} waypoints)"
        }
    except TimeoutError as exc:
        log_message(f"Mission upload timeout: {exc}", "ERROR")
        raise
    except Exception as exc:
        log_message(f"Mission upload failed: {exc}", "ERROR")
        raise


def _handle_get_mission(data):
    """Handle GET_MISSION command."""
    global current_mission
    if not master:
        raise Exception("No connection to vehicle")
    try:
        mission_waypoints = download_mission_from_vehicle()
        return {
            'status': 'success',
            'message': f'Downloaded {len(mission_waypoints)} waypoints',
            'waypoints': mission_waypoints
        }
    except Exception as e:
        log_message(f"Mission download failed: {e}", "WARNING")
        return {
            'status': 'success',
            'message': f'Using cached mission: {len(current_mission)} waypoints',
            'waypoints': current_mission
        }


COMMAND_HANDLERS = {
    'ARM_DISARM': _handle_arm_disarm,
    'SET_MODE': _handle_set_mode,
    'GOTO': _handle_goto,
    'UPLOAD_MISSION': _handle_upload_mission,
    'GET_MISSION': _handle_get_mission
}


@socketio.on('request_mission_logs')
def handle_request_mission_logs():
    try:
        snapshot = list(mission_log_history)
        emit('mission_logs_snapshot', snapshot, room=request.sid)
    except Exception as exc:
        log_message(f"Failed to serve mission logs snapshot: {exc}", 'ERROR')


# ============================================================================
# MISSION UPLOAD/DOWNLOAD
# ============================================================================

@socketio.on("mission_upload")
def on_mission_upload(data):
    """Mission upload via Socket.IO with progress updates."""
    global master, current_mission

    try:
        if master is None:
            emit("mission_uploaded", {"ok": False, "error": "Vehicle not connected"})
            return

        waypoints = data.get("waypoints", [])
        servo_config = data.get("servoConfig")

        if not waypoints:
            emit("mission_uploaded", {"ok": False, "error": "No waypoints provided"})
            return

        if servo_config:
            log_message(f"[mission_upload] Applying servo mode: {servo_config.get('mode')}")
            waypoints = apply_servo_modes(waypoints, servo_config)

        mission_count = len(waypoints)
        log_message(f"[mission_upload] Uploading {mission_count} waypoints")

        if not mission_upload_lock.acquire(blocking=False):
            emit("mission_uploaded", {"ok": False, "error": "Upload in progress"})
            return

        try:
            # Flush stale messages
            flushed = 0
            while True:
                with mavlink_io_lock:
                    pending = master.recv_match(blocking=False)
                if pending is None:
                    break
                flushed += 1
            if flushed:
                log_message(f"[mission_upload] Flushed {flushed} messages", "DEBUG")

            # Clear old mission
            master.mav.mission_clear_all_send(master.target_system, master.target_component)
            with mavlink_io_lock:
                ack = master.recv_match(type="MISSION_ACK", blocking=True, timeout=2.0)
            if ack is not None:
                log_message(f"[mission_upload] Clear ACK: {getattr(ack, 'type', None)}", "DEBUG")

            def send_count():
                master.mav.mission_count_send(master.target_system, master.target_component, mission_count)
                socketio.emit('mission_upload_progress', {'progress': 0})

            send_count()
            uploaded_set: set[int] = set()
            deadline = time.time() + max(20.0, mission_count * 3.0)
            last_activity = time.time()

            while True:
                remaining = max(0.0, deadline - time.time())
                if remaining <= 0:
                    raise RuntimeError("Timeout waiting for mission request/ack")

                if time.time() - last_activity > 2.5:
                    log_message("[mission_upload] Re-sending count", "WARNING")
                    send_count()
                    last_activity = time.time()

                with mavlink_io_lock:
                    msg = master.recv_match(
                        type=["MISSION_REQUEST", "MISSION_REQUEST_INT", "MISSION_ACK"],
                        blocking=True,
                        timeout=min(2.5, remaining)
                    )
                if not msg:
                    continue

                last_activity = time.time()
                mtype = msg.get_type()

                if mtype in ("MISSION_REQUEST", "MISSION_REQUEST_INT"):
                    seq = int(getattr(msg, "seq", 0))
                    if seq < 0 or seq >= mission_count:
                        raise RuntimeError(f"Invalid seq {seq}")

                    wp = waypoints[seq]
                    cmd_id = resolve_mav_command(wp.get("command"))
                    frame = int(safe_float(
                        wp.get("frame", mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT),
                        mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
                    ))
                    p1 = safe_float(wp.get("param1", 0.0))
                    p2 = safe_float(wp.get("param2", 0.0))
                    p3 = safe_float(wp.get("param3", 0.0))
                    p4 = safe_float(wp.get("param4", 0.0))

                    if command_requires_nav_coordinates(cmd_id):
                        lat = safe_float(wp.get("lat", 0.0))
                        lon = safe_float(wp.get("lng", wp.get("lon", 0.0)))
                        alt = safe_float(wp.get("alt", 0.0))
                        if mtype == "MISSION_REQUEST_INT":
                            master.mav.mission_item_int_send(
                                master.target_system, master.target_component,
                                seq, frame, cmd_id, 0, 1,
                                p1, p2, p3, p4,
                                int(lat * 1e7), int(lon * 1e7), alt
                            )
                        else:
                            master.mav.mission_item_send(
                                master.target_system, master.target_component,
                                seq, frame, cmd_id, 0, 1,
                                p1, p2, p3, p4,
                                lat, lon, alt
                            )
                    else:
                        alt = safe_float(wp.get("alt", wp.get("z", 0.0)))
                        if mtype == "MISSION_REQUEST_INT":
                            master.mav.mission_item_int_send(
                                master.target_system, master.target_component,
                                seq, frame, cmd_id, 0, 1,
                                p1, p2, p3, p4,
                                0, 0, alt
                            )
                        else:
                            master.mav.mission_item_send(
                                master.target_system, master.target_component,
                                seq, frame, cmd_id, 0, 1,
                                p1, p2, p3, p4,
                                0.0, 0.0, alt
                            )

                    if seq not in uploaded_set:
                        uploaded_set.add(seq)
                        progress = int(round(100.0 * len(uploaded_set) / mission_count))
                        socketio.emit('mission_upload_progress', {'progress': progress})

                elif mtype == "MISSION_ACK":
                    ack_type = getattr(msg, "type", None)
                    log_message(f"[mission_upload] ACK received (code={ack_type})")
                    if ack_type != mavlink.MAV_MISSION_ACCEPTED:
                        emit("mission_uploaded", {
                            "ok": False,
                            "error": f"ACK code {ack_type}",
                            "count": len(uploaded_set)
                        })
                        return

                    current_mission = list(waypoints)
                    current_state.completedWaypointIds = []
                    current_state.activeWaypointIndex = None
                    current_state.current_waypoint_id = None
                    mission_log_state['last_active_seq'] = None
                    mission_log_state['last_reached_seq'] = None
                    _record_mission_event(
                        f"Mission uploaded ({mission_count} items)",
                        status='MISSION_UPLOADED'
                    )
                    socketio.emit('mission_upload_progress', {'progress': 100})
                    emit("mission_uploaded", {"ok": True, "count": len(uploaded_set)})
                    return
        finally:
            try:
                mission_upload_lock.release()
            except Exception:
                pass

    except Exception as e:
        log_message(f"[mission_upload] Error: {e}", "ERROR")
        emit("mission_uploaded", {"ok": False, "error": str(e)})


def download_mission_from_vehicle():
    """Download mission from vehicle."""
    global current_mission
    if not master:
        raise Exception("No connection to vehicle")
    if not mission_download_lock.acquire(blocking=False):
        raise Exception("Another download in progress")

    try:
        # Flush stale messages
        flushed = 0
        while True:
            with mavlink_io_lock:
                pending = master.recv_match(blocking=False)
            if pending is None:
                break
            flushed += 1
        if flushed:
            log_message(f"[mission_download] Flushed {flushed} messages", "DEBUG")

        log_message("[mission_download] Requesting mission list...")
        master.mav.mission_request_list_send(master.target_system, master.target_component)

        with mavlink_io_lock:
            msg = master.recv_match(type='MISSION_COUNT', blocking=True, timeout=8.0)
        if not msg:
            master.mav.mission_request_list_send(master.target_system, master.target_component)
            with mavlink_io_lock:
                msg = master.recv_match(type='MISSION_COUNT', blocking=True, timeout=8.0)
        if not msg:
            raise Exception("Failed to receive mission count")

        mission_count = int(getattr(msg, 'count', 0))
        log_message(f"[mission_download] Vehicle reports {mission_count} waypoints")

        if mission_count <= 0:
            current_mission = []
            socketio.emit('mission_download_progress', {'progress': 100})
            return []

        downloaded: list[dict] = [None] * mission_count  # type: ignore
        received_set: set[int] = set()
        deadline = time.time() + max(20.0, mission_count * 3.0)

        def emit_progress():
            try:
                pct = int(round(100.0 * len(received_set) / mission_count))
                socketio.emit('mission_download_progress', {'progress': pct})
            except Exception:
                pass

        def request_seq(seq: int, prefer_int: bool = True) -> None:
            if prefer_int:
                master.mav.mission_request_int_send(
                    master.target_system, master.target_component, seq
                )
            else:
                master.mav.mission_request_send(
                    master.target_system, master.target_component, seq
                )

        prefer_int = True
        retries_per_item = 3

        for seq in range(mission_count):
            attempt = 0
            while attempt < retries_per_item and seq not in received_set:
                attempt += 1
                request_seq(seq, prefer_int=prefer_int)
                with mavlink_io_lock:
                    msg = master.recv_match(
                        type=['MISSION_ITEM_INT', 'MISSION_ITEM'],
                        blocking=True,
                        timeout=3.0
                    )
                if not msg:
                    if attempt == 2 and prefer_int:
                        prefer_int = False
                    continue

                mtype = msg.get_type()
                if mtype not in ('MISSION_ITEM_INT', 'MISSION_ITEM'):
                    continue

                msg_seq = int(getattr(msg, 'seq', -1))
                if msg_seq != seq:
                    continue

                if mtype == 'MISSION_ITEM_INT':
                    lat = getattr(msg, 'x', 0) / 1e7
                    lon = getattr(msg, 'y', 0) / 1e7
                else:
                    lat = getattr(msg, 'x', 0.0)
                    lon = getattr(msg, 'y', 0.0)

                cmd = int(getattr(msg, 'command', 0))
                waypoint = {
                    'id': msg_seq + 1,
                    'command': 'WAYPOINT' if cmd == mavlink.MAV_CMD_NAV_WAYPOINT else 'DO_SET_SERVO',
                    'lat': lat,
                    'lng': lon,
                    'alt': float(getattr(msg, 'z', 0.0)),
                    'frame': int(getattr(msg, 'frame', mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT)),
                    'param1': float(getattr(msg, 'param1', 0.0)),
                    'param2': float(getattr(msg, 'param2', 0.0)),
                    'param3': float(getattr(msg, 'param3', 0.0)),
                    'param4': float(getattr(msg, 'param4', 0.0)),
                }
                downloaded[msg_seq] = waypoint
                received_set.add(msg_seq)
                log_message(f"[mission_download] Received {msg_seq+1}/{mission_count}")
                emit_progress()
                deadline = time.time() + max(10.0, (mission_count - msg_seq) * 2.0)

            if seq not in received_set:
                raise Exception(f"Timeout receiving item {seq}")

        result = [wp for wp in downloaded if wp is not None]
        current_mission = result.copy()
        socketio.emit('mission_download_progress', {'progress': 100})
        log_message(f"[mission_download] Completed: {len(result)} waypoints", "SUCCESS")
        return result

    except Exception as e:
        log_message(f"Error downloading mission: {e}", "ERROR")
        raise e
    finally:
        try:
            mission_download_lock.release()
        except Exception:
            pass


# ============================================================================
# RTK INJECTION SYSTEM (Enhanced with Debug Headers)
# ============================================================================

rtk_thread: Optional[threading.Thread] = None
rtk_running = False
rtk_bytes_total = 0
rtk_bytes_lock = threading.Lock()
rtk_current_caster = None


# Replace the existing _ntrip_request() function with:
def _ntrip_request(host: str, mount: str, user: str, pwd: str) -> bytes:
    """Builds an exact NTRIP 2.0 HTTP/1.0 request identical to manual telnet test."""
    auth = base64.b64encode(f"{user}:{pwd}".encode()).decode()
    req = (
        f"GET /{mount} HTTP/1.0\r\n"
        f"User-Agent: NTRIP_RoverPlan/1.0\r\n"
        f"Ntrip-Version: Ntrip/2.0\r\n"
        f"Authorization: Basic {auth}\r\n"
        f"\r\n"
    )
    return req.encode("ascii", errors="ignore")


def _inject_rtcm_to_mav(rtcm_bytes: bytes) -> bool:
    """Send RTCM correction data to rover's GPS module."""
    if not master:
        log_message("[RTK] ⚠️ Cannot send - rover not connected!", "WARNING")
        return False

    MAX_CHUNK = 180
    try:
        total_sent = 0
        for i in range(0, len(rtcm_bytes), MAX_CHUNK):
            chunk = rtcm_bytes[i:i + MAX_CHUNK]
            if chunk:
                # Create a 180-byte buffer and place the chunk at the start.
                data_payload = bytearray(180)
                data_payload[:len(chunk)] = chunk
                master.mav.gps_rtcm_data_send(0, len(chunk), data_payload)
                total_sent += len(chunk)

        if total_sent > 0:
            with rtk_bytes_lock:
                global rtk_bytes_total
                rtk_bytes_total += total_sent
                current_total = rtk_bytes_total
            # Tell frontend how many bytes forwarded
            socketio.emit('rtk_forwarded', {
                'added': total_sent,
                'total': current_total,
            })
        return True
    except Exception as e:
        log_message(f"[RTK] ❌ Failed to send data: {e}", "ERROR")
        socketio.emit('rtk_log', {'message': f'❌ Send error: {e}'})
        return False


@socketio.on('connect_caster')
def handle_connect_caster(caster_details):
    """Start RTK corrections stream from caster network."""
    global rtk_thread, rtk_running, rtk_current_caster

    try:
        sender_id = request.sid
    except:
        sender_id = None

    # --- IMPROVEMENT: Automatically stop the existing stream ---
    if rtk_running:
        log_message("[RTK] 🔄 Restart requested. Stopping existing stream first...", "INFO")
        socketio.emit('rtk_log', {'message': '🔄 Restarting stream...'})
        rtk_running = False
        # Give the old thread a moment to see the flag and exit
        socketio.sleep(0.2)
    # --- END IMPROVEMENT ---

    try:
        host = str(caster_details.get('host', '')).strip()
        port = int(caster_details.get('port', 2101))
        mount = str(caster_details.get('mountpoint', '')).strip()
        user = str(caster_details.get('user') or caster_details.get('username', '')).strip()
        pwd = str(caster_details.get('password', '')).strip()

        if not host or not mount:
            raise ValueError("Missing host or mountpoint")
        # Enforce credentials presence to prevent unauthenticated streams
        if not user or not pwd:
            raise ValueError("Missing username or password")

    except Exception as e:
        msg = f'❌ Invalid settings: {str(e)}'
        socketio.emit('rtk_log', {'message': msg})
        socketio.emit('caster_status', {
            'status': 'error',
            'message': msg
        }, to=sender_id)
        return

    # --- FIX: Check if rover is connected before starting RTK ---
    if not is_vehicle_connected or not master:
        msg = '❌ Rover not connected! Connect rover first.'
        log_message(f"[RTK] Rejecting stream start: {msg}", "WARNING")
        socketio.emit('rtk_log', {'message': msg})
        socketio.emit('caster_status', {
            'status': 'error',
            'message': msg
        }, to=sender_id)
        return
    # --- END FIX ---

    

    rtk_current_caster = caster_details
    log_message(f"[RTK] 🚀 Starting stream from {host}:{port}/{mount}", "INFO")
    socketio.emit('rtk_log', {'message': f'🔄 Connecting to {host}:{port}...'})
    socketio.emit('caster_status', {
        'status': 'connecting',
        'message': f'Connecting to {mount}...'
    })

    rtk_running = True

    def rtk_worker():
        """Background thread for continuous RTK data streaming."""
        global rtk_running, rtk_bytes_total
        sock = None

        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(10)

            log_message(f"[RTK] Attempting connection to {host}:{port}...")
            socketio.emit('rtk_log', {'message': f'📡 Connecting to {host}:{port}...'})
            sock.connect((host, port))
            log_message(f"[RTK] Connection to {host}:{port} successful!", "SUCCESS")
            socketio.emit('rtk_log', {'message': f'📡 Connected to {host}:{port}'})

            request_data = _ntrip_request(host, mount, user, pwd)
            sock.sendall(request_data)
            socketio.emit('rtk_log', {
                'message': f'📤 Sent NTRIP request for mountpoint {mount}'
            })

            # --- Wait for optional HTTP header or direct binary stream ---
            buffer = b''
            header_timeout = time.time() + 12.0
            header_found = False

            while True:
                chunk = sock.recv(1024)
                if not chunk:
                    raise RuntimeError('Caster closed connection during handshake')
                buffer += chunk

                if b'\r\n\r\n' in buffer:
                    headers, leftover_data = buffer.split(b'\r\n\r\n', 1)
                    header_text = headers.decode('ascii', errors='ignore')
                    first_line = header_text.split('\n')[0]
                    # Emit the full caster response header for visibility
                    socketio.emit('rtk_log', {'message': f'🧾 Caster Response:\n{header_text}'})
                    upper_hdr = header_text.upper()
                    # Explicitly detect auth failures and sourcetable responses
                    if ('SOURCETABLE' in upper_hdr or
                        '401' in upper_hdr or 'UNAUTHORIZED' in upper_hdr or
                        '403' in upper_hdr or 'FORBIDDEN' in upper_hdr or
                        'NTRIP-ERROR' in upper_hdr):
                        raise RuntimeError('❌ Unauthorized or bad username/password!')
                    # Accept either ICY 200 or standard 200 OK
                    if b'ICY 200' not in headers and b'200 OK' not in headers:
                        raise RuntimeError(f'❌ Unexpected response:\n{header_text}')
                    socketio.emit('rtk_log', {'message': '✅ Connection established! Streaming RTCM data...'})
                    leftover = leftover_data
                    header_found = True
                    break

                # direct binary
                if any(b < 32 and b not in (b'\r'[0], b'\n'[0], b'\t'[0]) for b in chunk):
                    socketio.emit('rtk_log', {'message': '✅ No header — direct RTCM stream detected'})
                    leftover = buffer
                    break

                if time.time() > header_timeout:
                    raise TimeoutError('Caster took too long to respond')

            if header_found:
                socketio.emit('caster_status', {
                    'status': 'Connected',
                    'message': 'ICY 200 OK – RTCM stream active'
                })
            else:
                socketio.emit('caster_status', {
                    'status': 'Connected',
                    'message': 'RTCM stream active (no header)'
                })

            with rtk_bytes_lock:
                rtk_bytes_total = 0

            # Inject leftover RTCM if present
            if 'leftover' in locals() and leftover:
                if _inject_rtcm_to_mav(leftover):
                    socketio.emit('rtk_log', {
                        'message': f'📥 Sent initial {len(leftover)} bytes'
                    })
                    # Debug: print RTCM header bytes to browser console
                    if leftover.startswith(b'\xd3'):
                        msg_len = int.from_bytes(leftover[1:3], "big") & 0x03FF
                        socketio.emit('rtk_log', {
                            'message': f'🔍 RTCM Detected: Header 0xD3, length {msg_len} bytes'
                        })

            sock.settimeout(3.0)
            last_data_time = time.time()

            while rtk_running:
                try:
                    data = sock.recv(4096)

                    if not data:
                        if time.time() - last_data_time > 10:
                            raise RuntimeError('No data received for 10 seconds')
                        time.sleep(0.1)
                        continue

                    last_data_time = time.time()

                    # Debug: parse RTCM header
                    if data.startswith(b'\xd3') and len(data) >= 3:
                        msg_len = int.from_bytes(data[1:3], "big") & 0x03FF
                        socketio.emit('rtk_log', {
                            'message': f'🔍 RTCM Packet: 0xD3 len={msg_len}'
                        })

                    if _inject_rtcm_to_mav(data):
                        socketio.emit('rtcm_data', data)

                except socket.timeout:
                    continue

                except Exception as stream_error:
                    log_message(f"[RTK] Stream error: {stream_error}", "ERROR")
                    socketio.emit('rtk_log', {
                        'message': f'⚠️ Stream error: {stream_error}'
                    })
                    break

        except Exception as e:
            error_msg = str(e)
            log_message(f"[RTK] ❌ Connection failed: {error_msg}", "ERROR")
            socketio.emit('rtk_log', {'message': f'❌ Error: {error_msg}'})
            socketio.emit('caster_status', {
                'status': 'error',
                'message': error_msg
            })

        finally:
            rtk_running = False
            if sock:
                try:
                    sock.close()
                except:
                    pass

            socketio.emit('rtk_log', {'message': '🛑 RTK stream stopped'})
            socketio.emit('caster_status', {
                'status': 'Disconnected',
                'message': 'Stream ended'
            })
            log_message("[RTK] Stream thread ended", "INFO")

    rtk_thread = threading.Thread(target=rtk_worker, daemon=True, name="RTK-Worker")
    rtk_thread.start()

    socketio.emit('rtk_log', {'message': '🎬 RTK thread started'})


@socketio.on('disconnect_caster')
def handle_disconnect_caster():
    """Stop the RTK stream."""
    global rtk_running

    try:
        sender_id = request.sid
    except:
        sender_id = None

    if rtk_running:
        log_message("[RTK] 🛑 Stop requested by user", "INFO")
        rtk_running = False
        socketio.emit('rtk_log', {'message': '🛑 Stopping RTK stream...'})
        socketio.emit('caster_status', {
            'status': 'Disconnected',
            'message': 'Stop requested'
        }, to=sender_id)
    else:
        socketio.emit('rtk_log', {'message': 'ℹ️ Stream was not running'})
        socketio.emit('caster_status', {
            'status': 'Disconnected',
            'message': 'Already stopped'
        }, to=sender_id)



# ============================================================================
# SOCKET.IO EVENT HANDLERS
# ============================================================================

@socketio.on('connect')
def handle_connect():
    """Handles a new client connection."""
    log_message('Client connected')
    if is_vehicle_connected:
        log_message("Emitting 'CONNECTED_TO_ROVER' to NEW client...")
        emit('connection_status', {'status': 'CONNECTED_TO_ROVER'})
        rover_data = get_rover_data()
        if rover_data:
            log_message("Sending current rover data to new client")
            emit('rover_data', rover_data)
    else:
        log_message("Emitting 'WAITING_FOR_ROVER' to NEW client...")
        emit('connection_status', {'status': 'WAITING_FOR_ROVER'})

    # Also inform new clients of current RTK caster status for seamless UX
    try:
        if rtk_running:
            caster = rtk_current_caster or {}
            host = str(caster.get('host', ''))
            port = str(caster.get('port', ''))
            mount = str(caster.get('mountpoint', ''))
            emit('caster_status', {
                'status': 'Connected',
                'message': f'Active stream: {host}:{port}/{mount}'
            })
        else:
            emit('caster_status', {
                'status': 'Disconnected',
                'message': 'No active RTK stream'
            })
    except Exception as e:
        log_message(f"Failed to emit caster status to new client: {e}", 'WARNING')


@socketio.on('disconnect')
def handle_disconnect():
    log_message('Client disconnected')


@socketio.on('request_rover_reconnect')
def handle_rover_reconnect():
    """Force the backend to drop and re-establish the vehicle link."""
    global master, is_vehicle_connected

    log_message('Frontend requested rover reconnect', 'INFO')
    emit('rover_reconnect_ack', {'status': 'success', 'message': 'Reconnect initiated'})

    with mavlink_io_lock:
        if master is not None:
            try:
                master.close()
            except Exception as exc:
                log_message(f'Error closing MAVLink connection: {exc}', 'WARNING')
        master = None

    is_vehicle_connected = False
    current_state.last_heartbeat = None
    current_state.signal_strength = 'No Link'
    log_message('Flagged vehicle connection as lost; will retry', 'INFO')
    socketio.emit('connection_status', {
        'status': 'WAITING_FOR_ROVER',
        'message': 'Reconnect requested by operator'
    })


@socketio.on('send_command')
def handle_command(data):
    """Handle incoming commands from the frontend."""
    if not is_vehicle_connected or not master:
        log_message("Command rejected - vehicle not connected", "ERROR")
        emit('command_response', {'status': 'error', 'message': 'Vehicle not connected'})
        return

    command_type = data.get('command')
    log_message(f"Received command: {command_type}")

    try:
        handler = COMMAND_HANDLERS.get(command_type)
        if handler:
            wait_for_ack_flag = bool(data.get('wait_for_ack')) and command_type in ('ARM_DISARM', 'SET_MODE')
            ack_timeout = float(data.get('ack_timeout', 20)) if wait_for_ack_flag else None

            if command_type in ('ARM_DISARM', 'SET_MODE') and not wait_for_ack_flag:
                queue_pending_command(command_type, request.sid, f"{command_type} dispatched")

            result = handler(data)

            if wait_for_ack_flag:
                try:
                    final = wait_for_ack(command_type, timeout=ack_timeout or 20.0)
                except Exception as e:
                    final = {'status': 'error', 'message': str(e), 'command': command_type}
                emit('command_response', final)
                return

            if isinstance(result, dict):
                emit('command_response', result)
            elif isinstance(result, str):
                log_message(result, 'INFO')
            else:
                log_message(f"Command {command_type} executed", 'INFO')
        else:
            log_message(f"Unknown command: {command_type}", "ERROR")
            emit('command_response', {
                'status': 'error',
                'message': f'Unknown command: {command_type}'
            })
    except Exception as e:
        log_message(f"Command error ({command_type}): {e}", "ERROR")
        if command_type in ('ARM_DISARM', 'SET_MODE'):
            resolve_pending_command(command_type, {
                'status': 'error',
                'message': f'Command failed: {str(e)}'
            })
        emit('command_response', {
            'status': 'error',
            'message': f'Command failed: {str(e)}'
        })


# ============================================================================
# ERROR HANDLERS
# ============================================================================

@socketio.on_error_default
def default_error_handler(e):
    log_message(f"SocketIO error: {e}", "ERROR")


@app.errorhandler(Exception)
def handle_exception(e):
    log_message(f"Flask error: {e}", "ERROR")
    return {'error': str(e)}, 500


# ============================================================================
# MAIN EXECUTION
# ============================================================================

# ============================================================================
# SERVO CONTROL API (integrated for concurrent backend)
# ============================================================================
# Simple process manager to launch/stop servo scripts living under Backend/servo_manager
import subprocess as _subprocess

SERVO_BASE = os.path.join(os.path.dirname(__file__), "servo_manager")
CONFIG_PATH = os.path.join(SERVO_BASE, "config.json")
LOG_DIR = os.path.join(SERVO_BASE, "logs")
os.makedirs(LOG_DIR, exist_ok=True)

_running: dict[str, dict] = {}

def _is_pid_alive(pid: int | None) -> bool:
    if not pid:
        return False
    try:
        return os.system(f"ps -p {int(pid)} > /dev/null 2>&1") == 0
    except Exception:
        return False

def _cleanup_running() -> None:
    for m in list(_running.keys()):
        pid = _running[m].get("pid")
        if not _is_pid_alive(pid):
            try:
                del _running[m]
            except Exception:
                pass

def _kill_mode(mode: str, timeout_sec: float = 3.0) -> None:
    info = _running.get(mode)
    if not info:
        return
    pid = info.get("pid")
    if pid:
        try:
            os.kill(pid, signal.SIGTERM)
        except Exception:
            pass
        t0 = time.time()
        while time.time() - t0 < timeout_sec:
            if not _is_pid_alive(pid):
                break
            time.sleep(0.1)
        if _is_pid_alive(pid):
            try:
                os.kill(pid, signal.SIGKILL)
            except Exception:
                pass
    try:
        del _running[mode]
    except Exception:
        pass

def _find_running_mode(except_mode: str | None = None) -> str | None:
    _cleanup_running()
    for m, info in _running.items():
        if except_mode and m == except_mode:
            if _is_pid_alive(info.get("pid")):
                return m
            continue
        if _is_pid_alive(info.get("pid")):
            return m
    return None


@app.get("/servo/run")
def servo_run():
    mode = request.args.get("mode")
    if not mode:
        return jsonify({"error": "missing mode"}), 400

    scripts = {
        "wpmark": "wp_mark.py",
        "continuous": "continuous_line.py",
        "interval": "interval_spray.py",
    }
    if mode not in scripts:
        return jsonify({"error": "invalid mode"}), 400

    # Enforce only one script running at a time
    replace = str(request.args.get("replace", "0")).lower() in ("1", "true", "yes")
    current = _find_running_mode()
    if current and current != mode and not replace:
        return jsonify({
            "error": "another mode is running",
            "current_mode": current,
            "requested_mode": mode,
            "action": "pass replace=1 to switch"
        }), 409
    if current and current != mode and replace:
        _kill_mode(current)

    # If same mode already running, return its info
    info = _running.get(mode)
    if info and _is_pid_alive(info.get("pid")):
        return jsonify({
            "status": f"{mode} already running",
            "pid": info.get("pid"),
            "log": info.get("log"),
            "start": info.get("start")
        })

    log_path = os.path.join(LOG_DIR, f"{mode}_{int(time.time())}.log")
    log_file = open(log_path, "a")

    p = _subprocess.Popen(
        ["python3", os.path.join(SERVO_BASE, scripts[mode])],
        stdout=log_file, stderr=_subprocess.STDOUT
    )
    _running[mode] = {"pid": p.pid, "log": log_path, "start": time.time()}
    return jsonify({"status": f"{mode} started", "pid": p.pid, "log": log_path})


@app.get("/servo/stop")
def servo_stop():
    mode = request.args.get("mode")
    if not mode or mode not in _running:
        return jsonify({"error": "not running"}), 400
    try:
        _kill_mode(mode)
    except Exception as e:
        return jsonify({"error": str(e)}), 500
    return jsonify({"status": f"{mode} stopped"})


@app.get("/servo/status")
def servo_status():
    _cleanup_running()
    for m, info in list(_running.items()):
        alive = _is_pid_alive(info.get("pid"))
        info["running"] = bool(alive)
    return jsonify(_running)


@app.get("/servo/edit")
def servo_edit():
    data = dict(request.args)
    try:
        with open(CONFIG_PATH, "r") as f:
            cfg = json.load(f)
    except Exception:
        cfg = {}
    for k, v in data.items():
        # e.g. "wp_mark.delay_before_on"=2.0
        keys = k.split(".")
        ref = cfg
        for part in keys[:-1]:
            ref = ref.setdefault(part, {})
        try:
            val = float(v) if "." in v else int(v)
        except Exception:
            val = v
        ref[keys[-1]] = val
    with open(CONFIG_PATH, "w") as f:
        json.dump(cfg, f, indent=2)
    return jsonify({"status": "updated", "config": cfg})


@app.post("/servo/edit")
def servo_edit_post():
    """Accept JSON body to update servo config (more robust than querystring)."""
    body = request.get_json(silent=True) or {}
    try:
        with open(CONFIG_PATH, "r") as f:
            cfg = json.load(f)
    except Exception:
        cfg = {}

    def _coerce(s):
        try:
            if isinstance(s, (int, float)):
                return s
            if isinstance(s, str):
                if s.strip() == "":
                    return s
                if "." in s:
                    return float(s)
                return int(s)
        except Exception:
            return s
        return s

    for k, v in body.items():
        keys = str(k).split(".")
        ref = cfg
        for part in keys[:-1]:
            ref = ref.setdefault(part, {})
        ref[keys[-1]] = _coerce(v)

    with open(CONFIG_PATH, "w") as f:
        json.dump(cfg, f, indent=2)
    return jsonify({"status": "updated", "config": cfg})


@app.get("/servo/log")
def servo_log():
    path = request.args.get("path")
    if not path:
        return jsonify({"error": "invalid log path"}), 400
    try:
        # Prevent directory traversal: require logs under LOG_DIR
        abs_path = os.path.abspath(path)
        if not abs_path.startswith(os.path.abspath(LOG_DIR)):
            return jsonify({"error": "forbidden path"}), 403
        if not os.path.isfile(abs_path):
            return jsonify({"error": "invalid log path"}), 400
        with open(abs_path, "r") as f:
            lines = f.readlines()[-200:]
        return jsonify({"log": "".join(lines)})
    except Exception as e:
        return jsonify({"error": str(e)}), 500


if __name__ == '__main__':
    try:
        log_message("Starting Flask-SocketIO server...")

        # Start cooperative background tasks under Socket.IO's greenlet scheduler
        socketio.start_background_task(connect_to_drone)
        log_message("Vehicle connection task started")

        socketio.start_background_task(process_mavlink_messages)
        log_message("MAVLink message processing task started")

        socketio.start_background_task(telemetry_loop)
        log_message("Telemetry task started")

        log_message("All background tasks started successfully", "SUCCESS")
        log_message("Starting Flask-SocketIO server on http://0.0.0.0:5001", "SUCCESS")

        socketio.run(app, host='0.0.0.0', port=5001, debug=False)

    except Exception as e:
        log_message(f"Failed to start server: {e}", "ERROR")
        raise e
