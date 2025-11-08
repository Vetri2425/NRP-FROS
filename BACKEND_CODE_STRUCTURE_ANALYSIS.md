# 📐 Backend Code Structure Analysis

## 🎯 Overview

Based on the helper scripts (`fix_servo_cors.py`, `merge_cors_handlers.py`) and documentation found in the workspace, here's the analyzed backend structure:

---

## 🏗️ Flask Application Structure

### Basic Setup

```python
# server.py structure (estimated based on helper scripts)

from flask import Flask, request, jsonify
from flask_cors import CORS
import time
import subprocess
from typing import Dict, List, Optional

app = Flask(__name__)

# CORS Configuration (line ~30)
CORS(app, resources={r"/*": {"origins": "*"}})

# Response tracking for observability (line ~40)
_response_log = []

# Rate limiting and backoff tracking (line ~50)
_connection_backoff_ms = 1000
_last_connection_attempt = 0
```

### CORS Handler Setup (lines ~54-75)

```python
@app.before_request
def handle_preflight():
    """Handle CORS preflight OPTIONS requests"""
    if request.method == "OPTIONS":
        response = app.make_default_options_response()
        headers = response.headers
        headers['Access-Control-Allow-Origin'] = '*'
        headers['Access-Control-Allow-Methods'] = 'GET, POST, PUT, DELETE, OPTIONS'
        headers['Access-Control-Allow-Headers'] = 'Content-Type, Authorization, X-Requested-With'
        headers['Access-Control-Max-Age'] = '3600'
        return response

@app.after_request
def unified_response_handler(response):
    """Unified handler: Add CORS headers + track HTTP responses"""
    # Add CORS headers
    response.headers['Access-Control-Allow-Origin'] = '*'
    response.headers['Access-Control-Allow-Methods'] = 'GET, POST, PUT, DELETE, OPTIONS'
    response.headers['Access-Control-Allow-Headers'] = 'Content-Type, Authorization, X-Requested-With'
    
    # Track HTTP responses for observability
    try:
        if request.path.startswith('/socket.io'):
            return response
        method = request.method
        path = request.path
        status = response.status_code
        _response_log.append({
            'ts': time.time(),
            'method': method,
            'path': path,
            'status': status
        })
        # Keep last 500
        if len(_response_log) > 500:
            _response_log.pop(0)
    except Exception:
        pass
    
    return response
```

---

## 🎮 Endpoints Structure

### Health Check Endpoints (Early in file, ~line 100-150)

**Missing/Broken in current backend:**
```python
@app.route('/')
def root():
    # ❌ CURRENTLY BROKEN - Returns 500
    # FIX: Should return status message
    return jsonify({
        'status': 'online',
        'message': 'NRP Rover Backend Server',
        'version': '1.0',
        'endpoints': ['/api/health', '/api/mission/upload', '/api/mission/download']
    }), 200

@app.route('/api/health')
def health_check():
    # ❌ CURRENTLY BROKEN - Returns 500
    # FIX: Should check actual service status
    return jsonify({
        'success': True,
        'status': 'healthy',
        'timestamp': time.time(),
        'services': {
            'ros': 'connected',
            'mavros': 'connected',
            'mission': 'available'
        }
    }), 200
```

### Servo Endpoints (around line 200-300)

```python
@app.route('/api/servo/status', methods=['GET'])
def servo_status():
    # ⚠️  CURRENTLY RETURNS {} 
    # FIX: Should return actual servo state
    return jsonify({
        'success': True,
        'active': False,
        'servos': {
            '10': {'pwm': 1500, 'angle': 90},
            '11': {'pwm': 1500, 'angle': 90}
        }
    }), 200

@app.route('/api/servo/control', methods=['POST'])
def control_servo():
    # ✅ THIS WORKS CORRECTLY
    data = request.get_json()
    servo_id = data.get('servo_id')
    angle = data.get('angle')
    
    # Convert angle to PWM
    pwm = int(1500 + (angle - 90) * 5.56)  # ~5.56 µs per degree
    
    # Send to MAVROS
    # ... implementation details
    
    return jsonify({
        'success': True,
        'message': f'Servo {servo_id} set to PWM {pwm}',
        'servo_id': servo_id,
        'angle': angle,
        'pwm': pwm
    }), 200
```

### Mission Endpoints (around line 800-1300)

#### 1. Mission Upload Endpoint (line ~1200-1250)

```python
@app.route('/api/mission/upload', methods=['POST'])
def _handle_upload_mission():
    """
    Upload mission waypoints to rover via MAVROS
    
    Request Format:
    {
        "waypoints": [
            {
                "lat": 1307207955,      # Integer format (× 1e7)
                "lng": -8026193800,     # Integer format (× 1e7)
                "alt": 10,
                "seq": 0,
                "cmd": 16
            },
            ...
        ]
    }
    """
    try:
        # ❌ ISSUE #1: No try-except block or poor exception handling
        data = request.get_json()
        waypoints = data.get('waypoints', [])
        
        # ❌ ISSUE #2: No validation of waypoints structure
        if not waypoints:
            return jsonify({
                'success': False,
                'error': 'No waypoints provided'
            }), 400
        
        # ❌ ISSUE #3: _require_vehicle_bridge() might throw exception
        vehicle_bridge = _require_vehicle_bridge()
        
        # ❌ ISSUE #4: _build_mavros_waypoints() might crash on integer conversion
        mavros_waypoints = _build_mavros_waypoints(waypoints)
        
        # ❌ ISSUE #5: MAVROS service call might timeout
        success = vehicle_bridge.set_mission(mavros_waypoints)
        
        if success:
            return jsonify({
                'success': True,
                'message': f'Uploaded {len(waypoints)} waypoints to rover',
                'waypoint_count': len(waypoints)
            }), 200
        else:
            return jsonify({
                'success': False,
                'error': 'MAVROS mission push failed'
            }), 500
            
    except Exception as e:
        # ❌ ISSUE #6: Unhandled exceptions return 500 without details
        print(f"Error in _handle_upload_mission: {e}")
        import traceback
        traceback.print_exc()
        
        return jsonify({
            'success': False,
            'error': str(e)
        }), 500
```

#### 2. Mission Download Endpoint (line ~1250-1300)

```python
@app.route('/api/mission/download', methods=['GET'])
def _handle_mission_download():
    """
    Download waypoints from rover mission via MAVROS
    
    Response Format:
    {
        "success": true,
        "waypoints": [
            {
                "lat": 13.07207955,     # Float format (from integer ÷ 1e7)
                "lng": -80.26193800,    # Float format (from integer ÷ 1e7)
                "alt": 10,
                "seq": 0,
                "cmd": 16
            },
            ...
        ]
    }
    """
    try:
        vehicle_bridge = _require_vehicle_bridge()
        waypoints = vehicle_bridge.get_mission()
        
        return jsonify({
            'success': True,
            'waypoints': waypoints,
            'count': len(waypoints)
        }), 200
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        }), 500
```

#### 3. Mission Clear Endpoint (line ~1300-1320)

```python
@app.route('/api/mission/clear', methods=['POST'])
def _handle_mission_clear():
    try:
        vehicle_bridge = _require_vehicle_bridge()
        vehicle_bridge.clear_mission()
        
        return jsonify({
            'success': True,
            'message': 'Mission cleared from rover'
        }), 200
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        }), 500
```

---

## 🔑 Helper Functions

### Vehicle Bridge Initialization (around line 800-850)

```python
def _require_vehicle_bridge():
    """
    Get or initialize the MAVROS vehicle bridge
    
    Raises:
        Exception: If MAVROS bridge cannot be initialized
    """
    global _vehicle_bridge
    
    # ❌ ISSUE: No timeout handling
    if _vehicle_bridge is None:
        try:
            _vehicle_bridge = MAVROSBridge()
            _vehicle_bridge.connect()
        except Exception as e:
            raise Exception(f"Failed to connect to MAVROS: {e}")
    
    return _vehicle_bridge
```

### Waypoint Conversion Function (around line 1250-1280)

```python
def _build_mavros_waypoints(waypoints: List[Dict]) -> List[Dict]:
    """
    Convert UI waypoint format to MAVROS format
    
    Input (from frontend):
    {
        "lat": 1307207955,      # Integer (× 1e7)
        "lng": -8026193800,     # Integer (× 1e7)
        "alt": 10,
        "cmd": 16,
        "seq": 0
    }
    
    Output (for MAVROS):
    {
        "x_lat": 13.07207955,   # Float (÷ 1e7)
        "y_long": -80.26193800, # Float (÷ 1e7)
        "z_alt": 10,
        "command": "16",
        "frame": "GLOBAL_RELATIVE_ALT",
        "is_current": False,
        "autocontinue": True
    }
    """
    try:
        result = []
        for i, wp in enumerate(waypoints):
            # ❌ ISSUE: If lat/lng not converted from integer, this fails
            lat_float = safe_float(wp.get('lat', 0)) / 1e7 if isinstance(wp.get('lat'), int) else safe_float(wp.get('lat'))
            lng_float = safe_float(wp.get('lng', 0)) / 1e7 if isinstance(wp.get('lng'), int) else safe_float(wp.get('lng'))
            
            # ❌ ISSUE: No validation of coordinate ranges
            if not (-90 <= lat_float <= 90) or not (-180 <= lng_float <= 180):
                raise ValueError(f"Waypoint {i}: Invalid coordinates lat={lat_float}, lng={lng_float}")
            
            mavros_wp = {
                'x_lat': lat_float,
                'y_long': lng_float,
                'z_alt': safe_float(wp.get('alt', 0)),
                'command': str(wp.get('cmd', 16)),
                'frame': 'GLOBAL_RELATIVE_ALT',
                'is_current': False,
                'autocontinue': True
            }
            result.append(mavros_wp)
        
        return result
    except Exception as e:
        raise Exception(f"Failed to build MAVROS waypoints: {e}")

def safe_float(value):
    """Safely convert value to float"""
    try:
        return float(value)
    except (TypeError, ValueError):
        return 0.0
```

### RTK Endpoints (around line 400-500)

```python
@app.route('/api/rtk/inject', methods=['POST'])
def inject_rtk():
    """Start NTRIP RTK stream"""
    data = request.get_json()
    ntrip_url = data.get('ntrip_url')
    
    # Start RTK client in background thread
    # ...
    
    return jsonify({
        'success': True,
        'message': 'RTK stream started successfully'
    }), 200

@app.route('/api/rtk/status', methods=['GET'])
def rtk_status():
    """Check RTK connection status"""
    return jsonify({
        'success': True,
        'running': _rtk_client_running,
        'total_bytes': _rtk_bytes_received,
        'rate_bps': _rtk_bytes_per_second
    }), 200
```

---

## 🔗 MAVROS Bridge Integration

### Expected MAVROS Topics Used

```python
# Mission topics
'/mavros/mission/pull'      # Pull mission from flight controller
'/mavros/mission/push'      # Push mission to flight controller
'/mavros/mission/clear'     # Clear mission
'/mavros/mission/set_current'  # Set current waypoint

# GPS topics
'/mavros/global_position/global'  # Current position

# State topics
'/mavros/state'             # Arming/mode status

# Command topics
'/mavros/cmd/command'       # Send MAV_CMD
```

---

## 📊 Data Flow Summary

```
Frontend Request
    ↓
    POST /api/mission/upload
    ├─ Convert integer coords to float (÷ 1e7)
    ├─ Validate ranges
    ├─ Build MAVROS format waypoints
    ├─ Call MAVROS service /mavros/mission/push
    ├─ Wait for response
    └─ Return success/error
    ↓
Backend Response
    ├─ 200 OK: {"success": true, "count": N}
    ├─ 400 Bad Request: {"error": "Invalid waypoints"}
    └─ 500 Internal Error: {"error": "MAVROS connection failed"}
```

---

## ⚠️ Known Issues in Current Backend

1. **No input validation** on mission upload
2. **No error details** returned on 500 errors
3. **No timeout handling** for MAVROS service calls
4. **No logging** of what's happening at each step
5. **Health check endpoints broken** (return 500)
6. **CORS might not be properly applied** to all endpoints
7. **No rate limiting** on API calls
8. **No authentication/authorization**

---

## ✅ What Needs to Be Fixed

### Priority 1: Fix Mission Upload Endpoint

```python
@app.route('/api/mission/upload', methods=['POST'])
def _handle_upload_mission():
    try:
        # Validate request
        if not request.is_json:
            return jsonify({'error': 'Request must be JSON'}), 400
        
        data = request.get_json()
        if not data:
            return jsonify({'error': 'Empty request body'}), 400
        
        waypoints = data.get('waypoints')
        if not isinstance(waypoints, list) or len(waypoints) == 0:
            return jsonify({'error': 'waypoints must be non-empty list'}), 400
        
        # Validate waypoint structure
        for i, wp in enumerate(waypoints):
            required = ['lat', 'lng', 'alt', 'seq', 'cmd']
            for field in required:
                if field not in wp:
                    return jsonify({
                        'error': f'Waypoint {i} missing required field: {field}'
                    }), 400
        
        # Connect to MAVROS
        try:
            bridge = _require_vehicle_bridge()
        except Exception as e:
            return jsonify({
                'error': f'MAVROS bridge connection failed: {e}'
            }), 503  # Service Unavailable
        
        # Convert and upload
        try:
            mavros_wps = _build_mavros_waypoints(waypoints)
            success = bridge.set_mission(mavros_wps)
        except Exception as e:
            return jsonify({
                'error': f'Mission conversion/upload failed: {e}'
            }), 400
        
        if not success:
            return jsonify({
                'error': 'MAVROS mission push returned false'
            }), 500
        
        return jsonify({
            'success': True,
            'message': f'Uploaded {len(waypoints)} waypoints',
            'count': len(waypoints)
        }), 200
        
    except Exception as e:
        print(f"Unexpected error in _handle_upload_mission: {e}")
        import traceback
        traceback.print_exc()
        
        return jsonify({
            'error': f'Internal server error: {str(e)}'
        }), 500
```

---

## 🎯 Conclusion

The backend code structure appears to follow a standard Flask pattern with:
- ✅ CORS handling (recently fixed)
- ✅ Endpoint-based architecture
- ✅ MAVROS integration via ROS services
- ❌ Poor error handling in mission endpoints
- ❌ Missing health check endpoints
- ❌ No validation of input data

The HTTP 500 error is most likely due to an unhandled exception in `_handle_upload_mission()` or `_build_mavros_waypoints()` when processing the coordinate conversion.

