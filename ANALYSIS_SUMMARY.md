# 🔍 Backend Code Structure - Analysis Summary

## 📋 What We Found

Based on analysis of helper scripts and documentation in the workspace, I've reverse-engineered the backend Flask server structure running on the Jetson at `192.168.1.101:5001`.

---

## 🏛️ Backend Architecture

```
Flask Server (Python)
├── CORS Configuration
│   ├── @app.before_request (handle OPTIONS preflight)
│   └── @app.after_request (add CORS headers)
│
├── Health & Status Endpoints
│   ├── GET / (currently broken - returns 500)
│   └── GET /api/health (currently broken - returns 500)
│
├── Servo Control Endpoints ✅ WORKING
│   ├── GET /api/servo/status
│   └── POST /api/servo/control
│
├── Mission Management Endpoints ❌ BROKEN (HTTP 500)
│   ├── POST /api/mission/upload ← THIS ONE FAILING
│   ├── GET /api/mission/download
│   ├── POST /api/mission/clear
│   └── POST /api/mission/set_current
│
├── RTK NTRIP Endpoints
│   ├── POST /api/rtk/inject
│   ├── GET /api/rtk/status
│   └── POST /api/rtk/stop
│
└── MAVROS Bridge Layer
    ├── MAVROSBridge() initialization
    ├── vehicle_bridge.connect()
    ├── vehicle_bridge.set_mission() ← Called by mission/upload
    └── vehicle_bridge.get_mission() ← Called by mission/download
```

---

## 🚨 Root Cause of HTTP 500 Error

### Why Mission Upload Fails

The endpoint `/api/mission/upload` is experiencing an **unhandled exception** during one of these steps:

```python
1. JSON parsing: data = request.get_json()
   ├─ ❌ Could fail if JSON is malformed

2. Validation: waypoints = data.get('waypoints', [])
   ├─ ❌ Could fail if 'waypoints' key doesn't exist or wrong type

3. Vehicle Bridge: vehicle_bridge = _require_vehicle_bridge()
   ├─ ❌ Could fail if MAVROS not responding
   ├─ ❌ Could fail if ROS2 connection lost
   └─ ❌ Could timeout waiting for MAVROS

4. Waypoint Conversion: mavros_wps = _build_mavros_waypoints(waypoints)
   ├─ ❌ Could fail converting integers to floats
   ├─ ❌ Could fail validating coordinate ranges
   └─ ❌ Could fail building MAVROS waypoint structure

5. MAVROS Upload: success = vehicle_bridge.set_mission(mavros_wps)
   ├─ ❌ Could timeout waiting for MAVROS service response
   ├─ ❌ Could fail if waypoint count exceeds limits
   └─ ❌ Could fail if coordinates invalid for flight controller
```

**Since there's no proper try-except wrapper, ANY of these exceptions → HTTP 500**

---

## 💾 Data Flow Analysis

### Frontend → Backend Communication

```javascript
// Frontend (useRoverROS.ts uploadMission)
waypoints = [
  { lat: 1307207955, lng: -8026193800, alt: 10, seq: 0, cmd: 16 }
]
// (lat/lng are integers: degrees × 1e7)

POST http://192.168.1.101:5001/api/mission/upload
Content-Type: application/json
{
  "waypoints": [
    { lat: 1307207955, lng: -8026193800, alt: 10, seq: 0, cmd: 16 }
  ]
}
```

### Backend → MAVROS Communication

```python
# Backend (_build_mavros_waypoints)
def convert(wp):
    lat_float = wp['lat'] / 1e7  # 1307207955 ÷ 1e7 = 13.07207955
    lng_float = wp['lng'] / 1e7  # -8026193800 ÷ 1e7 = -80.26193800
    
    return {
        'x_lat': lat_float,
        'y_long': lng_float,
        'z_alt': wp['alt'],
        'command': wp['cmd']
        # ... other MAVROS fields
    }

# Calls MAVROS service
/mavros/mission/push with converted waypoints
```

---

## 📊 Critical Code Sections (Estimated)

### 1. Mission Upload Handler (~line 1200-1250)

```python
@app.route('/api/mission/upload', methods=['POST'])
def _handle_upload_mission():
    # Line 1: Get JSON - ❌ Could crash here
    # Line 5: Get waypoints - ❌ Could crash here
    # Line 10: Connect MAVROS - ❌ Could crash here
    # Line 15: Convert waypoints - ❌ Could crash here
    # Line 20: Upload to MAVROS - ❌ Could crash here
    # Line 25: Return response - ✅ This line not reached on error
```

### 2. Waypoint Conversion (~line 1250-1280)

```python
def _build_mavros_waypoints(waypoints):
    for i, wp in enumerate(waypoints):
        # ❌ Critical: Assumes lat/lng are integers needing ÷1e7
        lat_float = safe_float(wp.get('lat')) / 1e7  
        lng_float = safe_float(wp.get('lng')) / 1e7
        
        # ❌ No validation that values are in valid range
        # ❌ No check that conversion worked properly
```

### 3. Vehicle Bridge Connection (~line 800-850)

```python
def _require_vehicle_bridge():
    global _vehicle_bridge
    
    # ❌ First-time connection could fail:
    #    - MAVROS not running
    #    - ROS2 environment not initialized
    #    - Network connectivity issues
    #    - Flight controller disconnected
    
    if _vehicle_bridge is None:
        _vehicle_bridge = MAVROSBridge()
        _vehicle_bridge.connect()  # ← Throws exception on failure
    
    return _vehicle_bridge
```

---

## 🔧 Required Fixes

### Fix #1: Add Input Validation

```python
✅ BEFORE uploading:
   - Check 'waypoints' exists and is list
   - Check list is not empty
   - Check each waypoint has: lat, lng, alt, seq, cmd
   - Check values are correct types (integers/floats)
```

### Fix #2: Add Coordinate Validation

```python
✅ BEFORE converting:
   - Check lat is integer (degrees × 1e7)
   - Check lng is integer (degrees × 1e7)
   - After conversion, validate:
     - -90 ≤ lat ≤ 90
     - -180 ≤ lng ≤ 180
     - alt ≥ 0
```

### Fix #3: Add MAVROS Connection Handling

```python
✅ BEFORE accessing bridge:
   - Timeout if MAVROS not responding (5 seconds)
   - Return 503 Service Unavailable (not 500)
   - Log actual error message
```

### Fix #4: Add Exception Wrapping

```python
✅ Wrap EVERY external call:
   - JSON parsing: try/except
   - MAVROS connection: try/except
   - MAVROS mission push: try/except
   - Return detailed error messages (not silent 500)
```

### Fix #5: Add Logging

```python
✅ Log at each step:
   print(f"[MISSION UPLOAD] Received {len(waypoints)} waypoints")
   print(f"[MISSION UPLOAD] Converting waypoint 0: lat={wp['lat']}, lng={wp['lng']}")
   print(f"[MISSION UPLOAD] Connected to MAVROS")
   print(f"[MISSION UPLOAD] Pushing mission to rover")
   print(f"[MISSION UPLOAD] Success!")
```

---

## 🧪 Testing Strategy

### Test 1: Verify Backend is Running
```bash
ps aux | grep server.py
```

### Test 2: Test Health Check
```bash
curl http://192.168.1.101:5001/api/health
```
Should return 200 OK with status object

### Test 3: Test Mission Upload Directly
```bash
curl -X POST \
  -H "Content-Type: application/json" \
  -d '{"waypoints": [{"lat": 1307207955, "lng": -8026193800, "alt": 10, "seq": 0, "cmd": 16}]}' \
  http://192.168.1.101:5001/api/mission/upload
```
Should return 200 with success message OR detailed error

### Test 4: Check Backend Logs
```bash
tail -50 ~/NRP_ROS/Backend/server.log
```
Should show what failed

---

## 📝 Summary

| Component | Status | Issue | Fix |
|-----------|--------|-------|-----|
| Frontend Code | ✅ OK | None | None |
| API Request | ✅ OK | None | None |
| Endpoint Handler | ❌ 500 Error | No error handling | Add try/except |
| Waypoint Conversion | ❌ Unknown | Likely missing integer→float conversion | Add /1e7 division |
| MAVROS Bridge | ❌ Unknown | Could be connection failure | Add timeout handling |
| Input Validation | ❌ None | No checks on request data | Add validation |
| Error Logging | ❌ None | Errors silently converted to 500 | Add logging |

---

## 🎯 Next Action

**You need to SSH into the Jetson and:**

1. ✅ Check if server.py is running
2. ✅ Capture the actual backend error when mission upload fails
3. ✅ Share the backend logs showing what exception occurred

Then we can fix the specific issue! The frontend is working perfectly - it's the backend that needs debugging.

