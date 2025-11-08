# 🔴 Mission Upload HTTP 500 Error - Root Cause Analysis

## 📊 Executive Summary

**Problem:** Frontend mission upload returns HTTP 500 error from `POST /api/mission/upload`  
**Browser Console:** `192.168.1.101:5001/api/mission/upload — status 500 (INTERNAL SERVER ERROR)`  
**Attempted Action:** User clicked "Write Rover Mission" button with 2 waypoints  
**Request Status:** Multiple 404 errors on `/api/status` followed by 500 on mission upload  

---

## 🔍 Root Cause Analysis

### Frontend Code Path (VERIFIED ✅)

The frontend mission upload flow is **working correctly**:

```
Frontend (App.tsx)
  ↓
  handleWriteToRover()
  ├─ Validates waypoints exist
  ├─ Calls sanitizeWaypointsForUpload()
  ├─ Confirms with user
  ├─ Converts coordinates to MAVLink integer format (×1e7)
  └─ Calls services.uploadMission(sanitizedWaypoints)
     ↓
     useRoverROS.ts → uploadMission()
     ├─ Maps waypoints: lat/lng converted to integers
     ├─ Constructs payload: { waypoints: [...] }
     └─ Calls: postService('/mission/upload', payload)
        ↓
        fetchJson(`${API_BASE}/mission/upload`, POST)
        │
        ${API_BASE} = `${DEFAULT_HTTP_BASE}/api`
        DEFAULT_HTTP_BASE = BACKEND_URL.replace(/\/$/, '')
        BACKEND_URL = 'http://192.168.1.101:5001'  (from config.ts)
        │
        └─ Final URL: http://192.168.1.101:5001/api/mission/upload
```

### Frontend Payload (What's Being Sent)

```javascript
// From console log: "[MISSION UPLOAD] Payload prepared for upload: Array(2)"
{
  waypoints: [
    {
      alt: 10,
      cmd: 16,
      lat: 1307207955,      // 13.07207955° × 1e7 (7 decimal digits)
      lng: -8026193800,     // -80.26193800° × 1e7 (7 decimal digits)
      seq: 0,
      ...other fields
    },
    {
      alt: 10,
      cmd: 16,
      lat: 1307208000,      // 13.07208000° × 1e7
      lng: -8026193850,     // -80.26193850° × 1e7
      seq: 1,
      ...other fields
    }
  ]
}
```

### HTTP Request Details

**Method:** POST  
**URL:** `http://192.168.1.101:5001/api/mission/upload`  
**Content-Type:** `application/json`  
**Request Body:** Valid JSON with integer coordinates  

---

## ❌ Backend Issues (HTTP 500)

### Probable Causes

#### 1. **Endpoint Handler Crash** (MOST LIKELY)
The backend `_handle_upload_mission()` function is throwing an unhandled exception:

**Possible failure points:**
```python
# In server.py around /mission/upload endpoint:

@app.route('/api/mission/upload', methods=['POST'])
def handle_upload_mission():
    try:
        data = request.get_json()  # ← Could fail: invalid JSON
        waypoints = data.get('waypoints', [])  # ← Could fail: KeyError
        
        # ← Could fail: MAVROS bridge not initialized
        vehicle_bridge = _require_vehicle_bridge()
        
        # ← Could fail: _build_mavros_waypoints() error
        mavros_waypoints = _build_mavros_waypoints(waypoints)
        
        # ← Could fail: MAVROS service call timeout
        response = vehicle_bridge.set_mission(mavros_waypoints)
        
        return jsonify({'success': True}), 200
    except Exception as e:
        # ← If exception isn't caught here, Flask returns 500
        return jsonify({'error': str(e)}), 500
```

#### 2. **MAVROS Bridge Not Connected**
The `_require_vehicle_bridge()` check might be failing because MAVROS service is unreachable.

#### 3. **Invalid Waypoint Format After Conversion**
The `_build_mavros_waypoints()` function might not be handling the integer-format coordinates correctly.

---

## 🔧 Debugging Steps

### Step 1: Check Backend Process Status

```bash
ssh flash@192.168.1.101 "ps aux | grep server.py"
```

**Expected Output:**
```
flash    12345  0.5  1.2  456789 123456 ?  Sl  19:00  0:15 /usr/bin/python3 server.py
```

**If NOT running:**
```bash
cd ~/NRP_ROS/Backend
python3 server.py 2>&1 | tee server.log
```

### Step 2: Check /api/health Endpoint

```bash
curl -X GET http://192.168.1.101:5001/api/health
```

**Expected Response (200 OK):**
```json
{
  "success": true,
  "status": "healthy",
  "services": {
    "ros": "connected",
    "mavros": "ready",
    "mission": "available"
  }
}
```

**If Returns 404 or 500:** Backend endpoint registration is broken.

### Step 3: Direct POST Test to /api/mission/upload

```bash
curl -X POST \
  -H "Content-Type: application/json" \
  -d '{
    "waypoints": [
      {
        "lat": 1307207955,
        "lng": -8026193800,
        "alt": 10,
        "seq": 0,
        "cmd": 16
      }
    ]
  }' \
  http://192.168.1.101:5001/api/mission/upload
```

**This will show the EXACT error message** from the backend instead of generic 500.

### Step 4: Check Backend Logs for Errors

```bash
ssh flash@192.168.1.101 "tail -50 ~/NRP_ROS/Backend/server.log"
```

Look for:
- `Traceback` - Python exceptions
- `Error` - error messages
- `MAVROS` - MAVROS bridge connection issues
- `KeyError` - missing request fields

---

## 🎯 What to Check in Backend Code

### File Location
```
~/NRP_ROS/Backend/server.py
```

### Function: `_handle_upload_mission(data)` (Around line ~1200-1250)

**Check:**
1. ✅ Does it have proper error handling with try/except?
2. ✅ Does it validate `data['waypoints']` exists?
3. ✅ Does it call `_require_vehicle_bridge()` safely?
4. ✅ Does `_build_mavros_waypoints()` handle integer coordinates?

### Function: `_build_mavros_waypoints(waypoints)` (Around line ~1250-1280)

**Check:**
1. ✅ Does it convert integer lat/lng back to float? (divide by 1e7)
2. ✅ Does it validate coordinate ranges?
3. ✅ Does it create proper MAVROS Waypoint objects?

### Function: `_require_vehicle_bridge()` (Around line ~800-850)

**Check:**
1. ✅ Is MAVROS bridge properly initialized?
2. ✅ Does it timeout if MAVROS not responding?
3. ✅ Are timeouts being caught and handled?

---

## 📋 Checklist for Backend Developer

- [ ] Backend server is running (`ps aux | grep server.py`)
- [ ] `/api/health` returns 200 OK
- [ ] `/api/mission/upload` accepts POST requests
- [ ] Endpoint validates request structure before processing
- [ ] Error messages are returned (not silent crashes)
- [ ] MAVROS bridge status is checked before mission upload
- [ ] Integer coordinates are converted back to floats (÷1e7)
- [ ] Waypoint validation checks lat/lng ranges: ±180°
- [ ] Altitude validation checks positive values
- [ ] Exception handling wraps all external service calls
- [ ] Logs show what's happening at each step

---

## 📞 Next Actions

1. **SSH into Jetson** and capture backend error output
2. **Share backend logs** showing the HTTP 500 error
3. **Test `/api/health`** to verify endpoint structure
4. **Run direct curl POST** to get exact error message
5. **Check MAVROS bridge connectivity** via ROS2 CLI

Once we have the actual backend error, we can fix it immediately! 🚀

