# 🔍 RTK Disconnected - Debugging Guide

## Problem
RTK shows "disconnected" or "No Fix" in the frontend, but other telemetry (GPS position, battery, mode) is working fine.

---

## ✅ Step 1: Check Frontend Console Logs

I've added debug logging to help identify the issue. Open your browser:

1. **Open Developer Console**: Press `F12` or `Ctrl+Shift+I`
2. **Go to Console Tab**
3. **Look for these log messages**:

```
[RTK DEBUG] telemetry event received: {...}
[RTK DEBUG] telemetry.rtk: {...}
[RTK DEBUG] rover_data event received: {...}
[RTK DEBUG] rover_data.rtk_status: "..."
[TelemetryPanel RTK DEBUG] {...}
```

### What to Look For:

#### ✅ **GOOD - RTK Data is Being Received:**
```javascript
[RTK DEBUG] telemetry.rtk: {
  fix_type: 3,           // 1=GPS, 2=DGPS, 3=RTK Float, 4=RTK Fixed
  baseline_age: 0.5,
  base_linked: true
}
```

#### ❌ **BAD - RTK Data is Missing:**
```javascript
[RTK DEBUG] telemetry event received: {
  state: {...},
  global: {...},
  battery: {...},
  // rtk: undefined  ← Missing!
}
```

---

## ✅ Step 2: Check Backend is Running

SSH into your Jetson and verify the services are running:

```bash
ssh flash@192.168.1.100

# Check if backend server is running
ps aux | grep server.py

# Check if telemetry node is running
ps aux | grep telemetry_node.py

# Check ROS2 nodes
ros2 node list
# Should show: /telemetry_node, /mavros, /rosbridge_websocket
```

### If Services Are Not Running:
```bash
cd ~/NRP_ROS
sudo ./start_service.sh
```

---

## ✅ Step 3: Check Backend Logs

Check if the backend is receiving GPS/RTK data from MAVROS:

```bash
ssh flash@192.168.1.100

# Check backend server logs
journalctl -u nrp_backend.service -f
# OR if running manually:
cd ~/NRP_ROS
python3 -m Backend.server
```

### What to Look For:

**GOOD - Backend is receiving RTK data:**
```
Publishing telemetry with RTK fix_type: 3
Global position: lat=34.1234567, lon=-118.1234567, satellites=12
```

**BAD - Backend is NOT receiving RTK data:**
```
Publishing telemetry with RTK fix_type: 0  ← Always 0 means no GPS
Global position: lat=0.0, lon=0.0, satellites=0
```

---

## ✅ Step 4: Check MAVROS GPS Topic

Verify MAVROS is publishing GPS data:

```bash
ssh flash@192.168.1.100

# Check if GPS data is being published
ros2 topic echo /mavros/global_position/global --once

# Expected output (with GPS fix):
# header:
#   stamp:
#     sec: 1730000000
#     nanosec: 123456789
# status:
#   status: 2           ← 0=No Fix, 1=GPS, 2=DGPS, 3=RTK Float, 4=RTK Fixed
#   service: 1
# latitude: 34.1234567
# longitude: -118.1234567
# altitude: 250.5
```

### If No Data is Published:
```bash
# Check if MAVROS is connected to flight controller
ros2 topic echo /mavros/state --once

# Check MAVROS diagnostics
ros2 topic list | grep mavros
```

---

## ✅ Step 5: Check Telemetry Node Output

Verify the telemetry_node.py is publishing RTK data:

```bash
ssh flash@192.168.1.100

# Listen to telemetry node output
ros2 topic echo /nrp/telemetry --once

# Expected output (JSON format):
# data: '{
#   "state": {...},
#   "global": {
#     "latitude": 34.1234567,
#     "longitude": -118.1234567,
#     "altitude": 250.5,
#     "vel": 0.5,
#     "satellites_visible": 12
#   },
#   "rtk": {
#     "fix_type": 3,        ← Should be 1-4 if GPS is working
#     "baseline_age": 0.5,
#     "base_linked": true
#   }
# }'
```

### If RTK is Always 0:
The telemetry_node.py is not extracting RTK data properly. Check if you restarted it after the update.

---

## ✅ Step 6: Verify Backend server.py Update

Check if the backend server.py has the RTK processing code:

```bash
ssh flash@192.168.1.100

# Check if _merge_ros2_telemetry has RTK handling
grep -A 10 "rtk_data = payload.get" ~/NRP_ROS/Backend/server.py

# Should show:
# rtk_data = payload.get('rtk')
# if rtk_data:
#     fix_type_map = {
#         0: 0, 1: 1, 2: 2, 3: 3, 4: 4
#     }
#     fix_type = rtk_data.get('fix_type', 0)
```

### If Not Found:
The server.py update didn't apply. Re-run the update script:

```bash
cd ~/NRP_ROS/Backend
python3 update_server.py server.py
sudo systemctl restart nrp_backend.service
```

---

## ✅ Step 7: Check Socket.IO Network Traffic

In your browser:

1. Open Developer Tools (`F12`)
2. Go to **Network** tab
3. Filter by **WS** (WebSocket)
4. Click on the Socket.IO connection
5. Go to **Messages** tab
6. Look for `rover_data` or `telemetry` events

### What to Look For:

**GOOD - RTK data in Socket.IO:**
```json
{
  "type": "rover_data",
  "data": {
    "rtk_status": "RTK Float",  ← Should show status
    "fix_type": 3,
    "satellites_visible": 12
  }
}
```

**BAD - Missing RTK:**
```json
{
  "type": "rover_data",
  "data": {
    "rtk_status": "No Fix",  ← Always "No Fix"
    "fix_type": 0,           ← Always 0
    "satellites_visible": 0
  }
}
```

---

## ✅ Step 8: Check Frontend Data Transformation

If Socket.IO is receiving RTK data but the UI shows "No Fix", check the data transformation in `App.tsx`:

Open browser console and check:
```javascript
// In Console, type:
window.localStorage.debug = '*';
location.reload();
```

This enables verbose Socket.IO debugging.

---

## 🔧 Common Issues & Solutions

### Issue 1: RTK Shows "No Fix" (fix_type = 0)
**Cause**: MAVROS is not receiving GPS data from flight controller
**Solution**: 
- Check GPS module is connected
- Check flight controller serial port
- Run `ros2 topic echo /mavros/global_position/global`

### Issue 2: RTK Data Not in Socket.IO Events
**Cause**: Backend server.py is not processing RTK data from telemetry_node
**Solution**:
```bash
ssh flash@192.168.1.100
cd ~/NRP_ROS/Backend
python3 update_server.py server.py
sudo ./start_service.sh
```

### Issue 3: Socket.IO Not Connected
**Cause**: Backend server not running or firewall blocking port 5001
**Solution**:
```bash
# Check if port 5001 is open
curl http://192.168.1.100:5001/api/health

# Check firewall
sudo ufw status
sudo ufw allow 5001
```

### Issue 4: GPS Coordinates Work But RTK Status Doesn't
**Cause**: Frontend is receiving lat/lng from old `rover_data` format but not RTK status
**Solution**: Check if backend is emitting both formats. The `_merge_ros2_telemetry` function should populate both:
- `current_state['rtk_status']` (for old format compatibility)
- Emit RTK in telemetry events

---

## 📊 Expected Data Flow

```
GPS Module → Flight Controller → MAVROS (/mavros/global_position/global)
                                      ↓
                            telemetry_node.py (/nrp/telemetry)
                                      ↓
                            server.py (_merge_ros2_telemetry)
                                      ↓
                            Socket.IO (rover_data event)
                                      ↓
                            Frontend (useRoverROS hook)
                                      ↓
                            TelemetryPanel (RTK display)
```

---

## 🎯 Quick Checklist

- [ ] Frontend console shows RTK debug logs
- [ ] `fix_type` is not always 0
- [ ] Backend services are running (server.py, telemetry_node.py)
- [ ] MAVROS is publishing GPS data (`/mavros/global_position/global`)
- [ ] Telemetry node is publishing RTK data (`/nrp/telemetry`)
- [ ] Socket.IO events contain RTK data
- [ ] Browser can connect to `http://192.168.1.100:5001`
- [ ] GPS module is physically connected and has satellite lock

---

## 📝 Next Steps

1. **Start with Step 1** - Check frontend console logs
2. **If frontend shows fix_type=0**, go to **Step 4** (check MAVROS)
3. **If frontend shows no RTK data**, go to **Step 2** (check backend services)
4. **If MAVROS has no GPS**, check hardware GPS module connection

---

## 💡 Tips

- RTK requires GPS satellite lock first (fix_type >= 1)
- RTK Float (fix_type=3) requires base station corrections
- RTK Fixed (fix_type=4) requires good base station link and time
- If you only have GPS without RTK, you'll see fix_type=1 (GPS Fix)
- `satellites_visible` should be at least 4 for basic GPS fix

---

Good luck! Start with Step 1 and work through the checklist. The debug logs will help identify exactly where the RTK data is getting lost.
