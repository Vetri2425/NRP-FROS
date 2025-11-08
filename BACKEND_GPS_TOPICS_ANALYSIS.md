# Backend GPS Topics Analysis

## Expected ROS Topics for GPS Data

Based on the frontend code analysis, the backend should be subscribing to these MAVROS topics to get GPS data:

### Primary GPS Topics (MAVROS):

#### 1. **Global Position** (Main GPS Position)
```bash
/mavros/global_position/global
```
- **Type:** `sensor_msgs/NavSatFix`
- **Contains:** latitude, longitude, altitude
- **Update Rate:** ~5-10 Hz
- **Purpose:** Primary GPS position data

#### 2. **GPS Raw Data**
```bash
/mavros/gpsstatus/gps1/raw
```
- **Type:** `mavros_msgs/GPSRAW`
- **Contains:** satellites_visible, fix_type, eph, epv
- **Update Rate:** ~5 Hz
- **Purpose:** Raw GPS status and fix quality

#### 3. **GPS Status**
```bash
/mavros/gps_status
```
- **Type:** `mavros_msgs/GPSStatus`
- **Contains:** satellites_visible, satellites_used, satellite_info
- **Update Rate:** ~1 Hz
- **Purpose:** GPS constellation status

#### 4. **RTK GPS Data** (if using RTK)
```bash
/mavros/gpsstatus/gps2/raw
```
- **Type:** `mavros_msgs/GPSRAW`
- **Purpose:** RTK corrections and baseline data

#### 5. **Velocity Data**
```bash
/mavros/global_position/raw/gps_vel
```
- **Type:** `geometry_msgs/TwistStamped`
- **Contains:** linear velocity (groundspeed)

---

## Commands to Check Topics on Rover (SSH)

### 1. List all GPS-related topics:
```bash
rostopic list | grep -E "gps|global_position"
```

Expected output:
```
/mavros/global_position/compass_hdg
/mavros/global_position/global
/mavros/global_position/gp_lp_offset
/mavros/global_position/local
/mavros/global_position/raw/fix
/mavros/global_position/raw/gps_vel
/mavros/global_position/raw/satellites
/mavros/global_position/rel_alt
/mavros/gps_rtk/rtk_baseline
/mavros/gps_rtk/send_rtcm
/mavros/gpsstatus/gps1/raw
/mavros/gpsstatus/gps2/raw
```

### 2. Check GPS1 Raw data (satellites, fix type):
```bash
rostopic echo /mavros/gpsstatus/gps1/raw
```

Expected output:
```yaml
header: 
  seq: 1234
  stamp: 
    secs: 1699000000
    nsecs: 123456789
  frame_id: ''
fix_type: 3  # 0=No Fix, 1=GPS, 2=DGPS, 3=RTK Float, 4=RTK Fixed
satellites_visible: 18
eph: 121  # Horizontal position accuracy (cm)
epv: 152  # Vertical position accuracy (cm)
vel: 0    # Ground speed (cm/s)
cog: 0    # Course over ground (degrees * 100)
```

### 3. Check Global Position (lat/lon):
```bash
rostopic echo /mavros/global_position/global
```

Expected output:
```yaml
header: 
  seq: 5678
  stamp: 
    secs: 1699000000
    nsecs: 987654321
  frame_id: "map"
status: 
  status: 0  # 0=No Fix, 1=Fix, 2=SBAS Fix
  service: 1
latitude: 13.082700    # Degrees
longitude: 80.270700   # Degrees
altitude: 25.5         # Meters (MSL)
position_covariance: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
position_covariance_type: 0
```

### 4. Check topic publication rate:
```bash
rostopic hz /mavros/gpsstatus/gps1/raw
rostopic hz /mavros/global_position/global
```

Expected output:
```
average rate: 5.000
  min: 0.180s max: 0.220s std dev: 0.01234s window: 25
```

### 5. Check telemetry node info:
```bash
rosnode info /telemetry_node
```

Expected output should show subscriptions to:
```
Subscriptions: 
 * /mavros/global_position/global [sensor_msgs/NavSatFix]
 * /mavros/gpsstatus/gps1/raw [mavros_msgs/GPSRAW]
 * /mavros/state [mavros_msgs/State]
 * /mavros/battery [sensor_msgs/BatteryState]
 * /mavros/rc/out [mavros_msgs/RCOut]
```

### 6. Check telemetry node is running:
```bash
rosnode list | grep telemetry
```

### 7. View telemetry node logs:
```bash
tail -f ~/.ros/log/latest/telemetry_node*.log
```

---

## Compare with MAVProxy (ACM0)

### 1. Start MAVProxy on ACM0:
```bash
mavproxy.py --master=/dev/ttyACM0 --baudrate=57600
```

### 2. In MAVProxy console, check GPS:
```
MAV> gps
```

Expected output:
```
GPS: fix_type=3 num_sats=18 lat=13.0827 lon=80.2707 alt=25.5
```

### 3. Check GPS2 (RTK):
```
MAV> gps2
```

### 4. Monitor GPS status continuously:
```
MAV> watch GPS_RAW_INT
```

---

## Backend Telemetry Node Expected Behavior

The backend's **telemetry_node.py** should:

### 1. Subscribe to these topics:
```python
rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.gps_callback)
rospy.Subscriber('/mavros/gpsstatus/gps1/raw', GPSRAW, self.gps_raw_callback)
rospy.Subscriber('/mavros/state', State, self.state_callback)
```

### 2. Extract GPS data:
```python
def gps_callback(self, msg):
    self.telemetry['global'] = {
        'latitude': msg.latitude,
        'longitude': msg.longitude,
        'altitude': msg.altitude
    }

def gps_raw_callback(self, msg):
    self.telemetry['global']['satellites_visible'] = msg.satellites_visible
    self.telemetry['rtk'] = {
        'fix_type': msg.fix_type,
        'eph': msg.eph,  # Horizontal accuracy
        'epv': msg.epv   # Vertical accuracy
    }
```

### 3. Emit via Socket.IO:
```python
socketio.emit('telemetry', self.telemetry)
# OR
socketio.emit('rover_data', self.rover_data)
```

---

## Diagnostic Script

Save this as `check_gps_topics.sh` on your rover:

```bash
#!/bin/bash

echo "==================================="
echo "GPS Topics Diagnostic Script"
echo "==================================="
echo ""

echo "1. Checking available GPS topics..."
rostopic list | grep -E "gps|global_position"
echo ""

echo "2. Checking GPS1 Raw (5 messages)..."
timeout 2s rostopic echo -n 5 /mavros/gpsstatus/gps1/raw | grep -E "fix_type|satellites_visible"
echo ""

echo "3. Checking Global Position (5 messages)..."
timeout 2s rostopic echo -n 5 /mavros/global_position/global | grep -E "latitude|longitude|altitude"
echo ""

echo "4. Checking publication rates..."
echo "  - GPS1 Raw:"
timeout 5s rostopic hz /mavros/gpsstatus/gps1/raw 2>&1 | head -n 2
echo "  - Global Position:"
timeout 5s rostopic hz /mavros/global_position/global 2>&1 | head -n 2
echo ""

echo "5. Checking telemetry node..."
rosnode list | grep telemetry || echo "  ⚠ Telemetry node not found!"
echo ""

echo "6. Checking telemetry node subscriptions..."
rosnode info /telemetry_node 2>&1 | grep -A 10 "Subscriptions:" || echo "  ⚠ Cannot get telemetry node info"
echo ""

echo "==================================="
echo "Diagnostic complete!"
echo "==================================="
```

Make it executable:
```bash
chmod +x check_gps_topics.sh
./check_gps_topics.sh
```

---

## Expected Frontend Data Flow

Based on `GPS_DATA_FLOW.md`, the frontend expects:

### From Socket.IO event `telemetry`:
```javascript
{
  global: {
    latitude: 13.082700,     // degrees
    longitude: 80.270700,    // degrees
    altitude: 25.5,          // meters
    vel: 0.5,                // m/s
    satellites_visible: 18
  },
  rtk: {
    fix_type: 3,             // 0-4
    baseline_age: 0.5,       // seconds
    base_linked: true
  }
}
```

### From Socket.IO event `rover_data`:
```javascript
{
  position: {
    lat: 13.082700,
    lng: 80.270700
  },
  satellites_visible: 18,
  rtk_status: "RTK Float",   // or "RTK Fixed", "DGPS", "GPS Fix"
  rtk_fix_type: 3,
  rtk_baseline_age: 0.5,
  rtk_base_linked: true
}
```

---

## Verification Checklist

Run these commands on your rover (SSH: `flash@192.168.1.101`):

- [ ] `rostopic list | grep gps` - Shows GPS topics
- [ ] `rostopic echo /mavros/gpsstatus/gps1/raw` - Shows satellites, fix_type
- [ ] `rostopic echo /mavros/global_position/global` - Shows lat/lon/alt
- [ ] `rostopic hz /mavros/gpsstatus/gps1/raw` - Shows ~5 Hz update rate
- [ ] `rosnode info /telemetry_node` - Shows topic subscriptions
- [ ] `mavproxy.py --master=/dev/ttyACM0` then `gps` - Compare with ROS data

---

## Common Issues

### Issue 1: GPS topics not publishing
**Cause:** MAVROS not connected to flight controller
**Solution:**
```bash
roslaunch mavros apm.launch fcu_url:=/dev/ttyACM0:57600
```

### Issue 2: Telemetry node not subscribing
**Cause:** Wrong topic name or node not running
**Solution:**
```bash
# Check if telemetry node is running
rosnode list | grep telemetry

# Restart telemetry node
rosrun backend telemetry_node.py
```

### Issue 3: Frontend not receiving GPS data
**Cause:** Socket.IO event name mismatch or data format
**Solution:** Check browser console (F12) for:
```javascript
console.log('[RTK DEBUG] telemetry event received:', payload);
```

### Issue 4: GPS data is zeros
**Cause:** No GPS fix or GPS not connected
**Solution:**
```bash
# Check GPS connection in MAVProxy
mavproxy.py --master=/dev/ttyACM0
# Type: gps
# Should show fix_type > 0 and satellites > 4
```
