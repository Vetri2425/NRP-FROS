# Mission Planner GCS - Quick Reference Guide
## Essential Components for Rover GCS Development

---

## 🚀 Quick Start Priorities

### Phase 1: Minimum Viable GCS (6 weeks)

**Goal:** Connect to rover and see basic telemetry

#### Week 1-2: Connection
```
✅ COM port selector
✅ Baud rate selector (57600/115200)
✅ Connect/Disconnect button
✅ MAVLink heartbeat parser
✅ Connection status LED
```

#### Week 3-4: Basic HUD
```
✅ Ground speed
✅ Heading compass
✅ Battery voltage/percentage
✅ GPS satellites
✅ Flight mode display
✅ Armed/disarmed indicator
```

#### Week 5-6: Simple Map
```
✅ Map widget (OSM)
✅ Vehicle position marker
✅ Home marker
✅ Zoom/pan controls
```

**Result:** You can connect, monitor, and see vehicle location!

---

## 📊 Component Priority Matrix

### Critical (Must Have) - 120 components
- Connection interface
- MAVLink communication
- Basic HUD (speed, heading, battery, GPS)
- Map with vehicle position
- Arm/disarm controls
- Flight mode selection
- Waypoint planning basics
- Setup wizards (calibration)

### High Priority (Should Have) - 210 components
- Full HUD (all telemetry)
- Mission file operations
- Parameter system
- Advanced map features
- Action controls
- Status monitoring
- Pre-arm checks

### Medium Priority (Nice to Have) - 145 components
- Custom gauges
- Advanced tools
- Log analysis
- Parameter comparison
- Tuning interfaces

### Low Priority (Optional) - 72 components
- Advanced features
- Developer tools
- Experimental functions

---

## 🎯 Rover-Specific Checklist

### ✅ Must Implement for Rovers

**Flight Modes (Critical)**
- [ ] Manual - Direct RC control
- [ ] Hold - Stop and hold position
- [ ] Steering - Heading control
- [ ] Guided - GCS waypoint commands
- [ ] Auto - Execute mission
- [ ] RTL - Return to launch

**Speed Control (Rover-Specific)**
- [ ] Throttle = forward speed
- [ ] Reverse capability
- [ ] Speed display (m/s or km/h)
- [ ] Max speed limit

**Steering Configuration**
- [ ] Skid steering (tank drive)
- [ ] Ackermann steering (car-like)
- [ ] Steering angle display

**Navigation**
- [ ] Smaller waypoint radius (1-3m)
- [ ] Can stop at waypoints
- [ ] Distance to waypoint
- [ ] Crosstrack error

### ❌ Skip for Rovers

- Airspeed sensor
- Altitude hold mode
- Takeoff/land commands
- Survey grid tool (unless needed)
- Artificial horizon (optional only)
- Wind indicator

### 🔧 Adapt for Rovers

- HUD: Emphasize speed/heading, de-emphasize altitude
- Map: Add steering angle indicator
- Waypoints: Tighter acceptance radius
- Parameters: Expose rover-specific tuning

---

## 💻 Code Structure Template

```
custom-gcs/
├── src/
│   ├── communication/
│   │   ├── mavlink_handler.py
│   │   ├── connection_manager.py
│   │   └── message_parser.py
│   ├── ui/
│   │   ├── main_window.py
│   │   ├── hud_widget.py
│   │   ├── map_widget.py
│   │   └── mission_planner.py
│   ├── data/
│   │   ├── vehicle_state.py
│   │   ├── mission_manager.py
│   │   └── parameter_manager.py
│   ├── config/
│   │   ├── settings.py
│   │   └── defaults.py
│   └── utils/
│       ├── coordinates.py
│       └── calculations.py
├── resources/
│   ├── icons/
│   ├── maps/
│   └── styles/
├── tests/
└── docs/
```

---

## 📡 Essential MAVLink Messages

### Phase 1 (MVP)
```python
# Critical for basic operation
HEARTBEAT (0)         # 1 Hz - Keep connection alive
SYS_STATUS (1)        # 1 Hz - Battery, sensors
GPS_RAW_INT (24)      # 5 Hz - GPS position
ATTITUDE (30)         # 10 Hz - Roll, pitch, yaw
GLOBAL_POSITION_INT (33) # 4 Hz - Lat, lon, alt, heading
VFR_HUD (74)          # 4 Hz - Speed, heading, throttle
STATUSTEXT (253)      # On event - System messages
```

### Phase 2 (Enhanced)
```python
# Add for full functionality
RAW_IMU (27)             # 10 Hz - Sensor data
LOCAL_POSITION_NED (32)  # 4 Hz - Local coordinates
MISSION_CURRENT (42)     # 1 Hz - Current waypoint
NAV_CONTROLLER_OUTPUT (62) # 4 Hz - Navigation data
BATTERY_STATUS (147)     # 1 Hz - Detailed power
ESTIMATOR_STATUS (230)   # 1 Hz - EKF health
```

---

## 🔧 MAVLink Commands Quick Ref

### Navigation Commands
```
WAYPOINT (16)           - Navigate to coordinate
LOITER_TIME (19)        - Circle for n seconds
RETURN_TO_LAUNCH (20)   - Return home
```

### Action Commands
```
DO_CHANGE_SPEED (178)   - Adjust speed mid-mission
DO_SET_HOME (179)       - Change home location
DO_SET_RELAY (181)      - Control relay output
DO_SET_SERVO (183)      - Set servo position
```

### Condition Commands
```
CONDITION_DELAY (112)   - Wait n seconds
CONDITION_DISTANCE (114) - Wait for distance traveled
```

---

## 🎨 UI Layout Recommendations

### Optimal Screen Layout for Rover GCS

```
┌─────────────────────────────────────────────────┐
│  [Logo]  [DATA] [PLAN] [SETUP] [CONFIG]  [Conn]│
├──────────┬──────────────────────────────────────┤
│          │                                       │
│   HUD    │          MAP VIEW                    │
│ (Speed)  │    (Vehicle Position & Waypoints)    │
│ (Heading)│                                       │
│ (Battery)│                                       │
│  (GPS)   │                                       │
│          │                                       │
├──────────┴──────────────────────────────────────┤
│ [ARM] [MODE: Manual▼] [Speed: 2.5 m/s]         │
│ System: GPS 18 Sats | Battery 14.8V (85%)      │
└─────────────────────────────────────────────────┘
```

### HUD Priority (Rover-focused)
1. **Ground Speed** (Large, prominent)
2. **Heading Compass** (Large, circular)
3. **Battery Status** (Voltage + %)
4. **GPS Status** (Sats + fix type)
5. **Flight Mode** (Large text)
6. **Armed Status** (Red/Green indicator)
7. Distance to Home
8. Distance to Waypoint
9. System Messages

### Map Priority
1. Vehicle position (heading indicator)
2. Vehicle trail (breadcrumbs)
3. Home location
4. Waypoint markers
5. Mission path lines
6. Geofence (if configured)

---

## 🛠️ Development Tools

### Recommended IDE
- **PyCharm** (Python)
- **Qt Creator** (C++/Qt)
- **VS Code** (Any language)

### Testing Tools
- **SITL** (ArduPilot Software-in-the-Loop)
- **MAVProxy** (Command-line GCS)
- **Wireshark** (MAVLink packet inspection)

### Debugging
```bash
# Monitor MAVLink messages
mavproxy.py --master=COM3 --baudrate=57600

# SITL Simulation
sim_vehicle.py -v Rover --console --map

# Log MAVLink traffic
mavproxy.py --master=COM3 --out=udp:127.0.0.1:14550
```

---

## 📋 Pre-Flight Checklist Template

### Before First Connection
- [ ] Install ArduPilot firmware (ArduRover)
- [ ] Select frame type (Skid/Ackermann)
- [ ] Calibrate accelerometer
- [ ] Calibrate compass
- [ ] Calibrate radio
- [ ] Configure flight modes
- [ ] Set battery parameters
- [ ] Test motors (props off!)

### Before First Mission
- [ ] GPS 3D fix (6+ satellites)
- [ ] Compass heading correct
- [ ] Pre-arm checks pass
- [ ] Battery fully charged
- [ ] Waypoints uploaded
- [ ] Home location set
- [ ] Geofence configured (optional)
- [ ] Failsafe settings verified

### During Operation
- [ ] Monitor battery voltage
- [ ] Watch GPS satellite count
- [ ] Check system messages
- [ ] Verify waypoint progress
- [ ] Monitor link quality

---

## 🔍 Troubleshooting Quick Guide

### Connection Issues
```
Problem: Can't connect to vehicle
✓ Check COM port selection
✓ Verify baud rate (57600 or 115200)
✓ Test cable with MAVProxy
✓ Check vehicle is powered
✓ Verify USB driver installed
```

### GPS Issues
```
Problem: No GPS lock
✓ Must be outdoors (clear sky view)
✓ Wait 30-60 seconds for first fix
✓ Check antenna connection
✓ Verify GPS_TYPE parameter
✓ Check for interference
```

### Compass Issues
```
Problem: Compass error
✓ Recalibrate outdoors
✓ Away from metal/electronics
✓ Check compass orientation parameter
✓ Verify primary compass selected
✓ Check for interference
```

### Pre-Arm Check Failures
```
Problem: Won't arm
✓ Check all sensors calibrated
✓ GPS must have 3D fix (6+ sats)
✓ Compass variance low
✓ Battery voltage adequate
✓ RC receiver connected
✓ Read STATUSTEXT messages
```

---

## 📚 Essential Parameters (Rover)

### Speed Control
```
CRUISE_SPEED     - Default driving speed (m/s)
CRUISE_THROTTLE  - Throttle for cruise speed (%)
SPEED_TURN_GAIN  - Turning speed reduction
WP_SPEED        - Speed for waypoint navigation
```

### Steering (Ackermann)
```
STEER2SRV_P     - Steering P gain
STEER2SRV_I     - Steering I gain  
STEER2SRV_D     - Steering D gain
STEER2SRV_TCONST - Steering time constant
```

### Navigation
```
WP_RADIUS       - Waypoint acceptance radius (m)
WP_OVERSHOOT   - Waypoint overshoot distance
PIVOT_TURN_ANGLE - Angle to trigger pivot turn
TURN_MAX_G     - Maximum lateral acceleration
```

### Failsafe
```
FS_THR_ENABLE   - RC failsafe enable
FS_THR_VALUE    - RC failsafe PWM threshold
FS_GCS_ENABLE   - GCS failsafe enable
FS_CRASH_CHECK  - Crash detection
```

---

## 🎓 Learning Resources

### Documentation
- ArduPilot Docs: https://ardupilot.org/rover/
- MAVLink Guide: https://mavlink.io/
- Mission Planner: https://ardupilot.org/planner/

### Code Examples
- pymavlink: https://github.com/ArduPilot/pymavlink
- MAVSDK: https://mavsdk.mavlink.io/
- QGroundControl: https://github.com/mavlink/qgroundcontrol

### Community
- ArduPilot Forum: https://discuss.ardupilot.org/
- Discord: ArduPilot Community
- GitHub Issues: Report bugs/ask questions

---

## ⚡ Performance Tips

1. **Update Rates**
   - HUD: 10 Hz minimum
   - Map: 4 Hz sufficient
   - Parameters: 1 Hz or less

2. **Optimization**
   - Cache map tiles offline
   - Throttle UI updates
   - Use worker threads for I/O
   - Minimize redraws

3. **Memory**
   - Limit vehicle trail length
   - Clear old log data
   - Compress stored missions

4. **Battery**
   - Reduce screen brightness
   - Disable unused features
   - Use power-efficient map rendering

---

## 🎯 Success Metrics

### MVP Success (Phase 1)
- ✅ Connect reliably to vehicle
- ✅ Display real-time telemetry
- ✅ Show vehicle on map
- ✅ Arm/disarm vehicle
- ✅ Change flight modes

### Full GCS Success (Phase 10)
- ✅ Plan and execute missions
- ✅ Configure all parameters
- ✅ Complete setup wizards
- ✅ Analyze flight logs
- ✅ Handle all edge cases

---

**This quick reference covers 80% of what you need!**  
*Refer to full Excel checklist for complete component details.*

Last Updated: November 2, 2025
