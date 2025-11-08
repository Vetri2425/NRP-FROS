# 🎯 WP_MARK Mode - Backend Implementation Specification

## 📋 Executive Summary

**Purpose**: Implement an industrial-grade waypoint-based servo control system for precision agricultural spraying operations.

**Mode Name**: `WP_MARK`

**Architecture**: ROS2 + Flask REST API

**Execution Model**: Autonomous waypoint navigation with configurable servo actuation at each waypoint

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                         FRONTEND                                 │
│  User selects WP_MARK → Enters parameters → Clicks Start        │
└────────────────────────┬────────────────────────────────────────┘
                         │
                         │ HTTP POST /wp_mark/start
                         │
┌────────────────────────▼────────────────────────────────────────┐
│                      FLASK REST API                              │
│  • Receives configuration parameters                            │
│  • Validates inputs                                             │
│  • Saves to config.json                                         │
│  • Launches ROS2 node                                           │
│  • Returns status to frontend                                   │
└────────────────────────┬────────────────────────────────────────┘
                         │
                         │ Spawns/Controls
                         │
┌────────────────────────▼────────────────────────────────────────┐
│                   ROS2 WP_MARK NODE                              │
│  • Subscribes to GPS, State, Waypoint topics                    │
│  • Pulls mission waypoints from flight controller               │
│  • Executes waypoint navigation sequence                        │
│  • Controls servo via MAVROS commands                           │
│  • Publishes mission status                                     │
└────────────────────────┬────────────────────────────────────────┘
                         │
                         │ ROS2 Topics/Services
                         │
┌────────────────────────▼────────────────────────────────────────┐
│                    MAVROS / FLIGHT CONTROLLER                    │
│  • Processes navigation commands                                │
│  • Executes servo PWM commands                                  │
│  • Provides GPS position feedback                               │
│  • Reports mission state                                        │
└─────────────────────────────────────────────────────────────────┘
```

---

## 📊 Configuration Parameters

### **Input Schema** (from Frontend)

```json
{
  "delay_before_start": 2.0,    // seconds (float, 0-60)
  "pwm_start": 1500,            // microseconds (int, 1000-2000)
  "delay_before_stop": 5.0,     // seconds (float, 0-60)
  "pwm_stop": 1000,             // microseconds (int, 1000-2000)
  "delay_after_stop": 1.0       // seconds (float, 0-60)
}
```

### **Parameter Descriptions**

| Parameter | Type | Range | Unit | Description |
|-----------|------|-------|------|-------------|
| `delay_before_start` | float | 0-60 | seconds | Wait time after reaching waypoint before activating servo |
| `pwm_start` | int | 1000-2000 | microseconds | PWM value to activate servo (spray ON) |
| `delay_before_stop` | float | 0-60 | seconds | Duration servo stays active (spraying duration) |
| `pwm_stop` | int | 1000-2000 | microseconds | PWM value to deactivate servo (spray OFF) |
| `delay_after_stop` | float | 0-60 | seconds | Wait time after deactivating servo before moving to next waypoint |

### **Validation Rules**

```python
def validate_parameters(params: dict) -> tuple[bool, str]:
    """
    Validate WP_MARK configuration parameters
    Returns: (is_valid, error_message)
    """
    # Required fields
    required = ['delay_before_start', 'pwm_start', 'delay_before_stop', 
                'pwm_stop', 'delay_after_stop']
    
    for field in required:
        if field not in params:
            return False, f"Missing required field: {field}"
    
    # Type and range validation
    if not (0 <= params['delay_before_start'] <= 60):
        return False, "delay_before_start must be between 0-60 seconds"
    
    if not (0 <= params['delay_before_stop'] <= 60):
        return False, "delay_before_stop must be between 0-60 seconds"
    
    if not (0 <= params['delay_after_stop'] <= 60):
        return False, "delay_after_stop must be between 0-60 seconds"
    
    if not (1000 <= params['pwm_start'] <= 2000):
        return False, "pwm_start must be between 1000-2000 microseconds"
    
    if not (1000 <= params['pwm_stop'] <= 2000):
        return False, "pwm_stop must be between 1000-2000 microseconds"
    
    return True, "Valid"
```

---

## 🔧 Flask API Endpoints

### **1. Start Mission Endpoint**

**Endpoint**: `POST /wp_mark/start`

**Request Body**:
```json
{
  "delay_before_start": 2.0,
  "pwm_start": 1500,
  "delay_before_stop": 5.0,
  "pwm_stop": 1000,
  "delay_after_stop": 1.0
}
```

**Success Response** (200 OK):
```json
{
  "success": true,
  "message": "WP_MARK mission started successfully",
  "config": {
    "delay_before_start": 2.0,
    "pwm_start": 1500,
    "delay_before_stop": 5.0,
    "pwm_stop": 1000,
    "delay_after_stop": 1.0
  },
  "mission_info": {
    "total_waypoints": 5,
    "estimated_duration_minutes": 15.5,
    "started_at": "2025-11-05T10:30:00Z"
  }
}
```

**Error Response** (400 Bad Request):
```json
{
  "success": false,
  "error": "Invalid parameter: delay_before_start must be between 0-60 seconds"
}
```

**Error Response** (409 Conflict):
```json
{
  "success": false,
  "error": "WP_MARK mission already running. Stop current mission first."
}
```

### **2. Stop Mission Endpoint**

**Endpoint**: `POST /wp_mark/stop`

**Request Body**: None

**Success Response** (200 OK):
```json
{
  "success": true,
  "message": "WP_MARK mission stopped successfully",
  "stats": {
    "waypoints_completed": 3,
    "total_waypoints": 5,
    "duration_seconds": 127.5
  }
}
```

### **3. Mission Status Endpoint**

**Endpoint**: `GET /wp_mark/status`

**Success Response** (200 OK):
```json
{
  "running": true,
  "current_waypoint": 3,
  "total_waypoints": 5,
  "current_phase": "spraying",  // waiting, navigating, spraying, idle
  "config": {
    "delay_before_start": 2.0,
    "pwm_start": 1500,
    "delay_before_stop": 5.0,
    "pwm_stop": 1000,
    "delay_after_stop": 1.0
  },
  "uptime_seconds": 87.3,
  "last_action": "Servo activated at WP 3"
}
```

---

## 🚀 Mission Execution Flow

### **High-Level Sequence**

```
START
  │
  ├─► Validate rover state (GPS fix, armed status)
  │
  ├─► Pull waypoints from flight controller
  │
  ├─► FOR EACH WAYPOINT (1 to N):
  │     │
  │     ├─► 1. NAVIGATE: Set current waypoint → Send to flight controller
  │     │
  │     ├─► 2. WAIT FOR ARRIVAL: Monitor GPS distance until < 2 meters
  │     │
  │     ├─► 3. DELAY BEFORE START: Sleep(delay_before_start)
  │     │
  │     ├─► 4. ACTIVATE SERVO: Send PWM command (pwm_start)
  │     │
  │     ├─► 5. SPRAYING DURATION: Sleep(delay_before_stop)
  │     │
  │     ├─► 6. DEACTIVATE SERVO: Send PWM command (pwm_stop)
  │     │
  │     ├─► 7. DELAY AFTER STOP: Sleep(delay_after_stop)
  │     │
  │     └─► 8. Continue to next waypoint
  │
  └─► END: Mission complete
```

### **Detailed State Machine**

```python
class MissionPhase(Enum):
    IDLE = "idle"
    INITIALIZING = "initializing"
    NAVIGATING = "navigating"
    WAITING_ARRIVAL = "waiting_arrival"
    DELAY_BEFORE_START = "delay_before_start"
    SPRAYING = "spraying"
    DELAY_AFTER_STOP = "delay_after_stop"
    COMPLETED = "completed"
    ERROR = "error"
```

---

## 🔌 ROS2 Integration

### **Required ROS2 Subscriptions**

```python
# GPS Position
self.gps_sub = self.create_subscription(
    NavSatFix,
    '/mavros/global_position/global',
    self.gps_callback,
    10
)

# Flight Controller State
self.state_sub = self.create_subscription(
    State,
    '/mavros/state',
    self.state_callback,
    10
)

# Waypoint List
self.waypoint_sub = self.create_subscription(
    WaypointList,
    '/mavros/mission/waypoints',
    self.waypoint_callback,
    10
)
```

### **Required ROS2 Service Clients**

```python
# Pull waypoints from flight controller
self.waypoint_pull_client = self.create_client(
    WaypointPull,
    '/mavros/mission/pull'
)

# Set current waypoint
self.waypoint_set_current_client = self.create_client(
    WaypointSetCurrent,
    '/mavros/mission/set_current'
)

# Command servo
self.command_long_client = self.create_client(
    CommandLong,
    '/mavros/cmd/command'
)
```

### **MAVLink Command for Servo Control**

```python
def set_servo(self, pwm_value: int, servo_channel: int = 10) -> bool:
    """
    Set servo PWM value using MAV_CMD_DO_SET_SERVO
    
    Args:
        pwm_value: PWM in microseconds (1000-2000)
        servo_channel: Servo output channel (default: 10)
    
    Returns:
        True if command successful
    """
    request = CommandLong.Request()
    request.command = 183  # MAV_CMD_DO_SET_SERVO
    request.param1 = float(servo_channel)
    request.param2 = float(pwm_value)
    
    future = self.command_long_client.call_async(request)
    rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
    
    return future.result() and future.result().success
```

---

## 📐 GPS Distance Calculation

### **Haversine Formula Implementation**

```python
import math

def calculate_gps_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Calculate distance between two GPS coordinates using Haversine formula
    
    Args:
        lat1, lon1: Current position (degrees)
        lat2, lon2: Target position (degrees)
    
    Returns:
        Distance in meters
    """
    R = 6371000  # Earth radius in meters
    
    # Convert to radians
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    
    # Haversine formula
    a = (math.sin(delta_phi / 2) ** 2 +
         math.cos(phi1) * math.cos(phi2) *
         math.sin(delta_lambda / 2) ** 2)
    
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    
    distance = R * c
    return distance

# Usage in waypoint arrival detection
WAYPOINT_ARRIVAL_THRESHOLD = 2.0  # meters

def has_reached_waypoint(current_gps, target_waypoint) -> bool:
    distance = calculate_gps_distance(
        current_gps.latitude,
        current_gps.longitude,
        target_waypoint.x_lat,
        target_waypoint.y_long
    )
    return distance < WAYPOINT_ARRIVAL_THRESHOLD
```

---

## 🛡️ Industrial-Grade Features

### **1. Error Handling**

```python
class WPMarkError(Exception):
    """Base exception for WP_MARK mission errors"""
    pass

class GPSFixLostError(WPMarkError):
    """Raised when GPS fix is lost during mission"""
    pass

class WaypointTimeoutError(WPMarkError):
    """Raised when waypoint arrival times out"""
    pass

class ServoCommandFailedError(WPMarkError):
    """Raised when servo command fails"""
    pass

# Timeout configuration
WAYPOINT_ARRIVAL_TIMEOUT = 300.0  # 5 minutes max per waypoint
GPS_FIX_REQUIRED = 3  # Minimum GPS fix type (3D fix)
```

### **2. Safety Checks**

```python
def check_mission_safety(self) -> tuple[bool, str]:
    """
    Perform pre-mission safety checks
    
    Returns:
        (is_safe, error_message)
    """
    # Check GPS fix
    if not self.current_gps or self.current_gps.status.status < GPS_FIX_REQUIRED:
        return False, "Insufficient GPS fix. Require 3D fix or better."
    
    # Check armed state
    if not self.current_state or not self.current_state.armed:
        return False, "Rover must be armed before starting mission."
    
    # Check mode
    if self.current_state.mode not in ['AUTO', 'GUIDED']:
        return False, f"Invalid mode: {self.current_state.mode}. Require AUTO or GUIDED."
    
    # Check waypoint count
    if len(self.waypoints) == 0:
        return False, "No waypoints loaded. Upload mission first."
    
    return True, "All safety checks passed"
```

### **3. Logging and Telemetry**

```python
import logging
from datetime import datetime

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
    handlers=[
        logging.FileHandler(f'wp_mark_{datetime.now().strftime("%Y%m%d_%H%M%S")}.log'),
        logging.StreamHandler()
    ]
)

logger = logging.getLogger('WPMarkMission')

# Log mission events
def log_mission_event(self, event_type: str, details: dict):
    """
    Log mission event with timestamp and GPS coordinates
    """
    log_entry = {
        'timestamp': datetime.now().isoformat(),
        'event': event_type,
        'waypoint': self.current_waypoint_index,
        'gps': {
            'lat': self.current_gps.latitude if self.current_gps else None,
            'lon': self.current_gps.longitude if self.current_gps else None,
            'alt': self.current_gps.altitude if self.current_gps else None
        },
        'details': details
    }
    
    logger.info(f"{event_type}: {details}")
    
    # Save to mission log file
    with open('mission_log.json', 'a') as f:
        f.write(json.dumps(log_entry) + '\n')
```

### **4. Mission Recovery**

```python
def save_mission_state(self):
    """Save current mission state for recovery"""
    state = {
        'current_waypoint': self.current_waypoint_index,
        'phase': self.current_phase.value,
        'config': self.config,
        'timestamp': datetime.now().isoformat()
    }
    
    with open('mission_state.json', 'w') as f:
        json.dump(state, f, indent=2)

def load_mission_state(self) -> dict:
    """Load saved mission state"""
    try:
        with open('mission_state.json', 'r') as f:
            return json.load(f)
    except FileNotFoundError:
        return None
```

---

## 💾 Configuration Storage

### **Config File Structure**

**File**: `config/wp_mark_config.json`

```json
{
  "wp_mark": {
    "enabled": true,
    "version": "1.0.0",
    "parameters": {
      "delay_before_start": 2.0,
      "pwm_start": 1500,
      "delay_before_stop": 5.0,
      "pwm_stop": 1000,
      "delay_after_stop": 1.0
    },
    "settings": {
      "servo_channel": 10,
      "waypoint_arrival_threshold": 2.0,
      "waypoint_timeout": 300.0,
      "gps_fix_required": 3,
      "log_enabled": true
    },
    "last_updated": "2025-11-05T10:30:00Z"
  }
}
```

---

## 🎯 Complete Mission Execution Algorithm

```python
async def execute_mission(self):
    """
    Main mission execution loop
    """
    try:
        # Phase 1: Initialization
        self.current_phase = MissionPhase.INITIALIZING
        logger.info("=== WP_MARK Mission Starting ===")
        
        # Safety checks
        is_safe, error_msg = self.check_mission_safety()
        if not is_safe:
            raise WPMarkError(f"Safety check failed: {error_msg}")
        
        # Pull waypoints
        logger.info("Pulling waypoints from flight controller...")
        if not await self.pull_waypoints():
            raise WPMarkError("Failed to retrieve waypoints")
        
        logger.info(f"Mission loaded: {len(self.waypoints)} waypoints")
        
        # Phase 2: Execute waypoint sequence
        for wp_index, waypoint in enumerate(self.waypoints):
            if not self.mission_active:
                logger.warning("Mission stopped by user")
                break
            
            self.current_waypoint_index = wp_index
            logger.info(f"\n{'='*60}")
            logger.info(f"Waypoint {wp_index + 1}/{len(self.waypoints)}")
            logger.info(f"{'='*60}")
            
            # Step 1: Navigate to waypoint
            self.current_phase = MissionPhase.NAVIGATING
            logger.info(f"Setting waypoint {wp_index}...")
            if not await self.goto_waypoint(wp_index):
                logger.error(f"Failed to set waypoint {wp_index}")
                continue
            
            # Step 2: Wait for arrival
            self.current_phase = MissionPhase.WAITING_ARRIVAL
            logger.info("Waiting for arrival...")
            if not await self.wait_for_arrival(waypoint):
                logger.error(f"Timeout waiting for waypoint {wp_index}")
                continue
            
            logger.info("✅ Waypoint reached!")
            self.log_mission_event('WAYPOINT_REACHED', {'waypoint': wp_index})
            
            # Step 3: Delay before start
            self.current_phase = MissionPhase.DELAY_BEFORE_START
            logger.info(f"Delay before start: {self.config['delay_before_start']}s")
            await asyncio.sleep(self.config['delay_before_start'])
            
            if not self.mission_active:
                break
            
            # Step 4: Activate servo (spray ON)
            self.current_phase = MissionPhase.SPRAYING
            logger.info(f"💧 Activating servo (PWM: {self.config['pwm_start']})")
            if not self.set_servo(self.config['pwm_start']):
                logger.error("Failed to activate servo")
                continue
            
            self.log_mission_event('SERVO_ON', {
                'pwm': self.config['pwm_start'],
                'waypoint': wp_index
            })
            
            # Step 5: Spraying duration
            logger.info(f"Spraying for {self.config['delay_before_stop']}s...")
            await asyncio.sleep(self.config['delay_before_stop'])
            
            if not self.mission_active:
                # Emergency stop: turn off servo
                self.set_servo(self.config['pwm_stop'])
                break
            
            # Step 6: Deactivate servo (spray OFF)
            logger.info(f"🛑 Deactivating servo (PWM: {self.config['pwm_stop']})")
            if not self.set_servo(self.config['pwm_stop']):
                logger.error("Failed to deactivate servo")
            
            self.log_mission_event('SERVO_OFF', {
                'pwm': self.config['pwm_stop'],
                'waypoint': wp_index
            })
            
            # Step 7: Delay after stop
            self.current_phase = MissionPhase.DELAY_AFTER_STOP
            logger.info(f"Delay after stop: {self.config['delay_after_stop']}s")
            await asyncio.sleep(self.config['delay_after_stop'])
        
        # Phase 3: Mission complete
        self.current_phase = MissionPhase.COMPLETED
        logger.info("\n✅ Mission Complete!")
        self.log_mission_event('MISSION_COMPLETE', {
            'total_waypoints': len(self.waypoints),
            'completed_waypoints': self.current_waypoint_index + 1
        })
        
    except Exception as e:
        self.current_phase = MissionPhase.ERROR
        logger.error(f"Mission failed: {str(e)}")
        self.log_mission_event('MISSION_ERROR', {'error': str(e)})
        raise
    
    finally:
        # Cleanup
        self.save_mission_state()
        self.mission_active = False
```

---

## 📡 Frontend API Call Function

### **TypeScript Implementation**

```typescript
// src/services/wpMarkService.ts

import { BACKEND_URL } from '../config';

export interface WPMarkConfig {
  delay_before_start: number;
  pwm_start: number;
  delay_before_stop: number;
  pwm_stop: number;
  delay_after_stop: number;
}

export interface WPMarkResponse {
  success: boolean;
  message?: string;
  error?: string;
  config?: WPMarkConfig;
  mission_info?: {
    total_waypoints: number;
    estimated_duration_minutes: number;
    started_at: string;
  };
}

export interface WPMarkStatus {
  running: boolean;
  current_waypoint: number;
  total_waypoints: number;
  current_phase: string;
  config: WPMarkConfig;
  uptime_seconds: number;
  last_action: string;
}

/**
 * Start WP_MARK mission with configuration
 */
export async function startWPMarkMission(config: WPMarkConfig): Promise<WPMarkResponse> {
  try {
    const response = await fetch(`${BACKEND_URL}/wp_mark/start`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(config),
    });

    const data = await response.json();

    if (!response.ok) {
      throw new Error(data.error || `HTTP ${response.status}: Failed to start mission`);
    }

    return data;
  } catch (error) {
    console.error('Failed to start WP_MARK mission:', error);
    throw error;
  }
}

/**
 * Stop currently running WP_MARK mission
 */
export async function stopWPMarkMission(): Promise<WPMarkResponse> {
  try {
    const response = await fetch(`${BACKEND_URL}/wp_mark/stop`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
    });

    const data = await response.json();

    if (!response.ok) {
      throw new Error(data.error || `HTTP ${response.status}: Failed to stop mission`);
    }

    return data;
  } catch (error) {
    console.error('Failed to stop WP_MARK mission:', error);
    throw error;
  }
}

/**
 * Get current WP_MARK mission status
 */
export async function getWPMarkStatus(): Promise<WPMarkStatus> {
  try {
    const response = await fetch(`${BACKEND_URL}/wp_mark/status`, {
      method: 'GET',
      headers: {
        'Content-Type': 'application/json',
      },
    });

    const data = await response.json();

    if (!response.ok) {
      throw new Error(data.error || `HTTP ${response.status}: Failed to get status`);
    }

    return data;
  } catch (error) {
    console.error('Failed to get WP_MARK status:', error);
    throw error;
  }
}

/**
 * Poll mission status at regular intervals
 */
export function pollWPMarkStatus(
  onUpdate: (status: WPMarkStatus) => void,
  intervalMs: number = 2000
): () => void {
  const intervalId = setInterval(async () => {
    try {
      const status = await getWPMarkStatus();
      onUpdate(status);
    } catch (error) {
      console.error('Status polling error:', error);
    }
  }, intervalMs);

  // Return cleanup function
  return () => clearInterval(intervalId);
}
```

---

## 📦 File Structure

```
backend/
├── app.py                          # Flask application
├── wp_mark/
│   ├── __init__.py
│   ├── mission_controller.py      # ROS2 mission node
│   ├── api_routes.py              # Flask routes
│   ├── validators.py              # Parameter validation
│   └── utils.py                   # Helper functions
├── config/
│   └── wp_mark_config.json        # Configuration storage
└── logs/
    ├── wp_mark_YYYYMMDD_HHMMSS.log
    └── mission_log.json
```

---

## ✅ Implementation Checklist

### **Backend Tasks**

- [ ] Create Flask API endpoints (`/wp_mark/start`, `/wp_mark/stop`, `/wp_mark/status`)
- [ ] Implement parameter validation function
- [ ] Create ROS2 WP_MARK node class
- [ ] Implement waypoint pull service call
- [ ] Implement GPS distance calculation
- [ ] Implement servo control via MAVLink command
- [ ] Add safety checks (GPS fix, armed state, mode)
- [ ] Implement mission state machine
- [ ] Add error handling and recovery
- [ ] Implement logging and telemetry
- [ ] Create config file storage system
- [ ] Add mission state persistence
- [ ] Test waypoint navigation
- [ ] Test servo control
- [ ] Test complete mission sequence

### **Frontend Tasks**

- [x] Create WPMarkDialog component
- [x] Add WP_MARK to ModeSelector dropdown
- [x] Implement parameter input validation
- [x] Fix dialog z-index (appears above map)
- [ ] Create wpMarkService.ts API functions
- [ ] Add mission status polling
- [ ] Display mission progress in UI
- [ ] Add stop mission button
- [ ] Show mission logs

---

## 🎯 Expected Behavior

### **User Workflow**

1. User selects **WP_MARK** from mode dropdown
2. User clicks **Set** button
3. Parameter dialog appears
4. User enters 5 parameters
5. User clicks **Save & Start Mission**
6. Frontend calls `POST /wp_mark/start` with parameters
7. Backend validates parameters
8. Backend saves config to JSON file
9. Backend launches ROS2 mission node
10. Mission executes waypoint sequence
11. Frontend shows success message
12. User can monitor status via polling

### **Mission Execution**

```
WP 1: Navigate → Arrive → Wait 2s → Spray ON → Spray 5s → Spray OFF → Wait 1s
WP 2: Navigate → Arrive → Wait 2s → Spray ON → Spray 5s → Spray OFF → Wait 1s
WP 3: Navigate → Arrive → Wait 2s → Spray ON → Spray 5s → Spray OFF → Wait 1s
...
WP N: Navigate → Arrive → Wait 2s → Spray ON → Spray 5s → Spray OFF → Wait 1s
DONE: Mission Complete
```

---

## 🔒 Security and Safety

1. **Parameter Validation**: All inputs validated before execution
2. **GPS Requirements**: Minimum 3D GPS fix required
3. **Armed Check**: Rover must be armed before mission
4. **Mode Check**: Rover must be in AUTO or GUIDED mode
5. **Timeout Protection**: Waypoint arrival timeout prevents infinite loops
6. **Emergency Stop**: Mission can be stopped at any time
7. **State Persistence**: Mission state saved for recovery
8. **Complete Logging**: All events logged with GPS coordinates

---

## 📊 Performance Metrics

- **Waypoint Arrival Threshold**: 2.0 meters
- **Waypoint Timeout**: 300 seconds (5 minutes)
- **Servo Command Timeout**: 2 seconds
- **Status Update Frequency**: 2 seconds
- **GPS Fix Requirement**: Type 3 (3D fix) or better

---

## 🎉 Success Criteria

✅ Parameters validated correctly  
✅ Mission starts without errors  
✅ Waypoints navigated in sequence  
✅ Servo activates at correct times  
✅ GPS position accurate within threshold  
✅ All events logged with timestamps  
✅ Mission completes successfully  
✅ Emergency stop works immediately  
✅ Frontend receives status updates  
✅ Config persists between restarts  

---

**This specification provides everything needed to implement a production-ready, industrial-grade WP_MARK mission system.**
