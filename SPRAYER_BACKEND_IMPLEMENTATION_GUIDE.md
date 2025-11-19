# Sprayer Mission Control Backend Implementation Guide

## Overview

The Sprayer Mission Control system provides a complete frontend interface for managing agricultural spraying missions using ROS2 and MAVROS. This document outlines the backend implementation requirements to support the SprayerTab frontend component.

## Frontend Structure and Workflow

### Component Architecture

The SprayerTab consists of the following components:

1. **SprayerTab** (Main Container)
   - Manages overall state and coordinates between components
   - Focuses on mission control and configuration

2. **MissionControl**
   - Start/Stop mission buttons with status display
   - API calls to `/api/sprayer/start` and `/api/sprayer/stop`

3. **MissionConsole**
   - WebSocket connection to `/api/sprayer/logs`
   - Real-time mission logging and status updates

4. **MissionConfigPanel**
   - User input panel for mission parameters
   - Servo settings (number, PWM values)
   - Timing configurations (delays and durations)
   - Saves configuration to backend via `/api/sprayer/config`

**Plan Tab Integration:**
The Plan tab (Edit Plan mode) now includes jetson upload functionality in the Rover Control panel alongside existing rover upload/download buttons. This consolidates all upload operations in one location.

### Frontend Workflow

1. **Configuration Setup**
   - User configures mission parameters in MissionConfigPanel
   - Settings include servo number, PWM values, and timing delays
   - Configuration saved to backend via POST `/api/sprayer/config`

2. **Waypoint Management & Upload**
   - User creates or imports waypoints in Plan tab
   - User clicks "Upload to Jetson" button in Plan tab Rover Control panel
   - Backend stores waypoints for mission execution

3. **Mission Control**
   - User switches to Sprayer tab and clicks "Start Mission" → POST `/api/sprayer/start`
   - Backend begins ROS2 mission execution using saved configuration and waypoints
   - Real-time logs sent via WebSocket
   - User can stop mission → POST `/api/sprayer/stop`

4. **Live Monitoring**
   - WebSocket connection maintains live log feed
   - Mission progress and status updates displayed
   - Connection status indicator

## API Endpoints Specification

### 1. POST `/api/sprayer/upload`
**Purpose:** Upload waypoints file to Jetson for mission execution

**Request:**
- Content-Type: `multipart/form-data`
- Body: FormData with `file` field containing CSV/waypoint file

**Response:**
```json
{
  "success": true,
  "message": "Waypoints uploaded successfully",
  "waypoint_count": 15
}
```

**Error Response:**
```json
{
  "success": false,
  "message": "Invalid file format"
}
```

### 2. POST `/api/sprayer/start`
**Purpose:** Start the spray mission execution

**Request:**
- Content-Type: `application/json`
- Body: `{}` (empty for now, can add config later)

**Response:**
```json
{
  "success": true,
  "message": "Mission started successfully"
}
```

### 3. POST `/api/sprayer/stop`
**Purpose:** Stop the current mission

**Request:**
- Content-Type: `application/json`
- Body: `{}`

**Response:**
```json
{
  "success": true,
  "message": "Mission stopped successfully"
}
```

### 4. GET `/api/sprayer/status`
**Purpose:** Get current mission status

**Response:**
```json
{
  "status": "idle|running|stopped|error",
  "current_waypoint": 3,
  "total_waypoints": 15,
  "message": "Mission in progress"
}
```

### 5. WebSocket `/api/sprayer/logs`
**Purpose:** Real-time mission logging

**Message Format:**
```json
{
  "timestamp": "2025-11-08T10:30:45.123Z",
  "level": "info|warn|error",
  "message": "Rover connected, mission starting",
  "waypoint_id": 1
}
```

### 6. GET `/api/sprayer/config`
**Purpose:** Get current mission configuration parameters

**Response:**
```json
{
  "servoNumber": 9,
  "startPwm": 1900,
  "stopPwm": 1100,
  "beforeSprayDelay": 2.0,
  "sprayDuration": 5.0,
  "afterSprayDelay": 1.0
}
```

### 7. POST `/api/sprayer/config`
**Purpose:** Save mission configuration parameters

**Request:**
```json
{
  "servoNumber": 9,
  "startPwm": 1900,
  "stopPwm": 1100,
  "beforeSprayDelay": 2.0,
  "sprayDuration": 5.0,
  "afterSprayDelay": 1.0
}
```

**Response:**
```json
{
  "success": true,
  "message": "Configuration saved successfully"
}
```

## Backend Implementation Requirements

### 1. File Upload Handler

```python
@app.route('/api/sprayer/upload', methods=['POST'])
def upload_waypoints():
    if 'file' not in request.files:
        return jsonify({'success': False, 'message': 'No file provided'})

    file = request.files['file']
    if file.filename == '':
        return jsonify({'success': False, 'message': 'No file selected'})

    # Validate file extension
    allowed_extensions = {'csv', 'waypoint', 'waypoints', 'json', 'dxf', 'kml'}
    if not allowed_file(file.filename, allowed_extensions):
        return jsonify({'success': False, 'message': 'Invalid file type'})

    # Parse and store waypoints
    waypoints = parse_waypoints_file(file)
    store_waypoints(waypoints)

    return jsonify({
        'success': True,
        'message': 'Waypoints uploaded successfully',
        'waypoint_count': len(waypoints)
    })
```

### 2. Mission Control Handlers

```python
@app.route('/api/sprayer/start', methods=['POST'])
def start_mission():
    global mission_active, mission_thread

    if mission_active:
        return jsonify({'success': False, 'message': 'Mission already running'})

    waypoints = load_stored_waypoints()
    if not waypoints:
        return jsonify({'success': False, 'message': 'No waypoints loaded'})

    mission_active = True
    # Pass current configuration to the mission thread
    mission_thread = threading.Thread(target=execute_mission, args=(waypoints, mission_config))
    mission_thread.start()

    return jsonify({'success': True, 'message': 'Mission started'})

@app.route('/api/sprayer/stop', methods=['POST'])
def stop_mission():
    global mission_active
    mission_active = False
    # Signal ROS2 node to stop
    return jsonify({'success': True, 'message': 'Mission stopping'})
```

### 3. Status Handler

```python
@app.route('/api/sprayer/status', methods=['GET'])
def get_status():
    return jsonify({
        'status': 'running' if mission_active else 'idle',
        'current_waypoint': current_waypoint_index,
        'total_waypoints': len(stored_waypoints),
        'message': get_current_status_message()
    })
```

### 4. Configuration Handlers

```python
# Global configuration storage
mission_config = {
    'servoNumber': 9,
    'startPwm': 1900,
    'stopPwm': 1100,
    'beforeSprayDelay': 2.0,
    'sprayDuration': 5.0,
    'afterSprayDelay': 1.0
}

@app.route('/api/sprayer/config', methods=['GET'])
def get_config():
    """Get current mission configuration"""
    return jsonify(mission_config)

@app.route('/api/sprayer/config', methods=['POST'])
def save_config():
    """Save mission configuration parameters"""
    global mission_config

    try:
        data = request.get_json()

        # Validate required fields
        required_fields = ['servoNumber', 'startPwm', 'stopPwm', 'beforeSprayDelay', 'sprayDuration', 'afterSprayDelay']
        for field in required_fields:
            if field not in data:
                return jsonify({'success': False, 'message': f'Missing required field: {field}'})

        # Validate ranges
        if not (1 <= data['servoNumber'] <= 16):
            return jsonify({'success': False, 'message': 'Servo number must be between 1 and 16'})

        if not (1000 <= data['startPwm'] <= 2000) or not (1000 <= data['stopPwm'] <= 2000):
            return jsonify({'success': False, 'message': 'PWM values must be between 1000 and 2000'})

        if not (0 <= data['beforeSprayDelay'] <= 10) or not (0.1 <= data['sprayDuration'] <= 30) or not (0 <= data['afterSprayDelay'] <= 10):
            return jsonify({'success': False, 'message': 'Delay values out of valid range'})

        # Save configuration
        mission_config.update(data)

        # Optionally save to file for persistence
        save_config_to_file(mission_config)

        return jsonify({'success': True, 'message': 'Configuration saved successfully'})

    except Exception as e:
        return jsonify({'success': False, 'message': f'Error saving configuration: {str(e)}'})

def save_config_to_file(config):
    """Save configuration to persistent storage"""
    import json
    try:
        with open('sprayer_config.json', 'w') as f:
            json.dump(config, f, indent=2)
    except Exception as e:
        print(f"Failed to save config to file: {e}")

def load_config_from_file():
    """Load configuration from persistent storage"""
    global mission_config
    import json
    try:
        with open('sprayer_config.json', 'r') as f:
            saved_config = json.load(f)
            mission_config.update(saved_config)
    except FileNotFoundError:
        pass  # Use defaults
    except Exception as e:
        print(f"Failed to load config from file: {e}")

def execute_mission(waypoints, config):
    """Execute mission using ROS2 node with provided configuration"""
    import rclpy
    from spray_mission_node import SprayMissionNode

    # Initialize ROS2
    rclpy.init()

    try:
        # Create node with configuration
        node = SprayMissionNode(config)

        # Execute the mission
        node.execute_mission(waypoints)

    except Exception as e:
        print(f"Mission execution failed: {e}")
    finally:
        rclpy.shutdown()
```
```

## ROS2 Node Implementation

### Node Structure

Create a new ROS2 package: `sprayer_mission_node`

```python
# sprayer_mission_node/spray_mission_node.py
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode, CommandLong
from mavros_msgs.msg import WaypointList, Waypoint
from sensor_msgs.msg import NavSatFix
import time
import threading

class SprayMissionNode(Node):
    def __init__(self, config=None):
        super().__init__('spray_mission_node')

        # Load configuration parameters
        self.config = config or {
            'servoNumber': 9,
            'startPwm': 1900,
            'stopPwm': 1100,
            'beforeSprayDelay': 2.0,
            'sprayDuration': 5.0,
            'afterSprayDelay': 1.0
        }

        # MAVROS service clients
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.command_client = self.create_client(CommandLong, '/mavros/cmd/command')

        # Publishers for logging
        self.log_publisher = self.create_publisher(String, '/sprayer/logs', 10)

        # Mission state
        self.mission_active = False
        self.current_waypoint = 0
        self.waypoints = []

    def log_message(self, message, level='info'):
        """Send log message to frontend via ROS2 topic"""
        log_msg = String()
        log_msg.data = f'[{level.upper()}] {message}'
        self.log_publisher.publish(log_msg)

        # Also send to WebSocket (implement WebSocket bridge)
        websocket_broadcast({
            'timestamp': time.time(),
            'level': level,
            'message': message,
            'waypoint_id': self.current_waypoint
        })

    def execute_mission(self, waypoints):
        """Main mission execution logic"""
        self.waypoints = waypoints
        self.mission_active = True
        self.current_waypoint = 0

        try:
            self.log_message("Rover connected, mission starting")

            # 1. Set mode to GUIDED
            if not self.set_mode('GUIDED'):
                raise Exception("Failed to set GUIDED mode")

            self.log_message("Mode set to GUIDED")

            # 2. Execute waypoint sequence
            for i, wp in enumerate(self.waypoints):
                if not self.mission_active:
                    break

                self.current_waypoint = i + 1
                self.log_message(f"Navigating to waypoint {self.current_waypoint}")

                # Send go-to command
                if not self.goto_waypoint(wp):
                    self.log_message(f"Failed to navigate to waypoint {self.current_waypoint}", 'error')
                    continue

                # Wait for arrival
                if not self.wait_for_arrival(wp):
                    self.log_message(f"Failed to reach waypoint {self.current_waypoint}", 'error')
                    continue

                self.log_message(f"Arrived at waypoint {self.current_waypoint}")

                # Set mode to HOLD
                self.set_mode('HOLD')

                # Wait for stabilization delay (before spray delay)
                time.sleep(self.config['beforeSprayDelay'])

                # Start spraying
                self.set_servo_pwm(self.config['startPwm'], self.config['servoNumber'])
                self.log_message(f"Started spraying at waypoint {self.current_waypoint}")

                # Wait for spray duration
                time.sleep(self.config['sprayDuration'])

                # Stop spraying
                self.set_servo_pwm(self.config['stopPwm'], self.config['servoNumber'])
                self.log_message(f"Stopped spraying at waypoint {self.current_waypoint}")

                # Wait for after-spray delay
                time.sleep(self.config['afterSprayDelay'])

                # Set back to GUIDED for next waypoint
                self.set_mode('GUIDED')

            self.log_message("Mission completed successfully")

        except Exception as e:
            self.log_message(f"Mission failed: {str(e)}", 'error')

        finally:
            self.mission_active = False
            self.set_mode('HOLD')  # Safe mode

    def set_mode(self, mode):
        """Set MAVROS flight mode"""
        req = SetMode.Request()
        req.custom_mode = mode

        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        return future.result().mode_sent if future.done() else False

    def goto_waypoint(self, waypoint):
        """Send go-to waypoint command"""
        # Implementation depends on your MAVROS setup
        # Could use /mavros/mission/waypoint or direct position commands
        pass

    def wait_for_arrival(self, waypoint, timeout=30.0):
        """Wait for rover to arrive at waypoint"""
        # Monitor position and check distance
        pass

    def set_servo_pwm(self, pwm_value, servo_number=None):
        """Set servo PWM for spray control"""
        req = CommandLong.Request()
        req.command = 183  # MAV_CMD_DO_SET_SERVO
        req.param1 = servo_number or self.config['servoNumber']  # Servo channel
        req.param2 = pwm_value

        future = self.command_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        return future.result().success if future.done() else False
```

## Configuration Parameters

Mission configuration is now managed through the frontend MissionConfigPanel and stored via the `/api/sprayer/config` endpoints. The configuration includes:

- **servoNumber**: Servo channel number (1-16, default: 9)
- **startPwm**: PWM value to start spraying (1000-2000μs, default: 1900)
- **stopPwm**: PWM value to stop spraying (1000-2000μs, default: 1100)
- **beforeSprayDelay**: Delay after arriving at waypoint before spraying (0-10s, default: 2.0)
- **sprayDuration**: How long to spray at each waypoint (0.1-30s, default: 5.0)
- **afterSprayDelay**: Delay after stopping spray before moving to next waypoint (0-10s, default: 1.0)

Configuration is automatically loaded on startup and can be modified through the frontend interface. Changes are persisted to `sprayer_config.json` for durability across restarts.

## WebSocket Integration

Implement WebSocket server for real-time logging:

```python
# websocket_server.py
import asyncio
import websockets
import json

connected_clients = set()

async def broadcast_log(log_data):
    """Broadcast log message to all connected clients"""
    message = json.dumps(log_data)
    await asyncio.gather(
        *[client.send(message) for client in connected_clients],
        return_exceptions=True
    )

async def websocket_handler(websocket, path):
    """Handle WebSocket connections"""
    connected_clients.add(websocket)
    try:
        await websocket.wait_closed()
    finally:
        connected_clients.remove(websocket)

# Start WebSocket server
start_server = websockets.serve(websocket_handler, "localhost", 8765)

# Initialize configuration on startup
load_config_from_file()

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)
```

## Integration Steps

1. **Create ROS2 Package**
   ```bash
   ros2 pkg create --build-type ament_python sprayer_mission_node
   ```

2. **Add Dependencies**
   - MAVROS packages
   - Flask for HTTP API
   - websockets for real-time logging

3. **Launch Configuration**
   ```xml
   <!-- launch/spray_mission.launch.py -->
   from launch import LaunchDescription
   from launch_ros.actions import Node

   def generate_launch_description():
       return LaunchDescription([
           Node(
               package='sprayer_mission_node',
               executable='spray_mission_node',
               name='spray_mission_node',
           ),
       ])
   ```

4. **Testing**
   - Test waypoint upload with sample CSV
   - Verify MAVROS connection
   - Test mission start/stop commands
   - Monitor WebSocket log streaming

## Error Handling

- Validate waypoint coordinates before mission start
- Handle MAVROS service timeouts
- Provide detailed error messages to frontend
- Implement mission recovery mechanisms
- Log all errors with timestamps

## Security Considerations

- Validate uploaded files for malicious content
- Implement rate limiting on API endpoints
- Use secure WebSocket connections in production
- Validate all input parameters

This implementation provides a complete backend system that integrates with the SprayerTab frontend, offering full mission control capabilities with real-time monitoring and ROS2/MAVROS integration.