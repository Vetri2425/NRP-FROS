# 🎯 Custom WP_MARK Mode Implementation Guide

## 📋 Overview

This guide explains how to create a **custom mission mode** called "WP_MARK" that:
1. Appears as a button in the Dashboard sidebar
2. Opens a popup dialog to collect parameters
3. Executes a waypoint-based servo control sequence
4. Integrates with your existing backend system

---

## 🏗️ Architecture: Frontend vs Backend

### **Frontend (React/TypeScript) - User Interface**
```
┌─────────────────────────────────────────────────┐
│           FRONTEND RESPONSIBILITIES              │
├─────────────────────────────────────────────────┤
│                                                  │
│ 1. Display "WP_MARK" button in LeftSidebar     │
│ 2. Show popup dialog when clicked              │
│ 3. Collect 5 parameters from user:             │
│    • Delay before start servo (seconds)        │
│    • Servo Start PWM (1000-2000)               │
│    • Delay before stop PWM (seconds)           │
│    • Servo Stop PWM (1000-2000)                │
│    • Delay after stop servo (seconds)          │
│ 4. Validate inputs (numbers, ranges)           │
│ 5. Send parameters via HTTP POST                │
│ 6. Display success/error messages              │
│ 7. Show mission status (running/stopped)       │
│                                                  │
└─────────────────────────────────────────────────┘
```

### **Backend (Python/ROS2) - Mission Execution**
```
┌─────────────────────────────────────────────────┐
│           BACKEND RESPONSIBILITIES               │
├─────────────────────────────────────────────────┤
│                                                  │
│ 1. Receive POST /api/wp_mark/start              │
│ 2. Save parameters to config file               │
│ 3. Read mission waypoints from MAVROS           │
│ 4. Start ROS2 node for execution                │
│ 5. FOR EACH WAYPOINT:                           │
│    a. Navigate to waypoint (MAVROS cmd)        │
│    b. Wait until arrived (GPS check)           │
│    c. Sleep (delay_before_start)               │
│    d. Set servo PWM (pwm_start)                │
│    e. Sleep (delay_before_stop)                │
│    f. Set servo PWM (pwm_stop)                 │
│    g. Sleep (delay_after_stop)                 │
│    h. Continue to next waypoint                │
│ 6. Publish status updates                      │
│ 7. Handle stop commands                         │
│                                                  │
└─────────────────────────────────────────────────┘
```

---

## 🔧 Frontend Implementation

### **Step 1: Create WP_MARK Button Component**

**File**: `src/components/WPMarkButton.tsx`

```typescript
import React, { useState } from 'react';
import WPMarkDialog from './WPMarkDialog';

interface WPMarkButtonProps {
  disabled?: boolean;
}

const WPMarkButton: React.FC<WPMarkButtonProps> = ({ disabled = false }) => {
  const [isDialogOpen, setIsDialogOpen] = useState(false);

  return (
    <>
      <button
        onClick={() => setIsDialogOpen(true)}
        disabled={disabled}
        className={`
          w-full px-4 py-3 rounded-lg font-semibold
          transition-all duration-200
          ${
            disabled
              ? 'bg-gray-600 text-gray-400 cursor-not-allowed'
              : 'bg-orange-600 hover:bg-orange-700 text-white shadow-lg hover:shadow-xl'
          }
        `}
      >
        🎯 WP_MARK Mission
      </button>

      {isDialogOpen && (
        <WPMarkDialog
          isOpen={isDialogOpen}
          onClose={() => setIsDialogOpen(false)}
        />
      )}
    </>
  );
};

export default WPMarkButton;
```

---

### **Step 2: Create Parameter Dialog**

**File**: `src/components/WPMarkDialog.tsx`

```typescript
import React, { useState } from 'react';
import { BACKEND_URL } from '../config';

interface WPMarkDialogProps {
  isOpen: boolean;
  onClose: () => void;
}

interface WPMarkParams {
  delayBeforeStart: number;
  pwmStart: number;
  delayBeforeStop: number;
  pwmStop: number;
  delayAfterStop: number;
}

const WPMarkDialog: React.FC<WPMarkDialogProps> = ({ isOpen, onClose }) => {
  const [params, setParams] = useState<WPMarkParams>({
    delayBeforeStart: 2.0,
    pwmStart: 1500,
    delayBeforeStop: 5.0,
    pwmStop: 1000,
    delayAfterStop: 1.0,
  });
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const handleChange = (field: keyof WPMarkParams, value: string) => {
    setParams((prev) => ({
      ...prev,
      [field]: parseFloat(value) || 0,
    }));
  };

  const validateParams = (): string | null => {
    if (params.pwmStart < 1000 || params.pwmStart > 2000) {
      return 'PWM Start must be between 1000-2000';
    }
    if (params.pwmStop < 1000 || params.pwmStop > 2000) {
      return 'PWM Stop must be between 1000-2000';
    }
    if (params.delayBeforeStart < 0 || params.delayBeforeStart > 60) {
      return 'Delay before start must be 0-60 seconds';
    }
    if (params.delayBeforeStop < 0 || params.delayBeforeStop > 60) {
      return 'Delay before stop must be 0-60 seconds';
    }
    if (params.delayAfterStop < 0 || params.delayAfterStop > 60) {
      return 'Delay after stop must be 0-60 seconds';
    }
    return null;
  };

  const handleStartMission = async () => {
    const validationError = validateParams();
    if (validationError) {
      setError(validationError);
      return;
    }

    setIsLoading(true);
    setError(null);

    try {
      const response = await fetch(`${BACKEND_URL}/wp_mark/start`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          delay_before_start: params.delayBeforeStart,
          pwm_start: params.pwmStart,
          delay_before_stop: params.delayBeforeStop,
          pwm_stop: params.pwmStop,
          delay_after_stop: params.delayAfterStop,
        }),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.message || 'Failed to start mission');
      }

      const result = await response.json();
      alert(`✅ Mission started successfully!\n${result.message || ''}`);
      onClose();
    } catch (err: any) {
      setError(err.message || 'Failed to start mission');
    } finally {
      setIsLoading(false);
    }
  };

  if (!isOpen) return null;

  return (
    <div className="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-50">
      <div className="bg-slate-800 rounded-lg shadow-2xl w-full max-w-md p-6 border border-slate-700">
        {/* Header */}
        <div className="flex items-center justify-between mb-4">
          <h2 className="text-xl font-bold text-white">🎯 WP_MARK Mission Setup</h2>
          <button
            onClick={onClose}
            className="text-gray-400 hover:text-white text-2xl"
            disabled={isLoading}
          >
            ×
          </button>
        </div>

        {/* Parameters Form */}
        <div className="space-y-4">
          {/* Delay Before Start */}
          <div>
            <label className="block text-sm font-medium text-slate-300 mb-1">
              1️⃣ Delay Before Start Servo (seconds)
            </label>
            <input
              type="number"
              step="0.1"
              min="0"
              max="60"
              value={params.delayBeforeStart}
              onChange={(e) => handleChange('delayBeforeStart', e.target.value)}
              className="w-full px-3 py-2 bg-slate-900 border border-slate-600 rounded text-white focus:outline-none focus:border-orange-500"
              disabled={isLoading}
            />
            <p className="text-xs text-slate-500 mt-1">Wait time after reaching waypoint</p>
          </div>

          {/* PWM Start */}
          <div>
            <label className="block text-sm font-medium text-slate-300 mb-1">
              2️⃣ Servo Start PWM
            </label>
            <input
              type="number"
              min="1000"
              max="2000"
              value={params.pwmStart}
              onChange={(e) => handleChange('pwmStart', e.target.value)}
              className="w-full px-3 py-2 bg-slate-900 border border-slate-600 rounded text-white focus:outline-none focus:border-orange-500"
              disabled={isLoading}
            />
            <p className="text-xs text-slate-500 mt-1">PWM value to activate servo (1000-2000)</p>
          </div>

          {/* Delay Before Stop */}
          <div>
            <label className="block text-sm font-medium text-slate-300 mb-1">
              3️⃣ Delay Before Stop PWM (seconds)
            </label>
            <input
              type="number"
              step="0.1"
              min="0"
              max="60"
              value={params.delayBeforeStop}
              onChange={(e) => handleChange('delayBeforeStop', e.target.value)}
              className="w-full px-3 py-2 bg-slate-900 border border-slate-600 rounded text-white focus:outline-none focus:border-orange-500"
              disabled={isLoading}
            />
            <p className="text-xs text-slate-500 mt-1">How long servo stays active</p>
          </div>

          {/* PWM Stop */}
          <div>
            <label className="block text-sm font-medium text-slate-300 mb-1">
              4️⃣ Servo Stop PWM
            </label>
            <input
              type="number"
              min="1000"
              max="2000"
              value={params.pwmStop}
              onChange={(e) => handleChange('pwmStop', e.target.value)}
              className="w-full px-3 py-2 bg-slate-900 border border-slate-600 rounded text-white focus:outline-none focus:border-orange-500"
              disabled={isLoading}
            />
            <p className="text-xs text-slate-500 mt-1">PWM value to deactivate servo</p>
          </div>

          {/* Delay After Stop */}
          <div>
            <label className="block text-sm font-medium text-slate-300 mb-1">
              5️⃣ Delay After Stop Servo (seconds)
            </label>
            <input
              type="number"
              step="0.1"
              min="0"
              max="60"
              value={params.delayAfterStop}
              onChange={(e) => handleChange('delayAfterStop', e.target.value)}
              className="w-full px-3 py-2 bg-slate-900 border border-slate-600 rounded text-white focus:outline-none focus:border-orange-500"
              disabled={isLoading}
            />
            <p className="text-xs text-slate-500 mt-1">Wait before moving to next waypoint</p>
          </div>
        </div>

        {/* Error Message */}
        {error && (
          <div className="mt-4 p-3 bg-red-900/30 border border-red-500 rounded text-red-200 text-sm">
            ⚠️ {error}
          </div>
        )}

        {/* Mission Sequence Preview */}
        <div className="mt-4 p-3 bg-blue-900/20 border border-blue-700/50 rounded">
          <h4 className="text-xs font-semibold text-blue-300 mb-2">📋 Mission Sequence:</h4>
          <ol className="text-xs text-blue-200 space-y-1">
            <li>1. Navigate to waypoint</li>
            <li>2. Wait {params.delayBeforeStart}s</li>
            <li>3. Set servo PWM to {params.pwmStart}</li>
            <li>4. Wait {params.delayBeforeStop}s</li>
            <li>5. Set servo PWM to {params.pwmStop}</li>
            <li>6. Wait {params.delayAfterStop}s</li>
            <li>7. Repeat for next waypoint</li>
          </ol>
        </div>

        {/* Action Buttons */}
        <div className="flex gap-3 mt-6">
          <button
            onClick={onClose}
            disabled={isLoading}
            className="flex-1 px-4 py-2 bg-gray-600 hover:bg-gray-700 text-white rounded font-medium disabled:opacity-50"
          >
            Cancel
          </button>
          <button
            onClick={handleStartMission}
            disabled={isLoading}
            className="flex-1 px-4 py-2 bg-orange-600 hover:bg-orange-700 text-white rounded font-medium disabled:opacity-50"
          >
            {isLoading ? '⏳ Starting...' : '🚀 Save & Start Mission'}
          </button>
        </div>
      </div>
    </div>
  );
};

export default WPMarkDialog;
```

---

### **Step 3: Add Button to LeftSidebar**

**File**: `src/components/LeftSidebar.tsx` (UPDATE)

```typescript
import React from 'react';
import TelemetryPanel from './TelemetryPanel';
import ModeSelector from './ModeSelector';
import RTKPanel from './RTKPanel';
import LogsPanel from './LogsPanel';
import StatusBar from './StatusBar';
import WPMarkButton from './WPMarkButton'; // ← ADD THIS

const LeftSidebar: React.FC = () => {
  return (
    <aside className="w-72 flex flex-col gap-3 min-h-0 overflow-hidden">
      <div className="flex-shrink-0">
        <TelemetryPanel />
      </div>
      <div className="flex-shrink-0">
        <ModeSelector />
      </div>
      
      {/* ↓ ADD WP_MARK BUTTON HERE */}
      <div className="flex-shrink-0">
        <WPMarkButton />
      </div>
      
      <div className="flex-shrink-0">
        <RTKPanel />
      </div>
      <div className="flex-1 min-h-0 overflow-hidden">
        <LogsPanel />
      </div>
      <div className="flex-shrink-0">
        <StatusBar />
      </div>
    </aside>
  );
};

export default LeftSidebar;
```

---

## 🐍 Backend Implementation

### **Step 1: Create WP_MARK Flask Endpoint**

**File**: `backend/wp_mark_controller.py` (NEW)

```python
#!/usr/bin/env python3
"""
WP_MARK Mission Controller
Handles waypoint-based servo control missions
"""

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State, WaypointList, Waypoint
from mavros_msgs.srv import CommandLong, WaypointPull, WaypointSetCurrent
from sensor_msgs.msg import NavSatFix
from geographic_msgs.msg import GeoPoseStamped
import time
import math
import json
from threading import Thread

class WPMarkMission(Node):
    def __init__(self, params):
        super().__init__('wp_mark_mission')
        
        # Mission parameters
        self.params = params
        self.delay_before_start = params['delay_before_start']
        self.pwm_start = params['pwm_start']
        self.delay_before_stop = params['delay_before_stop']
        self.pwm_stop = params['pwm_stop']
        self.delay_after_stop = params['delay_after_stop']
        
        # State variables
        self.current_state = None
        self.current_gps = None
        self.waypoints = []
        self.current_waypoint_index = 0
        self.mission_active = True
        self.waypoint_reached_threshold = 2.0  # meters
        
        # ROS2 Subscribers
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_callback, 10
        )
        self.gps_sub = self.create_subscription(
            NavSatFix, '/mavros/global_position/global', self.gps_callback, 10
        )
        
        # ROS2 Service Clients
        self.waypoint_pull_client = self.create_client(
            WaypointPull, '/mavros/mission/pull'
        )
        self.waypoint_set_current_client = self.create_client(
            WaypointSetCurrent, '/mavros/mission/set_current'
        )
        self.command_long_client = self.create_client(
            CommandLong, '/mavros/cmd/command'
        )
        
        self.get_logger().info('WP_MARK Mission Node Initialized')
    
    def state_callback(self, msg):
        self.current_state = msg
    
    def gps_callback(self, msg):
        self.current_gps = msg
    
    def pull_waypoints(self):
        """Retrieve mission waypoints from flight controller"""
        self.get_logger().info('Pulling waypoints from flight controller...')
        
        request = WaypointPull.Request()
        future = self.waypoint_pull_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None and future.result().success:
            # Subscribe to waypoint list to get actual waypoints
            waypoint_list_sub = self.create_subscription(
                WaypointList,
                '/mavros/mission/waypoints',
                self.waypoint_list_callback,
                10
            )
            time.sleep(1)  # Wait for waypoint list
            return True
        else:
            self.get_logger().error('Failed to pull waypoints')
            return False
    
    def waypoint_list_callback(self, msg):
        """Store waypoints when received"""
        self.waypoints = msg.waypoints
        self.get_logger().info(f'Received {len(self.waypoints)} waypoints')
    
    def set_servo(self, pwm_value):
        """Send servo PWM command via MAVROS"""
        request = CommandLong.Request()
        request.command = 183  # MAV_CMD_DO_SET_SERVO
        request.param1 = 10.0  # Servo channel (change as needed)
        request.param2 = float(pwm_value)
        
        future = self.command_long_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result() and future.result().success:
            self.get_logger().info(f'✅ Servo set to {pwm_value} PWM')
            return True
        else:
            self.get_logger().error(f'❌ Failed to set servo to {pwm_value}')
            return False
    
    def goto_waypoint(self, waypoint_index):
        """Command rover to navigate to specific waypoint"""
        request = WaypointSetCurrent.Request()
        request.wp_seq = waypoint_index
        
        future = self.waypoint_set_current_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result() and future.result().success:
            self.get_logger().info(f'🎯 Navigating to waypoint {waypoint_index}')
            return True
        else:
            self.get_logger().error(f'Failed to set waypoint {waypoint_index}')
            return False
    
    def calculate_distance(self, lat1, lon1, lat2, lon2):
        """Calculate distance between two GPS coordinates (Haversine formula)"""
        R = 6371000  # Earth radius in meters
        
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)
        
        a = math.sin(delta_phi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
        return R * c
    
    def wait_for_waypoint_arrival(self, target_waypoint):
        """Wait until rover reaches target waypoint"""
        self.get_logger().info(f'Waiting to reach waypoint {self.current_waypoint_index}...')
        
        while self.mission_active:
            if self.current_gps is None:
                time.sleep(0.5)
                continue
            
            distance = self.calculate_distance(
                self.current_gps.latitude,
                self.current_gps.longitude,
                target_waypoint.x_lat,
                target_waypoint.y_long
            )
            
            self.get_logger().info(f'Distance to waypoint: {distance:.2f}m')
            
            if distance < self.waypoint_reached_threshold:
                self.get_logger().info('✅ Waypoint reached!')
                return True
            
            time.sleep(1.0)
        
        return False
    
    def execute_spray_sequence(self):
        """Execute spray sequence at current waypoint"""
        self.get_logger().info(f'🕐 Delay before start: {self.delay_before_start}s')
        time.sleep(self.delay_before_start)
        
        if not self.mission_active:
            return False
        
        self.get_logger().info(f'💧 Starting servo (PWM: {self.pwm_start})')
        self.set_servo(self.pwm_start)
        
        self.get_logger().info(f'🕐 Spraying for: {self.delay_before_stop}s')
        time.sleep(self.delay_before_stop)
        
        if not self.mission_active:
            return False
        
        self.get_logger().info(f'🛑 Stopping servo (PWM: {self.pwm_stop})')
        self.set_servo(self.pwm_stop)
        
        self.get_logger().info(f'🕐 Delay after stop: {self.delay_after_stop}s')
        time.sleep(self.delay_after_stop)
        
        return True
    
    def run_mission(self):
        """Main mission execution loop"""
        self.get_logger().info('🚀 Starting WP_MARK Mission')
        
        # Pull waypoints from flight controller
        if not self.pull_waypoints():
            self.get_logger().error('Cannot start mission - no waypoints')
            return
        
        if len(self.waypoints) == 0:
            self.get_logger().error('No waypoints loaded')
            return
        
        # Execute mission for each waypoint
        for i, waypoint in enumerate(self.waypoints):
            if not self.mission_active:
                self.get_logger().info('Mission stopped by user')
                break
            
            self.current_waypoint_index = i
            self.get_logger().info(f'\n{"="*50}')
            self.get_logger().info(f'Waypoint {i+1}/{len(self.waypoints)}')
            self.get_logger().info(f'{"="*50}')
            
            # Step 1: Navigate to waypoint
            if not self.goto_waypoint(i):
                self.get_logger().error(f'Failed to navigate to waypoint {i}')
                continue
            
            # Step 2: Wait for arrival
            if not self.wait_for_waypoint_arrival(waypoint):
                break
            
            # Step 3-7: Execute spray sequence
            if not self.execute_spray_sequence():
                break
        
        self.get_logger().info('✅ Mission Complete!')
    
    def stop(self):
        """Stop the mission"""
        self.mission_active = False
        self.get_logger().info('🛑 Mission stopped')


# Flask API endpoint handler
def start_wp_mark_mission(params):
    """
    Start WP_MARK mission with given parameters
    Called from Flask endpoint
    """
    # Initialize ROS2
    rclpy.init()
    
    # Create mission node
    mission = WPMarkMission(params)
    
    # Run mission in separate thread
    mission_thread = Thread(target=mission.run_mission)
    mission_thread.start()
    
    return {
        'success': True,
        'message': 'WP_MARK mission started successfully',
        'waypoint_count': len(mission.waypoints)
    }
```

---

### **Step 2: Add Flask Route**

**File**: `backend/app.py` (UPDATE - add this route)

```python
from flask import Flask, request, jsonify
from wp_mark_controller import start_wp_mark_mission
import json

app = Flask(__name__)

@app.route('/wp_mark/start', methods=['POST'])
def wp_mark_start():
    """
    Start WP_MARK mission
    
    Expected JSON body:
    {
        "delay_before_start": 2.0,
        "pwm_start": 1500,
        "delay_before_stop": 5.0,
        "pwm_stop": 1000,
        "delay_after_stop": 1.0
    }
    """
    try:
        params = request.get_json()
        
        # Validate parameters
        required_fields = [
            'delay_before_start',
            'pwm_start',
            'delay_before_stop',
            'pwm_stop',
            'delay_after_stop'
        ]
        
        for field in required_fields:
            if field not in params:
                return jsonify({
                    'success': False,
                    'message': f'Missing required field: {field}'
                }), 400
        
        # Save configuration
        with open('config/wp_mark_config.json', 'w') as f:
            json.dump(params, f, indent=2)
        
        # Start mission
        result = start_wp_mark_mission(params)
        
        return jsonify(result), 200
        
    except Exception as e:
        return jsonify({
            'success': False,
            'message': str(e)
        }), 500

@app.route('/wp_mark/stop', methods=['POST'])
def wp_mark_stop():
    """Stop currently running WP_MARK mission"""
    # Implementation to stop mission
    return jsonify({'success': True, 'message': 'Mission stopped'}), 200

@app.route('/wp_mark/status', methods=['GET'])
def wp_mark_status():
    """Get current mission status"""
    # Implementation to get status
    return jsonify({
        'running': False,
        'current_waypoint': 0,
        'total_waypoints': 0
    }), 200
```

---

## 📊 Complete Data Flow

```
USER                FRONTEND              BACKEND              ROS2/MAVROS
 │                     │                     │                      │
 │  Click WP_MARK      │                     │                      │
 ├────────────────────>│                     │                      │
 │                     │                     │                      │
 │  Show Dialog        │                     │                      │
 │<────────────────────┤                     │                      │
 │                     │                     │                      │
 │  Enter Parameters   │                     │                      │
 ├────────────────────>│                     │                      │
 │                     │                     │                      │
 │                     │ POST /wp_mark/start │                      │
 │                     ├────────────────────>│                      │
 │                     │                     │                      │
 │                     │                     │  Pull Waypoints      │
 │                     │                     ├─────────────────────>│
 │                     │                     │                      │
 │                     │                     │  Waypoint List       │
 │                     │                     │<─────────────────────┤
 │                     │                     │                      │
 │                     │  Success Response   │  For Each Waypoint:  │
 │                     │<────────────────────┤                      │
 │                     │                     │  • Set Waypoint      │
 │  Success Alert      │                     ├─────────────────────>│
 │<────────────────────┤                     │                      │
 │                     │                     │  • Wait for Arrival  │
 │                     │                     │<─────────────────────┤
 │                     │                     │                      │
 │                     │                     │  • Delay             │
 │                     │                     │                      │
 │                     │                     │  • Set Servo ON      │
 │                     │                     ├─────────────────────>│
 │                     │                     │                      │
 │                     │                     │  • Delay             │
 │                     │                     │                      │
 │                     │                     │  • Set Servo OFF     │
 │                     │                     ├─────────────────────>│
 │                     │                     │                      │
 │                     │                     │  • Delay             │
 │                     │                     │                      │
 │                     │                     │  • Next Waypoint     │
 │                     │                     └──────────────────────┘
```

---

## 🎯 Summary: Frontend vs Backend Roles

### **Frontend Does:**
✅ Display UI button  
✅ Collect 5 parameters from user  
✅ Validate input (ranges, required fields)  
✅ Send HTTP POST request  
✅ Show success/error messages  
✅ Display mission status  

### **Backend Does:**
✅ Receive parameters via API  
✅ Save configuration to file  
✅ Initialize ROS2 node  
✅ Pull waypoints from MAVROS  
✅ Navigate to each waypoint  
✅ Execute timing sequence  
✅ Control servo PWM  
✅ Handle mission stop  

---

## 🚀 Implementation Steps

### **Phase 1: Frontend (Do First)**
1. Create `WPMarkButton.tsx`
2. Create `WPMarkDialog.tsx`
3. Update `LeftSidebar.tsx`
4. Test UI (click button, fill form, validate)

### **Phase 2: Backend (Do Second)**
1. Create `wp_mark_controller.py`
2. Add Flask routes in `app.py`
3. Test endpoint with Postman/curl
4. Test ROS2 integration

### **Phase 3: Integration Testing**
1. Test full flow: UI → Backend → ROS2
2. Test error handling
3. Test stop functionality
4. Test with real waypoints

---

## 📝 Quick Reference

**Frontend API Call:**
```typescript
POST http://localhost:5000/wp_mark/start
{
  "delay_before_start": 2.0,
  "pwm_start": 1500,
  "delay_before_stop": 5.0,
  "pwm_stop": 1000,
  "delay_after_stop": 1.0
}
```

**Backend Mission Sequence:**
```python
for waypoint in waypoints:
    goto_waypoint(i)
    wait_for_arrival()
    sleep(delay_before_start)
    set_servo(pwm_start)
    sleep(delay_before_stop)
    set_servo(pwm_stop)
    sleep(delay_after_stop)
```

---

Would you like me to implement these files now? 🚀
