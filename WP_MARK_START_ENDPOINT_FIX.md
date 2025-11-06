# 🔧 WP_MARK Start Endpoint Fix - Context.init() Error

## 🐛 Problem

**Error**: `Internal error: Context.init() must only be called once`

**Cause**: The backend `/wp_mark/start` endpoint calls `rclpy.init()` every time it's invoked, but ROS2 context can only be initialized once per process.

## ✅ Frontend Verification

The frontend is **correctly wired**:

### 1. ModeSelector.tsx → wpMarkService.ts → Backend
```typescript
// src/components/ModeSelector.tsx (Line 97-121)
const startMission = useCallback(async () => {
  await runAction(
    async () => {
      // Only set mode to AUTO if not already in AUTO mode
      if (state.mode !== 'AUTO') {
        await services.setMode('AUTO');
        await new Promise(resolve => setTimeout(resolve, 500));
      }
      
      // ✅ CORRECTLY CALLS THE SERVICE
      const response = await startWPMarkMissionFromSavedConfig();
      if (!response.success) {
        throw new Error(response.error || 'Failed to start mission');
      }
      return response;
    },
    // ... success handler
  );
}, [runAction, services, state.mode]);
```

### 2. wpMarkService.ts sends to /wp_mark/start
```typescript
// src/services/wpMarkService.ts (Line 65-76)
export async function startWPMarkMission(config: WPMarkConfig): Promise<WPMarkResponse> {
  const response = await fetch(`${BACKEND_URL}/wp_mark/start`, {  // ✅ CORRECT ENDPOINT
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
}
```

### 3. Button Click Wiring
```typescript
// src/components/ModeSelector.tsx (Line 182-186)
<button
  onClick={startMission}  // ✅ PROPERLY WIRED
  disabled={isLoading || !state.armed}
  className="..."
>
  🚀 Start Mission
</button>
```

**Frontend Status**: ✅ **ALL CONNECTIONS ARE CORRECT**

---

## 🔥 Backend Issue (REQUIRES FIX)

### Current Problematic Code Pattern

Based on CUSTOM_WP_MARK_MODE_GUIDE.md (Line 661), the backend likely has:

```python
# ❌ PROBLEMATIC CODE - DO NOT USE
def start_wp_mark_mission(params):
    """Start WP_MARK mission with given parameters"""
    
    # ❌ THIS CAUSES THE ERROR - CALLED MULTIPLE TIMES
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

### Flask Endpoint (Likely in app.py or similar)
```python
@app.route('/wp_mark/start', methods=['POST'])
def wp_mark_start():
    """Start WP_MARK mission"""
    try:
        params = request.get_json()
        
        # Validate parameters...
        
        # ❌ THIS CALLS rclpy.init() EVERY TIME
        result = start_wp_mark_mission(params)
        
        return jsonify(result), 200
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)}), 500
```

---

## ✅ Solution: Fix Backend ROS2 Initialization

### Option 1: Initialize ROS2 Once at Application Startup (RECOMMENDED)

**File**: `app.py` (or main backend server file)

```python
import rclpy
from flask import Flask, request, jsonify
from threading import Thread
import atexit

app = Flask(__name__)

# ✅ INITIALIZE ROS2 CONTEXT ONCE AT STARTUP
def init_ros2():
    """Initialize ROS2 context once for the entire application"""
    if not rclpy.ok():
        rclpy.init()
        print("✅ ROS2 context initialized")

def shutdown_ros2():
    """Shutdown ROS2 context on app exit"""
    if rclpy.ok():
        rclpy.shutdown()
        print("✅ ROS2 context shutdown")

# Initialize ROS2 when Flask app starts
init_ros2()

# Ensure cleanup on exit
atexit.register(shutdown_ros2)


# ✅ FIXED: No longer calls rclpy.init()
def start_wp_mark_mission(params):
    """Start WP_MARK mission with given parameters"""
    
    # ✅ DON'T CALL rclpy.init() - ALREADY INITIALIZED
    
    # Create mission node (assumes ROS2 is already initialized)
    mission = WPMarkMission(params)
    
    # Run mission in separate thread
    mission_thread = Thread(target=mission.run_mission, daemon=True)
    mission_thread.start()
    
    return {
        'success': True,
        'message': 'WP_MARK mission started successfully',
        'waypoint_count': len(mission.waypoints)
    }


@app.route('/wp_mark/start', methods=['POST'])
def wp_mark_start():
    """Start WP_MARK mission"""
    try:
        params = request.get_json()
        
        # Validate parameters
        required_fields = [
            'delay_before_start',
            'pwm_start',
            'delay_before_stop',
            'pwm_stop',
            'delay_after_stop',
            'servo_number'
        ]
        
        for field in required_fields:
            if field not in params:
                return jsonify({
                    'success': False,
                    'error': f'Missing required field: {field}'
                }), 400
        
        # Save configuration
        with open('config/wp_mark_config.json', 'w') as f:
            json.dump(params, f, indent=2)
        
        # ✅ Start mission (no longer initializes ROS2)
        result = start_wp_mark_mission(params)
        
        return jsonify(result), 200
        
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        }), 500


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5001, debug=False)
```

---

### Option 2: Check if ROS2 is Already Initialized

**File**: `wp_mark_controller.py` or similar

```python
import rclpy

def start_wp_mark_mission(params):
    """Start WP_MARK mission with given parameters"""
    
    # ✅ ONLY INITIALIZE IF NOT ALREADY INITIALIZED
    if not rclpy.ok():
        rclpy.init()
        print("✅ ROS2 context initialized")
    else:
        print("ℹ️ ROS2 context already initialized")
    
    # Create mission node
    mission = WPMarkMission(params)
    
    # Run mission in separate thread
    mission_thread = Thread(target=mission.run_mission, daemon=True)
    mission_thread.start()
    
    return {
        'success': True,
        'message': 'WP_MARK mission started successfully',
        'waypoint_count': len(mission.waypoints)
    }
```

---

## 🎯 Recommended Implementation Steps

1. **Locate Backend Server File**: Find `app.py`, `server.py`, or main Flask/FastAPI file
2. **Find the `/wp_mark/start` endpoint handler**
3. **Locate where `rclpy.init()` is called** (likely in `start_wp_mark_mission()` or similar)
4. **Apply Option 1** (initialize once at startup) - MOST RELIABLE
5. **Test the fix**:
   ```bash
   # Restart backend server
   python3 app.py
   
   # Click "🚀 Start Mission" button multiple times
   # Should work without Context.init() error
   ```

---

## 🧪 Testing

### Test 1: Single Start
1. Click "🚀 Start Mission"
2. Should start successfully

### Test 2: Multiple Starts (Error Reproduction)
1. Click "🚀 Start Mission"
2. Wait for mission to start
3. Stop mission
4. Click "🚀 Start Mission" again
5. **Before fix**: `Context.init() error`
6. **After fix**: Should work correctly

### Test 3: Backend Restart
1. Restart backend server
2. Click "🚀 Start Mission"
3. Should work correctly

---

## 📍 Backend Server Location

Based on your configuration:
- **IP**: `192.168.1.101`
- **Port**: `5001`
- **Endpoint**: `http://192.168.1.101:5001/wp_mark/start`

### Find the Backend Code
Look for these files on the Jetson/Rover system:
```bash
# SSH into the rover
ssh user@192.168.1.101

# Search for Flask app
find ~ -name "app.py" -o -name "server.py" -o -name "main.py" 2>/dev/null

# Search for wp_mark endpoint
grep -r "wp_mark/start" ~ 2>/dev/null

# Check running processes
ps aux | grep python
```

---

## 📝 Summary

| Component | Status | Issue |
|-----------|--------|-------|
| Frontend ModeSelector.tsx | ✅ Correct | Properly calls startMission |
| Frontend wpMarkService.ts | ✅ Correct | Sends to `/wp_mark/start` |
| Backend Endpoint | ❌ NEEDS FIX | Calls `rclpy.init()` multiple times |
| Frontend → Backend Wiring | ✅ Correct | All connections verified |

**ACTION REQUIRED**: Update backend Python code to initialize ROS2 context only once at application startup, not on every `/wp_mark/start` request.

