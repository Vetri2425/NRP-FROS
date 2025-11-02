# 🔧 ROS2 Backend Diagnostics Guide

## Error: "rcl node's context is invalid"

This error occurs when the ROS2 node has lost its context, typically due to:
- ROS2 daemon crash
- MAVROS service failure
- Backend server restart without ROS2 cleanup

---

## 🚨 Diagnostic Commands (Run on Backend Server)

### 1. Check if ROS2 daemon is running
```bash
ros2 daemon status
```
**Expected**: "The daemon is running"
**If not**: Run `ros2 daemon start`

---

### 2. Check available ROS2 nodes
```bash
ros2 node list
```
**Expected output should include**:
- `/mavros`
- Your backend node (e.g., `/rover_backend_node`)

---

### 3. Check MAVROS arming service
```bash
ros2 service list | grep arming
```
**Expected**: `/mavros/cmd/arming`

**Test the service**:
```bash
ros2 service type /mavros/cmd/arming
```
**Expected**: `mavros_msgs/srv/CommandBool`

---

### 4. Check MAVROS connection status
```bash
ros2 topic echo /mavros/state --once
```
**Expected**: Should show `connected: true`

---

### 5. Check for ROS2 errors
```bash
ros2 wtf
```
This will show warnings and errors in your ROS2 system.

---

## 🔨 Fixes

### Fix 1: Restart ROS2 Daemon
```bash
ros2 daemon stop
ros2 daemon start
```

### Fix 2: Restart MAVROS
```bash
# Kill existing MAVROS
pkill -9 -f mavros

# Launch MAVROS for ArduPilot (adjust parameters as needed)
ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:57600
```

### Fix 3: Restart Backend Server
```bash
# Navigate to backend directory
cd /path/to/NRP_ROS_V2/Backend

# Kill old processes
pkill -9 -f "python.*app.py"

# Restart backend
python3 app.py
```

### Fix 4: Complete ROS2 Reset (Nuclear Option)
```bash
# Stop everything
pkill -9 -f ros2
pkill -9 -f mavros
ros2 daemon stop

# Clean ROS2 cache
rm -rf ~/.ros/log/*

# Restart daemon
ros2 daemon start

# Launch MAVROS
ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:57600

# In another terminal, start backend
python3 /path/to/Backend/app.py
```

---

## 🔍 Backend Code Issue

If the backend server itself is causing the issue, check your backend Python code for:

### Common Issues:

1. **Not initializing ROS2 context properly**:
```python
import rclpy

# CORRECT:
rclpy.init()
node = rclpy.create_node('rover_backend')

# After done:
node.destroy_node()
rclpy.shutdown()
```

2. **Service client not checking context**:
```python
# Add context validation before service calls
if not node.context.ok():
    print("ERROR: ROS2 context is invalid!")
    rclpy.init()  # Reinitialize
```

3. **Not handling service timeouts**:
```python
from rclpy.duration import Duration

# When calling arming service:
if arming_client.wait_for_service(timeout_sec=5.0):
    # Service is available
    future = arming_client.call_async(request)
else:
    print("ERROR: Arming service not available")
    # Return error to frontend
```

---

## 📡 Testing After Fixes

### Test 1: Check ROS2 service manually
```bash
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
```
**Expected**: Success response with `success: true`

### Test 2: Check from Python
```python
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool

rclpy.init()
node = Node('test_arming')

client = node.create_client(CommandBool, '/mavros/cmd/arming')
if client.wait_for_service(timeout_sec=5.0):
    request = CommandBool.Request()
    request.value = True
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    print(f"Result: {future.result()}")
else:
    print("Service not available!")

node.destroy_node()
rclpy.shutdown()
```

### Test 3: Check backend API endpoint
```bash
# From your Windows machine
curl -X POST http://192.168.1.101:5001/api/arm \
  -H "Content-Type: application/json" \
  -d '{"arm": true}'
```

---

## 🎯 Expected Successful Response

When working correctly, the backend should return:
```json
{
  "success": true,
  "message": "Armed successfully"
}
```

Current error response:
```json
{
  "success": false,
  "message": "Arm/disarm command failed (ROS2: failed to check service availability: rcl node's context is invalid, at ./src/rcl/node.c:428)"
}
```

---

## 📞 Frontend Integration

The frontend sends arm/disarm commands via:
- **Endpoint**: `POST /api/arm`
- **Payload**: `{"arm": true}` or `{"arm": false}`

Check `useRoverROS.ts` for the frontend implementation:
```typescript
const armDisarm = async (arm: boolean): Promise<boolean> => {
  const response = await fetch(`${BACKEND_URL}/api/arm`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ arm })
  });
  const data = await response.json();
  return data.success;
};
```

---

## 🔗 Related Files

- **Frontend**: `src/hooks/useRoverROS.ts` - Sends arm/disarm requests
- **Backend**: `Backend/app.py` - Handles `/api/arm` endpoint
- **Backend**: `Backend/ros2_interface.py` - ROS2 service client
- **Config**: `.env` - Backend URL configuration

---

## 💡 Prevention Tips

1. **Always check ROS2 context before service calls**
2. **Implement automatic reconnection in backend**
3. **Add health checks for ROS2 services**
4. **Monitor MAVROS connection status**
5. **Use systemd services for auto-restart** (on backend server)

---

## 📋 Quick Checklist

- [ ] ROS2 daemon is running (`ros2 daemon status`)
- [ ] MAVROS node is active (`ros2 node list`)
- [ ] Arming service exists (`ros2 service list | grep arming`)
- [ ] MAVROS is connected (`ros2 topic echo /mavros/state --once`)
- [ ] Backend server is running (`curl http://192.168.1.101:5001/api/status`)
- [ ] Flight controller is connected (USB/telemetry)
- [ ] Flight controller is in correct mode (MANUAL/HOLD)

---

## 🆘 Still Not Working?

If the issue persists:

1. **Check backend server logs** for detailed error messages
2. **Verify flight controller connection** (`ls /dev/ttyACM*` or `/dev/ttyUSB*`)
3. **Test MAVROS directly** without backend (using `ros2 service call`)
4. **Check ArduPilot parameters** - some prevent arming (e.g., ARMING_CHECK)
5. **Review MAVROS launch file** - ensure correct `fcu_url` and baud rate

---

Generated: $(date)
