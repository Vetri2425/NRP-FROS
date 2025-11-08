# Rover Map Visualization - Quick Reference Guide

## 🎯 At a Glance

### What Was Implemented
✅ **Directional Indicators** - RED/GREEN/BLACK/ORANGE bearing lines  
✅ **Position Throttling** - Smart map centering (11m threshold, 1s max rate)  
✅ **Batch Rendering** - HoldInvalidation pattern for smooth updates  
✅ **Velocity Interpolation** - 60fps smoothing from 20Hz telemetry  
✅ **High Precision** - 7 decimal GPS (1.1cm accuracy)  
✅ **Waypoint Targeting** - GUIDED mode navigation service  

### Quick Start

```typescript
// In your map component
import { useMapTelemetry } from '../hooks/useMapTelemetry';

useMapTelemetry({
  mapRef,
  vehicleLayerRef,
  trailLayerRef,
  enableMapCentering: true,        // Auto-follow rover
  enableVelocityInterpolation: true, // Smooth movement
});
```

## 🔧 Configuration Presets

### Default (Balanced)
```typescript
{
  throttleMs: 250,              // 4 Hz updates
  updateIntervalMs: 300,        // 0.3s batches
  maxTrailPoints: 500,
  enableMapCentering: false,
  enableVelocityInterpolation: true,
}
```

### High Performance
```typescript
{
  throttleMs: 100,              // 10 Hz updates
  updateIntervalMs: 200,        // 0.2s batches
  maxTrailPoints: 200,
  enableMapCentering: true,
  mapCenteringThreshold: 0.0002, // 22m threshold
}
```

### Battery Saver
```typescript
{
  throttleMs: 500,              // 2 Hz updates
  updateIntervalMs: 1000,       // 1s batches
  maxTrailPoints: 100,
  enableMapCentering: false,
  enableVelocityInterpolation: false,
}
```

## 🎨 Directional Indicator Colors

| Color | Meaning | Data Source |
|-------|---------|-------------|
| 🔴 **RED** | Compass Heading | `telemetry.attitude.yaw_deg` |
| 🟢 **GREEN** | Navigation Bearing | `telemetry.global.navigation_bearing` |
| ⚫ **BLACK** (dashed) | Course Over Ground | `telemetry.global.course_over_ground` |
| 🟠 **ORANGE** | Target Bearing | `telemetry.global.target_bearing` |

## 📡 Backend Integration

### Required Socket Event Data
```python
# In your ROS node, emit rover_data with:
{
    "position": {
        "lat": 13.0827123,  # 7 decimal places
        "lng": 80.2707456   # 7 decimal places
    },
    "heading": 45.2,        # IMU yaw in degrees
    
    # NEW: For interpolation
    "velocity": {
        "vx": 1.2,          # East velocity (m/s)
        "vy": 0.8,          # North velocity (m/s)
        "vz": -0.1          # Down velocity (m/s)
    },
    
    # NEW: For bearing lines
    "course_over_ground": 48.5,    # Actual movement direction
    "navigation_bearing": 42.3,    # Direction to waypoint
    "target_bearing": 45.0,        # MAVLink target
    
    "rtk_fix_type": 6  # GPS quality
}
```

### New API Endpoint
```python
@app.route('/api/set_target_position', methods=['POST'])
def set_target_position():
    """
    Navigate rover to precise GPS location (GUIDED mode)
    """
    data = request.json
    lat_int = data['lat_int']      # latitude × 1e7
    lng_int = data['lng_int']      # longitude × 1e7
    alt_int = data.get('alt_int')  # altitude in mm (optional)
    type_mask = data['type_mask']  # 0xFFF8 for position-only
    
    # Create MAVLink SET_POSITION_TARGET_GLOBAL_INT message
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,                    # time_boot_ms
        0, 1,                 # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        type_mask,
        lat_int,
        lng_int,
        alt_int or 0,
        0, 0, 0,             # vx, vy, vz (ignored)
        0, 0, 0,             # afx, afy, afz (ignored)
        0, 0                 # yaw, yaw_rate (ignored)
    )
    vehicle.send_mavlink(msg)
    
    return {"success": True}
```

## 🧪 Testing the Features

### Test Directional Indicators
1. Enable all bearing fields in backend
2. Move rover to see lines update
3. Lines should point:
   - RED: North (if heading = 0°)
   - GREEN: To next waypoint
   - BLACK: Direction of movement
   - ORANGE: MAVLink target

### Test Map Centering
```typescript
useMapTelemetry({
  enableMapCentering: true,
  mapCenteringThreshold: 0.0001, // ~11 meters
  mapCenteringIntervalMs: 1000,  // 1 second
});
```
- Map should follow rover
- No jitter from GPS noise
- Max 1 pan per second

### Test Velocity Interpolation
```typescript
useMapTelemetry({
  enableVelocityInterpolation: true,
});
```
- Rover movement should be smooth (60fps)
- No jerky jumps between telemetry updates
- Check browser DevTools: Look for 60fps

### Test Waypoint Targeting
```typescript
const { services } = useRover();

// Send rover to precise location
await services.setTargetPosition(
  13.0827123,  // 7 decimals = 1.1cm accuracy
  80.2707456,
  10           // 10 meters altitude
);
```
- Backend should receive lat_int, lng_int (× 1e7)
- Rover should navigate to exact coordinate

## 🐛 Troubleshooting

### Bearing Lines Not Showing
**Problem:** Directional indicators invisible  
**Solution:** Check backend sends bearing data:
```javascript
console.log(telemetry.global?.course_over_ground);
console.log(telemetry.global?.navigation_bearing);
console.log(telemetry.global?.target_bearing);
```

### Jerky Movement
**Problem:** Rover jumps instead of smooth movement  
**Solution:** 
1. Enable interpolation: `enableVelocityInterpolation: true`
2. Check backend sends vx, vy velocity:
```javascript
console.log(telemetry.global?.vx, telemetry.global?.vy);
```

### Map Not Centering
**Problem:** Map doesn't follow rover  
**Solution:**
1. Enable centering: `enableMapCentering: true`
2. Lower threshold: `mapCenteringThreshold: 0.00005` (5.5m)
3. Check rover is moving > threshold distance

### Performance Issues
**Problem:** Laggy map rendering  
**Solution:**
1. Increase throttle: `throttleMs: 500` (slower updates)
2. Reduce trail: `maxTrailPoints: 100`
3. Disable interpolation: `enableVelocityInterpolation: false`

## 📊 Performance Metrics

### Monitor in DevTools Console
```javascript
// Check update frequency
[BatchUpdate] Processing 3 updates
[MapCentering] Map centered to: ...

// Check data flow
[DATA FLOW] 🔵 Server → Position: ...
[DATA FLOW] 🎯 roverPosition memoized: ...
```

### Expected Console Output
```
[useMapTelemetry] ✅ Rover marker created at: { lat: 13.0827, lng: 80.2707 }
[useMapTelemetry] ✅ Trail system initialized
[BatchUpdate] Processing 2 updates
[BatchUpdate] ✓ Marker updated
[BatchUpdate] ✓ Trail updated
[MapCentering] Map centered to: { lat: 13.0827123, lng: 80.2707456 }
```

## 🎓 Advanced Usage

### Custom Bearing Line Colors
```typescript
// In RoverMarker.tsx, edit _updateBearingLines()
const headingLine = createBearingLine(this._heading, '#ff0000', 3); // Change color
```

### Adjust Line Length
```typescript
// In RoverMarker.tsx
const lineLength = 0.0005; // Longer lines (~55 meters)
```

### Dynamic Trail Color
```typescript
// In trail-system.ts
const segment = L.polyline(group.coords, {
  color: speed > 2 ? '#00ff00' : '#0ea5e9', // Green if fast
  weight: 3,
  opacity: group.opacity,
});
```

## 📚 API Reference

### useMapTelemetry Hook
```typescript
useMapTelemetry({
  mapRef: RefObject<Map>,
  vehicleLayerRef: RefObject<LayerGroup>,
  trailLayerRef: RefObject<LayerGroup>,
  throttleMs?: number,
  maxTrailPoints?: number,
  enableMapCentering?: boolean,
  mapCenteringThreshold?: number,
  mapCenteringIntervalMs?: number,
  updateIntervalMs?: number,
  enableVelocityInterpolation?: boolean,
})
```

### RoverMarker API
```typescript
marker.setHeading(deg: number)
marker.setCourseOverGround(deg?: number)
marker.setNavigationBearing(deg?: number)
marker.setTargetBearing(deg?: number)
marker.setShowDirectionalIndicators(show: boolean)
marker.setStatus('armed' | 'disarmed' | 'rtk')
marker.setLatLng({ lat, lng })
```

### RoverServices API
```typescript
services.setTargetPosition(
  lat: number,    // Latitude in degrees
  lng: number,    // Longitude in degrees
  alt?: number    // Altitude in meters (optional)
): Promise<ServiceResponse>
```

## 🔗 Related Documentation
- [Full Implementation Guide](./ROVER_MAP_VISUALIZATION_IMPLEMENTATION.md)
- [Trail System Documentation](./TRAIL_SYSTEM_GUIDE.md)
- [MAVLink Reference](https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT)

---

**Last Updated:** 2025-11-07  
**Version:** 1.0.0
