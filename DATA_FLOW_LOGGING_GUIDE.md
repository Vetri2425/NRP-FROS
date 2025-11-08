# Data Flow Logging Guide

## Overview
Comprehensive console logging has been added to track data flow from the server through the entire application stack.

## Data Flow Pipeline

```
Server (Socket.IO)
    ↓
useRoverROS Hook (toTelemetryEnvelopeFromRoverData)
    ↓
useRoverROS Hook (applyEnvelope)
    ↓
useRoverROS Hook (roverPosition memoization)
    ↓
RoverContext (Provider)
    ↓
UI Components (MapView, TelemetryPanel, RTKPanel, etc.)
```

## Log Categories

### 🔵 Server → useRoverROS
**Label:** `[DATA FLOW] 🔵 Server → useRoverROS (Position):`

**Location:** `src/hooks/useRoverROS.ts` - `toTelemetryEnvelopeFromRoverData()`

**Logs:**
- Raw position data from server
- Latitude/longitude values
- Position validation status
- Timestamp

**Example:**
```javascript
{
  raw_position: { lat: 13.0827, lng: 80.2707 },
  lat: 13.0827,
  lng: 80.2707,
  hasValidLat: true,
  hasValidLng: true,
  timestamp: "2025-11-01T10:30:45.123Z"
}
```

---

### 🟢 useRoverROS.applyEnvelope()
**Label:** `[DATA FLOW] 🟢 useRoverROS.applyEnvelope() received:`

**Location:** `src/hooks/useRoverROS.ts` - `applyEnvelope()`

**Logs:**
- Which envelope fields are present (state, global, battery, rtk, mission, servo)
- Detailed updates for each field type
- Position, RTK status, battery, mission progress

**Example:**
```javascript
{
  hasState: true,
  hasGlobal: true,
  hasBattery: true,
  hasRtk: true,
  hasMission: true,
  hasServo: false,
  timestamp: "2025-11-01T10:30:45.123Z"
}
```

**Sub-logs:**
- `📊 State updated:` - Vehicle mode, armed status
- `🌍 Global position updated:` - GPS coordinates, satellites
- `🔋 Battery updated:` - Voltage, percentage
- `📡 RTK updated:` - Fix type, baseline age, base linked
- `🎯 Mission updated:` - Current waypoint, progress
- `🎮 Servo updated:` - Servo positions

---

### 🚀 useRoverROS → React State Update
**Label:** `[DATA FLOW] 🚀 useRoverROS → React State Update (throttled):`

**Location:** `src/hooks/useRoverROS.ts` - `applyEnvelope()` (throttle logic)

**Logs:**
- Throttle timing (elapsed vs throttle threshold)
- Complete telemetry snapshot
- Position, mode, RTK status, battery

**Example:**
```javascript
{
  elapsed_ms: "35.2",
  throttle_ms: 33,
  position: "13.082700, 80.270700",
  mode: "AUTO",
  armed: true,
  rtk_fix_type: 6,
  satellites: 18,
  battery: "12.4V (95%)",
  timestamp: "2025-11-01T10:30:45.123Z"
}
```

**Throttled updates:**
- `⏱️ Update throttled (waiting):` - Shows remaining time before next update

---

### 🎯 useRoverROS.roverPosition Memoization
**Label:** `[DATA FLOW] 🎯 useRoverROS.roverPosition memoization triggered:`

**Location:** `src/hooks/useRoverROS.ts` - `roverPosition` useMemo

**Logs:**
- Memoization trigger
- Lat/lng values
- Timestamp
- Validation result (will return null for {0,0})

**Example:**
```javascript
{
  lat: 13.0827,
  lng: 80.2707,
  timestamp: 1698835845123,
  will_return_null: false
}
```

**Success:**
- `✅ useRoverROS → Context (roverPosition):` - Memoized position object

**Validation failure:**
- `⚠️ Returning null (invalid {0,0} position)` - Skipped invalid position

---

### 🔄 RoverContext Value Updated
**Label:** `[DATA FLOW] 🔄 RoverContext value updated:`

**Location:** `src/context/RoverContext.tsx` - `RoverProvider`

**Logs:**
- Connection state
- Rover position availability
- Telemetry coordinates

**Example:**
```javascript
{
  connectionState: "connected",
  hasRoverPosition: true,
  roverPosition: { lat: 13.0827, lng: 80.2707, timestamp: 1698835845123 },
  telemetry_lat: 13.0827,
  telemetry_lon: 80.2707,
  timestamp: "2025-11-01T10:30:45.123Z"
}
```

---

### 📥 Component Accessing RoverContext
**Label:** `[DATA FLOW] 📥 Component accessing RoverContext:`

**Location:** `src/context/RoverContext.tsx` - `useRover()`

**Logs:**
- Every time a component accesses the context
- Rover position availability
- Connection state

**Example:**
```javascript
{
  hasRoverPosition: true,
  roverPosition: { lat: 13.0827, lng: 80.2707, timestamp: 1698835845123 },
  connectionState: "connected"
}
```

---

### 🗺️ Context → MapView
**Label:** `[MapView] Rover position updated:`

**Location:** `src/components/MapView.tsx` - useEffect for position updates

**Logs:**
- Position updates received by MapView
- Timestamp for lag measurement

**Example:**
```javascript
{
  lat: 13.0827,
  lng: 80.2707,
  timestamp: "2025-11-01T10:30:45.123Z"
}
```

---

### 📊 Context → TelemetryPanel
**Label:** `[DATA FLOW] 📊 Context → TelemetryPanel:`

**Location:** `src/components/TelemetryPanel.tsx`

**Logs:**
- Complete telemetry data
- Connection state
- Rover position
- Mode, armed status, RTK, battery

**Example:**
```javascript
{
  component: "TelemetryPanel",
  connectionState: "connected",
  roverPosition: { lat: 13.0827, lng: 80.2707, timestamp: 1698835845123 },
  mode: "AUTO",
  armed: true,
  position: "13.082700, 80.270700",
  rtk_fix_type: 6,
  rtk_base_linked: true,
  satellites: 18,
  battery: "12.4V (95%)",
  timestamp: "2025-11-01T10:30:45.123Z"
}
```

---

### 📡 Context → RTKPanel
**Label:** `[DATA FLOW] 📡 Context → RTKPanel:`

**Location:** `src/components/RTKPanel.tsx`

**Logs:**
- RTK-specific data
- Fix type, base link status, baseline age
- Satellite count

**Example:**
```javascript
{
  component: "RTKPanel",
  rtk_fix_type: 6,
  rtk_base_linked: true,
  rtk_baseline_age: 0.5,
  satellites_visible: 18,
  roverPosition: { lat: 13.0827, lng: 80.2707, timestamp: 1698835845123 },
  timestamp: "2025-11-01T10:30:45.123Z"
}
```

---

### 📶 Context → StatusBar
**Label:** `[DATA FLOW] 📶 Context → StatusBar:`

**Location:** `src/components/StatusBar.tsx`

**Logs:**
- Connection state
- Battery status
- Network information

**Example:**
```javascript
{
  component: "StatusBar",
  connectionState: "connected",
  roverPosition: { lat: 13.0827, lng: 80.2707, timestamp: 1698835845123 },
  battery: "12.4V (95%)",
  network_type: "wifi",
  wifi_signal: 4,
  timestamp: "2025-11-01T10:30:45.123Z"
}
```

---

### 🎯 Context → MissionControl
**Label:** `[DATA FLOW] 🎯 Context → MissionControl:`

**Location:** `src/components/MissionControl.tsx`

**Logs:**
- Mission status
- Current/total waypoints
- Progress percentage

**Example:**
```javascript
{
  component: "MissionControl",
  roverPosition: { lat: 13.0827, lng: 80.2707, timestamp: 1698835845123 },
  mission_status: "ACTIVE",
  current_wp: 5,
  total_wp: 10,
  progress_pct: 50,
  timestamp: "2025-11-01T10:30:45.123Z"
}
```

---

## Performance Monitoring

### Throttling
- Updates are throttled to 33ms (~30 Hz)
- Logs show elapsed time vs throttle threshold
- Pending updates are logged with remaining wait time

### Position Validation
- {0,0} coordinates are filtered out
- Validation failures are logged
- Only valid positions are passed to components

### Memoization
- `roverPosition` is memoized to prevent unnecessary re-renders
- Dependencies: `lat`, `lng`, `timestamp`
- Logs show when memoization is triggered

---

## Debugging GPS Lag

### Key Metrics to Watch

1. **Server → useRoverROS timing:**
   - Look for timestamp in `🔵 Server → useRoverROS (Position)`
   
2. **Throttle delays:**
   - Check `🚀 useRoverROS → React State Update` elapsed_ms
   - Should be ~33ms or slightly higher
   
3. **Memoization triggers:**
   - `🎯 useRoverROS.roverPosition memoization triggered`
   - Should only trigger when lat/lng/timestamp changes
   
4. **MapView updates:**
   - `[MapView] Rover position updated:`
   - Compare timestamp to server data timestamp
   - Lag should be 20-50ms (not 413ms)

### Expected Flow for Position Update

```
1. 🔵 Server → useRoverROS (Position)      [T+0ms]
2. ✅ Position validated                    [T+0ms]
3. 🟢 applyEnvelope received               [T+1ms]
4. 🌍 Global position updated              [T+1ms]
5. 🚀 React State Update (throttled)       [T+33ms]
6. 🎯 roverPosition memoization            [T+34ms]
7. ✅ useRoverROS → Context                [T+34ms]
8. 🔄 RoverContext value updated           [T+35ms]
9. 🗺️ MapView position updated             [T+40ms]
```

**Total expected lag: 40-50ms** (vs previous 413ms)

---

## Console Filtering

To filter logs by component, use browser DevTools console filters:

- **All data flow:** `[DATA FLOW]`
- **Server data:** `🔵 Server`
- **Position updates:** `🌍 Global position` or `🗺️ MapView`
- **RTK data:** `📡 RTK`
- **Specific component:** `→ MapView` or `→ TelemetryPanel`
- **Performance:** `throttled` or `memoization`

---

## Troubleshooting

### No position updates
1. Check `🔵 Server → useRoverROS (Position)` - Is server sending data?
2. Check `❌ Position validation failed` - Are coordinates {0,0}?
3. Check `🚀 React State Update` - Is throttling working?

### Lag still present
1. Compare timestamps between `🔵 Server` and `🗺️ MapView`
2. Check for excessive `⏱️ Update throttled` logs
3. Verify memoization triggers only when position changes

### Component not receiving data
1. Check `📥 Component accessing RoverContext` - Is component using context?
2. Verify component-specific log (e.g., `📊 Context → TelemetryPanel`)
3. Check `🔄 RoverContext value updated` - Is context updating?

---

## Performance Targets

- **Server → useRoverROS:** < 5ms
- **applyEnvelope processing:** < 5ms
- **React state update (throttled):** ~33ms
- **Memoization:** < 1ms
- **Context → Component:** < 5ms
- **Total end-to-end lag:** 20-50ms ✅ (was 413ms ❌)

---

## Files Modified

1. `src/hooks/useRoverROS.ts`
2. `src/context/RoverContext.tsx`
3. `src/components/MapView.tsx`
4. `src/components/TelemetryPanel.tsx`
5. `src/components/RTKPanel.tsx`
6. `src/components/StatusBar.tsx`
7. `src/components/MissionControl.tsx`

---

## Next Steps

1. Run `npm run dev` to start development server
2. Open browser DevTools console
3. Connect to rover backend
4. Watch for data flow logs
5. Verify GPS lag is eliminated (20-50ms vs 413ms)
6. Check that rover icon follows path smoothly
