# GPS Data Flow Analysis

## Overview
This document traces how GPS data flows from the ROS backend server through the Socket.IO connection to the frontend components (TelemetryPanel and MapView).

---

## 📡 Data Flow Path

```
ROS Backend → Socket.IO → useRoverROS Hook → RoverContext → App.tsx → Components
```

---

## 1. Backend Server Events

The backend sends GPS data through two Socket.IO events:

### Event 1: `telemetry`
Structured telemetry from ROS bridge
```typescript
{
  state: { armed, mode, system_status, heartbeat_ts },
  global: {
    latitude: number,    // GPS latitude
    longitude: number,   // GPS longitude  
    altitude: number,    // Relative altitude
    vel: number,         // Velocity/ground speed
    satellites_visible: number
  },
  battery: { voltage, current, percentage },
  rtk: { fix_type, baseline_age, base_linked },
  mission: { total_wp, current_wp, status, progress_pct }
}
```

### Event 2: `rover_data`
Alternative format from rover state
```typescript
{
  mode: string,
  status: string,
  position: {
    lat: number,      // GPS latitude
    lng: number       // GPS longitude
  },
  satellites_visible: number,
  battery: number,
  voltage: number,
  current: number,
  rtk_status: string,        // "RTK Fixed", "RTK Float", "DGPS", "GPS Fix"
  rtk_fix_type: number,      // 0-4 (numeric fix type)
  rtk_baseline_age: number,
  rtk_base_linked: boolean,
  activeWaypointIndex: number,
  completedWaypointIds: number[]
}
```

---

## 2. useRoverROS Hook Processing

**File:** `src/hooks/useRoverROS.ts`

### Data Transformation Functions

#### `toTelemetryEnvelopeFromBridge(data)`
Processes `telemetry` event:
```typescript
// Extracts position data
if (data.position && typeof data.position === 'object') {
  envelope.global = {
    lat: data.position.latitude,
    lon: data.position.longitude,
    alt_rel: data.position.altitude,
    vel: 0,
    satellites_visible: 0
  };
}

// Also checks for separate global object
if (data.global && typeof data.global === 'object') {
  envelope.global = {
    lat: data.global.latitude,
    lon: data.global.longitude,
    alt_rel: data.global.altitude,
    vel: data.global.vel,
    satellites_visible: data.global.satellites_visible
  };
}
```

#### `toTelemetryEnvelopeFromRoverData(data)`
Processes `rover_data` event:
```typescript
if (data.position && typeof data.position === 'object') {
  const { lat, lng } = data.position;
  envelope.global = {
    lat: typeof lat === 'number' ? lat : 0,
    lon: typeof lng === 'number' ? lng : 0,
    alt_rel: typeof data.distanceToNext === 'number' ? data.distanceToNext : 0,
    vel: 0,
    satellites_visible: typeof data.satellites_visible === 'number' 
      ? data.satellites_visible : 0
  };
}
```

### Telemetry State Structure

**Type:** `RoverTelemetry` (defined in `src/types/ros.ts`)
```typescript
export interface RoverTelemetry {
  state: TelemetryState;
  global: TelemetryGlobal;  // ← GPS data here
  battery: TelemetryBattery;
  rtk: TelemetryRtk;
  mission: TelemetryMission;
  servo: ServoStatus;
  network: NetworkData;
  lastMessageTs: number | null;
}

export interface TelemetryGlobal {
  lat: number;              // ← GPS Latitude
  lon: number;              // ← GPS Longitude
  alt_rel: number;          // Relative altitude (meters)
  vel: number;              // Ground speed (m/s)
  satellites_visible: number;
}
```

### Default Values
```typescript
const DEFAULT_GLOBAL: TelemetryGlobal = {
  lat: 0,
  lon: 0,
  alt_rel: 0,
  vel: 0,
  satellites_visible: 0,
};
```

### Update Throttling
- Updates throttled to **~30 Hz** (33ms intervals)
- Uses `applyEnvelope()` to merge incoming data
- State updates via `setTelemetrySnapshot()`

---

## 3. RoverContext Distribution

**File:** `src/context/RoverContext.tsx`

Simple context wrapper that provides the full `useRoverROS` result:
```typescript
export interface RoverContextValue extends UseRoverROSResult {
  telemetry: RoverTelemetry;  // ← Contains global GPS data
  connectionState: ConnectionState;
  services: RoverServices;
  // ...
}
```

---

## 4. App.tsx Processing

**File:** `src/App.tsx`

### Converting Telemetry to RoverData

The `toRoverData()` function transforms telemetry for UI components:

```typescript
const toRoverData = (
  telemetry: RoverTelemetry, 
  connectionState: ConnectionState, 
  missionWaypoints: Waypoint[] = []
): RoverData => {
  // Extract position from telemetry.global
  const position =
    Number.isFinite(telemetry.global.lat) && Number.isFinite(telemetry.global.lon)
      ? { lat: telemetry.global.lat, lng: telemetry.global.lon }
      : null;

  return {
    connected: connectionState === 'connected',
    mode: telemetry.state.mode,
    status: telemetry.state.armed ? 'armed' : 'disarmed',
    position,                           // ← GPS position {lat, lng}
    latitude: position?.lat,            // ← Individual lat
    longitude: position?.lng,           // ← Individual lng
    altitude: telemetry.global.alt_rel, // ← Altitude
    groundspeed: telemetry.global.vel,  // ← Speed
    satellites_visible: telemetry.global.satellites_visible,
    // ... other fields
  };
};
```

### Usage in App Component
```typescript
function App() {
  const { telemetry, connectionState } = useRover();
  
  const uiRoverData = useMemo(
    () => toRoverData(telemetry, connectionState, missionWaypoints),
    [telemetry, connectionState, missionWaypoints]
  );

  // Pass to MapView
  <MapView
    roverPosition={uiRoverData.position}  // {lat, lng}
    telemetry={{
      speed: telemetry.global.vel,
      altitude: telemetry.global.alt_rel,
      satellites: telemetry.global.satellites_visible,
      // ...
    }}
  />
}
```

---

## 5. Component Usage

### A. TelemetryPanel

**File:** `src/components/TelemetryPanel.tsx`

Displays GPS data directly from telemetry:

```typescript
const TelemetryPanel: React.FC = () => {
  const { telemetry, connectionState } = useRover();
  const { state, global, battery, mission, rtk, lastMessageTs } = telemetry;

  return (
    <div>
      {/* GPS Position Display */}
      <div>
        <p>Latitude</p>
        <p>{safeFixed(global.lat, 7)}</p>  {/* 7 decimal places */}
      </div>
      <div>
        <p>Longitude</p>
        <p>{safeFixed(global.lon, 7)}</p>
      </div>
      <div>
        <p>Altitude (rel)</p>
        <p>{safeFixed(global.alt_rel, 1)} m</p>
      </div>
      <div>
        <p>Ground Speed</p>
        <p>{safeFixed(global.vel, 2)} m/s</p>
      </div>
    </div>
  );
};
```

**Data Used:**
- `telemetry.global.lat` → Latitude (displayed with 7 decimals)
- `telemetry.global.lon` → Longitude (displayed with 7 decimals)
- `telemetry.global.alt_rel` → Relative altitude in meters (1 decimal)
- `telemetry.global.vel` → Ground speed in m/s (2 decimals)
- `telemetry.global.satellites_visible` → Satellite count

---

### B. MapView

**File:** `src/components/MapView.tsx`

Uses GPS position for rover marker and tracking:

```typescript
interface MapViewProps {
  roverPosition?: { lat: number; lng: number } | null;  // ← GPS position
  heading?: number | null;
  telemetry?: {
    speed?: number;        // telemetry.global.vel
    altitude?: number;     // telemetry.global.alt_rel
    satellites?: number;   // telemetry.global.satellites_visible
  };
  // ...
}

const MapView: React.FC<MapViewProps> = ({
  roverPosition,
  heading,
  // ...
}) => {
  
  // Create/update rover marker
  useEffect(() => {
    if (roverPosition) {
      if (!roverMarkerRef.current) {
        // Create marker at GPS position
        roverMarkerRef.current = L.marker(
          [roverPosition.lat, roverPosition.lng], 
          { icon: roverIcon }
        ).addTo(mapRef.current);
      } else {
        // Update marker position
        roverMarkerRef.current.setLatLng([roverPosition.lat, roverPosition.lng]);
      }
    }
  }, [roverPosition, heading]);

  // Trail recording
  useEffect(() => {
    if (roverPosition) {
      // Add to trail if position changed
      if (Math.abs(lastPoint.lat - roverPosition.lat) > 0.00001 || 
          Math.abs(lastPoint.lng - roverPosition.lng) > 0.00001) {
        filtered.push({ 
          lat: roverPosition.lat, 
          lng: roverPosition.lng, 
          timestamp: now 
        });
      }
    }
  }, [roverPosition]);

  // Center on rover with Ctrl+C
  const handleKeyDown = (e: KeyboardEvent) => {
    if (e.ctrlKey && e.key === 'c') {
      if (roverPosition) {
        map.panTo([roverPosition.lat, roverPosition.lng]);
      }
    }
  };

  // Distance to next waypoint
  const distanceToNextWaypoint = useMemo(() => {
    if (!roverPosition || !activeWaypointIndex) return null;
    const nextWaypoint = pathWaypoints[activeWaypointIndex];
    if (!nextWaypoint) return null;
    
    return calculateDistance(
      roverPosition, 
      { lat: nextWaypoint.lat, lng: nextWaypoint.lng }
    );
  }, [roverPosition, activeWaypointIndex, pathWaypoints]);
};
```

**Data Used:**
- `roverPosition.lat` → Rover marker latitude
- `roverPosition.lng` → Rover marker longitude
- Used for:
  - Marker placement on map
  - Trail/path recording
  - Distance calculations to waypoints
  - Auto-centering map (Ctrl+C)

---

## 6. Data Precision & Validation

### Precision
- **Latitude/Longitude:** Displayed with 7 decimal places (~11mm precision)
- **Altitude:** 1 decimal place (0.1m precision)
- **Speed:** 2 decimal places (0.01 m/s precision)

### Validation
```typescript
// Check if position is valid (finite numbers)
const position = 
  Number.isFinite(telemetry.global.lat) && Number.isFinite(telemetry.global.lon)
    ? { lat: telemetry.global.lat, lng: telemetry.global.lon }
    : null;

// Safe display with fallback
const safeFixed = (value: number, digits: number): string => {
  if (!Number.isFinite(value)) {
    return '—';  // Em dash for invalid values
  }
  return value.toFixed(digits);
};
```

---

## 7. Debug Logging

GPS data logging points:

### In useRoverROS.ts:
```typescript
// When envelope applied
console.log('[useRoverROS applyEnvelope] RTK Updated:', {
  fix_type: next.rtk.fix_type,
  baseline_age: next.rtk.baseline_age,
  satellites: next.global.satellites_visible
});

// When state snapshot updated
console.log('[useRoverROS] Telemetry State Snapshot Updated:', {
  'position': `${next.global.lat.toFixed(6)}, ${next.global.lon.toFixed(6)}`,
  'satellites': next.global.satellites_visible,
  'timestamp': new Date().toLocaleTimeString()
});
```

### In App.tsx:
```typescript
console.log('[APP.TSX toRoverData] RTK Data:', {
  'position': position,
  'satellites_visible': telemetry.global.satellites_visible,
  'connectionState': connectionState
});
```

---

## 8. Summary

### GPS Data Fields

| Source Field | Type | Destination | Usage |
|-------------|------|-------------|-------|
| `telemetry.global.lat` | number | TelemetryPanel, MapView | Latitude |
| `telemetry.global.lon` | number | TelemetryPanel, MapView | Longitude |
| `telemetry.global.alt_rel` | number | TelemetryPanel | Relative altitude |
| `telemetry.global.vel` | number | TelemetryPanel | Ground speed |
| `telemetry.global.satellites_visible` | number | TelemetryPanel | Satellite count |

### Key Transform Points

1. **Backend → Socket.IO:** `telemetry` or `rover_data` events
2. **Socket → Hook:** `toTelemetryEnvelopeFromBridge()` or `toTelemetryEnvelopeFromRoverData()`
3. **Hook → Context:** `applyEnvelope()` merges into `RoverTelemetry`
4. **Context → App:** `toRoverData()` creates UI-friendly format
5. **App → Components:** Props (`roverPosition`, `telemetry`)

### Current Data Sources

The system accepts GPS data from:
- ✅ `telemetry.global.latitude/longitude` (from bridge)
- ✅ `telemetry.position.latitude/longitude` (alternative)
- ✅ `rover_data.position.lat/lng` (from rover state)

All sources converge into `telemetry.global` in the hook.
