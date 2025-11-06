# Disconnect Data Reset Implementation

## Overview
Implemented a comprehensive disconnect event listener system that clears all live rover data states and sets displayed parameters to 0 in both Dashboard and Live Report components when the rover disconnects. This prevents old cached data from being rendered until new data arrives after reconnection.

## Changes Made

### 1. `src/hooks/useRoverROS.ts`
Added automatic telemetry reset functionality on disconnect events:

#### New Function: `resetTelemetry`
- Created a `resetTelemetry()` callback function that:
  - Resets all telemetry data to default values (zeros)
  - Clears the mutable telemetry reference
  - Updates the telemetry snapshot state
  - Logs the reset action for debugging

#### Updated Event Handlers
Modified Socket.IO event handlers to call `resetTelemetry()`:
- **`disconnect` event**: Clears telemetry when socket disconnects
- **`connect_error` event**: Clears telemetry on connection errors
- **`teardownSocket` function**: Resets telemetry when manually disconnecting

#### Key Code Changes:
```typescript
// Reset telemetry to default state (called on disconnect)
const resetTelemetry = useCallback(() => {
  const defaultTelemetry = createDefaultTelemetry();
  mutableRef.current.telemetry = defaultTelemetry;
  mutableRef.current.lastEnvelopeTs = null;
  setTelemetrySnapshot(defaultTelemetry);
  console.log('[useRoverROS] Telemetry reset to default state');
}, []);
```

### 2. `src/App.tsx`
Added automatic state management for live report clearing on disconnect:

#### New useEffect Hook
Added a `useEffect` that monitors `connectionState` and:
- **On Disconnection**: Sets `isLiveReportCleared = true` when transitioning from 'connected' to any other state
- **On Reconnection**: Resets `isLiveReportCleared = false` when reconnecting to allow new data

#### Key Code Changes:
```typescript
// Handle disconnection: clear live report and reset states
useEffect(() => {
  const prevConnectionState = (window as any).__prevDisconnectState;
  
  // If we were connected before and now we're disconnected/error/connecting, clear live data
  if (prevConnectionState === 'connected' && connectionState !== 'connected') {
    console.log('[APP] Rover disconnected - clearing live data states');
    setIsLiveReportCleared(true);
  }
  
  // When reconnecting, reset the cleared flag to allow new data
  if (connectionState === 'connected' && prevConnectionState !== 'connected') {
    console.log('[APP] Rover reconnected - allowing new data');
    setIsLiveReportCleared(false);
  }
  
  (window as any).__prevDisconnectState = connectionState;
}, [connectionState]);
```

## How It Works

### Disconnect Flow:
1. **Socket Disconnects** → Socket.IO fires 'disconnect' event
2. **Reset Telemetry** → `resetTelemetry()` sets all telemetry to default values (zeros)
3. **Clear UI State** → App.tsx detects state change and sets `isLiveReportCleared = true`
4. **Update Components** → Components receive zeroed telemetry data via `uiRoverData`

### Reconnect Flow:
1. **Socket Reconnects** → Socket.IO fires 'connect' event
2. **Reset Cleared Flag** → App.tsx sets `isLiveReportCleared = false`
3. **Accept New Data** → Components start displaying fresh telemetry as it arrives

## Default Telemetry Values

When disconnected, all telemetry is reset to:
```typescript
{
  state: { armed: false, mode: 'UNKNOWN', system_status: 'STANDBY', heartbeat_ts: 0 },
  global: { lat: 0, lon: 0, alt_rel: 0, vel: 0, satellites_visible: 0 },
  battery: { voltage: 0, current: 0, percentage: 0 },
  rtk: { fix_type: 0, baseline_age: 0, base_linked: false },
  mission: { total_wp: 0, current_wp: 0, status: 'IDLE', progress_pct: 0 },
  servo: { servo_id: 0, active: false, last_command_ts: 0 },
  network: { connection_type: 'none', wifi_signal_strength: 0, wifi_rssi: -100, 
             interface: '', wifi_connected: false, lora_connected: false },
  hrms: 0,
  vrms: 0,
  imu_status: 'UNKNOWN',
  lastMessageTs: null
}
```

## Components Affected

### Live Report View
- **LiveStatusbar**: Displays zeroed battery, GPS, satellites, HRMS, VRMS, IMU values
- **LiveReportView**: Shows zero distance to next waypoint when `isCleared = true`
- **MapView**: Removes rover marker when position is null

### Dashboard View
- **MapView**: Removes rover marker and heading indicator
- **Telemetry Panels**: Show zero values for all metrics
- **MissionLogs**: Unaffected (logs are preserved independently)

## Benefits

1. **No Stale Data**: Users never see outdated position or telemetry information
2. **Clear Visual Feedback**: Zero values clearly indicate disconnection
3. **Automatic Reset**: No manual intervention required
4. **Clean Reconnection**: Fresh data immediately replaces zeros upon reconnection
5. **Consistent Behavior**: Both Dashboard and Live Report views handle disconnect uniformly

## Testing Recommendations

1. **Disconnect Test**: Disconnect rover and verify all live data shows zeros
2. **Reconnect Test**: Reconnect rover and verify fresh data appears
3. **Multiple Disconnect/Reconnect Cycles**: Ensure state management is robust
4. **Error State Test**: Simulate connection errors and verify data clears
5. **View Mode Switching**: Test disconnect behavior in both Dashboard and Live modes

## Console Logging

For debugging, the implementation logs:
- `[useRoverROS] Telemetry reset to default state` - when telemetry is cleared
- `[APP] Rover disconnected - clearing live data states` - when disconnect detected
- `[APP] Rover reconnected - allowing new data` - when reconnection detected
