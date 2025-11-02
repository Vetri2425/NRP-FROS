# 🔍 Debug Print Statements Guide

## Console Log Locations

I've added comprehensive debug logging throughout the RTK data flow. Here's where to find them:

---

## 📍 Print Statement Locations

### **1. Socket.IO Event Handlers** (`src/hooks/useRoverROS.ts`)

#### **Location**: Lines ~498-510
```typescript
const handleBridgeTelemetry = useCallback((payload: any) => {
  console.log('[RTK DEBUG] telemetry event received:', payload);
  if (payload.rtk) {
    console.log('[RTK DEBUG] telemetry.rtk:', payload.rtk);
  }
  if (payload.global) {
    console.log('[RTK DEBUG] telemetry.global:', payload.global);
  }
  // ...
});
```

**What it shows**: Raw data received from backend via Socket.IO `'telemetry'` event

---

#### **Location**: Lines ~515-523
```typescript
const handleRoverData = useCallback((payload: any) => {
  console.log('[RTK DEBUG] rover_data event received:', payload);
  if (payload.rtk_status) {
    console.log('[RTK DEBUG] rover_data.rtk_status:', payload.rtk_status);
  }
  // ...
});
```

**What it shows**: Raw data received from backend via Socket.IO `'rover_data'` event

---

### **2. RTK Data Processing** (`src/hooks/useRoverROS.ts`)

#### **Location**: Lines ~188-196 (in toTelemetryEnvelopeFromRoverData)
```typescript
console.log('[RTK DEBUG] rover_data - fix_type:', fixType, 
            'rtk_status:', data.rtk_status, 
            'base_linked:', envelope.rtk.base_linked);
```

**What it shows**: How `rtk_status` string is converted to `fix_type` number

---

### **3. Telemetry State Updates** (`src/hooks/useRoverROS.ts`)

#### **Location**: Lines ~410-417 (in applyEnvelope)
```typescript
if (envelope.rtk) {
  next.rtk = { ...next.rtk, ...envelope.rtk };
  console.log('[useRoverROS applyEnvelope] RTK Updated:', {
    fix_type: next.rtk.fix_type,
    baseline_age: next.rtk.baseline_age,
    base_linked: next.rtk.base_linked,
    satellites: next.global.satellites_visible
  });
}
```

**What it shows**: RTK data merged into telemetry state

---

#### **Location**: Lines ~435-445 (in applyEnvelope)
```typescript
console.log('[useRoverROS] Telemetry State Snapshot Updated:', {
  'connection': 'connected',
  'mode': next.state.mode,
  'armed': next.state.armed,
  'position': `${next.global.lat.toFixed(6)}, ${next.global.lon.toFixed(6)}`,
  'RTK fix_type': next.rtk.fix_type,
  'satellites': next.global.satellites_visible,
  'battery': `${next.battery.voltage.toFixed(1)}V (${next.battery.percentage.toFixed(0)}%)`,
  'timestamp': new Date().toLocaleTimeString()
});
```

**What it shows**: Complete telemetry snapshot sent to React state (throttled to ~30Hz)

---

### **4. Data Transformation** (`src/App.tsx`)

#### **Location**: Lines ~66-75 (in toRoverData function)
```typescript
console.log('[APP.TSX toRoverData] RTK Data:', {
  'fix_type (raw)': telemetry.rtk.fix_type,
  'rtk_status (mapped)': rtkStatus,
  'satellites_visible': telemetry.global.satellites_visible,
  'baseline_age': telemetry.rtk.baseline_age,
  'base_linked': telemetry.rtk.base_linked,
  'position': position,
  'connectionState': connectionState
});
```

**What it shows**: How `RoverTelemetry` is transformed to `RoverData` for UI components

---

### **5. TelemetryPanel Component** (`src/components/TelemetryPanel.tsx`)

#### **Location**: Lines ~25-32
```typescript
React.useEffect(() => {
  console.log('[TelemetryPanel RTK DEBUG]', {
    fix_type: rtk.fix_type,
    baseline_age: rtk.baseline_age,
    base_linked: rtk.base_linked,
    satellites: global.satellites_visible
  });
}, [rtk.fix_type, rtk.baseline_age, rtk.base_linked, global.satellites_visible]);
```

**What it shows**: RTK data received by TelemetryPanel component

---

### **6. LiveStatusbar Component** (`src/components/live/LiveStatusbar.tsx`)

#### **Location**: Lines ~64-73
```typescript
React.useEffect(() => {
  console.log('[LiveStatusbar] Received liveRoverData:', {
    rtk_status,
    fix_type,
    satellites_visible,
    battery,
    voltage,
    'has_position': !!liveRoverData.position
  });
}, [rtk_status, fix_type, satellites_visible, battery, voltage, liveRoverData.position]);
```

**What it shows**: Data received by LiveStatusbar component

---

#### **Location**: Lines ~104-108
```typescript
console.log('[LiveStatusbar] Display values:', {
  'GPS/RTK status': rtk_status || 'No Fix',
  'Satellites': satellitesCount,
  'Battery': batteryStatus
});
```

**What it shows**: Final values displayed in the UI

---

## 📊 Expected Console Output Flow

When everything is working, you should see logs in this order:

```
1. Socket connected successfully
   ↓
2. [RTK DEBUG] telemetry event received: {state: {...}, global: {...}, rtk: {...}}
   OR
   [RTK DEBUG] rover_data event received: {rtk_status: "GPS Fix", ...}
   ↓
3. [RTK DEBUG] telemetry.rtk: {fix_type: 1, baseline_age: 0, base_linked: false}
   OR
   [RTK DEBUG] rover_data.rtk_status: "GPS Fix"
   ↓
4. [RTK DEBUG] rover_data - fix_type: 1, rtk_status: "GPS Fix", base_linked: false
   ↓
5. [useRoverROS applyEnvelope] RTK Updated: {fix_type: 1, baseline_age: 0, ...}
   ↓
6. [useRoverROS] Telemetry State Snapshot Updated: {...RTK fix_type: 1...}
   ↓
7. [APP.TSX toRoverData] RTK Data: {fix_type (raw): 1, rtk_status (mapped): "GPS Fix"}
   ↓
8. [LiveStatusbar] Received liveRoverData: {rtk_status: "GPS Fix", fix_type: 1, ...}
   ↓
9. [TelemetryPanel RTK DEBUG] {fix_type: 1, baseline_age: 0, satellites: 8}
   ↓
10. [LiveStatusbar] Display values: {GPS/RTK status: "GPS Fix", Satellites: 8}
```

---

## 🎯 How to Use These Logs

### **Step 1: Open Browser Console**
- Press `F12` → Console tab
- Refresh the page

### **Step 2: Filter Logs**
You can filter by typing in the console filter box:
- `RTK DEBUG` - Shows all RTK-related logs
- `Socket connected` - Shows connection status
- `telemetry event` - Shows backend events
- `fix_type` - Shows RTK fix type values

### **Step 3: Identify Issues**

#### ✅ **If you see all 10 log types**:
Everything is working! Data flows from backend → frontend → UI.

#### ❌ **If you DON'T see logs 1-2**:
**Problem**: Backend not connected or not running
**Check**: Is backend server running on Jetson?

#### ❌ **If you see logs 1-2 but NOT 3-4**:
**Problem**: Backend sending events but NO RTK data in payload
**Check**: Backend not including `rtk` field in telemetry

#### ❌ **If you see logs 1-5 but fix_type is always 0**:
**Problem**: Backend connected but GPS has no satellite lock
**Check**: GPS hardware, MAVROS GPS topic

#### ❌ **If you see logs 1-7 but NOT 8-10**:
**Problem**: Data transformation issue in frontend
**Check**: Check if `liveRoverData` prop is passed correctly

---

## 🔍 Quick Diagnostic Commands

### **In Browser Console, Type:**

```javascript
// Check current telemetry state
JSON.parse(localStorage.getItem('telemetry') || '{}')

// Check backend URL
BACKEND_URL

// Check Socket.IO connection status
// (Look for socket.connected in logs)
```

---

## 📝 What to Look For

### **Good Signs** ✅
```
[RTK DEBUG] telemetry.rtk: {fix_type: 1, baseline_age: 0, base_linked: false}
[useRoverROS] Telemetry State Snapshot Updated: {RTK fix_type: 1, satellites: 8}
[APP.TSX toRoverData] RTK Data: {fix_type (raw): 1, rtk_status (mapped): "GPS Fix"}
```
- `fix_type` is 1 or higher
- `satellites` is greater than 0
- Data flows through all components

### **Bad Signs** ❌
```
[RTK DEBUG] telemetry event received: {state: {...}, global: {...}}
// No rtk field!
```
OR
```
[RTK DEBUG] telemetry.rtk: {fix_type: 0, baseline_age: 0, base_linked: false}
[useRoverROS] Telemetry State Snapshot Updated: {RTK fix_type: 0, satellites: 0}
```
- No `rtk` field in payload = Backend not sending RTK data
- `fix_type: 0` always = GPS has no satellite lock

---

## 💡 Tips

1. **Too many logs?** Comment out the snapshot log (line ~435 in useRoverROS.ts) - it updates every ~33ms
2. **Want to see raw Socket.IO?** Open Network tab → WS filter → Click socket.io → Messages tab
3. **Test with mock data?** Temporarily change `DEFAULT_RTK.fix_type` to 3 in useRoverROS.ts to verify UI works

---

## 🚀 Next Steps

1. **Open browser console** (F12)
2. **Refresh page**
3. **Copy all the RTK DEBUG logs** you see
4. **Share them** so I can tell you exactly what's wrong!

The logs will show the complete journey of RTK data from backend to UI! 🔍
