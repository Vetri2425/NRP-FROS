# 🎯 Frontend RTK Debugging - Quick Guide

## ✅ Changes Made (Frontend Only)

### **1. Fixed `toTelemetryEnvelopeFromBridge` Function**
**File**: `src/hooks/useRoverROS.ts`

**Problem**: The function was NOT extracting RTK data from the `telemetry` Socket.IO event.

**Fix**: Added extraction for:
- `data.rtk.fix_type` - RTK fix type (0-4)
- `data.rtk.baseline_age` - Age of RTK corrections
- `data.rtk.base_linked` - RTK base station connection status
- `data.global.satellites_visible` - Number of GPS satellites
- `data.battery` - Battery telemetry
- `data.mission` - Mission progress

**Impact**: Now when backend sends `telemetry` event with RTK data, frontend will properly extract and display it.

---

### **2. Enhanced `toTelemetryEnvelopeFromRoverData` Function**
**File**: `src/hooks/useRoverROS.ts`

**Problem**: Only handled `rtk_status` string, not direct `fix_type` number.

**Fix**: Now handles multiple RTK data formats:
- Direct `fix_type` number (0-4)
- String `rtk_status` ("RTK Fixed", "GPS Fix", etc.)
- `baseline_age` and `base_linked` fields

**Impact**: Works with any RTK data format the backend sends.

---

### **3. Added Debug Logging**
**Files**: 
- `src/hooks/useRoverROS.ts`
- `src/components/TelemetryPanel.tsx`

**What it logs**:
```
[RTK DEBUG] telemetry event received: {...}
[RTK DEBUG] telemetry.rtk: {fix_type: 3, baseline_age: 0.5, base_linked: true}
[RTK DEBUG] rover_data - fix_type: 3, rtk_status: "RTK Float", base_linked: true
[TelemetryPanel RTK DEBUG] {fix_type: 3, baseline_age: 0.5, base_linked: true, satellites: 12}
```

**Impact**: You can see in browser console exactly what RTK data is being received.

---

### **4. Created Debug Panel Component**
**File**: `src/components/TelemetryDebugPanel.tsx`

A visual debug panel showing all live telemetry data including RTK status.

---

## 🚀 How to Use the Debug Panel

### **Option A: Temporary Debug (Quick Test)**

Add this to your `App.tsx` temporarily:

```tsx
import TelemetryDebugPanel from './components/TelemetryDebugPanel';

// Inside your AppContent component, add:
<TelemetryDebugPanel />
```

### **Option B: Add to Specific View**

Add to any component where you want to debug:

```tsx
import TelemetryDebugPanel from './components/TelemetryDebugPanel';

return (
  <div>
    {/* Your existing content */}
    <TelemetryDebugPanel />
  </div>
);
```

---

## 🔍 Testing Steps

### **Step 1: Open Browser Console**
Press `F12` and go to Console tab. You'll see:

```
[RTK DEBUG] telemetry event received: {state: {...}, global: {...}, rtk: {...}}
```

### **Step 2: Check RTK Values**

Look for RTK debug logs. **What you should see:**

#### ✅ **GOOD - RTK Working:**
```
[RTK DEBUG] telemetry.rtk: {fix_type: 3, baseline_age: 0.5, base_linked: true}
[TelemetryPanel RTK DEBUG] {fix_type: 3, baseline_age: 0.5, base_linked: true, satellites: 12}
```
- `fix_type: 3` = RTK Float (working!)
- `satellites: 12` = GPS has lock
- RTK panel will show "RTK Float" or "RTK Fixed"

#### ⚠️ **OK - GPS Only (No RTK):**
```
[RTK DEBUG] telemetry.rtk: {fix_type: 1, baseline_age: 0, base_linked: false}
[TelemetryPanel RTK DEBUG] {fix_type: 1, baseline_age: 0, base_linked: false, satellites: 8}
```
- `fix_type: 1` = GPS Fix (no RTK corrections)
- `satellites: 8` = GPS works
- RTK panel will show "GPS Fix"

#### ❌ **BAD - No GPS:**
```
[RTK DEBUG] telemetry.rtk: {fix_type: 0, baseline_age: 0, base_linked: false}
[TelemetryPanel RTK DEBUG] {fix_type: 0, baseline_age: 0, base_linked: false, satellites: 0}
```
- `fix_type: 0` = No Fix
- `satellites: 0` = No GPS signal
- RTK panel will show "No Fix"

---

## 📊 What Each fix_type Means

| fix_type | Status | Meaning |
|----------|--------|---------|
| 0 | No Fix | No GPS signal at all |
| 1 | GPS Fix | Basic GPS (no RTK) - accuracy ~2-5m |
| 2 | DGPS | Differential GPS - accuracy ~1m |
| 3 | RTK Float | RTK with float ambiguity - accuracy ~0.5m |
| 4 | RTK Fixed | RTK with fixed ambiguity - accuracy ~2cm |

---

## 🎯 Troubleshooting Based on Console Logs

### **Case 1: No RTK logs at all**
**Problem**: Backend is not sending RTK data OR Socket.IO disconnected

**Check**:
1. Browser console shows "Socket connected successfully"?
2. Network tab shows WebSocket connection to `ws://192.168.1.100:5001`?

**Solution**: Check backend is running on Jetson

---

### **Case 2: Logs show fix_type: 0 (No Fix)**
**Problem**: GPS module has no satellite lock

**Check**:
1. Is GPS antenna connected?
2. Is rover outdoors with clear sky view?
3. On Jetson: `ros2 topic echo /mavros/global_position/global --once`

**Solution**: Backend/hardware issue - GPS module needs satellite lock

---

### **Case 3: Logs show fix_type: 1 (GPS Fix) but not RTK**
**Problem**: GPS works but no RTK corrections

**Check**:
1. Is RTK base station configured?
2. `satellites_visible` should be > 4
3. RTK requires additional NTRIP corrections

**Solution**: This is normal if you don't have RTK base station. GPS Fix is working!

---

### **Case 4: Logs show fix_type: 3 or 4 but UI shows "No Fix"**
**Problem**: Frontend display issue

**Check**:
1. Check `LiveStatusbar.tsx` uses `rtk_status` from `liveRoverData`
2. Check `TelemetryPanel.tsx` uses `rtk.fix_type`

**Solution**: Verify UI components are using the telemetry data correctly

---

## 🔧 Quick Fixes You Can Try (Frontend Only)

### **Fix 1: Force RTK Display to Show GPS Status**

If you just want to see GPS fix status (not RTK), edit `LiveStatusbar.tsx`:

```tsx
// Around line 102, change:
<StatusItem label="GPS/RTK" value="" status={rtk_status || 'No Fix'} />

// To show satellites count instead:
<StatusItem label="GPS" value={`${satellitesCount} sats`} status={rtk_status || 'No Fix'} />
```

### **Fix 2: Add Manual RTK Test Data**

To test if UI works with good RTK data, temporarily add to `useRoverROS.ts`:

```tsx
// In createDefaultTelemetry(), change:
const DEFAULT_RTK: TelemetryRtk = {
  fix_type: 3,  // Change from 0 to 3 for RTK Float
  baseline_age: 0.5,
  base_linked: true,
};
```

This will show "RTK Float" even if backend sends 0. If this shows "RTK Float" in UI, you know the problem is backend not sending fix_type > 0.

---

## ✅ Summary

**What I Fixed (Frontend Only):**
1. ✅ RTK data extraction from `telemetry` Socket.IO event
2. ✅ Enhanced RTK data parsing for multiple formats
3. ✅ Added comprehensive debug logging
4. ✅ Created visual debug panel component

**What You Need to Check:**
1. Open browser console (F12) - look for `[RTK DEBUG]` logs
2. Check if `fix_type` is > 0 (means GPS has fix)
3. If `fix_type` is always 0, the problem is backend/GPS hardware
4. If `fix_type` is > 0 but UI shows "No Fix", there's a UI bug

**Next Steps:**
1. Refresh your frontend
2. Open browser console (F12)
3. Look for RTK debug logs
4. Share what you see in the console logs!

---

## 💡 Expected Console Output (With Working GPS)

When everything is working, you should see logs like:

```
Socket connected successfully
[RTK DEBUG] telemetry event received: {state: {…}, global: {…}, rtk: {…}, battery: {…}}
[RTK DEBUG] telemetry.rtk: {fix_type: 1, baseline_age: 0, base_linked: false}
[RTK DEBUG] telemetry.global: {latitude: 34.1234567, longitude: -118.1234567, satellites_visible: 8}
[TelemetryPanel RTK DEBUG] {fix_type: 1, baseline_age: 0, base_linked: false, satellites: 8}
```

If you see `fix_type: 1` or higher, GPS is working! The frontend will display it correctly now.

---

Good luck! 🚀 Check the browser console and let me know what you see!
