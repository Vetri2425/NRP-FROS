# 🚀 Quick Backend Testing Guide

## Test 1: Check Backend Server Status

```bash
# SSH to Jetson
ssh flash@192.168.1.101

# Check if server.py is running
ps aux | grep server.py

# If running, check the process:
# Expected: python3 server.py or /usr/bin/python3 server.py

# If NOT running, start it:
cd ~/NRP_ROS/Backend
python3 server.py 2>&1
```

---

## Test 2: Test Health Check Endpoint

```bash
# From Windows PowerShell
curl -X GET http://192.168.1.101:5001/api/health

# Or using Invoke-WebRequest (more PowerShell-native):
$response = Invoke-WebRequest -Uri "http://192.168.1.101:5001/api/health" -Method GET
$response.StatusCode
$response.Content | ConvertFrom-Json
```

**Expected Status:** 200 OK  
**Expected Response:**
```json
{
  "success": true,
  "status": "healthy",
  "services": {
    "ros": "connected",
    "mavros": "ready"
  }
}
```

**If Returns 404 or 500:** The endpoint structure is broken.

---

## Test 3: Test Simple Mission Upload

```bash
# PowerShell script to test mission upload
$url = "http://192.168.1.101:5001/api/mission/upload"

$payload = @{
    waypoints = @(
        @{
            lat = 1307207955     # 13.07207955° × 1e7
            lng = -8026193800    # -80.26193800° × 1e7
            alt = 10
            seq = 0
            cmd = 16
        }
    )
} | ConvertTo-Json

Write-Host "Sending mission upload request..."
Write-Host "URL: $url"
Write-Host "Payload: $payload"

try {
    $response = Invoke-WebRequest -Uri $url `
        -Method POST `
        -ContentType "application/json" `
        -Body $payload `
        -ErrorAction Stop
    
    Write-Host "✅ Status: $($response.StatusCode)"
    Write-Host "Response: $($response.Content)"
} catch {
    Write-Host "❌ Error: $($_.Exception.Message)"
    Write-Host "Status Code: $($_.Exception.Response.StatusCode.Value)"
    Write-Host "Response: $($_.Exception.Response | ConvertFrom-Json | ConvertTo-Json)"
}
```

**Expected Status:** 200 OK or 400 (with detailed error message)  
**Worst Case Status:** 500 (unhandled exception)

---

## Test 4: Get EXACT Error Message from Backend

```bash
# PowerShell - This will show the actual error message
$url = "http://192.168.1.101:5001/api/mission/upload"

$payload = @{
    waypoints = @(
        @{ lat = 1307207955; lng = -8026193800; alt = 10; seq = 0; cmd = 16 }
    )
} | ConvertTo-Json

try {
    $response = Invoke-WebRequest -Uri $url -Method POST `
        -ContentType "application/json" -Body $payload
    Write-Host "Success: $($response.Content)"
} catch {
    # This catches the 500 error and shows what backend sent
    $errorResponse = $_.Exception.Response.GetResponseStream()
    $streamReader = [System.IO.StreamReader]::new($errorResponse)
    $errorContent = $streamReader.ReadToEnd()
    $streamReader.Close()
    
    Write-Host "⚠️  Status: $($_.Exception.Response.StatusCode)"
    Write-Host "Error Details: $errorContent"
}
```

---

## Test 5: Check MAVROS Bridge Connectivity

```bash
# From Jetson terminal (via SSH)
ssh flash@192.168.1.101 "ros2 node list | grep mavros"

# Should show: /mavros if MAVROS bridge is running

# Check specific MAVROS services:
ssh flash@192.168.1.101 "ros2 service list | grep mavros | head -20"

# Check mission services:
ssh flash@192.168.1.101 "ros2 service list | grep mission"

# Should include:
# /mavros/mission/pull
# /mavros/mission/push
# /mavros/mission/clear
```

---

## Test 6: Check Backend Logs

```bash
# SSH to Jetson
ssh flash@192.168.1.101

# Check if log file exists:
ls -la ~/NRP_ROS/Backend/server.log

# View last 50 lines of log:
tail -50 ~/NRP_ROS/Backend/server.log

# Or watch logs in real-time while you test:
tail -f ~/NRP_ROS/Backend/server.log

# Then open another terminal and trigger mission upload from frontend
```

---

## Test 7: Test Mission Download (Verify Precision)

```bash
# PowerShell - Download mission and check waypoint precision
$url = "http://192.168.1.101:5001/api/mission/download"

try {
    $response = Invoke-WebRequest -Uri $url -Method GET
    $mission = $response.Content | ConvertFrom-Json
    
    Write-Host "✅ Mission Downloaded Successfully"
    Write-Host "Waypoints: $($mission.waypoints | ConvertTo-Json)"
    
    # Check precision of first waypoint
    $wp = $mission.waypoints[0]
    Write-Host "`nFirst Waypoint:"
    Write-Host "  Lat: $($wp.lat)"
    Write-Host "  Lng: $($wp.lng)"
    Write-Host "  Alt: $($wp.alt)"
    
    # Show how many decimal digits
    $latStr = $wp.lat.ToString()
    $decimalPlaces = if ($latStr.Contains('.')) { 
        $latStr.Split('.')[1].Length 
    } else { 
        0 
    }
    Write-Host "  Decimal Places: $decimalPlaces"
} catch {
    Write-Host "❌ Error downloading mission: $($_.Exception.Message)"
}
```

---

## Summary of Expected Results

| Test | Expected Status | Issue if Different |
|------|-----------------|-------------------|
| Health Check | 200 OK | Endpoint not registered or MAVROS down |
| Mission Upload | 200 OK | Unhandled exception in _handle_upload_mission() |
| Mission Download | 200 OK with waypoints | Mission storage or MAVROS retrieval failure |
| MAVROS Bridge | Connected | ROS2/MAVROS not running on Jetson |
| Backend Process | Running | Server crashed or stopped |

---

## 📝 Information to Share After Testing

Please provide:
1. ✅ Backend process status (`ps aux | grep server.py`)
2. ✅ Health check response (status code + body)
3. ✅ Mission upload curl response (exact error message)
4. ✅ Backend server logs (last 50 lines)
5. ✅ MAVROS connectivity status (`ros2 node list`)
6. ✅ Any error messages from browser console

This will help us diagnose and fix the HTTP 500 error! 🚀
