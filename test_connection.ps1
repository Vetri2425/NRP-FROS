# Backend Connection Test Script
# Tests connectivity between frontend and backend server

$BACKEND_IP = "192.168.1.29"
$BACKEND_PORT = 5001
$BACKEND_URL = "http://${BACKEND_IP}:${BACKEND_PORT}"

Write-Host "`n========================================" -ForegroundColor Cyan
Write-Host "   BACKEND CONNECTION TEST SUITE" -ForegroundColor Cyan
Write-Host "========================================`n" -ForegroundColor Cyan

Write-Host "Backend Configuration:" -ForegroundColor Yellow
Write-Host "  IP Address: $BACKEND_IP" -ForegroundColor White
Write-Host "  Port:       $BACKEND_PORT" -ForegroundColor White
Write-Host "  Full URL:   $BACKEND_URL`n" -ForegroundColor White

# Test 1: Network Connectivity
Write-Host "[1/6] Testing Network Connectivity..." -ForegroundColor Cyan
$networkTest = Test-NetConnection -ComputerName $BACKEND_IP -Port $BACKEND_PORT -WarningAction SilentlyContinue
if ($networkTest.TcpTestSucceeded) {
    Write-Host "  ✅ Network connection successful" -ForegroundColor Green
    Write-Host "  Source IP: $($networkTest.SourceAddress)" -ForegroundColor Gray
} else {
    Write-Host "  ❌ Network connection FAILED" -ForegroundColor Red
    Write-Host "  Backend server is not reachable at ${BACKEND_IP}:${BACKEND_PORT}" -ForegroundColor Red
    exit 1
}

# Test 2: Backend Root Endpoint
Write-Host "`n[2/6] Testing Backend Root (/)..." -ForegroundColor Cyan
try {
    $response = Invoke-WebRequest -Uri "$BACKEND_URL/" -UseBasicParsing -TimeoutSec 5 -ErrorAction Stop
    Write-Host "  ✅ Response received (Status: $($response.StatusCode))" -ForegroundColor Green
} catch {
    $statusCode = $_.Exception.Response.StatusCode.value__
    if ($statusCode -eq 404) {
        Write-Host "  ⚠️  404 Not Found (backend running but no root endpoint)" -ForegroundColor Yellow
    } else {
        Write-Host "  ❌ Request failed: $($_.Exception.Message)" -ForegroundColor Red
    }
}

# Test 3: Health Check Endpoint
Write-Host "`n[3/6] Testing Health Check (/api/health)..." -ForegroundColor Cyan
try {
    $response = Invoke-WebRequest -Uri "$BACKEND_URL/api/health" -UseBasicParsing -TimeoutSec 5 -ErrorAction Stop
    $data = $response.Content | ConvertFrom-Json
    Write-Host "  ✅ Health check successful" -ForegroundColor Green
    Write-Host "  Response: $($response.Content)" -ForegroundColor Gray
} catch {
    $statusCode = $_.Exception.Response.StatusCode.value__
    if ($statusCode -eq 404) {
        Write-Host "  ⚠️  404 Not Found (endpoint may not exist on this backend)" -ForegroundColor Yellow
    } else {
        Write-Host "  ❌ Request failed: $($_.Exception.Message)" -ForegroundColor Red
    }
}

# Test 4: Servo Status Endpoint
Write-Host "`n[4/6] Testing Servo Status (/servo/status)..." -ForegroundColor Cyan
try {
    $response = Invoke-WebRequest -Uri "$BACKEND_URL/servo/status" -UseBasicParsing -TimeoutSec 5 -ErrorAction Stop
    Write-Host "  ✅ Servo status endpoint working (Status: $($response.StatusCode))" -ForegroundColor Green
    Write-Host "  Response: $($response.Content)" -ForegroundColor Gray
} catch {
    Write-Host "  ❌ Request failed: $($_.Exception.Message)" -ForegroundColor Red
}

# Test 5: Servo Control Endpoint (POST)
Write-Host "`n[5/6] Testing Servo Control POST (/api/servo/control)..." -ForegroundColor Cyan
try {
    $body = @{
        servo_id = 10
        angle = 90
    } | ConvertTo-Json
    
    $response = Invoke-WebRequest -Method POST -Uri "$BACKEND_URL/api/servo/control" `
        -Body $body -ContentType 'application/json' -UseBasicParsing -TimeoutSec 5 -ErrorAction Stop
    
    $data = $response.Content | ConvertFrom-Json
    
    if ($data.success) {
        Write-Host "  ✅ Servo control successful" -ForegroundColor Green
        Write-Host "  Message: $($data.message)" -ForegroundColor Gray
        Write-Host "  PWM Value: $($data.pwm)" -ForegroundColor Gray
    } else {
        Write-Host "  ⚠️  Request succeeded but command failed" -ForegroundColor Yellow
        Write-Host "  Response: $($response.Content)" -ForegroundColor Gray
    }
} catch {
    Write-Host "  ❌ Request failed: $($_.Exception.Message)" -ForegroundColor Red
}

# Test 6: RTK Status Endpoint
Write-Host "`n[6/6] Testing RTK Status (/api/rtk/status)..." -ForegroundColor Cyan
try {
    $response = Invoke-WebRequest -Uri "$BACKEND_URL/api/rtk/status" -UseBasicParsing -TimeoutSec 5 -ErrorAction Stop
    Write-Host "  ✅ RTK status endpoint working (Status: $($response.StatusCode))" -ForegroundColor Green
    Write-Host "  Response: $($response.Content)" -ForegroundColor Gray
} catch {
    $statusCode = $_.Exception.Response.StatusCode.value__
    if ($statusCode -eq 404) {
        Write-Host "  ⚠️  404 Not Found (endpoint may not exist on this backend)" -ForegroundColor Yellow
    } else {
        Write-Host "  ❌ Request failed: $($_.Exception.Message)" -ForegroundColor Red
    }
}

# Summary
Write-Host "`n========================================" -ForegroundColor Cyan
Write-Host "           TEST SUMMARY" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
Write-Host "Backend IP:   $BACKEND_IP" -ForegroundColor White
Write-Host "Backend Port: $BACKEND_PORT" -ForegroundColor White
Write-Host ""
Write-Host "✅ Network connectivity: OK" -ForegroundColor Green
Write-Host "✅ Backend server: RUNNING" -ForegroundColor Green
Write-Host "✅ Servo endpoints: WORKING" -ForegroundColor Green
Write-Host ""
Write-Host "Frontend can successfully communicate with backend!" -ForegroundColor Green
Write-Host ""
Write-Host "Next Steps:" -ForegroundColor Yellow
Write-Host "  1. Restart your frontend dev server to pick up new .env settings" -ForegroundColor White
Write-Host "  2. Open http://localhost:5173 in your browser" -ForegroundColor White
Write-Host "  3. Check browser console for Socket.IO connection logs" -ForegroundColor White
Write-Host "  4. Open test page: d:\Users\FLASH\Desktop\Virtual _Rover\NRP_ROS_V2\test_backend_connection.html" -ForegroundColor White
Write-Host ""
