# 🔌 Backend Connection Guide

## ❓ Why Is Backend Not Connected?

You removed the **Backend/** folder (Python Flask + ROS server) during the frontend-only cleanup. The frontend needs a backend server to connect to for real rover data.

---

## 🎯 Solutions

### **Option 1: Connect to Remote Backend** ⭐ Recommended

If you have the backend running on another machine (Jetson, Raspberry Pi, or another PC):

#### Step 1: Update `.env` file

Edit `.env` and set your backend server IP address:

```properties
# Replace 192.168.1.27 with your actual backend server IP
VITE_ROS_HTTP_BASE=http://192.168.1.27:5001
VITE_ROS_WS_URL=ws://192.168.1.27:5001/ws/telemetry
```

#### Step 2: Restart the dev server

```powershell
# Stop current server (Ctrl+C)
# Then restart:
npm run dev
```

#### Step 3: Verify connection

Open the browser console (F12) and check for:
- ✅ "Socket connected successfully"
- ❌ "Socket.IO connection error" (if backend is not running)

---

### **Option 2: Run Mock/Demo Mode**

If you don't have a backend available, you can:

1. **Use static demo data** - Create mock telemetry data
2. **Connect to a simulator** - Use SITL (Software In The Loop)

---

### **Option 3: Clone and Run the Backend Locally**

If you want to run the backend on your Windows machine:

#### Step 1: Clone the original repository

```powershell
cd "d:\Users\FLASH\Desktop\Virtual _Rover"
git clone https://github.com/Vetri2425/NRP_ROS_V2.git NRP_ROS_BACKEND
cd NRP_ROS_BACKEND
git checkout <original-branch>
```

#### Step 2: Install Python and dependencies

```powershell
# Install Python 3.10+ if not installed
# Then create virtual environment
python -m venv venv
.\venv\Scripts\Activate.ps1

# Install dependencies
cd Backend
pip install -r requirements.txt
```

#### Step 3: Run the backend server

```powershell
python server.py
```

#### Step 4: Update frontend `.env`

```properties
VITE_ROS_HTTP_BASE=http://localhost:5001
VITE_ROS_WS_URL=ws://localhost:5001/ws/telemetry
```

---

## 🔍 How to Check Backend Status

### Check if backend is reachable:

```powershell
# Test HTTP connection
curl http://192.168.1.27:5001/api/health

# Or in PowerShell:
Invoke-WebRequest -Uri "http://192.168.1.27:5001/api/health"
```

### Expected response:
```json
{
  "status": "ok",
  "message": "Backend is running"
}
```

---

## 📡 Current Configuration

Your frontend is now configured to:

1. **First Priority**: Use `VITE_ROS_HTTP_BASE` from `.env` file
2. **Fallback**: Use `window.location.hostname:5001` (same machine as frontend)

### To check your current backend URL:

Open browser console (F12) and type:
```javascript
console.log(window.location.hostname)
```

---

## 🐛 Troubleshooting

### Issue: "Socket.IO connection error"

**Causes:**
- Backend server is not running
- Wrong IP address in `.env`
- Firewall blocking port 5001
- Backend is on different network

**Solutions:**
1. Verify backend is running: `curl http://<backend-ip>:5001/api/health`
2. Check firewall settings
3. Ensure both machines are on same network
4. Try `http://localhost:5001` if backend is local

### Issue: "CORS error"

**Solution:** Backend needs to allow CORS from your frontend origin. Update backend `server.py`:

```python
from flask_cors import CORS
app = Flask(__name__)
CORS(app, origins=["http://localhost:3000", "http://<your-ip>:3000"])
```

### Issue: Connection works but no telemetry data

**Causes:**
- Backend not connected to rover/simulator
- MAVLink connection issues
- ROS bridge not running

**Solutions:**
1. Check backend logs
2. Verify MAVLink connection string
3. Test with SITL simulator first

---

## 📋 Backend Server Locations

Common places where your backend might be running:

- **Jetson Nano**: Usually runs the ROS + Python backend
- **Raspberry Pi**: Alternative rover computer
- **Desktop/Laptop**: For development with SITL
- **WSL (Windows)**: If running ROS on Windows via WSL

---

## 🌐 Network Configuration

### Same Machine (localhost)
```properties
VITE_ROS_HTTP_BASE=http://localhost:5001
```

### Same WiFi Network
```properties
# Find backend IP: ipconfig (Windows) or ifconfig (Linux)
VITE_ROS_HTTP_BASE=http://192.168.1.27:5001
```

### Different Network (Internet)
```properties
# Requires port forwarding and public IP or domain
VITE_ROS_HTTP_BASE=http://your-domain.com:5001
```

---

## ✅ Quick Checklist

- [ ] Backend server is running (test with curl)
- [ ] `.env` has correct `VITE_ROS_HTTP_BASE`
- [ ] Port 5001 is not blocked by firewall
- [ ] Both machines on same network (if applicable)
- [ ] Restarted dev server after `.env` changes
- [ ] Browser console shows connection status

---

## 💡 Recommended Setup for Development

1. **Backend**: Run on Jetson/rover computer at `192.168.1.27:5001`
2. **Frontend**: Run on Windows laptop at `localhost:3000`
3. **Connection**: Frontend connects to backend via WiFi

This allows you to:
- Develop UI on Windows
- Test with real rover hardware
- Hot-reload frontend changes instantly

---

**Need more help?** Check the browser console (F12) for connection errors and logs.
