#!/bin/bash

# Comprehensive GPS Diagnostics for Rover
# Run this script on the rover after SSH

echo "========================================="
echo "🛰️  ROVER GPS DIAGNOSTICS"
echo "========================================="
echo ""

# Source ROS environment
echo "Setting up ROS environment..."
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash 2>/dev/null || source ~/ros_ws/devel/setup.bash 2>/dev/null || echo "Using system ROS only"
echo ""

# 1. Check GPS Topics
echo "========================================="
echo "1️⃣  CHECKING GPS TOPICS"
echo "========================================="
rostopic list | grep -E "gps|global_position"
echo ""

# 2. Check GPS1 Raw
echo "========================================="
echo "2️⃣  GPS1 RAW DATA (Satellites & Fix Type)"
echo "========================================="
timeout 3s rostopic echo -n 1 /mavros/gpsstatus/gps1/raw 2>/dev/null || echo "❌ Topic not available or not publishing"
echo ""

# 3. Check Global Position
echo "========================================="
echo "3️⃣  GLOBAL POSITION (Lat/Lon/Alt)"
echo "========================================="
timeout 3s rostopic echo -n 1 /mavros/global_position/global 2>/dev/null || echo "❌ Topic not available or not publishing"
echo ""

# 4. Check Topic Rates
echo "========================================="
echo "4️⃣  PUBLICATION RATES"
echo "========================================="
echo "GPS1 Raw rate:"
timeout 5s rostopic hz /mavros/gpsstatus/gps1/raw 2>&1 | head -n 2
echo ""
echo "Global Position rate:"
timeout 5s rostopic hz /mavros/global_position/global 2>&1 | head -n 2
echo ""

# 5. Check Telemetry Node
echo "========================================="
echo "5️⃣  TELEMETRY NODE STATUS"
echo "========================================="
rosnode list | grep telemetry
echo ""

# 6. Check Telemetry Node Subscriptions
echo "========================================="
echo "6️⃣  TELEMETRY NODE SUBSCRIPTIONS"
echo "========================================="
TELEM_NODE=$(rosnode list | grep telemetry | head -n 1)
if [ -n "$TELEM_NODE" ]; then
    rosnode info $TELEM_NODE | grep -A 20 "Subscriptions:"
else
    echo "❌ Telemetry node not found"
fi
echo ""

# 7. Check MAVROS State
echo "========================================="
echo "7️⃣  MAVROS CONNECTION STATUS"
echo "========================================="
timeout 3s rostopic echo -n 1 /mavros/state 2>/dev/null || echo "❌ MAVROS not connected"
echo ""

# 8. Check Running Nodes
echo "========================================="
echo "8️⃣  RUNNING ROS NODES"
echo "========================================="
rosnode list
echo ""

# 9. Summary
echo "========================================="
echo "📊 SUMMARY"
echo "========================================="
echo "Total GPS topics:" $(rostopic list | grep -E "gps|global_position" | wc -l)
echo "Telemetry nodes:" $(rosnode list | grep -c telemetry)
echo "MAVROS nodes:" $(rosnode list | grep -c mavros)
echo ""

echo "========================================="
echo "✅ DIAGNOSTIC COMPLETE!"
echo "========================================="
echo ""
echo "Next step: Compare with MAVProxy"
echo "Run: mavproxy.py --master=/dev/ttyACM0:57600"
echo "Then type: gps"
echo ""
