#!/bin/bash

# GPS Topics Diagnostic Script for ROS/MAVROS
# Run this on your rover to check GPS topic subscriptions

echo "==================================="
echo "🛰️  GPS Topics Diagnostic"
echo "==================================="
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}1. Checking available GPS topics...${NC}"
GPS_TOPICS=$(rostopic list | grep -E "gps|global_position")
if [ -z "$GPS_TOPICS" ]; then
    echo -e "${RED}  ⚠️  No GPS topics found! Is MAVROS running?${NC}"
else
    echo "$GPS_TOPICS"
fi
echo ""

echo -e "${BLUE}2. Checking GPS1 Raw data (satellites & fix type)...${NC}"
GPS1_RAW=$(timeout 2s rostopic echo -n 1 /mavros/gpsstatus/gps1/raw 2>&1)
if [ $? -eq 0 ]; then
    echo "$GPS1_RAW" | grep -E "fix_type|satellites_visible|eph|epv"
    echo -e "${GREEN}  ✓ GPS1 Raw topic is publishing${NC}"
else
    echo -e "${RED}  ✗ GPS1 Raw topic is NOT publishing${NC}"
fi
echo ""

echo -e "${BLUE}3. Checking Global Position (lat/lon/alt)...${NC}"
GLOBAL_POS=$(timeout 2s rostopic echo -n 1 /mavros/global_position/global 2>&1)
if [ $? -eq 0 ]; then
    echo "$GLOBAL_POS" | grep -E "latitude|longitude|altitude" | head -n 3
    echo -e "${GREEN}  ✓ Global Position topic is publishing${NC}"
else
    echo -e "${RED}  ✗ Global Position topic is NOT publishing${NC}"
fi
echo ""

echo -e "${BLUE}4. Checking publication rates...${NC}"
echo -e "  ${YELLOW}GPS1 Raw:${NC}"
timeout 5s rostopic hz /mavros/gpsstatus/gps1/raw 2>&1 | head -n 2 || echo -e "${RED}  ✗ Not publishing${NC}"
echo -e "  ${YELLOW}Global Position:${NC}"
timeout 5s rostopic hz /mavros/global_position/global 2>&1 | head -n 2 || echo -e "${RED}  ✗ Not publishing${NC}"
echo ""

echo -e "${BLUE}5. Checking telemetry node status...${NC}"
TELEMETRY_NODE=$(rosnode list | grep telemetry)
if [ -z "$TELEMETRY_NODE" ]; then
    echo -e "${RED}  ⚠️  Telemetry node is NOT running!${NC}"
else
    echo -e "${GREEN}  ✓ Telemetry node found: $TELEMETRY_NODE${NC}"
fi
echo ""

echo -e "${BLUE}6. Checking telemetry node subscriptions...${NC}"
if [ -n "$TELEMETRY_NODE" ]; then
    rosnode info $TELEMETRY_NODE 2>&1 | grep -A 15 "Subscriptions:" || echo -e "${RED}  ✗ Cannot get node info${NC}"
else
    echo -e "${YELLOW}  ⚠️  Skipping (telemetry node not running)${NC}"
fi
echo ""

echo -e "${BLUE}7. Checking MAVROS connection...${NC}"
MAVROS_STATE=$(timeout 2s rostopic echo -n 1 /mavros/state 2>&1)
if [ $? -eq 0 ]; then
    echo "$MAVROS_STATE" | grep -E "connected|armed|mode"
    echo -e "${GREEN}  ✓ MAVROS is connected${NC}"
else
    echo -e "${RED}  ✗ MAVROS is NOT connected${NC}"
fi
echo ""

echo "==================================="
echo -e "${GREEN}✅ Diagnostic complete!${NC}"
echo "==================================="
echo ""
echo "Next steps:"
echo "  1. If GPS topics are not publishing, check MAVROS connection"
echo "  2. If telemetry node is not running, start it"
echo "  3. Compare GPS data with MAVProxy on /dev/ttyACM0"
echo ""
echo "To compare with MAVProxy:"
echo "  mavproxy.py --master=/dev/ttyACM0:57600"
echo "  Then type: gps"
echo ""
