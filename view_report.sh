#!/bin/bash

# Quick Report Viewer
# Opens the HTML report in default browser

echo "📄 Opening NRP ROS Project Report..."

HTML_FILE="/home/flash/NRP_ROS/PROJECT_REPORT.html"

if [ ! -f "$HTML_FILE" ]; then
    echo "❌ Report file not found!"
    exit 1
fi

# Try different methods to open the browser
if command -v xdg-open &> /dev/null; then
    xdg-open "$HTML_FILE"
elif command -v gnome-open &> /dev/null; then
    gnome-open "$HTML_FILE"
elif command -v firefox &> /dev/null; then
    firefox "$HTML_FILE" &
elif command -v google-chrome &> /dev/null; then
    google-chrome "$HTML_FILE" &
elif command -v chromium-browser &> /dev/null; then
    chromium-browser "$HTML_FILE" &
else
    echo "❌ Could not find a browser to open the report"
    echo "📍 Report location: $HTML_FILE"
    echo "🔍 Open manually in your browser"
    exit 1
fi

echo "✅ Report opened in browser!"
echo "📍 HTML: $HTML_FILE"
echo "📄 PDF:  /home/flash/NRP_ROS/PROJECT_REPORT.pdf"
