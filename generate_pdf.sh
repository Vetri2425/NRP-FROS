#!/bin/bash

# PDF Generation Script for NRP ROS Project Report
# This script converts the HTML report to a professional PDF

echo "📄 NRP ROS Project Report - PDF Generator"
echo "=========================================="
echo ""

HTML_FILE="/home/flash/NRP_ROS/PROJECT_REPORT.html"
PDF_FILE="/home/flash/NRP_ROS/PROJECT_REPORT.pdf"

# Check if HTML file exists
if [ ! -f "$HTML_FILE" ]; then
    echo "❌ Error: HTML file not found at $HTML_FILE"
    exit 1
fi

echo "📋 Source: $HTML_FILE"
echo "📄 Output: $PDF_FILE"
echo ""

# Method 1: Try using wkhtmltopdf (best quality)
if command -v wkhtmltopdf &> /dev/null; then
    echo "🔄 Generating PDF using wkhtmltopdf..."
    wkhtmltopdf \
        --enable-local-file-access \
        --page-size A4 \
        --margin-top 10mm \
        --margin-bottom 10mm \
        --margin-left 10mm \
        --margin-right 10mm \
        --print-media-type \
        "$HTML_FILE" "$PDF_FILE"
    
    if [ $? -eq 0 ]; then
        echo "✅ PDF generated successfully!"
        echo "📍 Location: $PDF_FILE"
        ls -lh "$PDF_FILE"
        exit 0
    fi
fi

# Method 2: Try using Google Chrome/Chromium headless
if command -v google-chrome &> /dev/null; then
    CHROME_CMD="google-chrome"
elif command -v chromium-browser &> /dev/null; then
    CHROME_CMD="chromium-browser"
elif command -v chromium &> /dev/null; then
    CHROME_CMD="chromium"
else
    CHROME_CMD=""
fi

if [ -n "$CHROME_CMD" ]; then
    echo "🔄 Generating PDF using $CHROME_CMD headless..."
    $CHROME_CMD \
        --headless \
        --disable-gpu \
        --print-to-pdf="$PDF_FILE" \
        --print-to-pdf-no-header \
        --no-margins \
        "$HTML_FILE"
    
    if [ $? -eq 0 ]; then
        echo "✅ PDF generated successfully!"
        echo "📍 Location: $PDF_FILE"
        ls -lh "$PDF_FILE"
        exit 0
    fi
fi

# Method 3: Try using Firefox headless
if command -v firefox &> /dev/null; then
    echo "🔄 Attempting Firefox headless (experimental)..."
    firefox --headless --print-to-pdf="$PDF_FILE" "$HTML_FILE" 2>/dev/null
    
    if [ -f "$PDF_FILE" ]; then
        echo "✅ PDF generated successfully!"
        echo "📍 Location: $PDF_FILE"
        ls -lh "$PDF_FILE"
        exit 0
    fi
fi

# If all methods fail
echo ""
echo "❌ Could not generate PDF automatically."
echo ""
echo "📝 Alternative methods:"
echo ""
echo "1️⃣  Install wkhtmltopdf (recommended):"
echo "   sudo apt-get install wkhtmltopdf"
echo "   Then run: ./generate_pdf.sh"
echo ""
echo "2️⃣  Use browser (manual method):"
echo "   - Open: file://$HTML_FILE"
echo "   - Press Ctrl+P"
echo "   - Select 'Save as PDF'"
echo "   - Save to: $PDF_FILE"
echo ""
echo "3️⃣  Use online converter:"
echo "   - Upload HTML file to https://www.web2pdfconvert.com/"
echo "   - Download generated PDF"
echo ""

exit 1
