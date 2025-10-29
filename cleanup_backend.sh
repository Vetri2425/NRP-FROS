#!/bin/bash
# ============================================
# FRONTEND-ONLY CLEANUP SCRIPT (Bash/Linux)
# ============================================
# This script removes all backend-related files
# Run this in bash/zsh from the repository root
# Make executable: chmod +x cleanup_backend.sh

echo "🧹 Starting cleanup of backend files..."

# Backend folders to remove
FOLDERS_TO_REMOVE=(
    "Backend"
    "clients"
    "utils"  # Duplicate of src/utils
)

# Backend files to remove
FILES_TO_REMOVE=(
    "generate_pdf.sh"
    "health_check.sh"
    "start_service.sh"
    "view_report.sh"
    "LIVE_CONTROLS_FIX.md"
    "MISSION_UPLOAD_FIX.md"
    "PROJECT_REPORT.html"
    "PROJECT_REPORT.pdf"
    "QUICKSTART.md"
    "REPORT_SUMMARY.md"
    "SERVICE_COMMANDS.md"
    "tmp_write_check.txt"
    "metadata.json"
)

# Remove folders
for folder in "${FOLDERS_TO_REMOVE[@]}"; do
    if [ -d "$folder" ]; then
        echo "  Removing folder: $folder"
        rm -rf "$folder"
    else
        echo "  Folder not found (already removed?): $folder"
    fi
done

# Remove files
for file in "${FILES_TO_REMOVE[@]}"; do
    if [ -f "$file" ]; then
        echo "  Removing file: $file"
        rm -f "$file"
    else
        echo "  File not found (already removed?): $file"
    fi
done

echo ""
echo "✅ Cleanup complete!"
echo ""
echo "📁 Frontend files preserved:"
echo "   - src/"
echo "   - package.json, package-lock.json"
echo "   - vite.config.ts, tsconfig.json"
echo "   - tailwind.config.js, postcss.config.js"
echo "   - index.html, index.tsx"
echo "   - .env, .gitignore"
echo "   - README.md (updated)"
echo ""
echo "🔄 Next steps:"
echo "   1. Run: npm install"
echo "   2. Run: npm run dev"
echo "   3. Commit changes: git add . && git commit -m 'Frontend-only cleanup'"
echo "   4. Push to GitHub: git push origin V3_Oct_29"
echo ""
