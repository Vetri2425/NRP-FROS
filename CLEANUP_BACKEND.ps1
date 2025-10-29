# ============================================
# FRONTEND-ONLY CLEANUP SCRIPT (PowerShell)
# ============================================
# This script removes all backend-related files
# Run this in PowerShell from the repository root

Write-Host "🧹 Starting cleanup of backend files..." -ForegroundColor Cyan

# Backend folders to remove
$foldersToRemove = @(
    "Backend",
    "clients",
    "utils"  # Duplicate of src/utils
)

# Backend files to remove
$filesToRemove = @(
    "generate_pdf.sh",
    "health_check.sh",
    "start_service.sh",
    "view_report.sh",
    "LIVE_CONTROLS_FIX.md",
    "MISSION_UPLOAD_FIX.md",
    "PROJECT_REPORT.html",
    "PROJECT_REPORT.pdf",
    "QUICKSTART.md",
    "REPORT_SUMMARY.md",
    "SERVICE_COMMANDS.md",
    "tmp_write_check.txt",
    "metadata.json"
)

# Remove folders
foreach ($folder in $foldersToRemove) {
    if (Test-Path $folder) {
        Write-Host "  Removing folder: $folder" -ForegroundColor Yellow
        Remove-Item -Path $folder -Recurse -Force
    } else {
        Write-Host "  Folder not found (already removed?): $folder" -ForegroundColor Gray
    }
}

# Remove files
foreach ($file in $filesToRemove) {
    if (Test-Path $file) {
        Write-Host "  Removing file: $file" -ForegroundColor Yellow
        Remove-Item -Path $file -Force
    } else {
        Write-Host "  File not found (already removed?): $file" -ForegroundColor Gray
    }
}

Write-Host ""
Write-Host "✅ Cleanup complete!" -ForegroundColor Green
Write-Host ""
Write-Host "📁 Frontend files preserved:" -ForegroundColor Cyan
Write-Host "   - src/" -ForegroundColor White
Write-Host "   - package.json, package-lock.json" -ForegroundColor White
Write-Host "   - vite.config.ts, tsconfig.json" -ForegroundColor White
Write-Host "   - tailwind.config.js, postcss.config.js" -ForegroundColor White
Write-Host "   - index.html, index.tsx" -ForegroundColor White
Write-Host "   - .env, .gitignore" -ForegroundColor White
Write-Host "   - README.md (updated)" -ForegroundColor White
Write-Host ""
Write-Host "🔄 Next steps:" -ForegroundColor Cyan
Write-Host "   1. Run: npm install" -ForegroundColor White
Write-Host "   2. Run: npm run dev" -ForegroundColor White
Write-Host "   3. Commit changes: git add ." -ForegroundColor White
Write-Host "   4. Then: git commit -m 'Frontend-only cleanup'" -ForegroundColor White
Write-Host "   5. Push to GitHub: git push origin V3_Oct_29" -ForegroundColor White
Write-Host ""
