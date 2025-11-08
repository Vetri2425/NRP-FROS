# 🎯 Frontend-Only Cleanup Guide

## ✅ What's Been Done

1. **Updated `.gitignore`** - Added proper frontend ignore patterns
2. **Updated `README.md`** - New frontend-only installation instructions
3. **Created cleanup scripts** - Both PowerShell and Bash versions

## 🗂️ Files to Delete

### Folders:
- `Backend/` - Python Flask + ROS backend
- `clients/` - WinForms client
- `utils/` - Duplicate of `src/utils`

### Files:
- `generate_pdf.sh`
- `health_check.sh`
- `start_service.sh`
- `view_report.sh`
- `LIVE_CONTROLS_FIX.md`
- `MISSION_UPLOAD_FIX.md`
- `PROJECT_REPORT.html`
- `PROJECT_REPORT.pdf`
- `QUICKSTART.md`
- `REPORT_SUMMARY.md`
- `SERVICE_COMMANDS.md`
- `tmp_write_check.txt`
- `metadata.json`

## 📦 Files to Keep

### Frontend Core:
- `src/` - All React components and code
- `index.html` - Entry HTML file
- `index.tsx` - Alternative entry point
- `package.json` - Dependencies
- `package-lock.json` - Lock file
- `.env` - Environment variables

### Configuration:
- `vite.config.ts` - Vite bundler config
- `tsconfig.json` - TypeScript config
- `tailwind.config.js` - Tailwind CSS config
- `postcss.config.js` - PostCSS config
- `vite-env.d.ts` - Vite type definitions

### Version Control:
- `.git/` - Git repository (KEEP!)
- `.gitignore` - Updated ignore file
- `.vscode/` - VSCode settings (optional)

### Documentation:
- `README.md` - Updated frontend instructions

## 🚀 Quick Start Commands

### Option 1: Run the cleanup script (Recommended)

**Windows PowerShell:**
```powershell
.\CLEANUP_BACKEND.ps1
```

**Linux/Mac Bash:**
```bash
chmod +x cleanup_backend.sh
./cleanup_backend.sh
```

### Option 2: Manual deletion commands

**Windows PowerShell:**
```powershell
# Remove folders
Remove-Item -Path "Backend" -Recurse -Force
Remove-Item -Path "clients" -Recurse -Force
Remove-Item -Path "utils" -Recurse -Force

# Remove files
Remove-Item -Path "generate_pdf.sh","health_check.sh","start_service.sh","view_report.sh" -Force
Remove-Item -Path "LIVE_CONTROLS_FIX.md","MISSION_UPLOAD_FIX.md","QUICKSTART.md" -Force
Remove-Item -Path "PROJECT_REPORT.html","PROJECT_REPORT.pdf","REPORT_SUMMARY.md" -Force
Remove-Item -Path "SERVICE_COMMANDS.md","tmp_write_check.txt","metadata.json" -Force
```

**Linux/Mac Bash:**
```bash
# Remove folders
rm -rf Backend/ clients/ utils/

# Remove files
rm -f generate_pdf.sh health_check.sh start_service.sh view_report.sh
rm -f LIVE_CONTROLS_FIX.md MISSION_UPLOAD_FIX.md QUICKSTART.md
rm -f PROJECT_REPORT.html PROJECT_REPORT.pdf REPORT_SUMMARY.md
rm -f SERVICE_COMMANDS.md tmp_write_check.txt metadata.json
```

## 📁 Final Folder Structure

```
NRP_ROS_V2/
├── .git/                      # Git repository (preserved)
├── .gitignore                 # Updated
├── .vscode/                   # VSCode settings
├── src/                       # React source code
│   ├── components/
│   ├── context/
│   ├── hooks/
│   ├── tools/
│   ├── types/
│   ├── utils/
│   ├── App.tsx
│   ├── config.ts
│   ├── index.css
│   ├── index.tsx
│   ├── types.ts
│   └── vite-env.d.ts
├── .env                       # Environment variables
├── index.html                 # HTML template
├── index.tsx                  # Entry point
├── package.json               # Dependencies
├── package-lock.json          # Lock file
├── postcss.config.js          # PostCSS config
├── README.md                  # Updated documentation
├── tailwind.config.js         # Tailwind config
├── tsconfig.json              # TypeScript config
├── vite.config.ts             # Vite config
└── vite-env.d.ts              # Type definitions
```

## 🔄 After Cleanup Steps

1. **Install dependencies:**
   ```bash
   npm install
   ```

2. **Run the development server:**
   ```bash
   npm run dev
   ```

3. **Verify the app works** - Open http://localhost:3000

4. **Commit the changes:**
   ```bash
   git add .
   git commit -m "chore: cleanup backend files for frontend-only deployment"
   ```

5. **Push to GitHub:**
   ```bash
   git push origin V3_Oct_29
   ```

## 📝 Notes

- The `.git` folder is preserved - you can still push/pull from GitHub
- All cleanup scripts are safe and check if files exist before deleting
- You can run the cleanup script multiple times safely
- After cleanup, you can delete `CLEANUP_BACKEND.ps1`, `cleanup_backend.sh`, and this guide

## 🆘 Troubleshooting

**If you need to restore files:**
- Use `git checkout <file>` to restore individual files
- Use `git reset --hard` to restore everything (loses uncommitted changes)

**If npm install fails:**
- Clear cache: `npm cache clean --force`
- Delete `node_modules` and `package-lock.json`, then run `npm install` again

**If the dev server won't start:**
- Check if port 3000 is available
- Try: `npm run dev -- --port 3001` to use a different port
