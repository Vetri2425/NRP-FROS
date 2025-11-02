# Keyboard Shortcuts Reference

## Mission Planning Shortcuts

### File Operations
| Shortcut | Action | Description |
|----------|--------|-------------|
| `Ctrl+U` | Upload Mission File | Opens file picker to upload a mission file (.waypoint, .waypoints, .csv, .dxf, .json, .kml) |
| `Ctrl+E` | Export Mission | Opens export dialog to save mission in various formats (QGC, JSON, KML, CSV) |
| `Ctrl+S` | Save/Export Mission | Same as Ctrl+E - saves current mission to file |

### Rover Communication
| Shortcut | Action | Description |
|----------|--------|-------------|
| `Ctrl+D` | Download from Rover | Reads current mission waypoints from the connected rover |

### Mission Editing
| Shortcut | Action | Description |
|----------|--------|-------------|
| `Ctrl+Z` | Undo | Reverts to previous mission state (up to 20 snapshots) |
| `Ctrl+Shift+Z` | Redo | Re-applies undone changes |
| `Ctrl+Y` | Redo (Alternative) | Same as Ctrl+Shift+Z - re-applies undone changes |
| `Ctrl+O` | Clear Mission | Clears all waypoints from current mission (with confirmation) |

### Help & Information
| Shortcut | Action | Description |
|----------|--------|-------------|
| `Shift+?` | Show Keyboard Shortcuts | Opens help dialog displaying all available keyboard shortcuts |

## Shortcut Behavior

### Smart Input Detection
- Shortcuts are **disabled** when typing in:
  - Text input fields
  - Text areas
  - Contenteditable elements
- This prevents accidental triggering while editing waypoint data

### Disabled States
Some shortcuts are contextually disabled:
- **Ctrl+E, Ctrl+S**: Disabled when no waypoints exist
- **Ctrl+O**: Disabled when mission is already empty
- **Ctrl+D**: Disabled when rover is not connected or already reading
- All shortcuts requiring mission data are disabled in empty state

### Browser Compatibility
- **Ctrl** key works on Windows/Linux
- **Cmd** (⌘) key automatically maps to Ctrl on macOS
- All shortcuts prevent default browser behavior to avoid conflicts

## Usage Tips

### Quick Workflow
1. **Ctrl+U** - Upload mission file
2. Edit waypoints on map
3. **Ctrl+Z/Y** - Undo/Redo as needed
4. **Ctrl+D** - Download existing rover mission
5. **Ctrl+E** - Export final mission
6. Upload to rover via "Write to Rover" button

### Confirmation Dialogs
- **Clear Mission (Ctrl+O)**: Requires confirmation to prevent accidental data loss
- **Upload to Rover**: Shows preview dialog with mission statistics (can be disabled)
- **Export Mission**: Shows format selection dialog

### Mission History
- Undo/Redo tracks up to **20 snapshots**
- Each waypoint change creates a new snapshot
- History is cleared when mission is cleared
- Deep cloning ensures snapshot integrity

## Accessibility

### Keyboard-Only Navigation
- All shortcuts work without mouse interaction
- Tab navigation supported throughout dialogs
- Escape key closes modal dialogs
- Focus management for screen readers

### Visual Indicators
- Connection status displayed in header
- Latency information on hover
- Progress bars for uploads
- Toast notifications for all actions

## Future Shortcuts (Planned)

The following shortcuts may be added in future updates:
- `Ctrl+N` - New mission
- `Ctrl+F` - Find waypoint
- `Ctrl+G` - Go to waypoint
- `Ctrl+P` - Preview mission path
- `Ctrl+Shift+S` - Quick save
- `Ctrl+Shift+E` - Export settings

## Customization

Keyboard shortcuts are defined in:
```
src/components/plan/PlanControls.tsx
```

To modify shortcuts, edit the `shortcuts` array in the `useKeyboardShortcuts` hook.

## Help & Support

- Press `Shift+?` to view shortcuts in-app
- Check browser console for keyboard event debugging
- Ensure rover connection for rover-dependent shortcuts

---

**Note**: This reference is for the Plan Tab. Other tabs may have different shortcuts in future updates.

**Version**: 1.0.0  
**Last Updated**: November 1, 2025
