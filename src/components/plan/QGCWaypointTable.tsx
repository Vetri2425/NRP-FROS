import React, { useState, useRef } from 'react';
import { TrashIcon } from '../icons/TrashIcon';
import { Waypoint } from '../../types';
import { getNavigationCommands, getDoCommands, getConditionCommands, getCommandDefinition } from '../../utils/mavlink_commands';
import { calculateDistance } from '../../utils/geo';

// Get all available commands organized by category
const NAV_COMMANDS = getNavigationCommands();
const DO_COMMANDS = getDoCommands();
const CONDITION_COMMANDS = getConditionCommands();

/**
 * Get parameter label based on command type
 */
function getParamLabel(command: string, paramNum: 1 | 2 | 3 | 4): string {
  const def = getCommandDefinition(command);
  if (!def) return `Param${paramNum}`;
  
  const paramKey = `param${paramNum}` as keyof typeof def.params;
  return def.params[paramKey] || `P${paramNum}`;
}

type QGCWaypointTableProps = {
  waypoints: Waypoint[];
  onDelete: (id: number) => void;
  onUpdate: (id: number, newValues: Partial<Omit<Waypoint, 'id'>>) => void;
  activeWaypointIndex: number | null;
  selectedWaypointIds: number[];
  onWaypointSelectionChange: (id: number, isSelected: boolean) => void;
  onSelectAll?: (selectAll: boolean) => void;
  onDeleteSelected?: () => void;
  onAddWaypoint?: (insertAfterId?: number | null) => void;
  // Anchor selection (separate from checkbox selection) - file-explorer like
  anchorSelectedIds?: number[];
  anchorLastClickedId?: number | null;
  onRowAnchorClick?: (id: number, ctrlKey: boolean) => void;
  // Distance edit handler: distance from previous waypoint to this waypoint (meters)
  onDistanceChange?: (id: number, distanceMeters: number) => void;
};

const QGCWaypointTable: React.FC<QGCWaypointTableProps> = ({ 
    waypoints, 
    onDelete, 
    onUpdate, 
    activeWaypointIndex,
    selectedWaypointIds,
    onWaypointSelectionChange,
    onSelectAll, onDeleteSelected, onAddWaypoint,
    anchorSelectedIds, anchorLastClickedId, onRowAnchorClick, onDistanceChange
}) => {
  const [isDragging, setIsDragging] = useState(false);
  const dragStartId = useRef<number | null>(null);
  const headers = ['#', 'Sel', 'Curr', 'Frame', 'Command', 'Param1', 'Param2', 'Param3', 'Param4', 'Dist (m)', 'X (Lat)', 'Y (Lon)', 'Z (Alt)', 'Auto', 'Actions'];

  const handleSetCurrent = (id: number) => {
    waypoints.forEach(wp => {
      if (wp.id === id) {
        onUpdate(wp.id, { current: 1 });
      } else if (wp.current) {
        onUpdate(wp.id, { current: 0 });
      }
    });
  };

  const handleAutoToggle = (id: number, checked: boolean) => {
    onUpdate(id, { autocontinue: checked ? 1 : 0 });
  };

  const handleValueChange = ( id: number, field: keyof Omit<Waypoint, 'id' | 'command' | 'action'>, value: string ) => {
    const numericValue = value === '' ? 0 : parseFloat(value);
    if (!isNaN(numericValue)) {
      onUpdate(id, { [field]: numericValue });
    }
  };

  const handleCommandChange = (e: React.ChangeEvent<HTMLSelectElement>, id: number) => {
    onUpdate(id, { command: e.target.value });
  };

  const handleMouseDown = (id: number, e?: React.MouseEvent) => {
    setIsDragging(true);
    dragStartId.current = id;
    onWaypointSelectionChange(id, !selectedWaypointIds.includes(id));
    // Anchor selection (separate): single click selects anchor; ctrl toggles multi
    if (onRowAnchorClick) onRowAnchorClick(id, !!(e && e.ctrlKey));
  };

  const handleMouseEnter = (id: number) => {
    if (isDragging && dragStartId.current !== null) {
      const start = Math.min(dragStartId.current, id);
      const end = Math.max(dragStartId.current, id);
      for (let i = start; i <= end; i++) {
        if (!selectedWaypointIds.includes(i)) {
          onWaypointSelectionChange(i, true);
        }
      }
    }
  };

  const handleMouseUp = () => {
    setIsDragging(false);
    dragStartId.current = null;
  };

  return (
    <div className="bg-[#111827] h-full rounded-md p-2 flex flex-col text-xs text-gray-300 overflow-hidden" onMouseUp={handleMouseUp}>
      <div className="flex items-start justify-between mb-2">
          <div className="flex items-center gap-2">
          <button
            onClick={() => onAddWaypoint && onAddWaypoint(anchorLastClickedId ?? null)}
            className="text-xs bg-green-600 hover:bg-green-500 text-white px-2 py-1 rounded-md font-semibold"
            aria-label="Add waypoint"
          >
            + Add WP
          </button>
          <div className="text-xs text-gray-400">QGC WPL 110</div>
        </div>
        <div className="flex items-center gap-2">
          <label className="flex items-center gap-1 text-xs text-gray-300">
            <input
              type="checkbox"
              className="form-checkbox"
              checked={waypoints.length > 0 && selectedWaypointIds.length === waypoints.length}
              onChange={(e) => onSelectAll && onSelectAll(e.target.checked)}
              aria-label="Select all waypoints"
            />
            Select All
          </label>
          <button
            onClick={() => onDeleteSelected && onDeleteSelected()}
            className="text-xs bg-red-600 hover:bg-red-500 text-white px-2 py-1 rounded-md font-semibold"
            disabled={selectedWaypointIds.length === 0}
            aria-disabled={selectedWaypointIds.length === 0}
            aria-label="Delete selected waypoints"
          >
            Delete ({selectedWaypointIds.length})
          </button>
        </div>
      </div>
      <h2 className="text-sm font-bold mb-1.5">Mission Plan (QGC WPL 110)</h2>
      <div className="flex-1 overflow-y-auto custom-scrollbar">
        <table className="w-full text-left">
          <thead className="sticky top-0 bg-[#111827]">
            <tr>
              {headers.map(header => (
                <th key={header} scope="col" className="px-1.5 py-1 text-xs text-gray-400 uppercase font-medium whitespace-nowrap">
                  {header}
                </th>
              ))}
            </tr>
          </thead>
          <tbody className="divide-y divide-gray-700">
            {waypoints.length > 0 ? (
              waypoints.map((wp, index) => {
                const isActive = wp.id === activeWaypointIndex;
          return (
            <tr key={wp.id} className={`hover:bg-gray-800 ${isActive ? 'bg-green-800/50' : ''} ${selectedWaypointIds.includes(wp.id) ? 'bg-sky-800/40' : ''} ${(anchorSelectedIds || []).includes(wp.id) ? 'ring-2 ring-yellow-400/40' : ''} ${anchorLastClickedId === wp.id ? 'ring-yellow-400' : ''}`}>
                    <td className="px-1.5 py-0.5 text-xs">{wp.id}</td>
                    <td
                      className="px-1.5 py-0.5"
                      onMouseDown={(e) => handleMouseDown(wp.id, e)}
                      onMouseEnter={() => handleMouseEnter(wp.id)}
                    >
                      <input
                        type="checkbox"
                        className="form-checkbox bg-gray-800 border-gray-600 text-sky-500 focus:ring-sky-500"
                        checked={selectedWaypointIds.includes(wp.id)}
                        readOnly
                      />
                    </td>
                    <td className="px-1.5 py-0.5 text-center">
                      <input type="radio" name="currentItem" checked={(wp.current ?? (index === 0 ? 1 : 0)) === 1}
                        onChange={() => handleSetCurrent(wp.id)} />
                    </td>
                    <td className="px-1.5 py-0.5"><input type="number" value={wp.frame} onChange={(e) => handleValueChange(wp.id, 'frame', e.target.value)} className="bg-gray-700 w-14 p-0.5 rounded text-xs" /></td>
                    <td className="px-1.5 py-0.5">
                      <select
                        value={wp.command}
                        onChange={(e) => handleCommandChange(e, wp.id)}
                        className="bg-gray-700 border-gray-600 rounded p-0.5 text-xs focus:outline-none focus:ring-1 focus:ring-green-500 w-full"
                      >
                        {/* Navigation Commands */}
                        <optgroup label="━━ Navigation ━━">
                          {NAV_COMMANDS.map(cmd => (
                            <option key={cmd.id} value={cmd.name}>{cmd.name}</option>
                          ))}
                        </optgroup>
                        {/* DO Commands */}
                        <optgroup label="━━ DO Commands ━━">
                          {DO_COMMANDS.map(cmd => (
                            <option key={cmd.id} value={cmd.name}>{cmd.name}</option>
                          ))}
                        </optgroup>
                        {/* Condition Commands */}
                        <optgroup label="━━ Conditions ━━">
                          {CONDITION_COMMANDS.map(cmd => (
                            <option key={cmd.id} value={cmd.name}>{cmd.name}</option>
                          ))}
                        </optgroup>
                      </select>
                    </td>
                    <td className="px-1.5 py-0.5">
                      <input 
                        type="number" 
                        value={wp.param1 || 0} 
                        onChange={(e) => handleValueChange(wp.id, 'param1', e.target.value)} 
                        className="bg-gray-700 w-16 p-0.5 rounded text-xs" 
                        title={getParamLabel(wp.command, 1)}
                        placeholder={getParamLabel(wp.command, 1)}
                      />
                    </td>
                    <td className="px-1.5 py-0.5">
                      <input 
                        type="number" 
                        value={wp.param2 || 0} 
                        onChange={(e) => handleValueChange(wp.id, 'param2', e.target.value)} 
                        className="bg-gray-700 w-16 p-0.5 rounded text-xs" 
                        title={getParamLabel(wp.command, 2)}
                        placeholder={getParamLabel(wp.command, 2)}
                      />
                    </td>
                    <td className="px-1.5 py-0.5">
                      <input 
                        type="number" 
                        value={wp.param3 || 0} 
                        onChange={(e) => handleValueChange(wp.id, 'param3', e.target.value)} 
                        className="bg-gray-700 w-16 p-0.5 rounded text-xs" 
                        title={getParamLabel(wp.command, 3)}
                        placeholder={getParamLabel(wp.command, 3)}
                      />
                    </td>
                    <td className="px-1.5 py-0.5">
                      <input 
                        type="number" 
                        value={wp.param4 || 0} 
                        onChange={(e) => handleValueChange(wp.id, 'param4', e.target.value)} 
                        className="bg-gray-700 w-16 p-0.5 rounded text-xs" 
                        title={getParamLabel(wp.command, 4)}
                        placeholder={getParamLabel(wp.command, 4)}
                      />
                    </td>
                    <td className="px-1.5 py-0.5">
                      {/* Distance from previous waypoint (meters). Disabled for first waypoint. */}
                      {index === 0 ? (
                        <input type="number" value={0} disabled className="bg-gray-800 w-20 p-0.5 rounded text-xs text-gray-400" />
                      ) : (
                        (() => {
                          const prev = waypoints[index - 1];
                          const distMeters = (typeof prev?.lat === 'number' && typeof wp.lat === 'number')
                            ? calculateDistance({ lat: prev.lat, lng: prev.lng }, { lat: wp.lat, lng: wp.lng })
                            : 0;
                          const display = Math.round(distMeters * 10) / 10; // one decimal
                          return (
                            <input
                              type="number"
                              step="0.1"
                              min={0}
                              max={999}
                              value={display}
                              onChange={(e) => {
                                const raw = parseFloat(e.target.value || '0');
                                let clamped = Number.isFinite(raw) ? raw : 0;
                                if (clamped < 0) clamped = 0;
                                if (clamped > 999.9) clamped = 999.9;
                                // Ensure one decimal precision
                                clamped = Math.round(clamped * 10) / 10;
                                if (onDistanceChange) onDistanceChange(wp.id, clamped);
                              }}
                              className="bg-gray-700 w-20 p-0.5 rounded text-xs font-mono"
                            />
                          );
                        })()
                      )}
                    </td>
                    <td className="px-1.5 py-0.5 font-mono"><input type="number" step="0.000001" value={wp.lat} onChange={(e) => handleValueChange(wp.id, 'lat', e.target.value)} className="bg-gray-700 w-24 p-0.5 rounded text-xs" /></td>
                    <td className="px-1.5 py-0.5 font-mono"><input type="number" step="0.000001" value={wp.lng} onChange={(e) => handleValueChange(wp.id, 'lng', e.target.value)} className="bg-gray-700 w-24 p-0.5 rounded text-xs" /></td>
                    <td className="px-1.5 py-0.5"><input type="number" value={wp.alt} onChange={(e) => handleValueChange(wp.id, 'alt', e.target.value)} className="bg-gray-700 w-16 p-0.5 rounded text-xs" /></td>
                    <td className="px-1.5 py-0.5 text-center">
                      <input type="checkbox" checked={(wp.autocontinue ?? 1) === 1} onChange={(e) => handleAutoToggle(wp.id, e.target.checked)} />
                    </td>
                    <td className="px-1.5 py-0.5">
                      <button onClick={() => onDelete(wp.id)} className="text-red-500 hover:text-red-400 p-0.5">
                        <TrashIcon className="w-4 h-4" />
                      </button>
                    </td>
                  </tr>
                );
              })
            ) : (
                <tr>
                    <td colSpan={headers.length} className="text-center py-4 text-gray-500 text-xs">
                        Upload a mission file or use the drawing tools to begin.
                    </td>
                </tr>
            )}
          </tbody>
        </table>
      </div>
    </div>
  );
};

export default QGCWaypointTable;
