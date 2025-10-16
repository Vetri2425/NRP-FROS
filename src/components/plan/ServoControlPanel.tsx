import React, { useEffect, useMemo, useState } from 'react';
import { WrenchIcon } from '../icons/WrenchIcon';
import type {
  ServoConfig,
  ServoMode,
  MarkAtWaypointConfig,
  ContinuousLineConfig,
  IntervalSprayConfig,
} from '../../types/servo';

type Props = {
  // Table selection helpers (keep your existing behavior)
  selectedWaypointIds: number[];
  areAllSelected: boolean;
  onSelectAll: () => void;
  onDeleteSelected: () => void;

  // NEW: notify parent with the latest config
  onConfigChange: (cfg: ServoConfig, action?: 'upload' | 'select') => void;

  // For showing WP range hints (optional)
  totalWaypoints?: number;
};

const ServoControlPanel: React.FC<Props> = ({
  selectedWaypointIds,
  areAllSelected,
  onSelectAll,
  onDeleteSelected,
  onConfigChange,
  totalWaypoints = 0,
}) => {
  const [isOpen, setIsOpen] = useState(false);
  const [mode, setMode] = useState<ServoMode>('MARK_AT_WAYPOINT');

  // common fields
  const [servoNumber, setServoNumber] = useState(10);
  const [pwmOn, setPwmOn] = useState(650);
  const [pwmOff, setPwmOff] = useState(1000);

  // mode-specific
  const [sprayDuration, setSprayDuration] = useState(0.5);

  // interval-specific (UI shows cm but stored/used in meters in ServoConfig)
  const [intervalCm, setIntervalCm] = useState(30);

  const [distanceOnMeters, setDistanceOnMeters] = useState(5);
  const [distanceOffMeters, setDistanceOffMeters] = useState(5);
  const [intervalStartWp, setIntervalStartWp] = useState(1);
  const [intervalEndWp, setIntervalEndWp] = useState(2);

  const wpHint = useMemo(() => {
    if (!totalWaypoints) return 'No mission loaded';
    return `Waypoints: 1 → ${totalWaypoints}`;
  }, [totalWaypoints]);

  const clamp = (n: number, min: number, max: number) =>
    Math.max(min, Math.min(max, Number.isFinite(n) ? n : min));

  const buildMarkConfig = (
    sv: number,
    onValue: number,
    offValue: number
  ): (MarkAtWaypointConfig & { fromWp: number; toWp: number }) | null => {
    if (!selectedWaypointIds.length) {
      alert('Select waypoints on the table or map to define a range.');
      return null;
    }
    const total = Math.max(1, totalWaypoints || 1);
    const minSel = Math.min(...selectedWaypointIds);
    const maxSel = Math.max(...selectedWaypointIds);
    const from = clamp(minSel, 1, total);
    const to = clamp(maxSel, from, total);
    const duration = clamp(sprayDuration, 0.1, 60);

    return {
      mode: 'MARK_AT_WAYPOINT',
      servoNumber: sv,
      pwmOn: onValue,
      pwmOff: offValue,
      fromWp: from,
      toWp: to,
      sprayDuration: duration,
      selectedIds: [...selectedWaypointIds],
    } as MarkAtWaypointConfig & { fromWp: number; toWp: number };
  };

  const buildContinuousConfig = (
    sv: number,
    onValue: number,
    offValue: number
  ): (ContinuousLineConfig & { startPoint: number; endPoint: number }) | null => {
    if (!selectedWaypointIds.length) {
      alert('Select a range of waypoints on the table or map to define the line.');
      return null;
    }
    const total = Math.max(1, totalWaypoints || 1);
    const minSel = Math.min(...selectedWaypointIds);
    const maxSel = Math.max(...selectedWaypointIds);
    const start = clamp(minSel, 1, total);
    const end = clamp(maxSel, start, total);

    if (start === end) {
      alert('Select at least two waypoints to define the line.');
      return null;
    }

    return {
      mode: 'CONTINUOUS_LINE',
      servoNumber: sv,
      pwmOn: onValue,
      pwmOff: offValue,
      startPoint: start,
      endPoint: end,
      selectedIds: [...selectedWaypointIds],
    } as ContinuousLineConfig & { startPoint: number; endPoint: number };
  };

  const buildIntervalConfig = (
    sv: number,
    onValue: number,
    offValue: number
  ): (IntervalSprayConfig & { startWp: number; endWp: number }) | null => {
    if (!totalWaypoints || totalWaypoints < 2) {
      alert('Load a mission with at least two waypoints before configuring interval spray.');
      return null;
    }
    const total = Math.max(1, totalWaypoints || 1);
    const startRaw = clamp(intervalStartWp, 1, total);
    const endRaw = clamp(intervalEndWp, 1, total);
    const start = Math.min(startRaw, endRaw);
    const end = Math.max(startRaw, endRaw);

    if (start === end) {
      alert('Start and end waypoints must be different for interval spray.');
      return null;
    }

  // intervalCm UI is centimeters; convert to meters for the config
  const onMeters = clamp(distanceOnMeters, 0.1, 100000);
  const offMeters = clamp(distanceOffMeters, 0.1, 100000);

    return {
      mode: 'INTERVAL_SPRAY',
      servoNumber: sv,
      pwmOn: onValue,
      pwmOff: offValue,
      distanceOnMeters: onMeters,
      distanceOffMeters: offMeters,
      // additional convenience field not part of the stored union but useful for UI/confirmation
      // (we still return a typed IntervalSprayConfig so callers can access startWp/endWp/etc.)
      startWp: start,
      endWp: end,
      selectedIds: [...selectedWaypointIds],
    };
  };

  useEffect(() => {
    if (mode !== 'INTERVAL_SPRAY') return;
    if (!selectedWaypointIds.length) return;
    const total = Math.max(1, totalWaypoints || 1);
    let start = clamp(Math.min(...selectedWaypointIds), 1, total);
    let end = clamp(Math.max(...selectedWaypointIds), start, total);
    if (start === end && end < total) {
      end = clamp(end + 1, start, total);
    }
    setIntervalStartWp(start);
    setIntervalEndWp(end);
  }, [selectedWaypointIds, mode, totalWaypoints]);

  const validateAndSave = () => {
    // basic safety clamps
    const sv = clamp(servoNumber, 1, 99);
    const on = clamp(pwmOn, 100, 2500);
    const off = clamp(pwmOff, 100, 2500);

    let cfg: ServoConfig | null = null;

    if (mode === 'MARK_AT_WAYPOINT') {
      cfg = buildMarkConfig(sv, on, off);
    } else if (mode === 'CONTINUOUS_LINE') {
      cfg = buildContinuousConfig(sv, on, off);
    } else {
      cfg = buildIntervalConfig(sv, on, off);
    }

    if (!cfg) return;

    onConfigChange(cfg);
    setIsOpen(false);
  };

  return (
    <div className="bg-[#111827] h-full rounded-md p-3 flex flex-col gap-4 text-sm text-gray-300">
      <div className="flex items-center gap-3 border-b border-gray-700 pb-2">
        <WrenchIcon className="w-5 h-5 text-sky-400" />
        <h2 className="text-md font-bold text-white">Servo Control</h2>
      </div>

      {/* Compact row: Mode + Configure button */}
      <div className="flex flex-col gap-2">
        <label className="text-xs text-gray-400">Mode</label>
        <select
          value={mode}
          onChange={(e) => setMode(e.target.value as ServoMode)}
          className="bg-[#1F2937] border border-gray-600 rounded-md p-2 focus:outline-none focus:ring-1 focus:ring-sky-500"
        >
          <option value="MARK_AT_WAYPOINT">Mark at Waypoint</option>
          <option value="CONTINUOUS_LINE">Continuous Line</option>
          <option value="INTERVAL_SPRAY">Interval Spray</option>
        </select>

        <button
          onClick={() => setIsOpen(true)}
          className="bg-sky-600 hover:bg-sky-700 py-2 rounded-md font-semibold"
        >
          Configure Parameters (Popup)
        </button>

        <div className="text-xs text-gray-400">{wpHint}</div>

        {/* NEW: Upload to Table Button */}
        <div className="pt-3">
          <button
            onClick={() => {
              const sv = clamp(servoNumber, 1, 99);
              const onValue = clamp(pwmOn, 100, 2500);
              const offValue = clamp(pwmOff, 100, 2500);

              let cfg: ServoConfig | null = null;
              const summaryLines: string[] = [];

              if (mode === 'MARK_AT_WAYPOINT') {
                const built = buildMarkConfig(sv, onValue, offValue);
                if (!built) return;
                cfg = built;
                summaryLines.push("Start WP: " + built.fromWp, "End WP: " + built.toWp, "Duration: " + built.sprayDuration + 's');
              } else if (mode === 'CONTINUOUS_LINE') {
                const built = buildContinuousConfig(sv, onValue, offValue);
                if (!built) return;
                cfg = built;
                summaryLines.push("Start WP: " + built.startPoint, "End WP: " + built.endPoint);
              } else {
                const built = buildIntervalConfig(sv, onValue, offValue);
                if (!built) return;
                cfg = built;
                summaryLines.push(
                  "Start WP: " + built.startWp,
                  "End WP: " + built.endWp,
                  "Distance ON: " + built.distanceOnMeters + ' m',
                  "Distance OFF: " + built.distanceOffMeters + ' m'
                );
              }

              const confirmed = window.confirm(
                [
                  "Mode: " + mode.replace(/_/g, ' '),
                  "Servo: " + sv,
                  "PWM ON: " + onValue + ', OFF: ' + offValue,
                  ...summaryLines,
                  'Apply to Mission Table?',
                ].join('\n')
              );

              if (!confirmed) {
                alert('??O Upload canceled.');
                return;
              }

              onConfigChange(cfg, 'upload');
            }}
            className="w-full bg-green-600 hover:bg-green-700 py-2 rounded-md font-semibold"
          >
            🚀 Upload to Table
          </button>
        </div>
      </div>

      {/* Bottom quick actions */}
      <div className="mt-auto grid grid-cols-3 gap-2 pt-4">
        <button
          onClick={onSelectAll}
          className="bg-sky-800 hover:bg-sky-700 py-2 rounded-md font-semibold"
        >
          {areAllSelected ? 'Deselect All' : 'Select All'}
        </button>
        <div className="flex items-center justify-center text-xs text-gray-300">
          Selected: {selectedWaypointIds.length}
        </div>
        <button
          onClick={onDeleteSelected}
          className="bg-red-600 hover:bg-red-700 py-2 rounded-md font-semibold disabled:opacity-50 disabled:cursor-not-allowed"
          disabled={selectedWaypointIds.length === 0}
        >
          Delete
        </button>
      </div>

      {/* POPUP / MODAL */}
      {isOpen && (
        <div className="fixed inset-0 z-[2000] flex items-center justify-center">
          {/* Backdrop */}
          <div
            className="absolute inset-0 bg-black/60"
            onClick={() => setIsOpen(false)}
          />

          {/* Modal card */}
          <div className="relative z-10 w-[420px] max-w-[95vw] bg-[#0F172A] border border-gray-700 rounded-xl p-4 shadow-xl">
            <div className="text-white font-semibold mb-2 flex items-center justify-between">
              <span>Servo Parameters</span>
              <button
                className="text-gray-400 hover:text-white"
                onClick={() => setIsOpen(false)}
                aria-label="Close"
              >
                ✕
              </button>
            </div>

            {/* COMMON FIELDS */}
            <div className="grid grid-cols-2 gap-3">
              <div>
                <label className="block text-xs text-gray-400 mb-1">Servo No</label>
                <input
                  type="number"
                  value={servoNumber}
                  onChange={(e) => setServoNumber(parseInt(e.target.value || '1', 10))}
                  className="w-full bg-[#1F2937] border border-gray-600 rounded-md p-2"
                  min={1}
                  max={99}
                />
              </div>
              <div>
                <label className="block text-xs text-gray-400 mb-1">Start PWM</label>
                <input
                  type="number"
                  value={pwmOn}
                  onChange={(e) => setPwmOn(parseInt(e.target.value || '650', 10))}
                  className="w-full bg-[#1F2937] border border-gray-600 rounded-md p-2"
                  min={100}
                  max={2500}
                />
              </div>
              <div>
                <label className="block text-xs text-gray-400 mb-1">Stop PWM</label>
                <input
                  type="number"
                  value={pwmOff}
                  onChange={(e) => setPwmOff(parseInt(e.target.value || '1000', 10))}
                  className="w-full bg-[#1F2937] border border-gray-600 rounded-md p-2"
                  min={100}
                  max={2500}
                />
              </div>
            </div>

            {/* MODE-SPECIFIC FIELDS */}
            {mode === 'MARK_AT_WAYPOINT' && (
              <div className="grid grid-cols-2 gap-3 mt-3">
                <div className="col-span-2 text-xs text-gray-400">
                  Select a contiguous window of waypoints in the table or on the map. The range is derived from your selection.
                </div>
                <div>
                  <label className="block text-xs text-gray-400 mb-1">From (selected)</label>
                  <div className="w-full bg-[#1F2937] border border-gray-600 rounded-md p-2 text-gray-200">
                    {selectedWaypointIds.length ? Math.min(...selectedWaypointIds) : 'N/A'}
                  </div>
                </div>
                <div>
                  <label className="block text-xs text-gray-400 mb-1">To (selected)</label>
                  <div className="w-full bg-[#1F2937] border border-gray-600 rounded-md p-2 text-gray-200">
                    {selectedWaypointIds.length ? Math.max(...selectedWaypointIds) : 'N/A'}
                  </div>
                </div>
                <div className="col-span-2">
                  <label className="block text-xs text-gray-400 mb-1">Spray Duration (s)</label>
                  <input
                    type="number"
                    step={0.1}
                    value={sprayDuration}
                    onChange={(e) => setSprayDuration(parseFloat(e.target.value || '0.5'))}
                    className="w-full bg-[#1F2937] border border-gray-600 rounded-md p-2"
                  />
                </div>
              </div>
            )}

            {mode === 'CONTINUOUS_LINE' && (
              <div className="grid grid-cols-2 gap-3 mt-3">
                <div className="col-span-2 text-xs text-gray-400">
                  Select start and end by highlighting a window of waypoints. The line will span the min → max selected IDs.
                </div>
                <div>
                  <label className="block text-xs text-gray-400 mb-1">Start (selected)</label>
                  <div className="w-full bg-[#1F2937] border border-gray-600 rounded-md p-2 text-gray-200">
                    {selectedWaypointIds.length ? Math.min(...selectedWaypointIds) : 'N/A'}
                  </div>
                </div>
                <div>
                  <label className="block text-xs text-gray-400 mb-1">End (selected)</label>
                  <div className="w-full bg-[#1F2937] border border-gray-600 rounded-md p-2 text-gray-200">
                    {selectedWaypointIds.length ? Math.max(...selectedWaypointIds) : 'N/A'}
                  </div>
                </div>
              </div>
            )}

            {mode === 'INTERVAL_SPRAY' && (
              <div className="grid grid-cols-2 gap-3 mt-3">
                <div className="col-span-2">
                  <label className="block text-xs text-gray-400 mb-1">Interval Distance (cm)</label>
                  <input
                    type="number"
                    value={intervalCm}
                    onChange={(e) => setIntervalCm(parseInt(e.target.value || '30', 10))}
                    className="w-full bg-[#1F2937] border border-gray-600 rounded-md p-2"
                    min={1}
                  />
                </div>
                <div>
                  <label className="block text-xs text-gray-400 mb-1">Start (selected)</label>
                  <div className="w-full bg-[#1F2937] border border-gray-600 rounded-md p-2 text-gray-200">
                    {selectedWaypointIds.length ? Math.min(...selectedWaypointIds) : 'N/A'}
                  </div>
                </div>
                <div>
                  <label className="block text-xs text-gray-400 mb-1">End (selected)</label>
                  <div className="w-full bg-[#1F2937] border border-gray-600 rounded-md p-2 text-gray-200">
                    {selectedWaypointIds.length ? Math.max(...selectedWaypointIds) : 'N/A'}
                  </div>
                </div>
              </div>
            )}

            {/* Footer buttons */}
            <div className="mt-4 flex flex-wrap justify-end gap-2">
              <button
                onClick={() => {
                  // Start a waypoint selection session in planning view
                  const cfg: any = {
                    mode,
                    servoNumber,
                    pwmOn,
                    pwmOff,
                  };
                  if (mode === 'MARK_AT_WAYPOINT') cfg.sprayDuration = sprayDuration;
                  if (mode === 'INTERVAL_SPRAY') cfg.intervalCm = intervalCm;
                  onConfigChange(cfg, 'select');
                  setIsOpen(false);
                }}
                className="px-3 py-2 rounded-md bg-sky-700 hover:bg-sky-600 text-white"
              >
                Select Waypoints
              </button>
              <button
                onClick={() => setIsOpen(false)}
                className="px-3 py-2 rounded-md bg-gray-700 hover:bg-gray-600 text-white"
              >
                Cancel
              </button>
              <button
                onClick={validateAndSave}
                className="px-3 py-2 rounded-md bg-green-600 hover:bg-green-700 text-white font-semibold"
              >
                Save Config
              </button>
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

export default ServoControlPanel;
