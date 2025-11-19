import React, { useEffect, useState } from 'react';
import { Waypoint } from '../../types';
import { useRover } from '../../context/RoverContext';

export type SprayerMissionLogsProps = {
  waypoints: Waypoint[];
};

type WpStatus = {
  reached?: string; // display string for reach time
  sprayed?: string; // marking status (completed|failed)
};

const SprayerMissionLogs: React.FC<SprayerMissionLogsProps> = ({ waypoints }) => {
  const { onMissionEvent } = useRover();
  const [statusMap, setStatusMap] = useState<Record<number, WpStatus>>({});
  const [missionMode, setMissionMode] = useState<string | null>(null);

  useEffect(() => {
    const handler = (ev: any) => {
      // Normalize wrapper shape
      const payload = ev && ev.type === 'mission_status' && ev.data ? ev.data : ev;
      console.log('Mission Event Received:', payload);

      // Update mission mode if present in payload (mission_mode is part of base payload)
      const mode = payload?.mission_mode ?? payload?.missionMode ?? null;
      if (mode) {
        setMissionMode(String(mode).toLowerCase());
      }

      const waypointId = payload?.waypoint_id ?? payload?.current_waypoint ?? payload?.waypointId;
      if (!waypointId) return;

      setStatusMap((prev) => {
        const copy: Record<number, WpStatus> = { ...prev };
        if (payload.event_type === 'waypoint_reached') {
          copy[waypointId] = {
            ...(copy[waypointId] || {}),
            reached: new Date(payload.timestamp).toLocaleTimeString(),
          };
        }
        if (payload.event_type === 'waypoint_marked') {
          copy[waypointId] = {
            ...(copy[waypointId] || {}),
            sprayed: payload.marking_status ?? payload.markingStatus ?? payload.sprayed ?? 'unknown',
          };
        }

        console.log('Status Map Updated:', copy);
        return copy;
      });
    };

    onMissionEvent(handler);
    return () => onMissionEvent(() => {});
  }, [onMissionEvent]);

  return (
    <div className="bg-[#111827] rounded-lg p-2 h-full flex flex-col">
      <div className="flex items-center justify-between mb-2">
        <h3 className="font-bold text-gray-400 text-left flex-shrink-0">Mission Waypoints</h3>
        <div>
          <span
            role="status"
            aria-label={`mission-mode-${missionMode ?? 'unknown'}`}
            className={`px-2 py-1 rounded text-xs font-semibold cursor-default ${
              missionMode === 'auto'
                ? 'bg-green-600 text-white'
                : missionMode === 'manual'
                ? 'bg-amber-400 text-black'
                : 'bg-gray-600 text-white'
            }`}
          >
            {(missionMode ? String(missionMode).toUpperCase() : 'UNKNOWN')}
          </span>
        </div>
      </div>
      <div className="flex-1 overflow-hidden">
        <div className="h-full overflow-y-auto">
          <table className="w-full text-xs text-left border-collapse">
            <thead className="bg-gray-800 sticky top-0">
              <tr>
                <th className="px-2 py-1">S/N</th>
                <th className="px-2 py-1">Command</th>
                <th className="px-2 py-1">Latitude</th>
                <th className="px-2 py-1">Longitude</th>
                <th className="px-2 py-1">WP_REACH</th>
                <th className="px-2 py-1">WP_MARK</th>
              </tr>
            </thead>
            <tbody>
              {waypoints.length === 0 ? (
                <tr>
                  <td colSpan={6} className="text-center py-4 text-gray-500">No waypoints loaded</td>
                </tr>
              ) : (
                waypoints.map((waypoint, idx) => (
                  <tr key={waypoint.id || idx} className={idx % 2 === 0 ? 'bg-gray-900' : 'bg-gray-800'}>
                    <td className="px-2 py-1">{idx + 1}</td>
                    <td className="px-2 py-1">{waypoint.command || `WP${idx + 1}`}</td>
                    <td className="px-2 py-1">{waypoint.lat.toFixed(7)}</td>
                    <td className="px-2 py-1">{waypoint.lng.toFixed(7)}</td>
                    <td className="px-2 py-1">{statusMap[waypoint.id]?.reached ?? '-'}</td>
                    <td className="px-2 py-1">{statusMap[waypoint.id]?.sprayed ?? '-'}</td>
                  </tr>
                ))
              )}
            </tbody>
          </table>
        </div>
      </div>
    </div>
  );
};

export default SprayerMissionLogs;
