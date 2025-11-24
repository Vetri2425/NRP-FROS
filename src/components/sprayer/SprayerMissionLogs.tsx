import React, { useEffect, useState } from 'react';
import { Waypoint } from '../../types';
import { useRover } from '../../context/RoverContext';
import { BACKEND_URL } from '../../config';

export type SprayerMissionLogsProps = {
  waypoints: Waypoint[];
};

type WpStatus = {
  reached?: boolean; // whether waypoint was reached
  marked?: boolean; // whether waypoint was marked/sprayed
  status?: 'completed' | 'loading' | 'skipped' | 'reached' | 'marked'; // combined status
  timestamp?: string; // actual timestamp when marked
  pile?: string | number; // pile identifier
  rowNo?: string | number; // row number
  remark?: string; // remarks/notes
};

const SprayerMissionLogs: React.FC<SprayerMissionLogsProps> = ({ waypoints }) => {
  const { onMissionEvent } = useRover();
  const [statusMap, setStatusMap] = useState<Record<number, WpStatus>>({});
  const [missionMode, setMissionMode] = useState<string | null>(null);

  // Fetch initial mission mode from backend on mount
  useEffect(() => {
    let mounted = true;
    (async () => {
      try {
        const res = await fetch(`${BACKEND_URL.replace(/\/$/, '')}/api/mission/mode`);
        if (!mounted) return;
        if (!res.ok) return;
        const body = await res.json().catch(() => null);
        if (body && (body.mode === 'auto' || body.mode === 'manual')) {
          setMissionMode(body.mode);
        } else if (typeof body === 'string' && (body === 'auto' || body === 'manual')) {
          setMissionMode(body as 'auto' | 'manual');
        }
      } catch (err) {
        console.debug('Could not fetch current mission mode:', err);
      }
    })();
    return () => { mounted = false; };
  }, []);

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
          console.log('Raw timestamp from backend:', payload.timestamp, 'Type:', typeof payload.timestamp);
          copy[waypointId] = {
            ...(copy[waypointId] || {}),
            reached: true,
            status: 'reached',
            pile: payload.pile ?? copy[waypointId]?.pile,
            rowNo: payload.rowNo ?? payload.row_no ?? copy[waypointId]?.rowNo,
          };
        }
        if (payload.event_type === 'waypoint_marked') {
          const timestamp = payload.timestamp;
          // Extract time directly from ISO string without Date conversion to avoid timezone issues
          // Backend format: "2025-11-20T16:36:49.297846"
          const displayTime = timestamp && typeof timestamp === 'string'
            ? timestamp.split('T')[1]?.split('.')[0] || '-'
            : '-';
          
          const markingStatus = payload.marking_status ?? payload.markingStatus ?? payload.sprayed ?? 'completed';
          copy[waypointId] = {
            ...(copy[waypointId] || {}),
            marked: true,
            status: markingStatus === 'completed' ? 'completed' : markingStatus === 'skipped' ? 'skipped' : 'marked',
            timestamp: displayTime,
            pile: payload.pile ?? copy[waypointId]?.pile,
            rowNo: payload.rowNo ?? payload.row_no ?? copy[waypointId]?.rowNo,
            remark: payload.remark ?? copy[waypointId]?.remark,
          };
          console.log('Backend timestamp:', timestamp, '→ Display time:', displayTime);
        }

        console.log('Status Map Updated:', copy);
        return copy;
      });
    };

    onMissionEvent(handler);
    // Cleanup: unregister the handler by passing a no-op function
    return () => {
      onMissionEvent(() => {});
    };
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
                <th className="px-2 py-1">PILE</th>
                <th className="px-2 py-1">ROW No</th>
                <th className="px-2 py-1">Latitude</th>
                <th className="px-2 py-1">Longitude</th>
                <th className="px-2 py-1">Status</th>
                <th className="px-2 py-1">Timestamp</th>
                <th className="px-2 py-1">Remark</th>
              </tr>
            </thead>
            <tbody>
              {waypoints.length === 0 ? (
                <tr>
                  <td colSpan={8} className="text-center py-4 text-gray-500">No waypoints loaded</td>
                </tr>
              ) : (
                waypoints.map((waypoint, idx) => {
                  const wpStatus = statusMap[waypoint.id];
                  const pile = wpStatus?.pile ?? Math.floor(idx / 10) + 1;
                  const rowNo = wpStatus?.rowNo ?? (idx % 10) + 1;
                  
                  // Status rendering logic
                  let statusDisplay = '-';
                  let statusColor = 'text-gray-400';
                  
                  if (wpStatus?.status === 'completed') {
                    statusDisplay = '✓✓';
                    statusColor = 'text-green-500 font-bold';
                  } else if (wpStatus?.marked) {
                    statusDisplay = '✓✓';
                    statusColor = 'text-blue-500 font-bold';
                  } else if (wpStatus?.reached) {
                    statusDisplay = '✓';
                    statusColor = 'text-green-400';
                  } else if (wpStatus?.status === 'loading') {
                    statusDisplay = '⟳';
                    statusColor = 'text-yellow-400 animate-spin inline-block';
                  } else if (wpStatus?.status === 'skipped') {
                    statusDisplay = '⊗';
                    statusColor = 'text-gray-500';
                  }
                  
                  return (
                    <tr key={waypoint.id || idx} className={idx % 2 === 0 ? 'bg-gray-900' : 'bg-gray-800'}>
                      <td className="px-2 py-1">{idx + 1}</td>
                      <td className="px-2 py-1">{pile}</td>
                      <td className="px-2 py-1">{rowNo}</td>
                      <td className="px-2 py-1">{waypoint.lat.toFixed(7)}</td>
                      <td className="px-2 py-1">{waypoint.lng.toFixed(7)}</td>
                      <td className={`px-2 py-1 ${statusColor}`}>{statusDisplay}</td>
                      <td className="px-2 py-1 text-xs">{wpStatus?.timestamp ?? '-'}</td>
                      <td className="px-2 py-1 text-xs text-gray-400">{wpStatus?.remark ?? '-'}</td>
                    </tr>
                  );
                })
              )}
            </tbody>
          </table>
        </div>
      </div>
    </div>
  );
};

export default SprayerMissionLogs;
