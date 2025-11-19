
import React, { useEffect, useState, useCallback } from 'react';
import { Waypoint, RoverData } from '../../types';
import { useRover } from '../../context/RoverContext';
import type { SprayStatus } from './SprayStatusIndicator';
import MapView from '../MapView';
import WaypointStatusList from './WaypointStatusList';
import LiveControls from './LiveControls';
import MissionController, { MissionStatus } from './MissionController';
import LiveStatusbar from './LiveStatusbar';
import { getWPMarkStatus, pollWPMarkStatus, WPMarkStatus } from '../../services/wpMarkService';

type LiveReportViewProps = {
  missionWaypoints: Waypoint[];
  liveRoverData: RoverData;
  missionName: string | null;
  isConnected: boolean;
  onClearLogs?: () => void;
  isCleared?: boolean;
  clearTrail?: () => void;
  onClearLogsWithTrail?: () => void;
  onClearAll?: () => void;
  hasMissionLogs?: boolean;
  hasWpMarkStatus?: boolean;
  isSprayerMode?: boolean; // New prop to indicate sprayer mode
};

const LiveReportView: React.FC<LiveReportViewProps> = ({
  missionWaypoints,
  liveRoverData,
  missionName,
  isConnected,
  onClearLogs,
  isCleared = false,
  clearTrail,
  onClearLogsWithTrail,
  onClearAll,
  hasMissionLogs = false,
  hasWpMarkStatus = false,
  isSprayerMode = false, // Default to false for backward compatibility
}) => {
  const { telemetry, onMissionEvent } = useRover();
  const [wpMarkStatus, setWpMarkStatus] = useState<WPMarkStatus | null>(null);
  const [isPollingWpMark, setIsPollingWpMark] = useState(false);
  const [missionStatus, setMissionStatus] = useState<MissionStatus | null>(null);
  const [sprayStatuses, setSprayStatuses] = useState<SprayStatus[]>([]);

  // Combined clear function that clears trails and logs
  const handleClearLogsWithTrail = useCallback(async () => {
    try {
      // Stop WP_MARK mission if running
      if (wpMarkStatus?.running) {
        const { stopWPMarkMission } = await import('../../services/wpMarkService');
        await stopWPMarkMission();
        console.log('[LiveReportView] Stopped WP_MARK mission');
      }
    } catch (error) {
      console.error('[LiveReportView] Failed to stop WP_MARK mission:', error);
    }

    // Clear trails
    clearTrail?.();

    // Clear logs and waypoints
    onClearLogs?.();
  }, [clearTrail, onClearLogs, wpMarkStatus?.running]);

  // Use onClearAll if provided, otherwise use the combined function
  const effectiveClearHandler = onClearAll || handleClearLogsWithTrail;

  const activeWaypointId =
    !isCleared && liveRoverData.activeWaypointIndex !== null && liveRoverData.activeWaypointIndex !== undefined
      ? liveRoverData.activeWaypointIndex + 1
      : null;

  // Use mission_progress data if available, otherwise fall back to activeWaypointIndex
  const currentWaypointSeq = isCleared
    ? 0
    : (liveRoverData.mission_progress?.current || 
      liveRoverData.current_waypoint_id || 
      (liveRoverData.activeWaypointIndex !== null ? liveRoverData.activeWaypointIndex + 1 : 0));

  // Build a display-safe rover data when cleared (zero distance, no completed, no active)
  const displayRoverData: RoverData = isCleared
    ? {
        ...liveRoverData,
        completedWaypointIds: [],
        activeWaypointIndex: null,
        distanceToNext: 0,
      }
    : liveRoverData;

  // Poll WP_MARK status when connected and not cleared
  useEffect(() => {
    if (!isConnected || isCleared) {
      setWpMarkStatus(null);
      setIsPollingWpMark(false);
      return;
    }

    let stopPolling: (() => void) | null = null;

    const checkAndStartPolling = async () => {
      try {
        const status = await getWPMarkStatus();
        if (status.running) {
          setWpMarkStatus(status);
          if (!isPollingWpMark) {
            setIsPollingWpMark(true);
            stopPolling = pollWPMarkStatus((updatedStatus) => {
              setWpMarkStatus(updatedStatus);
            }, 2000); // Poll every 2 seconds
          }
        } else {
          setWpMarkStatus(null);
          setIsPollingWpMark(false);
        }
      } catch (error) {
        console.error('Failed to check WP_MARK status:', error);
        setWpMarkStatus(null);
        setIsPollingWpMark(false);
      }
    };

    // Initial check
    checkAndStartPolling();

    // Cleanup
    return () => {
      if (stopPolling) {
        stopPolling();
      }
      setIsPollingWpMark(false);
    };
  }, [isConnected, isCleared, isPollingWpMark]);

  // Subscribe to mission status events to update per-waypoint spray statuses
  useEffect(() => {
    const handler = (ev: any) => {
      // Support both old wrapped shape ({ type: 'mission_status', data })
      // and the flat server payload emitted as described in the backend.
      const payload = ev && ev.type === 'mission_status' && ev.data ? ev.data : ev;

      if (!payload || typeof payload !== 'object') return;

      const et = payload.event_type;
      const wpId = payload.waypoint_id ?? payload.waypointId ?? payload.current_waypoint ?? null;
      if (!wpId) return;

      setSprayStatuses((prev) => {
        const copy = [...prev];
        const idx = copy.findIndex(s => s.waypointId === wpId);
        // Initialize if missing
        if (idx === -1) {
          copy.push({ waypointId: wpId, status: 'pending' });
        }

        const targetIndex = copy.findIndex(s => s.waypointId === wpId);
        const current = copy[targetIndex];

        if (et === 'waypoint_reached') {
          // Vehicle reached the waypoint; set to navigating (about to mark)
          current.status = 'navigating';
          current.sprayStartTime = payload.timestamp || new Date(payload.server_timestamp ? payload.server_timestamp * 1000 : Date.now()).toISOString();
        } else if (et === 'waypoint_marked') {
          const mark = payload.marking_status || payload.markingStatus || payload.marking || null;
          const dur = typeof payload.spray_duration === 'number' ? payload.spray_duration : (payload.sprayDuration ?? null);
          if (mark === 'completed') {
            current.status = 'completed';
            if (dur != null) current.duration = dur;
            current.sprayEndTime = payload.timestamp || new Date().toISOString();
          } else if (mark === 'failed') {
            current.status = 'failed';
            current.sprayEndTime = payload.timestamp || new Date().toISOString();
            if (dur != null) current.duration = dur;
          } else {
            // Unknown marking status — store message and set failed
            current.status = 'failed';
            current.sprayEndTime = payload.timestamp || new Date().toISOString();
          }
        } else if (et === 'waypoint_error') {
          current.status = 'failed';
          current.sprayEndTime = payload.timestamp || new Date().toISOString();
        }

        // Ensure consistent ordering by waypoint id
        copy.sort((a, b) => a.waypointId - b.waypointId);
        return copy;
      });
    };

    onMissionEvent(handler);
    return () => onMissionEvent(() => {});
  }, [onMissionEvent]);

  return (
    <div className="flex-1 flex flex-col p-3 gap-3 overflow-hidden min-h-0">
      {/* Top section with 3 panels */}
      <div className="flex-1 flex gap-3 overflow-hidden min-h-0">
  {/* Left Panel: Waypoint List */}
  <aside className="w-[240px] flex-shrink-0 bg-[#111827] rounded-lg overflow-hidden" style={{height: '98%'}}>
          <WaypointStatusList
            waypoints={missionWaypoints}
            activeWaypointId={activeWaypointId}
            completedWaypointIds={isCleared ? [] : liveRoverData.completedWaypointIds}
            sprayStatuses={sprayStatuses}
            isSprayerMode={isSprayerMode}
          />
        </aside>

        {/* Center Panel: Map View */}
        <main className="flex-1 flex flex-col min-h-0" style={{height: '98%'}}>
          <MapView
            missionWaypoints={missionWaypoints}
            onMapClick={() => {}}
            roverPosition={liveRoverData.position}
            activeWaypointIndex={isCleared ? null : liveRoverData.activeWaypointIndex}
            heading={liveRoverData.heading}
            viewMode="live"
            isFullScreen={false} // Full screen is handled by the browser
            onNewMissionDrawn={() => {}}
            isConnectedToRover={false} // In live view, we just display data
            onUpdateWaypointPosition={() => {}}
            clearTrail={clearTrail}
            telemetry={{
              speed: telemetry.global.vel,
              battery: telemetry.battery.percentage,
              signalStrength: isConnected ? 85 : 0, // Mock signal strength based on connection
              altitude: telemetry.global.alt_rel,
              satellites: telemetry.global.satellites_visible,
            }}
          />
        </main>

        {/* Right Panel: Controls */}
        <aside className="w-[200px] flex-shrink-0" style={{height: '98%'}}>
          {isSprayerMode ? (
            <MissionController
              isConnected={isConnected}
              waypoints={missionWaypoints}
              missionStatus={missionStatus}
              onMissionLoad={(waypoints, config) => {
                console.log('[LiveReportView] Mission loaded:', { waypoints: waypoints.length, config });
              }}
              onMissionStart={() => {
                console.log('[LiveReportView] Mission started');
              }}
              onMissionStop={() => {
                console.log('[LiveReportView] Mission stopped');
              }}
              onMissionPause={() => {
                console.log('[LiveReportView] Mission paused');
              }}
              onMissionResume={() => {
                console.log('[LiveReportView] Mission resumed');
              }}
              onMissionNext={() => {
                console.log('[LiveReportView] Next waypoint requested');
              }}
            />
          ) : (
            <LiveControls 
              isConnected={isConnected}
              currentWaypoint={currentWaypointSeq}
              wpMarkStatus={wpMarkStatus}
              onClearLogs={onClearAll || onClearLogsWithTrail || effectiveClearHandler}
            />
          )}
        </aside>
      </div>

      <footer className="h-60 flex-shrink-0">
        <LiveStatusbar 
            missionName={missionName}
            waypoints={missionWaypoints}
            liveRoverData={displayRoverData}
            wpMarkStatus={wpMarkStatus}
        />
      </footer>
    </div>
  );
};

export default LiveReportView;
