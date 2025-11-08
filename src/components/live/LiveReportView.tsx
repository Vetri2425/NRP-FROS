
import React, { useEffect, useState, useCallback } from 'react';
import { Waypoint, RoverData } from '../../types';
import { useRover } from '../../context/RoverContext';
import MapView from '../MapView';
import WaypointStatusList from './WaypointStatusList';
import LiveControls from './LiveControls';
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
}) => {
  const { telemetry } = useRover();
  const [wpMarkStatus, setWpMarkStatus] = useState<WPMarkStatus | null>(null);
  const [isPollingWpMark, setIsPollingWpMark] = useState(false);

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
          <LiveControls 
            isConnected={isConnected}
            currentWaypoint={currentWaypointSeq}
            wpMarkStatus={wpMarkStatus}
            onClearLogs={onClearAll || onClearLogsWithTrail || effectiveClearHandler}
          />
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
