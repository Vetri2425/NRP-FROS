
import React, { useEffect, useState } from 'react';
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
};

const LiveReportView: React.FC<LiveReportViewProps> = ({
  missionWaypoints,
  liveRoverData,
  missionName,
  isConnected,
  onClearLogs,
  isCleared = false,
}) => {
  const { telemetry } = useRover();
  const [wpMarkStatus, setWpMarkStatus] = useState<WPMarkStatus | null>(null);
  const [isPollingWpMark, setIsPollingWpMark] = useState(false);
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
            onClearLogs={onClearLogs}
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
