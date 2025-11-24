// src/components/sprayer/SprayerReportView.tsx
import React, { useEffect, useState } from 'react';
import { toast } from 'react-toastify';
import { Waypoint, RoverData } from '../../types';
import { useRover } from '../../context/RoverContext';
import MapView from '../MapView';
import WaypointStatusList from '../live/WaypointStatusList';
import SprayerControls from './SprayerLiveControls';
import SprayerStatusbar from './SprayerStatusbar';
import SprayerMissionLogs from './SprayerMissionLogs';
import type { SprayStatus } from '../live/SprayStatusIndicator';

type SprayerReportViewProps = {
  missionWaypoints: Waypoint[];
  liveRoverData: RoverData;
  missionName: string | null;
  isConnected: boolean;
  onClearAll?: () => void;
};

const SprayerReportView: React.FC<SprayerReportViewProps> = ({
  missionWaypoints,
  liveRoverData,
  missionName,
  isConnected,
  onClearAll,
}) => {
  const { telemetry, onMissionEvent, services } = useRover();
  const [logs, setLogs] = useState<any[]>([]);
  const [sprayStatuses, setSprayStatuses] = useState<SprayStatus[]>([]);

  // NOTE: Sprayer configuration UI removed — frontend only loads waypoints

  // ---------- � Load Mission (Waypoints only) ----------
  const handleLoadMission = async () => {
    try {
      if (missionWaypoints.length === 0) {
        toast.warning('No waypoints to load');
        return;
      }
      
      // Remove any per-waypoint autocontinue flags; backend will respect global mode
      const sanitized = missionWaypoints.map(({ autocontinue, ...rest }) => ({ ...rest }));
      const response = await services.loadMissionToController(sanitized as any);
      if (response.success) {
        toast.success('Mission loaded successfully!');
      } else {
        toast.error(response.message || 'Failed to load mission');
      }
    } catch (error) {
      console.error('[SprayerReportView] Load Error:', error);
      toast.error('Failed to load mission');
    }
  };

  // ---------- ▶️ Start Mission ----------
  const handleStart = async () => {
    try {
      const response = await services.startMissionController();
      if (response.success) {
        toast.success('Mission started successfully!');
      } else {
        toast.error(response.message || 'Failed to start mission');
      }
    } catch (error) {
      console.error('[SprayerReportView] Start Error:', error);
      toast.error('Failed to start mission');
    }
  };

  // ---------- ⏸️ Pause Mission ----------
  const handlePause = async () => {
    try {
      const response = await services.pauseMission();
      if (response.success) {
        toast.success('Mission paused');
      } else {
        toast.error(response.message || 'Failed to pause mission');
      }
    } catch (error) {
      console.error('[SprayerReportView] Pause Error:', error);
      toast.error('Failed to pause mission');
    }
  };

  // ---------- 🔁 Resume Mission ----------
  const handleResume = async () => {
    try {
      const response = await services.resumeMission();
      if (response.success) {
        toast.success('Mission resumed');
      } else {
        toast.error(response.message || 'Failed to resume mission');
      }
    } catch (error) {
      console.error('[SprayerReportView] Resume Error:', error);
      toast.error('Failed to resume mission');
    }
  };

  // ---------- ⏹️ Stop Mission ----------
  const handleStop = async () => {
    try {
      const response = await services.stopMissionController();
      if (response.success) {
        toast.success('Mission stopped successfully!');
      } else {
        toast.error(response.message || 'Failed to stop mission');
      }
    } catch (error) {
      console.error('[SprayerReportView] Stop Error:', error);
      toast.error('Failed to stop mission');
    }
  };

  // ---------- 🔄 Restart Mission ----------
  const handleRestart = async () => {
    try {
      const response = await services.restartMissionController();
      if (response.success) {
        toast.success('Mission restarted successfully!');
      } else {
        toast.error(response.message || 'Failed to restart mission');
      }
    } catch (error) {
      console.error('[SprayerReportView] Restart Error:', error);
      toast.error('Failed to restart mission');
    }
  };

  // ---------- ⏭️ Next Waypoint ----------
  const handleNext = async () => {
    try {
      const response = await services.nextWaypoint();
      if (response.success) {
        toast.success('Moved to next waypoint');
      } else {
        toast.error(response.message || 'Failed to move to next waypoint');
      }
    } catch (error) {
      console.error('[SprayerReportView] Next Error:', error);
      toast.error('Failed to move to next waypoint');
    }
  };

  // ---------- 🧾 Mission Event Logging ----------
  useEffect(() => {
    const handleMissionEvent = (event: any) => {
      setLogs((prev) => [
        ...prev,
        {
          timestamp: Date.now(),
          message: event.message || 'Mission event',
          type: event.status || 'info',
        },
      ]);
    };

    const wrapped = (ev: any) => {
      handleMissionEvent(ev);

      // Also update per-waypoint spray statuses when applicable
      const payload = ev && ev.type === 'mission_status' && ev.data ? ev.data : ev;
      if (!payload || typeof payload !== 'object') return;
      const et = payload.event_type;
      const wpId = payload.waypoint_id ?? payload.waypointId ?? payload.current_waypoint ?? null;
      if (!wpId) return;

      setSprayStatuses((prev) => {
        const copy = [...prev];
        const idx = copy.findIndex(s => s.waypointId === wpId);
        if (idx === -1) copy.push({ waypointId: wpId, status: 'pending' });
        const target = copy.find(s => s.waypointId === wpId)!;

        if (et === 'waypoint_reached') {
          target.status = 'navigating';
          target.sprayStartTime = payload.timestamp || new Date().toISOString();
        } else if (et === 'waypoint_marked') {
          const mark = payload.marking_status || payload.markingStatus || null;
          const dur = typeof payload.spray_duration === 'number' ? payload.spray_duration : (payload.sprayDuration ?? null);
          if (mark === 'completed') {
            target.status = 'completed';
            if (dur != null) target.duration = dur;
            target.sprayEndTime = payload.timestamp || new Date().toISOString();
          } else {
            target.status = 'failed';
            if (dur != null) target.duration = dur;
            target.sprayEndTime = payload.timestamp || new Date().toISOString();
          }
        } else if (et === 'waypoint_error') {
          target.status = 'failed';
          target.sprayEndTime = payload.timestamp || new Date().toISOString();
        }

        copy.sort((a, b) => a.waypointId - b.waypointId);
        return copy;
      });
    };

    const unregister = onMissionEvent(wrapped);
    return () => {
      try {
        unregister();
      } catch (err) {
        // swallow any errors during cleanup to avoid disrupting unmount
        console.debug('[SprayerReportView] cleanup error:', err);
      }
    };
  }, [onMissionEvent]);

  // ---------- 🧠 UI Rendering (unchanged) ----------
  return (
    <div className="flex-1 flex flex-col p-3 gap-3 overflow-hidden min-h-0">
      <div className="flex-1 flex gap-3 overflow-hidden min-h-0">
        {/* Waypoint List - optional, disabled for sprayer */}
        {false && (
          <aside className="w-[240px] flex-shrink-0 bg-[#111827] rounded-lg overflow-hidden" style={{ height: '98%' }}>
            <WaypointStatusList
              waypoints={missionWaypoints}
              activeWaypointId={
                liveRoverData.activeWaypointIndex !== null
                  ? liveRoverData.activeWaypointIndex + 1
                  : null
              }
              completedWaypointIds={liveRoverData.completedWaypointIds}
              sprayStatuses={[]}
              isSprayerMode={true}
            />
          </aside>
        )}

        {/* Center Map */}
        <main className="flex-1 flex flex-col min-h-0" style={{ height: '99%' }}>
          <MapView
            missionWaypoints={missionWaypoints}
            onMapClick={() => {}}
            roverPosition={liveRoverData.position}
            activeWaypointIndex={liveRoverData.activeWaypointIndex}
            heading={liveRoverData.heading}
            viewMode="sprayer"
            isFullScreen={false}
            onNewMissionDrawn={() => {}}
            isConnectedToRover={isConnected}
            onUpdateWaypointPosition={() => {}}
            telemetry={{
              speed: telemetry.global.vel,
              battery: telemetry.battery.percentage,
              signalStrength: isConnected ? 85 : 0,
              altitude: telemetry.global.alt_rel,
              satellites: telemetry.global.satellites_visible,
            }}
          />
        </main>

        {/* Right Controls */}
        <aside className="w-[200px] flex-shrink-0" style={{ height: '99%' }}>
          <SprayerControls
            isConnected={isConnected}
            onLoadMission={handleLoadMission}
            onStart={handleStart}
            onPause={handlePause}
            onResume={handleResume}
            onStop={handleStop}
            onNext={handleNext}
            onRestart={handleRestart}
            waypoints={missionWaypoints}  // Pass waypoints to controls
          />
        </aside>
      </div>

      {/* Footer */}
      <footer className="flex gap-3 h-60 w-full flex-shrink-0">
        <div style={{ width: '180px', minWidth: '120px', height: '100%' }}>
          <SprayerStatusbar
            missionName={missionName}
            waypoints={missionWaypoints}
            liveRoverData={liveRoverData}
          />
        </div>
        <div style={{ flex: 1, height: '100%' }}>
          <SprayerMissionLogs waypoints={missionWaypoints} />
        </div>
      </footer>
    </div>
  );
};

export default SprayerReportView;
