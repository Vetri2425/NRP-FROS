import React, { useCallback, useEffect, useMemo, useState } from 'react';
import { RoverProvider, useRover } from './context/RoverContext';
import Header from './components/Header';
import LeftSidebar from './components/LeftSidebar';
import MapView from './components/MapView';
import PlanControls from './components/plan/PlanControls';
import QGCWaypointTable from './components/plan/QGCWaypointTable';
import LiveReportView from './components/live/LiveReportView';
import MissionLogs from './components/MissionLogs';
import SetupTab from './components/setup/SetupTab';
import ServoControlTab from './components/ServoControl/ServoControlTab';
import { MissionFileInfo, Waypoint, ViewMode, RoverData } from './types';
import { usePersistentState } from './hooks/usePersistentState';
import { useMissionLogs } from './hooks/useMissionLogs';
import { useMissionHistory } from './hooks/useMissionHistory';
import { calculateDistancesForMission, calculateDistance } from './utils/geo';
import { toQGCWPL110 } from './utils/missionParser';
import { sanitizeWaypointsForUpload } from './utils/waypointValidator';
import { RoverTelemetry } from './types/ros';
import { ConnectionState } from './hooks/useRoverROS';
import { ToastContainer } from 'react-toastify';
import 'react-toastify/dist/ReactToastify.css';
// The useTelemetry hook is no longer needed as useRoverROS provides comprehensive telemetry
// import { useTelemetry } from './hooks/useTelemetry';

const mapFixTypeToLabel = (fixType: number): string => {
  switch (fixType) {
    case 4:
      return 'RTK Fixed';
    case 3:
      return 'RTK Float';
    case 2:
      return 'DGPS';
    case 1:
      return 'GPS Fix';
    default:
      return 'No Fix';
  }
};

const toRoverData = (
  telemetry: RoverTelemetry, 
  connectionState: ConnectionState, 
  missionWaypoints: Waypoint[] = []
): RoverData => {
  const isConnected = connectionState === 'connected';
  const position =
    Number.isFinite(telemetry.global.lat) && Number.isFinite(telemetry.global.lon)
      ? { lat: telemetry.global.lat, lng: telemetry.global.lon }
      : null;

  const heading = telemetry.attitude?.yaw_deg ?? 0; // Ensure heading is extracted from telemetry

  // 🔍 DEBUG: Log heading data
  console.log('[APP.TSX toRoverData] Heading Data:', {
    'heading (raw)': telemetry.attitude?.yaw_deg,
    'position': position,
    'connectionState': connectionState,
  });

  const currentWp = telemetry.mission.current_wp ?? 0;
  const completedWaypointIds =
    currentWp > 1 ? Array.from({ length: currentWp - 1 }, (_, idx) => idx + 1) : [];

  // Calculate distance to next waypoint
  let distanceToNext = 0;
  if (position && currentWp > 0 && currentWp <= missionWaypoints.length) {
    const nextWaypoint = missionWaypoints[currentWp - 1]; // currentWp is 1-indexed
    if (nextWaypoint) {
      const distanceMeters = calculateDistance(position, { lat: nextWaypoint.lat, lng: nextWaypoint.lng });
      distanceToNext = distanceMeters * 3.28084; // Convert meters to feet
    }
  }

  // 🔍 DEBUG: Log RTK transformation
  const rtkStatus = mapFixTypeToLabel(telemetry.rtk.fix_type);
  console.log('[APP.TSX toRoverData] RTK Data:', {
    'fix_type (raw)': telemetry.rtk.fix_type,
    'rtk_status (mapped)': rtkStatus,
    'satellites_visible': telemetry.global.satellites_visible,
    'baseline_age': telemetry.rtk.baseline_age,
    'base_linked': telemetry.rtk.base_linked,
    'position': position,
    'connectionState': connectionState
  });

  return {
    connected: isConnected,
    mode: telemetry.state.mode ?? 'UNKNOWN',
    status: telemetry.state.armed ? 'armed' : 'disarmed',
    position,
    latitude: position?.lat ?? undefined,
    longitude: position?.lng ?? undefined,
    altitude: telemetry.global.alt_rel,
    relative_altitude: telemetry.global.alt_rel,
    heading, // Pass heading data
    groundspeed: telemetry.global.vel,
    rtk_status: rtkStatus,
    fix_type: telemetry.rtk.fix_type,
    satellites_visible: telemetry.global.satellites_visible,
    hrms: 0.000, // TODO: Add HRMS to telemetry
    vrms: 0.000, // TODO: Add VRMS to telemetry
    battery: telemetry.battery.percentage,
    voltage: telemetry.battery.voltage,
    current: telemetry.battery.current,
    cpu_load: undefined,
    drop_rate: undefined,
    rc_connected: true,
    signal_strength: isConnected ? 'Good' : 'No Link',
    imu_status: 'ALIGNED', // TODO: Add IMU status to telemetry
    mission_progress: {
      current: telemetry.mission.current_wp,
      total: telemetry.mission.total_wp,
    },
    current_waypoint_id: telemetry.mission.current_wp,
    activeWaypointIndex:
      telemetry.mission.current_wp && telemetry.mission.current_wp > 0
        ? telemetry.mission.current_wp - 1
        : null,
    completedWaypointIds,
    distanceToNext,
    lastUpdate: telemetry.lastMessageTs ?? Date.now(),
    telemetryAgeMs: telemetry.lastMessageTs ? Date.now() - telemetry.lastMessageTs : undefined,
  };
};

const AppContent: React.FC = () => {
  const { telemetry, connectionState, services, onMissionEvent } = useRover();

  const [viewMode, setViewMode] = usePersistentState<ViewMode>('app:viewMode', 'dashboard');
  const [missionWaypoints, setMissionWaypoints] = usePersistentState<Waypoint[]>(
    'app:missionWaypoints',
    [],
  );
  const [isFullScreen, setIsFullScreen] = useState(false);
  const [currentMissionFileName, setCurrentMissionFileName] = usePersistentState<string | null>(
    'app:missionFileName',
    null,
  );
  const [missionFileInfo, setMissionFileInfo] = usePersistentState<MissionFileInfo | null>(
    'app:missionFileInfo',
    null,
  );
  const [selectedWaypointIds, setSelectedWaypointIds] = usePersistentState<number[]>(
    'app:selectedWaypoints',
    [],
  );

  const {
    missionLogs,
    getActiveLogEntries,
    clearLogs,
    addLogEntry,
  } = useMissionLogs();

  // When cleared, suppress live report metrics (distance, marked point, next point)
  const [isLiveReportCleared, setIsLiveReportCleared] = useState(false);

  // Mission history for undo/redo
  const {
    canUndo,
    canRedo,
    undo,
    redo,
    pushSnapshot,
    clearHistory,
    getHistorySize
  } = useMissionHistory(20);

  // Subscribe to mission events from backend via socket
  useEffect(() => {
    onMissionEvent((event) => {
      // Convert backend mission event to log entry format
      addLogEntry({
        event: event.message,
        lat: event.lat ?? null,
        lng: event.lng ?? null,
        waypointId: event.waypointId ?? null,
        status: event.status ?? null,
        servoAction: event.servoAction ?? null,
        timestamp: event.timestamp,
      });
      // New live mission activity should un-clear the report automatically
      setIsLiveReportCleared(false);
    });
  }, [onMissionEvent, addLogEntry]);

  useEffect(() => {
    const onFullscreenChange = () => setIsFullScreen(Boolean(document.fullscreenElement));
    document.addEventListener('fullscreenchange', onFullscreenChange);
    return () => document.removeEventListener('fullscreenchange', onFullscreenChange);
  }, []);

  // Keyboard shortcuts for undo/redo
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      // Ctrl+Z or Cmd+Z for undo
      if ((e.ctrlKey || e.metaKey) && e.key === 'z' && !e.shiftKey) {
        e.preventDefault();
        handleUndo();
      }
      // Ctrl+Shift+Z or Cmd+Shift+Z for redo
      else if ((e.ctrlKey || e.metaKey) && e.key === 'z' && e.shiftKey) {
        e.preventDefault();
        handleRedo();
      }
      // Ctrl+Y or Cmd+Y for redo (alternative)
      else if ((e.ctrlKey || e.metaKey) && e.key === 'y') {
        e.preventDefault();
        handleRedo();
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [canUndo, canRedo]); // Re-bind when undo/redo availability changes

  // Create snapshot when mission waypoints change
  useEffect(() => {
    if (missionWaypoints.length > 0) {
      pushSnapshot(missionWaypoints, missionFileInfo);
    }
  }, [missionWaypoints, missionFileInfo, pushSnapshot]);

  const isConnectedToRover = connectionState === 'connected';

  const uiRoverData: RoverData = useMemo(
    () => toRoverData(telemetry, connectionState, missionWaypoints),
    [telemetry, connectionState, missionWaypoints],
  );

  const handleToggleFullScreen = () => {
    if (!document.fullscreenElement) {
      document.documentElement.requestFullscreen();
    } else if (document.exitFullscreen) {
      document.exitFullscreen();
    }
  };

  const handleMissionUpload = useCallback(
    (waypoints: Waypoint[], info: MissionFileInfo) => {
      setMissionWaypoints(calculateDistancesForMission(waypoints));
      setCurrentMissionFileName(info.name);
      setMissionFileInfo(info);
    },
    [setMissionWaypoints, setCurrentMissionFileName, setMissionFileInfo],
  );

  const handleMissionDownloaded = useCallback(
    (waypoints: Waypoint[]) => {
      const withDistance = calculateDistancesForMission(waypoints);
      setMissionWaypoints(withDistance);
      setCurrentMissionFileName(`Mission from Rover - ${new Date().toLocaleTimeString()}`);
      setMissionFileInfo({
        name: 'Mission from Rover',
        size: withDistance.length,
        type: 'downloaded',
        uploadedAt: new Date().toISOString(),
        waypointCount: withDistance.length,
        source: 'downloaded',
      });
    },
    [setMissionWaypoints, setCurrentMissionFileName, setMissionFileInfo],
  );

  const handleClearMission = useCallback(() => {
    setMissionWaypoints([]);
    setCurrentMissionFileName(null);
    setMissionFileInfo(null);
    setSelectedWaypointIds([]);
    clearHistory(); // Clear undo/redo history when mission is cleared
  }, [setMissionWaypoints, setCurrentMissionFileName, setMissionFileInfo, setSelectedWaypointIds, clearHistory]);

  // Undo handler
  const handleUndo = useCallback(() => {
    if (!canUndo) {
      console.log('No more actions to undo');
      return;
    }

    const snapshot = undo();
    if (snapshot) {
      setMissionWaypoints(snapshot.waypoints);
      setMissionFileInfo(snapshot.fileInfo);
      console.log(`Undone to snapshot from ${new Date(snapshot.timestamp).toLocaleTimeString()}`);
    }
  }, [canUndo, undo, setMissionWaypoints, setMissionFileInfo]);

  // Redo handler
  const handleRedo = useCallback(() => {
    if (!canRedo) {
      console.log('No more actions to redo');
      return;
    }

    const snapshot = redo();
    if (snapshot) {
      setMissionWaypoints(snapshot.waypoints);
      setMissionFileInfo(snapshot.fileInfo);
      console.log(`Redone to snapshot from ${new Date(snapshot.timestamp).toLocaleTimeString()}`);
    }
  }, [canRedo, redo, setMissionWaypoints, setMissionFileInfo]);

  const handleWriteToRover = useCallback(async (): Promise<boolean> => {
    if (missionWaypoints.length === 0) {
      alert('No mission loaded. Add waypoints before uploading.');
      return false;
    }
    try {
      // Sanitize waypoints before upload (e.g., convert negative altitudes to positive)
      const sanitizedWaypoints = sanitizeWaypointsForUpload(missionWaypoints);
      
      const response = await services.uploadMission(sanitizedWaypoints);
      if (response.success) {
        alert(`Mission upload requested (${missionWaypoints.length} waypoints).`);
        return true;
      }
      alert(response.message ?? 'Mission upload failed.');
      return false;
    } catch (error) {
      alert(error instanceof Error ? error.message : 'Mission upload failed.');
      return false;
    }
  }, [missionWaypoints, services]);

  const handleReadFromRover = useCallback(async () => {
    try {
      const response = await services.downloadMission();
      const fetched = Array.isArray((response as any).waypoints) ? (response as any).waypoints : [];
      
      if (response.success && fetched.length > 0) {
        // Validate waypoints before accepting them
        const validWaypoints = fetched.filter((wp: any) => {
          const hasValidCoords = 
            typeof wp.lat === 'number' && 
            typeof wp.lng === 'number' &&
            !isNaN(wp.lat) && 
            !isNaN(wp.lng) &&
            Math.abs(wp.lat) <= 90 && 
            Math.abs(wp.lng) <= 180 &&
            !(wp.lat === 0 && wp.lng === 0); // Reject null island
          
          if (!hasValidCoords) {
            console.warn('Invalid waypoint coordinates:', wp);
          }
          return hasValidCoords;
        });

        if (validWaypoints.length === 0) {
          throw new Error('All downloaded waypoints have invalid coordinates');
        }

        if (validWaypoints.length < fetched.length) {
          console.warn(`Filtered out ${fetched.length - validWaypoints.length} invalid waypoints`);
        }

        handleMissionDownloaded(validWaypoints);
        
        const warningMsg = (response as any).warning;
        if (warningMsg === 'timeout') {
          alert(`⚠️ ${response.message || `Downloaded ${validWaypoints.length} waypoints (with timeout warning)`}`);
        } else if (warningMsg === 'fallback') {
          alert(`⚠️ ${response.message || `Using cached ${validWaypoints.length} waypoints`}`);
        } else if (validWaypoints.length < fetched.length) {
          alert(`✅ Downloaded ${validWaypoints.length} valid waypoints (${fetched.length - validWaypoints.length} invalid filtered out)`);
        } else {
          alert(`✅ Downloaded ${validWaypoints.length} waypoints from rover`);
        }
      } else if (response.success) {
        throw new Error('Rover mission is empty');
      } else {
        throw new Error(response.message ?? 'Failed to download mission from rover');
      }
    } catch (error) {
      const errorMsg = error instanceof Error ? error.message : 'Failed to download mission from rover';
      console.error('Mission download error:', error);
      // Error is now handled in PlanControls with better UI feedback
      throw error; // Re-throw to let PlanControls handle it
    }
  }, [services, handleMissionDownloaded]);

  const handleMapClick = () => {
    // placeholder for future map click behaviour
  };

  const handleDeleteWaypoint = useCallback(
    (id: number) => {
      setMissionWaypoints((prev) => calculateDistancesForMission(prev.filter((wp) => wp.id !== id)));
      setSelectedWaypointIds((prev) => prev.filter((selectedId) => selectedId !== id));
    },
    [setMissionWaypoints, setSelectedWaypointIds],
  );

  const handleUpdateWaypoint = useCallback(
    (id: number, newValues: Partial<Omit<Waypoint, 'id'>>) => {
      setMissionWaypoints((prev) => prev.map((wp) => (wp.id === id ? { ...wp, ...newValues } : wp)));
    },
    [setMissionWaypoints],
  );

  const handleUpdateWaypointPosition = useCallback(
    (waypointId: number, newPosition: { lat: number; lng: number }) => {
      setMissionWaypoints((prev) =>
        calculateDistancesForMission(
          prev.map((wp) => (wp.id === waypointId ? { ...wp, ...newPosition } : wp)),
        ),
      );
    },
    [setMissionWaypoints],
  );

  const handleNewMissionDrawn = useCallback(
    (points: { lat: number; lng: number }[]) => {
      const newWaypoints: Waypoint[] = points.map((p, index) => ({
        id: index + 1,
        command: 'WAYPOINT',
        lat: p.lat,
        lng: p.lng,
        alt: 50,
        frame: 3,
        current: index === 0 ? 1 : 0,
        autocontinue: 1,
      }));
      handleMissionUpload(newWaypoints, {
        name: `Drawn Mission - ${new Date().toLocaleTimeString()}`,
        size: newWaypoints.length,
        type: 'drawn',
        uploadedAt: new Date().toISOString(),
        waypointCount: newWaypoints.length,
        source: 'drawn',
      });
    },
    [handleMissionUpload],
  );

  const handleExportMission = useCallback(() => {
    if (missionWaypoints.length === 0) {
      alert('No mission to export.');
      return;
    }
    const blob = new Blob([toQGCWPL110(missionWaypoints)], { type: 'text/plain;charset=utf-8' });
    const link = document.createElement('a');
    link.href = URL.createObjectURL(blob);
    link.download = 'mission.waypoints';
    link.click();
    URL.revokeObjectURL(link.href);
  }, [missionWaypoints]);

  const handleWaypointSelectionChange = useCallback(
    (id: number, isSelected: boolean) => {
      setSelectedWaypointIds((prev) =>
        isSelected ? [...prev, id] : prev.filter((selectedId) => selectedId !== id),
      );
    },
    [setSelectedWaypointIds],
  );

  const commonMapProps = {
    missionWaypoints,
    onMapClick: handleMapClick,
    roverPosition: uiRoverData.position, // Ensure roverPosition is passed
    heading: uiRoverData.heading, // Pass heading data to MapView
    viewMode,
    isFullScreen,
    onNewMissionDrawn: handleNewMissionDrawn,
    isConnectedToRover,
    onUpdateWaypointPosition: handleUpdateWaypointPosition,
    telemetry: {
      speed: telemetry.global.vel,
      battery: telemetry.battery.percentage,
      signalStrength: isConnectedToRover ? 85 : 0, // Mock signal strength based on connection
      altitude: telemetry.global.alt_rel,
      satellites: telemetry.global.satellites_visible,
    },
  };

  const renderMainContent = () => {
    switch (viewMode) {
      case 'servo':
        return (
          <main className="flex-1 flex p-3 overflow-hidden min-h-0">
            <ServoControlTab />
          </main>
        );
      case 'planning':
        return (
          <main className="flex-1 flex p-3 gap-3 overflow-hidden min-h-0">
            <div className="flex-1 flex flex-col gap-3 min-h-0">
              <div className="flex-1 min-h-0">
                <MapView {...commonMapProps} />
              </div>
              <div className="h-48 overflow-hidden">
                <QGCWaypointTable
                  waypoints={missionWaypoints}
                  onDelete={handleDeleteWaypoint}
                  onUpdate={handleUpdateWaypoint}
                  activeWaypointIndex={uiRoverData.activeWaypointIndex}
                  selectedWaypointIds={selectedWaypointIds}
                  onWaypointSelectionChange={handleWaypointSelectionChange}
                />
              </div>
            </div>
            <aside className="w-72 flex flex-col gap-3 min-h-0">
              <PlanControls
                missionWaypoints={missionWaypoints}
                onUpload={handleMissionUpload}
                onExport={handleExportMission}
                onUploadInitiated={() => {}}
                onWriteToRover={handleWriteToRover}
                onReadFromRover={handleReadFromRover}
                missionFileInfo={missionFileInfo}
                onClearMission={handleClearMission}
                isConnected={isConnectedToRover}
                uploadProgress={0}
              />
            </aside>
          </main>
        );
      case 'live':
        return (
          <LiveReportView
            missionWaypoints={missionWaypoints}
            liveRoverData={uiRoverData}
            missionName={currentMissionFileName}
            isConnected={isConnectedToRover}
            isCleared={isLiveReportCleared}
            onClearLogs={() => {
              if (!missionLogs.length) return;
              const ok = window.confirm('This will clear the current mission logs and waypoints. Continue?');
              if (ok) {
                clearLogs();
                setMissionWaypoints([]); // Clear all waypoints
                setIsLiveReportCleared(true); // Clear live report metrics
              }
            }}
          />
        );
      case 'setup':
        return (
          <main className="flex-1 flex p-3 overflow-hidden min-h-0">
            <SetupTab />
          </main>
        );
      case 'dashboard':
      default:
        return (
          <main className="flex-1 flex p-3 gap-3 overflow-hidden min-h-0">
            <LeftSidebar />
            <div className="flex-1 flex flex-col gap-3 min-h-0">
              <div className="flex-1 min-h-0">
                <MapView {...commonMapProps} />
              </div>
              <div className="h-44 overflow-hidden">
                <MissionLogs
                  logEntries={getActiveLogEntries()}
                  onDownload={() => {
                    const entries = getActiveLogEntries();
                    if (!entries.length) {
                      alert('No mission logs available to download yet.');
                      return;
                    }
                    const header = [
                      'Timestamp',
                      'Latitude',
                      'Longitude',
                      'Waypoint',
                      'Status',
                      'Servo Action',
                      'Remark',
                    ];
                    const rows = entries.map((entry) => {
                      const safeEvent = entry.event.replace(/"/g, '""');
                      const lat = entry.lat != null ? entry.lat.toFixed(7) : '';
                      const lng = entry.lng != null ? entry.lng.toFixed(7) : '';
                      const waypoint = entry.waypointId ?? '';
                      const status = entry.status ?? '';
                      const servo = entry.servoAction ?? '';
                      return [
                        entry.timestamp,
                        lat,
                        lng,
                        waypoint,
                        status,
                        servo,
                        `"${safeEvent}"`,
                      ].join(',');
                    });
                    const csvContent = [header.join(','), ...rows].join('\n');
                    const blob = new Blob([csvContent], { type: 'text/csv;charset=utf-8;' });
                    const url = URL.createObjectURL(blob);
                    const link = document.createElement('a');
                    link.href = url;
                    link.download = `mission-log-${new Date().toISOString().replace(/[:.]/g, '-')}.csv`;
                    document.body.appendChild(link);
                    link.click();
                    document.body.removeChild(link);
                    URL.revokeObjectURL(url);
                  }}
                  onClear={() => {
                    if (missionLogs.length === 0) {
                      return;
                    }
                    const confirmClear = window.confirm(
                      'This will clear the current mission logs. Continue?',
                    );
                    if (confirmClear) {
                      clearLogs();
                    }
                  }}
                />
              </div>
            </div>
          </main>
        );
    }
  };

  return (
    <div className="text-white w-screen h-screen flex flex-col font-sans bg-slate-800 overflow-hidden">
      <Header
        viewMode={viewMode}
        setViewMode={setViewMode}
        isFullScreen={isFullScreen}
        onToggleFullScreen={handleToggleFullScreen}
      />
      {renderMainContent()}
    </div>
  );
};

const App: React.FC = () => (
  <RoverProvider>
    <AppContent />
    <ToastContainer
      position="top-right"
      autoClose={3000}
      hideProgressBar={false}
      newestOnTop={true}
      closeOnClick
      rtl={false}
      pauseOnFocusLoss
      draggable
      pauseOnHover
      theme="dark"
    />
  </RoverProvider>
);

export default App;
