import React, { useCallback, useEffect, useMemo, useState, useRef } from 'react';
import { RoverProvider, useRover } from './context/RoverContext';
import Header from './components/Header';
import LeftSidebar from './components/LeftSidebar';
import MapView from './components/MapView';
import PlanControls from './components/plan/PlanControls';
import QGCWaypointTable from './components/plan/QGCWaypointTable';
import LiveReportView from './components/live/LiveReportView';
import { SprayerReportView } from './components/sprayer';
import MissionLogs from './components/MissionLogs';
import SetupTab from './components/setup/SetupTab';
import { MissionFileInfo, Waypoint, ViewMode, RoverData } from './types';
import { usePersistentState } from './hooks/usePersistentState';
import { useMissionLogs } from './hooks/useMissionLogs';
import { useMissionHistory } from './hooks/useMissionHistory';
import { calculateDistancesForMission, calculateDistance, calculateBearing, calculateDestination } from './utils/geo';
import { toQGCWPL110 } from './utils/missionParser';
import { sanitizeWaypointsForUpload } from './utils/waypointValidator';
import { RoverTelemetry } from './types/ros';
import { ConnectionState } from './hooks/useRoverROS';
import { ToastContainer } from 'react-toastify';
import 'react-toastify/dist/ReactToastify.css';
import { useDialog } from './hooks/useDialog';
import GenericDialog from './components/GenericDialog';
// The useTelemetry hook is no longer needed as useRoverROS provides comprehensive telemetry
// import { useTelemetry } from './hooks/useTelemetry';

const mapFixTypeToLabel = (fixType: number): string => {
  switch (fixType) {
    // 0: No GPS, 1: No Fix, 2: 2D Fix, 3: 3D Fix, 4: DGPS, 5: RTK Float, 6: RTK Fixed
    case 0:
      return 'No GPS';
    case 1:
      return 'No Fix';
    case 2:
      return '2D Fix';
    case 3:
      return '3D Fix';
    case 4:
      return 'DGPS';
    case 5:
      return 'RTK Float';
    case 6:
      return 'RTK Fixed';
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

  // Calculate distance to next waypoint with robust validation
  let distanceToNext = 0;
  if (position && currentWp > 0 && currentWp <= missionWaypoints.length) {
    const nextWaypoint = missionWaypoints[currentWp - 1]; // currentWp is 1-indexed
    if (nextWaypoint && 
        Number.isFinite(nextWaypoint.lat) && 
        Number.isFinite(nextWaypoint.lng)) {
      try {
        const distanceMeters = calculateDistance(position, { lat: nextWaypoint.lat, lng: nextWaypoint.lng });
        // Use meters directly
        distanceToNext = Number.isFinite(distanceMeters) && distanceMeters >= 0 
          ? distanceMeters 
          : 0;
        
        // 🔍 DEBUG: Log distance calculation
        console.log('[APP.TSX toRoverData] Distance calculation:', {
          'rover_position': position,
          'next_waypoint': { lat: nextWaypoint.lat, lng: nextWaypoint.lng },
          'distance_meters': distanceMeters,
          'distance_feet': distanceToNext
        });
      } catch (error) {
        console.error('[APP.TSX toRoverData] Distance calculation error:', error);
        distanceToNext = 0;
      }
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
  hrms: typeof telemetry.hrms === 'number' ? telemetry.hrms : (typeof telemetry.hrms === 'string' ? parseFloat(telemetry.hrms) || 0 : 0),
  vrms: typeof telemetry.vrms === 'number' ? telemetry.vrms : (typeof telemetry.vrms === 'string' ? parseFloat(telemetry.vrms) || 0 : 0),
    battery: telemetry.battery.percentage,
    voltage: telemetry.battery.voltage,
    current: telemetry.battery.current,
    cpu_load: undefined,
    drop_rate: undefined,
    rc_connected: true,
    signal_strength: isConnected ? 'Good' : 'No Link',
  imu_status: telemetry.imu_status ?? 'ALIGNED', // prefer backend-supplied IMU status when available
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
  // Anchor selection separate from checkboxes (file-explorer style)
  const [anchorSelectedIds, setAnchorSelectedIds] = useState<number[]>([]);
  const [anchorLastClickedId, setAnchorLastClickedId] = useState<number | null>(null);

  const {
    missionLogs,
    getActiveLogEntries,
    clearLogs,
    addLogEntry,
  } = useMissionLogs();

  // When cleared, suppress live report metrics (distance, marked point, next point)
  const [isLiveReportCleared, setIsLiveReportCleared] = useState(false);
  // Home position state (can be set by user or auto-updated by rover armed/disarm events)
  const [homePosition, setHomePosition] = usePersistentState<{ lat: number; lng: number; alt?: number } | null>('app:homePosition', null);
  const [awaitingHomeClick, setAwaitingHomeClick] = useState(false);
  const firstArmedSeenRef = useRef<boolean>(false);

  // Table resize state
  const [tableHeight, setTableHeight] = usePersistentState<number>('app:tableHeight', 192); // Default 192px (h-48)
  const [isResizing, setIsResizing] = useState(false);
  const [activeDrawingTool, setActiveDrawingTool] = useState<string | null>(null);

  // Mission history for undo/redo
  const {
    canUndo,
    canRedo,
    undo,
    redo,
    pushSnapshot,
  // import SprayerTab from './components/SprayerTab';
    clearHistory,
    getHistorySize
  } = useMissionHistory(20);

  const { dialogState, showConfirm, showAlert } = useDialog();

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
      await showAlert('No Mission', 'No mission loaded. Add waypoints before uploading.');
      return false;
    }
    try {
      // Sanitize waypoints before upload (e.g., convert negative altitudes to positive)
      const sanitizedWaypoints = sanitizeWaypointsForUpload(missionWaypoints);
      // Confirmation: show counts and ask user to confirm upload
      const total = sanitizedWaypoints.length;
      const placeholders = sanitizedWaypoints.filter(wp => wp.showOnMap === false || (typeof wp.lat === 'number' && wp.lat === 0 && typeof wp.lng === 'number' && wp.lng === 0)).length;
      const confirmMsg = `Upload ${total} waypoint(s) to rover. ${placeholders} placeholder/command-only waypoint(s) detected. Continue?`;
      const confirmed = await showConfirm('Confirm Mission Upload', confirmMsg);
      if (!confirmed) {
        console.log('User cancelled mission upload');
        return false;
      }
      // Log the exact payload so developers can inspect what is being sent to the rover
      console.log('[MISSION UPLOAD] Payload prepared for upload:', sanitizedWaypoints);
      
      const response = await services.uploadMission(sanitizedWaypoints);
      if (response.success) {
        // Immediately attempt to read mission back from rover to verify what was written
        try {
          const verifyResponse = await services.downloadMission();
          const fetched = Array.isArray((verifyResponse as any).waypoints) ? (verifyResponse as any).waypoints : [];
          console.log('[MISSION VERIFY] Rover returned waypoints on download:', fetched);
          await showAlert('Upload Successful', `Mission upload requested (${missionWaypoints.length} waypoints). Rover reports ${fetched.length} waypoints after upload.`);
        } catch (verifyErr) {
          console.warn('Mission verification failed:', verifyErr);
          await showAlert('Upload Completed', `Mission upload requested (${missionWaypoints.length} waypoints). Verification failed to read mission from rover.`);
        }
        return true;
      }
      await showAlert('Upload Failed', response.message ?? 'Mission upload failed.');
      return false;
    } catch (error) {
      await showAlert('Upload Error', error instanceof Error ? error.message : 'Mission upload failed.');
      return false;
    }
  }, [missionWaypoints, services, showAlert]);

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
          await showAlert('Download Warning', `⚠️ ${response.message || `Downloaded ${validWaypoints.length} waypoints (with timeout warning)`}`);
        } else if (warningMsg === 'fallback') {
          await showAlert('Download Warning', `⚠️ ${response.message || `Using cached ${validWaypoints.length} waypoints`}`);
        } else if (validWaypoints.length < fetched.length) {
          await showAlert('Download Success', `✅ Downloaded ${validWaypoints.length} valid waypoints (${fetched.length - validWaypoints.length} invalid filtered out)`);
        } else {
          await showAlert('Download Success', `✅ Downloaded ${validWaypoints.length} waypoints from rover`);
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

  // Start interactive set-home flow: next map click will set home
  const startSetHome = useCallback(async () => {
    setAwaitingHomeClick(true);
    await showAlert('Set Home Position', 'Click on the map to set Home position.');
  }, [showAlert]);

  // Map click handler: if awaitingHomeClick, set home; otherwise noop
  const handleMapClickActual = useCallback(async (lat: number, lng: number) => {
    if (awaitingHomeClick) {
      const newHome = { lat, lng, alt: 0 };
      setHomePosition(newHome);
      setAwaitingHomeClick(false);
      await showAlert('Home Position Set', `Home set to ${lat.toFixed(6)}, ${lng.toFixed(6)}`);
      return;
    }
    // other map click behavior can go here
  }, [awaitingHomeClick, setHomePosition, showAlert]);

  // Telemetry-driven home update: set home on first arm after connect, and on disarm
  useEffect(() => {
    // track previous armed state
    const prevArmed = (window as any).__prevArmedState ?? null;
    const armedNow = telemetry.state?.armed ?? false;

    if (connectionState === 'connected' && (window as any).__prevConnectionState !== 'connected') {
      // just connected: reset first armed seen flag
      firstArmedSeenRef.current = false;
    }

    // Arm transition: first arm after connect sets home if not set yet
    if (armedNow && !firstArmedSeenRef.current) {
      if (uiRoverData.position) {
        setHomePosition({ lat: uiRoverData.position.lat, lng: uiRoverData.position.lng, alt: uiRoverData.altitude ?? 0 });
        firstArmedSeenRef.current = true;
        console.log('[APP] Home set from first arm at connect:', uiRoverData.position);
      }
    }

    // Disarm transition: when rover disarms, update home to disarm position
    if (!armedNow && prevArmed === true) {
      if (uiRoverData.position) {
        setHomePosition({ lat: uiRoverData.position.lat, lng: uiRoverData.position.lng, alt: uiRoverData.altitude ?? 0 });
        firstArmedSeenRef.current = false;
        console.log('[APP] Home set from disarm position:', uiRoverData.position);
      }
    }

    (window as any).__prevArmedState = armedNow;
    (window as any).__prevConnectionState = connectionState;
  }, [telemetry.state?.armed, connectionState, uiRoverData.position, uiRoverData.altitude, setHomePosition]);

  // Handle disconnection: clear live report and reset states
  useEffect(() => {
    const prevConnectionState = (window as any).__prevDisconnectState;
    
    // If we were connected before and now we're disconnected/error/connecting, clear live data
    if (prevConnectionState === 'connected' && connectionState !== 'connected') {
      console.log('[APP] Rover disconnected - clearing live data states');
      setIsLiveReportCleared(true);
    }
    
    // When reconnecting, reset the cleared flag to allow new data
    if (connectionState === 'connected' && prevConnectionState !== 'connected') {
      console.log('[APP] Rover reconnected - allowing new data');
      setIsLiveReportCleared(false);
    }
    
    (window as any).__prevDisconnectState = connectionState;
  }, [connectionState]);

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

  const handleExportMission = useCallback(async () => {
    if (missionWaypoints.length === 0) {
      await showAlert('Export Error', 'No mission to export.');
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

  const handleSelectAllWaypoints = useCallback((selectAll: boolean) => {
    if (!selectAll) {
      setSelectedWaypointIds([]);
      return;
    }
    setSelectedWaypointIds(missionWaypoints.map(wp => wp.id));
  }, [missionWaypoints, setSelectedWaypointIds]);

  const handleDeleteSelectedWaypoints = useCallback(async () => {
    if (!selectedWaypointIds.length) return;
    const confirmed = await showConfirm(
      'Delete Waypoints',
      `Delete ${selectedWaypointIds.length} selected waypoint(s)? This cannot be undone.`
    );
    if (!confirmed) return;
    setMissionWaypoints(prev => {
      const next = prev.filter(wp => !selectedWaypointIds.includes(wp.id));
      // Reindex ids to maintain 1..N sequence
      return calculateDistancesForMission(next.map((wp, idx) => ({ ...wp, id: idx + 1 })));
    });
    setSelectedWaypointIds([]);
  }, [selectedWaypointIds, setMissionWaypoints, setSelectedWaypointIds, showConfirm]);

  const handleAddEmptyWaypoint = useCallback((insertAfterId?: number | null) => {
    setMissionWaypoints(prev => {
      const newWp: Waypoint = {
        id: 0, // placeholder, we'll reindex
        // Create as a CONDITION_DELAY placeholder (delay next waypoint) as default
        command: 'CONDITION_DELAY',
        frame: 3,
        lat: 0,
        lng: 0,
        alt: 0,
        current: 0,
        autocontinue: 0,
        param1: 1, // default 1 second delay
        // Explicitly mark this as not shown on the map so MapView ignores it until coordinates assigned
        showOnMap: false,
      };
      const next = prev.slice();
      if (typeof insertAfterId === 'number') {
        const idx = next.findIndex(wp => wp.id === insertAfterId);
        const insertAt = idx >= 0 ? idx + 1 : next.length;
        // Copy coordinates from previous waypoint (the anchor) into the new waypoint
        if (idx >= 0) {
          const anchor = next[idx];
          if (anchor) {
            newWp.lat = anchor.lat ?? 0;
            newWp.lng = anchor.lng ?? 0;
            newWp.alt = anchor.alt ?? 0;
            // Show on map if anchor had valid coords
            newWp.showOnMap = !!(typeof anchor.lat === 'number' && typeof anchor.lng === 'number' && !(anchor.lat === 0 && anchor.lng === 0));
          }
        }
        next.splice(insertAt, 0, newWp);
      } else {
        // append: if there is at least one waypoint, copy last one's coords
        const last = next[next.length - 1];
        if (last) {
          newWp.lat = last.lat ?? 0;
          newWp.lng = last.lng ?? 0;
          newWp.alt = last.alt ?? 0;
          newWp.showOnMap = !!(typeof last.lat === 'number' && typeof last.lng === 'number' && !(last.lat === 0 && last.lng === 0));
        }
        next.push(newWp);
      }
      // Reindex to maintain sequential 1..N ids
      const reindexed = next.map((wp, i) => ({ ...wp, id: i + 1 }));
      return calculateDistancesForMission(reindexed);
    });
  }, [setMissionWaypoints]);

  const handleRowAnchorClick = useCallback((id: number, ctrlKey: boolean) => {
    setAnchorLastClickedId(id);
    setAnchorSelectedIds(prev => {
      if (ctrlKey) {
        // toggle
        if (prev.includes(id)) return prev.filter(x => x !== id);
        return [...prev, id];
      }
      return [id];
    });
  }, []);

  const handleDistanceChange = useCallback((id: number, distanceMeters: number) => {
    const idx = missionWaypoints.findIndex(wp => wp.id === id);
    if (idx <= 0) return; // first waypoint cannot be moved by distance
    const prevWp = missionWaypoints[idx - 1];
    const currWp = missionWaypoints[idx];
    if (!prevWp || !currWp) return;
    // Compute bearing from previous to current; if current has invalid coords, default bearing 0
    const bearing = (typeof prevWp.lat === 'number' && typeof prevWp.lng === 'number' && typeof currWp.lat === 'number' && typeof currWp.lng === 'number')
      ? calculateBearing({ lat: prevWp.lat, lng: prevWp.lng }, { lat: currWp.lat, lng: currWp.lng })
      : 0;
    const dest = calculateDestination({ lat: prevWp.lat, lng: prevWp.lng }, bearing, distanceMeters);
    handleUpdateWaypointPosition(id, { lat: dest.lat, lng: dest.lng });
  }, [missionWaypoints, handleUpdateWaypointPosition]);

  // Table resize handlers
  const handleResizeStart = useCallback((e: React.MouseEvent) => {
    setIsResizing(true);
    e.preventDefault();
  }, []);

  const handleResizeMove = useCallback((e: MouseEvent) => {
    if (!isResizing) return;
    
    const container = document.querySelector('[data-resize-container]');
    if (!container) return;
    
    const rect = container.getBoundingClientRect();
    const newHeight = rect.height - (e.clientY - rect.top);
    
    // Constrain height between 120px and 400px
    const constrainedHeight = Math.max(120, Math.min(400, newHeight));
    setTableHeight(constrainedHeight);
  }, [isResizing, setTableHeight]);

  const handleResizeEnd = useCallback(() => {
    setIsResizing(false);
  }, []);

  // Add global mouse event listeners for resize
  React.useEffect(() => {
    if (isResizing) {
      document.addEventListener('mousemove', handleResizeMove);
      document.addEventListener('mouseup', handleResizeEnd);
      return () => {
        document.removeEventListener('mousemove', handleResizeMove);
        document.removeEventListener('mouseup', handleResizeEnd);
      };
    }
  }, [isResizing, handleResizeMove, handleResizeEnd]);

  const commonMapProps = {
    missionWaypoints,
    onMapClick: handleMapClickActual,
    roverPosition: uiRoverData.position, // Ensure roverPosition is passed
    heading: uiRoverData.heading, // Pass heading data to MapView
    viewMode,
    isFullScreen,
    onNewMissionDrawn: handleNewMissionDrawn,
    isConnectedToRover,
    onUpdateWaypointPosition: handleUpdateWaypointPosition,
    activeDrawingTool,
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
      case 'planning':
        return (
          <main className="flex-1 flex p-3 gap-3 overflow-hidden min-h-0">
            <div className="flex-1 flex flex-col gap-3 min-h-0" data-resize-container>
              <div className="flex-1 min-h-0">
                <MapView {...commonMapProps} />
              </div>
              
              {/* Resize Handle */}
              <div 
                className="flex items-center justify-center py-1 cursor-row-resize hover:bg-slate-600 transition-colors"
                onMouseDown={handleResizeStart}
              >
                <div className="w-8 h-1 bg-slate-400 rounded-full"></div>
              </div>
              
              <div 
                className="overflow-hidden"
                style={{ height: `${tableHeight}px` }}
              >
                <QGCWaypointTable
                  waypoints={missionWaypoints}
                  onDelete={handleDeleteWaypoint}
                  onUpdate={handleUpdateWaypoint}
                  activeWaypointIndex={uiRoverData.activeWaypointIndex}
                  selectedWaypointIds={selectedWaypointIds}
                  onWaypointSelectionChange={handleWaypointSelectionChange}
                  onSelectAll={handleSelectAllWaypoints}
                  onDeleteSelected={handleDeleteSelectedWaypoints}
                  onAddWaypoint={handleAddEmptyWaypoint}
                  anchorSelectedIds={anchorSelectedIds}
                  anchorLastClickedId={anchorLastClickedId}
                  onRowAnchorClick={handleRowAnchorClick}
                  onDistanceChange={handleDistanceChange}
                  missionFileInfo={missionFileInfo}
                />
              </div>
            </div>
            <aside className="w-72 flex flex-col gap-[12.6px] min-h-0">
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
                homePosition={homePosition}
                onStartSetHome={startSetHome}
                activeDrawingTool={activeDrawingTool}
                onDrawingToolSelect={setActiveDrawingTool}
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
            hasMissionLogs={missionLogs.length > 0}
            hasWpMarkStatus={false} // TODO: Pass actual WP_MARK status from LiveReportView
            onClearAll={async () => {
              try {
                // Stop any running WP_MARK mission
                const { stopWPMarkMission } = await import('./services/wpMarkService');
                await stopWPMarkMission();
                console.log('[App] Stopped WP_MARK mission during clear');
              } catch (error) {
                console.error('[App] Failed to stop WP_MARK mission:', error);
              }

              // Clear logs
              clearLogs();

              // Clear waypoints
              setMissionWaypoints([]);

              // Clear live report metrics
              setIsLiveReportCleared(true);

              console.log('[App] Cleared all mission data');
            }}
          />
        );
      case 'sprayer':
        return (
          <SprayerReportView
            missionWaypoints={missionWaypoints}
            liveRoverData={uiRoverData}
            missionName={currentMissionFileName}
            isConnected={isConnectedToRover}
            onClearAll={async () => {
              // Sprayer-specific clear logic
              clearLogs();
              setMissionWaypoints([]);
              setIsLiveReportCleared(true);
              console.log('[App] Cleared all sprayer mission data');
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
                  onDownload={async () => {
                    const entries = getActiveLogEntries();
                    if (!entries.length) {
                      await showAlert('Download Error', 'No mission logs available to download yet.');
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
                  onClear={async () => {
                    if (missionLogs.length === 0) {
                      return;
                    }
                    const confirmClear = await showConfirm(
                      'Clear Mission Logs',
                      'This will clear the current mission logs. Continue?'
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

      {/* Generic Dialog for confirmations and alerts */}
      <GenericDialog
        isOpen={dialogState.isOpen}
        type={dialogState.type}
        title={dialogState.title}
        message={dialogState.message}
        onConfirm={dialogState.onConfirm}
        onCancel={dialogState.onCancel}
        confirmText={dialogState.confirmText}
        cancelText={dialogState.cancelText}
      />
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
