import React, { useState, useEffect, useRef } from 'react';
import { SocketProvider } from './context/SocketContext';
import { RtkProvider } from './context/RtkContext';
import Header from './components/Header';
import LeftSidebar from './components/LeftSidebar';
import MapView from './components/MapView';
import PlanControls from './components/plan/PlanControls';
import QGCWaypointTable from './components/plan/QGCWaypointTable';
import LiveReportView from './components/live/LiveReportView';
import MissionLogs from './components/MissionLogs';
import { BackendLogEntry, MissionFileInfo, Waypoint, ViewMode } from './types';
import type { ServoConfig } from './types/servo';
import { toQGCWPL110 } from './utils/missionParser';
import { useRoverConnection, RoverData } from './hooks/useRoverConnection';
import { useMissionLogs } from './hooks/useMissionLogs';
import { calculateDistancesForMission } from './utils/geo';
import ConnectionError from './components/ConnectionError';
import CommandErrorModal from './components/CommandErrorModal';
import SetupTab from './components/setup/SetupTab';
import RTKInjectorPanel from "./components/setup/RTKInjectorPanel"; // add import for rtk injector panel
import { usePersistentState } from './hooks/usePersistentState';
import ServoControlTab from './components/ServoControl/ServoControlTab';

type CommandError = {
  title: string;
  message: string;
  causes: string[];
};

const App: React.FC = () => {
  const [viewMode, setViewMode] = usePersistentState<ViewMode>('app:viewMode', 'dashboard');
  const [missionWaypoints, setMissionWaypoints] = usePersistentState<Waypoint[]>('app:missionWaypoints', []);
  const [isFullScreen, setIsFullScreen] = useState(false);
  const uploadInitiatedRef = useRef(false);
  const [currentMissionFileName, setCurrentMissionFileName] = usePersistentState<string | null>('app:missionFileName', null);
  const [missionFileInfo, setMissionFileInfo] = usePersistentState<MissionFileInfo | null>('app:missionFileInfo', null);
  const [commandError, setCommandError] = useState<CommandError | null>(null);
  const [selectedWaypointIds, setSelectedWaypointIds] = usePersistentState<number[]>('app:selectedWaypoints', []);
  const [rtkToast, setRtkToast] = useState<string | null>(null);
  // Servo tab state (legacy config support retained for future use)
  const [savedServoConfig, setSavedServoConfig] = usePersistentState<ServoConfig | null>('app:servoConfig', null);

  const {
    missionLogs,
    createNewLog,
    addLogEntry,
    updateActiveLogStatus,
    getActiveLogEntries,
    clearLogs,
  } = useMissionLogs();

  const seenLogKeysRef = useRef<Set<string>>(new Set());
  const roverPositionRef = useRef<{ lat: number; lng: number } | null>(null);
  const activeLogPresentRef = useRef<boolean>(false);

  useEffect(() => {
    const activeLog = missionLogs.find(log => log.status === 'In Progress');
    activeLogPresentRef.current = !!activeLog;
    if (activeLog) {
      const existingKeys = new Set<string>();
      activeLog.entries.forEach(entry => {
        const key = `${entry.timestamp}|${entry.event}|${entry.status ?? ''}|${entry.servoAction ?? ''}|${entry.waypointId ?? ''}`;
        existingKeys.add(key);
      });
      seenLogKeysRef.current = existingKeys;
    } else {
      seenLogKeysRef.current = new Set();
    }
  }, [missionLogs]);

  const ensureActiveLog = React.useCallback(() => {
    if (!activeLogPresentRef.current) {
      activeLogPresentRef.current = true;
      seenLogKeysRef.current = new Set();
      createNewLog(`Live Session ${new Date().toLocaleTimeString()}`);
    }
  }, [createNewLog]);

  const handleMissionEvent = React.useCallback((entry: BackendLogEntry) => {
    ensureActiveLog();
    const message = entry.message?.toString().trim();
    if (!message) {
      return;
    }
    const key = `${entry.timestamp}|${message}|${entry.status ?? ''}|${entry.servoAction ?? ''}|${entry.waypointId ?? ''}`;
    if (seenLogKeysRef.current.has(key)) {
      return;
    }
    seenLogKeysRef.current.add(key);

    const position = roverPositionRef.current;
    const lat = typeof entry.lat === 'number' ? entry.lat : position?.lat ?? null;
    const lng = typeof entry.lng === 'number' ? entry.lng : position?.lng ?? null;

    addLogEntry({
      lat,
      lng,
      event: message,
      timestamp: entry.timestamp,
      waypointId: entry.waypointId ?? null,
      status: entry.status ?? null,
      servoAction: entry.servoAction ?? null,
    });
  }, [ensureActiveLog, addLogEntry]);

  const handleMissionLogSnapshot = React.useCallback((entries: BackendLogEntry[]) => {
    if (!entries.length) {
      return;
    }
    entries.forEach(entry => handleMissionEvent(entry));
  }, [handleMissionEvent]);

  const handleDownloadLogs = React.useCallback(() => {
    const entries = getActiveLogEntries();
    if (!entries.length) {
      alert('No mission logs available to download yet.');
      return;
    }

    const header = ['Timestamp', 'Latitude', 'Longitude', 'Waypoint', 'Status', 'Servo Action', 'Remark'];
    const rows = entries.map(entry => {
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
  }, [getActiveLogEntries]);

  const handleClearLogs = React.useCallback(() => {
    if (missionLogs.length === 0) {
      return;
    }
    const confirmClear = window.confirm('This will clear the current mission logs. Continue?');
    if (!confirmClear) {
      return;
    }
    clearLogs();
    seenLogKeysRef.current = new Set();
    activeLogPresentRef.current = false;
  }, [missionLogs.length, clearLogs]);
  
  const {
    connectionStatus,
    backendStatus,
    roverData,
    connect,
    disconnect,
    requestRoverReconnect,
    sendCommand,
    addCommandResponseListener,
    writeMissionToRover,
    readMissionFromRover,
    uploadProgress,
  } = useRoverConnection({
    onMissionEvent: handleMissionEvent,
    onMissionLogsSnapshot: handleMissionLogSnapshot,
  });

  useEffect(() => {
    if (connectionStatus === 'CONNECTED_TO_ROVER' && addCommandResponseListener) {
      const removeListener = addCommandResponseListener((response) => {
        if (response.status === 'pending') {
          return;
        }
        if (response.status === 'error') {
          let causes = ['The vehicle is not in a state to accept this command.', 'Check vehicle logs for more details.'];
          if (response.message.includes("is not armable")) {
              causes = ["Vehicle's pre-arm checks failed (e.g., GPS lock, IMU calibration).", "Safety switch on the vehicle may not be pressed.", "Review the vehicle's pre-arm check list."];
          } else if (response.message.includes("Mode")) {
              causes = ["The vehicle cannot switch to this mode in its current state.", "The requested mode may not be available for this vehicle type."];
          }
          setCommandError({ title: 'Command Rejected by Vehicle', message: response.message, causes });
        }
      });
      return () => removeListener();
    }
  }, [connectionStatus, addCommandResponseListener]);

  // Global small popup when RTK streaming stops
  useEffect(() => {
    const handler = (e: Event) => {
      const detail = (e as CustomEvent<{ status: string; message: string }>).detail;
      if (!detail) return;
      setRtkToast(`🛑 RTK streaming stopped${detail.message ? `: ${detail.message}` : ''}`);
      const t = setTimeout(() => setRtkToast(null), 3000);
      return () => clearTimeout(t);
    };
    window.addEventListener('rtk_stream_status', handler as EventListener);
    return () => window.removeEventListener('rtk_stream_status', handler as EventListener);
  }, []);

  const handleArmDisarm = () => {
    if (roverData.status === 'armed') {
      sendCommand({ command: 'ARM_DISARM', arm: false });
      return;
    }
    if (!isConnectedToRover) {
      setCommandError({ title: 'Rover Not Connected', message: 'A stable connection to the rover is required before arming.', causes: ['Click the "CONNECT" button.', 'Verify the Jetson backend is running.', 'Check Wi-Fi and network settings.'], });
      return;
    }
    const { mode } = roverData;
    if ((mode === 'AUTO' || mode === 'GUIDED') && missionWaypoints.length === 0) {
      setCommandError({ title: 'Mission Not Loaded', message: 'A mission file must be loaded to arm in AUTO or GUIDED mode.', causes: ['Go to the "Edit Plan" tab and upload a valid mission file.'], });
      return;
    }
    if (mode === 'MANUAL' && !roverData.rc_connected) {
      setCommandError({ title: 'RC Transmitter Not Detected', message: 'A remote control signal is required to arm in MANUAL mode.', causes: ['Turn on your RC transmitter.', 'Ensure the RC receiver is properly connected and bound to the transmitter.'], });
      return;
    }
    sendCommand({ command: 'ARM_DISARM', arm: true });
  };

  const handleWaypointSelectionChange = (id: number, isSelected: boolean) => {
    setSelectedWaypointIds(prev => isSelected ? [...prev, id] : prev.filter(selectedId => selectedId !== id));
  };

  const handleSelectAll = () => {
    if (selectedWaypointIds.length === missionWaypoints.length) {
      setSelectedWaypointIds([]);
    } else {
      setSelectedWaypointIds(missionWaypoints.map(wp => wp.id));
    }
  };

  // Removed servo command application helper (no longer used in Plan tab)

  useEffect(() => {
    if (connectionStatus === 'DISCONNECTED') {
      activeLogPresentRef.current = false;
      updateActiveLogStatus('Incomplete');
    }
  }, [connectionStatus, updateActiveLogStatus]);

  const handleWriteToRover = async (): Promise<boolean> => {
    if (missionWaypoints.length === 0) {
      setCommandError({
        title: 'No Mission to Upload',
        message: 'Please load a mission before writing to rover.',
        causes: ['Upload a mission file or draw waypoints on the map.']
      });
      return false;
    }

    if (connectionStatus !== 'CONNECTED_TO_ROVER') {
      setCommandError({
        title: 'Rover Not Connected',
        message: 'Cannot upload mission - rover is not connected.',
        causes: ['Click CONNECT button and wait for rover connection.']
      });
      return false;
    }

    try {
      const ok = await writeMissionToRover(missionWaypoints);

      if (ok) {
        alert(`Mission uploaded to rover successfully! ${missionWaypoints.length} waypoints sent.`);
        return true;
      } else {
        setCommandError({
          title: 'Mission Upload Failed',
          message: 'Failed to upload mission to rover.',
          causes: ['Check rover connection.', 'Verify mission data is valid.', 'Try again.']
        });
        return false;
      }
    } catch (error) {
      setCommandError({
        title: 'Upload Error',
        message: 'An error occurred while uploading mission.',
        causes: ['Check console for details.', 'Try reconnecting to rover.']
      });
      return false;
    }
  };

  const handleReadFromRover = async () => {
    if (connectionStatus !== 'CONNECTED_TO_ROVER') {
      setCommandError({
        title: 'Rover Not Connected',
        message: 'Cannot download mission - rover is not connected.',
        causes: ['Click CONNECT button and wait for rover connection.']
      });
      return;
    }

    try {
      const result = await readMissionFromRover();
      
      if (result.status === 'success' && result.waypoints && result.waypoints.length > 0) {
        setMissionWaypoints(calculateDistancesForMission(result.waypoints));
        setCurrentMissionFileName(`Mission from Rover - ${new Date().toLocaleTimeString()}`);
        
        alert(`Downloaded ${result.waypoints.length} waypoints from rover!`);
      } else if (result.status === 'success' && (!result.waypoints || result.waypoints.length === 0)) {
        alert('No mission found on rover.');
      } else {
        setCommandError({
          title: 'Mission Download Failed',
          message: result.message || 'Failed to download mission from rover.',
          causes: ['Check if rover has a mission loaded.', 'Verify rover connection.', 'Try again.']
        });
      }
    } catch (error) {
      setCommandError({
        title: 'Download Error',
        message: 'An error occurred while downloading mission.',
        causes: ['Check console for details.', 'Try reconnecting to rover.']
      });
    }
  };

  const handleToggleFullScreen = () => { 
    if (!document.fullscreenElement) { 
      document.documentElement.requestFullscreen(); 
    } else if (document.exitFullscreen) { 
      document.exitFullscreen(); 
    } 
  };
  
  const handleToggleConnection = () => {
    if (backendStatus !== 'ONLINE') {
      connect();
      return;
    }

    requestRoverReconnect();
  };
  
  const handleChangeMode = (mode: string) => sendCommand({ command: 'SET_MODE', mode: mode });
  const handleMapClick = (lat: number, lng: number) => { /* Map click logging disabled */ };
  const handleUploadInitiated = () => { if (document.fullscreenElement) uploadInitiatedRef.current = true; };
  const handleMissionUpload = (waypoints: Waypoint[], info: MissionFileInfo) => { 
    setMissionWaypoints(calculateDistancesForMission(waypoints)); 
    setCurrentMissionFileName(info.name); 
    setMissionFileInfo(info);
  };
  const handleClearMission = () => { 
    setMissionWaypoints([]); 
    setCurrentMissionFileName(null); 
    setMissionFileInfo(null);
  };
  const handleDeleteWaypoint = (id: number) => { 
    setMissionWaypoints(prev => calculateDistancesForMission(prev.filter(wp => wp.id !== id))); 
    setSelectedWaypointIds(prev => prev.filter(sid => sid !== id)); 
  };
  const handleDeleteSelected = () => {
    if (selectedWaypointIds.length === 0) {
      alert('No waypoints selected.');
      return;
    }
    setMissionWaypoints(prev => {
      const filtered = prev.filter(wp => !selectedWaypointIds.includes(wp.id));
      const reindexed = filtered.map((wp, idx) => ({ ...wp, id: idx + 1 }));
      return calculateDistancesForMission(reindexed);
    });
    setSelectedWaypointIds([]);
  };
  const handleUpdateWaypoint = (id: number, newValues: Partial<Omit<Waypoint, 'id'>>) => {
    setMissionWaypoints(prev => prev.map(wp => (wp.id === id ? { ...wp, ...newValues } : wp)));
  };
  const handleUpdateWaypointPosition = (waypointId: number, newPosition: { lat: number, lng: number }) => { 
    setMissionWaypoints(prev => calculateDistancesForMission(prev.map(wp => wp.id === waypointId ? { ...wp, ...newPosition } : wp))); 
  };
  const handleNewMissionDrawn = (points: { lat: number, lng: number }[]) => { 
    const newWaypoints: Waypoint[] = points.map((p, index) => ({ 
      id: index + 1, 
      command: 'WAYPOINT', 
      lat: p.lat, 
      lng: p.lng, 
      alt: 50, 
      frame: 3,
      current: index === 0 ? 1 : 0,
      autocontinue: 1
    })); 
    const name = `Drawn Mission - ${new Date().toLocaleTimeString()}`;
    handleMissionUpload(newWaypoints, {
      name,
      size: 0,
      type: 'drawn',
      uploadedAt: new Date().toISOString(),
      waypointCount: newWaypoints.length,
      source: 'drawn',
    }); 
  };
  const handleExportMission = () => { 
    if (missionWaypoints.length === 0) return alert("No mission to export."); 
    const blob = new Blob([toQGCWPL110(missionWaypoints)], { type: 'text/plain;charset=utf-8' }); 
    const link = document.createElement('a'); 
    link.href = URL.createObjectURL(blob); 
    link.download = 'mission.waypoints'; 
    link.click(); 
  };

  // Apply servo config to current mission (for Servo tab)
  const applyServoConfigToMission = (cfg: ServoConfig) => {
    if (!missionWaypoints.length) {
      alert('No mission loaded.');
      return;
    }

    const original = [...missionWaypoints];
    const updated: Waypoint[] = [];

    const makeServo = (servoNumber: number, pwm: number, action: string): Waypoint => ({
      id: 0,
      command: 'DO_SET_SERVO',
      frame: 0,
      current: 0,
      autocontinue: 1,
      param1: servoNumber,
      param2: pwm,
      param3: 0,
      param4: 0,
      lat: 0,
      lng: 0,
      alt: 0,
      action,
    });

    const haversine = (a: Waypoint, b: Waypoint) => {
      const R = 6371000; // meters
      const dLat = (b.lat - a.lat) * Math.PI / 180;
      const dLng = (b.lng - a.lng) * Math.PI / 180;
      const la1 = a.lat * Math.PI / 180;
      const la2 = b.lat * Math.PI / 180;
      const s = Math.sin(dLat/2) ** 2 + Math.cos(la1) * Math.cos(la2) * Math.sin(dLng/2) ** 2;
      return 2 * R * Math.atan2(Math.sqrt(s), Math.sqrt(1 - s));
    };

    if (cfg.mode === 'MARK_AT_WAYPOINT') {
      const { servoNumber, pwmOn, pwmOff, sprayDuration, selectedIds } = cfg;
      const selected = new Set((selectedIds ?? []).map(Number));
      let marked = 0;
      for (let i = 0; i < original.length; i++) {
        const wp = original[i];
        updated.push(wp);
        if (selected.has(wp.id) && wp.command === 'WAYPOINT') {
          marked++;
          updated.push(makeServo(servoNumber, pwmOn, `Spray ON @WP${wp.id}`));
          updated.push({
            id: 0,
            command: 'CONDITION_DELAY',
            frame: 0,
            current: 0,
            autocontinue: 1,
            param1: typeof sprayDuration === 'number' ? sprayDuration : 0.5,
            param2: 0, param3: 0, param4: 0,
            lat: 0, lng: 0, alt: 0,
            action: 'Delay',
          });
          updated.push(makeServo(servoNumber, pwmOff, `Spray OFF @WP${wp.id}`));
        }
      }
      alert(`✅ Mode: Mark at Waypoint\nMarked ${marked} waypoints\nTotal commands: ${updated.length}`);
    } else if (cfg.mode === 'CONTINUOUS_LINE') {
      const { servoNumber, pwmOn, pwmOff, selectedIds } = cfg;
      if (!selectedIds || selectedIds.length < 2) {
        alert('Select at least two waypoints to define the line.');
        return;
      }
      const startId = Math.min(...selectedIds);
      const endId = Math.max(...selectedIds);
      let onAdded = false, offAdded = false;
      for (let i = 0; i < original.length; i++) {
        const wp = original[i];
        updated.push(wp);
        if (wp.id === startId && !onAdded && wp.command === 'WAYPOINT') {
          updated.push(makeServo(servoNumber, pwmOn, `Spray ON @WP${startId}`));
          onAdded = true;
        }
        if (wp.id === endId && !offAdded && wp.command === 'WAYPOINT') {
          updated.push(makeServo(servoNumber, pwmOff, `Spray OFF @WP${endId}`));
          offAdded = true;
        }
      }
    } else if (cfg.mode === 'INTERVAL_SPRAY') {
      const { servoNumber, pwmOn, pwmOff, startWp, endWp, distanceOnMeters, distanceOffMeters } = cfg;
      const startId = Math.min(startWp, endWp);
      const endId = Math.max(startWp, endWp);
      let prev: Waypoint | null = null;
      let state: 'OFF' | 'ON' = 'OFF';
      let remaining = distanceOffMeters > 0 ? distanceOffMeters : 1;
      for (let i = 0; i < original.length; i++) {
        const wp = original[i];
        updated.push(wp);
        if (wp.id < startId || wp.id > endId || wp.command !== 'WAYPOINT') {
          prev = wp;
          continue;
        }
        if (prev && prev.command === 'WAYPOINT') {
          let d = haversine(prev, wp);
          while (d >= remaining) {
            const used = remaining; // distance to reach the toggle point
            // Toggle state at this waypoint
            if (state === 'OFF') {
              updated.push(makeServo(servoNumber, pwmOn, `Spray ON @WP${wp.id}`));
              state = 'ON';
              remaining = distanceOnMeters > 0 ? distanceOnMeters : 1;
            } else {
              updated.push(makeServo(servoNumber, pwmOff, `Spray OFF @WP${wp.id}`));
              state = 'OFF';
              remaining = distanceOffMeters > 0 ? distanceOffMeters : 1;
            }
            d -= used; // continue with leftover distance in this segment
          }
          // consume leftover segment distance
          remaining -= Math.min(remaining, d);
        }
        prev = wp;
      }
      // Ensure we end OFF at the final boundary
      if (state === 'ON') {
        const last = original.find((w) => w.id === endId);
        if (last) updated.push(makeServo(servoNumber, pwmOff, `Spray OFF @WP${last.id}`));
      }
    }

    // Reindex and commit
    updated.forEach((w, idx) => (w.id = idx + 1));
    handleMissionUpload(updated, {
      name: 'Servo Action Mission',
      size: 0,
      type: 'generated',
      uploadedAt: new Date().toISOString(),
      waypointCount: updated.length,
      source: 'generated',
    });
    alert('✅ Mission updated in QGC Table!');
  };

  const handleServoConfigChange = (cfg: ServoConfig, action?: 'upload' | 'select') => {
    setSavedServoConfig(cfg);
    if (action === 'upload') {
      applyServoConfigToMission(cfg);
      return;
    }
    if (action === 'select') {
      alert('Switching to Edit Plan to select waypoints. Come back to Servo Control to upload.');
      setViewMode('planning');
      return;
    }
  };

  const isConnectedToRover = connectionStatus === 'CONNECTED_TO_ROVER';

  const uiRoverData: RoverData = React.useMemo(() => {
    const disconnectedData: RoverData = {
      position: null,
      heading: 0,
      battery: -1,
      status: 'disarmed',
      mode: 'UNKNOWN',
      rtk_status: 'N/A',
      signal_strength: 'N/A',
      current_waypoint_id: null,
      rc_connected: false,
      hrms: '0.000',
      vrms: '0.000',
      imu_status: 'UNALIGNED',
      activeWaypointIndex: null,
      completedWaypointIds: [],
      distanceToNext: 0,
    };

    if (isConnectedToRover) {
      return {
        ...disconnectedData,
        ...roverData,
        activeWaypointIndex: roverData.current_waypoint_id,
      };
    }

    return disconnectedData;
  }, [isConnectedToRover, roverData]);

  useEffect(() => {
    roverPositionRef.current = uiRoverData.position;
  }, [uiRoverData.position]);


  const commonMapProps = {
    missionWaypoints, 
    onMapClick: handleMapClick, 
    roverPosition: uiRoverData.position,
    activeWaypointIndex: uiRoverData.activeWaypointIndex, 
    heading: uiRoverData.heading,
    viewMode, 
    isFullScreen, 
    onNewMissionDrawn: handleNewMissionDrawn, 
    isConnectedToRover,
    onUpdateWaypointPosition: handleUpdateWaypointPosition
  };
  
  const commonSidebarProps = {
    onMissionUpload: handleMissionUpload, 
    onUploadInitiated: handleUploadInitiated, 
    onClearMission: handleClearMission,
    roverData: uiRoverData, 
    isConnected: isConnectedToRover, 
    onChangeMode: handleChangeMode, 
    onArmDisarm: handleArmDisarm,
    missionLogs,
    missionFileInfo
  };

  const renderMainContent = () => {
    switch (viewMode) {
      case 'servo':
        return (
          <main className="flex-1 flex p-4 overflow-hidden">
            <ServoControlTab />
          </main>
        );
      case 'planning':
        return (
          <main className="flex-1 flex p-4 gap-4 overflow-hidden relative">
            <div className="flex-1 flex flex-col gap-4">
              <MapView {...commonMapProps} />
              <div className="flex-[0_0_240px] overflow-hidden">
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
            <aside className="w-1/4 max-w-xs grid grid-rows-2 gap-4">
              <PlanControls
                  missionWaypoints={missionWaypoints}
                  onUpload={handleMissionUpload}
                  onExport={handleExportMission}
                  onUploadInitiated={handleUploadInitiated}
                  onWriteToRover={handleWriteToRover}
                  onReadFromRover={handleReadFromRover}
                  missionFileInfo={missionFileInfo}
                  onClearMission={handleClearMission}
                  isConnected={isConnectedToRover}
                  uploadProgress={uploadProgress}
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
          />
        );
      
      case 'setup':
        return (
          <main className="flex-1 flex p-4 overflow-hidden">
            <SetupTab />
          </main>
        );

      case 'dashboard': 
      default: 
        return ( 
          <main className="flex-1 flex p-4 gap-4 overflow-hidden"> 
            <LeftSidebar {...commonSidebarProps} /> 
            <div className="flex-1 flex flex-col gap-4"> 
              <MapView {...commonMapProps} /> 
              <div className="flex-[0_0_240px] overflow-hidden"> 
                <MissionLogs 
                  logEntries={getActiveLogEntries()} 
                  onDownload={handleDownloadLogs}
                  onClear={handleClearLogs}
                /> 
              </div> 
            </div> 
          </main> 
        );
    }
  };

  return (
    <SocketProvider>
      <RtkProvider>
        <>
          <div className={`text-white min-h-screen flex flex-col font-sans bg-slate-800 transition-all duration-300 ${commandError || connectionStatus === 'ERROR' ? 'blur-sm pointer-events-none' : ''}`}>
            <Header
              viewMode={viewMode} 
              setViewMode={setViewMode} 
              isFullScreen={isFullScreen}
              onToggleFullScreen={handleToggleFullScreen} 
              connectionStatus={connectionStatus} 
              backendStatus={backendStatus}
              onToggleConnection={handleToggleConnection}
            />
            {renderMainContent()}
            {rtkToast && (
              <div className="fixed bottom-4 right-4 bg-slate-800 border border-slate-700 text-white rounded px-4 py-2 shadow-lg z-50">
                {rtkToast}
              </div>
            )}
          </div>
          
          {connectionStatus === 'ERROR' && ( 
            <ConnectionError 
              onRetry={connect} 
              onClose={disconnect} 
              failedIp={import.meta.env.VITE_JETSON_BACKEND_URL || 'UNKNOWN IP'} 
            /> 
          )}
          
          <CommandErrorModal 
            isOpen={!!commandError} 
            onClose={() => setCommandError(null)} 
            errorInfo={commandError} 
          />
        </>
      </RtkProvider>
    </SocketProvider>
  );
};

export default App;
