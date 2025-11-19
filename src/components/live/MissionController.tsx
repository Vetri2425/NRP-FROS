import React, { useState, useEffect, useCallback } from 'react';
import { useRover } from '../../context/RoverContext';
import { PlayIcon } from '../icons/PlayIcon';
import { PauseIcon } from '../icons/PauseIcon';
import { StopIcon } from '../icons/StopIcon';
import { NextIcon } from '../icons/NextIcon';
import { SettingsIcon } from '../icons/SettingsIcon';
import { useDialog } from '../../hooks/useDialog';
import GenericDialog from '../GenericDialog';
import { io, Socket } from 'socket.io-client';
import { BACKEND_URL } from '../../config';

export interface MissionConfig {
  servoNumber: number;
  pwmStart: number;
  pwmStop: number;
  sprayDuration: number;
  delayBeforeSpray: number;
  delayAfterSpray: number;
  mode: 'auto' | 'manual';
}

export interface MissionStatus {
  state: 'idle' | 'mission_loaded' | 'mission_running' | 'mission_paused' | 'mission_stopped' | 'mission_completed';
  currentWaypoint: number;
  totalWaypoints: number;
  eventType?: string;
  message?: string;
  timestamp?: string;
  sprayActive?: boolean;
  distanceToNext?: number;
}

type MissionControllerProps = {
  isConnected: boolean;
  waypoints: any[];
  onMissionLoad?: (waypoints: any[], config: MissionConfig) => void;
  onMissionStart?: () => void;
  onMissionStop?: () => void;
  onMissionPause?: () => void;
  onMissionResume?: () => void;
  onMissionNext?: () => void;
  missionStatus?: MissionStatus | null;
};

const MissionController: React.FC<MissionControllerProps> = ({
  isConnected,
  waypoints,
  onMissionLoad,
  onMissionStart,
  onMissionStop,
  onMissionPause,
  onMissionResume,
  onMissionNext,
  missionStatus: externalMissionStatus
}) => {
  const { services } = useRover();
  const [isLoading, setIsLoading] = useState(false);
  const [showConfig, setShowConfig] = useState(false);
  const [socket, setSocket] = useState<Socket | null>(null);
  const [internalMissionStatus, setInternalMissionStatus] = useState<MissionStatus | null>(null);
  const [config, setConfig] = useState<MissionConfig>({
    servoNumber: 9,
    pwmStart: 1900,
    pwmStop: 1100,
    sprayDuration: 5,
    delayBeforeSpray: 2,
    delayAfterSpray: 2,
    mode: 'auto'
  });

  const { dialogState, showConfirm, showAlert } = useDialog();

  // Socket.IO connection for mission controller
  useEffect(() => {
    if (!isConnected) {
      if (socket) {
        socket.disconnect();
        setSocket(null);
      }
      return;
    }

    // Connect to backend Socket.IO server
    const newSocket = io(BACKEND_URL, {
      transports: ['websocket', 'polling']
    });

    newSocket.on('connect', () => {
      console.log('[MissionController] Connected to backend');
    });

    newSocket.on('disconnect', () => {
      console.log('[MissionController] Disconnected from backend');
    });

    // Listen for mission status updates
    newSocket.on('mission_status', (data: MissionStatus) => {
      console.log('[MissionController] Mission status update:', data);
      setInternalMissionStatus(data);
    });

    // Listen for mission load responses
    newSocket.on('mission_load_response', (data: { success: boolean; message?: string }) => {
      console.log('[MissionController] Mission load response:', data);
      if (data.success) {
        setInternalMissionStatus(prev => prev ? { ...prev, state: 'mission_loaded' } : {
          state: 'mission_loaded',
          currentWaypoint: 0,
          totalWaypoints: waypoints.length
        });
      }
    });

    setSocket(newSocket);

    return () => {
      newSocket.disconnect();
    };
  }, [isConnected, waypoints.length]);

  // Use external mission status if provided, otherwise use internal
  const currentMissionStatus = externalMissionStatus || internalMissionStatus;

  // Use external mission status if provided, otherwise use internal
  const missionStatus = externalMissionStatus || internalMissionStatus;

  const handleLoadMission = useCallback(async () => {
    if (!isConnected || !socket || waypoints.length === 0) {
      await showAlert('Load Mission', 'No waypoints available, not connected, or socket not ready');
      return;
    }

    setIsLoading(true);
    try {
      // Send mission load command via Socket.IO
      socket.emit('mission_load', {
        waypoints: waypoints.map((wp, index) => ({
          id: index + 1,
          lat: wp.lat,
          lng: wp.lng,
          alt: wp.alt || 0,
          command: wp.command || 16, // NAV_WAYPOINT
          param1: wp.param1 || 0,
          param2: wp.param2 || 0,
          param3: wp.param3 || 0,
          param4: wp.param4 || 0
        })),
        config: config
      });

      onMissionLoad?.(waypoints, config);
    } catch (error) {
      await showAlert('Load Failed', error instanceof Error ? error.message : 'Failed to load mission');
    } finally {
      setIsLoading(false);
    }
  }, [isConnected, socket, waypoints, config, onMissionLoad, showAlert]);

  const handleStartMission = useCallback(async () => {
    if (!isConnected || !socket) {
      await showAlert('Start Mission', 'Not connected to rover or socket not ready');
      return;
    }

    if (currentMissionStatus?.state !== 'mission_loaded') {
      await showAlert('Start Mission', 'Please load a mission first');
      return;
    }

    const confirmed = await showConfirm(
      'Start Mission',
      `Start mission with ${waypoints.length} waypoints in ${config.mode} mode?`
    );
    if (!confirmed) return;

    setIsLoading(true);
    try {
      socket.emit('mission_start');
      onMissionStart?.();
    } catch (error) {
      await showAlert('Start Failed', error instanceof Error ? error.message : 'Failed to start mission');
    } finally {
      setIsLoading(false);
    }
  }, [isConnected, socket, currentMissionStatus, waypoints.length, config.mode, onMissionStart, showConfirm, showAlert]);

  const handleStopMission = useCallback(async () => {
    if (!isConnected || !socket) return;

    const confirmed = await showConfirm('Stop Mission', 'Are you sure you want to stop the mission?');
    if (!confirmed) return;

    setIsLoading(true);
    try {
      socket.emit('mission_stop');
      onMissionStop?.();
    } catch (error) {
      await showAlert('Stop Failed', error instanceof Error ? error.message : 'Failed to stop mission');
    } finally {
      setIsLoading(false);
    }
  }, [isConnected, socket, onMissionStop, showConfirm, showAlert]);

  const handlePauseResumeMission = useCallback(async () => {
    if (!isConnected || !socket) return;

    setIsLoading(true);
    try {
      if (currentMissionStatus?.state === 'mission_paused') {
        socket.emit('mission_resume');
        onMissionResume?.();
      } else {
        socket.emit('mission_pause');
        onMissionPause?.();
      }
    } catch (error) {
      await showAlert('Control Failed', error instanceof Error ? error.message : 'Failed to pause/resume mission');
    } finally {
      setIsLoading(false);
    }
  }, [isConnected, socket, currentMissionStatus?.state, onMissionResume, onMissionPause, showAlert]);

  const handleNextWaypoint = useCallback(async () => {
    if (!isConnected || !socket) return;

    setIsLoading(true);
    try {
      socket.emit('mission_next_sequence');
      onMissionNext?.();
    } catch (error) {
      await showAlert('Next Failed', error instanceof Error ? error.message : 'Failed to advance to next waypoint');
    } finally {
      setIsLoading(false);
    }
  }, [isConnected, socket, onMissionNext, showAlert]);

  const getStatusColor = (state: string) => {
    switch (state) {
      case 'idle': return 'text-gray-400';
      case 'mission_loaded': return 'text-blue-400';
      case 'mission_running': return 'text-green-400';
      case 'mission_paused': return 'text-yellow-400';
      case 'mission_stopped': return 'text-red-400';
      case 'mission_completed': return 'text-purple-400';
      default: return 'text-gray-400';
    }
  };

  const getStatusText = (state: string) => {
    switch (state) {
      case 'idle': return 'IDLE';
      case 'mission_loaded': return 'MISSION LOADED';
      case 'mission_running': return 'RUNNING';
      case 'mission_paused': return 'PAUSED';
      case 'mission_stopped': return 'STOPPED';
      case 'mission_completed': return 'COMPLETED';
      default: return 'UNKNOWN';
    }
  };

  const commonClass = 'w-full flex-1 text-white font-bold text-sm rounded-lg flex items-center justify-center gap-2 transition-colors disabled:opacity-50 disabled:cursor-not-allowed py-2';

  return (
    <div className="h-full flex flex-col gap-3 relative">
      {/* Mission Status */}
      <div className="bg-slate-800/50 border border-slate-700 rounded-lg p-3">
        <div className="text-center">
          <h3 className="text-sm font-bold text-slate-300 mb-1">Mission Status</h3>
          <p className={`text-lg font-mono font-bold ${getStatusColor(currentMissionStatus?.state || 'idle')}`}>
            {getStatusText(currentMissionStatus?.state || 'idle')}
          </p>
          {currentMissionStatus && (
            <p className="text-xs text-slate-400 mt-1">
              WP: {currentMissionStatus.currentWaypoint}/{currentMissionStatus.totalWaypoints}
              {currentMissionStatus.distanceToNext !== undefined && (
                <span className="ml-2">Dist: {currentMissionStatus.distanceToNext.toFixed(1)}m</span>
              )}
            </p>
          )}
          {currentMissionStatus?.sprayActive && (
            <p className="text-sm text-blue-400 font-bold mt-1">💧 SPRAYING ACTIVE</p>
          )}
        </div>
      </div>

      {/* Mission Configuration */}
      <div className="bg-slate-800/50 border border-slate-700 rounded-lg p-3">
        <button
          onClick={() => setShowConfig(!showConfig)}
          className="w-full flex items-center justify-center gap-2 text-slate-300 hover:text-white transition-colors"
        >
          <SettingsIcon className="w-4 h-4" />
          <span className="text-sm font-medium">Mission Config</span>
        </button>

        {showConfig && (
          <div className="mt-3 space-y-2">
            <div className="grid grid-cols-2 gap-2">
              <div>
                <label className="block text-xs text-slate-400 mb-1">Servo #</label>
                <input
                  type="number"
                  value={config.servoNumber}
                  onChange={(e) => setConfig(prev => ({ ...prev, servoNumber: parseInt(e.target.value) || 9 }))}
                  className="w-full bg-slate-700 border border-slate-600 rounded px-2 py-1 text-xs text-white"
                  min="1" max="16"
                />
              </div>
              <div>
                <label className="block text-xs text-slate-400 mb-1">Mode</label>
                <select
                  value={config.mode}
                  onChange={(e) => setConfig(prev => ({ ...prev, mode: e.target.value as 'auto' | 'manual' }))}
                  className="w-full bg-slate-700 border border-slate-600 rounded px-2 py-1 text-xs text-white"
                >
                  <option value="auto">Auto</option>
                  <option value="manual">Manual</option>
                </select>
              </div>
            </div>

            <div className="grid grid-cols-2 gap-2">
              <div>
                <label className="block text-xs text-slate-400 mb-1">PWM Start</label>
                <input
                  type="number"
                  value={config.pwmStart}
                  onChange={(e) => setConfig(prev => ({ ...prev, pwmStart: parseInt(e.target.value) || 1900 }))}
                  className="w-full bg-slate-700 border border-slate-600 rounded px-2 py-1 text-xs text-white"
                  min="100" max="2000"
                />
              </div>
              <div>
                <label className="block text-xs text-slate-400 mb-1">PWM Stop</label>
                <input
                  type="number"
                  value={config.pwmStop}
                  onChange={(e) => setConfig(prev => ({ ...prev, pwmStop: parseInt(e.target.value) || 1100 }))}
                  className="w-full bg-slate-700 border border-slate-600 rounded px-2 py-1 text-xs text-white"
                  min="100" max="2000"
                />
              </div>
            </div>

            <div className="grid grid-cols-3 gap-2">
              <div>
                <label className="block text-xs text-slate-400 mb-1">Spray (s)</label>
                <input
                  type="number"
                  value={config.sprayDuration}
                  onChange={(e) => setConfig(prev => ({ ...prev, sprayDuration: parseFloat(e.target.value) || 5 }))}
                  className="w-full bg-slate-700 border border-slate-600 rounded px-2 py-1 text-xs text-white"
                  min="0.1" step="0.1"
                />
              </div>
              <div>
                <label className="block text-xs text-slate-400 mb-1">Delay Before</label>
                <input
                  type="number"
                  value={config.delayBeforeSpray}
                  onChange={(e) => setConfig(prev => ({ ...prev, delayBeforeSpray: parseFloat(e.target.value) || 2 }))}
                  className="w-full bg-slate-700 border border-slate-600 rounded px-2 py-1 text-xs text-white"
                  min="0" step="0.1"
                />
              </div>
              <div>
                <label className="block text-xs text-slate-400 mb-1">Delay After</label>
                <input
                  type="number"
                  value={config.delayAfterSpray}
                  onChange={(e) => setConfig(prev => ({ ...prev, delayAfterSpray: parseFloat(e.target.value) || 2 }))}
                  className="w-full bg-slate-700 border border-slate-600 rounded px-2 py-1 text-xs text-white"
                  min="0" step="0.1"
                />
              </div>
            </div>
          </div>
        )}
      </div>

      {/* Mission Controls */}
      <div className="flex flex-col gap-2 flex-1">
        <button
          onClick={handleLoadMission}
          disabled={!isConnected || isLoading || waypoints.length === 0}
          className={`${commonClass} bg-blue-600 hover:bg-blue-700`}
          title="Load mission with current waypoints and configuration"
        >
          <SettingsIcon className="w-4 h-4" />
          LOAD MISSION
        </button>

        <button
          onClick={handleStartMission}
          disabled={!isConnected || isLoading || currentMissionStatus?.state !== 'mission_loaded'}
          className={`${commonClass} bg-green-600 hover:bg-green-700`}
          title="Start the loaded mission"
        >
          <PlayIcon className="w-4 h-4" />
          START
        </button>

        <button
          onClick={handlePauseResumeMission}
          disabled={!isConnected || isLoading || !['mission_running', 'mission_paused'].includes(currentMissionStatus?.state || '')}
          className={`${commonClass} ${currentMissionStatus?.state === 'mission_paused' ? 'bg-green-600 hover:bg-green-700' : 'bg-yellow-600 hover:bg-yellow-700'}`}
          title={currentMissionStatus?.state === 'mission_paused' ? 'Resume mission' : 'Pause mission'}
        >
          {currentMissionStatus?.state === 'mission_paused' ? (
            <><PlayIcon className="w-4 h-4" /> RESUME</>
          ) : (
            <><PauseIcon className="w-4 h-4" /> PAUSE</>
          )}
        </button>

        <button
          onClick={handleStopMission}
          disabled={!isConnected || isLoading || !['mission_running', 'mission_paused'].includes(currentMissionStatus?.state || '')}
          className={`${commonClass} bg-red-600 hover:bg-red-700`}
          title="Stop the mission"
        >
          <StopIcon className="w-4 h-4" />
          STOP
        </button>

        {config.mode === 'manual' && currentMissionStatus?.state === 'mission_running' && (
          <button
            onClick={handleNextWaypoint}
            disabled={!isConnected || isLoading}
            className={`${commonClass} bg-purple-600 hover:bg-purple-700`}
            title="Advance to next waypoint (manual mode)"
          >
            <NextIcon className="w-4 h-4" />
            NEXT WAYPOINT
          </button>
        )}
      </div>

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

export default MissionController;