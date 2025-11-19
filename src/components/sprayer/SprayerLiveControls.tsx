import React from 'react';
import { toast } from 'react-toastify';
import { useRover } from '../../context/RoverContext';
import { BACKEND_URL } from '../../config';
import { Waypoint } from '../../types';

export type SprayerLiveControlsProps = {
  isConnected: boolean;
  onStart?: () => Promise<void>;
  onStop?: () => Promise<void>;
  onPause?: () => void;
  onResume?: () => void;
  onNext?: () => void;
  onLoadMission?: () => Promise<void>;
  onRestart?: () => void;  // optional
  waypoints?: Waypoint[]; // Add waypoints prop
};

const SprayerControls: React.FC<SprayerLiveControlsProps> = ({
  isConnected,
  onStart,
  onStop,
  onPause,
  onResume,
  onNext,
  onLoadMission,
  waypoints = [], // Default to empty array
}) => {
  const { services } = useRover();
  const [mode, setMode] = React.useState<'auto' | 'manual'>('auto');
  const [isTogglingMode, setIsTogglingMode] = React.useState(false);
  const [isRunning, setIsRunning] = React.useState(false);
  const [isPaused, setIsPaused] = React.useState(false);
  const [isLoadingMission, setIsLoadingMission] = React.useState(false);
  const [isStartingMission, setIsStartingMission] = React.useState(false);
  const [isStoppingMission, setIsStoppingMission] = React.useState(false);
  // Waypoint preview dialog state
  const [showWaypointDialog, setShowWaypointDialog] = React.useState(false);

  // mode stays local (no backend config UI)

  const handleLoadMission = async () => {
    if (waypoints.length === 0) {
      toast.warning('No waypoints available to load');
      return;
    }

    // Show waypoint preview dialog
    setShowWaypointDialog(true);
  };

  // Seed current mode from backend on mount
  React.useEffect(() => {
    let mounted = true;
    (async () => {
      try {
        const res = await fetch(`${BACKEND_URL.replace(/\/$/, '')}/api/mission/mode`);
        if (!mounted) return;
        if (!res.ok) return;
        const body = await res.json().catch(() => null);
        if (body && (body.mode === 'auto' || body.mode === 'manual')) {
          setMode(body.mode);
        } else if (typeof body === 'string' && (body === 'auto' || body === 'manual')) {
          setMode(body as 'auto' | 'manual');
        }
      } catch (err) {
        // ignore - keep default
        console.debug('Could not fetch current mission mode:', err);
      }
    })();
    return () => { mounted = false; };
  }, []);

  const handleConfirmUpload = async () => {
    setShowWaypointDialog(false);
    setIsLoadingMission(true);

    try {
      // Upload only waypoints to mission controller (frontend-only load)
      // Remove any autocontinue field so backend determines behavior based on global mode
      const sanitized = waypoints.map(({ autocontinue, ...rest }) => ({ ...rest }));
      const response = await services.loadMissionToController(sanitized as any);

      if (response.success) {
        toast.success(`Mission loaded successfully! (${waypoints.length} waypoints)`);
      } else {
        toast.error(response.message || 'Failed to load mission');
      }
    } catch (error) {
      console.error('Load mission failed:', error);
      toast.error('Failed to load mission');
    } finally {
      setIsLoadingMission(false);
    }
  };

  const handleStartMission = async () => {
    if (!onStart) return;
    
    setIsStartingMission(true);
    try {
      await onStart();
      setIsRunning(true);
    } catch (error) {
      console.error('Start mission failed:', error);
    } finally {
      setIsStartingMission(false);
    }
  };

  const handleStopMission = async () => {
    if (!onStop) return;
    
    setIsStoppingMission(true);
    try {
      await onStop();
      setIsRunning(false);
      setIsPaused(false);
    } catch (error) {
      console.error('Stop mission failed:', error);
    } finally {
      setIsStoppingMission(false);
    }
  };

  // Confirmation dialog state
  const [confirmAction, setConfirmAction] = React.useState<null | { action: string, onConfirm: () => void }>(null);

  // Helper to check connection and show error
  const requireConnection = (cb: () => void) => {
    if (!isConnected) {
      toast.error('Not connected to rover!');
      return;
    }
    cb();
  };

  return (
    <div className="bg-[#111827] rounded-lg p-3 flex flex-col gap-2 h-full">
      <h3 className="text-sm font-bold text-slate-300 mb-2 text-center">Sprayer Control</h3>
      <button
        className="w-full py-3 bg-indigo-600 text-white rounded hover:bg-indigo-700"
        onClick={() => requireConnection(handleLoadMission)}
      >
        {isLoadingMission ? '⏳ Loading Mission...' : `Load Mission (${waypoints.length} WP)`}
      </button>
      {/* Config UI removed - only load waypoints from frontend */}
      <div className="flex items-center justify-between py-2">
        <span className="text-xs text-slate-300 font-bold">Mode:</span>
        <label className="flex items-center gap-2 cursor-pointer">
          <span className={mode === 'auto' ? 'text-green-400 font-bold' : 'text-slate-400'}>Auto</span>
            <input
              type="checkbox"
              checked={mode === 'manual'}
              onChange={async () => {
                const prev = mode;
                const newMode = mode === 'auto' ? 'manual' : 'auto';
                // optimistic UI
                setMode(newMode);
                setIsTogglingMode(true);
                try {
                  const res = await fetch(`${BACKEND_URL.replace(/\/$/, '')}/api/mission/mode`, {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ mode: newMode }),
                  });

                  if (!res.ok) {
                    const txt = await res.text().catch(() => '');
                    throw new Error(`HTTP ${res.status}: ${txt}`);
                  }

                  // try parse response (may be { mode: 'auto' } or similar)
                  try {
                    const body = await res.json().catch(() => null);
                    // if backend echoes mode, use it
                    if (body && (body.mode === 'auto' || body.mode === 'manual')) {
                      setMode(body.mode);
                    }
                  } catch (e) {
                    // ignore parse errors
                  }

                  toast.success(`Mode set to ${newMode}`);
                } catch (err) {
                  console.error('Failed to set mission mode:', err);
                  toast.error('Failed to set mode; reverting');
                  setMode(prev);
                } finally {
                  setIsTogglingMode(false);
                }
              }}
              className="form-checkbox h-4 w-4 text-blue-600"
              disabled={isTogglingMode}
            />
          <span className={mode === 'manual' ? 'text-blue-400 font-bold' : 'text-slate-400'}>Manual</span>
        </label>
      </div>
      <button
        className={`w-full py-2 rounded text-white ${isRunning ? 'bg-red-600 hover:bg-red-700' : 'bg-green-600 hover:bg-green-700'}`}
        onClick={() => {
          if (!isConnected) {
            toast.error('Not connected to rover!');
            return;
          }
          if (isRunning) {
            setConfirmAction({ action: 'Stop Mission', onConfirm: async () => { await handleStopMission(); setConfirmAction(null); } });
          } else {
            setConfirmAction({ action: 'Start Mission', onConfirm: async () => { await handleStartMission(); setConfirmAction(null); } });
          }
        }}
      >
        {isStartingMission 
          ? '⏳ Starting Mission...' 
          : isStoppingMission
          ? '⏳ Stopping Mission...'
          : isRunning 
            ? 'Stop Mission' 
            : 'Start Mission'
        }
      </button>
      <button
        className={`w-full py-3 rounded text-white ${isPaused ? 'bg-blue-500 hover:bg-blue-600' : 'bg-yellow-500 hover:bg-yellow-600'}`}
        onClick={() => {
          if (!isConnected) {
            toast.error('Not connected to rover!');
            return;
          }
          if (!isRunning) {
            toast.error('Mission not running!');
            return;
          }
          if (isPaused) {
            setIsPaused(false);
            if (onResume) onResume();
          } else {
            setIsPaused(true);
            if (onPause) onPause();
          }
        }}
      >{isPaused ? 'Resume' : 'Pause'}</button>
      <button
        className="w-full py-3 bg-gray-700 text-white rounded hover:bg-gray-800"
        onClick={() => {
          if (!isConnected) {
            toast.error('Not connected to rover!');
            return;
          }
          if (onNext) onNext();
        }}
      >Next Waypoint</button>
      <button
        className="w-full py-3 bg-orange-500 text-white rounded hover:bg-orange-600"
        onClick={() => {
          if (!isConnected) {
            toast.error('Not connected to rover!');
            return;
          }
          toast.info('Skip Waypoint clicked');
        }}
      >Skip Waypoint</button>

      {/* Confirmation Dialog */}
      {confirmAction && (
        <div className="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-[5000]">
          <div className="bg-gray-800 rounded-lg p-6 max-w-sm w-full mx-4">
            <h3 className="text-lg font-semibold text-white text-center mb-4">
              Confirm {confirmAction.action}
            </h3>
            <div className="flex gap-3 mt-6">
              <button
                onClick={() => setConfirmAction(null)}
                className="flex-1 py-2 bg-gray-600 text-white rounded hover:bg-gray-700"
              >
                Cancel
              </button>
              <button
                onClick={confirmAction.onConfirm}
                className="flex-1 py-2 bg-blue-600 text-white rounded hover:bg-blue-700"
              >
                Confirm
              </button>
            </div>
          </div>
        </div>
      )}

      {/* Load Mission Progress Dialog */}
      {isLoadingMission && (
        <div className="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-50">
          <div className="bg-gray-800 rounded-lg p-6 max-w-sm w-full mx-4">
            <div className="flex items-center justify-center mb-4">
              <div className="animate-spin rounded-full h-8 w-8 border-b-2 border-indigo-500"></div>
            </div>
            <h3 className="text-lg font-semibold text-white text-center mb-2">
              Loading Mission
            </h3>
            <p className="text-gray-300 text-center text-sm">
              Sending waypoints to controller...
            </p>
          </div>
        </div>
      )}

      {/* Start Mission Progress Dialog */}
      {isStartingMission && (
        <div className="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-50">
          <div className="bg-gray-800 rounded-lg p-6 max-w-sm w-full mx-4">
            <div className="flex items-center justify-center mb-4">
              <div className="animate-spin rounded-full h-8 w-8 border-b-2 border-green-500"></div>
            </div>
            <h3 className="text-lg font-semibold text-white text-center mb-2">
              Starting Mission
            </h3>
            <p className="text-gray-300 text-center text-sm">
              Initializing mission controller...
            </p>
          </div>
        </div>
      )}

      {/* Stop Mission Progress Dialog */}
      {isStoppingMission && (
        <div className="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-50">
          <div className="bg-gray-800 rounded-lg p-6 max-w-sm w-full mx-4">
            <div className="flex items-center justify-center mb-4">
              <div className="animate-spin rounded-full h-8 w-8 border-b-2 border-red-500"></div>
            </div>
            <h3 className="text-lg font-semibold text-white text-center mb-2">
              Stopping Mission
            </h3>
            <p className="text-gray-300 text-center text-sm">
              Shutting down mission controller...
            </p>
          </div>
        </div>
      )}

      {/* Waypoint Preview Dialog */}
      {showWaypointDialog && (
        <div className="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-[9999]">
          <div className="bg-slate-800 rounded-lg p-6 max-w-md w-full mx-4 max-h-[80vh] overflow-y-auto">
            <h3 className="text-lg font-semibold text-white mb-4">Confirm Mission Upload</h3>

            <div className="mb-4">
              <p className="text-slate-300 text-sm mb-3">
                Ready to upload <span className="font-semibold text-blue-400">{waypoints.length}</span> waypoints to the mission controller?
              </p>

              <div className="bg-slate-700 rounded p-3 max-h-48 overflow-y-auto">
                <h4 className="text-sm font-medium text-slate-200 mb-2">Waypoint Summary:</h4>
                <div className="space-y-1">
                  {waypoints.slice(0, 10).map((wp, index) => (
                    <div key={index} className="text-xs text-slate-400 flex justify-between">
                      <span>WP {index + 1}:</span>
                      <span>{wp.lat.toFixed(6)}, {wp.lng.toFixed(6)}</span>
                    </div>
                  ))}
                  {waypoints.length > 10 && (
                    <div className="text-xs text-slate-500 text-center pt-1">
                      ... and {waypoints.length - 10} more waypoints
                    </div>
                  )}
                </div>
              </div>
            </div>

            <div className="flex gap-3">
              <button
                onClick={() => setShowWaypointDialog(false)}
                className="flex-1 py-2 bg-gray-600 text-white rounded hover:bg-gray-700"
              >
                Cancel
              </button>
              <button
                onClick={handleConfirmUpload}
                className="flex-1 py-2 bg-green-600 text-white rounded hover:bg-green-700 disabled:opacity-50"
                disabled={isLoadingMission}
              >
                {isLoadingMission ? 'Uploading...' : 'Upload Mission'}
              </button>
            </div>
          </div>
        </div>
      )}

      {/* Configuration UI removed */}
    </div>
  );
};

export default SprayerControls;
