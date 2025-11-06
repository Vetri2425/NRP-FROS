import React, { useCallback, useEffect, useMemo, useState } from 'react';
import { useRover } from '../context/RoverContext';
import { startWPMarkMissionFromSavedConfig } from '../services/wpMarkService';
import WPMarkDialog from './WPMarkDialog';

const AVAILABLE_MODES = ['AUTO', 'GUIDED', 'HOLD', 'MANUAL', 'WP_MARK'];

const ModeSelector: React.FC = () => {
  const {
    telemetry: { state },
    services,
  } = useRover();
  const [isLoading, setIsLoading] = useState(false);
  const [message, setMessage] = useState<{ type: 'success' | 'error'; text: string } | null>(null);
  const [isWPMarkDialogOpen, setIsWPMarkDialogOpen] = useState(false);

  const normalizedMode = useMemo(
    () => state.mode?.toUpperCase?.() ?? 'UNKNOWN',
    [state.mode],
  );

  const options = useMemo(() => {
    if (normalizedMode !== 'UNKNOWN' && !AVAILABLE_MODES.includes(normalizedMode)) {
      return [...AVAILABLE_MODES, normalizedMode];
    }
    return AVAILABLE_MODES;
  }, [normalizedMode]);

  const [selectedMode, setSelectedMode] = useState(normalizedMode);

  useEffect(() => {
    setSelectedMode(normalizedMode);
  }, [normalizedMode]);

  const runAction = useCallback(
    async (
      action: () => Promise<{ success: boolean; message?: string }>,
      buildDefaultMessage: (success: boolean) => string,
    ) => {
      setMessage(null);
      setIsLoading(true);
      try {
        const resp = await action();
        if (resp.success) {
          setMessage({
            type: 'success',
            text: resp.message ?? buildDefaultMessage(true),
          });
        } else {
          setMessage({
            type: 'error',
            text: resp.message ?? buildDefaultMessage(false),
          });
        }
      } catch (err) {
        setMessage({
          type: 'error',
          text: err instanceof Error ? err.message : 'Command failed',
        });
      } finally {
        setIsLoading(false);
      }
    },
    [],
  );

  const requestModeChange = useCallback(async () => {
    if (!selectedMode) {
      return;
    }
    
    // If WP_MARK mode is selected, open the dialog instead of changing mode
    if (selectedMode === 'WP_MARK') {
      setIsWPMarkDialogOpen(true);
      return;
    }
    
    await runAction(
      () => services.setMode(selectedMode),
      (success) => (success ? `Mode change requested: ${selectedMode}` : 'Failed to set mode'),
    );
  }, [runAction, selectedMode, services]);

  const toggleArmState = useCallback(async () => {
    const currentlyArmed = Boolean(state.armed);
    await runAction(
      () => (currentlyArmed ? services.disarmVehicle() : services.armVehicle()),
      (success) => {
        if (!success) {
          return currentlyArmed ? 'Failed to disarm vehicle' : 'Failed to arm vehicle';
        }
        return currentlyArmed ? 'Disarm request sent' : 'Arm request sent';
      },
    );
  }, [runAction, services, state.armed]);

  const startMission = useCallback(async () => {
    await runAction(
      async () => {
        // Only set mode to AUTO if not already in AUTO mode to avoid Context.init() error
        if (state.mode !== 'AUTO') {
          await services.setMode('AUTO');
          // Add small delay to allow mode change to complete
          await new Promise(resolve => setTimeout(resolve, 500));
        }

        const response = await startWPMarkMissionFromSavedConfig();
        if (!response.success) {
          throw new Error(response.error || 'Failed to start mission');
        }
        return response;
      },
      (success) => {
        if (!success) {
          return 'Failed to start mission';
        }
        const missionInfo = (success as any).mission_info;
        return `Mission started! ${missionInfo?.total_waypoints || 'N/A'} waypoints, ~${missionInfo?.estimated_duration_minutes?.toFixed(1) || 'N/A'} min`;
      },
    );
  }, [runAction, services, state.mode]);

  return (
    <div className="bg-[#111827] rounded-lg flex flex-col overflow-hidden">
      {/* Header with Indigo Background */}
      <div className="bg-indigo-700 text-white text-xs font-bold px-3 py-2 flex items-center justify-between flex-shrink-0">
        <span className="font-semibold text-white tracking-wide text-sm">Mode Control</span>
        <span className="text-xs text-indigo-100 uppercase font-mono">{normalizedMode}</span>
      </div>

      {/* Panel Content */}
      <div className="p-3.5 flex flex-col gap-2">

      <div className="flex items-center gap-2">
        <select
          value={selectedMode}
          onChange={(event) => {
            setMessage(null);
            setSelectedMode(event.target.value);
          }}
          disabled={isLoading}
          className="bg-[#1F2937] border border-slate-600 rounded-md px-2 py-1.5 text-xs focus:outline-none focus:ring-2 focus:ring-indigo-500 flex-1"
        >
          {options.map((mode) => (
            <option key={mode} value={mode}>
              {mode}
            </option>
          ))}
        </select>
        <button
          onClick={requestModeChange}
          disabled={isLoading || !selectedMode || selectedMode === normalizedMode}
          className="px-2.5 py-1.5 rounded-md text-xs font-semibold bg-blue-600 hover:bg-blue-500 text-white disabled:opacity-50 disabled:cursor-not-allowed transition-colors"
        >
          Set
        </button>
      </div>

      <div className="flex items-center gap-2">
        <span
          className={`inline-flex items-center gap-1 text-xs px-2 py-1 rounded ${
            state.armed ? 'bg-green-900 text-green-200' : 'bg-slate-700 text-slate-300'
          }`}
        >
          {state.armed ? 'Armed' : 'Disarmed'}
        </span>
        <button
          onClick={toggleArmState}
          disabled={isLoading}
          className={`px-2.5 py-1.5 rounded-md text-xs font-semibold transition-colors flex-1 ${
            state.armed
              ? 'bg-red-600 hover:bg-red-500 text-white'
              : 'bg-green-600 hover:bg-green-500 text-white'
          } disabled:opacity-50 disabled:cursor-not-allowed`}
        >
          {state.armed ? 'Disarm' : 'Arm'}
        </button>
      </div>

      <div className="flex items-center gap-2">
        <button
          onClick={startMission}
          disabled={isLoading || !state.armed}
          className="px-2.5 py-1.5 rounded-md text-xs font-semibold bg-orange-600 hover:bg-orange-500 text-white disabled:opacity-50 disabled:cursor-not-allowed transition-colors flex-1"
        >
          🚀 Start Mission
        </button>
      </div>

      {message && (
        <p
          className={`text-xs ${
            message.type === 'success' ? 'text-green-300' : 'text-red-300'
          }`}
        >
          {message.text}
        </p>
      )}
      </div>

      {/* WP_MARK Mission Dialog */}
      {isWPMarkDialogOpen && (
        <WPMarkDialog
          isOpen={isWPMarkDialogOpen}
          onClose={() => {
            setIsWPMarkDialogOpen(false);
            setSelectedMode(normalizedMode); // Reset to current mode
          }}
        />
      )}
    </div>
  );
};

export default ModeSelector;
