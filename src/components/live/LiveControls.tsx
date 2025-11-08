
import React, { useState } from 'react';
import { useRover } from '../../context/RoverContext';
import { PlayIcon } from '../icons/PlayIcon';
import { PauseIcon } from '../icons/PauseIcon';
import { stopWPMarkMission } from '../../services/wpMarkService';
import { WPMarkStatus } from '../../services/wpMarkService';
import { TrashIcon } from '../icons/TrashIcon';
import { useDialog } from '../../hooks/useDialog';
import GenericDialog from '../GenericDialog';

type LiveControlsProps = {
    isConnected: boolean;
    currentWaypoint?: number;
    wpMarkStatus?: WPMarkStatus | null;
    onClearLogs?: () => void;
};

const LiveControls: React.FC<LiveControlsProps> = ({ isConnected, currentWaypoint = 0, wpMarkStatus, onClearLogs }) => {
    const { services, telemetry } = useRover();
    const [isPaused, setIsPaused] = useState(false);
    const [isLoading, setIsLoading] = useState(false);

    const { dialogState, showConfirm, showAlert } = useDialog();

    // Use telemetry data as source of truth
    const totalWaypoints = telemetry.mission.total_wp || 0;
    const currentWp = telemetry.mission.current_wp || currentWaypoint || 0;

    const handleSkip = async () => {
        if (!isConnected || isLoading) return;
        
        const nextWp = currentWp + 1;
        if (nextWp >= totalWaypoints) {
            await showAlert('Navigation Error', 'Already at last waypoint');
            return;
        }

        setIsLoading(true);
        try {
            const response = await services.setCurrentWaypoint(nextWp);
            if (response.success) {
                console.log(`Skipped to waypoint ${nextWp}`);
            } else {
                await showAlert('Skip Failed', response.message || 'Failed to skip waypoint');
            }
        } catch (error) {
            await showAlert('Skip Error', error instanceof Error ? error.message : 'Failed to skip waypoint');
        } finally {
            setIsLoading(false);
        }
    };

    const handleGoBack = async () => {
        if (!isConnected || isLoading) return;
        
        const prevWp = currentWp - 1;
        if (prevWp < 0) {
            await showAlert('Navigation Error', 'Already at first waypoint');
            return;
        }

        setIsLoading(true);
        try {
            const response = await services.setCurrentWaypoint(prevWp);
            if (response.success) {
                console.log(`Went back to waypoint ${prevWp}`);
            } else {
                await showAlert('Go Back Failed', response.message || 'Failed to go back');
            }
        } catch (error) {
            await showAlert('Go Back Error', error instanceof Error ? error.message : 'Failed to go back');
        } finally {
            setIsLoading(false);
        }
    };

    const handlePauseResume = async () => {
        if (!isConnected || isLoading) return;

        setIsLoading(true);
        try {
            if (isPaused) {
                const response = await services.resumeMission();
                if (response.success) {
                    setIsPaused(false);
                    console.log('Mission resumed');
                } else {
                    await showAlert('Resume Failed', response.message || 'Failed to resume mission');
                }
            } else {
                const response = await services.pauseMission();
                if (response.success) {
                    setIsPaused(true);
                    console.log('Mission paused');
                } else {
                    await showAlert('Pause Failed', response.message || 'Failed to pause mission');
                }
            }
        } catch (error) {
            await showAlert('Mission Control Error', error instanceof Error ? error.message : 'Failed to pause/resume mission');
        } finally {
            setIsLoading(false);
        }
    };

    const handleStop = async () => {
        if (!isConnected || isLoading) return;

        const confirmed = await showConfirm('Stop Mission', 'Are you sure you want to stop the mission? The rover will switch to HOLD mode.');
        if (!confirmed) return;

        setIsLoading(true);
        try {
            const response = await services.pauseMission();
            if (response.success) {
                setIsPaused(true);
                console.log('Mission stopped (HOLD mode)');
            } else {
                await showAlert('Stop Failed', response.message || 'Failed to stop mission');
            }
        } catch (error) {
            await showAlert('Stop Error', error instanceof Error ? error.message : 'Failed to stop mission');
        } finally {
            setIsLoading(false);
        }
    };

    const handleStopWpMark = async () => {
        if (!isConnected || isLoading) return;

        const confirmed = await showConfirm('Stop WP_MARK Mission', 'Are you sure you want to disarm and stop the WP_MARK mission? The rover will switch to HOLD mode.');
        if (!confirmed) return;

        setIsLoading(true);
        try {
            // First stop the WP_MARK mission
            const stopResponse = await stopWPMarkMission();
            if (!stopResponse.success) {
                throw new Error(stopResponse.error || 'Failed to stop WP_MARK mission');
            }

            // Then put rover in HOLD mode (pause mission)
            const holdResponse = await services.pauseMission();
            if (holdResponse.success) {
                setIsPaused(true);
                await showAlert('WP_MARK Stopped', 'WP_MARK mission disarmed. Rover is now in HOLD mode.');
            } else {
                await showAlert('Partial Success', 'WP_MARK mission stopped but failed to enter HOLD mode.');
            }
        } catch (error) {
            await showAlert('Disarm Failed', error instanceof Error ? error.message : 'Failed to disarm WP_MARK mission');
        } finally {
            setIsLoading(false);
        }
    };

    const commonClass =
        'w-full flex-1 text-white font-bold text-xl rounded-lg flex items-center justify-center gap-3 transition-colors disabled:opacity-50 disabled:cursor-not-allowed';

    return (
        <div className="h-full flex flex-col gap-4 relative">
            {/* WP_MARK Mission Status */}
            {wpMarkStatus && wpMarkStatus.running && (
                <div className="bg-orange-900/20 border border-orange-700/50 rounded-lg p-3 mb-2">
                    <div className="text-center">
                        <h3 className="text-sm font-bold text-orange-400">🎯 WP_MARK Active</h3>
                        <p className="text-xs text-orange-300 mt-1">
                            Phase: {wpMarkStatus.current_phase} | 
                            Progress: {wpMarkStatus.current_waypoint}/{wpMarkStatus.total_waypoints}
                        </p>
                        <p className="text-xs text-gray-400 mt-1">
                            Uptime: {wpMarkStatus.uptime_seconds.toFixed(1)}s
                        </p>
                    </div>
                </div>
            )}

            {/* Standard Mission Controls */}
            {!wpMarkStatus?.running && (
                <>
                    <button
                        onClick={handleSkip}
                        disabled={!isConnected || isLoading || currentWp >= totalWaypoints - 1}
                        className={`${commonClass} bg-orange-500 hover:bg-orange-600`}
                        title="Skip to next waypoint"
                    >
                        <PlayIcon className="w-6 h-6 rotate-90" /> SKIP
                    </button>
                    <button
                        onClick={handleGoBack}
                        disabled={!isConnected || isLoading || currentWp <= 0}
                        className={`${commonClass} bg-blue-600 hover:bg-blue-700`}
                        title="Go back to previous waypoint"
                    >
                        <span className="transform -scale-x-100 inline-block">
                            <PlayIcon className="w-6 h-6 -rotate-90" />
                        </span>
                         GO BACK
                    </button>
                    <button
                        onClick={handlePauseResume}
                        disabled={!isConnected || isLoading}
                        className={`${commonClass} ${isPaused ? 'bg-green-600 hover:bg-green-700' : 'bg-blue-600 hover:bg-blue-700'}`}
                        title={isPaused ? 'Resume mission' : 'Pause mission'}
                    >
                        {isPaused ? (
                            <><PlayIcon className="w-6 h-6" /> RESUME</>
                        ) : (
                            <><PauseIcon className="w-6 h-6" /> PAUSE</>
                        )}
                    </button>
                    <button
                        onClick={handleStop}
                        disabled={!isConnected || isLoading}
                        className={`${commonClass} bg-red-600 hover:bg-red-700`}
                        title="Stop mission"
                    >
                        <div className="w-5 h-5 bg-white" /> STOP
                    </button>
                    {onClearLogs && (
                        <button
                            onClick={onClearLogs}
                            className={`${commonClass} bg-gray-600 hover:bg-gray-700`}
                            title="Clear mission logs and data"
                        >
                            <TrashIcon className="w-6 h-6" /> CLEAR LOGS
                        </button>
                    )}
                </>
            )}

            {/* WP_MARK Stop Control */}
            {wpMarkStatus && wpMarkStatus.running && (
                <button
                    onClick={handleStopWpMark}
                    disabled={!isConnected || isLoading}
                    className={`${commonClass} bg-red-600 hover:bg-red-700`}
                    title="Disarm and stop WP_MARK mission (rover will enter HOLD mode)"
                >
                    <div className="w-5 h-5 bg-white" /> DISARM & STOP
                </button>
            )}

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

export default LiveControls;
