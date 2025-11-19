import React from 'react';

export interface SprayStatus {
  waypointId: number;
  status: 'pending' | 'navigating' | 'spraying' | 'completed' | 'failed';
  sprayStartTime?: string;
  sprayEndTime?: string;
  duration?: number;
}

type SprayStatusIndicatorProps = {
  sprayStatuses: SprayStatus[];
  currentWaypoint?: number;
};

const SprayStatusIndicator: React.FC<SprayStatusIndicatorProps> = ({
  sprayStatuses,
  currentWaypoint
}) => {
  const getStatusIcon = (status: string) => {
    switch (status) {
      case 'pending': return '⏳';
      case 'navigating': return '🚶';
      case 'spraying': return '💧';
      case 'completed': return '✅';
      case 'failed': return '❌';
      default: return '❓';
    }
  };

  const getStatusColor = (status: string) => {
    switch (status) {
      case 'pending': return 'text-gray-400';
      case 'navigating': return 'text-blue-400';
      case 'spraying': return 'text-cyan-400';
      case 'completed': return 'text-green-400';
      case 'failed': return 'text-red-400';
      default: return 'text-gray-400';
    }
  };

  if (sprayStatuses.length === 0) {
    return (
      <div className="text-xs text-gray-500 text-center py-2">
        No spray data available
      </div>
    );
  }

  return (
    <div className="space-y-1 max-h-32 overflow-y-auto">
      {sprayStatuses.map((sprayStatus) => (
        <div
          key={sprayStatus.waypointId}
          className={`flex items-center justify-between text-xs p-1 rounded ${
            sprayStatus.waypointId === currentWaypoint ? 'bg-blue-900/30' : 'bg-slate-800/30'
          }`}
        >
          <div className="flex items-center gap-2">
            <span className={getStatusColor(sprayStatus.status)}>
              {getStatusIcon(sprayStatus.status)}
            </span>
            <span className="text-slate-300">WP {sprayStatus.waypointId}</span>
          </div>
          <div className="text-right">
            {sprayStatus.status === 'spraying' && sprayStatus.duration && (
              <span className="text-cyan-400">{sprayStatus.duration.toFixed(1)}s</span>
            )}
            {sprayStatus.status === 'completed' && sprayStatus.duration && (
              <span className="text-green-400">{sprayStatus.duration.toFixed(1)}s</span>
            )}
          </div>
        </div>
      ))}
    </div>
  );
};

export default SprayStatusIndicator;