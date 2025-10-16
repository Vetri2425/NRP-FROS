import React, { useMemo } from 'react';
import { BatteryIcon } from './icons/BatteryIcon';
import { RtkIcon } from './icons/RtkIcon';
import { SignalIcon } from './icons/SignalIcon';
import { ImuIcon } from './icons/ImuIcon';
import { RoverData } from '../hooks/useRoverConnection';
import { useFrameTicker } from '../hooks/useFrameTicker';

type StatusPanelProps = {
  roverData: RoverData;
  isConnected: boolean;
}

const StatusPanel: React.FC<StatusPanelProps> = ({ roverData, isConnected }) => {
  // Generate a unique key from critical data to force re-renders
  const dataKey = `${isConnected}-${roverData.mode}-${roverData.status}-${roverData.battery}`;
  
  // High-frequency UI ticker (~60 FPS) using a shared hook
  const frameNow = useFrameTicker();

  // Derive telemetry age in milliseconds (updates every animation frame)
  const ageMs = useMemo(() => {
    const last = (roverData as any).lastUpdate
      ? new Date((roverData as any).lastUpdate as any).getTime()
      : (typeof roverData.telemetryAgeMs === 'number'
          ? Date.now() - roverData.telemetryAgeMs
          : undefined);
    if (typeof last !== 'number' || Number.isNaN(last)) return null;
    return Math.max(0, Date.now() - last);
  }, [frameNow, (roverData as any).lastUpdate, roverData.telemetryAgeMs]);

  const modeLabel = !isConnected
    ? 'Offline'
    : `${roverData.mode ?? 'UNKNOWN'} • ${roverData.status === 'armed' ? 'Armed' : 'Disarmed'}`;

  const batteryLabel = !isConnected
    ? 'N/A'
    : roverData.battery >= 0
      ? `${roverData.battery}%`
      : 'Waiting';

  const gpsLabel = !isConnected
    ? 'N/A'
    : [roverData.rtk_status, roverData.hrms ? `${roverData.hrms}m` : undefined]
        .filter(Boolean)
        .join(' • ');

  const linkLabel = !isConnected
    ? 'No Link'
    : `${roverData.signal_strength || 'Weak'}${roverData.rc_connected ? ' • RC OK' : ' • RC Lost'}`;

  const isTelemetryStale = ageMs !== null && ageMs > 5000;
  const headerStatus = !isConnected
    ? 'Disconnected'
    : ageMs == null
      ? 'Waiting'
      : (ageMs < 250
          ? 'Telemetry live'
          : (ageMs < 1000
              ? `Telemetry ${Math.round(ageMs)} ms old`
              : `Telemetry ${(ageMs / 1000).toFixed(1)}s old`));

  const statusItems = [
    { 
      name: 'Mode', 
      value: modeLabel, 
      icon: <ImuIcon className="w-5 h-5 text-yellow-400" /> 
    },
    { 
      name: 'Battery', 
      value: batteryLabel, 
      icon: <BatteryIcon level={isConnected ? (roverData.battery ?? -1) : -1} className="w-5 h-5" /> 
    },
    { 
      name: 'GPS', 
      value: gpsLabel || 'Waiting', 
      icon: <RtkIcon className="w-5 h-5 text-blue-400" /> 
    },
    { 
      name: 'Link', 
      value: linkLabel, 
      icon: <SignalIcon className="w-5 h-5 text-sky-400" /> 
    },
  ];

  return (
    <div className="bg-[#111827] rounded-lg overflow-hidden" key={dataKey}>
      <div className="bg-purple-700 text-white text-md font-bold p-3 flex items-center justify-between">
        <span>STATUS</span>
        <span className={`text-xs font-semibold ${isTelemetryStale ? 'text-yellow-200' : 'text-purple-100'}`}>
          {headerStatus}
        </span>
      </div>
      <ul className="p-4 space-y-4">
        {statusItems.map((item) => (
          <li key={item.name} className="flex items-center justify-between">
            <div className="flex items-center gap-3">
              {item.icon}
              <span className="font-medium text-gray-300">{item.name}</span>
            </div>
            <span className="font-mono text-sm text-gray-200 uppercase text-right max-w-[160px] truncate" title={item.value}>
              {item.value}
            </span>
          </li>
        ))}
      </ul>
    </div>
  );
};

export default StatusPanel;
