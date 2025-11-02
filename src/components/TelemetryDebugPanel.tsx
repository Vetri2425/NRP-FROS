import React, { useState } from 'react';
import { useRover } from '../context/RoverContext';

/**
 * TelemetryDebugPanel - A simple debug panel to inspect raw telemetry data
 * 
 * To use: Add this component anywhere in your app to see live telemetry
 * Example: <TelemetryDebugPanel />
 */
const TelemetryDebugPanel: React.FC = () => {
  const { telemetry, connectionState } = useRover();
  const [isExpanded, setIsExpanded] = useState(false);

  return (
    <div className="fixed bottom-4 right-4 z-50">
      <button
        onClick={() => setIsExpanded(!isExpanded)}
        className="bg-purple-600 hover:bg-purple-700 text-white px-4 py-2 rounded-lg shadow-lg font-mono text-sm"
      >
        {isExpanded ? '📊 Hide Debug' : '🔍 Show Telemetry Debug'}
      </button>

      {isExpanded && (
        <div className="mt-2 bg-gray-900 border border-purple-500 rounded-lg shadow-2xl p-4 w-96 max-h-96 overflow-y-auto">
          <h3 className="text-purple-400 font-bold mb-2 text-sm">Live Telemetry Data</h3>
          
          {/* Connection State */}
          <div className="mb-3 pb-2 border-b border-gray-700">
            <p className="text-xs text-gray-400">Connection:</p>
            <p className={`font-mono text-sm ${connectionState === 'connected' ? 'text-green-400' : 'text-red-400'}`}>
              {connectionState.toUpperCase()}
            </p>
          </div>

          {/* RTK Data - HIGHLIGHTED */}
          <div className="mb-3 pb-2 border-b border-yellow-600 bg-yellow-900 bg-opacity-20 p-2 rounded">
            <p className="text-xs text-yellow-400 font-bold">RTK Status:</p>
            <div className="font-mono text-xs text-white space-y-1">
              <div className="flex justify-between">
                <span className="text-gray-400">fix_type:</span>
                <span className={`font-bold ${telemetry.rtk.fix_type > 0 ? 'text-green-400' : 'text-red-400'}`}>
                  {telemetry.rtk.fix_type} {telemetry.rtk.fix_type === 4 ? '(RTK Fixed)' : 
                   telemetry.rtk.fix_type === 3 ? '(RTK Float)' : 
                   telemetry.rtk.fix_type === 2 ? '(DGPS)' : 
                   telemetry.rtk.fix_type === 1 ? '(GPS Fix)' : '(No Fix)'}
                </span>
              </div>
              <div className="flex justify-between">
                <span className="text-gray-400">baseline_age:</span>
                <span>{telemetry.rtk.baseline_age.toFixed(2)}s</span>
              </div>
              <div className="flex justify-between">
                <span className="text-gray-400">base_linked:</span>
                <span className={telemetry.rtk.base_linked ? 'text-green-400' : 'text-red-400'}>
                  {telemetry.rtk.base_linked ? 'YES' : 'NO'}
                </span>
              </div>
            </div>
          </div>

          {/* GPS/Global Data */}
          <div className="mb-3 pb-2 border-b border-gray-700">
            <p className="text-xs text-blue-400 font-bold">GPS Position:</p>
            <div className="font-mono text-xs text-white space-y-1">
              <div className="flex justify-between">
                <span className="text-gray-400">Latitude:</span>
                <span>{telemetry.global.lat.toFixed(7)}</span>
              </div>
              <div className="flex justify-between">
                <span className="text-gray-400">Longitude:</span>
                <span>{telemetry.global.lon.toFixed(7)}</span>
              </div>
              <div className="flex justify-between">
                <span className="text-gray-400">Satellites:</span>
                <span className={telemetry.global.satellites_visible > 0 ? 'text-green-400' : 'text-red-400'}>
                  {telemetry.global.satellites_visible}
                </span>
              </div>
              <div className="flex justify-between">
                <span className="text-gray-400">Altitude (rel):</span>
                <span>{telemetry.global.alt_rel.toFixed(1)} m</span>
              </div>
              <div className="flex justify-between">
                <span className="text-gray-400">Velocity:</span>
                <span>{telemetry.global.vel.toFixed(2)} m/s</span>
              </div>
            </div>
          </div>

          {/* State */}
          <div className="mb-3 pb-2 border-b border-gray-700">
            <p className="text-xs text-green-400 font-bold">State:</p>
            <div className="font-mono text-xs text-white space-y-1">
              <div className="flex justify-between">
                <span className="text-gray-400">Mode:</span>
                <span>{telemetry.state.mode}</span>
              </div>
              <div className="flex justify-between">
                <span className="text-gray-400">Armed:</span>
                <span className={telemetry.state.armed ? 'text-red-400' : 'text-green-400'}>
                  {telemetry.state.armed ? 'ARMED' : 'DISARMED'}
                </span>
              </div>
              <div className="flex justify-between">
                <span className="text-gray-400">System:</span>
                <span>{telemetry.state.system_status}</span>
              </div>
            </div>
          </div>

          {/* Battery */}
          <div className="mb-3 pb-2 border-b border-gray-700">
            <p className="text-xs text-yellow-400 font-bold">Battery:</p>
            <div className="font-mono text-xs text-white space-y-1">
              <div className="flex justify-between">
                <span className="text-gray-400">Voltage:</span>
                <span>{telemetry.battery.voltage.toFixed(2)} V</span>
              </div>
              <div className="flex justify-between">
                <span className="text-gray-400">Current:</span>
                <span>{telemetry.battery.current.toFixed(2)} A</span>
              </div>
              <div className="flex justify-between">
                <span className="text-gray-400">Percentage:</span>
                <span>{telemetry.battery.percentage.toFixed(1)}%</span>
              </div>
            </div>
          </div>

          {/* Mission */}
          <div className="mb-3 pb-2 border-b border-gray-700">
            <p className="text-xs text-purple-400 font-bold">Mission:</p>
            <div className="font-mono text-xs text-white space-y-1">
              <div className="flex justify-between">
                <span className="text-gray-400">Current WP:</span>
                <span>{telemetry.mission.current_wp} / {telemetry.mission.total_wp}</span>
              </div>
              <div className="flex justify-between">
                <span className="text-gray-400">Status:</span>
                <span>{telemetry.mission.status}</span>
              </div>
              <div className="flex justify-between">
                <span className="text-gray-400">Progress:</span>
                <span>{telemetry.mission.progress_pct.toFixed(1)}%</span>
              </div>
            </div>
          </div>

          {/* Last Update */}
          <div className="text-xs text-gray-500 text-center">
            Last update: {telemetry.lastMessageTs 
              ? new Date(telemetry.lastMessageTs).toLocaleTimeString() 
              : 'Never'}
          </div>
        </div>
      )}
    </div>
  );
};

export default TelemetryDebugPanel;
