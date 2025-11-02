import React from 'react';

// MissionLog and missionLogs props removed (no longer used)

/**
 * LogsPanel serves as a dedicated container in the sidebar for displaying
 * mission logs via the LogManager component. It is designed to fill
 * the remaining vertical space.
 */
const LogsPanel: React.FC = () => {
  return (
    <div className="bg-[#111827] rounded-lg flex flex-col flex-1 min-h-0 overflow-hidden">
      {/* Telemetry and mode status removed - already available in TelemetryPanel */}
    </div>
  );
};

export default LogsPanel;
