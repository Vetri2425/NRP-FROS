import React from 'react';
import TelemetryPanel from './TelemetryPanel';
import ModeSelector from './ModeSelector';
import RTKPanel from './RTKPanel';
import LogsPanel from './LogsPanel';
import StatusBar from './StatusBar';

const LeftSidebar: React.FC = () => {
  return (
    <aside className="w-72 flex flex-col gap-3 min-h-0 overflow-hidden">
      <div className="flex-shrink-0">
        <TelemetryPanel />
      </div>
      <div className="flex-shrink-0">
        <ModeSelector />
      </div>
      <div className="flex-shrink-0">
        <RTKPanel />
      </div>
      <div className="flex-1 min-h-0 overflow-hidden">
        <LogsPanel />
      </div>
      <div className="flex-shrink-0">
        <StatusBar />
      </div>
    </aside>
  );
};

export default LeftSidebar;
