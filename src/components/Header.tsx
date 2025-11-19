import React, { useMemo } from 'react';
import { WrenchIcon } from './icons/WrenchIcon';
import { FullScreenToggleIcon } from './icons/FullScreenToggleIcon';
import { useRover } from '../context/RoverContext';
import { ViewMode } from '../types';
import { ConnectionStatus } from './ConnectionStatus';

type HeaderProps = {
  viewMode: ViewMode;
  setViewMode: (mode: ViewMode) => void;
  isFullScreen: boolean;
  onToggleFullScreen: () => void;
};

const CONNECTION_BADGE: Record<string, { label: string; className: string }> = {
  connecting: { label: 'Connecting…', className: 'bg-yellow-500 animate-pulse' },
  connected: { label: 'Connected', className: 'bg-green-600' },
  error: { label: 'Error', className: 'bg-red-600' },
  disconnected: { label: 'Disconnected', className: 'bg-slate-600' },
};

const Header: React.FC<HeaderProps> = ({
  viewMode,
  setViewMode,
  isFullScreen,
  onToggleFullScreen,
}) => {
  const { connectionState, reconnect } = useRover();

  const connectionBadge = useMemo(() => {
    return CONNECTION_BADGE[connectionState] ?? CONNECTION_BADGE.disconnected;
  }, [connectionState]);

  return (
    <header className="bg-[#111827] flex items-center justify-between px-3 py-2 shadow-lg flex-shrink-0">
      <div className="flex items-center gap-3">
        <div className="flex items-center gap-2">
          <div className="bg-orange-500 p-1.5 rounded-md">
            <WrenchIcon className="w-5 h-5 text-gray-900" />
          </div>
          <h1 className="text-lg font-bold text-orange-400">LAND ROVER</h1>
        </div>
  <nav className="flex items-center bg-[#1F2937] rounded-lg">
          <button
            onClick={() => setViewMode('dashboard')}
            className={`px-3 py-1.5 text-xs font-semibold rounded-lg ${
              viewMode === 'dashboard' ? 'bg-green-500 text-white' : 'text-gray-300 hover:bg-gray-700'
            }`}
          >
            Dashboard
          </button>
          <button
            onClick={() => setViewMode('planning')}
            className={`px-3 py-1.5 text-xs font-semibold rounded-lg ${
              viewMode === 'planning' ? 'bg-green-500 text-white' : 'text-gray-300 hover:bg-gray-700'
            }`}
          >
            Edit Plan
          </button>
          <button
            onClick={() => setViewMode('live')}
            className={`px-3 py-1.5 text-xs font-semibold rounded-lg ${
              viewMode === 'live' ? 'bg-green-500 text-white' : 'text-gray-300 hover:bg-gray-700'
            }`}
          >
            Live Report
          </button>
          <button
            onClick={() => setViewMode('sprayer')}
            className={`px-3 py-1.5 text-xs font-semibold rounded-lg ${
              viewMode === 'sprayer' ? 'bg-green-500 text-white' : 'text-gray-300 hover:bg-gray-700'
            }`}
          >
            Sprayer
          </button>
          <button
            onClick={() => setViewMode('setup')}
            className={`px-3 py-1.5 text-xs font-semibold rounded-lg ${
              viewMode === 'setup' ? 'bg-green-500 text-white' : 'text-gray-300 hover:bg-gray-700'
            }`}
          >
            Setup
          </button>
        </nav>
      </div>

      <div className="flex items-center gap-3">
        {/* HTTP Backend Connection Status */}
        <div className="flex items-center gap-2 px-2.5 py-1 rounded-full bg-gray-800 border border-gray-700">
          <ConnectionStatus showLabel={true} />
        </div>
        
        {/* ROS WebSocket Connection Status */}
        <span
          className={`flex items-center gap-2 px-2.5 py-1 rounded-full text-xs font-semibold uppercase tracking-wide text-white ${connectionBadge.className}`}
        >
          <span className="w-2 h-2 rounded-full bg-white" />
          {connectionBadge.label}
        </span>
        <button
          onClick={reconnect}
          className="font-bold px-3 py-1.5 rounded-lg bg-indigo-600 hover:bg-indigo-500 transition-colors text-white text-xs"
        >
          Reconnect
        </button>
        <button
          onClick={onToggleFullScreen}
          className="p-1.5 rounded-lg text-gray-300 hover:bg-gray-700 transition-colors"
          aria-label={isFullScreen ? 'Exit full screen' : 'Enter full screen'}
          title={isFullScreen ? 'Exit full screen' : 'Enter full screen'}
        >
          <FullScreenToggleIcon isFullScreen={isFullScreen} className="w-4 h-4" />
        </button>
      </div>
    </header>
  );
};

export default Header;
