import React from 'react';
import { WrenchIcon } from './icons/WrenchIcon';
import { FullScreenToggleIcon } from './icons/FullScreenToggleIcon';
import { ConnectionStatus, BackendLinkStatus } from '../hooks/useRoverConnection';
import { ViewMode } from '../types';

type HeaderProps = {
  viewMode: ViewMode;
  setViewMode: (mode: ViewMode) => void;
  isFullScreen: boolean;
  onToggleFullScreen: () => void;
  connectionStatus: ConnectionStatus;
  backendStatus: BackendLinkStatus;
  onToggleConnection: () => void;
};

// --- Connection Indicators (unchanged) ---
const ConnectionIndicator: React.FC<{ status: ConnectionStatus }> = React.memo(({ status }) => {
  const styles = {
    DISCONNECTED: { text: 'Disconnected', color: 'bg-gray-500' },
    ERROR: { text: 'Error', color: 'bg-red-500' },
    CONNECTING: { text: 'Connecting...', color: 'bg-yellow-500 animate-pulse' },
    WAITING_FOR_ROVER: { text: 'Waiting for Rover...', color: 'bg-yellow-500 animate-pulse' },
    CONNECTED_TO_ROVER: { text: 'Connected', color: 'bg-green-500' },
  };
  const currentStyle = styles[status] || styles['DISCONNECTED'];

  return (
    <div className={`flex items-center space-x-2 px-3 py-1 rounded-full text-xs font-medium text-white ${currentStyle.color}`}>
      <div className={`w-2 h-2 rounded-full ${status === 'CONNECTING' || status === 'WAITING_FOR_ROVER' ? 'animate-pulse' : ''} bg-white`}></div>
      <span>{currentStyle.text}</span>
    </div>
  );
});
ConnectionIndicator.displayName = 'ConnectionIndicator';

const BackendIndicator: React.FC<{ status: BackendLinkStatus }> = React.memo(({ status }) => {
  const styles: Record<BackendLinkStatus, { text: string; color: string }> = {
    OFFLINE: { text: 'Backend Offline', color: 'bg-gray-600' },
    CONNECTING: { text: 'Backend Connecting...', color: 'bg-blue-500 animate-pulse' },
    ONLINE: { text: 'Backend Online', color: 'bg-blue-600' },
    ERROR: { text: 'Backend Error', color: 'bg-red-600' },
  };
  const currentStyle = styles[status];
  return (
    <div className={`flex items-center space-x-2 px-3 py-1 rounded-full text-xs font-medium text-white ${currentStyle.color}`}>
      <div className={`w-2 h-2 rounded-full ${status === 'CONNECTING' ? 'animate-pulse' : ''} bg-white`}></div>
      <span>{currentStyle.text}</span>
    </div>
  );
});
BackendIndicator.displayName = 'BackendIndicator';

// --- Main Header Component ---
const Header: React.FC<HeaderProps> = ({
  viewMode,
  setViewMode,
  isFullScreen,
  onToggleFullScreen,
  connectionStatus,
  backendStatus,
  onToggleConnection,
}) => {

  const getConnectionButtonState = () => {
    if (backendStatus === 'CONNECTING') {
      return { text: 'Backend Connecting...', className: 'bg-blue-600 cursor-not-allowed', disabled: true };
    }
    if (backendStatus === 'ERROR') {
      return { text: 'Retry Backend', className: 'bg-red-600 hover:bg-red-700', disabled: false };
    }
    if (backendStatus === 'OFFLINE') {
      return { text: 'Connect Backend', className: 'bg-blue-600 hover:bg-blue-700', disabled: false };
    }
    if (connectionStatus === 'CONNECTED_TO_ROVER') {
      return { text: 'Reconnect Rover', className: 'bg-green-600 hover:bg-green-700', disabled: false };
    }
    if (connectionStatus === 'WAITING_FOR_ROVER') {
      return { text: 'Retry Rover', className: 'bg-orange-500 hover:bg-orange-600', disabled: false };
    }
    if (connectionStatus === 'CONNECTING') {
      return { text: 'Connecting Rover...', className: 'bg-orange-500 cursor-not-allowed', disabled: true };
    }
    return { text: 'Connect Rover', className: 'bg-green-500 hover:bg-green-600', disabled: false };
  };

  const { text: connButtonText, className: connButtonClassName, disabled: connButtonDisabled } = getConnectionButtonState();

  return (
    <header className="bg-[#111827] flex items-center justify-between p-3 shadow-lg">
      {/* === Left Section === */}
      <div className="flex items-center gap-4">
        {/* Logo and Title */}
        <div className="flex items-center gap-2">
          <div className="bg-orange-500 p-2 rounded-md">
            <WrenchIcon className="w-6 h-6 text-gray-900" />
          </div>
          <h1 className="text-xl font-bold text-orange-400">LAND ROVER</h1>
        </div>

        {/* === Navigation Buttons === */}
        <nav className="flex items-center bg-[#1F2937] rounded-lg">
          <button
            onClick={() => setViewMode('dashboard')}
            className={`px-4 py-2 text-sm font-semibold rounded-lg ${
              viewMode === 'dashboard' ? 'bg-green-500 text-white' : 'text-gray-300 hover:bg-gray-700'
            }`}
          >
            Dashboard
          </button>
          <button
            onClick={() => setViewMode('planning')}
            className={`px-4 py-2 text-sm font-semibold rounded-lg ${
              viewMode === 'planning' ? 'bg-green-500 text-white' : 'text-gray-300 hover:bg-gray-700'
            }`}
          >
            Edit Plan
          </button>
          <button
            onClick={() => setViewMode('servo')}
            className={`px-4 py-2 text-sm font-semibold rounded-lg ${
              viewMode === 'servo' ? 'bg-green-500 text-white' : 'text-gray-300 hover:bg-gray-700'
            }`}
          >
            Servo Control
          </button>
          <button
            onClick={() => setViewMode('live')}
            className={`px-4 py-2 text-sm font-semibold rounded-lg ${
              viewMode === 'live'
                ? 'bg-green-500 text-white'
                : 'text-gray-300 hover:bg-gray-700'
            }`}
          >
            Live Report
          </button>

          {/* ✅ New Setup Button */}
          <button
            onClick={() => setViewMode('setup')}
            className={`px-4 py-2 text-sm font-semibold rounded-lg ${
              viewMode === 'setup' ? 'bg-green-500 text-white' : 'text-gray-300 hover:bg-gray-700'
            }`}
          >
            Setup
          </button>
        </nav>
      </div>

      {/* === Right Section === */}
      <div className="flex items-center gap-4">
        <BackendIndicator status={backendStatus} />
        <ConnectionIndicator status={connectionStatus} />
        <button
          onClick={onToggleConnection}
          disabled={connButtonDisabled}
          className={`font-bold px-6 py-2 rounded-lg transition-colors ${connButtonClassName}`}
        >
          {connButtonText}
        </button>
        <button
          onClick={onToggleFullScreen}
          className="p-2 rounded-lg text-gray-300 hover:bg-gray-700 transition-colors"
          aria-label={isFullScreen ? "Exit full screen" : "Enter full screen"}
          title={isFullScreen ? "Exit full screen" : "Enter full screen"}
        >
          <FullScreenToggleIcon isFullScreen={isFullScreen} className="w-5 h-5" />
        </button>
      </div>
    </header>
  );
};

export default Header;
