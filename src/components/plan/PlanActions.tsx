import React from 'react';
import { Waypoint } from '../../types';
import { RoverIcon } from '../icons/RoverIcon';

type PlanActionsProps = {
  // Rover communication props
  onWriteToRover: () => Promise<void>;
  onReadFromRover: () => Promise<void>;
  isConnected: boolean;
  uploadProgress: number;
  missionWaypoints: Waypoint[];
  isWriting: boolean;
  lastWriteStatus: null | 'success' | 'error';
  isReading: boolean;
  lastReadStatus: null | 'success' | 'error' | 'empty';
  readErrorMessage: string | null;

  // Home location props
  homePosition?: { lat: number; lng: number; alt?: number } | null;
  onStartSetHome?: () => void;
};

const PlanActions: React.FC<PlanActionsProps> = ({
  onWriteToRover,
  onReadFromRover,
  isConnected,
  uploadProgress,
  missionWaypoints,
  isWriting,
  lastWriteStatus,
  isReading,
  lastReadStatus,
  readErrorMessage,
  homePosition,
  onStartSetHome,
}) => {
  const homeLocation = (homePosition ?? (missionWaypoints.length > 0 ? missionWaypoints[0] : null));

  return (
    <div className="bg-[#111827] rounded-lg overflow-hidden mb-2">
      <header className="bg-indigo-700 px-3 py-2 flex items-center justify-between">
        <div className="flex items-center gap-2">
          <RoverIcon className="w-5 h-5 text-indigo-300" />
          <span className="font-semibold text-white tracking-wide text-sm">Rover Control</span>
        </div>
      </header>

      <div className="p-2">
        {/* Rover IO */}
        <div className="border-t border-slate-600 pt-2 mt-2">

          <button
            onClick={onReadFromRover}
            disabled={!isConnected || isReading}
            className={`w-full px-3 py-1.5 rounded-md transition-all duration-200 text-xs font-medium mb-1.5 ${
              isConnected && !isReading
                ? lastReadStatus === 'success'
                  ? 'bg-green-600 hover:bg-green-700 text-white'
                  : lastReadStatus === 'error'
                  ? 'bg-red-600 hover:bg-red-700 text-white'
                  : lastReadStatus === 'empty'
                  ? 'bg-yellow-600 hover:bg-yellow-700 text-white'
                  : 'bg-purple-600 hover:bg-purple-700 text-white'
                : 'bg-gray-500 text-gray-300 cursor-not-allowed'
            }`}
          >
            {isReading 
              ? '⏳ Reading...' 
              : lastReadStatus === 'success'
              ? '✅ Downloaded'
              : lastReadStatus === 'error'
              ? '❌ Failed'
              : lastReadStatus === 'empty'
              ? '⚠️ Empty'
              : '📥 Read from Rover'}
          </button>

          {/* Error/Success Message Display */}
          {readErrorMessage && (
            <div className={`mb-1.5 p-1.5 rounded text-xs ${
              lastReadStatus === 'error' 
                ? 'bg-red-900/50 text-red-200 border border-red-700'
                : lastReadStatus === 'empty'
                ? 'bg-yellow-900/50 text-yellow-200 border border-yellow-700'
                : 'bg-blue-900/50 text-blue-200 border border-blue-700'
            }`}>
              <div className="font-semibold mb-0.5">
                {lastReadStatus === 'error' ? '⚠️ Download Failed' : 'ℹ️ Info'}
              </div>
              <div>{readErrorMessage}</div>
            </div>
          )}

          <button
            onClick={onWriteToRover}
            disabled={!isConnected || missionWaypoints.length === 0 || isWriting}
            className={`w-full px-3 py-1.5 rounded-md transition-all duration-200 text-xs font-medium ${
              isConnected && missionWaypoints.length > 0 && !isWriting
                ? 'bg-orange-600 hover:bg-orange-700 text-white'
                : 'bg-gray-500 text-gray-300 cursor-not-allowed'
            }`}
          >
            {isWriting
              ? `Uploading (${missionWaypoints.length} WP)`
              : lastWriteStatus === 'success'
              ? '✅ Uploaded'
              : lastWriteStatus === 'error'
              ? '❌ Failed'
              : '📤 Write to Rover'}
          </button>

          {uploadProgress > 0 && (
            <div className="w-full bg-gray-700 rounded-full h-1.5 mt-1.5">
              <div
                className="bg-green-600 h-1.5 rounded-full"
                style={{ width: `${uploadProgress}%` }}
              ></div>
            </div>
          )}

        </div>

        {/* Home Info */}
        <div className="bg-slate-700 p-2 rounded-md mt-2">
          <div className="flex items-center justify-between mb-1">
            <div className="text-xs font-medium text-white">Home Location</div>
            <div>
              <button onClick={() => onStartSetHome && onStartSetHome()} className="text-xs bg-indigo-600 hover:bg-indigo-500 px-2 py-0.5 rounded text-white">Set Home</button>
            </div>
          </div>
          <div className="grid grid-cols-2 gap-1.5 text-xs">
            <div>
              <span className="text-slate-300">Lat:</span>
              <div className="text-white font-mono text-xs">
                {homeLocation ? homeLocation.lat.toFixed(8) : 'N/A'}
              </div>
            </div>
            <div>
              <span className="text-slate-300">Long:</span>
              <div className="text-white font-mono text-xs">
                {homeLocation ? homeLocation.lng.toFixed(8) : 'N/A'}
              </div>
            </div>
            <div className="col-span-2">
              <span className="text-slate-300">ASL:</span>
              <div className="text-white font-mono text-xs">
                {homeLocation ? homeLocation.alt.toFixed(2) : 'N/A'}
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};export default PlanActions;