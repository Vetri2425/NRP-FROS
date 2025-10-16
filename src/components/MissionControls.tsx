
import React, { useRef, useState, useCallback, useEffect } from 'react';
import { FolderIcon } from './icons/FolderIcon';
import { LoadingIcon } from './icons/LoadingIcon';
import { SuccessIcon } from './icons/SuccessIcon';
import { ErrorIcon } from './icons/ErrorIcon';
import { parseMissionFile } from '../utils/missionParser';
import { ParsedWaypoint } from '../utils/missionParser';
import { MissionFileInfo, Waypoint } from '../types';

type MissionControlsProps = {
  onMissionUpload: (waypoints: Waypoint[], info: MissionFileInfo) => void;
  onUploadInitiated: () => void;
  onClearMission: () => void;
  roverMode: string;
  roverStatus: 'armed' | 'disarmed';
  isConnected: boolean;
  onChangeMode: (mode: string) => void;
  onArmDisarm: () => void;
  currentMissionInfo: MissionFileInfo | null;
};

type UploadState = 'idle' | 'parsing' | 'completed' | 'error';

const ROVER_MODES = ['AUTO', 'MANUAL', 'GUIDED', 'HOLD'];

const MissionControls: React.FC<MissionControlsProps> = ({ 
  onMissionUpload, 
  onUploadInitiated, 
  onClearMission,
  roverMode,
  roverStatus,
  isConnected,
  onChangeMode,
  onArmDisarm,
  currentMissionInfo,
}) => {
  const fileInputRef = useRef<HTMLInputElement>(null);
  const [isDragging, setIsDragging] = useState(false);
  const [uploadState, setUploadState] = useState<UploadState>('idle');
  const [fileInfo, setFileInfo] = useState<MissionFileInfo | null>(null);
  const [errorMessage, setErrorMessage] = useState('');
  const [desiredMode, setDesiredMode] = useState<string>(roverMode);

  useEffect(() => {
    setDesiredMode(roverMode);
  }, [roverMode]);

  const resetUploaderUI = () => {
    setUploadState('idle');
    setFileInfo(null);
    setErrorMessage('');
    if (fileInputRef.current) {
      fileInputRef.current.value = '';
    }
  };

  useEffect(() => {
    if (currentMissionInfo) {
      setFileInfo(currentMissionInfo);
      setUploadState('completed');
    } else if (uploadState !== 'parsing') {
      resetUploaderUI();
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [currentMissionInfo]);
  
  const handleRemoveMission = () => {
    onClearMission();
    resetUploaderUI();
  }

  const processFiles = async (files: FileList | null) => {
    if (!files || files.length === 0) return;
    
    const file = files[0];
    const ext = file.name.split('.').pop()?.toLowerCase() || '';
    setUploadState('parsing');
    setFileInfo({
      name: file.name,
      size: file.size,
      type: file.type || ext || 'unknown',
      uploadedAt: new Date().toISOString(),
      waypointCount: 0,
      source: 'file',
    });
    setErrorMessage('');

    try {
      await new Promise(resolve => setTimeout(resolve, 500)); 

      const parsedWaypoints: ParsedWaypoint[] = await parseMissionFile(file);
      if (parsedWaypoints.length === 0 && file.name.split('.').pop()?.toLowerCase() !== 'dxf') {
        throw new Error("No valid waypoints found in the file.");
      }
      
      const mission: Waypoint[] = parsedWaypoints.map((wp, index) => ({
          ...wp,
          id: index + 1,
          command: wp.command || 'WAYPOINT',
      }));
      const info: MissionFileInfo = {
        name: file.name,
        size: file.size,
        type: file.type || ext || 'unknown',
        uploadedAt: new Date().toISOString(),
        waypointCount: mission.length,
        source: 'file',
      };
      onMissionUpload(mission, info);
    } catch (error) {
      console.error("Error parsing mission file:", error);
      setUploadState('error');
      setErrorMessage((error as Error).message);
    }
  };

  const handleClick = () => {
    onUploadInitiated();
    fileInputRef.current?.click();
  };
  
  const handleDragEnter = useCallback((e: React.DragEvent) => {
    e.preventDefault(); e.stopPropagation();
    if (uploadState !== 'parsing' && uploadState !== 'completed') setIsDragging(true);
  }, [uploadState]);
  
  const handleDragLeave = useCallback((e: React.DragEvent) => {
    e.preventDefault(); e.stopPropagation();
    setIsDragging(false);
  }, []);
  
  const handleDragOver = useCallback((e: React.DragEvent) => {
    e.preventDefault(); e.stopPropagation();
  }, []);

  const handleDrop = useCallback((e: React.DragEvent) => {
    e.preventDefault(); e.stopPropagation();
    setIsDragging(false);
    if (uploadState !== 'parsing' && uploadState !== 'completed') {
      processFiles(e.dataTransfer.files);
    }
  }, [uploadState, processFiles]);

  const formatFileSize = (bytes: number) => {
    if (bytes === 0) return '0 Bytes';
    const k = 1024;
    const sizes = ['Bytes', 'KB', 'MB', 'GB'];
    const i = Math.floor(Math.log(bytes) / Math.log(k));
    return parseFloat((bytes / Math.pow(k, i)).toFixed(2)) + ' ' + sizes[i];
  };
  
  const isArmed = roverStatus === 'armed';

  return (
    <div className="bg-[#111827] p-4 rounded-lg flex flex-col gap-5">
      <h2 className="text-lg font-bold text-white">Mission Controls</h2>
      
      <div 
        onDragEnter={handleDragEnter}
        onDragLeave={handleDragLeave}
        onDragOver={handleDragOver}
        onDrop={handleDrop}
        className={`bg-[#1F2937] border border-dashed border-gray-600 rounded-lg p-6 flex flex-col items-center justify-center text-center transition-all ${isDragging ? 'border-green-500 scale-105' : ''} ${(uploadState === 'parsing') ? 'cursor-default' : 'cursor-pointer'}`}
      >
        <input type="file" ref={fileInputRef} className="hidden" onChange={(e) => processFiles(e.target.files)} accept=".waypoint,.csv,.dxf" disabled={uploadState === 'parsing'} />

        {(uploadState === 'idle' || (uploadState === 'completed' && !fileInfo)) && (
          <div onClick={handleClick} className="w-full">
            <FolderIcon className="w-12 h-12 text-orange-500 mb-2 mx-auto" />
            <p className="font-semibold text-orange-400">Upload Mission File</p>
            <p className="text-xs text-gray-400">.waypoint, .csv, .dxf</p>
          </div>
        )}

        {uploadState === 'parsing' && (
            <div className="flex flex-col items-center gap-3">
                <LoadingIcon className="w-10 h-10 text-orange-500" />
                <p className="font-semibold text-orange-400">Converting...</p>
                {fileInfo && <p className="text-xs text-gray-400 truncate max-w-full px-2">{fileInfo.name}</p>}
            </div>
        )}
        
        {uploadState === 'completed' && fileInfo && (
            <div className="flex flex-col items-center gap-2 w-full">
                <SuccessIcon className="w-10 h-10 text-green-500" />
                <p className="font-semibold text-green-400">Mission Loaded</p>
                <p className="text-sm text-gray-300 truncate max-w-full px-2 font-mono" title={fileInfo.name}>{fileInfo.name}</p>
                <p className="text-xs text-gray-500">{formatFileSize(fileInfo.size)}</p>
                <div className="mt-3 w-full flex flex-col gap-2">
                    <button onClick={handleClick} className="w-full bg-gray-600 text-gray-200 text-sm font-bold py-2 rounded-lg hover:bg-gray-700 transition-colors">Upload Another</button>
                    <button onClick={handleRemoveMission} className="w-full bg-red-800 text-red-100 text-sm font-bold py-2 rounded-lg hover:bg-red-700 transition-colors">Remove Mission</button>
                </div>
            </div>
        )}

        {uploadState === 'error' && (
            <div className="flex flex-col items-center gap-2 w-full">
                <ErrorIcon className="w-10 h-10 text-red-500" />
                <p className="font-semibold text-red-400">Upload Failed</p>
                <p className="text-xs text-gray-400 text-center max-w-full px-2">{errorMessage}</p>
                <button onClick={resetUploaderUI} className="mt-3 w-full bg-gray-600 text-gray-200 text-sm font-bold py-2 rounded-lg hover:bg-gray-700 transition-colors">Try Again</button>
            </div>
        )}
      </div>

      <div>
        <label htmlFor="mode-select" className="block text-sm font-medium text-gray-300 mb-1">Mode</label>
        <div className="relative">
          <select 
            id="mode-select" 
            value={desiredMode}
            onChange={(e) => setDesiredMode(e.target.value)}
            disabled={!isConnected}
            className="w-full bg-[#1F2937] border border-gray-600 rounded-md px-3 py-2 appearance-none focus:outline-none focus:ring-2 focus:ring-green-500"
          >
            {ROVER_MODES.map(mode => (
              <option key={mode} value={mode}>{mode}</option>
            ))}
             {/* If rover reports a mode not in our list, show it but disabled so the user knows the current state */}
            {!ROVER_MODES.includes(roverMode) && roverMode !== 'UNKNOWN' && (
              <option key={roverMode} value={roverMode} disabled>{roverMode}</option>
            )}
          </select>
          <div className="pointer-events-none absolute inset-y-0 right-0 flex items-center px-2 text-gray-400">
            <svg className="fill-current h-4 w-4" xmlns="http://www.w3.org/2000/svg" viewBox="0 0 20 20"><path d="M9.293 12.95l.707.707L15.657 8l-1.414-1.414L10 10.828 5.757 6.586 4.343 8z"/></svg>
          </div>
        </div>
        <p className="mt-2 text-xs text-gray-400">
          {isConnected
            ? `Status: ${roverStatus === 'armed' ? 'Armed' : 'Disarmed'} • Mode: ${roverMode || 'UNKNOWN'}`
            : 'Rover disconnected'}
        </p>
        <button
          onClick={() => onChangeMode(desiredMode)}
          disabled={!isConnected || desiredMode === roverMode}
          className={`mt-3 w-full bg-blue-600 text-white text-sm font-semibold py-2 rounded-lg transition-colors disabled:bg-gray-600 disabled:text-gray-300 disabled:cursor-not-allowed ${
            !isConnected || desiredMode === roverMode ? '' : 'hover:bg-blue-700'
          }`}
        >
          Set Mode
        </button>
      </div>

      <div className="flex flex-col gap-3">
        <button 
          onClick={onArmDisarm}
          disabled={!isConnected}
          className={`w-full text-white font-bold py-3 rounded-lg transition-colors ${
            isArmed 
              ? 'bg-red-600 hover:bg-red-700' 
              : 'bg-green-500 hover:bg-green-600'
          }`}
        >
          {isArmed ? 'Disarm' : 'Arm'}
        </button>
      </div>
    </div>
  );
};

export default MissionControls;
