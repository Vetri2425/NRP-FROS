import React, { useRef, useState } from 'react';
import { toast } from 'react-toastify';
import CircleTool from '../../tools/CircleTool';
import PolygonTool from '../../tools/PolygonTool';
import SurveyGridTool from '../../tools/SurveyGridTool';
import SplineWaypointTool from '../../tools/SplineWaypointTool';
import { ChevronDownIcon } from '../icons/ChevronDownIcon';
import { MissionFileInfo, Waypoint } from '../../types';
import { parseMissionFile, ParsedWaypoint } from '../../utils/missionParser';
import { validateWaypoints, hasCriticalErrors, getCriticalErrors, getWarnings, formatValidationErrors, sanitizeWaypointsForUpload } from '../../utils/waypointValidator';
import UploadProgressDialog from '../UploadProgressDialog';
import MissionUploadPreviewDialog from '../MissionUploadPreviewDialog';
import MissionExportPreviewDialog from '../MissionExportPreviewDialog';
import { useKeyboardShortcuts } from '../../hooks/useKeyboardShortcuts';
import { KeyboardShortcutsHelp } from '../KeyboardShortcutsHelp';
import { HelpIcon } from '../icons/HelpIcon';

type PlanControlsProps = {
  onUpload: (waypoints: Waypoint[], info: MissionFileInfo) => void;
  onExport: () => void;
  onUploadInitiated: () => void;
  missionWaypoints: Waypoint[];

  onWriteToRover: () => Promise<boolean>;
  onReadFromRover: () => Promise<void>;
  isConnected: boolean;
  uploadProgress: number;
  missionFileInfo: MissionFileInfo | null;
  onClearMission: () => void;
};

const PlanControls: React.FC<PlanControlsProps> = ({
  onUpload,
  onExport,
  onUploadInitiated,
  missionWaypoints,
  onWriteToRover,
  onReadFromRover,
  isConnected,
  uploadProgress,
  missionFileInfo,
  onClearMission,
}) => {
  const fileInputRef = useRef<HTMLInputElement>(null);
  const [isWriting, setIsWriting] = useState(false);
  const [lastWriteStatus, setLastWriteStatus] = useState<null | 'success' | 'error'>(null);
  const [isReading, setIsReading] = useState(false);
  const [lastReadStatus, setLastReadStatus] = useState<null | 'success' | 'error' | 'empty'>(null);
  const [readErrorMessage, setReadErrorMessage] = useState<string | null>(null);
  const [currentFile, setCurrentFile] = useState<MissionFileInfo | null>(null);
  const [showCircleTool, setShowCircleTool] = useState(false);
  const [showPolygonTool, setShowPolygonTool] = useState(false);
  const [showSurveyGridTool, setShowSurveyGridTool] = useState(false);
  const [showSplineTool, setShowSplineTool] = useState(false);

  // Upload progress tracking state
  const [uploadProgressState, setUploadProgressState] = useState({
    isOpen: false,
    progress: 0,
    currentWaypoint: 0,
    totalWaypoints: 0,
    uploadSpeed: 0,
    estimatedTimeRemaining: 0,
    status: 'uploading' as 'uploading' | 'completed' | 'failed',
    errorMessage: ''
  });

  // Preview dialog state
  const [showUploadPreview, setShowUploadPreview] = useState(false);
  const [showExportPreview, setShowExportPreview] = useState(false);
  const [selectedExportFormat, setSelectedExportFormat] = useState<'qgc' | 'json' | 'kml' | 'csv'>('qgc');
  
  // Keyboard shortcuts help state
  const [showShortcutsHelp, setShowShortcutsHelp] = useState(false);

  React.useEffect(() => {
    if (missionFileInfo) {
      setCurrentFile(missionFileInfo);
    } else {
      setCurrentFile(null);
    }
  }, [missionFileInfo]);


  const validateFile = (file: File): boolean => {
    const validExtensions = ['waypoint', 'waypoints', 'csv', 'dxf', 'json', 'kml'];
    const ext = file.name.split('.').pop()?.toLowerCase() || '';
    if (!validExtensions.includes(ext)) {
      toast.error(`Unsupported file format: .${ext}. Allowed: ${validExtensions.join(', ')}`);
      return false;
    }
    if (file.size === 0) {
      toast.error('File is empty. Please upload a valid mission file.');
      return false;
    }
    return true;
  };

  const processFiles = async (files: FileList | null) => {
    if (files && files.length > 0) {
      const file = files[0];
      if (!validateFile(file)) return;

      const ext = file.name.split('.').pop()?.toLowerCase() || '';

      try {
        const parsedWaypoints: ParsedWaypoint[] = await parseMissionFile(file);
        const mission: Waypoint[] = parsedWaypoints.map((wp, index) => ({
          ...wp,
          id: index + 1,
          command: wp.command || 'WAYPOINT',
        }));

        // Validate waypoints
        const validationErrors = validateWaypoints(mission);
        const criticalErrors = getCriticalErrors(validationErrors);
        const warnings = getWarnings(validationErrors);

        // Block upload if critical errors found
        if (hasCriticalErrors(validationErrors)) {
          const errorMessage = formatValidationErrors(criticalErrors, 3);
          toast.error(`Validation failed:\n${errorMessage}`);
          console.error('Critical validation errors:', criticalErrors);
          return;
        }

        // Show warnings but allow upload
        if (warnings.length > 0) {
          const warningMessage = formatValidationErrors(warnings, 3);
          toast.warning(`${warnings.length} warning(s) found:\n${warningMessage}`);
          console.warn('Validation warnings:', warnings);
        }

        const info: MissionFileInfo = {
          name: file.name,
          size: file.size,
          type: file.type || ext || 'unknown',
          uploadedAt: new Date().toISOString(),
          waypointCount: mission.length,
          source: 'file',
        };
        setCurrentFile(info);
        onUpload(mission, info);
        toast.success(`Successfully loaded ${mission.length} waypoints from ${file.name}`);
      } catch (error) {
        console.error('Error parsing mission file:', error);
        toast.error(`Failed to parse ${file.name}: ${(error as Error).message}`);
      }
    }
  };

  const handleUploadClick = () => {
    onUploadInitiated();
    fileInputRef.current?.click();
  };

  const handleWriteToRover = async () => {
    if (!isConnected) {
      toast.error('Rover not connected.');
      return;
    }
    if (!missionWaypoints.length) {
      toast.warning('No waypoints to upload.');
      return;
    }

    // Validate waypoints before uploading to rover
    const validationErrors = validateWaypoints(missionWaypoints);
    const criticalErrors = getCriticalErrors(validationErrors);
    const warnings = getWarnings(validationErrors);

    // Block upload if critical errors found
    if (hasCriticalErrors(validationErrors)) {
      const errorMessage = formatValidationErrors(criticalErrors, 3);
      toast.error(`Cannot upload to rover. Validation failed:\n${errorMessage}`);
      console.error('Critical validation errors before rover upload:', criticalErrors);
      return;
    }

    // Show warnings but allow upload
    if (warnings.length > 0) {
      toast.warning(`${warnings.length} warning(s) detected. Review before mission execution.`);
      console.warn('Validation warnings before rover upload:', warnings);
    }

    // Check if user disabled upload preview
    const dontShowUploadPreview = localStorage.getItem('dontShowUploadPreview') === 'true';
    
    // Show preview dialog if not disabled
    if (!dontShowUploadPreview) {
      setShowUploadPreview(true);
      return;
    }

    // Proceed with upload
    await executeRoverUpload();
  };

  const executeRoverUpload = async () => {
    const startTime = Date.now();
    setUploadProgressState({
      isOpen: true,
      progress: 0,
      currentWaypoint: 0,
      totalWaypoints: missionWaypoints.length,
      uploadSpeed: 0,
      estimatedTimeRemaining: missionWaypoints.length * 0.5, // rough estimate
      status: 'uploading',
      errorMessage: ''
    });

    setIsWriting(true);
    try {
      // Simulate progress updates (in real implementation, this would come from the upload service)
      // For now, we'll update progress as the upload progresses
      const progressInterval = setInterval(() => {
        setUploadProgressState(prev => {
          if (prev.status !== 'uploading') {
            clearInterval(progressInterval);
            return prev;
          }

          const elapsedSeconds = (Date.now() - startTime) / 1000;
          const estimatedProgress = Math.min((elapsedSeconds / (missionWaypoints.length * 0.5)) * 100, 95);
          const currentWp = Math.floor((estimatedProgress / 100) * missionWaypoints.length);
          const uploadSpeed = currentWp / Math.max(elapsedSeconds, 0.1);
          const remainingWaypoints = missionWaypoints.length - currentWp;
          const estimatedTimeRemaining = remainingWaypoints / Math.max(uploadSpeed, 0.1);

          return {
            ...prev,
            progress: estimatedProgress,
            currentWaypoint: currentWp,
            uploadSpeed,
            estimatedTimeRemaining: Math.max(0, estimatedTimeRemaining)
          };
        });
      }, 200);

      const ok = await onWriteToRover();
      
      clearInterval(progressInterval);

      if (ok) {
        setUploadProgressState(prev => ({
          ...prev,
          progress: 100,
          currentWaypoint: missionWaypoints.length,
          status: 'completed'
        }));

        toast.success(`Mission uploaded with ${missionWaypoints.length} waypoints`);

        // Auto-close after 2 seconds
        setTimeout(() => {
          setUploadProgressState(prev => ({ ...prev, isOpen: false }));
        }, 2000);

        setLastWriteStatus('success');
      } else {
        setUploadProgressState(prev => ({
          ...prev,
          status: 'failed',
          errorMessage: 'Upload failed'
        }));
        setLastWriteStatus('error');
      }
    } catch (error) {
      console.error('PlanControls.handleWriteToRover error', error);
      
      setUploadProgressState(prev => ({
        ...prev,
        status: 'failed',
        errorMessage: error instanceof Error ? error.message : 'Unknown error'
      }));
      
      setLastWriteStatus('error');
      toast.error(error instanceof Error ? error.message : 'Upload failed');
    } finally {
      setIsWriting(false);
      setTimeout(() => setLastWriteStatus(null), 3000);
    }
  };

  const handleReadFromRover = async () => {
    if (!isConnected) {
      setReadErrorMessage('Rover not connected. Please connect to the rover first.');
      setLastReadStatus('error');
      setTimeout(() => {
        setReadErrorMessage(null);
        setLastReadStatus(null);
      }, 5000);
      return;
    }

    setIsReading(true);
    setReadErrorMessage(null);
    setLastReadStatus(null);

    try {
      await onReadFromRover();
      
      // Check if we actually got waypoints
      // Note: This check happens after onReadFromRover updates the mission
      // We'll add a small delay to let state update
      setTimeout(() => {
        if (missionWaypoints.length === 0) {
          setLastReadStatus('empty');
          setReadErrorMessage('Rover mission is empty. No waypoints to download.');
        } else {
          setLastReadStatus('success');
          setReadErrorMessage(null);
        }
        
        // Clear status after 5 seconds
        setTimeout(() => {
          setLastReadStatus(null);
          setReadErrorMessage(null);
        }, 5000);
      }, 100);
      
    } catch (error) {
      const errorMsg = error instanceof Error ? error.message : 'Failed to download mission from rover';
      setReadErrorMessage(errorMsg);
      setLastReadStatus('error');
      console.error('Mission download error:', error);
      
      // Clear error after 5 seconds
      setTimeout(() => {
        setReadErrorMessage(null);
        setLastReadStatus(null);
      }, 5000);
    } finally {
      setIsReading(false);
    }
  };

  // Register keyboard shortcuts (after all function definitions)
  const { getAllShortcuts } = useKeyboardShortcuts({
    shortcuts: [
      {
        key: 'u',
        ctrl: true,
        description: 'Upload mission file',
        action: handleUploadClick,
        disabled: false
      },
      {
        key: 'e',
        ctrl: true,
        description: 'Export mission',
        action: () => setShowExportPreview(true),
        disabled: missionWaypoints.length === 0
      },
      {
        key: 's',
        ctrl: true,
        description: 'Save/Export mission',
        action: () => setShowExportPreview(true),
        disabled: missionWaypoints.length === 0
      },
      {
        key: 'o',
        ctrl: true,
        description: 'Clear mission',
        action: () => {
          if (missionWaypoints.length > 0 && window.confirm('Clear all waypoints?')) {
            onClearMission();
          }
        },
        disabled: missionWaypoints.length === 0
      },
      {
        key: 'd',
        ctrl: true,
        description: 'Download from rover',
        action: handleReadFromRover,
        disabled: !isConnected || isReading
      },
      {
        key: '?',
        shift: true,
        description: 'Show keyboard shortcuts',
        action: () => setShowShortcutsHelp(true),
        disabled: false
      }
    ],
    enabled: true,
    preventDefault: true
  });

  const homeLocation = missionWaypoints.length > 0 ? missionWaypoints[0] : null;

  // Servo control logic removed from Plan tab

  return (
    <>
      <div className="bg-slate-800 p-2 rounded-lg h-full overflow-y-auto custom-scrollbar">
        {/* Hidden file input */}
        <input
          ref={fileInputRef}
          type="file"
          className="hidden"
          onChange={(e) => processFiles(e.target.files)}
          accept=".waypoint,.waypoints,.csv,.dxf,.json,.kml"
        />

        <div className="space-y-2">
          {/* Grid picker (unchanged) */}
          <div className="flex items-center justify-between text-white">
            <span className="text-xs">Grid</span>
            <div className="flex items-center space-x-1 text-xs">
              <select className="bg-slate-700 text-white px-1.5 py-0.5 rounded text-xs">
                <option>GoogleHybridMap</option>
                <option>OpenStreetMap</option>
                <option>GoogleSatellite</option>
              </select>
              <ChevronDownIcon className="w-3 h-3" />
            </div>
          </div>

          <div className="text-xs text-green-400">Status: loaded tiles</div>

          {/* Mission file actions */}
          <div className="space-y-1.5">
            <button
              onClick={handleUploadClick}
              className="w-full bg-green-600 hover:bg-green-700 text-white px-3 py-1.5 rounded-md transition-all duration-200 text-xs font-medium"
            >
              📁 Upload Mission
            </button>

            <button
              onClick={() => setShowExportPreview(true)}
              disabled={missionWaypoints.length === 0}
              className={`w-full px-3 py-1.5 rounded-md transition-all duration-200 text-xs font-medium ${
                missionWaypoints.length > 0
                  ? 'bg-blue-600 hover:bg-blue-700 text-white'
                  : 'bg-gray-500 text-gray-300 cursor-not-allowed'
              }`}
            >
              💾 Export Mission
            </button>

            {currentFile && (
              <div className="bg-slate-700 p-1.5 mt-2 rounded text-xs text-slate-300">
                <div>
                  📄 <strong>{currentFile.name}</strong>
                </div>
                <div>Type: {currentFile.type || 'Unknown'}</div>
                <div>Size: {(currentFile.size / 1024).toFixed(1)} KB</div>
                <div>Waypoints: {missionWaypoints.length}</div>
              </div>
            )}

            {/* Auto tools */}
            <div className="border-t border-slate-600 pt-2 mt-2">
              <div className="text-xs text-slate-300 mb-1.5">Auto Waypoint Tools</div>
              <div className="space-y-1.5">
                <button
                  onClick={() => setShowSurveyGridTool(true)}
                  className="w-full bg-green-700 hover:bg-green-800 text-white py-1.5 rounded-md text-xs font-medium flex items-center justify-center gap-1.5"
                >
                  <span>📐</span>
                  <span>Survey Grid</span>
                  <span className="text-xs bg-green-900 px-1 py-0.5 rounded">MP</span>
                </button>
                <button
                  onClick={() => setShowPolygonTool(true)}
                  className="w-full bg-purple-700 hover:bg-purple-800 text-white py-1.5 rounded-md text-xs font-medium"
                >
                  🗺️ Polygon Mission
                </button>
                <button
                  onClick={() => setShowCircleTool(true)}
                  className="w-full bg-sky-700 hover:bg-sky-800 text-white py-1.5 rounded-md text-xs font-medium"
                >
                  🌀 Circle
                </button>
                <button
                  onClick={() => setShowSplineTool(true)}
                  disabled={missionWaypoints.length === 0}
                  className="w-full bg-indigo-700 hover:bg-indigo-800 disabled:opacity-50 disabled:cursor-not-allowed text-white py-1.5 rounded-md text-xs font-medium"
                  title="Convert waypoints to smooth spline paths"
                >
                  〰️ Spline
                </button>
              </div>
            </div>

            {/* Servo Control moved to App level */}

            {/* Rover IO */}
            <div className="border-t border-slate-600 pt-2 mt-2">
              <div className="text-xs text-slate-300 mb-1.5">Rover Communication</div>

              <button
                onClick={handleReadFromRover}
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
                onClick={handleWriteToRover}
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

              <div
                className={`text-xs mt-1.5 px-1.5 py-0.5 rounded text-center ${
                  isConnected ? 'text-green-300 bg-green-900/30' : 'text-red-300 bg-red-900/30'
                }`}
              >
                {isConnected ? '✅ Connected' : '❌ Disconnected'}
              </div>
            </div>
          </div>

          {/* Home Info */}
          <div className="bg-slate-700 p-2 rounded-md mt-2">
            <div className="text-xs font-medium text-white mb-1">Home Location</div>
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

          {/* Keyboard Shortcuts Help Button */}
          <div className="mt-2">
            <button
              onClick={() => setShowShortcutsHelp(true)}
              className="w-full bg-slate-700 hover:bg-slate-600 text-slate-300 hover:text-white py-1.5 rounded-md text-xs font-medium flex items-center justify-center gap-1.5 transition-colors"
              title="Show keyboard shortcuts (Shift+?)"
            >
              <HelpIcon className="w-3.5 h-3.5" />
              <span>Keyboard Shortcuts</span>
            </button>
          </div>
        </div>
      </div>

      {showCircleTool && (
        <CircleTool
        onGenerate={(wps) =>
          onUpload(wps, {
            name: 'Circle Mission',
            size: 0,
            type: 'generated',
            uploadedAt: new Date().toISOString(),
            waypointCount: wps.length,
            source: 'generated',
          })
        }
        onClose={() => setShowCircleTool(false)}
      />
      )}
      {showPolygonTool && (
        <PolygonTool
        onGenerate={(wps) =>
          onUpload(wps, {
            name: 'Polygon Mission',
            size: 0,
            type: 'generated',
            uploadedAt: new Date().toISOString(),
            waypointCount: wps.length,
            source: 'generated',
          })
        }
        onClose={() => setShowPolygonTool(false)}
      />
      )}
      {showSurveyGridTool && (
        <SurveyGridTool
        onGenerate={(wps) =>
          onUpload(wps, {
            name: 'Survey Grid Mission',
            size: 0,
            type: 'generated',
            uploadedAt: new Date().toISOString(),
            waypointCount: wps.length,
            source: 'generated',
          })
        }
        onClose={() => setShowSurveyGridTool(false)}
      />
      )}
      {showSplineTool && (
        <SplineWaypointTool
        currentWaypoints={missionWaypoints}
        onConvert={(wps) =>
          onUpload(wps, {
            name: currentFile?.name.replace('.waypoints', '_spline.waypoints') || 'Spline Mission',
            size: 0,
            type: 'generated',
            uploadedAt: new Date().toISOString(),
            waypointCount: wps.length,
            source: 'generated',
          })
        }
        onClose={() => setShowSplineTool(false)}
      />
      )}

      {/* Upload Progress Dialog */}
      <UploadProgressDialog
        isOpen={uploadProgressState.isOpen}
        progress={uploadProgressState.progress}
        currentWaypoint={uploadProgressState.currentWaypoint}
        totalWaypoints={uploadProgressState.totalWaypoints}
        uploadSpeed={uploadProgressState.uploadSpeed}
        estimatedTimeRemaining={uploadProgressState.estimatedTimeRemaining}
        status={uploadProgressState.status}
        errorMessage={uploadProgressState.errorMessage}
        onCancel={() => setUploadProgressState(prev => ({ ...prev, isOpen: false }))}
      />

      {/* Mission Upload Preview Dialog */}
      <MissionUploadPreviewDialog
        isOpen={showUploadPreview}
        waypoints={missionWaypoints}
        validationErrors={validateWaypoints(missionWaypoints)}
        onConfirm={() => {
          setShowUploadPreview(false);
          executeRoverUpload();
        }}
        onCancel={() => setShowUploadPreview(false)}
      />

      {/* Mission Export Preview Dialog */}
      <MissionExportPreviewDialog
        isOpen={showExportPreview}
        waypoints={missionWaypoints}
        currentFormat={selectedExportFormat}
        onConfirm={(format) => {
          setSelectedExportFormat(format);
          setShowExportPreview(false);
          onExport();
        }}
        onCancel={() => setShowExportPreview(false)}
      />

      {/* Keyboard Shortcuts Help Modal */}
      <KeyboardShortcutsHelp
        isOpen={showShortcutsHelp}
        onClose={() => setShowShortcutsHelp(false)}
        shortcuts={getAllShortcuts()}
      />
    </>
  );
};

export default PlanControls;





