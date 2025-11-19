import React, { useState } from 'react';
import { toast } from 'react-toastify';
import CircleTool from '../../tools/CircleTool';
import SurveyGridTool from '../../tools/SurveyGridTool';
import { MissionFileInfo, Waypoint } from '../../types';
import { validateWaypoints, hasCriticalErrors, getCriticalErrors, getWarnings, formatValidationErrors, sanitizeWaypointsForUpload } from '../../utils/waypointValidator';
import UploadProgressDialog from '../UploadProgressDialog';
import MissionUploadPreviewDialog from '../MissionUploadPreviewDialog';
import MissionExportPreviewDialog from '../MissionExportPreviewDialog';
import { useKeyboardShortcuts } from '../../hooks/useKeyboardShortcuts';
import { useDialog } from '../../hooks/useDialog';
import GenericDialog from '../GenericDialog';
import MapSelector from './MapSelector';
import FileSection from './FileSection';
import Tools from './Tools';
import PlanActions from './PlanActions';
import { BACKEND_URL } from '../../config';

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
  // Optional home position provided by App (set via map click or telemetry)
  homePosition?: { lat: number; lng: number; alt?: number } | null;
  // Trigger interactive set-home mode (next map click sets home)
  onStartSetHome?: () => void;
  // Drawing tool state
  activeDrawingTool?: string | null;
  onDrawingToolSelect?: (tool: string | null) => void;
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
  homePosition,
  onStartSetHome,
  activeDrawingTool,
  onDrawingToolSelect,
}) => {
  const [isWriting, setIsWriting] = useState(false);
  const [lastWriteStatus, setLastWriteStatus] = useState<null | 'success' | 'error'>(null);
  const { dialogState, showConfirm } = useDialog();
  const [isReading, setIsReading] = useState(false);
  const [lastReadStatus, setLastReadStatus] = useState<null | 'success' | 'error' | 'empty'>(null);
  const [readErrorMessage, setReadErrorMessage] = useState<string | null>(null);
  const [isUploadingToJetson, setIsUploadingToJetson] = useState(false);
  const [lastJetsonUploadStatus, setLastJetsonUploadStatus] = useState<null | 'success' | 'error'>(null);
  const [currentFile, setCurrentFile] = useState<MissionFileInfo | null>(null);
  const [showCircleTool, setShowCircleTool] = useState(false);
  const [showSurveyGridTool, setShowSurveyGridTool] = useState(false);

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
  
  // Keyboard shortcuts help state - commented out as Shift+? is known shortcut
  // const [showShortcutsHelp, setShowShortcutsHelp] = useState(false);

  React.useEffect(() => {
    if (missionFileInfo) {
      setCurrentFile(missionFileInfo);
    } else {
      setCurrentFile(null);
    }
  }, [missionFileInfo]);


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

  const handleUploadToJetson = async () => {
    if (!missionWaypoints.length) {
      toast.warning('No waypoints to upload to Jetson.');
      return;
    }

    setIsUploadingToJetson(true);
    setLastJetsonUploadStatus(null);

    try {
      // Send waypoints to load_controller endpoint
      const response = await fetch(`${BACKEND_URL}/api/mission/load_controller`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ waypoints: missionWaypoints })
      });

      if (response.ok) {
        const result = await response.json();
        setLastJetsonUploadStatus('success');
        toast.success(`Waypoints uploaded to Jetson successfully! (${result.message})`);
      } else {
        throw new Error('Upload to Jetson failed');
      }
    } catch (error) {
      console.error('Jetson upload error:', error);
      setLastJetsonUploadStatus('error');
      toast.error('Failed to upload waypoints to Jetson');
    } finally {
      setIsUploadingToJetson(false);
      setTimeout(() => setLastJetsonUploadStatus(null), 3000);
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

  // Register keyboard shortcuts (after all function definitions) - keeping only essential shortcuts
  const { getAllShortcuts } = useKeyboardShortcuts({
    shortcuts: [
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
        action: async () => {
          if (missionWaypoints.length > 0) {
            const confirmed = await showConfirm('Clear Mission', 'Clear all waypoints?');
            if (confirmed) {
              onClearMission();
            }
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
      }
      // Removed keyboard shortcuts help shortcut as Shift+? is known
      // {
      //   key: '?',
      //   shift: true,
      //   description: 'Show keyboard shortcuts',
      //   action: () => setShowShortcutsHelp(true),
      //   disabled: false
      // }
    ],
    enabled: true,
    preventDefault: true
  });

  // Servo control logic removed from Plan tab

  return (
    <>
      <div className="bg-slate-800 p-2 rounded-lg h-full overflow-y-auto custom-scrollbar">
        <div className="space-y-2">
          <MapSelector />

          <FileSection
            onUpload={onUpload}
            onExport={() => setShowExportPreview(true)}
            onUploadInitiated={onUploadInitiated}
            missionWaypoints={missionWaypoints}
            currentFile={currentFile}
            onFileProcessed={setCurrentFile}
          />

          <Tools
            onShowSurveyGridTool={() => setShowSurveyGridTool(true)}
            onShowCircleTool={() => setShowCircleTool(true)}
            onDrawingToolSelect={onDrawingToolSelect}
            activeDrawingTool={activeDrawingTool}
          />

          <PlanActions
            onWriteToRover={handleWriteToRover}
            onReadFromRover={handleReadFromRover}
            isConnected={isConnected}
            uploadProgress={uploadProgress}
            missionWaypoints={missionWaypoints}
            isWriting={isWriting}
            lastWriteStatus={lastWriteStatus}
            isReading={isReading}
            lastReadStatus={lastReadStatus}
            readErrorMessage={readErrorMessage}
            onUploadToJetson={handleUploadToJetson}
            isUploadingToJetson={isUploadingToJetson}
            lastJetsonUploadStatus={lastJetsonUploadStatus}
            homePosition={homePosition}
            onStartSetHome={onStartSetHome}
          />
        </div>
      </div>

      {showCircleTool && (
        <CircleTool
          defaultCenter={homePosition ?? undefined}
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
      {showSurveyGridTool && (
        <SurveyGridTool
          defaultCenter={homePosition ?? undefined}
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

      {/* Keyboard Shortcuts Help Button - commented out as Shift+? is known shortcut */}
      {/*
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
      */}

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

      {/* Generic Dialog for confirmations and alerts */}
      <GenericDialog
        isOpen={dialogState.isOpen}
        type={dialogState.type}
        title={dialogState.title}
        message={dialogState.message}
        onConfirm={dialogState.onConfirm}
        onCancel={dialogState.onCancel}
        confirmText={dialogState.confirmText}
        cancelText={dialogState.cancelText}
      />
    </>
  );
};

export default PlanControls;





