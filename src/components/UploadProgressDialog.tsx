import React, { useEffect } from 'react';
import { ProgressBar } from './ProgressBar';
import './ProgressBar.css';

interface UploadProgressDialogProps {
  isOpen: boolean;
  progress: number;              // 0-100
  currentWaypoint: number;       // e.g., 3
  totalWaypoints: number;        // e.g., 6
  uploadSpeed: number;           // waypoints per second
  estimatedTimeRemaining: number; // seconds
  status: 'uploading' | 'completed' | 'failed';
  errorMessage?: string;
  onCancel: () => void;
}

export const UploadProgressDialog: React.FC<UploadProgressDialogProps> = ({
  isOpen,
  progress,
  currentWaypoint,
  totalWaypoints,
  uploadSpeed,
  estimatedTimeRemaining,
  status,
  errorMessage,
  onCancel
}) => {
  const dialogRef = React.useRef<HTMLDivElement>(null);

  // Focus management when dialog opens
  useEffect(() => {
    if (isOpen && dialogRef.current) {
      dialogRef.current.focus();
    }
  }, [isOpen]);

  if (!isOpen) return null;

  const formatTime = (seconds: number): string => {
    if (seconds < 60) {
      return `${Math.ceil(seconds)} second${Math.ceil(seconds) !== 1 ? 's' : ''}`;
    }
    const minutes = Math.ceil(seconds / 60);
    return `${minutes} minute${minutes !== 1 ? 's' : ''}`;
  };

  const getStatusText = () => {
    switch (status) {
      case 'completed':
        return '✅ Upload Complete!';
      case 'failed':
        return '❌ Upload Failed';
      default:
        return '📤 Uploading Mission...';
    }
  };

  return (
    <div className="dialog-overlay" role="presentation">
      <div
        className="dialog-container"
        role="dialog"
        aria-labelledby="dialog-title"
        aria-describedby="dialog-description"
        aria-modal="true"
        ref={dialogRef}
        tabIndex={-1}
      >
        <div className="dialog-header">
          <h2 id="dialog-title">{getStatusText()}</h2>
        </div>

        <div className="dialog-content" id="dialog-description">
          {status === 'uploading' && (
            <>
              <ProgressBar
                progress={progress}
                status="loading"
                animated={true}
                showPercentage={true}
                label="Upload Progress"
                size="large"
              />

              <div className="progress-stats">
                <div className="stat-row">
                  <span className="stat-label">Waypoint:</span>
                  <span className="stat-value">
                    {currentWaypoint} of {totalWaypoints}
                  </span>
                </div>

                <div className="stat-row">
                  <span className="stat-label">Speed:</span>
                  <span className="stat-value">
                    {uploadSpeed.toFixed(2)} waypoints/sec
                  </span>
                </div>

                <div className="stat-row">
                  <span className="stat-label">ETA:</span>
                  <span className="stat-value">
                    {formatTime(estimatedTimeRemaining)}
                  </span>
                </div>
              </div>
            </>
          )}

          {status === 'completed' && (
            <div className="success-message">
              <p>✓ Successfully uploaded {totalWaypoints} waypoints to rover</p>
            </div>
          )}

          {status === 'failed' && (
            <div className="error-message">
              <p>⚠ {errorMessage || 'Upload failed. Please try again.'}</p>
            </div>
          )}
        </div>

        <div className="dialog-actions">
          <button
            onClick={onCancel}
            disabled={status === 'uploading'}
            className="btn btn-primary"
            aria-label={status === 'uploading' ? 'Cancel upload' : 'Close dialog'}
          >
            {status === 'uploading' ? 'Cancel Upload' : 'Close'}
          </button>
        </div>
      </div>
    </div>
  );
};

export default UploadProgressDialog;
