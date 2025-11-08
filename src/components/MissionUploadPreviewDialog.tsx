import React, { useState, useEffect } from 'react';
import { Waypoint } from '../types';
import { ValidationError } from '../utils/waypointValidator';
import { calculateMissionStatistics, formatDistance, formatFlightTime } from '../utils/missionCalculator';
import './MissionPreviewDialog.css';

interface MissionUploadPreviewDialogProps {
  isOpen: boolean;
  waypoints: Waypoint[];
  validationErrors: ValidationError[];
  onConfirm: () => void;
  onCancel: () => void;
}

export const MissionUploadPreviewDialog: React.FC<MissionUploadPreviewDialogProps> = ({
  isOpen,
  waypoints,
  validationErrors,
  onConfirm,
  onCancel
}) => {
  const [dontShowAgain, setDontShowAgain] = useState(false);
  const dialogRef = React.useRef<HTMLDivElement>(null);

  useEffect(() => {
    if (isOpen && dialogRef.current) {
      dialogRef.current.focus();
    }
  }, [isOpen]);

  if (!isOpen || waypoints.length === 0) return null;

  const stats = calculateMissionStatistics(waypoints);
  const errorCount = validationErrors.filter(e => e.type === 'error').length;
  const warningCount = validationErrors.filter(e => e.type === 'warning').length;

  return (
    <div className="dialog-overlay" role="presentation">
      <div
        className="dialog-container mission-preview-dialog"
        role="dialog"
        aria-labelledby="upload-preview-title"
        aria-describedby="upload-preview-desc"
        aria-modal="true"
        ref={dialogRef}
        tabIndex={-1}
      >
        <div className="dialog-header">
          <h2 id="upload-preview-title">📋 Review Mission Before Upload</h2>
          <p id="upload-preview-desc" className="dialog-subtitle">
            Please review the mission details before uploading to rover
          </p>
        </div>

        <div className="dialog-content">
          {/* Mission Statistics */}
          <section className="statistics-section">
            <h3>Mission Statistics</h3>
            <div className="stats-grid">
              <div className="stat-item">
                <span className="stat-label">Waypoints:</span>
                <span className="stat-value">{stats.waypointCount}</span>
              </div>
              <div className="stat-item">
                <span className="stat-label">Total Distance:</span>
                <span className="stat-value">{formatDistance(stats.totalDistance)}</span>
              </div>
              <div className="stat-item">
                <span className="stat-label">Estimated Time:</span>
                <span className="stat-value">{formatFlightTime(stats.estimatedTime)}</span>
              </div>
              <div className="stat-item">
                <span className="stat-label">Altitude Range:</span>
                <span className="stat-value">
                  {Math.round(stats.altitudeRange.min)}m - {Math.round(stats.altitudeRange.max)}m
                </span>
              </div>
              <div className="stat-item">
                <span className="stat-label">Mission Bounds:</span>
                <span className="stat-value">
                  {stats.boundingBox.north.toFixed(4)}°N to {stats.boundingBox.south.toFixed(4)}°S
                </span>
              </div>
              <div className="stat-item">
                <span className="stat-label">East/West:</span>
                <span className="stat-value">
                  {stats.boundingBox.east.toFixed(4)}°E to {stats.boundingBox.west.toFixed(4)}°W
                </span>
              </div>
            </div>
          </section>

          {/* Validation Status */}
          {(errorCount > 0 || warningCount > 0) && (
            <section className="validation-section">
              <h3>Validation Status</h3>
              {errorCount > 0 && (
                <div className="validation-error">
                  ⚠ {errorCount} error{errorCount !== 1 ? 's' : ''} found
                </div>
              )}
              {warningCount > 0 && (
                <div className="validation-warning">
                  ℹ {warningCount} warning{warningCount !== 1 ? 's' : ''} found
                </div>
              )}
            </section>
          )}

          {/* Waypoints Table */}
          <section className="waypoints-section">
            <h3>Waypoints Preview</h3>
            <div className="waypoints-table-wrapper">
              <table className="waypoints-table" role="table">
                <thead>
                  <tr>
                    <th scope="col">#</th>
                    <th scope="col">Latitude</th>
                    <th scope="col">Longitude</th>
                    <th scope="col">Altitude (m)</th>
                    <th scope="col">Command</th>
                  </tr>
                </thead>
                <tbody>
                  {waypoints.map((wp, idx) => (
                    <tr key={idx}>
                      <td>{idx}</td>
                      <td>{wp.lat.toFixed(6)}</td>
                      <td>{wp.lng.toFixed(6)}</td>
                      <td>{wp.alt}</td>
                      <td>{wp.command || 'WAYPOINT'}</td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
          </section>

          {/* Don't Show Again */}
          <div className="dialog-checkbox">
            <input
              type="checkbox"
              id="dont-show-again"
              checked={dontShowAgain}
              onChange={(e) => setDontShowAgain(e.target.checked)}
            />
            <label htmlFor="dont-show-again">Don't show this preview again</label>
          </div>
        </div>

        <div className="dialog-actions">
          <button
            onClick={onCancel}
            className="btn btn-secondary"
            aria-label="Cancel upload"
          >
            Cancel
          </button>
          <button
            onClick={onConfirm}
            className="btn btn-primary"
            disabled={errorCount > 0}
            aria-label={errorCount > 0 ? 'Cannot upload with validation errors' : 'Confirm and upload mission'}
          >
            Upload ✓
          </button>
        </div>
      </div>
    </div>
  );
};

export default MissionUploadPreviewDialog;
