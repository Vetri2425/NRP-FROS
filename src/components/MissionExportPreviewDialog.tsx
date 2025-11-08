import React, { useState, useEffect } from 'react';
import { Waypoint } from '../types';
import './MissionPreviewDialog.css';

type ExportFormat = 'qgc' | 'json' | 'kml' | 'csv';

interface MissionExportPreviewDialogProps {
  isOpen: boolean;
  waypoints: Waypoint[];
  currentFormat: ExportFormat;
  onConfirm: (format: ExportFormat) => void;
  onCancel: () => void;
}

export const MissionExportPreviewDialog: React.FC<MissionExportPreviewDialogProps> = ({
  isOpen,
  waypoints,
  currentFormat,
  onConfirm,
  onCancel
}) => {
  const [selectedFormat, setSelectedFormat] = useState<ExportFormat>(currentFormat);
  const [includeMetadata, setIncludeMetadata] = useState(true);
  const [dontShowAgain, setDontShowAgain] = useState(false);
  const dialogRef = React.useRef<HTMLDivElement>(null);

  useEffect(() => {
    if (isOpen && dialogRef.current) {
      dialogRef.current.focus();
    }
  }, [isOpen]);

  useEffect(() => {
    setSelectedFormat(currentFormat);
  }, [currentFormat]);

  if (!isOpen) return null;

  const formatDescriptions: Record<ExportFormat, { name: string; description: string; extension: string }> = {
    qgc: {
      name: 'QGC WPL 110',
      description: 'Standard format compatible with QGroundControl and ArduPilot',
      extension: '.waypoints'
    },
    json: {
      name: 'JSON',
      description: 'Web-friendly format for programmatic use',
      extension: '.json'
    },
    kml: {
      name: 'KML (Google Earth)',
      description: 'GIS-compatible format for mapping applications',
      extension: '.kml'
    },
    csv: {
      name: 'CSV (Spreadsheet)',
      description: 'Import into Excel or Google Sheets',
      extension: '.csv'
    }
  };

  const formatInfo = formatDescriptions[selectedFormat];
  const estimatedFileSize = Math.ceil(waypoints.length * 50 / 1024); // rough estimate

  return (
    <div className="dialog-overlay" role="presentation">
      <div
        className="dialog-container mission-preview-dialog"
        role="dialog"
        aria-labelledby="export-preview-title"
        aria-modal="true"
        ref={dialogRef}
        tabIndex={-1}
      >
        <div className="dialog-header">
          <h2 id="export-preview-title">📤 Export Mission</h2>
        </div>

        <div className="dialog-content">
          {/* Current Format Info */}
          <section className="format-info-section">
            <h3>Export Format</h3>
            <div className="format-info">
              <div className="info-item">
                <span className="info-label">Format:</span>
                <span className="info-value">{formatInfo.name}</span>
              </div>
              <div className="info-item">
                <span className="info-label">Extension:</span>
                <span className="info-value">{formatInfo.extension}</span>
              </div>
              <div className="info-item">
                <span className="info-label">Waypoints:</span>
                <span className="info-value">{waypoints.length}</span>
              </div>
              <div className="info-item">
                <span className="info-label">File Size:</span>
                <span className="info-value">~{estimatedFileSize} KB</span>
              </div>
            </div>
          </section>

          {/* Format Selection */}
          <section className="format-selection-section">
            <h3>Available Formats</h3>
            <div className="format-options">
              {(Object.keys(formatDescriptions) as ExportFormat[]).map(format => (
                <label key={format} className="format-option">
                  <input
                    type="radio"
                    name="export-format"
                    value={format}
                    checked={selectedFormat === format}
                    onChange={(e) => setSelectedFormat(e.target.value as ExportFormat)}
                  />
                  <div className="format-option-content">
                    <span className="format-name">{formatDescriptions[format].name}</span>
                    <span className="format-description">
                      {formatDescriptions[format].description}
                    </span>
                  </div>
                </label>
              ))}
            </div>
          </section>

          {/* Export Options */}
          <section className="export-options-section">
            <h3>Export Options</h3>
            <label className="option-checkbox">
              <input
                type="checkbox"
                checked={includeMetadata}
                onChange={(e) => setIncludeMetadata(e.target.checked)}
              />
              <span>Include mission metadata</span>
            </label>
          </section>

          {/* Don't Show Again */}
          <div className="dialog-checkbox">
            <input
              type="checkbox"
              id="export-dont-show-again"
              checked={dontShowAgain}
              onChange={(e) => setDontShowAgain(e.target.checked)}
            />
            <label htmlFor="export-dont-show-again">Don't show this dialog again</label>
          </div>
        </div>

        <div className="dialog-actions">
          <button
            onClick={onCancel}
            className="btn btn-secondary"
            aria-label="Cancel export"
          >
            Cancel
          </button>
          <button
            onClick={() => onConfirm(selectedFormat)}
            className="btn btn-primary"
            aria-label={`Export mission as ${formatInfo.name}`}
          >
            Export ✓
          </button>
        </div>
      </div>
    </div>
  );
};

export default MissionExportPreviewDialog;
