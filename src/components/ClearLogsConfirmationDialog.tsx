import React from 'react';
import { createPortal } from 'react-dom';

type ClearLogsConfirmationDialogProps = {
  isOpen: boolean;
  onConfirm: () => void;
  onCancel: () => void;
  isEmpty?: boolean;
};

const ClearLogsConfirmationDialog: React.FC<ClearLogsConfirmationDialogProps> = ({
  isOpen,
  onConfirm,
  onCancel,
  isEmpty = false,
}) => {
  if (!isOpen) return null;

  const dialogContent = (
    <div className="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-[1000]">
      <div className="bg-gray-800 rounded-lg p-6 max-w-md w-full mx-4 border border-gray-700">
        <h3 className="text-lg font-semibold text-white mb-4">
          {isEmpty ? 'Live Report Already Empty' : 'Clear Mission Logs'}
        </h3>

        {isEmpty ? (
          <div className="text-gray-300 mb-6">
            <p className="mb-2">The live report is already empty. There are no mission logs, waypoints, or trails to clear.</p>
            <p className="text-sm text-gray-400">The rover position and current telemetry will remain visible.</p>
          </div>
        ) : (
          <div className="text-gray-300 mb-6">
            <p className="mb-3">This will clear the following mission data:</p>
            <ul className="list-disc list-inside space-y-1 text-sm">
              <li>Mission logs and history</li>
              <li>All waypoints from the map</li>
              <li>Rover trail/path on the map</li>
              <li>Mission progress indicators</li>
              <li>Waypoint completion status</li>
              <li>WP_MARK operations (if running)</li>
            </ul>
            <p className="mt-3 text-sm text-yellow-400">
              <strong>Note:</strong> Rover position, telemetry, and connection status will remain visible.
            </p>
          </div>
        )}

        <div className="flex justify-end space-x-3">
          <button
            onClick={onCancel}
            className="px-4 py-2 bg-gray-600 hover:bg-gray-700 text-white rounded transition-colors"
          >
            {isEmpty ? 'Close' : 'Cancel'}
          </button>
          {!isEmpty && (
            <button
              onClick={onConfirm}
              className="px-4 py-2 bg-red-600 hover:bg-red-700 text-white rounded transition-colors"
            >
              Clear All
            </button>
          )}
        </div>
      </div>
    </div>
  );

  return createPortal(dialogContent, document.body);
};

export default ClearLogsConfirmationDialog;