import React from 'react';
import { ErrorIcon } from './icons/ErrorIcon';

type ConnectionErrorProps = {
  onRetry: () => void;
  onClose: () => void;
  failedIp: string;
};

const ConnectionError: React.FC<ConnectionErrorProps> = ({ onRetry, onClose, failedIp }) => {
  return (
    <div
      className="fixed inset-0 z-[2000] flex items-center justify-center bg-black bg-opacity-75"
      role="alertdialog"
      aria-modal="true"
    >
      <div className="bg-[#1F2937] text-white rounded-lg shadow-xl w-full max-w-lg p-6 m-4 border border-red-500/50">
        <div className="flex items-center gap-4">
          <ErrorIcon className="w-8 h-8 text-red-500" />
          <div>
            <h2 className="text-xl font-bold text-red-400">Connection Failed</h2>
            <p className="text-gray-300 mt-1">
              Could not establish a connection to the rover backend at <code className="bg-gray-800 px-2 py-1 rounded-md font-mono">{failedIp}</code>.
            </p>
          </div>
        </div>
        
        {/* FIX: Added action buttons */}
        <div className="mt-6 flex justify-end gap-4">
          <button
            onClick={onClose}
            className="bg-gray-600 hover:bg-gray-700 text-white font-bold py-2 px-6 rounded-lg transition-colors"
          >
            Close
          </button>
          <button
            onClick={onRetry}
            className="bg-green-600 hover:bg-green-700 text-white font-bold py-2 px-6 rounded-lg transition-colors"
          >
            Retry
          </button>
        </div>
      </div>
    </div>
  );
};

export default ConnectionError;