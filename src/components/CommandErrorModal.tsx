import React from 'react';
import { ErrorIcon } from './icons/ErrorIcon';

type CommandErrorModalProps = {
  isOpen: boolean;
  onClose: () => void;
  errorInfo: {
    title: string;
    message: string;
    causes: string[];
  } | null;
};

const CommandErrorModal: React.FC<CommandErrorModalProps> = ({ isOpen, onClose, errorInfo }) => {
  if (!isOpen || !errorInfo) return null;

  return (
    <div
      className="fixed inset-0 z-[2000] flex items-center justify-center bg-black bg-opacity-50"
      aria-labelledby="error-title"
      role="alertdialog"
      aria-modal="true"
    >
      <div className="bg-[#1F2937] text-white rounded-lg shadow-xl w-full max-w-lg p-6 m-4 border border-red-500/50">
        <div className="flex items-start gap-4">
          <div className="mt-1">
            <ErrorIcon className="w-8 h-8 text-red-500" />
          </div>
          <div className="flex-1">
            <h2 id="error-title" className="text-xl font-bold text-red-400 mb-2">
              {errorInfo.title}
            </h2>
            <p className="text-gray-300 bg-gray-800 p-3 rounded-md font-mono text-sm">
              {errorInfo.message}
            </p>
            <div className="mt-4">
              <h3 className="font-semibold text-gray-200 mb-2">Possible Root Causes:</h3>
              <ul className="list-disc list-inside space-y-1 text-gray-400 text-sm">
                {errorInfo.causes.map((cause, index) => (
                  <li key={index}>{cause}</li>
                ))}
              </ul>
            </div>
          </div>
        </div>
        <div className="mt-6 flex justify-end">
          <button
            onClick={onClose}
            className="bg-red-600 hover:bg-red-700 text-white font-bold py-2 px-6 rounded-lg transition-colors"
          >
            Acknowledge
          </button>
        </div>
      </div>
    </div>
  );
};

export default CommandErrorModal;