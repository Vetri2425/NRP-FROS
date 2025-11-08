import React, { useEffect, useRef } from 'react';
import { createPortal } from 'react-dom';

export type DialogType = 'alert' | 'confirm';

interface GenericDialogProps {
  isOpen: boolean;
  type: DialogType;
  title: string;
  message: string;
  onConfirm?: () => void;
  onCancel?: () => void;
  confirmText?: string;
  cancelText?: string;
}

const GenericDialog: React.FC<GenericDialogProps> = ({
  isOpen,
  type,
  title,
  message,
  onConfirm,
  onCancel,
  confirmText = 'OK',
  cancelText = 'Cancel',
}) => {
  const dialogRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    if (isOpen && dialogRef.current) {
      dialogRef.current.focus();
    }
  }, [isOpen]);

  const handleKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === 'Escape' && type === 'confirm' && onCancel) {
      onCancel();
    } else if (e.key === 'Enter' && onConfirm) {
      onConfirm();
    }
  };

  if (!isOpen) return null;

  const dialogContent = (
    <div
      className="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-[1000]"
      onClick={(e) => {
        if (e.target === e.currentTarget && type === 'confirm' && onCancel) {
          onCancel();
        }
      }}
    >
      <div
        ref={dialogRef}
        className="bg-gray-800 rounded-lg p-6 max-w-md w-full mx-4 border border-gray-700 shadow-xl"
        onKeyDown={handleKeyDown}
        tabIndex={-1}
        role="dialog"
        aria-modal="true"
        aria-labelledby="dialog-title"
        aria-describedby="dialog-message"
      >
        <h3 id="dialog-title" className="text-lg font-semibold text-white mb-4">
          {title}
        </h3>

        <div id="dialog-message" className="text-gray-300 mb-6 whitespace-pre-line">
          {message}
        </div>

        <div className="flex justify-end space-x-3">
          {type === 'confirm' && onCancel && (
            <button
              onClick={onCancel}
              className="px-4 py-2 bg-gray-600 hover:bg-gray-500 text-white rounded-md transition-colors focus:outline-none focus:ring-2 focus:ring-gray-400"
            >
              {cancelText}
            </button>
          )}
          <button
            onClick={onConfirm}
            className={`px-4 py-2 text-white rounded-md transition-colors focus:outline-none focus:ring-2 ${
              type === 'alert'
                ? 'bg-blue-600 hover:bg-blue-500 focus:ring-blue-400'
                : 'bg-red-600 hover:bg-red-500 focus:ring-red-400'
            }`}
            autoFocus
          >
            {confirmText}
          </button>
        </div>
      </div>
    </div>
  );

  return createPortal(dialogContent, document.body);
};

export default GenericDialog;