import React from 'react';
import './KeyboardShortcutsHelp.css';

interface ShortcutItem {
  shortcut: string;
  description: string;
}

interface KeyboardShortcutsHelpProps {
  isOpen: boolean;
  onClose: () => void;
  shortcuts: ShortcutItem[];
}

/**
 * Modal showing all available keyboard shortcuts
 */
export const KeyboardShortcutsHelp: React.FC<KeyboardShortcutsHelpProps> = ({
  isOpen,
  onClose,
  shortcuts
}) => {
  if (!isOpen) {
    return null;
  }

  const handleBackdropClick = (e: React.MouseEvent) => {
    if (e.target === e.currentTarget) {
      onClose();
    }
  };

  const handleKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === 'Escape') {
      onClose();
    }
  };

  return (
    <div 
      className="shortcuts-help-overlay"
      onClick={handleBackdropClick}
      onKeyDown={handleKeyDown}
      role="dialog"
      aria-modal="true"
      aria-labelledby="shortcuts-help-title"
    >
      <div className="shortcuts-help-modal">
        <div className="shortcuts-help-header">
          <h2 id="shortcuts-help-title">Keyboard Shortcuts</h2>
          <button 
            className="shortcuts-help-close"
            onClick={onClose}
            aria-label="Close shortcuts help"
          >
            ✕
          </button>
        </div>

        <div className="shortcuts-help-content">
          {shortcuts.length === 0 ? (
            <p className="shortcuts-help-empty">No keyboard shortcuts available</p>
          ) : (
            <table className="shortcuts-help-table">
              <thead>
                <tr>
                  <th>Shortcut</th>
                  <th>Action</th>
                </tr>
              </thead>
              <tbody>
                {shortcuts.map((item, index) => (
                  <tr key={index}>
                    <td>
                      <kbd className="shortcuts-help-kbd">{item.shortcut}</kbd>
                    </td>
                    <td>{item.description}</td>
                  </tr>
                ))}
              </tbody>
            </table>
          )}
        </div>

        <div className="shortcuts-help-footer">
          <p className="shortcuts-help-hint">
            Press <kbd>Esc</kbd> or click outside to close
          </p>
        </div>
      </div>
    </div>
  );
};
