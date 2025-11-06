import React, { useState } from 'react';
import WPMarkDialog from './WPMarkDialog';

interface WPMarkButtonProps {
  disabled?: boolean;
}

const WPMarkButton: React.FC<WPMarkButtonProps> = ({ disabled = false }) => {
  const [isDialogOpen, setIsDialogOpen] = useState(false);

  return (
    <>
      <button
        onClick={() => setIsDialogOpen(true)}
        disabled={disabled}
        className={`
          w-full px-4 py-3 rounded-lg font-semibold
          transition-all duration-200
          ${
            disabled
              ? 'bg-gray-600 text-gray-400 cursor-not-allowed'
              : 'bg-orange-600 hover:bg-orange-700 text-white shadow-lg hover:shadow-xl'
          }
        `}
      >
        🎯 WP_MARK Mission
      </button>

      {isDialogOpen && (
        <WPMarkDialog
          isOpen={isDialogOpen}
          onClose={() => setIsDialogOpen(false)}
        />
      )}
    </>
  );
};

export default WPMarkButton;
