
import React from 'react';

type FullScreenToggleIconProps = {
  isFullScreen: boolean;
  className?: string;
};

export const FullScreenToggleIcon: React.FC<FullScreenToggleIconProps> = ({ isFullScreen, className }) => {
  if (isFullScreen) {
    // Exit fullscreen (compress) icon
    return (
      <svg xmlns="http://www.w3.org/2000/svg" className={className} fill="none" viewBox="0 0 24 24" stroke="currentColor" strokeWidth={2}>
        <path strokeLinecap="round" strokeLinejoin="round" d="M15 3h6m0 0v6m0-6l-7 7M9 21H3m0 0v-6m0 6l7-7" />
      </svg>
    );
  }

  // Enter fullscreen (expand) icon
  return (
    <svg xmlns="http://www.w3.org/2000/svg" className={className} fill="none" viewBox="0 0 24 24" stroke="currentColor" strokeWidth={2}>
      <path strokeLinecap="round" strokeLinejoin="round" d="M3 8V4m0 0h4M3 4l4 4m8 0V4m0 0h-4m4 0l-4 4m-8 8v4m0 0h4m-4 0l4-4m8 4v-4m0 0h-4m4 0l-4-4" />
    </svg>
  );
};