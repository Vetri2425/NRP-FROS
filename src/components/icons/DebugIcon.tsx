import React from 'react';

interface DebugIconProps {
  className?: string;
}

export const DebugIcon: React.FC<DebugIconProps> = ({ className = "w-6 h-6" }) => {
  return (
    <svg className={className} fill="none" stroke="currentColor" viewBox="0 0 24 24" xmlns="http://www.w3.org/2000/svg">
      <path strokeLinecap="round" strokeLinejoin="round" strokeWidth="2" d="M19 13.586V10a7 7 0 10-14 0v3.586l-.707.707A1 1 0 004 15v4a1 1 0 001 1h14a1 1 0 001-1v-4a1 1 0 00-.293-.707L19 13.586z"/>
      <path strokeLinecap="round" strokeLinejoin="round" strokeWidth="2" d="M15 8a3 3 0 11-6 0 3 3 0 016 0z"/>
      <circle cx="12" cy="18" r="2" fill="currentColor"/>
    </svg>
  );
};