import React from 'react';

interface GeofenceIconProps {
  className?: string;
}

export const GeofenceIcon: React.FC<GeofenceIconProps> = ({ className = "w-6 h-6" }) => {
  return (
    <svg className={className} fill="none" stroke="currentColor" viewBox="0 0 24 24" xmlns="http://www.w3.org/2000/svg">
      <circle cx="12" cy="12" r="8" strokeWidth="2" strokeDasharray="4 2"/>
      <circle cx="12" cy="12" r="4" strokeWidth="2"/>
      <path d="M12 8v8M8 12h8" strokeWidth="1" opacity="0.7"/>
    </svg>
  );
};