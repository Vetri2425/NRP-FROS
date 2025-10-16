
import React from 'react';

type IconProps = {
  className?: string;
  level?: number;
};

export const BatteryIcon: React.FC<IconProps> = ({ className, level }) => {
  const getColor = () => {
    if (level === undefined || level === null) return 'text-gray-400';
    if (level > 50) return 'text-green-400';
    if (level > 20) return 'text-yellow-400';
    return 'text-red-500';
  }

  return (
    <svg xmlns="http://www.w3.org/2000/svg" className={`${className} ${getColor()}`} fill="none" viewBox="0 0 24 24" stroke="currentColor" strokeWidth={2}>
      <path strokeLinecap="round" strokeLinejoin="round" d="M9 20l-5.447-2.724A1 1 0 013 16.382V7.618a1 1 0 011.447-.894L9 9m0 11V9m0 11a2 2 0 01-2 2H7a2 2 0 01-2-2V7a2 2 0 012-2h1a2 2 0 012 2m0 11v-3.333a2 2 0 012-2h3a2 2 0 012 2V20a2 2 0 01-2 2h-3a2 2 0 01-2-2z" />
    </svg>
  );
};