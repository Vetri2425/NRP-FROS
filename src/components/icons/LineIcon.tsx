import React from 'react';

type IconProps = {
  className?: string;
};

export const LineIcon: React.FC<IconProps> = ({ className }) => (
  <svg xmlns="http://www.w3.org/2000/svg" className={className} fill="none" viewBox="0 0 24 24" stroke="currentColor" strokeWidth={2}>
    <path strokeLinecap="round" strokeLinejoin="round" d="M18.364 18.364l-12.728-12.728m12.728 0L5.636 18.364" />
    <circle cx="6" cy="6" r="2" fill="currentColor" stroke="none" />
    <circle cx="18" cy="18" r="2" fill="currentColor" stroke="none" />
  </svg>
);