
import React from 'react';

type IconProps = {
  className?: string;
};

export const PlanIcon: React.FC<IconProps> = ({ className }) => (
  <svg xmlns="http://www.w3.org/2000/svg" className={className} fill="none" viewBox="0 0 24 24" stroke="currentColor" strokeWidth={2}>
    <path strokeLinecap="round" strokeLinejoin="round" d="M9 20l-5.447-2.724A1 1 0 013 16.382V7.618a1 1 0 011.447-.894L9 9m12 0l-5.447 2.724A1 1 0 0115 12.618V19.382a1 1 0 01-1.447.894L9 18m9-9l-9 4.5" />
    <path strokeLinecap="round" strokeLinejoin="round" d="M9 9v9" />
  </svg>
);