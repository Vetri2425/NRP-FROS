
import React from 'react';

type IconProps = {
  className?: string;
};

export const SignalIcon: React.FC<IconProps> = ({ className }) => (
  <svg xmlns="http://www.w3.org/2000/svg" className={className} fill="none" viewBox="0 0 24 24" stroke="currentColor" strokeWidth={2}>
    <path strokeLinecap="round" strokeLinejoin="round" d="M20.25 8.511c.884.284 1.5 1.128 1.5 2.097v4.286c0 1.136-.847 2.1-1.98 2.193l-3.72-8.284z" />
    <path strokeLinecap="round" strokeLinejoin="round" d="M18 19.5a1.5 1.5 0 11-3 0 1.5 1.5 0 013 0z" />
    <path strokeLinecap="round" strokeLinejoin="round" d="M12 12.75h.008v.008H12v-.008z" />
    <path strokeLinecap="round" strokeLinejoin="round" d="M3.75 6.108A2.25 2.25 0 016 3.858h9.75A2.25 2.25 0 0118 6.108V11.25a2.25 2.25 0 01-2.25 2.25H6a2.25 2.25 0 01-2.25-2.25V6.108z" />
  </svg>
);