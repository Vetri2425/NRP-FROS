
import React from 'react';

type IconProps = {
  className?: string;
};

export const RulerIcon: React.FC<IconProps> = ({ className }) => (
  <svg xmlns="http://www.w3.org/2000/svg" className={className} fill="none" viewBox="0 0 24 24" stroke="currentColor" strokeWidth={2}>
    <path strokeLinecap="round" strokeLinejoin="round" d="M4.33 19.33l1.414-1.414M1 23l2.828-2.828M4.33 4.33l1.414 1.414M1 1l2.828 2.828m18.344 18.344l-1.414-1.414M23 1l-2.828 2.828m-18.344 0l1.414-1.414M23 23l-2.828-2.828M12 4.5v15" />
  </svg>
);