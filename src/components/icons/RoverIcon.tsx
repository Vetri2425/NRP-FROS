
import React from 'react';

type IconProps = {
  className?: string;
  heading?: number;
};

export const RoverIcon: React.FC<IconProps> = ({ className, heading = 0 }) => (
  <div
    style={{
      transform: `rotate(${heading}deg)`,
      transition: 'transform 0.2s linear',
    }}
  >
    <svg xmlns="http://www.w3.org/2000/svg" className={className} viewBox="0 0 24 24" fill="currentColor">
        <path d="M12 2L2.5 6.5v9L12 20l9.5-4.5v-9L12 2zm0 2.47L19.5 9v1.88l-7.5 3.33-7.5-3.33V9L12 4.47zM4 14.55l7 3.11v-4.5L4 10.05v4.5zm16 0v-4.5l-7 3.11v4.5l7-3.11z"/>
    </svg>
  </div>
);